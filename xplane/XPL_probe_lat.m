function out = XPL_probe_lat(cfg)
%XPL_PROBE_LAT Sonda de identificacao LATERAL do DH no X-Plane.
%
% Fluxo: reload do .acf (motor fresco) -> escreve raios de giracao via
% dref (Jyy=GUINADA, Jzz=ROLAGEM, unitmass = R^2 em m^2; reload reseta,
% por isso escreve DEPOIS do reload) -> teleporta a MSL 600/V0 ->
% autotrim (manete<-V, theta_ref<-hdot=0) -> DOUBLET no eixo pedido ->
% grava 8 s a ~20 Hz -> metricas.
%
% cfg (struct, todos opcionais):
%   .eixo    'ail' | 'rudd'        (default 'rudd')
%   .Jyy     R^2 guinada [m^2]     (NaN = nao mexe; default NaN)
%   .Jzz     R^2 rolagem [m^2]     (NaN = nao mexe)
%   .V0      m/s                   (default 17)
%   .amp     amplitude do doublet  (default 0.5, normalizado)
%   .tp      duracao de cada lobo  (default 0.4 s)
%
% out: .t .p .q .r .phi .V (arrays), .J (lidos pos-escrita),
%      .pico  (max |taxa primaria| deg/s)
%      .fDR   (Hz, media dos periodos por cruzamentos de zero de r pos-doublet)
%      .zeta  (razao de decaimento dos picos de |r| -> amortecimento)
%      .trim_ok, .de0, .thr0

    if nargin < 1, cfg = struct(); end
    eixo = getdef(cfg,'eixo','rudd'); Jyy = getdef(cfg,'Jyy',NaN); Jzz = getdef(cfg,'Jzz',NaN);
    V0 = getdef(cfg,'V0',17); amp = getdef(cfg,'amp',0.5); tp = getdef(cfg,'tp',0.4);

    import XPlaneConnect.*
    global GlobalSocket

    %% reload (motor fresco) + escrita das inercias
    xp_reload_acf;
    try, closeUDP(GlobalSocket); catch, end
    GlobalSocket = openUDP('127.0.0.1', 49009, 0, 500);
    if ~isnan(Jyy), sendDREF('sim/aircraft/weight/acf_Jyy_unitmass', Jyy, GlobalSocket); end
    if ~isnan(Jzz), sendDREF('sim/aircraft/weight/acf_Jzz_unitmass', Jzz, GlobalSocket); end
    pause(0.2);
    out.J = double(getDREFs({'sim/aircraft/weight/acf_Jxx_unitmass', ...
        'sim/aircraft/weight/acf_Jyy_unitmass','sim/aircraft/weight/acf_Jzz_unitmass'}, GlobalSocket));

    %% teleporte + autotrim (mesma lei do VR_replay_xp, hdot0 = 0)
    r0 = double(getDREFs({'sim/flightmodel/position/latitude', ...
        'sim/flightmodel/position/longitude','sim/flightmodel/position/psi'}, GlobalSocket));
    lat = r0(1); lon = r0(2); psi0 = r0(3); hdg = deg2rad(psi0);
    for rep = 1:2
        sendPOSI([lat, lon, 600, 3, 0, psi0, -998], 0, GlobalSocket); pause(0.05);
        sendDREF('sim/flightmodel/position/local_vx',  V0*sin(hdg), GlobalSocket);
        sendDREF('sim/flightmodel/position/local_vy',  0,           GlobalSocket);
        sendDREF('sim/flightmodel/position/local_vz', -V0*cos(hdg), GlobalSocket);
        sendDREF('sim/flightmodel/position/Prad',0,GlobalSocket);
        sendDREF('sim/flightmodel/position/Qrad',0,GlobalSocket);
        sendDREF('sim/flightmodel/position/Rrad',0,GlobalSocket);
        pause(0.05);
    end
    drefs = {'sim/flightmodel/position/true_airspeed','sim/flightmodel/position/Prad', ...
        'sim/flightmodel/position/Qrad','sim/flightmodel/position/Rrad', ...
        'sim/flightmodel/position/phi','sim/flightmodel/position/theta', ...
        'sim/flightmodel/position/elevation'};
    DT = 0.05; Kq=0.5; Kth=0.8; Kith=0.25; Kp=0.4; Kphi=0.6; Kv=0.08; Khd=0.010;
    de=0; ith=0; thr=0.45; th_ref=deg2rad(3); hf=0; h_ant=NaN; tp_=0;
    t0=tic; nconv=0; conv=false; TR=[];
    while toc(t0) < 40
        raw = ler_(drefs);
        if isempty(raw), pause(DT); continue; end
        t=toc(t0); dt=max(t-tp_,1e-3); tp_=t;
        if ~isnan(h_ant), hf = 0.9*hf + 0.1*(raw(7)-h_ant)/dt; end
        h_ant = raw(7);
        th_ref = max(deg2rad(-15), min(deg2rad(15), th_ref + Khd*(0 - hf)*dt));
        eth = deg2rad(raw(6)) - th_ref;
        ith = max(-0.5, min(0.5, ith + Kith*(-eth)*dt));
        de  = max(-1, min(1, ith - Kq*raw(3) - Kth*eth));
        da  = max(-1, min(1, -Kp*raw(2) - Kphi*deg2rad(raw(5))));
        thr = max(0, min(1, thr + Kv*(V0 - raw(1))*dt));
        sendCTRL([de, da, 0, thr, -998, -998], 0, GlobalSocket);
        TR(end+1,:) = [t de thr raw(1)]; %#ok<AGROW>
        if t > 8 && abs(raw(1)-V0) < 1.5 && abs(hf) < 1 && abs(raw(3)) < deg2rad(3)
            nconv = nconv + 1; if nconv >= round(2/DT), conv = true; break; end
        else
            nconv = 0;
        end
        resto = size(TR,1)*DT - toc(t0); if resto > 0, pause(resto); end
    end
    ult = TR(TR(:,1) > toc(t0)-2, :);
    out.de0 = mean(ult(:,2)); out.thr0 = mean(ult(:,3)); out.trim_ok = conv;

    %% doublet + gravacao (8 s)
    D = []; t0 = tic;
    while toc(t0) < 8
        t = toc(t0);
        u = 0;
        if t < tp, u = amp; elseif t < 2*tp, u = -amp; end
        da_c = 0; dr_c = 0;
        if strcmp(eixo,'ail'), da_c = u; else, dr_c = u; end
        sendCTRL([out.de0, da_c, dr_c, out.thr0, -998, -998], 0, GlobalSocket);
        raw = ler_(drefs);
        if ~isempty(raw), D(end+1,:) = [t raw(:)']; end %#ok<AGROW>
        resto = (size(D,1))*DT - toc(t0); if resto > 0, pause(resto); end
    end
    out.t = D(:,1); out.V = D(:,2); out.p = D(:,3); out.q = D(:,4); out.r = D(:,5);
    out.phi = D(:,6);

    %% metricas
    if strcmp(eixo,'ail'), prim = out.p; else, prim = out.r; end
    out.pico = max(abs(rad2deg(prim)));
    out.fDR = NaN; out.zeta = NaN;
    if strcmp(eixo,'rudd')
        m = out.t > 2*tp + 0.2;                    % pos-doublet
        tt = out.t(m); rr = detrend(rad2deg(out.r(m)));
        zc = tt([false; diff(sign(rr)) ~= 0]);     % cruzamentos de zero
        if numel(zc) >= 4
            T = 2*mean(diff(zc(1:min(6,numel(zc)))));   % periodo medio
            out.fDR = 1/T;
        end
        % picos sucessivos de |r| -> decaimento
        [pk, ~] = findpeaks_(abs(rr));
        pk = pk(pk > 0.5);                          % ignora ruido
        if numel(pk) >= 2
            dr = mean(pk(2:min(4,numel(pk))) ./ pk(1:min(4,numel(pk))-1));
            ld = -log(max(dr, 1e-3)) * 2;           % decremento por ciclo (~2 meio-ciclos)
            out.zeta = ld / sqrt(4*pi^2 + ld^2);
        end
    end
end

function v = getdef(s, f, dv)
    if isfield(s, f) && ~isempty(s.(f)), v = s.(f); else, v = dv; end
end

function raw = ler_(drefs)
    global GlobalSocket
    import XPlaneConnect.*
    raw = [];
    try
        raw = double(getDREFs(drefs, GlobalSocket));
        if numel(raw) < 7 || raw(1) < -1 || raw(1) > 200, raw = []; end
    catch
        try, closeUDP(GlobalSocket); catch, end
        try, GlobalSocket = openUDP('127.0.0.1', 49009, 0, 500); catch, end
    end
end

function [pk, loc] = findpeaks_(y)
    % findpeaks minimo (sem toolbox): maximos locais
    pk = []; loc = [];
    for k = 2:numel(y)-1
        if y(k) > y(k-1) && y(k) >= y(k+1)
            pk(end+1) = y(k); loc(end+1) = k; %#ok<AGROW>
        end
    end
end
