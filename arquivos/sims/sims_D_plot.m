%% sims_D_plot — 3 curvas por cenario (PID como esta / PID equiparado / LQRy)
%  + metricas medidas contra a MESMA referencia (degrau cru) nos tres.

raiz  = 'C:\Users\kaue\Documents\PID_DH\HIL_PID\Matlab';
pasta = fullfile(raiz, 'Dados_mat_comparacao');
figdir = 'C:\Users\kaue\Documents\PID_DH\Artigo\figs';
artigo = 'C:\Users\kaue\Documents\PID_DH\Artigo';
nomes = {'vel','latero','long','comb'};
titulos = struct('vel','Scenario 1 - airspeed step (12.0 to 15.2 m/s)', ...
                 'latero','Scenario 2 - lateral-directional (\phi doublet, 5 deg)', ...
                 'long','Scenario 3 - longitudinal (\theta doublet, 5 deg)', ...
                 'comb','Scenario 4 - combined maneuver');

cP=[0 0.447 0.741]; cE=[0.10 0.55 0.30]; cL=[0.85 0.325 0.098]; cR=[0.45 0.45 0.45];
paineis = {'phi_NL',2,'phi','\phi','deg'; 'theta_NL',2,'theta','\theta','deg';
           'VT_NL',2,'VT','Airspeed V_T','m/s'; 'H_NL',2,'','Altitude','m';
           'Throttle_NL',1,'','Throttle','-'; 'elev_NL',1,'','Elevator','deg'};

fid = fopen(fullfile(artigo,'dados_sims_D.txt'),'w');
fprintf(fid,'===== 3 curvas: PID como esta / PID equiparado / LQRy =====\n');
fprintf(fid,'Metricas medidas contra a MESMA referencia (degrau cru).\n\n');

for c = 1:numel(nomes)
    cn = nomes{c};
    A = load(fullfile(pasta, sprintf('pid_%s_comoesta.mat', cn)));   oA = A.S;
    B = load(fullfile(pasta, sprintf('pid_%s_equiparado.mat', cn))); oB = B.S;
    C = load(fullfile(pasta, sprintf('lqry_%s.mat', cn)));           oC = C.S;

    fig = figure('Color','w'); try, set(fig,'Theme','light'); catch, end
    set(fig,'Units','normalized','Position',[0.05 0.05 0.9 0.85]);
    tl = tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
    title(tl, [titulos.(cn) ' - PID (as is) vs PID (equalized) vs LQRy'], ...
        'FontWeight','bold','Interpreter','tex');

    for k = 1:6
        vA = oA.(paineis{k,1}); vB = oB.(paineis{k,1}); vC = oC.(paineis{k,1});
        ax = nexttile(tl); hold(ax,'on'); grid(ax,'on'); leg = {};
        if ~isempty(paineis{k,3})
            tr = vA.time; rr = ref_crua(tr, paineis{k,3}, cn);
            if strcmp(paineis{k,3},'theta'), rr = rr + 14.44; end
            plot(ax, tr, rr, ':', 'Color', cR, 'LineWidth', 1.1); leg{end+1}='Reference';
        end
        plot(ax, vA.time, vA.signals.values(:,paineis{k,2}), '-',  'Color',cP,'LineWidth',1.3);
        plot(ax, vB.time, vB.signals.values(:,paineis{k,2}), '-',  'Color',cE,'LineWidth',1.3);
        plot(ax, vC.time, vC.signals.values(:,paineis{k,2}), '--', 'Color',cL,'LineWidth',1.3);
        leg = [leg {'PID (as is)','PID (equalized)','LQRy'}];
        title(ax, paineis{k,4}); ylabel(ax, paineis{k,5}); xlim(ax,[0 300]);
        if k>=5, xlabel(ax,'Time [s]'); end
        if k==1, legend(ax, leg, 'Location','best'); end
    end
    exportgraphics(fig, fullfile(figdir, sprintf('fig_eq_%s.png', cn)), ...
        'Resolution',300,'BackgroundColor','white');
    close(fig);

    % ---- metricas contra a referencia crua ----
    janelas = {'VT','VT_NL',5,60; 'phi','phi_NL',50,100; 'theta','theta_NL',100,150};
    for j = 1:3
        tipo = janelas{j,1};
        if strcmp(tipo,'VT')   && ~strcmp(cn,'vel'), continue; end
        if strcmp(tipo,'phi')  && ~any(strcmp(cn,{'latero','comb'})), continue; end
        if strcmp(tipo,'theta') && ~any(strcmp(cn,{'long','comb'})), continue; end
        res = cell(1,3); dados = {oA, oB, oC};
        for r = 1:3
            v = dados{r}.(janelas{j,2});
            t = v.time; y = v.signals.values(:,2);
            rr = ref_crua(t, tipo, cn);
            if strcmp(tipo,'theta'), rr = rr + 14.44; end
            res{r} = metricas(t, y, rr, janelas{j,3}, janelas{j,4});
        end
        fprintf(fid,'%-7s %-6s | PID: OS=%5.1f%% tr=%5.2f ts=%6.2f ess=%+7.3f | EQ: OS=%5.1f%% tr=%5.2f ts=%6.2f ess=%+7.3f | LQRy: OS=%5.1f%% tr=%5.2f ts=%6.2f ess=%+7.3f\n', ...
            cn, tipo, res{1}.os,res{1}.tr,res{1}.ts,res{1}.ess, ...
            res{2}.os,res{2}.tr,res{2}.ts,res{2}.ess, res{3}.os,res{3}.tr,res{3}.ts,res{3}.ess);
    end
    fprintf('cenario %s ok\n', cn);
end
fclose(fid);
type(fullfile(artigo,'dados_sims_D.txt'));
close all

% referencia CRUA (degrau/doublet comandado), a mesma p/ os tres
function r = ref_crua(t, tipo, cenario)
    r = zeros(size(t));
    switch tipo
        case 'phi'
            if any(strcmp(cenario,{'latero','comb'}))
                r = 5*((t>=50 & t<100) - (t>=100 & t<150));
            end
        case 'theta'
            if any(strcmp(cenario,{'long','comb'}))
                r = 5*((t>=100 & t<150) - (t>=150 & t<200));
            end
        case 'VT'
            r = 12.0*ones(size(t));
            if strcmp(cenario,'vel'), r(t>=5) = 15.2; end
    end
end

function m = metricas(t, y, ref, t0, t1)
    seg = t>=t0 & t<=t1; ts_=t(seg); ys=y(seg); rs=ref(seg);
    y0 = ys(1); yf = mean(rs(ts_ > t0+ (t1-t0)*0.5));
    dy = yf - y0;
    if abs(dy) < 1e-6, m = struct('os',0,'tr',0,'ts',0,'ess',0); return; end
    yn = (ys-y0)/dy;
    i10 = find(yn>=0.1,1); i90 = find(yn>=0.9,1);
    if isempty(i10)||isempty(i90), m.tr = NaN; else, m.tr = ts_(i90)-ts_(i10); end
    m.os = max(0,(max(yn)-1)*100);
    fora = find(abs(yn-1)>0.02,1,'last');
    if isempty(fora), m.ts = 0; else, m.ts = ts_(min(fora+1,numel(ts_)))-t0; end
    m.ess = mean(ys(ts_ > t1-3)) - yf;
end
