function XP_gui_waypoints()
%XP_GUI_WAYPOINTS  GUI de missao por waypoints do DH no X-Plane (PID).
%
% Adaptacao do gui_waypoints.m do PIPER-1-6 (Julio) para o DH: mesmo
% front-end (mapa clicavel + tabela + raio de aceitacao), mas o botao
% VOAR arma as variaveis do XP_missao.m e voa DE VERDADE no X-Plane,
% com os ganhos 100% da dissertacao (DH_inicializacao).
%
% Uso:
%   >> XP_gui_waypoints
%   1) Clique no mapa para adicionar WPs (alt/vel dos campos ao lado)
%   2) RECARREGUE o aviao no X-Plane (File > Open Aircraft) — energia
%      do motor eletrico do XP9 dura ~90-150 s por reload
%   3) VOAR NO X-PLANE
%
% Referencial: PROA DE ENGATE, fixo ("para cima" no mapa = frente do
% nariz no engate; a missao gira com a proa). O modo NE absoluto foi
% removido da GUI em 2026-08-31 (decisao do Kaue: nao e' necessario);
% o XP_missao continua aceitando XP_WPs_NE por script, se preciso.
%
% Pos-voo: .mat + PNGs em xplane/voos (via XP_missao/plot_XP_missao) e
% trajetoria voada sobreposta no mapa da GUI (tracejado azul).
%
% O engate e' a ORIGEM do mapa (marcador verde): a guiagem parte mirando
% o WP1 da tabela — nao inclua a origem como waypoint.

    xpDir = fileparts(mfilename('fullpath'));

    %% ========== Estado ==========
    wp_data  = zeros(0,4);      % [frente  direita  altMSL  vel] (proa de engate)
    traj     = [];              % trajetoria voada (mapa) p/ overlay
    vista    = [];              % [] = enquadramento automatico; [x1 x2 y1 y2] = pan/zoom manual
    press    = struct('on',false,'pt',[0 0],'p0',[0 0],'px',[0 0],'moved',false);
    liveTrail = zeros(0,2);     % rastreio AO VIVO durante o voo (via global XP_LIVE
    hTrailLive = []; hAviao = []; tLive = [];   %  publicada pelo xp_read_dh a 20 Hz)

    %% ========== Figura ==========
    % sem cores fixas: a GUI segue o tema do MATLAB (claro OU escuro)
    fig = uifigure('Name', 'Missao DH no X-Plane — waypoints (PID dissertacao)', ...
        'Position', [80 80 1000 650]);

    ax = uiaxes(fig, 'Position', [20 20 580 600]);
    ax.XGrid = 'on'; ax.YGrid = 'on'; ax.Box = 'on'; ax.FontSize = 11;
    hold(ax, 'on');
    disableDefaultInteractivity(ax);   % pan/zoom nativos brigariam c/ os nossos
    ax.ButtonDownFcn = @mapClick;

    % pan/zoom manuais: arrastar move o mapa, roda do mouse da zoom no
    % cursor, clique PARADO (<5 px de arrasto) marca waypoint
    fig.WindowButtonMotionFcn = @mapMotion;
    fig.WindowButtonUpFcn     = @mapUp;
    fig.WindowScrollWheelFcn  = @mapWheel;
    uibutton(fig, 'Position', [30 26 110 24], 'Text', 'Ajustar vista', ...
        'Tooltip', 'Volta ao enquadramento automatico dos waypoints', ...
        'ButtonPushedFcn', @(~,~) resetVista());

    %% ========== Painel lateral ==========
    panelX = 620; panelW = 360;

    uilabel(fig, 'Position', [panelX 605 panelW 30], ...
        'Text', 'Waypoints — DH (PID)', 'FontSize', 16, ...
        'FontWeight', 'bold', 'HorizontalAlignment', 'center');

    tbl = uitable(fig, 'Position', [panelX 360 panelW 235], ...
        'ColumnWidth', {26, 84, 84, 62, 50}, ...
        'ColumnEditable', [false true true true true], ...
        'CellEditCallback', @tableEdited);

    uibutton(fig, 'Position', [panelX 320 170 30], 'Text', 'Remover ultimo', ...
        'ButtonPushedFcn', @removeLastWP);
    uibutton(fig, 'Position', [panelX+180 320 170 30], 'Text', 'Limpar tudo', ...
        'ButtonPushedFcn', @clearWPs);
    uibutton(fig, 'Position', [panelX 285 panelW 26], 'Text', 'Carregar circuito OVAL (6 WPs)', ...
        'ButtonPushedFcn', @loadG2);

    yPos = 245;
    uilabel(fig, 'Position', [panelX yPos 165 22], 'Text', 'Altitude do proximo WP:', 'FontSize', 11);
    fldAlt = uieditfield(fig, 'numeric', 'Position', [panelX+170 yPos 80 22], ...
        'Value', 600, 'Limits', [400 1000]);
    uilabel(fig, 'Position', [panelX+255 yPos 60 22], 'Text', 'm MSL');

    yPos = yPos - 30;
    uilabel(fig, 'Position', [panelX yPos 165 22], 'Text', 'Velocidade do proximo WP:', 'FontSize', 11);
    fldVel = uieditfield(fig, 'numeric', 'Position', [panelX+170 yPos 80 22], ...
        'Value', 12, 'Limits', [8 20]);
    uilabel(fig, 'Position', [panelX+255 yPos 40 22], 'Text', 'm/s');

    yPos = yPos - 30;
    uilabel(fig, 'Position', [panelX yPos 165 22], 'Text', 'Raio de aceitacao:', 'FontSize', 11);
    fldRaccept = uieditfield(fig, 'numeric', 'Position', [panelX+170 yPos 80 22], ...
        'Value', 80, 'Limits', [10 500]);
    uilabel(fig, 'Position', [panelX+255 yPos 30 22], 'Text', 'm');

    yPos = yPos - 30;
    uilabel(fig, 'Position', [panelX yPos 165 22], 'Text', 'Duracao (0 = automatica):', 'FontSize', 11);
    fldTime = uieditfield(fig, 'numeric', 'Position', [panelX+170 yPos 80 22], ...
        'Value', 0, 'Limits', [0 1200], 'ValueChangedFcn', @(~,~) updateDuracao());
    uilabel(fig, 'Position', [panelX+255 yPos 30 22], 'Text', 's');

    btnVoar = uibutton(fig, 'Position', [panelX 90 panelW 46], ...
        'Text', 'VOAR NO X-PLANE', 'FontSize', 16, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.2 0.6 0.2], 'FontColor', 'white', ...
        'ButtonPushedFcn', @runMission);

    lblStatus = uilabel(fig, 'Position', [panelX 52 panelW 34], ...
        'Text', ['Pronto. Clique marca waypoint; ARRASTE para mover o mapa; ' ...
                 'roda do mouse da zoom.'], ...
        'FontSize', 10, 'WordWrap', 'on');
    corNeutra = lblStatus.FontColor;           % cor default do tema atual
    corInfo = [0.40 0.60 1.00]; corOk = [0.25 0.75 0.35]; corErro = [0.90 0.30 0.30];
    corAviso = [0.95 0.55 0.15];

    % indicador de duracao vs energia do motor (~130 s de voo motorizado
    % por reload — PENDENCIA_MOTOR.md; alem disso a missao vira planeio)
    lblDur = uilabel(fig, 'Position', [panelX 138 panelW 16], ...
        'Text', '', 'FontSize', 10, 'WordWrap', 'off');

    uilabel(fig, 'Position', [panelX 6 panelW 44], ...
        'Text', ['Engate na ORIGEM (marcador verde); a guiagem parte mirando o WP1. ' ...
        'Recarregue o aviao no X-Plane antes de cada voo (motor ~90-150 s por reload).'], ...
        'FontSize', 9, 'WordWrap', 'on');

    updateTable();
    updateMap();

    %% ==================== CALLBACKS ====================

    function mapClick(~, event)
        % so registra o press; a decisao clique-marca vs arrasto-pan e'
        % tomada no mapUp/mapMotion (limiar de 5 px)
        pt = event.IntersectionPoint;
        press.on = true;  press.moved = false;
        press.pt = pt(1:2);              % candidato a WP (coords do mapa)
        press.p0 = pt(1:2);              % ancora do pan (coords do mapa)
        press.px = fig.CurrentPoint;     % press em pixels da janela
    end

    function mapMotion(~, ~)
        if ~press.on, return; end
        if ~press.moved
            if norm(fig.CurrentPoint - press.px) < 5, return; end
            press.moved = true;          % virou arrasto: pan, nao marca WP
        end
        p = ax.CurrentPoint(1, 1:2);
        d = press.p0 - p;
        ax.XLim = ax.XLim + d(1);  ax.YLim = ax.YLim + d(2);
        vista = [ax.XLim ax.YLim];
    end

    function mapUp(~, ~)
        if ~press.on, return; end
        press.on = false;
        if press.moved, return; end      % foi pan
        pt = press.pt;
        wp_data(end+1, :) = [pt(2), pt(1), fldAlt.Value, fldVel.Value];
        updateTable();
        updateMap();
        lblStatus.Text = sprintf('WP%d: %s=%.0f  %s=%.0f  alt=%.0f  vel=%.0f', ...
            size(wp_data,1), nomeY(), pt(2), nomeX(), pt(1), fldAlt.Value, fldVel.Value);
        lblStatus.FontColor = corNeutra;
    end

    function mapWheel(~, event)
        % zoom no cursor, so com o mouse sobre o mapa
        cp = fig.CurrentPoint;
        apos = ax.Position;
        if cp(1) < apos(1) || cp(1) > apos(1)+apos(3) || ...
           cp(2) < apos(2) || cp(2) > apos(2)+apos(4), return; end
        f = 1.15 ^ event.VerticalScrollCount;      % >1 afasta, <1 aproxima
        c = ax.CurrentPoint(1, 1:2);
        xl = c(1) + (ax.XLim - c(1)) * f;
        yl = c(2) + (ax.YLim - c(2)) * f;
        if diff(xl) < 50 || diff(xl) > 40000, return; end   % limites de zoom
        ax.XLim = xl;  ax.YLim = yl;
        vista = [xl yl];
    end

    function resetVista()
        vista = [];
        updateMap();
    end

    function liveTick()
        % desenha a posicao ao vivo (XP_LIVE = [xN xE psi_eng psi t_xp],
        % publicada pelo xp_read_dh; NE -> frame da proa de engate)
        global XP_LIVE
        try
            if isempty(XP_LIVE) || isempty(hTrailLive) || ~isvalid(hTrailLive), return; end
            pe = XP_LIVE(3);
            xf = XP_LIVE(2)*cos(pe) - XP_LIVE(1)*sin(pe);   % a direita
            yf = XP_LIVE(1)*cos(pe) + XP_LIVE(2)*sin(pe);   % a frente
            liveTrail(end+1,:) = [xf yf];
            set(hTrailLive, 'XData', liveTrail(:,1), 'YData', liveTrail(:,2));
            th = XP_LIVE(4) - pe;                           % proa no frame do mapa
            L  = 0.035 * max(diff(ax.XLim), diff(ax.YLim));
            B  = [0 1.3*L; -0.5*L -0.6*L; 0.5*L -0.6*L];    % triangulo apontando +y
            c = cos(th); s = sin(th);
            set(hAviao, 'XData', xf + B(:,1)*c + B(:,2)*s, ...
                        'YData', yf - B(:,1)*s + B(:,2)*c);
            drawnow limitrate
        catch
        end
    end

    function removeLastWP(~, ~)
        if ~isempty(wp_data)
            wp_data(end, :) = [];
            updateTable(); updateMap();
            lblStatus.Text = 'Ultimo waypoint removido.';
            lblStatus.FontColor = corNeutra;
        end
    end

    function clearWPs(~, ~)
        wp_data = zeros(0,4); traj = [];
        updateTable(); updateMap();
        lblStatus.Text = 'Waypoints limpos.';
        lblStatus.FontColor = corNeutra;
    end

    function loadG2(~, ~)
        % o G2 oficial e' definido na proa de engate
        % OVAL "stadium" de 6 WPs: retas de 160 m + pontas semicirculares
        % de raio 100 (folgado vs raio natural de curva ~83 m do DH a
        % 12 m/s), WP no apice de cada ponta. R_accept 60 (espacamento
        % minimo entre WPs 141 m > 2R: circulos sem sobreposicao).
        % Perimetro 884 m -> ~126 s, cabe nos ~130 s de motor por reload.
        % (validado no SIL 2026-08-31: R80/pontas justas perdia WP3 por 2 m)
        wp_data = [160    0  600  12;
                   260  100  600  12;
                   160  200  600  12;
                     0  200  600  12;
                  -100  100  600  12;
                     0    0  600  12];
        fldRaccept.Value = 60;
        traj = [];
        updateTable(); updateMap();
        lblStatus.Text = 'Circuito OVAL carregado (6 WPs, retas 160 m + pontas R100; R_accept 60).';
        lblStatus.FontColor = corNeutra;
    end

    function tableEdited(~, event)
        r = event.Indices(1); c = event.Indices(2);
        if c >= 2 && c <= 5
            wp_data(r, c-1) = event.NewData;
            updateMap();
        end
    end

    function s = nomeX()
        s = 'a direita';
    end
    function s = nomeY()
        s = 'a frente';
    end

    function updateTable()
        tbl.ColumnName = {'#', [nomeY() ' (m)'], [nomeX() ' (m)'], 'Alt (m)', 'Vel (m/s)'};
        n = size(wp_data, 1);
        tbl.Data = [num2cell((1:n)'), num2cell(wp_data)];
        updateDuracao();
    end

    function updateDuracao()
        % espelha a formula do XP_missao: perimetro/12 x 1.5 + 15 (auto)
        MOTOR_S = 130;   % ~s de voo motorizado por reload (PENDENCIA_MOTOR.md)
        if fldTime.Value > 0
            T = fldTime.Value; org = 'manual';
        elseif isempty(wp_data)
            lblDur.Text = ''; return
        else
            pts = [0 0; wp_data(:, [2 1])];          % [x=direita y=frente]
            per = sum(vecnorm(diff(pts), 2, 2));
            T = ceil(per/12*1.5 + 15); org = 'auto';
        end
        if T <= MOTOR_S
            lblDur.Text = sprintf('Duracao %s: ~%d s  (motor ~%d s: OK)', org, T, MOTOR_S);
            lblDur.FontColor = corNeutra;
        else
            lblDur.Text = sprintf('Duracao %s: ~%d s > motor ~%d s — apos isso vira PLANEIO', ...
                org, T, MOTOR_S);
            lblDur.FontColor = corAviso;
        end
    end

    function updateMap()
        cla(ax); hold(ax, 'on');
        n = size(wp_data, 1);
        R = fldRaccept.Value;
        tc = linspace(0, 2*pi, 100);

        if ~isempty(traj)
            plot(ax, traj(:,1), traj(:,2), '-', 'Color', [0.35 0.65 1.0], 'LineWidth', 1.2);
        end
        if n > 0
            plot(ax, [0; wp_data(:,2)], [0; wp_data(:,1)], '--', ...
                'Color', [0.5 0.5 0.5], 'LineWidth', 1);
        end
        for i = 1:n
            plot(ax, wp_data(i,2) + R*cos(tc), wp_data(i,1) + R*sin(tc), ...
                'r--', 'LineWidth', 0.5);
            plot(ax, wp_data(i,2), wp_data(i,1), 'rs', ...
                'MarkerSize', 10, 'MarkerFaceColor', 'r');
            text(ax, wp_data(i,2) + R*0.3, wp_data(i,1) + R*0.3, ...
                sprintf('WP%d\n%.0fm', i, wp_data(i,3)), ...
                'FontSize', 9, 'FontWeight', 'bold', 'Color', 'r');
        end
        plot(ax, 0, 0, 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');

        if ~isempty(vista)
            % vista manual (pan/zoom do usuario) — preservada nos redraws
            ax.XLim = vista(1:2); ax.YLim = vista(3:4);
        elseif n > 0 || ~isempty(traj)
            allX = [0; wp_data(:,2)]; allY = [0; wp_data(:,1)];
            if ~isempty(traj), allX = [allX; traj(:,1)]; allY = [allY; traj(:,2)]; end
            m = max(fldRaccept.Value*2, 100);
            ax.XLim = [min(allX)-m, max(allX)+m];
            ax.YLim = [min(allY)-m, max(allY)+m];
        else
            ax.XLim = [-600 600]; ax.YLim = [-600 600];
        end
        ax.XLabel.String = [nomeX() ' (m)'];
        ax.YLabel.String = [nomeY() ' (m)'];
        ax.Title.String  = sprintf('Missao DH (%d WPs) — engate na origem', n);
        ax.ButtonDownFcn = @mapClick;
        hold(ax, 'off');
    end

    function runMission(~, ~)
        if isempty(wp_data)
            lblStatus.Text = 'Adicione pelo menos 1 waypoint.';
            lblStatus.FontColor = corErro;
            return;
        end
        sel = uiconfirm(fig, ...
            ['O aviao foi RECARREGADO no X-Plane (File > Open Aircraft)? ' ...
             'Sem reload o motor pode estar sem energia e o voo vira planeio.'], ...
            'Pre-voo', 'Options', {'Sim, voar', 'Cancelar'}, ...
            'DefaultOption', 1, 'CancelOption', 2);
        if ~strcmp(sel, 'Sim, voar'), return; end

        lblStatus.Text = 'Voando no X-Plane... (acompanhe o console do MATLAB)';
        lblStatus.FontColor = corInfo;
        btnVoar.Enable = 'off'; drawnow;

        % rastreio ao vivo: o timer desenha nas janelas de pause do pacing
        global XP_LIVE
        XP_LIVE = [];
        liveTrail = zeros(0,2);
        try, if ~isempty(tLive) && isvalid(tLive), stop(tLive); delete(tLive); end, catch, end
        hold(ax, 'on');
        hTrailLive = plot(ax, NaN, NaN, '-', 'Color', [0.35 0.65 1.0], 'LineWidth', 1.2);
        hAviao     = patch(ax, NaN, NaN, [0.10 0.45 0.90], 'EdgeColor', 'none');
        hold(ax, 'off');
        tLive = timer('Period', 0.5, 'ExecutionMode', 'fixedSpacing', ...
            'TimerFcn', @(~,~) liveTick());
        start(tLive);

        try
            % arma as variaveis do XP_missao no base (sobrevivem via setpref)
            assignin('base', 'XP_WPs_frame', wp_data);
            assignin('base', 'XP_WPs_NE',    []);
            assignin('base', 'XP_R_accept', fldRaccept.Value);
            if fldTime.Value > 0
                assignin('base', 'XP_TimeXP', fldTime.Value);
            else
                assignin('base', 'XP_TimeXP', []);
            end
            assignin('base', 'XP_tag', 'GUI');

            evalin('base', ['run(''' fullfile(xpDir, 'XP_missao.m') ''')']);

            % overlay da trajetoria voada (NE -> proa de engate)
            try
                voo = evalin('base', 'voo');
                xN = voo.Y(11,:)'; xE = voo.Y(12,:)';
                psr = deg2rad(voo.psi_engate);
                traj = [xE*cos(psr) - xN*sin(psr), ...   % a direita
                        xN*cos(psr) + xE*sin(psr)];      % a frente
            catch, traj = []; end
            updateMap();

            % status: quantos WPs a guiagem avancou (resumo completo no console)
            txt = 'Voo concluido. Resumo no console; .mat + PNGs em xplane/voos.';
            try
                nWP = size(voo.WPs,1);
                txt = sprintf('Voo concluido: guiagem avancou ate o WP %d de %d (capturas no console). PNGs em xplane/voos.', ...
                    voo.wp_idx(end), nWP);
            catch, end
            lblStatus.Text = txt;
            lblStatus.FontColor = corOk;
        catch ME
            lblStatus.Text = sprintf('Erro: %s', ME.message);
            lblStatus.FontColor = corErro;
            fprintf('Erro na missao:\n%s\n', getReport(ME));
        end
        try, stop(tLive); delete(tLive); catch, end
        tLive = [];
        btnVoar.Enable = 'on';
    end
end
