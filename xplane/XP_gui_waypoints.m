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
%   1) Escolha o referencial (proa de engate = "para cima" e' a frente
%      do nariz no engate; NE = Norte/Leste absolutos do ponto de engate)
%   2) Clique no mapa para adicionar WPs (alt/vel dos campos ao lado)
%   3) RECARREGUE o aviao no X-Plane (File > Open Aircraft) — energia
%      do motor eletrico do XP9 dura ~90-150 s por reload
%   4) VOAR NO X-PLANE
%
% Pos-voo: .mat + PNGs em xplane/voos (via XP_missao/plot_XP_missao) e
% trajetoria voada sobreposta no mapa da GUI (tracejado azul).
%
% O engate e' a ORIGEM do mapa (marcador verde): a guiagem parte mirando
% o WP1 da tabela — nao inclua a origem como waypoint.

    xpDir = fileparts(mfilename('fullpath'));

    %% ========== Estado ==========
    wp_data  = zeros(0,4);      % [frente/N  direita/E  altMSL  vel]
    traj     = [];              % trajetoria voada (mapa) p/ overlay
    modoNE   = false;           % false = proa de engate; true = NE

    %% ========== Figura ==========
    fig = uifigure('Name', 'Missao DH no X-Plane — waypoints (PID dissertacao)', ...
        'Position', [80 80 1000 650], 'Color', [0.95 0.95 0.95]);

    ax = uiaxes(fig, 'Position', [20 20 580 600]);
    ax.XGrid = 'on'; ax.YGrid = 'on'; ax.Box = 'on'; ax.FontSize = 11;
    hold(ax, 'on');
    ax.ButtonDownFcn = @mapClick;

    %% ========== Painel lateral ==========
    panelX = 620; panelW = 360;

    uilabel(fig, 'Position', [panelX 605 panelW 30], ...
        'Text', 'Waypoints — DH (PID)', 'FontSize', 16, ...
        'FontWeight', 'bold', 'HorizontalAlignment', 'center');

    uilabel(fig, 'Position', [panelX 575 90 22], 'Text', 'Referencial:', 'FontSize', 11);
    ddFrame = uidropdown(fig, 'Position', [panelX+95 575 255 22], ...
        'Items', {'Proa de engate (cima = nariz)', 'NE do ponto de engate'}, ...
        'ValueChangedFcn', @frameChanged);

    tbl = uitable(fig, 'Position', [panelX 360 panelW 205], ...
        'ColumnWidth', {30, 78, 78, 70, 62}, ...
        'ColumnEditable', [false true true true true], ...
        'CellEditCallback', @tableEdited);

    uibutton(fig, 'Position', [panelX 320 170 30], 'Text', 'Remover ultimo', ...
        'ButtonPushedFcn', @removeLastWP);
    uibutton(fig, 'Position', [panelX+180 320 170 30], 'Text', 'Limpar tudo', ...
        'ButtonPushedFcn', @clearWPs);
    uibutton(fig, 'Position', [panelX 285 panelW 26], 'Text', 'Carregar G2 (quadrado 500 m)', ...
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
        'Value', 0, 'Limits', [0 1200]);
    uilabel(fig, 'Position', [panelX+255 yPos 30 22], 'Text', 's');

    btnVoar = uibutton(fig, 'Position', [panelX 90 panelW 46], ...
        'Text', 'VOAR NO X-PLANE', 'FontSize', 16, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.2 0.6 0.2], 'FontColor', 'white', ...
        'ButtonPushedFcn', @runMission);

    lblStatus = uilabel(fig, 'Position', [panelX 52 panelW 34], ...
        'Text', 'Pronto. Clique no mapa para adicionar waypoints.', ...
        'FontSize', 10, 'WordWrap', 'on', 'FontColor', [0.3 0.3 0.3]);

    uilabel(fig, 'Position', [panelX 6 panelW 44], ...
        'Text', ['Engate na ORIGEM (marcador verde); a guiagem parte mirando o WP1. ' ...
        'Recarregue o aviao no X-Plane antes de cada voo (motor ~90-150 s por reload).'], ...
        'FontSize', 9, 'WordWrap', 'on', 'FontColor', [0.5 0.5 0.5]);

    updateTable();
    updateMap();

    %% ==================== CALLBACKS ====================

    function frameChanged(~, ~)
        modoNE = strncmp(ddFrame.Value, 'NE', 2);
        traj = [];                 % trajetoria antiga era do outro frame
        updateTable();
        updateMap();
    end

    function mapClick(~, event)
        pt = event.IntersectionPoint;
        wp_data(end+1, :) = [pt(2), pt(1), fldAlt.Value, fldVel.Value];
        updateTable();
        updateMap();
        lblStatus.Text = sprintf('WP%d: %s=%.0f  %s=%.0f  alt=%.0f  vel=%.0f', ...
            size(wp_data,1), nomeY(), pt(2), nomeX(), pt(1), fldAlt.Value, fldVel.Value);
        lblStatus.FontColor = [0.3 0.3 0.3];
    end

    function removeLastWP(~, ~)
        if ~isempty(wp_data)
            wp_data(end, :) = [];
            updateTable(); updateMap();
            lblStatus.Text = 'Ultimo waypoint removido.';
        end
    end

    function clearWPs(~, ~)
        wp_data = zeros(0,4); traj = [];
        updateTable(); updateMap();
        lblStatus.Text = 'Waypoints limpos.';
    end

    function loadG2(~, ~)
        % o G2 oficial e' definido na proa de engate
        if modoNE
            ddFrame.Value = ddFrame.Items{1};
            modoNE = false;
        end
        wp_data = [500    0  600  12;
                   500  500  600  12;
                     0  500  600  12;
                     0    0  600  12];
        traj = [];
        updateTable(); updateMap();
        lblStatus.Text = 'Missao G2 carregada (quadrado 500x500 m, 12 m/s).';
    end

    function tableEdited(~, event)
        r = event.Indices(1); c = event.Indices(2);
        if c >= 2 && c <= 5
            wp_data(r, c-1) = event.NewData;
            updateMap();
        end
    end

    function s = nomeX()
        if modoNE, s = 'Leste'; else, s = 'a direita'; end
    end
    function s = nomeY()
        if modoNE, s = 'Norte'; else, s = 'a frente'; end
    end

    function updateTable()
        tbl.ColumnName = {'#', [nomeY() ' (m)'], [nomeX() ' (m)'], 'Alt (m)', 'Vel (m/s)'};
        n = size(wp_data, 1);
        tbl.Data = [num2cell((1:n)'), num2cell(wp_data)];
    end

    function updateMap()
        cla(ax); hold(ax, 'on');
        n = size(wp_data, 1);
        R = fldRaccept.Value;
        tc = linspace(0, 2*pi, 100);

        if ~isempty(traj)
            plot(ax, traj(:,1), traj(:,2), 'b-', 'LineWidth', 1.2);
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

        if n > 0 || ~isempty(traj)
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
            lblStatus.FontColor = [0.8 0 0];
            return;
        end
        sel = uiconfirm(fig, ...
            ['O aviao foi RECARREGADO no X-Plane (File > Open Aircraft)? ' ...
             'Sem reload o motor pode estar sem energia e o voo vira planeio.'], ...
            'Pre-voo', 'Options', {'Sim, voar', 'Cancelar'}, ...
            'DefaultOption', 1, 'CancelOption', 2);
        if ~strcmp(sel, 'Sim, voar'), return; end

        lblStatus.Text = 'Voando no X-Plane... (acompanhe o console do MATLAB)';
        lblStatus.FontColor = [0 0 0.6];
        btnVoar.Enable = 'off'; drawnow;

        try
            % arma as variaveis do XP_missao no base (sobrevivem via setpref)
            if modoNE
                assignin('base', 'XP_WPs_NE',    wp_data);
                assignin('base', 'XP_WPs_frame', []);
            else
                assignin('base', 'XP_WPs_frame', wp_data);
                assignin('base', 'XP_WPs_NE',    []);
            end
            assignin('base', 'XP_R_accept', fldRaccept.Value);
            if fldTime.Value > 0
                assignin('base', 'XP_TimeXP', fldTime.Value);
            else
                assignin('base', 'XP_TimeXP', []);
            end
            assignin('base', 'XP_tag', 'GUI');

            evalin('base', ['run(''' fullfile(xpDir, 'XP_missao.m') ''')']);

            % overlay da trajetoria voada (NE -> frame se preciso)
            try
                voo = evalin('base', 'voo');
                xN = voo.Y(11,:)'; xE = voo.Y(12,:)';
                if modoNE
                    traj = [xE, xN];
                else
                    psr = deg2rad(voo.psi_engate);
                    traj = [xE*cos(psr) - xN*sin(psr), ...   % a direita
                            xN*cos(psr) + xE*sin(psr)];      % a frente
                end
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
            lblStatus.FontColor = [0 0.5 0];
        catch ME
            lblStatus.Text = sprintf('Erro: %s', ME.message);
            lblStatus.FontColor = [0.8 0 0];
            fprintf('Erro na missao:\n%s\n', getReport(ME));
        end
        btnVoar.Enable = 'on';
    end
end
