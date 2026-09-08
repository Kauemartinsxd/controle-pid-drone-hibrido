function gui_guiagem_NL()
%GUI_GUIAGEM_NL  Mapa clicavel de waypoints para o guiagem_NL (LQRy + guiagem LOS no modelo NL).
%
%   >> gui_guiagem_NL
%   1) clique no mapa para marcar waypoints (altitude e velocidade dos campos ao lado),
%      ou use os botoes "Circuito OVAL" / "Circuito AGRESSIVO";
%   2) escolha os ganhos (originais do Mirko por padrao, ou v3) e o motor;
%   3) SIMULAR NO NL -> roda guiagem_NL.m e desenha a trajetoria por cima do mapa.
%
% Referencial: o aviao engata na ORIGEM com proa 0 ("para cima" no mapa = Norte = frente).
% Arraste para mover o mapa, roda do mouse para zoom, "Ajustar vista" volta ao enquadramento.
% O resumo completo (capturas, extremos) sai no console; figuras e .mat em resultados/.

    here = fileparts(mfilename('fullpath'));
    wp_data = zeros(0, 4);        % [N E h V]
    traj = [];                    % trajetoria da ultima simulacao [E N]
    vista = [];                   % [] = automatico; [x1 x2 y1 y2] = pan/zoom manual
    press = struct('on', false, 'pt', [0 0], 'p0', [0 0], 'px', [0 0], 'moved', false);

    %% ---------- figura ----------
    fig = uifigure('Name', 'LQRy + guiagem por waypoints no modelo NL (DH)', 'Position', [80 80 1000 640]);
    ax = uiaxes(fig, 'Position', [20 50 580 570]);
    ax.XGrid = 'on'; ax.YGrid = 'on'; ax.Box = 'on'; ax.FontSize = 11;
    hold(ax, 'on'); disableDefaultInteractivity(ax);
    ax.ButtonDownFcn = @mapClick;
    fig.WindowButtonMotionFcn = @mapMotion; fig.WindowButtonUpFcn = @mapUp; fig.WindowScrollWheelFcn = @mapWheel;
    uibutton(fig, 'Position', [30 14 110 24], 'Text', 'Ajustar vista', 'ButtonPushedFcn', @(~,~) resetVista());

    %% ---------- painel ----------
    px = 620; pw = 360;
    uilabel(fig, 'Position', [px 600 pw 30], 'Text', 'Waypoints [N  E  h  V]', 'FontSize', 16, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    tbl = uitable(fig, 'Position', [px 380 pw 215], 'ColumnWidth', {26, 80, 80, 66, 60}, ...
        'ColumnEditable', [false true true true true], 'CellEditCallback', @tableEdited);
    uibutton(fig, 'Position', [px 345 170 28], 'Text', 'Remover ultimo', 'ButtonPushedFcn', @(~,~) removeLast());
    uibutton(fig, 'Position', [px+190 345 170 28], 'Text', 'Limpar tudo', 'ButtonPushedFcn', @(~,~) clearAll());
    uibutton(fig, 'Position', [px 312 170 28], 'Text', 'Circuito OVAL (6 WPs)', 'ButtonPushedFcn', @(~,~) preset('oval'));
    uibutton(fig, 'Position', [px+190 312 170 28], 'Text', 'Circuito AGRESSIVO (4 WPs)', 'ButtonPushedFcn', @(~,~) preset('agressivo'));

    y = 272;
    uilabel(fig, 'Position', [px y 170 22], 'Text', 'Altitude do proximo WP [m]:');
    fldAlt = uieditfield(fig, 'numeric', 'Position', [px+180 y 80 22], 'Value', 600, 'Limits', [400 1000]);
    y = y - 28;
    uilabel(fig, 'Position', [px y 170 22], 'Text', 'Vel. do proximo WP [m/s]:');
    fldVel = uieditfield(fig, 'numeric', 'Position', [px+180 y 80 22], 'Value', 15, 'Limits', [8 20]);
    y = y - 28;
    uilabel(fig, 'Position', [px y 170 22], 'Text', 'Raio de captura [m]:');
    fldR = uieditfield(fig, 'numeric', 'Position', [px+180 y 80 22], 'Value', 100, 'Limits', [10 500]);
    y = y - 28;
    uilabel(fig, 'Position', [px y 170 22], 'Text', 'Ganhos do LQRy:');
    ddG = uidropdown(fig, 'Position', [px+180 y 180 22], 'Items', {'originais (Mirko)', 'v3 (re-sintese)'}, 'ItemsData', {'mirko', 'v3'}, 'Value', 'mirko');
    y = y - 28;
    uilabel(fig, 'Position', [px y 170 22], 'Text', 'Planta / velocidade de engate:');
    ddV = uidropdown(fig, 'Position', [px+180 y 180 22], 'Items', {'auto (mediana dos WPs)', '12 m/s', '15 m/s', '18 m/s'}, 'ItemsData', {0, 12, 15, 18}, 'Value', 0);
    y = y - 28;
    uilabel(fig, 'Position', [px y 170 22], 'Text', 'Motor: constante de tempo [s]:');
    fldTau = uieditfield(fig, 'numeric', 'Position', [px+180 y 80 22], 'Value', 0.3, 'Limits', [0.05 10]);
    uilabel(fig, 'Position', [px+265 y 95 22], 'Text', '(X-Plane 9: 3,5)', 'FontSize', 9);

    btn = uibutton(fig, 'Position', [px 60 pw 46], 'Text', 'SIMULAR NO NL', 'FontSize', 16, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.25 0.45 0.75], 'FontColor', 'white', 'ButtonPushedFcn', @(~,~) runSim());
    lbl = uilabel(fig, 'Position', [px 8 pw 46], 'Text', 'Pronto. Clique marca waypoint; arraste move o mapa; roda do mouse da zoom.', 'FontSize', 10, 'WordWrap', 'on');
    corNeutra = lbl.FontColor; corOk = [0.25 0.75 0.35]; corErro = [0.90 0.30 0.30]; corInfo = [0.40 0.60 1.00];

    updateTable(); updateMap();

    %% ---------- callbacks ----------
    function mapClick(~, ev)
        pt = ev.IntersectionPoint; press.on = true; press.moved = false;
        press.pt = pt(1:2); press.p0 = pt(1:2); press.px = fig.CurrentPoint;
    end
    function mapMotion(~, ~)
        if ~press.on, return; end
        if ~press.moved, if norm(fig.CurrentPoint - press.px) < 5, return; end, press.moved = true; end
        p = ax.CurrentPoint(1, 1:2); d = press.p0 - p;
        ax.XLim = ax.XLim + d(1); ax.YLim = ax.YLim + d(2); vista = [ax.XLim ax.YLim];
    end
    function mapUp(~, ~)
        if ~press.on, return; end
        press.on = false; if press.moved, return; end
        pt = press.pt;
        wp_data(end+1, :) = [pt(2), pt(1), fldAlt.Value, fldVel.Value];   % [N E h V]
        updateTable(); updateMap();
        lbl.Text = sprintf('WP%d: N = %.0f, E = %.0f, h = %.0f m, V = %.0f m/s', size(wp_data,1), pt(2), pt(1), fldAlt.Value, fldVel.Value); lbl.FontColor = corNeutra;
    end
    function mapWheel(~, ev)
        cp = fig.CurrentPoint; ap = ax.Position;
        if cp(1) < ap(1) || cp(1) > ap(1)+ap(3) || cp(2) < ap(2) || cp(2) > ap(2)+ap(4), return; end
        f = 1.15 ^ ev.VerticalScrollCount; c = ax.CurrentPoint(1, 1:2);
        xl = c(1) + (ax.XLim - c(1))*f; yl = c(2) + (ax.YLim - c(2))*f;
        if diff(xl) < 50 || diff(xl) > 40000, return; end
        ax.XLim = xl; ax.YLim = yl; vista = [xl yl];
    end
    function resetVista(), vista = []; updateMap(); end
    function removeLast()
        if ~isempty(wp_data), wp_data(end, :) = []; updateTable(); updateMap(); end
    end
    function clearAll()
        wp_data = zeros(0, 4); traj = []; updateTable(); updateMap();
    end
    function preset(nome)
        switch nome
            case 'oval'
                wp_data = [256 0 600 15; 416 160 600 15; 256 320 600 15; 0 320 600 15; -160 160 600 15; 0 0 600 15]; fldR.Value = 100;
                lbl.Text = 'Circuito OVAL: retas 256 m + pontas R 160 m, 15 m/s, R_accept 100 m (~1417 m).';
            case 'agressivo'
                wp_data = [260 0 620 18; 260 260 600 15; 0 260 620 18; 0 0 600 15]; fldR.Value = 110;
                lbl.Text = 'Circuito AGRESSIVO: quadrado 260 m, curvas de 90 graus, h 600/620 m, V 18/15 m/s, R_accept 110 m.';
        end
        lbl.FontColor = corNeutra; traj = []; updateTable(); updateMap();
    end
    function tableEdited(~, ev)
        r = ev.Indices(1); c = ev.Indices(2);
        if c >= 2, wp_data(r, c-1) = ev.NewData; updateMap(); end
    end
    function updateTable()
        tbl.ColumnName = {'#', 'N (m)', 'E (m)', 'h (m)', 'V (m/s)'};
        n = size(wp_data, 1); tbl.Data = [num2cell((1:n)'), num2cell(wp_data)];
    end
    function updateMap()
        cla(ax); hold(ax, 'on');
        n = size(wp_data, 1); R = fldR.Value; tc = linspace(0, 2*pi, 100);
        if ~isempty(traj), plot(ax, traj(:,1), traj(:,2), '-', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.4); end
        if n > 0, plot(ax, [0; wp_data(:,2)], [0; wp_data(:,1)], '--', 'Color', [0.5 0.5 0.5]); end
        for k = 1:n
            plot(ax, wp_data(k,2) + R*cos(tc), wp_data(k,1) + R*sin(tc), 'r--', 'LineWidth', 0.5);
            plot(ax, wp_data(k,2), wp_data(k,1), 'rs', 'MarkerSize', 9, 'MarkerFaceColor', 'r');
            text(ax, wp_data(k,2) + 0.3*R, wp_data(k,1) + 0.3*R, sprintf('WP%d\n%.0f m, %.0f m/s', k, wp_data(k,3), wp_data(k,4)), 'FontSize', 9, 'FontWeight', 'bold', 'Color', 'r');
        end
        plot(ax, 0, 0, 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
        if ~isempty(vista), ax.XLim = vista(1:2); ax.YLim = vista(3:4);
        elseif n > 0 || ~isempty(traj)
            X = [0; wp_data(:,2)]; Y = [0; wp_data(:,1)];
            if ~isempty(traj), X = [X; traj(:,1)]; Y = [Y; traj(:,2)]; end
            m = max(2*R, 100); ax.XLim = [min(X)-m, max(X)+m]; ax.YLim = [min(Y)-m, max(Y)+m];
        else, ax.XLim = [-600 600]; ax.YLim = [-600 600]; end
        ax.XLabel.String = 'E [m] (a direita do engate)'; ax.YLabel.String = 'N [m] (a frente do engate)';
        ax.Title.String = sprintf('Missao com %d WPs (engate na origem, proa 0)', n);
        ax.ButtonDownFcn = @mapClick; hold(ax, 'off');
    end
    function vt = vtPlanta()
        vt = ddV.Value;
        if vt == 0, vm = median(wp_data(:,4)); cand = [12 15 18]; [~, k] = min(abs(cand - vm) + 1e-6*cand); vt = cand(k); end
    end
    function runSim()
        if isempty(wp_data), lbl.Text = 'Adicione pelo menos 1 waypoint.'; lbl.FontColor = corErro; return; end
        lbl.Text = 'Simulando... (acompanhe o console)'; lbl.FontColor = corInfo; btn.Enable = 'off'; drawnow;
        try
            evalin('base', 'clear ganhos VT_missao WPs R_accept eng_tau');
            assignin('base', 'ganhos', ddG.Value); assignin('base', 'VT_missao', vtPlanta());
            assignin('base', 'WPs', wp_data); assignin('base', 'R_accept', fldR.Value); assignin('base', 'eng_tau', fldTau.Value);
            evalin('base', ['run(''' fullfile(here, 'guiagem_NL.m') ''')']);
            Y = evalin('base', 'Y'); n_hit = evalin('base', 'n_hit');
            traj = [Y(12,:)', Y(11,:)'];                                  % [E N]
            updateMap();
            lbl.Text = sprintf('Concluido: %d/%d capturas (ganhos %s, planta %g m/s). Resumo no console; figuras em resultados/.', ...
                n_hit, size(wp_data,1), ddG.Value, vtPlanta());
            lbl.FontColor = corOk;
        catch ME
            lbl.Text = ['Erro: ' ME.message]; lbl.FontColor = corErro; fprintf('%s\n', getReport(ME));
        end
        btn.Enable = 'on';
    end
end
