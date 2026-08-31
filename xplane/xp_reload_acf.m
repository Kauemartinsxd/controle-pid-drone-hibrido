function xp_reload_acf()
% xp_reload_acf — recarrega o DH no X-Plane 9 SEM tocar na UI manualmente.
% Substitui o ritual File -> Open Aircraft que "recarrega a bateria" do
% motor eletrico (o estoque de energia do XP9 e' interno, nao exposto por
% dref — ver PENDENCIA_MOTOR.md; sondas de 2026-08-31 descartaram fuel,
% bateria eletrica e FADEC).
%
% Como funciona:
%   1) traz a janela do X-Plane p/ frente (AppActivate 'X-System');
%   2) pacote UDP legado MENU "600" na porta 49000 = menu Open Aircraft
%      (numeros em Resources/menus/English/X-Plane.txt; funciona ate em voo);
%   3) 2 cliques de mouse REAIS (java.awt.Robot) no dialogo:
%      linha 'DH-Lon-REV-03.acf' e botao 'Open' — coordenadas relativas
%      ao CENTRO da area cliente (o dialogo do XP9 e' centrado e de
%      tamanho fixo), medidas na calibracao de 2026-08-31;
%   4) verifica: tempo de voo resetou, aviao no solo, TRQ > +0.05 com
%      throttle (criterio de motor VIVO — RPM no solo e' invalido).
%
% Requisitos: X-Plane 9 visivel (janela nao minimizada); arquivo
% DH-Lon-REV-03.acf como 2o item da pasta Radio Control no dialogo
% (apos 'airfoils'; novos arquivos que ordenem ANTES dele quebram o
% clique — a verificacao acusa e o script erra com mensagem clara).
%
% MOUS/CHAR por UDP NAO funcionam no dialogo (sao do painel);
% ACFN troca o modelo mas NAO rearma a energia — por isso os cliques.

here = fileparts(mfilename('fullpath'));
addpath(here);
addpath(fullfile(fileparts(fileparts(here)), 'trabalho_julio', 'PIPER-1-6-roll_back', ...
    'PIPER-1-6-roll_back', 'xplane', 'XPlaneConnect-master', 'MATLAB'));
import XPlaneConnect.*

% ---- 0) geometria da janela (client rect via PowerShell/user32) ----
ps1 = fullfile(tempdir, 'xp_client_rect.ps1');
fid = fopen(ps1, 'w');
fprintf(fid, '%s\n', ...
    'Add-Type @"', ...
    'using System;', ...
    'using System.Runtime.InteropServices;', ...
    'public class W {', ...
    '  [DllImport("user32.dll")] public static extern bool GetClientRect(IntPtr h, out R r);', ...
    '  [DllImport("user32.dll")] public static extern bool ClientToScreen(IntPtr h, ref P p);', ...
    '  public struct R { public int L, T, Rt, B; }', ...
    '  public struct P { public int X, Y; }', ...
    '}', ...
    '"@', ...
    '$p = Get-Process X-Plane -ErrorAction Stop | Select-Object -First 1', ...
    '(New-Object -ComObject WScript.Shell).AppActivate(''X-System'') | Out-Null', ...
    '$r = New-Object W+R; [W]::GetClientRect($p.MainWindowHandle, [ref]$r) | Out-Null', ...
    '$q = New-Object W+P; $q.X = 0; $q.Y = 0', ...
    '[W]::ClientToScreen($p.MainWindowHandle, [ref]$q) | Out-Null', ...
    'Write-Output "$($q.X) $($q.Y) $($r.Rt) $($r.B)"');
fclose(fid);
[st, out] = system(['powershell -NoProfile -ExecutionPolicy Bypass -File "' ps1 '"']);
g = sscanf(strtrim(out), '%d %d %d %d');   % [left top width height] do client
if st ~= 0 || numel(g) < 4
    error('xp_reload_acf: nao achei a janela do X-Plane (%s)', strtrim(out));
end
cx = g(1) + g(3)/2;  cy = g(2) + g(4)/2;   % centro em pixels de TELA
pause(0.5);                                 % janela vindo p/ frente

% offsets medidos (client 2560x1377; dialogo centrado, tamanho fixo)
p_row  = [cx - 188, cy + 26];    % linha 'DH-Lon-REV-03.acf'
p_open = [cx      , cy + 217];   % botao 'Open'

% ---- 1) t_xp de referencia (para detectar o reset) ----
global GlobalSocket
t_pre = NaN;
try
    t_pre = max(double(getDREFs({'sim/time/total_flight_time_sec'}, GlobalSocket)));
catch
    try, closeUDP(GlobalSocket); catch, end
    try
        GlobalSocket = openUDP('127.0.0.1', 49009, 0, 500);
        t_pre = max(double(getDREFs({'sim/time/total_flight_time_sec'}, GlobalSocket)));
    catch
        % XPC mudo (dialogo ja aberto?) — segue mesmo assim
    end
end

% ---- 2) abre o dialogo: MENU "600" ----
u = udpport('datagram');
write(u, [uint8('MENU'), uint8(0), uint8('600'), uint8(0)], 'uint8', '127.0.0.1', 49000);
clear u
pause(2.0);

% ---- 3) cliques ----
clica(p_row);  pause(0.6);
clica(p_open);
fprintf('xp_reload_acf: dialogo acionado, aguardando o load');
for k = 1:5, pause(2); fprintf('.'); end
fprintf('\n');

% ---- 4) verificacao ----
try, closeUDP(GlobalSocket); catch, end
GlobalSocket = openUDP('127.0.0.1', 49009, 0, 500);
s = double(getDREFs({'sim/time/total_flight_time_sec', ...
    'sim/flightmodel/position/y_agl','sim/flightmodel/position/true_airspeed'}, GlobalSocket));
resetou = isnan(t_pre) || s(1) < t_pre;
if ~(resetou && s(2) < 3 && s(3) < 3)
    error(['xp_reload_acf: reload NAO confirmado (t %.1f->%.1f, AGL %.1f, VT %.1f). ' ...
        'Confira a janela do X-Plane (dialogo aberto? arquivo novo mudou a ordem da lista?).'], ...
        t_pre, s(1), s(2), s(3));
end
xp_send_dh([1.0; 0; 0; 0]); pause(2.5);
raw = getDREFs({'sim/flightmodel/engine/ENGN_TRQ'}, GlobalSocket);
if iscell(raw), a = double(raw{1}); else, a = double(raw); end
xp_send_dh([0; 0; 0; 0]);
if a(1) <= 0.05
    error('xp_reload_acf: aviao recarregou mas motor MORTO (TRQ %.3f) — reload nao pegou; rode de novo.', a(1));
end
fprintf('xp_reload_acf: OK — aviao na rampa, motor VIVO (TRQ %.2f), bateria cheia.\n', a(1));
end

function clica(p)
import java.awt.Robot; import java.awt.event.InputEvent;
r = Robot();
r.mouseMove(round(p(1)), round(p(2))); pause(0.15);
r.mousePress(InputEvent.BUTTON1_DOWN_MASK); pause(0.05);
r.mouseRelease(InputEvent.BUTTON1_DOWN_MASK);
end
