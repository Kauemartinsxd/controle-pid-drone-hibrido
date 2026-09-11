# Verificação do pacote `LQRy_Guiagem.rar` do Mirko (2026-09-10)

Pasta de verificação **fora** dos arquivos do Mirko (nada em `..\` foi alterado). Harness:
`verifica_guiagem.m` (função; mesmas variáveis e valores do `guiagem_NL.m`, passadas por
`Simulink.SimulationInput/setVariable`, sem `clear all`), `captura_sequencial.m`,
`tabela_verificacao.m`, `dump_sistema.m`, `dump_linhas.m`, `compara_entradas_controlador.m`.
Resultados (PNG + .mat) em `resultados/`. Ganhos ψ antigos (os do `Nova pasta`, idênticos aos
do pacote enviado em 03/09) em `ganhos_mirko_antigo/`.

## 1. O que mudou em relação ao pacote enviado em 03/09 (`lqry_v3/pacote_mirko_guiagem_NL`)

| Arquivo | Situação |
|---|---|
| `ganhos_mirko/Ganho_hold_psi.mat` (09/09 08:56) | **único ganho alterado** (os outros 4 `.mat` + `Dados_Trim.mat` são idênticos, md5) |
| `ganhos_v3/`, `planta/` | idênticos |
| `CL_NL_DH_GUIA_Original.slx` | = o modelo que eu mandei, re-salvo (XML só difere em cache de Scope e ordem de branches) |
| `CL_NL_DH_GUIA.slx` (novo, "em blocos") | ver §3 — **não** é só reorganização |
| `CL_NL_DH_18_jun_2026.slx` | = original de junho (md5) |
| `guiagem_NL.m` | `clear all; clc` no topo (**quebra o botão SIMULAR NO NL da GUI**, que passa `ganhos/WPs/R_accept/VT_missao/eng_tau` pelo workspace), `bdclose` comentado, `save` do `.mat` comentado, print do engate removido, `mdl` fixo em `CL_NL_DH_GUIA` |
| `plot_guiagem_NL.m` | `exportgraphics` comentado (não salva mais PNG) |
| `gui_guiagem_NL.m`, `README.md` | idênticos |

## 2. O "ganho de ψ" novo é o ψ Hold inteiro re-sintonizado

`Ganho_hold_psi.mat` tem `GstateLat_psi{1..9}` (5 estados) e `Gintlat_psi{1..9}`; os 9 conjuntos
mudaram, de forma diferente por planta (razão novo/antigo):

| Planta | `Gintlat_psi` | `GstateLat_psi` (5 termos) |
|---|---|---|
| 12 m/s (1–3) | ×0,375 | ×2,9 / ×2,7 / ×2,3 / ×2,7 / ×1,85 |
| 15 m/s H590 (4) | ×0,375 | ×2,6 / ×2,7 / ×0,5 / ×2,5 / ×1,6 |
| **15 m/s H600 (5, a da missão)** | **×0,027** (−5,45 → −0,145) | **×0,14 / ×0,31 / ×0,20 / ×0,30 / ×0,12** (ψ→φ_ref −5,05 → −0,63) |
| 15 m/s H610 (6) | ×0,027 | idem planta 5 |
| 18 m/s (7–9) | ×0,12 | ×0,67 / ×1,08 / sinal trocado no 3.º / ×0,84 / ×0,48 |

Estrutura intacta (só Q/R/ganhos), como combinado. Mas na dissertação descrever como
"ψ Hold re-sintonizado (ganhos de estado e integral nas 9 plantas)", não "um ganho".

## 3. O modelo novo `CL_NL_DH_GUIA.slx` (comparado ao `_Original`)

**Controlador: intacto.** 109 de 111 blocos com parâmetros iguais; 71/71 ligações internas
iguais; as 15 entradas (11 saídas da planta + 4 referências da guiagem) chegam aos mesmos
blocos/portas (`Sum10:2, Sum1:2, Sum12:2, Sum2:2, Sum18:2, Sum8:2, Sum18:1, Sum8:1, Sum10:1,
Switch2:1`). Os 2 blocos diferentes: `Integrator2` (Alt Hold) com CI `xi_alt0` → `0`
(perde o engate sem transiente; harness, não controlador) e `Degrees to Radians3/Gain1`
180/π → π/180 (só no caminho `ElevActuator → Mux → Outport → Scope`, log).

**Planta: perdeu os atuadores.** Ausentes no novo: `Sat_Elevator/Aileron/Rudder` (±15°),
`Sat_Throttle` [0,1], `RL_*` (150°/s, manete 1/s), `Servo_*` (0,05 s), `Eng_Throttle`
(`eng.tau`), `alpha_protection`, `Sum_Hrel`, `C_h_ref0`. O comando do controlador (+trim) vai
**direto** à `sfunction_DH` (`Sum_trim → Dmx_cmd4 → Mux_u4act → Mux_U7 → Planta_NL_DH`);
`eng_tau`, `sat_deg`, `act_*` do script **não têm efeito** nesse modelo. A altitude de controle
virou `Subsystem1` = ∫ V_T·(θ−α) dt a partir de 0 (o estimador dele), em vez de h − h0.

## 4. Corridas (planta 5, 15 m/s, R_accept oval 100 m / agressivo 110 m)

`cap_sequencial` = capturas na ordem da missão (o critério "dist min" do `guiagem_NL.m`
conta o último WP, na origem, em t = 0, então "2/6" = 1 real). `fim_auto` = a simulação parou
sozinha 5 s após o último WP (prova de missão completa). `manete_fora01` = % do tempo em que
o **comando** de manete pediu < 0 ou > 1 (`U_xp` é o comando, antes do atuador).

| tag | modelo | missão | τ motor | cap. dist min | cap. sequencial | fim auto | departure | manete fora [0,1] | sup. > 15° | φ máx | α máx | h | V_T |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| A | novo (sem atuadores) | oval | 0,3 | 6/6 | **6/6** | sim (86,6 s) | não | 0 % | 0 % | 17,6° | 8,8° | 599,7–600,0 | 14,9–15,0 |
| B | Original (atuadores) | oval | 0,3 | 6/6 | **6/6** | sim (86,6 s) | não | 0 % | 0 % | 17,6° | 8,5° | 600,0 | 14,8–15,0 |
| C | Original, **ψ antigo** | oval | 0,3 | 2/6 | 1/6 | não | **sim** (φ 488°, h −2186 m) | 93 % | 94 % | 488° | 18,8° | −2186..600 | 14,7–21,4 |
| D | novo, **ψ antigo** | oval | 0,3 | (T 20 s) 2/6 | 1/6 | — | **sim** aos 15–20 s; NaN na S-function aos 20–25 s | 23 % | 32 % | 1326° | 90° | 589–604 | 0,4–31,5 |
| E | Original (atuadores) | agressivo | 0,3 | 4/4 | **4/4** | sim (57,4 s) | não | **88 %** | 1 % | 23,9° | **22,7°** | 599,7–621,0 | **10,2–25,7** |
| E2 | novo (sem atuadores) | agressivo | 0,3 | 4/4 | 4/4 | sim (56,1 s) | não | 17 % (manete −0,47..1,35 entra na planta) | 0 % | 21,1° | 14,8° | 599,1–619,7 | 14,9–18,1 |
| F τ1 | Original | oval | **1,0** | 6/6 | 6/6 | sim (87 s) | não | 69 % | 0 % | 17,5° | 19,1° | 599,8–600,4 | 10,9–19,3 |
| F τ2 | Original | oval | **2,0** | 6/6 | 5/6 | não | **sim** após o WP5 | 92 % | 76 % | 899° | 90° | 20..609 | 3,5–27,8 |
| F τ3,5 | Original | oval | **3,5 (motor do XP9)** | 5/6 | 4/6 | não | **sim** após o WP4 (~50 s) | 92 % | 80 % | 1620° | 90° | −1..618 | 2,2–36,9 |

Distâncias mínimas (B): 12,8 / 32,5 / 26,3 / 19,5 / 31,3 / 0,0 m (A: 12,7 / 32,4 / 26,4 / 19,5 / 31,3).
Valores de referência do repo (03/09): originais 2/6, v3 6/6 (9,9/5,8/14,1/2,0/8,4/0 m).

## 5. Leitura

1. **A guiagem funciona com o ψ Hold novo — confirmado**, e não é artefato do modelo sem
   atuadores: no modelo com ±15°, rate limit, servo e motor (B) dá 6/6 com 0 % de saturação,
   δe +2,0..+2,7°, manete 0,24–0,42, h 600,0 m, φ ≤ 17,6°. O contra-teste (C/D) com o ψ antigo
   nos mesmos modelos reproduz o departure de 03/09 ⇒ a causa era mesmo o ψ Hold (ζ 0,09).
2. **Circuito agressivo**: fecha 4/4, mas com o **comando de manete nos batentes 88 % do tempo**,
   α até 22,7° (estol 18,5°; o modelo NL da Ana não estola), θ −22..+43°, V_T 10–26 m/s. Os
   holds longitudinais (Alt 28,6°/m, V_T 78,8 %/(m/s)) continuam os de atuador ideal. No gêmeo
   X-Plane isso deve estolar (o v3 fez 4/4 com α ≤ 11°).
3. **Motor do X-Plane 9 (τ 3,5 s)**: o oval faz WP1–WP4 e perde o controle (windup da manete,
   mecanismo dos adendos 10/11). Tolera até τ ≈ 1 s (já com 69 % de manete nos batentes).
   ⇒ o teste no gêmeo v2 com estes ganhos **provavelmente não fecha** sem também suavizar
   V_T Hold / Alt Hold (ou anti-windup, que muda a estrutura).
4. Para continuar trabalhando no modelo novo do Mirko com números críveis: recolocar a cadeia
   de atuadores dentro de `Planta` (entre `Sum_trim` e `Mux_u4act`) e a CI `xi_alt0` no
   `Integrator2`; ou usar `CL_NL_DH_GUIA_Original` para os números.

## 6. Como repetir

```matlab
cd C:\Users\kaue\Documents\Dissertacao_Mestrado\controle-pid-drone-hibrido\lqry_v3\LQRy_Guiagem
addpath(fullfile(pwd, 'verificacao_20260910'))
R = verifica_guiagem('mdl', 'CL_NL_DH_GUIA_Original', 'missao', 'oval', 'eng_tau', 0.3, 'tag', 'teste');
R = verifica_guiagem('mdl', 'CL_NL_DH_GUIA', 'psi_dir', fullfile(pwd, 'verificacao_20260910', 'ganhos_mirko_antigo'), 'tag', 'psi_antigo');
tabela_verificacao;                                    % tabela de todos os verif_*.mat
compara_entradas_controlador; dump_sistema('CL_NL_DH_GUIA/Planta');
```

## 7. X-Plane (gêmeo v2), 2026-09-10 11:08 — oval com o ψ Hold novo

Lançador `controle-pid-drone-hibrido\lqry_v3\XP_missao_lqry3.m` (controlador do Mirko intacto no
`modelo_XP_LQRY2_GUIA`, laço 100 Hz, reload do .acf com assinatura v2 conferida), ganhos de
`lqry_v3\LQRy_Guiagem\ganhos_mirko` via `XP3_ganhos_dir`, âncoras de trim v2 a 15 m/s (thr 0,42, δe +2,6°, pitch 9°).
Voo: `xplane\voos\XP_missao_20260910_110802_LQRYmirko_psiNovo_oval.{mat,_traj.png,_series.png,_compNL.png}`.

| | X-Plane v2 | NL (mesma missão, automático) |
|---|---|---|
| capturas sequenciais | **6/6** (10,6 / 26,2 / 39,6 / 55,6 / 69,0 / 81,7 s) | 6/6 |
| distâncias mínimas [m] | 4,1 / 32,0 / 30,5 / 27,5 / 30,6 / 23,8 | 12,8 / 32,5 / 26,3 / 19,5 / 31,3 / 0 |
| fim automático | 86,8 s | 86,6 s |
| h [m] | 599,7–600,4 (RMS 0,15) | 600,0 (RMS 0,01) |
| V_T [m/s] | 13,5–16,6 (RMS 0,72) | 14,8–15,0 |
| φ máx / α máx | 26,3° / 11,2° | 17,6° / 8,5° |
| δe / δa / δr [°] | −2,4..+6,6 / −2,0..+1,7 / −3,7..+4,2 | +2,0..+2,7 / ±0,4 / ±1,0 |
| comando de manete | **−0,92..2,28, fora de [0,1] 52 % do tempo** (ciclo-limite ~2,4 s) | 0,24–0,42 |

Sem estol nem departure. A previsão do NL com motor 3,5 s (departure após o WP4) **não** se confirmou:
no gêmeo o V_T Hold entra em ciclo-limite (bang-bang da manete, V_T ±1,5 m/s) mas não enrola.

### 7.1 Repetição sem vento (11:16 e 11:24)

Ventos zerados pelo Kaue nas 3 camadas (`sim/weather/wind_speed_kt[0..2]` = 0; turbulência 0,19 nas camadas 1 e 3).

| voo | reload | capturas sequenciais | fim | h [m] (RMS) | V_T [m/s] (RMS) | φ máx | α máx | manete cmd (fora [0,1]) |
|---|---|---|---|---|---|---|---|---|
| 11:08 `..._oval` | automático, assinatura OK, engate a t_xp 19,8 s | 6/6 (10,6/26,2/39,6/55,6/69,0/81,7 s) | 86,8 s | 599,7–600,4 (0,15) | 13,5–16,6 (0,72) | 26,3° | 11,2° | −0,92..2,28 (52 %) |
| 11:16 `..._semvento` | **não verificado**, avião 128 s na rampa com motor ligado | 3/6 (10,6/25,2/40,9 s), estol 36,5 s, departure 39,3 s | 162 s (teto) | até 35 s: 599,8–600,2 | até 35 s: 11,6–17,4 | 179° | 180° | 90 % já antes da queda |
| 11:19 `..._semvento_reload` | automático tolerante (chamada interrompida pelo Kaue, mas o MATLAB executou), engate a t_xp 16,5 s | **6/6** (10,4/25,0/38,1/54,4/68,7/81,8 s) | 86,8 s | 600 | 13,8–16,4 | — | — | — |
| 11:23 `..._semvento_reloadOK` (1.º) | laço de detecção (chamada interrompida, executou mesmo assim) engatou 2,3 s após o reset, avião ainda inicializando | 0/6, estol 1,1 s, departure 4,9 s | 162 s (teto) | 41–606 | 0–25 | — | — | artefato de engate, não é resultado |
| 11:24 `..._semvento_reloadOK` (2.º) | manual (Kaue), engate a t_xp 31,7 s | **6/6** (10,4/25,1/38,1/54,4/68,7/81,9 s) | 86,9 s | 599,7–600,4 (0,14) | 13,8–16,1 (0,66) | 24,1° | 10,9° | −0,56..1,50 (52 %) |

O voo das 11:16 é artefato do motor elétrico do XP9 (estoque interno esgota em ~150 s de motor ligado,
`PENDENCIA_MOTOR.md`): a V_T cai de 15 para 8 m/s a partir de ~40 s com a manete saturada em 1, e só então
vem o estol. Regra: reload verificado imediatamente antes do engate (o `xp_reload_acf` exige a janela do
X-Plane maximizada; com o Kaue presente o reload é manual e o voo só parte após o aviso dele).
Sem vento o ciclo-limite da manete fica menor (comando −0,56..1,50 contra −0,92..2,28), V_T ±1,2 m/s
com período ~2,4 s, mas continua presente: é o V_T Hold de atuador ideal com o motor lento.

### 7.2 Oval em subida, +10 m por WP (11:28, sem vento, reload do Kaue)

`XP3_WPs_frame` = oval da GUI com h = 610/620/630/640/650/660 m, engate a 600 m
(`xplane\voos\XP_missao_20260910_112804_LQRYmirko_psiNovo_oval_subida10m.*`, `_compNL.png`).

| | X-Plane v2 | NL (automático) |
|---|---|---|
| capturas sequenciais | **6/6** (10,5 / 25,2 / 38,3 / 54,7 / 69,1 / 82,3 s), fim 87,3 s | 6/6, fim 86,9 s |
| distâncias mínimas [m] | 13,5 / 30,9 / 24,2 / 18,6 / 28,8 / 28,6 | 12,4 / 33,1 / 28,3 / 19,0 / 31,9 / 0 |
| h na captura vs alvo | 610,0 / 620,1 / 630,1 / 640,2 / 649,9 / 660,0 (erro ≤ 0,2 m); cada degrau de 10 m em ~5 s | idem |
| V_T [m/s] | 12,5–17,4 (RMS 0,80) | 13,9–15,7 |
| θ máx / α máx | 29,5° / **17,2°** (em t = 0,9–1,1 s, transiente do engate com h_ref já em 610); 1,6 s com α > 15° | 24,4° / 13,8° |
| φ máx | 30,1° | 18,5° |
| δe [°] | −6,8..+11,9 | −0,5..+9,2 |
| comando de manete | −1,00..3,35, fora de [0,1] 51 % | 0,08–1,34, fora 5 % |

O Alt Hold segue os degraus no gêmeo (erro ≤ 0,2 m na captura), mas cada degrau custa θ de 25–30° e
α até 17° (estol 18,5°) por ~1 s: com degraus maiores que 10 m (o agressivo tem ±20 m) o estol é provável.
