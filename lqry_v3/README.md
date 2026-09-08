# LQRy v3 — o LQRy do Mirko com ganhos implementáveis (estrutura intacta)

**Resultado (2026-09-03):** com a **mesma arquitetura** do LQRy do Mirko
(5 holds, gain scheduling 3×3, referência só pelo integrador, `CL_NL_DH_18_jun_2026` /
`modelo_XP_LQRY2_GUIA`) e **só os ganhos** re-sintetizados, o controlador cumpre as
duas missões da GUI **no modelo NL da Ana e no X-Plane (gêmeo v1.2)**:

| Missão (GUI) | NL (Ana) | X-Plane (gêmeo v1.2, laço 100 Hz) |
|---|---|---|
| Circuito OVAL (6 WPs, 15 m/s, R 100) | **6/6**, passa a 2–14 m dos WPs, h 599,9–600,1 m, φ máx 13° | **6/6**, 0,3–11,6 m, h 599,1–601,7 m, φ máx 12°, α ≤ 9,9°, δe −3..+5°, manete 0,27–0,86 |
| Circuito AGRESSIVO (4 WPs, 90°, h 600/620, V 15/18, R 110) | **4/4**, 2–16 m, h 600–619 | **4/4**, 0,6–9,2 m, h 600–618, φ máx 15°, α ≤ 11°, δe −6..+5° |

Voos: `xplane/voos/XP_missao_20260903_004204_LQRY3_oval.*`,
`XP_missao_20260903_004442_LQRY3_agressivo.*` (+ `_compNL.png` com a mesma missão no NL) e a
repetição pelo caminho exato da GUI: `XP_missao_20260903_005906_GUI_LQRY3.*` (oval 6/6,
0,5–11,5 m) e `XP_missao_20260903_010030_GUI_LQRY3_agressivo.*` (4/4, 10–15 m).
NL: `NL_missao_20260903_00384{6,8}_LQRY3_*.mat`.

**Ganhos ORIGINAIS com a mesma estrutura e o mesmo harness** (cláusula "caso não dê com os
ganhos atuais"): NL oval **2/6** (departure após o WP1: φ 488°, H −1625 m, δe comandado
40 121°), NL agressivo **2/4** (departure na primeira curva de 90°), X-Plane oval **1/6**
(estol em 2,9 s, departure lateral em 4,3 s) — `NL_missao_20260903_0055*_LQRYorig_*` e
`XP_missao_20260903_005716_LQRYorig_oval_estruturaIntacta.*`.

## Por que o original aguentava o modelo do Sato e não aguenta o X-Plane

Os ganhos originais são "ótimos" para um atuador **ideal** (superfícies sem batente,
servo 24 rad/s, manete com lag de 0,1 s — é o rig do SIL e do HIL). Números do projeto
original (planta 5, 15 m/s; `lqry_v3_projeto` reproduz a análise):

| Malha | Ganho original (efetivo) | Consequência com atuador real |
|---|---|---|
| θ Hold | **8,4° de profundor por 1° de θ** (+ 10,9°/(°·s) integral) | profundor de ±15° satura com **1,8° de erro** |
| Alt Hold | **28,6° de θ_ref por metro** (+ α→θ_ref 5,5 rad/rad, θ→θ_ref −6,2) | 10° de θ_ref com 35 cm de erro de altitude |
| VT Hold | **78,8 % de manete por m/s** | manete satura com 1,3 m/s; com o motor do XP9 (τ 2,6–3,5 s) vira liga-desliga + windup |
| ψ Hold | polo −0,18 ± 1,98j (**ζ = 0,09**) já no projeto linear | oscilação lateral "espontânea" em voo reto; com 20–100 Hz de laço, departure |

Malha fechada do θ Hold no projeto: polos em ~25 rad/s (atuador ideal). Um laço de
alta banda com ganho enorme é **insensível a parâmetros da planta** (Sato: C_mq 4×,
C_mδe 3,3×, látero com sinais invertidos — e o controlador "nem vê", porque a
realimentação domina) mas **não tolera dinâmica não modelada nem saturação**: o motor
elétrico do XP9 (τ 3,5 s contra 0,1 s de projeto), o curso de ±15° e o efeito de potência
na arfagem (C_m,thr ≈ 0,25) fazem os comandos saturarem em décimos de segundo e os
integradores enrolarem. Robustez paramétrica ≠ robustez a atuador. O SIL prova isso com o
**mesmo integrador do Simulink** (`lqry_v3_sil_check`, Caso 4 do artigo, 15 m/s, 150 s):

| SIL (CL_NL_DH_18_jun_2026) | atuador ideal | atuador real (motor 3,5 s, ±15°, C_m,thr 0,25) |
|---|---|---|
| ganhos originais, planta Ana | converge (θ −6..16°, H 595–605) | **diverge** (θ −213..277°, H −767 m, δe 4·10⁷°, manete saturada 80 %) |
| ganhos v3, planta Ana | converge (θ 2..12°, H 595–605, δe 0,3..3,4°) | **converge** (θ 2..12°, H 595–605, δe −1..4,6°, manete 0,23–0,42, 0 % saturação) |
| ganhos v3, planta **Sato** | converge (θ −3..8°, H 595–605) | **converge** (θ −3..8°, H 596–605) |
| ganhos originais, planta Sato (15 m/s) | a S-function aborta em t≈0 (comando explode) | — |

Figuras: `sil/fig_robustez_aero_vs_atuador.png`, `sil/fig_convergentes_zoom.png`.
(A robustez ao Sato da apresentação de agosto foi com o LQRy **v1 de ganho fixo** a
12 m/s, `PID_DH/HIL_PID/Matlab/mirko_definitivo`; o v2 agendado de junho é mais agressivo.)

Sobre a hipótese "integrador do MATLAB ≠ X-Plane": o SIL acima usa exatamente o mesmo
integrador (ode4, 100 Hz) e reproduz a divergência quando a planta ganha o motor lento e
os batentes — a diferença de integrador não é a causa. O núcleo verdadeiro da hipótese é
outro: o θ Hold original fecha a malha em ~25 rad/s e o ψ Hold com ζ 0,09; nenhum laço
discreto de 20–100 Hz com 13–75 ms de latência sustenta isso (medido: 100 Hz dobra a
sobrevida e não resolve — adendo 8).

## O que é o v3

`lqry_v3_projeto.m` re-sintetiza os 10 conjuntos de ganhos (`GstateLong`, `GintLong`,
`GstateLong_Alt`, `GintLong_Alt`, `GstateLong_speed`, `GintLong_speed`, `GstateLat`,
`Gintlat`, `GstateLat_psi`, `Gintlat_psi`; cells 1×9 = as 9 plantas do `Dados_Trim.mat`)
com **as mesmas unidades e sinais** dos `Ganho_hold_*.mat` do Mirko — são *drop-in*.

- **Método:** LQ com ação integral (LQR na planta aumentada com o integrador do erro) em
  cada hold, sobre as matrizes A/B de `Dados_Trim.mat` (modelo da Ana) com os atuadores
  do próprio modelo (24/(s+24), manete 0,1 s). Onde a estrutura do Mirko **não**
  realimenta um estado (atuador, integrador da malha interna, motor), o ganho é refinado
  como **realimentação de saída estática** minimizando tr(P·X) com P da equação de
  Lyapunov — a formulação LQRy do artigo (eq. 11–16, Stevens & Lewis).
- **Pesos (regra de Bryson)** ancorados nos limites reais: θ 8°, q 30°/s, α 8°, δe 10°,
  H 5 m, θ_ref 6°, manete 25 % (12 m/s: 25 % com integral fraca), β 6°, p 60°/s, r 30°/s,
  φ 15°, δa 8°, δr 6°, ψ 25°, φ_ref 12°; constantes de tempo dos pesos integrais
  θ 1,2 s, H 2,5 s, V_T 8 s, φ 1 s, β 4 s, ψ 3 s. Na estrutura do Mirko a referência entra
  **só pelo integrador**, logo a resposta a um degrau de referência é de 2.ª ordem com
  ω_n = √(K·G_i) e ζ = G_s√K/(2√G_i): integradores lentos dão resposta arrastada (a curva
  de 90° "nunca chega"), integradores rápidos dão sobressinal — os Ti acima fecham ζ ≈ 0,8
  com ω_n ≈ 0,25–0,3 rad/s em ψ e H.
- **Verificação** (por planta, `ganhos/relatorio_projeto.txt`): polos de malha fechada com
  motor 0,1/0,3/3,5 s e atraso de 20 ms (Padé) — todas estáveis, ζ_min ≥ 0,32 (long, motor
  3,5 s) e ≥ 0,56 (lat); margens na **entrada da planta** (onde estão atraso e saturação):
  profundor PM ≥ 94°, margem de atraso ≥ 190 ms; aileron PM ≥ 90°, ≥ 200 ms; manete PM
  40–66° com motor de 3,5 s; resposta linear a ψ_ref = 90°: pico de φ_ref 15–18°, 90 % em
  12 s, OS ≈ 1 %; a H_ref +20 m: pico de θ_ref 11–17°, 90 % em 11–22 s.
- **Ganhos efetivos (planta 5, 15 m/s):** θ 1,63°/° (satura com 9° de erro), q 0,27°/(°/s),
  integral 1,04°/(°·s); Alt 3,4°/m de θ_ref; V_T 5,4 %/(m/s); φ→δa 0,58 rad/rad;
  ψ→φ_ref 0,98 rad/rad. (PID da dissertação, para escala: θ 0,92, Alt 2,9°/m, V_T 32 %/(m/s),
  φ 0,45, ψ 0,20.)

**O que NÃO mudou:** nenhum bloco do controlador. **Harness** (fora do controlador, como
já era nos lançadores do PID/LQRY2): condição inicial do integrador de H no engate
(`xi_H(0) = θ₀/G_i,Alt`, o "bumpless" que `XP_missao_lqry2` já fazia com
`XP_IC_int_alt`), neutralização do pulso de V_T de 10–20 s embutido no modelo de doublets,
fim de missão automático 5 s após o último WP, laço X-Plane a 100 Hz, β com o sinal do
XP9 corrigido, reload com verificação de assinatura do .acf.

Opções **experimentais, opt-in** (alteram a estrutura; ficaram desligadas por decisão do
Kaue): saturação de φ_ref (`*_phimax_deg`), anti-windup por clamping (`*_antiwindup`),
referência no termo proporcional + θ de trim no Alt Hold (`*_ref_prop`), clamp de θ_ref
(`*_clamp_deg`). Registro: com essas mudanças até os ganhos ORIGINAIS fecham o oval no NL
(6/6, θ até 22°, δe no batente) — evidência de que parte do problema original era a
resposta a referências grandes, mas fora do entregável.

## Casos do artigo (Tabela 11 e Fig. 38) com os dois conjuntos de ganhos

`lqry_v3_casos_artigo` roda os 8 casos da Tabela 11 (e o Caso 4 com inércia ×0,9 / ×1,1 /
×1,9) no SIL do Mirko com os ganhos v3; `lqry_v3_figuras_casos` sobrepõe às corridas
originais (`reproducao_SIL/caso*_nominal.mat`) e mede OS, t_r (10–90 %), t_s (2 %) e e_ss no
primeiro degrau de cada doublet. Saída em `sil/casos_artigo/` (`Caso_0k_orig_vs_v3.png`,
`Fig38_v3_caso4_inercia.png`, `metricas_casos_orig_vs_v3.{txt,csv,tex}`). Atuador ideal,
15 m/s, refs 5 (o rig do artigo), 150 s.

| Canal (1.º degrau) | Original: OS / t_r / t_s | v3: OS / t_r / t_s | Leitura |
|---|---|---|---|
| θ Hold (+5°, casos 1/3/5/7) | 3,2 % / 1,3 s / 3,8 s | 0,5 % / 3,2 s / 6,5 s | v3 2,4× mais lento, dentro da Tab. 5 (t_s < 10 s, OS < 10 %) |
| φ Hold (+5°, casos 1/2/5/6) | 3,9 % / 1,6 s / 4,6 s | 0 % / 2,4 s / 4,4 s | mesmo t_s, sem sobressinal |
| ψ Hold (+5°, casos 3/4/7/8) | **31 %** / 1,1 s / 13,8 s | 0,9 % / 9,4 s / 15,4 s | original viola o OS < 10 % da Tab. 5 e usa φ até −18°; v3 usa φ ≤ 1,8° |
| Altitude (+5 m, casos 2/4) | 0,9 % / 3,5 s / 5,7 s | 0 % / 9,8 s / >20 s (98 % em 20 s) | v3 ~3× mais lento, sem sobressinal; original pede θ −6..+16° e manete 0,00..0,64 |
| Airspeed (+3 m/s por 10 s, casos 1–4) | 0 % / 3,1 s / 5,8 s | não completa o pulso (59 % em 10 s) | preço da malha de V_T lenta (0,2 rad/s) que tolera o motor de 3,5 s |
| Inércia ×0,9 / ×1,1 / ×1,9 (Caso 4) | ψ OS 30,7–33,7 %, t_r 1,02–1,11 s | ψ OS 0,90–0,93 %, t_r 9,39–9,40 s | mesma insensibilidade à inércia nos dois |

Resumo: no rig ideal do artigo o v3 é 2–3× mais lento em θ, φ e H e não completa o pulso de
V_T de 10 s, em troca de zero sobressinal, um quarto do curso de superfície e manete
0,24–0,41 em vez de 0,00–0,64. A robustez à inércia é a mesma. É a leitura esperada: o
original gasta banda que só existe com atuador ideal; o v3 gasta a banda que o avião tem.

## Pacote para o Mirko (`pacote_mirko_guiagem_NL/`, zip ao lado)

Versão autocontida para enviar: `CL_NL_DH_GUIA.slx` = controlador dele intacto com a
guiagem LOS ligada **direto** nos holds (sem Steps de doublet, sem `Add_*_guia`, sem sondas,
sem saturação de θ_ref; `Switch1 → Sum1` e índices `{i}` restaurados), planta com os `.m`
originais dele + atuadores, `ganhos_mirko/` e `ganhos_v3/`, script único `guiagem_NL.m`
(`ganhos = 'mirko' | 'v3'`, oval por padrão) e `README.md` com o diff. Gerado por
`build_pacote_mirko.m`; validado reproduzindo exatamente o NL do repositório (v3 6/6 com
9,9/5,8/14,1/2,0/8,4/0 m; originais 2/6).

## Como usar

```matlab
addpath('C:\Users\kaue\Documents\Dissertacao_Mestrado\controle-pid-drone-hibrido\lqry_v3')
lqry_v3_projeto;                 % (re)gera ganhos/Ganho_hold_*.mat + relatorio_projeto.txt (4 s)
lqry_v3_sil_check;               % matriz SIL orig/v3 x Ana/Sato x ideal/real (~45 s cada; sil/*.mat)
NL_missao_lqry3;                 % oval no NL (30 s de CPU); NL3_WPs/NL3_R_accept/NL3_VT/NL3_eng_tau/NL3_Cm_thr/NL3_coef...
XP_auto_reload = true; XP_missao_lqry3;   % oval no X-Plane (XP3_WPs_frame/XP3_R_accept/XP3_VT/XP3_Ts_io/...)
XP_gui_waypoints                 % GUI: dropdown "Controlador" -> PID | LQRy v3
```

- `modelo_NL_LQRY_GUIA.slx` = `modelo_XP_LQRY2_GUIA` com a Planta_XP trocada pela
  `sfunction_DH` de `mirko_run` (aceita a planta híbrida `global HYB`) + cadeia de atuadores
  do `modelo_NL_DH_GUIA` (saturação ±`sat_surf_rad`, rate limit, servo, motor com
  `eng.tau`). Gerado por `lqry_v3_build_nl_model.m`.
- `lqry_v3_prepara_modelo.m`: edições em memória (o .slx não é salvo) — CI dos integradores,
  fim automático, taxa do laço; e as opções experimentais acima.
- Velocidade da missão define a planta: 12 → i=2, 15 → i=5, 18 → i=8 (sem interpolação,
  como nos lançadores anteriores). O LQRy engata na velocidade do WP1 (o PID engata a 12
  e acelera).
- Âncoras de engate do gêmeo a 15 m/s: manete 0,40, δe +5°, pitch 9° (`XP3_thr0/de0_deg/pitch0`).

## Arquivos

| | |
|---|---|
| `lqry_v3_projeto.m` | síntese + verificação + relatório (`ganhos/`) |
| `lqry_v3_sil_check.m` | matriz SIL no modelo do Mirko (`sil/`) |
| `lqry_v3_build_nl_model.m`, `modelo_NL_LQRY_GUIA.slx` | gêmeo NL do modelo X-Plane do LQRy |
| `lqry_v3_prepara_modelo.m` | harness em memória (e opções experimentais) |
| `NL_missao_lqry3.m`, `XP_missao_lqry3.m` | missões por waypoints (formato de voo = `XP_missao`) |
| `ganhos/Ganho_hold_*.mat`, `ganhos/LQRY_v3.mat`, `ganhos/relatorio_projeto.txt` | ganhos drop-in + análise |
| `sil/*.mat`, `sil/fig_*.png` | corridas e figuras do SIL |
