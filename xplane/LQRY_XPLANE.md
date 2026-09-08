# LQRY no X-Plane — experimento e resultado (2026-08-30)

## ADENDO 14 (2026-09-03) — (ok) LQRy v3: ganhos re-sintetizados, estrutura intacta — OVAL 6/6 e AGRESSIVO 4/4 no NL **e** no X-Plane

Pedido do Kaue: o LQRy tem de cumprir as missões da GUI nos dois mundos, **mexendo só
nos ganhos** (estrutura do Mirko intocada). Feito em `lqry_v3/` (README lá tem tudo):

- **Diagnóstico que faltava (números):** o θ Hold original entrega **8,4° de profundor
  por grau de θ** (integral 10,9°/(°·s)) → ±15° saturam com 1,8° de erro; o Alt Hold
  pede **28,6° de θ_ref por metro**; o VT Hold **78,8 %/(m/s)**; o ψ Hold já nasce com
  ζ = 0,09 (polo −0,18 ± 1,98j) no projeto linear. Malha do θ Hold fecha em ~25 rad/s
  com atuador ideal. Isso responde à pergunta do Sato: laço de ganho enorme é insensível
  a parâmetros aerodinâmicos (por isso aguentou o Sato no SIL com atuador ideal) e
  intolerante a dinâmica não modelada/saturação (motor τ 3,5 s, ±15°, C_m,thr) — o SIL
  com o MESMO integrador reproduz: original+Ana+atuador real diverge (H −767 m, manete
  saturada 80 %); v3 converge em Ana e em Sato, ideal e real. Hipótese do amigo
  ("integrador do MATLAB ≠ X-Plane") descartada pelo mesmo experimento; o que sobra dela
  é a banda irrealizável (25 rad/s / ζ 0,09) num laço de 20–100 Hz.
- **v3:** LQ com ação integral + refino por realimentação de saída (Lyapunov, eq. 11–16
  do artigo), pesos de Bryson nos limites reais; ganhos efetivos (15 m/s): θ 1,63°/°,
  Alt 3,4°/m, V_T 5,4 %/(m/s), φ 0,58, ψ 0,98 rad/rad; margens na entrada da planta com
  motor 3,5 s: PM δe 104°/193 ms, δa 102°/236 ms, manete 40°. Na estrutura do Mirko a
  referência entra só pelo integrador ⇒ resposta de 2.ª ordem (ω_n = √(K·G_i)); os pesos
  integrais foram escolhidos p/ ζ ≈ 0,8 e ω_n ≈ 0,25–0,3 rad/s em ψ e H (φ_ref de pico
  15–18° numa curva de 90°, sem limitador).
- **Harness (não é estrutura):** CI do integrador de H no engate (θ₀/G_i,Alt — o bumpless
  que o XP_missao_lqry2 já fazia), pulso de V_T dos doublets neutralizado, fim automático,
  100 Hz, β corrigido, reload verificado. Sem anti-windup, sem saturação de φ_ref, sem
  clamp de θ_ref, sem alpha-protection.
- **Resultados:** NL oval 6/6 (2–14 m dos WPs, h ±0,1 m), NL agressivo 4/4, NL oval com
  planta "X-Plane" (motor 3,5 s + C_m,thr) 6/6; **X-Plane gêmeo v1.2 oval 6/6
  (0,3–11,6 m, h 599–602, α ≤ 9,9°, φ ≤ 12°, manete 0,27–0,86, `XP_missao_20260903_004204`)
  e agressivo 4/4 (0,6–9,2 m, h 600–618, α ≤ 11°, `XP_missao_20260903_004442`)**, com
  `_compNL.png` sobrepondo o NL. GUI ganhou o dropdown "Controlador" (PID | LQRy v3).
- Experimento registrado e DESLIGADO (mudava a estrutura): erro no termo proporcional +
  θ_e feed-forward + saturação de φ_ref + anti-windup — com isso até os ganhos originais
  fecham o oval no NL (6/6, θ 22°, δe no batente). Fica como nota.
- **Cláusula "com os ganhos atuais" (mesma estrutura, mesmo harness):** NL oval 2/6
  (departure após o WP1), NL agressivo 2/4, X-Plane oval 1/6 (estol 2,9 s, departure 4,3 s)
  — `voos/*_LQRYorig_*_estruturaIntacta.*`. Repetição pelo caminho exato da GUI (dropdown
  LQRy v3, botões OVAL/AGRESSIVO, XP3_VT = mediana das velocidades → 15): **oval 6/6
  (`_005906_GUI_LQRY3`), agressivo 4/4 (`_010030_GUI_LQRY3_agressivo`)**.

## ADENDO 13 (2026-09-02, 14:00–14:45) — O RELOAD AUTOMÁTICO ESTAVA CARREGANDO O .acf ORIGINAL. Invalida os voos X-Plane deste dia (e provavelmente os DBL de 09-01)

**Como foi descoberto.** O Kaue pediu para confirmar o ajuste dos raios de
giração no Plane Maker (v1.2: pitch 1,47 / yaw 1,47 / roll 0,92 ft), porque os
drefs `acf_J*_unitmass` liam 0,053/0,080/0,029 m² e m 3,175 kg (ADENDO 10, nota).
Sequência:
1. Diff binário v1.1→v1.2: só 6 bytes — os três raios (1,47/0,86/0,53 →
   1,47/1,47/0,92 ft, offsets 136361/365/369) + 1 byte @147341 (1→0). **O ajuste
   foi salvo certo.** (Religar o byte @147341 vira OUTRA aeronave — não é a
   caixa dos raios; descartado.)
2. Ident de arfagem (`XP_ident_theta` + `XP_fit_ident_wn`, mesma receita de
   08-30) com o "v1.2" carregado: **ωn 10,3 rad/s** (v1.1 em 08-30: 6,1–6,3).
   Com o v1.1 copiado para o ativo, hoje: 10,2–10,8 — igual. Logo o que estava
   voando NÃO era o arquivo ativo.
3. Com a pasta reduzida a 1 arquivo, o clique do reload caiu fora da lista
   (X-Plane travou no diálogo; ESC via Robot destravou). Com 7 arquivos TODOS
   iguais ao v1.2: **J 0,2008/0,2008/0,0786 m² = (1,47/1,47/0,92 ft)² exatos,
   m 2,223 kg, cursos 25°, TRQ 0,43, ωn 6,3/6,1** — o gêmeo v1.2 de verdade.

**Diagnóstico.** `xp_reload_acf` clica numa LINHA FIXA do diálogo Open Aircraft
(coords relativas ao centro da janela). Com 7 `DH-Lon-REV-03*.acf` na pasta
(ativo + 6 backups), a linha clicada era a 2ª = `_backup_20260819.acf` =
**ORIGINAL** (assinatura: TRQ 0,35, cursos 15°, m 3,175 kg, raios automáticos,
α_trim baixo). A verificação antiga (t_xp resetou, AGL<3, TRQ>0,05) não
distingue aeronaves — a memória já avisava ("clique pode abrir QUALQUER linha
DH-Lon-REV-03*"), mas a pasta foi restaurada com backups distintos depois da
campanha A/B de 09-01 e ninguém re-verificou.

**O que fica INVÁLIDO como "gêmeo"** (era o ORIGINAL): todos os voos e sondas
X-Plane de 2026-09-02 até 14:29 — ADENDO 8 (100 Hz), 10 (voos 09-02), 11
(aquecimento, anti-windup, motor τ 3,5 s, β, cursos), 12 (FULLFIX, efeito de
potência, autoridade de profundor, trim "do gêmeo"). **Provavelmente também os
DBL de 09-01 17:38–18:13** (mesma pasta, mesmo clique, X-Plane não reiniciado
desde 08-31): a "paridade H Hold/φ Hold" do Caso 2 5°/5 m (ADENDO 8, figura
`SILxXP_caso2_V15_prot_5deg.png`) — no gêmeo v1.2 VERDADEIRO o mesmo voo a
20 Hz faz departure em **16,6 s** (`_144157`), a 100 Hz em 18,7 s (`_143927`).
**O que fica VÁLIDO**: tudo que é SIL (ADENDOS 4, 6, 7, 9; a planta híbrida e
as reproduções dos ADENDOS 10–12 são do SIL); o sinal de β do dref (convenção
do XP9, re-confirmado no gêmeo: leme +0,5 → r +12°/s, Δψ +8°, β_XP +6,7);
o bug da guiagem (ψ_ref no engate, estrutural); o hardcode `Ts = 0.05` dos
charts; o `i_lat`. Os ADENDOS 1–5 (08-30) usavam reload MANUAL do Kaue.

**Re-medições no gêmeo v1.2 (14:30+, assinatura verificada em cada reload):**
| Medida | Gêmeo v1.2 | (Original, manhã) |
|---|---|---|
| Motor: RPM τ63 / t95 | **2,6 s / 8,1 s** (VT 14,7→28 com +0,4) | 3,5 / 5,3 s |
| Efeito de potência: δe +2,2° fixo, 3 s, θ0 8° → θ | **−7° (thr 0) / +1,8° (0,34) / +29° (0,70)** | −8 / −7 / +19 |
| Profundor +15°: Δθ em 1 s | +20° / +32° / +20° | +24 / +28 / +23 |
| β do dref | invertido (idem) | invertido |
| Trim (aquecimento 40 s, janelas) | thr 0,44, δe +1,2°, θ 6,6°, VT 14,7 (não convergiu) | thr 0,45, δe +8,3°, θ 9,7° |
| ψ Hold puro 5°/5 m, 100 Hz | **departure 4,5 s**, estol 3,1 s | 9,9–11,2 s |
| φ Hold + prot 5°/5 m, 100 Hz | **departure 18,7 s** | (91 s c/ 10°) |
| φ Hold + prot 5°/5 m, 20 Hz | **departure 16,6 s** | "completou" (09-01, aeronave incerta) |

**Leitura.** No gêmeo verdadeiro o LQRY é PIOR que no original em todas as
malhas: o efeito de potência é ~1,5× maior (θ +29° em 3 s com thr 0,70), o
motor é o de 0,80 hp, e o α de trim é 8° (margem ao estol 18,5° de 10°). A
conclusão dos ADENDOS 10–12 (VT hold × motor × efeito de potência → windup →
mush/estol → departure; anti-windup não basta; reprojeto necessário) fica
MAIS forte, não mais fraca — mas a alegação "H Hold e φ Hold transportam"
CAI até ser refeita no gêmeo (hoje, a 20 e 100 Hz, não transportou).

**Correções permanentes de harness (feitas):**
- Pasta do X-Plane: ativo + 6 clones IDÊNTICOS (`DH-Lon-REV-03_zz_clone*_do_v1_2_NAO_E_BACKUP.acf`,
  md5 5b808d0b) + `LEIA-ME_clones.txt`; backups reais movidos para
  `Dissertacao_Mestrado/acf_backups_20260902/` (hashes originais preservados).
- `XP_sync_acf_clones.m`: re-copia o ativo para os clones após editar no PM.
- `xp_reload_acf`: **verifica a assinatura da aeronave** após cada reload
  (J_unitmass, massa, curso do profundor vs `global XP_ACF_SIG`, default =
  gêmeo v1.2) e ABORTA se for outra. `XP_ACF_SIG = 'off'` desliga.
- `XP_fit_ident_wn.m`: fit 2ª ordem q/δe (receita de 08-30) para checar ωn.

**Pendente (decisão do Kaue):** auditar quais resultados de 09-01 (DBL, VR
pós-restauração) foram no original; refazer no gêmeo a figura de paridade
do Caso 2; reescrever os ADENDOS 8/11/12 com a etiqueta "ORIGINAL".

## ADENDO 12 (2026-09-02, fim de tarde) — Guiagem neutralizada, trim do gêmeo medido, e o mecanismo que falta no modelo: EFEITO DE POTÊNCIA na arfagem × motor lento

### Feito
- **Bug 2 corrigido** (`XP_dbl_guia_psi_off`, default 1): `Goto_PsiRefGuia` recebe
  Constante 0 em memória (o chart fica terminado). Com isso φ_ref no engate
  passou de −18°/−58° (0,2/1 s) para −5..−8°, e o lateral ficou quieto nos
  primeiros 5 s (φ ±5°). O bug 2 era real e está fora do caminho.
- **Trim do gêmeo a 15 m/s medido e guardado** (`voos/XP_trim_gemeo_V15_20260902.mat`,
  aquecimento por janelas, 32 s, convergiu): **thr 0,45, δe +8,3°, θ 9,7°,
  VT 14,9, ḣ 0**. Ressalva: o motor esgota energia (~130 s) e o mesmo thr
  0,45 com motor fresco acelera a 19 m/s subindo 3 m/s — o trim de manete
  "verdadeiro" com motor fresco é ~0,33–0,35 (= Ana 0,337!); o δe +8,3° vale
  para thr 0,45 (efeito de potência, abaixo).
- **Voo FULLFIX** (`_133936`: β corrigido + guiagem off + âncoras do gêmeo +
  aquecimento simples 10 s + anti-windup + 100 Hz): **departure 11,2 s**.
  Longitudinal: θ_ref = +27° já em t=0 (ΔVT +4 m/s, Δα −3,5°, Δθ +3° pelas
  matrizes GstateLong_Alt = [−0,04 5,5 −0,2 −6,2 −0,5]) → throttle 0 (VT hold
  −79 %/(m/s)) → θ cai 11→0° com profundor cravado +15° por 7 s → mush → estol
  29,9 s. Anti-windup não ajuda: o termo é PROPORCIONAL (GstateLong θ = **−482
  rad/rad**: 2° de erro de θ = 15° de profundor).

### Duas medições diretas no gêmeo (malha aberta, 15 m/s, 600 m)
1. **Autoridade do profundor NÃO some com manete zero**: degrau de +8→+15° dá
   Δθ +24° em 1 s com thr 0, +28° com thr 0,34, +23° com thr 0,70. (A hipótese
   "sopro ×3" do VR não explica a falha; kw0 = 0,33 no SIL também não.)
2. **EFEITO DE POTÊNCIA na arfagem** (o que faltava): com δe FIXO em +8° e
   θ0 = 9°, em 3 s o gêmeo vai a **θ −8° com thr 0,00, θ −7° com thr 0,34 e
   θ +19° com thr 0,70**. A manete muda o momento de arfagem (sopro da hélice
   na cauda / linha de tração); Δthr 0,36 ≈ 5° de profundor ≈ Cm_thr ≈ 0,25 por
   unidade de manete. O modelo da Ana NÃO tem esse termo (F só em x-body).

### Reprodução no SIL (planta híbrida, `HYB.Cm_thr`, patch em `modelo_DH.m`)
| Configuração (i=5, Caso 4, refs do artigo) | Resultado |
|---|---|
| Cm_thr 0,25 SÓ, motor 0,1 s (modelo), sem AW | converge (thr sat 1 %) |
| Cm_thr 0,25 + motor 3,5 s + batente + Cmδe 0,75 + sopro + **AW**, engate exato | thr sat **80 %**, H 600→185 |
| idem, engate +3°/+2 m/s | thr sat 87 %, H → 161 |
| Cm_thr 0,50 + tudo + AW | DIVERGE (H −374) |
| Cm_thr 0,25 + tudo, SEM AW | DIVERGE |
(`reproducao_SIL/efeito_potencia_c5.mat`; sem Cm_thr, tudo + AW convergia — `final_engate_tau_c5.mat`, `sopro_helice_c5.mat`.)

**Leitura.** O par **efeito de potência × motor lento** fecha o mecanismo: o
VT hold (−79 %/(m/s), integral) bate a manete; a manete, além do empuxo com
3,5 s de atraso, injeta momento de arfagem imediato; o H hold (θ_ref ← 5,5·α
− 6,2·θ) e o θ hold (−482·θ) respondem com o profundor no batente; o
sistema entra em oscilação throttle↔arfagem (0,15 Hz, ADENDO 10) e afunda
ou diverge. Anti-windup só remove o enrolamento dos integradores; a
instabilidade é dos termos proporcionais contra uma planta com dois
acoplamentos que a planta de projeto não tem. É por isso que nenhum
"remédio de harness" fecha a paridade: taxa (ADENDO 8), atraso (9),
aquecimento, β, guiagem, âncoras e anti-windup (11–12) foram todos
necessários ou corretos, e nenhum suficiente.

### Conclusão prática para a dissertação
- Paridade LQRY×PID a 15 m/s **não é obtenível com o controlador do Mirko
  intacto** sobre uma planta com motor elétrico real (τ 3,5 s) e efeito de
  potência na arfagem. O PID sobrevive porque seus ganhos são ~1/50 dos do
  LQRY em θ e a malha de VT é lenta.
- O que dá para afirmar com dados: (i) SIL reproduz Fig. 38 (ADENDO 4);
  (ii) no gêmeo, H Hold e φ Hold transportam (ADENDO 8, Caso 2 5°/5 m);
  (iii) o ψ Hold/VT Hold não transportam por dois acoplamentos de planta
  medidos e reproduzidos no SIL (este adendo + ADENDO 10); (iv) o custo de
  implantação mínimo NÃO é anti-windup — é reprojeto com a dinâmica do
  motor e o efeito de potência na planta de projeto (fora do escopo desta
  dissertação, e é uma conclusão, não uma falha).
- Para a banca: o gêmeo foi validado contra voo real (VALIDACAO_VOO_REAL);
  os dois acoplamentos existem no drone real (motor elétrico + hélice à
  frente da cauda), logo o resultado transfere.

### Estado final do harness (defaults)
`XP_dbl_Ts_io` 0,01 | `XP_dbl_guia_psi_off` 1 | `XP_beta_sign` −1 |
`XP_dbl_warmup` 0 | `XP_dbl_antiwindup` 0 | `XP_dbl_VT_Throttle` 1 |
âncoras: Ana (i≠2) — para o gêmeo a 15 m/s usar `XP_dbl_thr0=0.34,
XP_dbl_de0_deg=8.3, XP_dbl_pitch0=9.7` (δe a validar com thr 0,34).
Planta híbrida em `mirko_run/` (`global HYB`: k_Iroll k_Iyaw k_Clda k_Cnb
k_Clb k_Clp k_Cnr k_Cmde de_lim de_bias k_F kw0 Cm_thr dX0; vazio = Ana
intacta); `lqry_delay_run.m`, `lqry_trace_gains.m`, `XP_sonda_motor_tau.m`.

## ADENDO 11 (2026-09-02, tarde) — Passos 1 e 2 executados: o que fechou, o que não fechou, e o que apareceu no caminho

Plano acordado com o Kaue: (1) engatar com o motor em regime; (2) anti-windup
nos integradores; (3) refazer as figuras de paridade. Executados (1) e (2)
em 8 voos (`voos/XP_dbl_20260902_12*..13*`). **Paridade do ψ Hold a 15 m/s
ainda NÃO obtida** — mas cinco fatos novos, dois deles bugs de harness.

### Medições novas
- **Motor do XP9 em voo** (`XP_sonda_motor_tau.m`, `voos/XP_sonda_motor_tau_*.mat`):
  degrau de manete +0,4 → **RPM τ63 = 3,55 s, t95 = 5,3 s** (TRQ instantâneo; o
  dref POINT_thrust cai com a velocidade e não serve). O atuador de manete do
  modelo do Mirko (`throttle 1`) é 1/(0,1s+1). **No SIL, τ_motor = 2 s já
  satura o throttle 66–72 %; τ = 3 s reproduz a divergência completa
  (δe cmd 1,6·10⁸, H −1594)** — com engate no trim exato, sem batente, sem
  lateral. Este é o experimento discriminante da causa raiz do ADENDO 10.
  (`reproducao_SIL/motor_lag_c5.mat`.)
- **Anti-windup por clamping** (limite de saída dos 3 integradores ao alcance
  do atuador: VT ±100 %, θ ±25°, H ±20° de θ_ref) **resgata o SIL até
  τ_motor = 5 s** (thr sat 0 %, δe máx 3,7°; custo: H afunda a 559 m no doublet
  de VT e recupera devagar — limites a afinar). `reproducao_SIL/antiwindup_motor_c5.mat`.
- **Trim do gêmeo a 15 m/s ≠ trim da Ana**: aquecimentos mediram δe +4,3..+9,9°
  (Ana +2,22°), manete ~0,30–0,32 com motor em regime (Ana 0,337; 0,45 acelera
  a 18,8 m/s subindo 1,7 m/s), θ ~9°. Engatar nas âncoras da Ana dá pitch-down
  de −25°/s no 1º segundo (todos os voos anteriores). Lançador ganhou
  `XP_dbl_pitch0` (além de thr0/de0_deg).
- **Energia do motor limita o aquecimento**: 120 s de trim por janelas
  terminaram com manete 0,93 para h constante (motor esgotando, ~130 s
  conhecidos). Aquecimento útil = curto (≤12 s, só o spool).

### BUG 1 — sinal de β do X-Plane (CORRIGIDO em `xp_read_dh`)
Sonda de leme em voo (2×, sinais opostos): leme XPC +0,5 → r +3,4°/s, Δψ +7°
(nariz p/ direita), **β_XP +4,7°**; leme −0,5 → r −0,8, Δψ −2,3, β_XP −4,6.
No modelo (β = asin(v/V)) nariz p/ direita ⇒ v<0 ⇒ β<0. **O dref `beta` do
XP9 é o negativo da convenção do modelo.** O PID nunca usou β (só log); o
ψ Hold do LQRY realimenta β com o maior ganho da linha (−4,7) — sinal errado
= realimentação positiva de derrapagem. Correção: `global XP_beta_sign`
(−1 = corrigido, default; +1 = antigo). Todos os voos LQRY anteriores a
2026-09-02 13:18 rodaram com β invertido. **Sozinha, a correção NÃO salvou
o ψ Hold** (departure 9,9–11,2 s vs 6–23 s antes) — havia mais.

### BUG 2 — ψ_ref da guiagem no engate (NÃO corrigido; identificado)
Sondas `probe_phiref/thetaref` (já existiam no modelo, ficam em `out`): no
engate, φ_ref = −1,8° em t=0 mas **−18° em 0,2 s, −40° em 0,5 s, −58° em 1 s**
com φ real ≈ +2,6° — é a saída do ψ Hold (Kψ −5,05 + integral −5,45) para um
erro de ψ de ~4° presente desde t=0. Fonte: `Add_psi_guia` soma `PsiRefGuia`
(LOS da guiagem para WPs a 10 km na proa PRÉ-voo, 187–189°) à referência; após
teleporte+aquecimento a proa real é ~190–193° ⇒ ψ_ref_rel ≈ −4° no engate,
que o SIL nunca vê (ψ_ref = 0 até 90 s). Com o ψ Hold pedindo 58° de bank em
1 s, o φ loop (ganho proporcional só 0,12 rad/rad — a integral faz o trabalho)
atrasa, a integral de ψ enrola, φ_ref oscila −58/+95/−170° e o lateral
diverge em 5–7 s. **Correção pendente**: WPs na proa REAL do engate (ou
PsiRefGuia = 0 no modo doublets).

### Voos do dia (ψ Hold puro, 5°/5 m, 15 m/s, 100 Hz) — departure |φ|>60°
| Config | dep. [s] | Observação |
|---|---|---|
| referência (β invertido, âncoras Ana) _103818/_104324 (10°) | 22,7 / 11,0 | ADENDO 8 |
| + aquecimento 30 s (manete←V) _124308 | 3,5 | trim errado: thr→0, engate a 534 m afundando (descartado) |
| + aquecimento 45 s (manete←ḣ contínuo) _124656 | 1,2 | oscilando (motor τ 3,5 s), φ_ref −58° em 1 s |
| + anti-windup, engate imediato _130803 | 12,6 | pitch-down −25°/s no engate (âncoras Ana) |
| + aquecimento 120 s por janelas + AW _131319 | 1,0 | motor esgotou (thr 0,93) |
| β corrigido + AW _131850 | 11,2 | |
| β corrigido, sem AW _132051 | 9,9 | |
| β corrigido + âncoras gêmeo (0,45/8,5°/9°) + AW _132409 | 7,2 | engate longitudinal limpo por 1 s; ψ_ref −4° → φ_ref −58° |
| idem + aquecimento simples 12 s _132939 | 5,7 | thr 0,45 acelerou a 18,8 m/s; φ_ref −18° em 0,2 s |

### Estado do harness ao fim do dia
`XP_doublets_lqry2`: flags `XP_dbl_Ts_io` (0,01), `XP_dbl_VT_Throttle` (1),
`XP_dbl_warmup` (0 = OFF), `XP_dbl_warm_simple` (1), `XP_dbl_antiwindup`
(0 = OFF), `XP_dbl_pitch0`; `xp_read_dh`: aquecimento (dois modos) +
`XP_TRIM_DELTA` somado em `xp_send_dh` + `XP_beta_sign`. Defaults preservam
o comportamento antigo, EXCETO o sinal de β (corrigido por padrão — decisão
técnica, não de escopo).

### Próximo passo (o que falta para tentar a paridade de verdade)
1. Neutralizar `PsiRefGuia` no modo doublets (ou WPs pela proa real após o
   aquecimento) — sem isso o ψ Hold nasce com −4° de erro e 58° de φ_ref.
2. Âncoras do gêmeo a 15 m/s com motor em regime: thr ≈ 0,30, δe ≈ +6°,
   θ ≈ 9° (medir uma vez com aquecimento por janelas de 40 s e GUARDAR).
3. Aquecimento simples 10 s nessas âncoras + anti-windup + β corrigido, 100 Hz.
4. Se ainda divergir: o resíduo é o motor (τ 3,5 s vs 0,1 s de projeto) — e aí
   a resposta honesta para a banca é a do ADENDO 10, com o SIL-τ como prova.

## ADENDO 10 (2026-09-02) — CAUSA RAIZ a 15 m/s: armadilha de windup por saturação do throttle. Reproduzida no SIL

**Pergunta do Kaue:** "não dá para ter paridade LQRY SIL×X-Plane a 15 m/s?"
Sequência de testes de meia hora no SIL do Mirko (planta da Ana), todos a
15 m/s (i=5), refs do artigo, Caso 4 (ψ Hold), via planta "híbrida"
(`global HYB` nos `modelo_DH.m`/`obs_rigidbody_DH.m`/`sfunction_DH.m` de
`mirko_run/` — vazio = modelo intacto; campos k_Iroll/k_Iyaw/k_Clda/k_Cnb/
k_Clb/k_Clp/k_Cnr/k_Cmde/de_lim/de_bias/k_F/dX0). Dados em
`reproducao_SIL/{hibrida_lateral,ic_engate,zeta_dr_sweep,batente_cmde,
bias_trim,engate_empuxo}_c5.mat`.

**O que NÃO reproduz a divergência do X-Plane (SIL converge em todos):**

| Perturbação testada no SIL | Resultado |
|---|---|
| Atraso no comando até 75 ms (ADENDO 9) | converge |
| Lateral do gêmeo: I_roll ×0,64, Clδa ×0,8, Cnβ ×0,53 (f_DR 0,96→0,76 Hz), combinados | converge |
| Amortecimento do dutch roll ζ 0,45 → 0,08 (Cnr até −0,25×) | converge; só ζ 0,08 + tudo junto diverge |
| Erro de engate φ 5°/ψ 2° (+50 ms, + híbrida); φ 20°/ψ 10° | converge (só 20/10 + híbrida + 50 ms diverge) |
| Batente ±15° nas superfícies; Cm_δe ×0,75 (gêmeo); ambos; + engate + 50 ms | converge, δe máx 7° |
| Desalinhamento de trim do profundor até −8° | converge, integrador acomoda (δe até 15°) |
| Engate com VT −3 m/s e α +3° | converge |
| Empuxo ×0,5 (spool/lei de hélice) | converge (throttle satura 8 % do tempo) |
| Clδa ×0,5 | DIVERGE |
| **Empuxo ×0,3 → throttle saturado 99 % do tempo** | **DIVERGE com a MESMA assinatura do X-Plane** |

**A assinatura no X-Plane (todos os 14 voos DBL de 09-01/09-02):** o
comando de profundor do controlador ultrapassa o batente de 15° já em
t ≈ 0,1–0,8 s e cresce LINEARMENTE (~1000°/s): +1042° em 2 s, +3654° em
20 s (20 Hz), +18136° em 20 s (100 Hz); fica acima de 15° em 64–96 % do
tempo nos voos de ψ Hold. O comando de throttle idem (−168..+258 = ±25000 %).
O `xp_send_dh` recorta em ±15°/[0,1] — a aeronave recebe a superfície
CRAVADA no batente e o controlador não sabe. Nos voos de Caso 2 a 20 Hz
que "completavam", a saturação nos primeiros 20 s era 0–2 % — por isso
completavam. Frequência dominante de φ, r, VT e throttle nos voos de
Caso 2 = 0,15 Hz, TODOS iguais: a oscilação lateral está travada no
ciclo do throttle, não é dutch roll (ADENDO 8 corrigido: o "ciclo-limite
do motor" é a mesma armadilha em regime oscilatório).

**Mecanismo (única cadeia consistente com tudo):** o hold de VT
(G_s = −78,8 %/(m/s), integral, SEM anti-windup) pede ao motor do XP9 mais
do que ele entrega no engate (teleporte derruba o RPM, spool ~20 s;
lei de hélice; energia do elétrico caindo) → throttle satura → o integrador
de VT enrola → VT/H fogem → o hold de H pede θ_ref (α→θ_ref = +4,5 rad/rad)
→ profundor satura em 15° (o gêmeo tem 25–30 % menos Cm_δe que a Ana) →
integradores de θ e H enrolam → profundor cravado → mush/estol → o ψ Hold
atua na asa estolada → departure lateral. **O lateral é a VÍTIMA, não a
causa** (por isso nenhuma perturbação lateral no SIL reproduz; por isso
o φ Hold com α-protection sobrevivia: a proteção corta o profundor
cravado). Reproduzido no SIL só com empuxo ×0,3: throttle 99 % saturado,
δe cmd → 1,4·10⁸, θ −87..296°, H −1486. Figura
`reproducao_SIL/windup_empuxo_SILxXP_c5.png` (SIL 100/50/30 % de empuxo ×
X-Plane, mesma escala; δe em log).

**Resposta à pergunta.** Paridade a 15 m/s exige que o hold de VT não
entre na armadilha. Três caminhos, em ordem de custo: (1) **anti-windup
nos 3 integradores** (θ, H, VT) — mudança mínima e canônica de
implementação (back-calculation/clamping), NÃO altera os ganhos do Mirko,
e é a única alavanca ainda não testada (já apontada nos ADENDOS 3–5 como
"decisão de escopo"); (2) engate com o motor JÁ em regime (esperar o
spool antes de engatar; hoje engata ~1 s após o teleporte) — barato, testa
a parte "spool" do mecanismo sem tocar no controlador; (3) reprojeto dos
ganhos com limites de atuador. Recomendação: fazer (2) e (1), nesta
ordem — (2) diz quanto é bancada, (1) é o "custo de implantação" que a
dissertação pode declarar sem descaracterizar o controlador.

**Throttle fixo (Casos 5–8 da Tab. 11) no X-Plane a 100 Hz** (voos
`_120927` ψ Hold e `_121127` φ Hold, 5°/5 m): não isola nada — com o
throttle no trim da Ana (0,337) o gêmeo mush a 11,4 m/s (α 17–18°), e o
profundor já satura em 0,1 s pelo mesmo mecanismo (H hold sem VT);
ψ Hold: estol 6,5 s → departure 6,9 s; φ Hold: completa mas h 600→450.
Lançador ganhou `XP_dbl_VT_Throttle` (default 1).

**Nota de harness (2026-09-02):** drefs `acf_J{xx,yy,zz}_unitmass` leram
0,053/0,080/0,029 m² e `m_total` 3,175 kg com o `DH-Lon-REV-03.acf` =
v1.2 (md5 5b808d0b, 1ª linha do diálogo — pasta conferida), inclusive
logo após reload. Diverge do registrado no VR (0,201/0,201/0,079). Não
afeta as comparações de hoje (mesmo .acf em todos os voos de 09-01/09-02),
mas a identidade "v1.2 = raios 1,47/1,47/0,92 ft" precisa ser reconferida
no Plane Maker antes de citar esses números.

## ADENDO 9 (2026-09-02) — Margem de atraso a 15 m/s: 75–100 ms. CORRIGE a generalização do ADENDO 6

**Contexto.** O ADENDO 6 mediu a tolerância a atraso puro no comando SÓ a
12 m/s (i=2): 1 ms converge, 25 ms diverge. Esse número vinha sendo usado
como "a margem do LQRY" em geral. Repetido hoje a 15 m/s (i=5, condição do
artigo, refs = 5, Caso 4 ψ Hold e Caso 2 φ Hold, 150 s), com o mesmo
modelo `CL_NL_DH_delay_test.slx` (agora copiado para `mirko_run/`; corrida
via `lqry_delay_run(i, caso, Td)`; modo NORMAL — a S-function nível 1 não
compila em Accelerator; dados em `reproducao_SIL/delay_sweep_c5.mat`).

| Td no comando | i=5, Caso 4 (ψ Hold) | i=5, Caso 2 (φ Hold) | i=2, Caso 4 |
|---|---|---|---|
| 1 ms | converge | converge | converge |
| 10 / 15 / 20 / 25 ms | converge, idêntico (θ −6,4..15,7; H 595..605) | idem | 25 ms: **diverge** (reproduz o ADENDO 6) |
| 50 ms | converge (φ −19,9..12,7) | — | — |
| 75 ms | converge (φ −17,9..10,5; θ até 16,2) | — | — |
| 100 / 150 / 200 / 300 ms | **diverge** (S-function retorna NaN) | — | — |

Figura: `reproducao_SIL/delay_sweep_c5_caso4.png`.

**Leitura.** A 15 m/s o LQRY tolera entre 75 e 100 ms de atraso no
comando — 4× a margem de 12 m/s e bem acima de qualquer laço X-Plane
(20 Hz ≈ 50–75 ms; 100 Hz ≈ 15 ms). Logo, **na condição do artigo, o
atraso NÃO explica a falha no gêmeo**: nem a 20 Hz o laço deveria matar o
ψ Hold segundo o próprio SIL. Isso desloca a causa da divergência a 15 m/s
para as DIFERENÇAS DE PLANTA entre o modelo da Ana e o gêmeo:

1. **Látero-direcional**: o gêmeo v1.2 foi calibrado contra o VOO REAL
   (raios de giração yaw/roll, dutch roll 0,54–0,93 Hz) e a VALIDACAO_VOO_REAL
   já registrou que o lateral do modelo da Ana provavelmente está "quente"
   vs o real (aileron 2×, nunca validado em voo). Os ganhos do ψ Hold foram
   projetados no lateral da Ana e encontram outro lateral no gêmeo — e o
   ψ Hold é exatamente a malha que diverge em voo reto. Candidato principal.
2. **Motor**: lei de empuxo/spool do XP9 vs empuxo instantâneo do modelo
   (ciclo-limite do throttle 0↔1 no Caso 2, ADENDO 8).
3. **Estol a 18,5° e cursos ±15°** (o SIL não tem nenhum dos dois).

O experimento discriminante que falta é rodar o LQRY sobre uma planta
"Ana + lateral do gêmeo" (ou o inverso) — fora do escopo de hoje.

**O que continua verdadeiro.** A 12 m/s a margem é <25 ms (ADENDO 6) e a
ausência de anti-windup no hold de VT segue como deficiência estrutural.
A conclusão "100 Hz é necessário mas não suficiente" (ADENDO 8) se
mantém, com uma correção de causa: a 15 m/s, o que falta não é taxa nem
atraso, é a planta lateral que o controlador não conhece.

## ADENDO 8 (2026-09-02) — Laço a 100 Hz (exigência do Mirko): ajuda, não resolve

**Motivação.** O Mirko foi enfático: o LQRy só funciona com laço de pelo menos
100 Hz. O laço X-Plane↔Simulink dos modelos `modelo_XP_*` roda a **20 Hz**.

**Onde a taxa mora (achado de harness).** Não é só o sample time dos blocos:
os MATLAB Function blocks `Planta/read_xp` e `Planta/send_xp` têm
`Ts = 0.05;` **hardcoded no script** e só chamam `xp_read_dh`/`xp_send_dh`
quando `floor(t/Ts)` muda. Trocar o sample time do chart para 0,01 s (via
`Stateflow.EMChart.SampleTime`) compila a 100 Hz mas o script continua
lendo a cada 50 ms — o 1º voo "100 Hz" (_103342, descartado) tinha 10001
amostras e só 2001 valores distintos de `t_xplane` (5 repetições cravadas).
Correção em `XP_doublets_lqry2.m` (§8): config `XP_dbl_Ts_io` (default
0,01) ajusta EM MEMÓRIA o `SampleTime` e faz `strrep` do `Ts = 0.05;` no
`Script` dos dois charts; `voo.t` e `voo.Ts_io` seguem a taxa. O .slx segue
intacto (20 Hz). `XP_missao_lqry2.m` NÃO recebeu o ajuste (pendente).

**Viabilidade medida (2026-09-02, MATLAB sozinho).** `getDREFs` de 13 drefs:
RTT mediana 2,5 ms (p95 2,6, máx 3,2); `sendCTRL` 0,04 ms; física do XP9
a ~400–430 fps no solo E em voo (`frame_rate_period` 2,2–2,4 ms), toda
leitura em laço cego traz valor novo. 100 Hz cabe com folga; em voo o
laço real ficou com mediana de 13 ms entre leituras (10001/10001
distintas) — ~75–100 Hz efetivos.

**Resultado — ψ Hold puro (Caso 4), 15 m/s, doublets do doc, IC=0**
(`voos/LQRY_taxa_20Hz_vs_100Hz_psiHold_V15.png`):

| Laço | departure \|φ\|>60° | φ máx até 10 s | VT em 10 s |
|---|---|---|---|
| 20 Hz (_174540, 2026-09-01) | **6,8 s** | 150° | 22,2 (zoom) |
| 100 Hz r1 (_103818) | **22,7 s** | 5,3° | 14,0 |
| 100 Hz r2 (_104324) | **11,0 s** | 179° (diverge de 8 s) | — |

A 100 Hz o voo reto + doublet de VT (10–20 s) fica LIMPO (φ ±5°, VT no
ref, sem zoom) — a 20 Hz o zoom para 22 m/s e o wing rock já estão em
andamento aos 5 s. Mas a oscilação lateral cresce sozinha depois: r1
segurou até 22,7 s, r2 só até 11 s (o mesmo caos do transiente visto nos
ADENDOS 3–5). **Consistente com o experimento de atraso (ADENDO 7):
a margem é <25 ms; 20→100 Hz tira ~40 ms de atraso médio e dobra o tempo
até o departure, sem cruzar a fronteira de estabilidade.**

**Resultado — φ Hold + α-protection (Caso 2), 15 m/s, o caso que
COMPLETAVA a 20 Hz** (`voos/LQRY_taxa_20Hz_vs_100Hz_phiHold_V15.png`):
a 100 Hz (r1 _104049 e r2 _104525, quase bit a bit iguais → determinístico)
é IGUAL ao de 20 Hz até 60 s (φ ≤6°, h 577..611 nos doublets de H), mas
nos doublets de φ (62–88 s) o laço mais rápido fica mais nervoso: φ
−22/+18° (20 Hz: −14/+17°), θ 20–40° com α cravado no teto da proteção
(16°) e h subindo a 653 m (20 Hz: 607); no retorno do último degrau
(88 s) faz departure em 91,0/91,1 s (mergulho a 340 m, VT 39). A 20 Hz
o mesmo voo termina em 607 m, θ 17°. Ou seja: **a 100 Hz o Caso 2
DEIXA de completar** (0/2) — o laço mais lento estava, por acaso,
amortecendo a arfagem no limite da proteção.

**Leitura.** A taxa de 20 Hz NÃO era a causa raiz: (1) 100 Hz melhora o
transiente de engate e dobra a sobrevida do ψ Hold, mas ele ainda diverge
em voo reto; (2) o Caso 2 piora. As deficiências estruturais já isoladas
(loop α→θ_ref, hold de VT sem anti-windup, marginalidade lateral a
12–15 m/s) permanecem. Mantém-se a síntese: PID 4/4; LQRY teto 2/4.
Comparação de referência (Mirko): o SIL dele tem atraso ZERO e roda a
100 Hz com planta contínua — a exigência de "≥100 Hz" é condição
necessária, não suficiente, para esta margem.

**Como reproduzir.** `XP_dbl_Ts_io = 0.01` (ou 0.05 p/ o laço original)
antes de `XP_doublets_lqry2`; Caso 4: `XP_dbl_phi_psi=0, refPsi=10,
prot=0`; Caso 2: `XP_dbl_phi_psi=1, refPhi=10, refPsi=0, prot=1`.
Arquivos: `voos/XP_dbl_20260902_{103818,104324}_DBL10_V15_puro_IC0_100Hz*.mat`,
`voos/XP_dbl_20260902_{104049,104525}_DBL10_V15_caso2_phiHold_prot_IC0_100Hz*.mat`.

## ADENDO 7 (2026-09-01) — BUG nos ICs do lançador + doublets 10°/10 m no gêmeo

**Bug de harness (não do controlador).** `XP_missao_lqry2.m` pré-carrega os
integradores com `x4 = [VT; α; 0; θ]` ABSOLUTOS, mas o controlador v2 trabalha
em DESVIOS (`TrimConst*` subtrai `Xe(1)/α_e/θ_e`; no SIL os integradores partem
de ZERO e o engate é bumpless). Efeito medido nos voos `XP_dbl_20260901_1738*`
(15 m/s) e `_1740*` (12 m/s): no engate, throttle **+1182 %** (IC_speed −23,6 ×
G_i −50), profundor **+66°** (IC_theta −0,106 × −625) e θ_ref **+46°** (IC_alt
−4,02 × −0,2025). O "zoom do engate" e parte do "windup do throttle" atribuídos
ao spool de RPM / ao controlador nos ADENDOS 2–5 eram, ao menos em parte,
esse chute. **Todos os voos LQRY2 anteriores (G2, arco suave, t1–t5) usaram os
ICs errados** — releitura necessária. `XP_missao_lqry2.m` NÃO foi alterado;
o lançador novo `XP_doublets_lqry2.m` usa ICs = 0 (idêntico ao SIL).

**Experimento:** voo reto + os MESMOS doublets do modelo do Mirko (V_T +3 m/s
em 10–20 s; H ±10 m; ψ ou φ ±10°), agenda comprimida p/ o motor do XP9
(40→28, 60→41, 80→54, 90→62, 110→75, 130→88; 100 s), reload automático,
ganhos/estrutura intocados, gêmeo v1.2. Gêmeos SIL com a mesma agenda em
`lqry_mirko_atualizado\reproducao_SIL\sil_sched_*.mat`; figuras `SILxXP_*.png`.

| Voo | Config | Resultado |
|---|---|---|
| `_1745_DBL10_V15_puro_IC0` | 15 m/s, Caso 4 (ψ Hold), puro | oscilação longitudinal desde t=0 (θ 8→2→13→6→11°), lateral crescente (período ~4 s): departure |φ|>60° em **6,8 s**, estol 7,2 s — antes do 1.º doublet |
| `_1748_DBL10_V15_prot_IC0` | 15 m/s, Caso 4 + alpha-protection + clamp | α contido até 23 s, mas departure lateral em **6,1 s** — o ψ Hold é instável no gêmeo com ou sem proteção |
| `_1751_DBL10_V15_caso2_phiHold_prot_IC0` | 15 m/s, **Caso 2 (φ Hold, K_bank_guia=0)** + proteções | **COMPLETOU 100 s**: φ ±10° rastreado (pico 15,5°, ψ girou 90° e voltou); H ±10 m virou 564..631 m (−24/+31 m), throttle saturou em 0 e depois em 1 (integrador −10,8 → +19), θ no clamp 17,3° e subindo no fim; oscilação longitudinal de ~5 s o voo inteiro |
| `_1754_DBL10_V12_caso2_phiHold_prot_IC0` | 12 m/s, Caso 2 + proteções | lateral OK (φ ±10°), longitudinal em "mush": throttle 0 o voo inteiro (integrador −42), δe comandado +25° (curso real 15°), α 13,5°, descida 600 → 415 m |
| `_1811_DBL5_V15_caso4_puro_IC0` | 15 m/s, Caso 4 (ψ Hold) puro, **5°/5 m = manobra do artigo** | departure lateral em **5,7 s**, antes do 1.º degrau — a amplitude não é o fator: o ψ Hold diverge em voo reto |
| `_1813_DBL5_V15_caso2_phiHold_prot_IC0` | 15 m/s, Caso 2 (φ Hold) + proteções, **5°/5 m** | **COMPLETOU 100 s** com folga: H 594..605 m (ref 595..605), φ máx 10,3° (ref 5°), ψ girou 24° e voltou; throttle ainda em liga-desliga (comando −1,9 a +3,3) |

**Leitura:** com o harness corrigido, o LQRy v2 do Mirko voa a manobra
10°/10 m no gêmeo SOMENTE no modo φ Hold a 15 m/s e só com as proteções da
plataforma — e mesmo assim com windup de throttle/altitude e excursões 3× o
comando. O ψ Hold (Caso 4, vitrine do artigo) diverge em voo reto em ~6 s em
qualquer configuração; a 12 m/s o par VT Hold + H Hold entra em conflito de
energia (motor cortado + profundor no batente). Lançador: `XP_doublets_lqry2.m`.

## ADENDO 6 — Por que o SIL fica estável: margem menor que 25 ms

Pergunta do Kaue: "como as simulações NL no Simulink funcionam e
deixam a aeronave estável?" Resposta em 4 partes + 1 experimento:

1. Nascem NO equilíbrio exato (estados = trim, integradores
   pré-carregados) — nada precisa convergir, só não divergir;
2. Manobras minúsculas (doublets de ±5°/±5 m) — estabilidade LOCAL,
   que é o que o LQR promete;
3. Perturbação ZERO (sem vento/ruído/atraso/erro de trim) — sistema
   marginal parado fica parado, e no gráfico parece robusto;
4. O modelo perdoa excursões (sem estol, sem limite de superfície).

**Experimento do atraso** (cópia do SIL + Transport Delay no comando,
ganhos/planta/refs intocados; i=2, manobras do doc, modo normal):

| Atraso de laço | Resultado |
|---|---|
| 1 ms | converge (idêntico ao original) |
| **25 ms** | **DIVERGE (NaN)** |
| 50 ms | DIVERGE |
| 75 ms (≈ laço real 20 Hz + UDP) | DIVERGE |

A margem de estabilidade do LQRY a 12 m/s é menor que 25 ms de
latência — 1/3 do laço de qualquer implementação real. O SIL fica
estável porque tem atraso zero. (Modelo do teste:
`scratchpad/CL_NL_DH_delay_test.slx`, temporário.)

**Complemento — "e nem voo reto nivelado?"**: com 25 ms e TODAS as
refs no trim (zero manobra), o modelo matemático não diverge em 150 s
mas entra em CICLO-LIMITE permanente de arfagem (θ 8,6–18,0° em torno
do trim 14,4° sem nenhuma perturbação); um único erro de proa de 2°
(o que todo engate real tem) vira oscilação de φ ±14° (amplificação
7×). O degradê completo: sem atraso = nivelado perfeito; 25 ms =
ciclo-limite; mundo real (50–75 ms + spool + saturações + estol) =
oscilação crescente e departure em ~12 s. Não é a missão que é
difícil — é a margem que é fina demais para qualquer implementação.

**Primeira sobreposição G4 (SIL × X-Plane, mesmo controlador, voo
reto, 12 m/s)**: `voos/G4_LQRY_SILxXP_voo_reto.png` — SIL liso e
estável (inclusive no doublet de VT); X-Plane com φ oscilando desde
t=0, departure em 12,5 s, mergulho. Lado NL salvo em
`voos/SIL_LQRY2_i2_manobras_doc.mat` (refs: VT doublet t=10–20
[11,62→14,62]; H doublet t=40/60/80 [595↔605]; ψ t=90).

## ADENDO 5 — Teste da "missão mansa": REFUTADA — departure em VOO RETO

Hipótese do Kaue pós-auditoria: missão dentro do envelope pequeno
(curvas ≤10°) o LQRY cumpriria. Missão-teste: 3 WPs em arco suave
(curvas de ~10°/perna, 12 m/s, h constante, alpha-protection, WP1 na
proa = zero degrau inicial), gêmeo v1.1, âncoras corretas
(`voos/XP_missao_20260830_193300_*_arco_suave_v2.*`; o voo _191046 é
inválido — engatou com âncoras da planta matemática, ver nota abaixo).

**Resultado: 1/3, com departure em t=12,5 s AINDA NA PERNA RETA** —
nenhuma curva havia sido comandada (erro de ψ ~2° no engate):
φ ±29° já nos primeiros 5 s → ±40° → >70° em 12,5 s; windup do
throttle a partir de t=4 s (zoom de engate → VT 20,6 → motor idle).
A captura do WP1 (50 m, t=14,8 s) ocorreu já rolando pelo círculo.

**Conclusão final do arco LQRY**: a 12 m/s na planta real, o envelope
estável do PsiHold é NULO na prática — a oscilação lateral cresce
sozinha em voo reto (o SIL, mais otimista, dava ±5–10°; as camadas de
implementação — latência 20 Hz, spool pós-teleporte, windup com
saturação real, windmill — consomem a margem restante). Todas as
alavancas foram exauridas: planta estática (v1), dinâmica (v1.1),
proteção de envelope, âncoras, velocidade (12/15), amplitude da missão
(90°→10°→reta). Melhor caso permanece **2/4** (v1, 12 m/s, sorte do
caos no transiente). Única alavanca não testada: anti-windup DENTRO do
controlador — com expectativa moderada, pois a marginalidade lateral
independe do motor.

Nota de harness: âncoras do gêmeo (thr 0.45 / δe +5.50) agora são
DEFAULT INTERNO do lançador p/ i=2 — não dependem de override no
workspace (os clears de outros lançadores as apagavam e o engate caía
no trim da planta matemática, thr 0.284 → windup garantido).

## ADENDO 4 — Auditoria a pedido do Kaue: "funciona no NL" vale só p/ degraus ≤5–15°

Verificação completa contra o documento de setup do Mirko
(`lqry_mirko_atualizado/Nova pasta/Novo(a) Documento de Texto.txt`:
i=5, refVel=15.2, refs=5, PsiHold, AltHold, VelHold):

1. **Ganhos conferidos**: os 8 conjuntos usados em TODOS os voos
   (salvos em `voo.cfg` de cada .mat) são bit a bit idênticos aos
   `Ganho_hold_*.mat` atuais da pasta (diff = 0). Trim `Ue` idem.
   Não há ganho desatualizado no harness.
2. **SIL reproduzido**: com a config exata do documento, o SIL dele
   converge (VT/H/ψ nos refs) — "funciona no NL" confirmado NAS
   AMPLITUDES DO DOCUMENTO: doublets de ±5°/±5 m/+0,35 m/s.
3. **Na nossa velocidade (i=2, 12 m/s)**: converge com os doublets de
   5, mas θ excursiona a **+25,7°** e VT cai a **9,8 m/s** — o modelo
   matemático atravessa porque NÃO TEM estol (gêmeo/real: 18,5°) nem
   limite de superfície (única saturação da planta SIL: throttle
   [0,1]; profundor/aileron/leme ILIMITADOS).
4. **Degraus de proa em amplitude de missão, no PRÓPRIO SIL** (modelo
   recarregado antes de cada run — trocar `i` sem recarregar mistura
   planta velha compilada e dá falso NaN):

   | Degrau de proa | i=5 (15 m/s, planta do doc) | i=2 (12 m/s, nossa) |
   |---|---|---|
   | 5°  | OK (φ ±18°) | OK (φ ±17°; θ até +25,7° nos doublets long.) |
   | 15° | OK, mas φ chega a **−53°** | **DIVERGE (NaN)** |
   | 30° | **DIVERGE (NaN)** | DIVERGE |
   | 90° (exigido pela missão G2, 4x) | **DIVERGE (NaN)** | DIVERGE |

**Conclusão da auditoria**: não existe contradição "funciona no NL,
falha no X-Plane". Nas amplitudes de MISSÃO (curvas de ~90°), o LQRY
**não funciona nem no modelo não linear da própria planta de projeto**
— o PsiHold tem envelope estável de ±15° a 15 m/s e ±5–10° a 12 m/s.
O X-Plane foi na verdade MAIS benigno que o SIL: com as superfícies
saturando em ±25° a resposta fica limitada (oscila, captura 2/4) em
vez de integrar até NaN. As validações em SIL eram demonstrações de
pequenos sinais na vizinhança do trim — necessárias, mas não
suficientes para a tarefa de guiagem por waypoints.

## ADENDO 3 — GÊMEO v1.1 (dinâmica casada): a falha fica ISOLADA no windup

Com a inércia de arfagem calibrada (gêmeo v1.1: ωn 6,25 vs alvo 6,3 —
EQUIVALENCIA_ACF.md), o LQRY foi re-voado no G2 em duas configurações:

- **Puro** (`voos/XP_missao_20260830_171651_*_G2_puro.*`): 1/4 (WP1 a
  63,7 m). Transiente de engate CAIU de θ 29° (v1) p/ 21,6° — a
  dinâmica casada reduz a amplificação do loop de α — mas ainda tocou
  o estol (18,5°); asa solta → mergulho (VT 18,4) → windup do hold de
  VT → mush sem potência em α 36–46° → departure na perna 2.
- **Com alpha-protection** (`voos/XP_missao_20260830_172755_*_G2_prot.*`):
  **arfagem 100% resolvida** — α máx 16,2° pré-departure, zero estol
  longitudinal, engate limpo (θ pico 16,2 nos primeiros s). Mas o zoom
  residual do engate (θ 21° a 1,3 s, reação ao spool de RPM
  pós-teleporte) levou VT a 20 m/s → hold de VT (Gs=−78,8 %/(m/s))
  comandou −630%, saturou em 0 e o integrador afundou (−5→−25%, pico
  −153/+1097% no voo) → **motor em idle de t≈4 s até o fim** → voo sem
  potência a α≈15° → wing rock lateral (φ ±17→±73°) → departure
  t≈18 s. 0/4.

**Isolamento final**: depois de (1) equivalência estática (v1),
(2) equivalência dinâmica (v1.1) e (3) proteção de envelope na
plataforma, a ÚNICA deficiência restante é o **windup do integrador do
hold de velocidade** sob saturação de throttle — deficiência de
IMPLEMENTAÇÃO clássica (o projeto LQRY não especifica anti-windup e o
ganho de −78,8 %/(m/s) satura o atuador com ±1,3 m/s de erro). Ela é
interna ao controlador: não existe correção possível do lado da
planta/harness. Corrigi-la (1 clamp no integrador, ganhos intocados)
seria o experimento final "custo de implantação do LQRY"; sem ela, o
melhor caso permanece 2/4 (v1, 12 m/s).

## ADENDO 2 — Alpha-protection na PLANTA: a falha MIGRA de camada

Última tentativa de 4/4 sem tocar ganhos: proteção de envelope no
COMANDO, implementada na `Planta_XP` (o "avião", estilo fly-by-wire) —
chart `alpha_protection` entre o `Sum_trim` e o `send_xp`: quando o α
MEDIDO excede 16°, o teto nose-up do δe desce em rampa (2° de α) até o
δe de trim (+5,5°). Params `prot_on`/`alpha_prot`/`de_trim` no
lançador.

Resultado (`voos/XP_missao_20260830_163223_LQRY2_gemeo_G2_prot.*` +
dry-run `_162519_*`):

- **A proteção longitudinal FUNCIONA**: 335 amostras com α>16° e δe
  cravado em +5,50° em TODAS — o LQRY **nunca mais estolou de
  arfagem** (α confinado a 14–16° na fase controlada; sem os θ_ref
  24–29° das rodadas anteriores).
- **Mas a falha migrou para a camada seguinte**: com α contido, o hold
  de velocidade (Gs(VT) = −78,8 %/(m/s), sem anti-windup) saturou o
  throttle em 0 (comando −17%→−25% e afundando ~Gint) → sem empuxo,
  voo em pré-estol crônico → **wing rock lateral** (φ ±40→±70° em
  ~6 s, t=13–21 s) → departure pelo eixo LATERAL, que a proteção não
  cobre. 0/4 nesta rodada.

**Conclusão do arco completo**: não existe UMA prótese que leve o LQRY
ao 4/4 — cada proteção adicionada expõe a deficiência estrutural
seguinte (loop de α → windup do throttle → lateral em pré-estol). Um
4/4 exigiria empilhar alpha-protection + anti-windup DENTRO do
controlador + amortecimento lateral extra, i.e., re-engenharia do
controlador — fora do escopo (ganhos/estrutura do Mirko intocáveis).
**Melhor caso do LQRY permanece 2/4 a 12 m/s.** O contraste com o PID
em cascata (4/4 com clamps triviais de referência) vira o argumento
central: a arquitetura define onde e COMO se protege o envelope.

### Placar consolidado (todas as rodadas, gêmeo v1)

| Configuração | Capturas | Modo de falha |
|---|---|---|
| PID cascata, 12 m/s | **4/4** | — (nominal ~150 s + planeio guiado) |
| LQRY 12 m/s | **2/4** (melhor caso) | ciclo-limite tocando estol → departure longitudinal |
| LQRY 12 m/s + clamp θ_ref | 2/4 | idem (clamp vaza: só afeta o termo integral) |
| LQRY 15 m/s (3 variantes) | 0/4 | vazamento do clamp cresce 0,6°→4,6° |
| LQRY 12 m/s + alpha-protection na planta | 0/4 | **sem estol longitudinal**; windup do throttle → wing rock lateral |

## ADENDO FINAL — LQRY no GÊMEO v1 (planta equivalente ao projeto)

Com o .acf transformado em gêmeo do drone real (EQUIVALENCIA_ACF.md:
α de trim 14,50 vs 14,44; CLα idêntico), o LQRY v2 foi re-voado com
i=2 (a planta nominal, que agora corresponde ao avião) e âncoras/ICs
do trim de projeto (`voos/XP_missao_20260830_1538*_LQRY2_gemeo_*`):

- **t1 (1 WP)**: CAPTUROU (37,5 m) — primeira captura do LQRY na
  campanha. Sem explosão; mas ciclo-limite de arfagem: o loop de α
  amplifica o transiente de engate, pede θ até 29° (α 32°) e o gêmeo
  ESTOLA a 18,5° (não-linearidade ausente no projeto) → oscila α 10–30°.
- **G2 (4 WPs)**: **2/4 capturas** (WP1 62 m, WP2 54 m, incluindo a
  1ª curva de 90°); na perna do WP3 o ciclo-limite tocou o estol uma
  vez demais → departure (tumbling) → missão perdida.

### Tabela-síntese do experimento completo

| Controlador | .acf divergente | Gêmeo (= planta de projeto) |
|---|---|---|
| PID cascata (com theta_ref_clamp) | 4/4, nominal | 4/4, nominal |
| LQRY (sem proteção de envelope) | explode ~6 s, 0/4 | 2/4, ciclo-limite no estol, departure |

**Conclusão consolidada**: a equivalência da planta recupera a
funcionalidade do LQRY (de "explode" para "voa e captura"), mas a
diferença decisiva entre capturar e CUMPRIR a missão é a proteção de
envelope: o clamp de θ_ref do PID (teto 17,3°, 1° abaixo do estol)
contra o LQRY que comanda θ 28°+ sem conhecer o estol. Em planta com
não-linearidade dura, saturações bem postas pesam mais que a lei de
controle em si.

### Experimento de confirmação: clamp de θ_ref NO LQRY → INÓCUO

Adicionado `Sat_thetaref_envelope` (mesmos limites do PID, [4,3°;17,3°],
var `XP_clamp_lqry`) entre o hold H e o hold θ do LQRY, ganhos
intocados. G2 no gêmeo COM clamp
(`voos/XP_missao_20260830_155351_*_clamp.*`): **mesmo placar** (2/4,
departure na perna do WP3), θ ainda a +24° (vs 29° sem clamp — efeito
apenas parcial). **Causa estrutural**: no LQRY o comando é
`δe = Gs·[VT α q θ δe] + Gi·∫(θ−θ_ref)` — o clamp de referência limita
só o termo integral; a realimentação direta de estados contorna o
limite e leva o avião além do estol mesmo assim. No PID em cascata, a
malha interna só enxerga o erro da referência limitada → o clamp é
parede absoluta. Proteção equivalente no LQRY exigiria
*alpha-protection no comando* (saturar δe em função do α medido),
i.e., alterar a estrutura do controlador.

**Refinamento final da conclusão**: proteção de envelope é trivial e
efetiva na arquitetura em CASCATA; é semi-permeável num regulador de
estados plano — a arquitetura, não apenas os ganhos, define a
capacidade de confinar o voo ao envelope seguro.

## ADENDO — versão ATUALIZADA do Mirko (lqry_mirko_atualizado, 18-jun-2026)

A versão nova (gain scheduling 3×3 Ve×He via var `i`; PsiHold
RE-PROJETADO — validado no SIL: φ(0)=3° converge em ~12 s, a réplica
antiga explodia; atuadores modelados; controlador em DESVIOS com o trim
`Plantas(i).Ue` somado na planta) foi transplantada com o mesmo rigor:
`modelo_XP_LQRY2_GUIA.slx` + `XP_missao_lqry2.m` (trim somado na
Planta_XP, ICs em deltas, ref de VT da missão em vez do trim da planta
agendada, scheduling POR EIXO: longitudinal pela planta de α compatível
(i=8, α_e=4,5° ≈ .acf), lateral pela pressão dinâmica (i_lat=2, 12 m/s)).

**Resultado: diverge igualmente (5 voos, t1–t5).** O voo instrumentado
(`voos/XP_missao_20260830_135225_LQRY2_t5_probe.mat`, probes nas
referências internas) isola o mecanismo de forma definitiva:

- refVT = 12,00 e refH = 0,00 CONSTANTES (referências perfeitas);
- h cravada em 600 m (erro de altitude ~0);
- e ainda assim **θ_ref do hold de altitude: +8° → +32° (1,5 s) → +65°
  (3 s)** → estol. Com ZERO erro de altitude, quem move θ_ref é o
  termo de estado em α: `GstateLong_Alt(α) ≈ +4,5 rad/rad` (idêntico em
  TODAS as 9 plantas e na v1 — assinatura do projeto). No .acf, o loop
  α → θ_ref → δe → α↑ é instável (na planta matemática, a dinâmica de
  curto período com α_e alto o fecha estável).
- Secundários corrigidos no caminho (documentados no lançador): a ref
  de VT do harness dele parte de `Plantas(i).Xe(1)` (17,95 p/ i=8 → o
  throttle% com Gint=−50 ia a +297%/s); o hold VT novo tem
  `Gs(VT) = −78,8 %/(m/s)` e satura/winda sem anti-windup quando o .acf
  entra em pré-estol.

**Conclusão consolidada (v1 E v2)**: com ganhos e estrutura intocados, o
LQRY não estabiliza o .acf — não por afinação, mas pelo acoplamento
estrutural de α na malha longitudinal, cuja estabilidade depende da
dinâmica exata da planta de projeto. O PID cascata (sem realimentação
de α; erros de saída + clamps) voou a mesma missão com ganhos originais.

> Mesma metodologia do PID (mesmo harness, mesma guiagem LOS, mesmas
> âncoras de engate, mesma missão G2), controlador LQRY da réplica do
> Mirko **com ganhos e estrutura 100% intocados**.

## O transplante (modelo_XP_LQRY_GUIA.slx + XP_missao_lqry.m)

- Cópia de `mirko_replica/CL_NL_DH_SIL_manobras.slx` com **apenas a
  `Planta` substituída** pelo X-Plane (interface 2 in/11 out preservada:
  u_long=[thr;de_rad], u_lat=[da;dr]; saídas VT/α/q/θ/p/r/β/φ/ψ_rel/H).
- H de controle **relativo ao engate** (equivale ao estimador integrador
  do Mirko, que partia de 0); ψ relativo ao engate (como no PID).
- α lido do X-Plane (canal 14 novo do `xp_read_dh`).
- Guiagem LOS idêntica à do PID + lei bank-to-turn
  (φ_ref = sat(0.1975·wrap(ψ_los−ψ), ±20°) — mesma agressividade do
  K_heading do PID) para usar o bank hold nativo.
- **Engate bumpless**: ICs dos integradores pré-carregados (thr(0)=0.55,
  de(0)=+2°, θ_ref(0)=θ₀) — verificado nos logs (u(0) = trim real).
- Paths isolados da árvore do PID (funções homônimas) — lançador faz
  `restoredefaultpath` e só adiciona Dados_mat_SIL + xplane + XPC.

## Resultado: o LQRY original NÃO estabiliza o DH do X-Plane

Voos: `voos/XP_missao_20260830_*_LQRY_t1..t4.*` (todos: engate limpo →
divergência em 3–6 s → estol/rolagem; α 2°→16-33°, φ até ±140°).

| Modo | SIL (planta do Mirko) | X-Plane (.acf v3) |
|---|---|---|
| phi_psi=0 (heading hold) | **INSTÁVEL: φ(0)=3° → NaN** | diverge ~6 s |
| att_alt=0 (hold H) + bank | estável (sem perturbação) | **estol ~5 s** |
| att_alt=1 (θ-hold, modo do artigo) ref +2° | estável | **diverge igual** |

## Causa (análise)

1. **Heading hold nunca foi estabilizante** — nem na planta matemática
   original (as sims do artigo usavam phi_psi=1/bank e não o revelaram;
   `GstateLat_psi` tem −9.15 em β e −1.99 em φ).
2. **Longitudinal**: os ganhos de estado têm α com peso alto e positivo
   (`GstateLong_Alt(α)=+4.54`, `GstateLong(α)=+5.36` rad/rad) — casados
   ao ponto de projeto da planta matemática (α_e=θ_e=14.4°). No .acf
   (α_trim≈5°, estol≈15°, derivadas diferentes) esse termo vira
   realimentação desestabilizante: qualquer Δα>0 → θ_ref/δe sobem → α
   sobe → **estol em poucos segundos**. Estados ABSOLUTOS realimentados
   levam o sistema ao equilíbrio embutido nos ganhos — que no .acf é
   pós-estol.

## Leitura para o artigo/dissertação

Mesma planta X-Plane, mesma missão, mesmos trims: o **PID cascata**
(erros de saída + integradores + clamps de proteção) voou o G2 com os
ganhos originais (4/4 capturas, 4 missões); o **LQRY** (realimentação de
estados, model-based) diverge em segundos com os ganhos originais — a
sensibilidade à discrepância planta-de-projeto × planta-real é
estruturalmente maior. Um re-projeto do LQRY sobre a planta do X-Plane
(fora do escopo: ganhos do Mirko são intocáveis neste trabalho)
provavelmente voaria — o ponto é robustez, não capacidade da técnica.

## Reproduzir

```matlab
% X-Plane aberto (File > Open Aircraft no DH antes), MATLAB:
XP_WPs_frame = [600 0 600 12]; XP_TimeXP = 45; XP_tag = 'LQRY_repro';
run xplane/XP_missao_lqry.m     % modos default: hold H + bank-to-turn
% overrides p/ os outros modos: XP_att_alt=1; XP_reftheta=-12.44;
```
