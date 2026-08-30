# LQRY no X-Plane — experimento e resultado (2026-08-30)

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
