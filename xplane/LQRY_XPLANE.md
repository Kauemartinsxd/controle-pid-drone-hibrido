# LQRY no X-Plane — experimento e resultado (2026-08-30)

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
