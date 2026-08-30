# LQRY no X-Plane — experimento e resultado (2026-08-30)

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
