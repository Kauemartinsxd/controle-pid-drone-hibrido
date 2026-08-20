# Plano: Guiagem por Waypoints do DH no X-Plane (estilo PIPER-1-6)

> Plano autocontido para implementação em sessão futura. Contexto completo do
> harness em `xplane/` (commit 6f1be53) e na memória do projeto do agente.
> Referência: guiagem LOS do Julio Machado em
> `trabalho_julio/PIPER-1-6-roll_back/PIPER-1-6-roll_back/guiagem/`.

## STATUS 2026-08-20 — implementado; G2 nominal aguarda conserto do motor

- **Fase 1 FEITA**: `xp_read_dh` com 13 canais (xN/xE/ψ_abs, âncora no
  engate); `modelo_XP_DH_CL` atualizado (Demux 13). Engate melhorado:
  re-zero de atitude/taxas após a confirmação do teleporte.
- **Fase 2 FEITA**: `modelo_XP_DH_GUIA.slx` com `Guidance_Star` (LOS do
  Julio + conversão ψ_abs→ψ_rel; wp_idx inicia em 1; hold de proa após
  capturar o último WP — adaptação ao envelope do DH, o Julio orbita) e
  `Step_theta_test` (degrau de θ_ref p/ validação da Fase 0).
- **Fase 3 FEITA**: `XP_missao.m` (WPs no referencial da proa de engate ou
  NE, interpolação Δh>20 m, TimeXP auto, pre-flight tolerante, resumo de
  capturas) + `plot_XP_missao.m` (trajetória 2D + séries, salva PNGs).
- **Fase 0 FEITA E SEPARADA (opt-in)**: identificação q/δe no X-Plane
  (`XP_ident_theta.m`, dados em `voos/XP_ident_theta_*.mat`) e retune
  documentado em `XP_retune_Ctheta.m` (C_theta_XP: Kp 1,6/Ki 0,9 — OS 8%
  vs 14,9% do original nesta planta). **Por decisão do Kaue, o trabalho
  oficial voa com os ganhos da dissertação**; o retune só entra com
  `XP_use_Ctheta_XP = true` no `XP_missao`.
- **Campanha**: G1 ✓ (captura a 7,8 m). **G2-planeio ✓** (2026-08-19 23:16,
  `voos/XP_missao_20260819_231632_G2_planeio.*`): quadrado 500×500 m
  rotacionado pela proa de engate, **4/4 capturas (26,6/10,4/27,7/0,0 m),
  φ max 22,5° < 25°, sync 0,994, ganhos originais** — trajetória passa
  pelos 4 círculos (ver PNG). VT ficou 10±1 (não 12±1) e h desceu em
  rampa: causa única = **hélice do .acf sem empuxo em voo** (≤0 N; ver
  `PENDENCIA_MOTOR.md`). Lição de voo: teto de θ_ref +5° é obrigatório
  (teto 6° → ciclo de estol, reproduzido e registrado em
  `voos/XP_missao_20260819_225422_G1.mat`).
- **Falta**: conserto da hélice no Plane Maker (checklist em
  `PENDENCIA_MOTOR.md`) → re-voar G2 nominal (h constante, VT 12±1);
  G3 (Δh) e G4 (SIL) na sequência.

## Objetivo

Voar o DH no X-Plane por uma **missão de waypoints** — matriz
`WPs = [Norte(m), Leste(m), Altitude(m), Velocidade(m/s)]` com troca por raio
de aceitação — usando a cascata PID da dissertação **intocada** como malha
interna. Meta de validação: circuito quadrado 500×500 m em altitude constante.

## Estado de partida (fim de 2026-08-19)

- Harness `xplane/` FUNCIONAL: `XP_voo.m` = config dourada (voo sustentado,
  VT 11,93±0,4 no ponto de projeto, ψ ±1°, thr/de com folga).
- Limitação conhecida: regulação de h assenta ~20 m abaixo da ref (teto do
  `theta_ref_clamp` = proteção de energia; ver §Pré-requisito).
- `.acf` v3 (massa 2,2 kg, superfícies ±25°, h-stab −2°, asa +5°, hélice
  +3° manual, 1,6 hp) — fora do repo, em
  `X-Plane 9/X-Plane 9/Aircraft/Radio Control/DH-Lon-REV-03.acf`
  (backup `_backup_20260819` = original do Sato).

## Pré-requisito recomendado (Fase 0): retune do C_theta

O overshoot de θ (malha interna sintonizada para a dinâmica de arfagem do
modelo da Ana) é o que obriga o clamp apertado e causa o offset de h.
Com C_theta re-sintonizado para a planta do X-Plane, o clamp pode abrir e o
C_alt recupera altitude dos dois lados — o que a guiagem exige (WPs com Δh).

Método (mesma metodologia do retuning 2026-08-10 da dissertação):
1. Identificação rápida: degraus de δe em malha aberta no X-Plane
   (harness `XP_malha_aberta.m` adaptado) → ajuste de 2ª ordem q/δe;
2. `pidtune` no modelo identificado (wc ~3 rad/s, PM 75°) OU ajuste
   empírico (reduzir Kp/Ki do C_theta ~30–50% e validar overshoot < 10%);
3. Validar: degrau de θ_ref ±5° sem overshoot > 10%, depois reabrir o clamp
   gradualmente (teto +3 → +6) verificando ausência dos ciclos de estol
   (assinatura: θ oscilando ±5°, φ ±30°, VT < 10 — ver voos `_teto6` e
   `_douradav2` em `xplane/voos/`).

Alternativa se a Fase 0 for adiada: guiagem com **altitude constante**
(WPs todos na mesma h) e Δh futuro ≤ 15 m — funciona com o clamp atual.

## Fase 1 — Posição NE no `xp_read_dh`

Adicionar canais de posição ao vetor de leitura (hoje 10 elementos):

- `y(11) = xN`, `y(12) = xE` — posição relativa ao ENGATE [m], âncora
  persistente zerada no `cmd==1` (mesmo padrão do ψ relativo);
- `y(13) = psi_abs` — proa absoluta [rad] (a LOS é calculada em NE absoluto;
  o ψ relativo do canal 7 continua alimentando o heading hold nas manobras).
- Fonte (padrão do `ins_read_xplane` do Julio, XP9 OpenGL):
  `local_x` = Leste, `local_z` = Sul →
  `xE = local_x − x0`, `xN = −(local_z − z0)`, âncora (x0, z0) no engate.
- Atualizar: Demux do modelo (10→13), `XP_inicializacao`, `plot_XP_DH`.
- ATENÇÃO (lições de 2026-08-19, todas documentadas nos comentários do
  `xp_read_dh.m`): manter o pacing por `t_sim`, o reopen do socket
  pós-confirmação e o teleporte-no-1º-sample. Não usar `pauseSim` para o
  teleporte (não persiste). Blocos novos de log: `MaxDataPoints = inf`.

## Fase 2 — Modelo `modelo_XP_DH_GUIA.slx`

Cópia do `modelo_XP_DH_CL.slx` (via `save_system`, como foi feito hoje) com a
geração de referências substituída pela guiagem:

1. **Bloco `Guidance_Star`** (MATLAB Function, discreto Ts=0.05 — setar via
   `Stateflow.EMChart` como hoje: `ChartUpdate='DISCRETE'`, `SampleTime='0.05'`):
   portar quase literal de `chart_44` do NL_guidance do Julio (código completo
   extraído em 2026-08-19; ~40 linhas):
   - entradas: `xN, xE` (canais 11–12), `psi_abs` (13), `psi_rel` (7);
   - parâmetros (workspace): `WPs`, `R_accept`;
   - persistente `wp_idx` (inicia em 1 — nosso WP1 já é o primeiro alvo,
     diferente do Julio que nasce no WP1);
   - troca: `dist <= R_accept && wp_idx < num_wps → wp_idx+1`;
   - LOS: `psi_los = atan2(E_alvo − xE, N_alvo − xN)` (absoluto);
   - **conversão para a convenção relativa do harness**:
     `psi_ref_rel = psi_rel + wrap(psi_los − psi_abs)` com
     `wrap(x) = mod(x+pi, 2*pi) − pi` (o `calc_erro_proa` do Julio) —
     assim NADA muda dentro do subsistema `controle`;
   - saídas: `psi_ref_rel, h_ref_wp, v_ref_wp, wp_idx_mon, dist_mon`.
2. **Fiação**: as saídas substituem os caminhos de Step/doublet dentro de
   `controle` — opção A (menos invasiva): somar `psi_ref_rel` no ponto do
   `Add_dbl_Step1` (blk_782) com steps zerados; h_ref e VT_ref viram os
   sinais do bloco (hoje são constantes de workspace — trocar as Constants
   `blk_488`/`Step_VT_ref` por Inports ligados à guiagem). Mapear IDs com
   `model_read` na hora (IDs de hoje: `controle`=blk_476, ver memória).
3. **Logs**: `wp_idx_mon`, `dist_mon`, `xN/xE` → ToWorkspace
   (`MaxDataPoints=inf`) para o plot da trajetória 2D.
4. Suavização de Δh: adotar a interpolação do Julio (WPs intermediários se
   Δh > 20 m) — no lançador (Fase 3), não no bloco.

## Fase 3 — Lançador `XP_missao.m`

Clone do `XP_voo.m` (manter TODAS as âncoras da config dourada) com:

- `WPs` e `R_accept` (default 80 m) configuráveis no topo;
- WP1 default = 500 m à frente da proa de engate, na h de engate;
- interpolação automática de Δh > 20 m (copiar lógica do `scr_aux_wp.m`);
- `TimeXP` dimensionado pela missão (perímetro/12 m/s × 1,5);
- critério de fim: `wp_idx == num_wps && dist < R_accept` por 5 s → pode
  simplesmente deixar o tempo esgotar segurando o último WP (Julio faz isso);
- pós-voo: salvar `.mat` em `xplane/voos/` + plot 2D da trajetória com os
  WPs e círculos de R_accept (adaptar `plot3d_voo_xplane.m` do Julio).

## Fase 4 — Campanha de validação

| Teste | Missão | Critério |
|---|---|---|
| G1 | 1 WP a 500 m na proa de engate, mesma h | captura (dist < 80 m) sem oscilação lateral |
| G2 | Quadrado 500×500 m, h constante (= circuito do Julio) | 4 capturas, φ < 25° nas curvas, VT 12±1 |
| G3 | Quadrado com Δh = +15 m em um trecho (pós Fase 0: ±30 m) | captura + h converge no trecho |
| G4 (extra) | Mesma missão no SIL (portar Guidance_Star ao modelo_NL_DH_CL) | comparação SIL × X-Plane p/ dissertação |

## Riscos e mitigação

- **Curvas de 90° com K_heading=0.1975 e clamp de φ implícito**: LOS puro dá
  degrau de ~90° em ψ_ref na troca de WP → φ_ref = K_heading×1.57 ≈ 17° — OK
  (o C_phi segura; validado φ ±40° não diverge). O pré-filtro de ψ_ref
  (tau_ref=3 s) já suaviza a troca.
- **Bistabilidade de energia** (lição de hoje): curvas convertem energia —
  monitorar VT nos logs; se cair < 10,5 m/s nas curvas, reduzir R_accept ou
  ganho K_heading temporariamente (documentar).
- **ψ_abs vs ψ_rel**: cuidado com o unwrap do canal 7 — a conversão da Fase 2
  usa ambos exatamente para não mexer no `controle`.
- **XP9**: todas as manhas de DataRef/UDP estão nos comentários de
  `xp_read_dh.m`/`xp_send_dh.m` — LER ANTES de editar.

## Definição de pronto

`XP_missao` roda o quadrado 500×500 (G2) de ponta a ponta sem intervenção,
salva o `.mat` e gera o plot 2D com a trajetória passando pelos 4 círculos
de aceitação. Bônus: G3 e G4.
