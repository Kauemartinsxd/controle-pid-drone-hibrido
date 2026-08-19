# Pendências do artigo — fechar na máquina da bancada (Windows)

Documento autocontido para completar o draft `arquivos/main.tex` do artigo
IEEE Access **"Cascade PID Versus Output-Feedback LQR for the Fixed-Wing
Mode of a Hybrid VTOL UAV"**. Todos os pontos abertos estão marcados no
LaTeX com `\pendente{...}` (aparecem em vermelho no PDF). Para listá-los:

```
grep -rn "pendente{" arquivos/sections/ arquivos/main.tex
```

## Contexto editorial (decidido em 18/08/2026 — não reverter)

- A lei comparada no artigo é o **LQRy de GANHO FIXO** (a entrega
  embarcada na bancada, campanha de 10/ago) — síntese única no trim
  (12 m/s, 600 m), sem gain scheduling.
- O **LQRy gain-scheduled foi removido por completo do artigo** (nem
  como "sonda de tuning"). `mirko2026companion` segue citado apenas
  pela metodologia de síntese.
- Narrativa: **PID vence desempenho nominal; LQRy vence robustez**
  (conclui a missão com o modelo do Sato ≈ nominal enquanto o PID a
  perde).
- As figuras de comparação/robustez/acoplamento atuais são as da
  apresentação de alinhamento (10/ago), embutidas em **tema escuro**
  (`arquivos/figs/fig_v1_*.png`) — as curvas estão corretas; falta só
  a versão em fundo branco.

## Onde estão os dados

- `.mat` das campanhas: `C:\Users\kaue\Documents\PID_DH\HIL_PID\Matlab\`
  — subpasta `Dados_mat_ROBUSTEZ` contém pelo menos `PID_sato.mat`,
  `LQR_sato.mat` (v1 = ganho fixo) e `LQR2_sato_parcial.mat`
  (scheduled — ignorar); os demais cenários seguem nomenclatura
  análoga. Conferir a pasta.
- Scripts de referência (formato dos dados, métrica e export):
  `arquivos/sims/fig_sato_v3.m` (padrão de figura branca, 300 dpi),
  `arquivos/sims/pid_recampanha_v2.m` e
  `arquivos/sims/compute_metrics_v2.m` (convenção de RMS).
- Números v1 de referência para conferência cruzada (deck
  `docs/Apresentacao_alinhamento_artigo.html`, campanha 10/ago):
  - SIL nominal — PID RMS 0,07 / 1,27 / 1,65 (V_T [m/s] / ψ [°] / h [m]);
    LQRy 0,38 / 2,92 / 1,78.
  - Sato completo — LQRy RMS 0,353 / 2,61 / 1,77 (degradação 0,93/0,89/0,99).
  - Vento — LQRy h ≈ 5,2 m RMS (2,9× nominal; pico 11,2 m), ψ pico 18,6°.
  - HIL — PID 0,07 / 1,27 / 1,63; LQRy 0,37 / 2,92 / 1,78.
- Missão (mesma em tudo): V_T 12→15,2 m/s em 5 s; doublet de ψ ±5° em
  50/100/150 s; doublet de h ±5 m em 80/160/240 s; 300 s; métricas com
  janela t > 12 s.

## P1 — Tabela de ganhos do LQRy v1 (com o Mirko)

**Onde:** `arquivos/sections/03_control_design.tex`, marcador
"inserir aqui a tabela com os ganhos do LQRy de ganho fixo".

**O quê:** os ganhos de registro da entrega embarcada, nas cinco malhas
(ThetaHold, AltitudeHold, VelHold, PhiHold 2×2 com integrais de φ e β,
PsiHold), no formato u = G_x·x + K_I·∫e dt — mesmo layout da antiga
tabela (vetor de estados, G_x, K_I por malha). Fonte: Mirko / workspace
da entrega embarcada na bancada.

## P2 — Confirmar o harness v1 (com o Mirko)

**Onde:** `03_control_design.tex`, marcador "confirmar com o Mirko que
o harness da entrega de ganho fixo contém exatamente estes elementos".

**O quê:** o texto afirma que o rig do LQRy tem lags de atuador de
24 rad/s (superfícies) e 0,1 s (manete), saturação de manete [0,1], e
NÃO tem rate limits, saturação de posição, prefiltro de referência nem
clamp de θ_ref (referências entram como degrau cru). Confirmar item a
item no rig v1; corrigir o texto se algo divergir (§III-C e a lista de
assimetrias em §IV-B).

## P3 — RMS exatos do LQRy v1 nos cenários B, C e D

**Onde:** `arquivos/sections/05_robustness.tex`, Tabela 7
(`tab:robustness`) — células "≈ nominal, all channels" (B e C) e as
células "—" do cenário D; nota de rodapé b.

**O quê:** calcular RMS (t > 12 s) do erro de V_T, ψ e h do **LQRy v1**
nos cenários: (B) Sato só-longitudinal, (C) Ana ×1,10 em todas as 23
derivadas, (D) vento e rajadas — mesma convenção de
`pid_recampanha_v2.m`/`compute_metrics_v2.m`, a partir dos `.mat` da
campanha de 10/ago. Preencher também a degradação (RMS cenário / RMS
nominal, nominal = 0,38 / 2,92 / 1,78). Conferência: em D, h deve dar
≈ 5,2 m (2,9×). Depois de preencher, apagar a nota `\pendente` do
rodapé b e trocar as células qualitativas pelos números.

## P4 — Re-exportar as 5 figuras v1 em fundo branco

**Onde:** `arquivos/figs/` — sobrescrever com os MESMOS nomes (o .tex
não precisa mudar):

| Arquivo | Conteúdo | Cenário |
|---|---|---|
| `fig_v1_comb.png` | missão combinada PID × LQRy (estados esq., atuadores dir.) | SIL nominal |
| `fig_v1_rob_sato.png` | 4 painéis V_T/ψ/h/θ, PID + LQRy + ref | Sato completo |
| `fig_v1_rob_satoL.png` | idem | Sato só-longitudinal |
| `fig_v1_rob_vento.png` | painel do vento (W_N/W_E/W_D) + V_T/ψ/h/θ | vento e rajadas |
| `fig_v1_acopla.png` | varredura só-PID φ=5/15/30° (φ, Δθ, Δh, Δelevador) | acoplamento |

**Como:** os scripts que geraram essas figuras para o deck existem na
máquina da bancada; basta trocar o estilo escuro por fundo branco
(padrão de `fig_sato_v3.m`: `figure('Color','w')`,
`exportgraphics(...,'Resolution',300,'BackgroundColor','white')`),
mantendo curvas, cores das séries e layout. Depois apagar os
`\pendente{re-exportar em fundo branco...}` das legendas
(sections 03, 04 e 05).

## P5 — `fig_eq_comb.png` (ablação de equalização)

**Onde:** `arquivos/figs/fig_eq_comb.png`; marcador na legenda em
`04_sil_simulations.tex`.

**O quê:** a figura atual mostra PID "como entregue" × PID equalizado
com uma curva de contexto do LQRy **errada** (era do scheduled).
Re-exportar com a curva de contexto do LQRy de ganho fixo — ou sem
curva de contexto nenhuma (nesse caso, ajustar a legenda removendo
"with the LQRy response for context").

## P6 — Com os autores (não é tarefa da máquina)

- `mirko2026companion` (refs.bib): confirmar status do artigo companion
  (submetido? a submeter?) e dados bibliográficos antes da submissão.
- `angelo2026` (refs.bib): fechar dados bibliográficos definitivos da
  campanha de identificação com a Ana.
- Biografias, e-mails e ORCIDs dos coautores (marcadores PENDENTE no
  `main.tex`).

## Depois de completar

1. Recompilar: `pdflatex main` → `bibtex main` → 2× `pdflatex main`
   (na pasta `arquivos/`). Os 2 erros recuperados na carga do
   tikz/pgf são atrito conhecido da classe — ignorar.
2. Conferir que `grep -rn "pendente{" arquivos/` só retorna o que for
   deliberadamente deixado para depois.
3. Commit na `main` descrevendo o que foi fechado.
