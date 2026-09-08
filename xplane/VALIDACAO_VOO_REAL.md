# Validação do gêmeo X-Plane contra VOO REAL do DH — com contraponto do .acf original

**Campanhas:** 2026-09-01 · **Modelos:** GÊMEO v1.1 × **.acf ORIGINAL intocado**
(backup 2026-08-19, como recebido) · X-Plane 9 + XPC
**Dados reais:** 2 voos de dez/2024, ArduPlane V4.5.6 em Pixhawk 1, drone híbrido
em asa-fixa pura (`Q_ENABLE=0`), **modo MANUAL** (RCOU = passthrough do rádio =
entrada da planta, verificado; manete corr 0,99). Cruzeiro 15,5–19,8 m/s.

- `Log-DH-longitudinal-dez-24.bin-440203 (1).mat` (raiz do projeto)
- `Log-DH-laterodirecional-dez-24.bin-449987 (1).mat`

## Método (pipeline `VR_*`)

1. **`VR_extrai_segmentos.m`** — 21 janelas válidas de 10 s (2 s pré + 8 s pós
   doublet; a 22ª era o pouso), PWM→normalizado ((PWM−1500)/400; manete
   (PWM−1100)/800), saídas a 25 Hz (ATT/IMU/ARSP/BARO). Sinais de convenção
   servo→X-Plane estimados do próprio dado (correlação entrada×taxa):
   `elev +1, ail −1, rudd −1` — consistentes entre os dois voos.
2. **`VR_replay_xp.m`** — por janela: **reload do .acf** (motor fresco — ver
   Protocolo), teleporte a MSL 600 na V₀ real, **autotrim** (manete←erro de V;
   θ_ref lento←erro de ḣ₀ real do BARO; converge com |V−V₀|<1,5, |ḣ−ḣ₀|<1,
   |q|<3°/s por 2 s), replay em **malha aberta, modo delta**
   (`u_XP = trim_XP + Δu_real`, ZOH 25 Hz, ~20 Hz), **sem re-teleporte**
   trim→replay.
3. **`VR_plot_comp.m`** — métricas na janela dinâmica (1 s antes → 4 s após):
   RMS/FIT% da taxa primária, razão de picos, freq. do dutch roll (FFT de r).

### Protocolo: motor fresco por segmento (lição do dia)

O estoque de energia do motor elétrico do XP9 dura ~130 s de voo motorizado
(pendência conhecida). Campanhas iniciais voaram blocos de 4–7 segmentos sem
reload ⇒ segmentos tardios voaram com motor fraco/morto (manete saturada,
planeio). **Isso muda a resposta**: no XP9 o sopro da hélice sobre as
superfícies altera a autoridade de controle — com motor morto a resposta
lateral medida caiu ~3×. Protocolo final: `xp_reload_acf` antes de CADA
segmento, nas duas aeronaves. Campanhas contaminadas renomeadas `SUSPEITO_*`
(a 1ª rodada "gêmeo g=1,3 → picos 1,00" era condição de baixo sopro; mantida
como observação, não como resultado).

## Resultados — protocolo justo, ganho 1,0, mesmos 21 segmentos

### Estática / capacidade de voar a condição real (robusto ao sopro)

| Métrica | GÊMEO v1.1 | .acf ORIGINAL |
|---|---|---|
| Trims convergidos (V, ḣ e q nos critérios; NÃO é queda — o original voa, mas rápido demais e fora da condição real) | **21/21** | 14/21 |
| Trim de profundor vs real (real ≈ **neutro**, −0,02) | −0,11 (≈ −1,7°) | **+0,41 (≈ +6,1°)** |
| Erro médio \|δe_trim − real\| | **0,112** | 0,407 (3,6× pior) |

O avião real cruza a 15–20 m/s com profundor essencialmente neutro. O gêmeo
reproduz isso; o original precisa de ~6° cabrado nas mesmas condições — o
desbalanceamento de arfagem (trim α 5° vs 14,4° do modelo/real) agora
demonstrado **contra dados de voo**, não só contra o modelo da Ana.

### Dinâmica (doublets, janela 1–6 s) — resumo por eixo

| Eixo | GÊMEO v1.1 | .acf ORIGINAL | Real |
|---|---|---|---|
| Profundor: pico Δθ do doublet, ref. LOCAL (média 7 seg)¹ | **16,0° (−13%)** | 12,5° (−32%) | 18,4° |
| Profundor: razão pico q (7 seg) | 0,47–0,99 (méd 0,72; g=1,3 → méd 0,85) | 0,60–1,23 (méd 0,95, espalhado) | 1 |
| Aileron: razão pico p (6 seg) | 2,7–3,4 (**~3× quente**) | **1,06–1,39** | 1 |
| Leme: razão pico r (8 seg) | 2,2–3,4 | 1,0–1,6 | 1 |
| Dutch roll (real 0,54–0,73 Hz, pouco amortecido) | sobreamortecido, morre em ~1 ciclo | freq. espalhada 0,2–1,4 Hz, fase errada | oscila ≥3 s |

¹ Pico de Δθ medido do valor LOCAL do próprio traço em t=1,9 s (logo antes
do doublet) até o máximo em 2–3,2 s. Referência local é OBRIGATÓRIA: em
malha aberta cada modelo deriva diferente nos 2 s pré-doublet (o original
afunda ~3° — o desbalanceamento dele agindo), e ler a figura sem isso
exagera a diferença (aparenta +22/+9 vs real +20; o justo é a tabela).
Ordem gêmeo>original mantém-se em 6/7 doublets.

Figura-síntese: `voos/VR_3vias_real_gemeo_original.png` (real × gêmeo ×
original, 1 segmento por eixo, taxa + atitude — ATENÇÃO à ressalva ¹ ao ler
os painéis de atitude).

## Leitura honesta (para o texto e para a banca)

1. **Longitudinal: o gêmeo vence com folga e por razões físicas.** Trim de
   profundor neutro como o real (erro 3,6× menor que o original), 21/21
   condições sustentadas, amplitude de Δθ do doublet a −13% do real
   (original −32%; ref. local, ver nota ¹), forma/fase do short-period
   corretas. É o eixo que a
   Fase B calibrou (asa/perfil/incidência + inércia de arfagem) — e é onde a
   validação com voo real confirma a calibração.
2. **Látero-direcional: nenhum dos dois está calibrado — e o gêmeo está
   pior em amplitude** (~3× quente em p e r com motor vivo; o original fica
   1,1–1,4×). Coerente com o histórico: as inércias de rolagem/guinada NUNCA
   foram calibradas (v1.1 calibrou só arfagem) e as mudanças de perfil da
   Fase B alteraram o eixo lateral sem validação dedicada. O dutch roll
   falha nos dois (gêmeo sobreamortecido; original com fase/freq erradas).
   **Caminho já identificado**: calibrar raios de giração de roll/yaw pelo
   mesmo método do pitch + revisar diedro/deriva.
3. **Sensibilidade ao sopro da hélice (achado de plataforma)**: no XP9 a
   autoridade de controle depende fortemente do estado do motor (resposta
   lateral ~3× entre motor vivo e morto). Toda comparação de amplitude deve
   declarar o estado do motor; o protocolo de reload por segmento controla
   isso.
4. **Correção de alegação anterior**: o ".acf exige manete ~1,0 nivelado a
   17–19 m/s" era artefato do motor esgotando (~130 s); com motor fresco o
   gêmeo nivela com manete 0,2–0,6 nessas velocidades.
5. **Curso do profundor**: a razão de pico <1 do gêmeo persiste em ambos os
   protocolos (méd 0,72–0,78) e é compatível com curso real > ±15° do .acf;
   o ganho 1,3 levou a média a 0,85 (motor vivo) / 1,00 (baixo sopro). A
   estimativa "curso efetivo ~19–20°" fica como hipótese sustentada mas com
   incerteza pela dependência do sopro.

## Calibração látero-direcional (2026-09-01 tarde) — mesma receita do pitch

Método idêntico ao v1.1 (raio de giração → resposta), agora com o **voo real
como gabarito** e ajuste EM TEMPO DE EXECUÇÃO via dref (`sim/aircraft/weight/
acf_J{yy,zz}_unitmass` = R² em m²; **escrita aceita e honrada pela física;
reload reseta ⇒ escrever após cada reload** — `VR_J` no `VR_replay_xp`,
sonda `XPL_probe_lat.m`).

Alvos extraídos dos 8 doublets reais de leme: **f_DR = 0,70 Hz, ζ ≈ 0,1,
pico r 45–70 °/s**. Varredura (produto r·Jyy ≈ constante, como previsto):

| Jyy [m²] | pico r [°/s] | f_DR [Hz] | ζ |
|---|---|---|---|
| 0,069 (auto) | 180 | — (sobreamortecido) | ~0,24 |
| 0,140 | 101 | — | 0,25 |
| **0,210** | **73** | **0,42** | **−0,04** |

**Valores calibrados: Jyy = 0,20 m² (R_yaw = 0,45 m = 1,47 ft — igual ao
raio de pitch!), Jzz = 0,078 m² (R_roll = 0,28 m = 0,92 ft).** Os raios
automáticos do XP9 são sistematicamente ~2,5–3× baixos em J — mesmo fator do
pitch (0,94→1,47 ft ⇒ ×2,4).

Validação (replay dos 14 segmentos laterais reais, protocolo justo):

| Métrica | raios AUTO | raios CALIBRADOS | real |
|---|---|---|---|
| Leme: razão pico r (8 seg) | 2,2–3,4 | **0,75–1,53 (mediana 1,07)** | 1 |
| f_DR X-Plane | ~inexistente (sobreamort.) | **0,54–0,93 Hz em 6/8** | 0,54–0,73 |
| Aileron: razão pico p (6 seg) | 2,7–3,4 | 1,26–2,40 (mediana ~2,0) | 1 |
| Fits aileron | −61..−117 % | −21..+56 % (2 positivos) | — |

Figura: `voos/VR_lat_antes_depois.png`; campanha
`voos/VR_replay_GEMEO_JCAL_MERGED.mat`; figs `VR_comp_*_JCAL.png`.

**O que a inércia NÃO conserta (esperado e verificado):** o pico de rolagem é
quase-estacionário (`p_ss = Cl_δa/Cl_p`, independe de I_xx — Jzz×3 só levou
679→469 °/s na sonda). O excesso residual de p (~2×) é ganho de entrada/
eficácia: ou o curso real do aileron é menor que os ±15° do .acf (medir NO
DRONE — análogo invertido do profundor), ou Cl_δa/Cl_p do .acf está alto
(aileron/perfil no Plane Maker). Pendências menores: amortecimento do dutch
roll ainda um pouco alto e acoplamento Δφ fraco (diedro/Cl_β — Plane Maker).

**PERMANENTE — GÊMEO v1.2 (2026-09-01)**: Kaue aplicou no Plane Maker
(Standard → Weight & Balance, painel na ordem pitch/yaw/roll): pitch 1,47 /
**yaw 1,47 / roll 0,92 ft**. Verificado por dref no .acf salvo: Jyy 0,201,
Jzz 0,079 m² sem escrita em runtime — `VR_J` não é mais necessário. Backup
congelado `DH-Lon-REV-03_gemeo_v1_2_20260901.acf` (hash 5B808D0B; v1.1 =
E0CDB818).

## Cursos das superfícies confirmados: ±15° (2026-09-01)

Informação do responsável físico do DH: as superfícies trabalham em **−15° a +15°**
(configuração adotada para os próximos voos; dez/24 provavelmente igual —
confirmar). Consequências:

- A conversão PWM→deflexão com o curso de ±15° do .acf está correta **sem
  fator de ganho**. A hipótese "curso efetivo ~19–20°" do profundor deixa de
  ser a explicação primária.
- **Profundor**: a razão de pico 0,72–0,78 vs real passa a indicar
  **eficácia de profundor (Cm_δe) ~25% baixa no .acf** — coerente com o
  resíduo já conhecido vs modelo da Ana ("ganho de q ~30% < modelo; o fit casou
  polos, não numerador"). Fix: aumentar a autoridade do profundor no Plane
  Maker (corda/área) e revalidar nos 7 doublets → v1.3.
- **Aileron**: o gêmeo ~2× quente vs real, com o original em 1,1–1,4×. A Fase B
  aumentou o aileron (razão 0,27) para casar a "rolagem −20%" vs modelo da
  Ana ⇒ o eixo de rolagem do modelo da Ana provavelmente está quente frente ao
  avião real (nunca foi validado em voo). Fix prático: reduzir a eficácia de
  aileron ~2× no Plane Maker (próximo ao original) e revalidar nos 6 doublets.

## Figura estilo Sato (comparável à Fig. 6.13 do precedente)

`VR_plot_sato_real.m` → `voos/VR_sato_doublet_real.png`: o doublet REAL de
profundor (seg 2, V₀ 18,8 m/s) nos três mundos com a MESMA convenção visual
da Fig. 6.13 do Sato (absolutos, eixos largos, preto=medido, vermelho=modelo
identificado, azul tracejado=X-Plane): Medido × **modelo da Ana não linear
trimado na velocidade real + replay delta** (análogo exato do "M1
Identificado" do Sato) × gêmeo v1.2. Resultado: q dos três praticamente
sobrepostos; θ do modelo da Ana colado no medido; V_T dos três numa faixa de
±2–3 m/s (o X-Plane do Sato caía a 5 m/s e errava θ por 20–30°). **BÔNUS —
validação independente do modelo da Ana contra voo real**: trimado a
18,8 m/s o modelo prevê δe −1,4° e manete 0,462; o voo real cruzava com δe
≈ −0,3° e manete ≈ 0,47. Leitura p/ a banca: por que a Fig. 6.13 "parece"
boa — eixos largos comprimem erros, e a curva colada é o modelo AJUSTADO
àqueles dados, não o X-Plane; na mesma convenção visual, nosso conjunto é
mais fechado que o do precedente. **Par com o original**: variante
`VR_sato_doublet_real_ORIG.png` (mesma figura, X-Plane = .acf original via
`VR_sato_campanha`) — o azul aparece deslocado em bloco: δe de operação
+4,5° (real −0,3), θ andando 7–13° abaixo do medido com blip de doublet
minguado, V_T subindo a +4,5 m/s do alvo. O par de figuras é o argumento
visual completo: mesma convenção, troca-se só o .acf.

## Figura-evidência: "o original não representa o DH"

`voos/VR_evidencia_original_nao_representa.png` — a única figura que afirma
sozinha o que as tabelas dizem (pedido do Kaue: "qual imagem diz isso?"):

- **Esquerda**: δe de trim que cada modelo precisou para voar AS MESMAS 21
  condições do voo real. Real: −0,3° (média, faixa −0,6…+0,4). Gêmeo v1.2:
  −1,9° (faixa −4,7…+0,1). **Original: +5,9° (faixa +3,5…+9,6)**. Três
  faixas que não se tocam — o original voa o mesmo envelope com o profundor
  em OUTRO lugar (α de trim 5° vs 14,4°), o que os controladores projetados
  no modelo da Ana sentem como feedforward errado + windup desde o engate.
- **Direita**: pico de Δθ (ref. local) nos 7 doublets de profundor. Real
  18,4° / gêmeo 16,0° (−13%) / original 12,5° (−32%); original é o menor em
  6/7.

Usar ESTA figura (e não as de eixo largo) sempre que a afirmação for "os
dois .acf não representam igualmente a aeronave" — as de estilo Sato servem
só para comparabilidade com o precedente.

## Arquivos

- Segmentos: `voos/VR_segmentos.mat` + `VR_segmentos_overview.png`
- Campanhas (protocolo justo): `voos/VR_replay_GEMEO_MERGED.mat`,
  `voos/VR_replay_ORIGINAL_MERGED.mat`, `voos/VR_replay_GEMEO_ELEV13_MERGED.mat`
- Comparações: `VR_comp_seg*_*.png` (gêmeo), `*_ORIG.png` (original),
  `*_G13.png` (gêmeo g=1,3); resumos `VR_comp_resumo*.png`;
  **síntese 3 vias: `VR_3vias_real_gemeo_original.png`**
- Campanhas descartadas (motor sem controle): `SUSPEITO_VR_replay_*.mat`,
  `VR_replay_20260901_MERGED.mat` (gêmeo manhã), `_111251.mat` (g=1,3 manhã),
  `_113558/_113844.mat` (leis de trim antigas)

## Reprodução

```matlab
cd xplane
VR_extrai_segmentos                     % 1) janelas dos logs
% X-Plane 9 aberto + XPC; .acf desejado ATIVO (e backups iguais a ele
% na pasta, pois o reload automatico pode abrir qualquer linha do dialogo):
VR_lista = [];  VR_replay_xp            % 2) campanha (reload/segmento)
VR_plot_comp                            % 3) figuras + tabela
```

ATENÇÃO: O clique do `xp_reload_acf` pode abrir qualquer arquivo `DH-Lon-REV-03*`
do diálogo — durante campanhas A/B, deixar TODOS os arquivos da pasta com o
conteúdo da aeronave em teste (cópias de segurança fora da pasta) e restaurar
ao final. Estado final desta sessão: pasta restaurada, `DH-Lon-REV-03.acf` =
gêmeo v1.1 (hash E0CDB818), backups nos conteúdos verdadeiros.
