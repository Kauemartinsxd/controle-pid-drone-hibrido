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

---

# 2026-09-08 — Dois erros de harness corrigidos, replay refeito e calibração látero-direcional v1.3 por dref

Pergunta do Kaue: "dá para aproximar mais o modelo X-Plane do voo real? O
longitudinal parece bom, o látero não." Ao revisar as figuras do leme, a
resposta do X-Plane aparecia como **imagem espelhada** da real (lida até então
como "atraso de ~0,3 s"). Investigando, dois erros no pipeline VR:

## Erro 1 — sinal do leme invertido na conversão servo → X-Plane

`VR_extrai_segmentos` estimava o sinal por correlação entrada × taxa com lag
livre 0..0,6 s. Com dutch roll pouco amortecido e o 2º lobo do doublet
invertendo r no pico, a |correlação| máxima caía em lag ~0,3–0,4 s **com sinal
trocado** → `rudd −1`. Critério robusto (inclinação inicial da taxa nos
primeiros 0,15 s do 1º lobo — efeito direto da superfície): **rudd +1 em 8/8
segmentos**, elev +1 (8/8), ail −1 (6/6). Só o leme estava errado.
Consequência: TODAS as comparações de leme de 09-01 (X-Plane e modelo NL nos
slides/TIC) estavam espelhadas — só no papel, espelhar r do X-Plane leva o
fit médio do leme de −92 % para −11 %. Corrigido em `VR_extrai_segmentos.m`;
`voos/VR_segmentos.mat` re-salvo com os sinais novos (SEG intacto; backup
`VR_segmentos_pre20260908_rudd-1.mat`). ATENÇÃO: os logs ArduPilot brutos
não estão mais na raiz do projeto (só atalhos em Recent) — o .mat foi
corrigido a partir dos próprios SEG; para re-extrair, restaurar os .mat dos
logs na raiz `Dissertacao_Mestrado/`.

## Erro 2 — deflexão 1,67× no gêmeo (cursos ±25° do .acf × ±15° do drone)

Os drefs `acf_elev_up/dn`, `acf_ail1_up/dn`, `acf_rudd_lr` do gêmeo v1.2 leem
**25°** nas três superfícies (o original tem 15°). O replay mandava o PWM real
normalizado (±1 = ±15° físicos) direto no `sendCTRL`, cujo ±1 é o curso do
.acf ⇒ o gêmeo recebia **25/15 = 1,67× a deflexão real em todos os eixos**
(o original, 1,0×). Todos os números de amplitude de 09-01 carregam esse
fator: "aileron 2× quente" era 1,2× por grau; "leme 1,07 após Jyy" era 0,64;
"profundor 0,72–0,78" era 0,43–0,47. `VR_replay_xp` agora lê os cursos do
.acf e aplica `esc = VR_curso_real/curso_acf` por eixo (`VR_curso_real = 15`
default; `[]` = comportamento antigo), gravando `esc/lims` no .mat da
campanha. Também: `VR_drefs` (cell {nome, valor} escrito após cada reload),
salvamento incremental por segmento e `reload_robusto` (ESC + retentativa
quando o diálogo Open Aircraft fica aberto e o XPC emudece — visto 2×).

## Gêmeo v1.2 re-medido com o pipeline corrigido (22 segmentos, motor fresco por segmento)

Campanha `voos/VR_replay_GEMEO_FIX_MERGED.mat`; figs `VR_comp_*_FIX.png`,
`VR_comp_resumo_FIX.png`; antes×depois `VR_lat_fix_vs_jcal.png`.

| Eixo (n) | Métrica | 09-01 (leme −1, 1,67×) | **09-08 corrigido** | real |
|---|---|---|---|---|
| Aileron (6) | pico p XP/real, mediana [faixa] | 1,98 [1,26–2,40] | **0,84 [0,68–1,04]** | 1 |
| | fit % de p (médio) / TIC | +14 / 0,35 | **+52 / 0,27** | |
| Leme (8) | fit % de r (médio) / TIC | −92 / 0,86 | **+23 / 0,43** | |
| | pico r XP/real, mediana | 1,07 | 0,71 [0,45–1,00] | 1 |
| | f_DR X-Plane (onde a FFT acha) | 0,54–0,93 | 0,63–0,83 | 0,54–0,73 Hz |
| Profundor (8) | pico q XP/real (média) | 0,72 | **0,30** | 1 |
| | Δθ local médio (7 doublets) | 16,1° (−3 %) | **9,2° (−45 %)** | 16,6° |

Leitura: com a entrada certa, o aileron do v1.2 já é ~15 % fraco (não 2×
quente), o leme ~30 % fraco, e **o profundor fica com ~30 % da autoridade real**
— o "−13 %" do longitudinal de 09-01 era artefato da entrada 1,67×. O trim de
profundor do gêmeo também é maior em graus do que o registrado (−0,32 a
18,8 m/s = −8°, com 25°; o real voa neutro).

## Calibração em tempo de execução por dref (XP9 honra; reload reseta)

`DataRefs.txt` do XP9 expõe como graváveis: `sim/aircraft/controls/acf_{elev,
ail1,rudd}_crat` (razão de corda da superfície: gêmeo 0,20 / 0,27 / 0,30),
`sim/aircraft/parts/acf_dihed1[56]` (diedro por parte; asa = partes 8–9),
`acf_Croot/Ctip/semilen_SEG[56]` (asa 8–9: 0,60 m × 0,25/0,20; h-stab 16–17:
0,226 × 0,094 m; v-stab 18–19: 0,119 × 0,171/0,079 m), os `acf_J**_unitmass` e
`sim/flightmodel/misc/cgz_ref_to_default` (CG, +z = atrás). Teste de honra (1
segmento cada): elev crat 0,2→0,6 dobrou o pico de q (0,27→0,51); rudd crat
0,3→0,6: r 0,45→0,70; ail crat 0,27→0,40: p 0,68→0,76; diedro −4°: Δφ muda.
⇒ dá para calibrar sem Plane Maker e só depois gravar os valores no PM.

**Inércias do drone (modelo da Ana, `modelo_DH.m`): Ixx 0,144 / Iyy 0,116 /
Izz 0,257 kg·m²; massa 2,2 kg.** O gêmeo v1.2 está com Iyy 0,446 (**3,9×**),
Izz 0,446 (1,7×), Ixx 0,175 (1,2×): as inércias de guinada/arfagem foram
infladas em 09-01/08-30 para casar picos medidos com entrada 1,67× e um Cmα
alto. Configurações voadas (8 segmentos: leme 19/20/21/7, aileron 10/12/14/13):

| Config | pico r (med.) | f_DR XP/real | ζ_DR XP/real | pico p (med.) | fit p |
|---|---|---|---|---|---|
| v1.2 corrigido (mesmos 8) | 0,59 | — | 0,17/0,09 | 0,84 | +55 % |
| L1: rudd crat 0,55 + ail 0,45 | 0,65 | 0,65/0,61 | 0,13/0,09 | 0,88 | +53 % |
| **L2: inércias da Ana (yaw/roll), crat originais** | **0,88** | **0,61/0,61** | 0,23/0,09 | 0,84 | +55 % |
| L3: L2 + rudd 0,40 + ail 0,45 | 1,07 [0,77–1,41] | 0,49/0,61 | 0,13/0,09 | 0,91 | +56 % |

A razão de corda do leme tem efeito fraco no XP9 (0,30→0,55: +10 %) e
espalha; o que fecha o pico de r é a **inércia de guinada física**. O pico de
p é quase-estacionário (independe de Ixx) e responde pouco ao crat (0,27→0,45:
+8 %). A dispersão entre corridas do mesmo segmento (±20–30 %) está ligada à
manete de trim (sopro da hélice sobre a empenagem): corridas com thr0 ~0,75
(trim não convergido a 17–18 m/s) dão r maior.

## v1.3-lat (candidata) — validada em 14 segmentos, `VR_replay_GEMEO_V13LAT_MERGED.mat`

**Definição: inércias da Ana em guinada e rolagem + aileron crat 0,45; leme
0,30 e profundor/arfagem do v1.2 intocados.** Figura `VR_lat_v13_vs_v12.png`;
figs `VR_comp_*_V13LAT.png`, `VR_comp_resumo_V13LAT.png`. (Decisão do Kaue,
09-08: "não precisa deixar IGUAL" — parou-se aqui.)

| Eixo (n) | Métrica | v1.2 corrigido | **v1.3-lat** | real |
|---|---|---|---|---|
| Aileron (6) | pico p, mediana [faixa] | 0,84 [0,68–1,04] | **0,94 [0,84–1,17]** | 1 |
| | fit p / TIC (medianas) | +52 % / 0,27 | +53 % / **0,24** | |
| | Δφ pico XP/real | 1,19 | **1,08** | 1 |
| Leme (8) | pico r, mediana [faixa] | 0,71 [0,45–1,00] | **1,00 [0,83–1,31]** | 1 |
| | fit r / TIC | +23 % / 0,43 | +12 % / 0,43 | |
| | f_DR XP (segs em que a FFT acha) | 0,63–0,83 | 0,49–0,83 | 0,54–0,73 Hz |
| | ζ_DR (decremento log., mediana) | 0,22 | 0,20 | 0,14 |
| | Δφ pico XP/real | 1,62 | 1,41 | 1 |

Resíduos declarados: (a) dutch roll do X-Plane mais amortecido que o real
(ζ 0,20 vs 0,14; a f_DR bate) — knob é braço/área da deriva (PM), não
testado; (b) os 4 doublets de leme do voo longitudinal (segs 5/6/8/22, V0
17–18 m/s) trimam com manete 0,75 e dão r 1,0–1,3 (sopro) — o fit médio de r
cai de +23 para +12 % por eles, embora a mediana do pico esteja em 1,00;
(c) deriva lenta de φ/V em malha aberta (espiral + correções reais de aileron
replayadas) domina a FFT em metade dos segmentos (f_DR "0,22" = borda da
banda, não medida).

### Receita para o Plane Maker (Kaue) — v1.3-lat

1. Standard → Weight & Balance → raios de giração (ordem pitch / **yaw** /
   **roll**): pitch **1,47 ft** (mantém), yaw **1,12 ft** (era 1,47;
   Jyy_unitmass 0,1157 m² = Izz 0,257/2,22), roll **0,84 ft** (era 0,92;
   Jzz_unitmass 0,0648 m² = Ixx 0,144/2,22).
2. Aileron: razão de corda **0,27 → 0,45** (`acf_ail1_crat`; tela da asa /
   Control Geometry do PM 9).
3. Leme 0,30 e profundor 0,20 mantidos; nada no longitudinal.
4. Depois: `XP_sync_acf_clones`; conferir por dref (J 0,2008/0,1157/0,0648,
   m 2,22, elev 25, ail1_crat 0,45); `xp_reload_acf` aceita agora as duas
   assinaturas (v1.2 e v1.3-lat) e imprime qual carregou; re-rodar
   `G5_xp` dblpsi (malha fechada ψ) para confirmar que o PID segue 4/4 —
   esperado: 1ª curva de ψ menos lenta (aileron +) e OS de ψ menor.
   Fidelidade dos voos G4/G5 longitudinais não muda.

## Achado longitudinal (documentado, NÃO aplicado)

Com a entrada certa o profundor do gêmeo dá **0,30** do pico de q real (Δθ
−45 %). Causa dupla: Iyy 3,9× a real e profundor minúsculo (crat 0,20 numa
corda de 9,4 cm). Testes no seg 2 (V0 18,8 m/s; ajuste 2ª ordem q/δe por
`vr_fit_q_de`, mesmo ajuste no real: ωn 6,4 rad/s, ζ 1,4 — bem amortecido):
- v1.2: pico q 0,27, ωn 7,6 / ζ 0,58, trim δe −8°;
- D1 = Iyy da Ana (Jxx_unitmass 0,052 = R 0,75 ft): pico q **0,67**, mas ωn
  **13,4** (gêmeo rígido demais em arfagem — explica também a curva de trim
  íngreme: −8° a 18,8 m/s onde o real voa neutro);
- D2 = D1 + CG 4 cm atrás (`cgz_ref_to_default` +0,04): ωn 10,5, ζ 0,67, pico
  0,68, trim −11° (piora o trim);
- elev crat 0,2→0,6 (com Iyy v1.2): pico 0,27→0,51.
Conclusão: o longitudinal v1.3 exige mexer junto em CG, incidência do
h-stab, Iyy e corda do profundor (trim neutro + ωn ~6–7 + ζ alto + pico q 1) —
é trabalho de PM com o Kaue, fora deste passo. Mantido o longitudinal v1.2:
a malha PID compensa o ganho baixo de profundor (G4/G5 continuam válidos);
para o LQRy a alegação "gêmeo −25 % Cm_δe" passa a ser **≈ −55…−70 %** —
reforça a conclusão dos ADENDOS 10–13 (saturação/windup) sem mudá-la.

## Reprodução (09-08)

```matlab
cd xplane
% X-Plane 9 aberto, gemeo v1.2 ativo (6 clones iguais na pasta), XPC ativo
VR_lista = [5:14 19:22]; VR_curso_real = 15;                 % lateral, escala 15/25
VR_drefs = {'sim/aircraft/weight/acf_Jyy_unitmass', 0.25716/2.2226; ...
            'sim/aircraft/weight/acf_Jzz_unitmass', 0.14410/2.2226; ...
            'sim/aircraft/controls/acf_ail1_crat', 0.45};       % v1.3-lat por dref
VR_replay_xp                                                  % salva incremental
VR_merge_campanhas('VR_replay_2026MMDD_*.mat', 'VR_replay_X_MERGED.mat', [5:14 19:22]);
VR_analise_lat('VR_replay_X_MERGED.mat', 'VR_replay_GEMEO_FIX_MERGED.mat', {'X','v1.2'}, 'fig.png');
VR_arq_replay = 'voos\VR_replay_X_MERGED.mat'; VR_sufixo = '_X'; VR_plot_comp
```

### v1.3-lat APLICADA no Plane Maker (2026-09-08, 13:24, Kaue)

`DH-Lon-REV-03.acf` ativo agora é o **gêmeo v1.3-lat** (md5 e6090986; backup
`acf_backups_20260902/DH-Lon-REV-03_gemeo_v1_3lat_20260908.acf`; v1.2 continua
em `_gemeo_v1_2_20260901.acf`, md5 5b808d0b). `XP_sync_acf_clones` rodado
(6 clones iguais). Conferido por dref após reload: J 0,2008 / **0,1165** /
**0,0656** m² (= 1,47 / 1,12 / 0,84 ft), m 2,22 kg, cursos 25/25/25°,
**ail1_crat 0,45**, elev_crat 0,20, rudd_crat 0,30 — `xp_reload_acf` imprime
"assinatura OK = gemeo v1.3-lat". Falta: G5 dblpsi de conferência em malha
fechada (esperado: PID 4/4, 1ª curva de ψ menos lenta).

**Armadilha nova (13:25): a janela do Plane Maker também se chama "X-System".**
Com o PM aberto, o `AppActivate('X-System')` do `xp_reload_acf` trazia o
PLANE MAKER para a frente: os 2 cliques do reload e os ESC de recuperação
iam para o PM, o diálogo Open Aircraft ficava aberto no X-Plane e o XPC (e o
UDP 49000) emudeciam. `AppActivate(pid)` também não bastou; o que funcionou foi
user32 `SetForegroundWindow` no `MainWindowHandle` do processo X-Plane
precedido de `SendKeys('%')` (ALT). Corrigido em `xp_reload_acf` (ps1) e no
novo `xp_activate.m` (usado pelo `reload_robusto` do `VR_replay_xp`).
Regra prática: **fechar o Plane Maker antes de qualquer campanha** e, se o
PM estava aberto durante um reload falho, conferir se os cliques perdidos
não alteraram algo nele antes de salvar de novo.

### G5 dblpsi de conferência no v1.3-lat (2026-09-08 13:36, `voos/XP_voo_20260908_133643.mat`)

Doublet de proa ψ ±15° (10/45/80 s) em malha fechada, PID 100 % da
dissertação, vento leve na UI (1–1,5 kt, turb. 0,19 — nem o "com vento" nem o
"calmo" de 31/08; comparação só indicativa). Figura
`G5_dblpsi_SILxXP_v13lat.png`; referências re-plotadas em
`G5_dblpsi_SILxXP_v11ref.png` (v1.1 com vento) e `_v11calmo_ref.png`.

| Fase | Métrica | SIL | v1.1 vento | v1.1 calmo | **v1.3-lat** |
|---|---|---|---|---|---|
| ψ +15 | OS % / ts5 % s / ess ° | −1,7 / 24,3 / −0,32 | 9,9 / 33,0 / 0,14 | −4,0 / 31,6 / −0,72 | **3,0 / 35,0 / −0,52** |
| ψ −15 | OS / ts / ess | −1,6 / 24,1 / 0,62 | 3,7 / 35,0 / 1,09 | −0,2 / 22,0 / 0,20 | **3,2 / 34,0 / 0,03** |
| retorno | OS / ts / ess | −2,4 / 23,5 / −0,47 | 8,9 / 29,6 / −1,08 | −3,8 / 26,2 / −0,69 | **9,0 / 30,0 / −0,89** |
| global | RMS ψ ° / RMS h m / RMS VT m/s | 8,39 / 0 / 0 | 8,52 / 0,33 / 0,20 | 8,82 / 0,32 / 0,18 | **8,64 / 0,29 / 0,21** |

Leitura: a malha de proa do PID fecha as três fases (OS ≤ 9 %, ess < 1°),
h 599–601 m, φ máx 3,6°, VT 11,1–12,8 — mesmo comportamento e mesma faixa de
métricas do v1.1; nenhuma regressão. A hipótese "1ª curva menos lenta" NÃO se
confirmou (ts5 % 35 vs 33 s): o ts de ψ é ditado pelo ganho da malha, não pela
planta; o OS de +15 caiu de 9,9 para 3,0 %, mas com atmosfera diferente, sem
atribuição. Conclusão: a v1.3-lat é transparente para o PID no envelope suave
(φ ≤ 5°) e mais fiel ao voo real em malha aberta — pode substituir a v1.2 como
gêmeo padrão. (Voos G4/G5 anteriores em v1.1 permanecem válidos como registro.)

### TIC de Theil recalculado (2026-09-08, `VR_tic.m`, `voos/VR_tic_20260908.mat`)

Janela dinâmica 1–6 s, medianas por eixo, **real × NL da Ana | real × X-Plane**.
Níveis pré-doublet removidos (dVT, dθ, dφ). NL = trim da Ana em V0 + replay
delta das 4 entradas reais (sinais de `VR_segmentos.mat`; o "antes" usa
leme −1 também no NL, como nos slides de 01/09).

| Eixo | sinal | antes 01/09 (leme −1, XP 1,67×) | hoje v1.2 corrigido | hoje v1.3-lat |
|---|---|---|---|---|
| Profundor (n=7–8) | q | 0,19 \| 0,43 | 0,19 \| **0,62** | — |
| | dθ | 0,36 \| 0,62 | 0,40 \| 0,80 | — |
| | dVT | 0,79 \| 0,63 | 0,72 \| 0,85 | — |
| Aileron (n=6) | p | 0,45 \| 0,35 | 0,45 \| 0,27 | 0,45 \| **0,24** |
| | dφ | 0,83 \| 0,80 | 0,83 \| 0,82 | 0,83 \| 0,78 |
| Leme (n=8) | r | 0,87 \| 0,86 | **0,46 \| 0,43** | 0,46 \| 0,43 |
| | dφ | 0,67 \| 0,73 | 0,88 \| 0,54 | 0,88 \| 0,74 |

Segmentos dos slides (seg 2 / 12 / 20): q 0,19|0,41 → 0,19|0,63; p 0,47|0,20
→ 0,47|0,29 (v1.3); r 0,93|0,92 → **0,36|0,36** (v1.3).

Leitura: (1) o sinal do leme corrigido tira o TIC de r de ~0,9 (espelho) para
~0,45 nos DOIS modelos — o NL da Ana também estava espelhado nos slides;
(2) aileron p do X-Plane v1.3 = 0,24 (< 0,3 de Jategaonkar) — o NL fica em
0,45 (rola ~0,8× e com forma diferente; lateral da Ana nunca validado em voo);
(3) dφ é ruim para todos: deriva lenta de φ em malha aberta (espiral/trim de
rolagem + correções reais replayadas numa planta diferente) — métrica pouco
informativa nesta janela; (4) profundor do X-Plane PIOROU com a entrada
certa (q 0,43 → 0,62), coerente com o pico 0,30: é o déficit real de
autoridade do profundor, mascarado antes pelos 1,67×; só a v1.3
longitudinal (Iyy/CG/estabilizador/corda, PM) recupera — o teste D1 (só Iyy
da Ana) já levaria o pico a 0,67. Melhorias possíveis no látero: diedro/
Cl_β (varredura por dref `acf_dihed1` na asa) para a oscilação de φ com o
dutch roll, e braço/área da deriva para ζ_DR — ganhos esperados de
0,05–0,15 no TIC de r/dφ, não mais.

---

# 2026-09-08 (tarde) — Calibração LONGITUDINAL por dref: candidata v1.3-long (NÃO gravada no .acf)

Régua: o mesmo ajuste 2ª ordem q/δe (`vr_fit_q_de`, 1,5–4,5 s) aplicado ao
REAL e ao modelo NL da Ana (trim em V0 + replay das 4 entradas). Nos 5
doublets de profundor a Ana reproduz o real em pico de q (0,75–0,95) e Δθ
(±10 %); sob o mesmo fit, real ωn 6,4–9,0 / ζ 0,5–1,6 (fit ruidoso), Ana ωn
7,3–9,8 / ζ 0,7–1,0. Trim real de profundor: −0,3° médio, inclinação
**−0,45°/(m/s)** entre 15,5 e 19,8 m/s (8 segmentos).

## Confundidor descoberto: sopro da hélice no instante do doublet

O teleporte derruba o RPM (spool ~20 s) e o trim automático convergia em
13–19 s ⇒ o doublet caía com o motor em estados diferentes. Medido com o
empuxo (`POINT_thrust`) no fim do trim: mesma configuração dá pico de q 0,90
com 2,4 N e 0,41 em molinete (−1,5 N). Correção no `VR_replay_xp`:
`VR_trim_min` (≥ 25 s para calibração), `VR_trim_max`, e `R.thrust0/trq0`
gravados. Trims longos (70 s) esgotam o motor (0,49 N) — limite prático
25–45 s. Vale também para o leme (dispersão ±30 % de 09-08 manhã).

## Varredura (seg 2, V0 18,8 m/s; Iyy da Ana = Jxx_unitmass 0,052 m² = R 0,75 ft)

| Config | CG (m, +atrás) | elev crat | h-stab | empuxo N | 1º lobo q XP/real | 2º lobo | Δθ XP/real | ωn / ζ (fit) | fit % | de0 |
|---|---|---|---|---|---|---|---|---|---|---|
| v1.2 (Iyy 0,446, crat 0,20) | 0 | 0,20 | 1,0 | — | 0,38 | 0,27 | 0,33 | 7,6 / 0,58 | 79 | −8,0° |
| D1: Iyy Ana | 0 | 0,20 | 1,0 | — | — | 0,67 | 0,42 | 13,4 / 0,59 | 84 | −8,0° |
| D2: D1 + CG +4 cm | 0,04 | 0,20 | 1,0 | — | — | 0,68 | 0,47 | 10,5 / 0,67 | 87 | −11° |
| D3/D3b: CG +6 + crat 0,5 | 0,06 | 0,50 | 1,0 | 2,47 | 0,67 | 0,90 | 0,65 | 8,7 / 0,81 | 86 | −9,3° |
| D6: CG +9 + crat 0,6 (trim falhou, molinete) | 0,09 | 0,60 | 1,0 | −1,50 | — | 0,41 | 0,61 | 6,8 / 0,95 | 89 | −12° |
| D7: D3 + servo 0,15 s | 0,06 | 0,60 | 1,0 | 2,43 | — | 0,89 | 0,67 | 9,5 / 0,49 | 78 | −8,7° |
| **D8: CG +7 + crat 0,8 + h-stab ×1,15** | **0,07** | **0,80** | **×1,15** | 2,42 | **0,73** | **1,11** | **0,71** | **8,1 / 0,86** | 83 | −8,6° |
| D9: D8 com CG +9 (trim 70 s, falhou) | 0,09 | 0,80 | ×1,15 | 0,49 | 0,75 | 0,93 | 0,74 | 5,1 / 1,22 | 87 | −10° |
| **D8 @ seg 16 (15,5 m/s)** | 0,07 | 0,80 | ×1,15 | 3,10 | **0,81** | **0,96** | 0,67 | 10,5 / 0,72 | 90 | −6,7° |

Arquivos: D1 `VR_replay_20260908_111001`, D2 `_111245`, D3 `_135723` (segs 2 e
16), D4 `_140806`, D5 `_141007`, D3b `_142138`, D6 `_142234`, D7 `_142348`,
D8 `_144108` (seg 2) e `_144923` (seg 16), D9 `_144514`; figura das formas
`voos/VR_long_D3b_seg2_formas.png` (real × Ana × v1.2 × D3b).

Leitura: (1) só a inércia física (Iyy 0,116 em vez de 0,446) leva o pico de q
de 0,27 a 0,67 e expõe a rigidez estática excessiva (ωn 13,4; trim −8° a
18,8 m/s onde o real voa neutro); (2) CG para trás corrige ωn e a inclinação
do trim (−2,3 → −0,6°/(m/s); real −0,45) — +9 cm seria ainda melhor (ζ 1,2,
lobos mais simétricos) mas o trim automático não segura; (3) a assimetria de
lobos (1º = nariz para cima 0,73–0,81; 2º 0,96–1,11) é rigidez restauradora
Cmα ainda alta frente a Cm_δe — o real tem razão 2º/1º 1,4, a Ana 1,2, o
X-Plane 1,8–2,1; (4) razão de corda do profundor e corda do estabilizador são
honradas por dref; servo de 0,15 s não ajuda (descartado); (5) o modelo NL da
Ana, sob a mesma régua, é uma boa referência longitudinal.

## Candidata v1.3-long = D8 — receita para o Plane Maker (a aplicar com o Kaue)

1. W&B: raio de giração de **pitch 1,47 → 0,75 ft** (Iyy 0,116 kg·m²); yaw
   1,12 e roll 0,84 ft (v1.3-lat) mantidos.
2. W&B: **CG +7 cm para trás** (= +0,23 ft; o "CG default" de +0,07 ft vai a
   ≈ +0,30 ft e o limite traseiro, hoje 0,20 ft, precisa subir para ≥ 0,35).
3. Controls: **razão de corda do profundor 0,20 → 0,80**.
4. Wings (h-stab, 2 partes): corda raiz e ponta **0,31 → 0,357 ft** (×1,15).
5. **Incidência do h-stab** (não há dref): o trim de profundor da candidata
   fica em −8,6° (18,8 m/s) / −6,7° (15,5) onde o real usa ≈ −0,3°. Para
   deslocar o trim +7…+8° de profundor com τ≈0,9, a incidência deve ir de −6°
   para **≈ 0 a +1°** — iterar no PM até o trim a 18,8 m/s ficar entre −1 e
   +1° (conferir com `VR_replay_xp`, segs 2/16, de0 impresso em graus) e
   re-conferir o α de trim a 12 m/s (alvo 14,4°, pode exigir retoque de
   perfil/incidência da asa).
6. Depois: `XP_sync_acf_clones`, nova assinatura em `xp_reload_acf`
   (J 0,052/0,1165/0,0656), campanha completa dos 22 segmentos, e
   **recalibrar as âncoras de missão** (`XP_missao`/`XP_voo`: Xe(8),
   TrimInput, pitch0, thr0, clamp de θ_ref) — mudam com o trim novo — e
   refazer G4/G5 e os voos LQRy v3 no X-Plane.

Esperado após a incidência: pico q ≈ 1 (2º lobo) / 0,75–0,8 (1º lobo), Δθ
≈ 0,7–0,8 do real, ωn 8–10 / ζ 0,7–0,9, TIC de q ≈ 0,3–0,4 (v1.2: 0,62).
Resíduo honesto: 1º lobo 20–25 % fraco — CG mais atrás resolveria, exige
trim automático melhor (ou trim manual) para ser validado.

### v2 candidato GRAVADO no PM e testado (2026-09-08 15:20–15:40) — revertido para v1.3-lat

Kaue confirmou com o Alysson: **cursos ±15° nas três superfícies** (vale para
dez/24) ⇒ escala 15/25 e déficit do profundor confirmados. Gravou no PM
(`acf_backups_20260902/DH-Lon-REV-03_gemeo_v2_cand_inc0_20260908.acf`, md5
12e6c606): raios 0,75/1,12/0,84 ft, CG default 0,30 ft (aft 0,40), profundor
crat 0,50 (máximo do PM 9) e cursos **15/15** nas três superfícies, h-stab
0,37 ft, **incidência do h-stab −6° → 0°**. Conferido por dref (tudo OK,
`xp_reload_acf` reconhece "gemeo v2 candidato, cursos 15").

Resultado (segs 2 e 16, escala 1:1): **trim de profundor −1,3° / +0,1°
(real −0,3), inclinação −0,4°/(m/s) (real −0,45) — o trim fechou com a
incidência 0°.** Mas a dinâmica ficou QUENTE e assimétrica: 1º lobo (nariz
p/ cima) 2,25× / 1,96× o real, 2º lobo 1,40 / 0,86, Δθ 36° vs 18°, ωn 17,5
(seg 2). Testes por dref sobre esse arquivo (seg 2): h-stab 0,31 ft → igual
(2,23/1,22); h-stab ×0,5 → 1,74/0,82, ωn 3 (corda É honrada; não é tamanho);
profundor crat 0,25 → 1,93/1,29 (efeito fraco). Em todos, o trim automático
terminou com **5,3 N de empuxo** (manete 0,55 ainda subindo, V 18,3 < 18,8)
contra 2,4 N nas configurações da tarde — o doublet cai com o dobro de sopro.
Leitura: (a) a calibração D3–D10 foi feita com a cauda a −6° + profundor −10°
(regime não linear do XP9) e não transporta para a cauda a 0°; (b) a
assimetria nariz-cima/nariz-baixo com a cauda a 0° e o empuxo variável do
autotrim impedem calibrar o profundor por picos sem um trim com empuxo
reprodutível. **Decisão: .acf ativo revertido para v1.3-lat** (validado e
coerente com as âncoras de missão); v2 candidato preservado no backup.

Próximos passos (sessão futura, ~1 tarde): (1) trim com manete FIXA por
velocidade (tabela nível de voo da aeronave) e θ_ref←V, ou critério de
convergência com manete estável por ≥4 s — empuxo reprodutível no doublet;
(2) recalibrar profundor/estabilizador/CG por dref com a cauda a 0° (regime
linear), alvos 1º lobo ≈1, 2º ≈1, ωn 8–10, trim ±1°; (3) gravar no PM,
validar 22 segmentos, recalibrar âncoras, refazer G4/G5/missões/LQRy v3.
Arquivos: `VR_replay_20260908_152016` (v2, segs 2/16), `_152740` (h-stab
0,31), `_153237` (h-stab ×0,5), `_153716` (crat 0,25).

### 15:45–16:15 — trim com manete fixa e série E (cauda a 0°, cursos 15): candidata v2 = E6

`VR_trim_modo = 'thr_fixo'` (`VR_thr_fixo` 0,42; θ_ref persegue V0, γ livre; o
modo 'classico' manete←ḣ divergiu): **empuxo reprodutível** — 2 corridas
iguais (3,02/3,03 N, lobos 1,95/1,96). Achados por dref sobre o v2 gravado
(seg 2, 18,8 m/s, 3,0 N): corda do h-stab **NÃO é honrada** por dref (×0,5 =
igual; o "D13" da tarde era efeito do empuxo) → só PM; profundor crat **é**
honrado (0,5→0,2: 1º lobo 1,95→1,58, 2º 1,2→1,05); CG à frente simetriza os
lobos (−5 cm: 1,98→1,59 / 1,16→1,39) e sobe o trim (−1,6°→+1,7°).

| Config (Iyy Ana, h-stab 0,37 ft, inc 0°, cursos 15, 3 N) | 1º lobo | 2º lobo | Δθ | trim δe | seg |
|---|---|---|---|---|---|
| v2 gravado (crat 0,5, CG 0,30 ft) | 1,95 | 1,13–1,27 | 1,7–1,85 | −1,6° | 2 |
| E3: crat 0,20 | 1,58 | 1,05 | 1,32 | −2,1° | 2 |
| E4: crat 0,20 + CG 0,14 ft | 1,23 | 1,11 | 1,09 | +1,8° | 2 |
| E5: crat 0,10 + CG 0,14 ft | 1,11 / 1,07 / **0,79** | 1,03 / 1,11 / 1,01 | 0,99 / 1,05 / 0,71 | +1,9 / +2,9 / +5,0° | 2 / 1 / 16 |
| **E6: crat 0,10 + CG 0,20 ft** | **1,31 / 1,01** | **1,12 / 1,03** | **1,12 / 0,86** | **+0,3 / +2,8°** | 2 / 16 |

Figura `voos/VR_long_E5_seg2_formas.png` (real × Ana × v1.2 × E5: q dos três
sobrepostos, pico 101/106/112 °/s no mesmo instante). **Escolha: E6** (melhor
compromisso entre velocidades e trim). Receita PM (3 campos sobre o v2
gravado): profundor crat **0,50 → 0,10**; CG default **0,30 → 0,20 ft**;
incidência do h-stab **0 → −0,5°** (centrar o trim: alvo −1…+1° em 15–19 m/s).
Depois: validação 22 segmentos com `thr_fixo`, âncoras, G4/G5, LQRy v3.
Arquivos E: `_155113` (2 runs), `_155822` (E1), `_160100` (E2), `_160334`
(E3), `_160606` (E4), `_160803`/`_161046`/`_161142` (E5 segs 2/16/1),
`_161258`/`_161354` (E6 segs 2/16).

### 16:20–16:35 — v2 GRAVADO e fechado (backup `acf_backups_20260902/DH-Lon-REV-03_gemeo_v2_20260908.acf`)

Iterações no PM (segs 2/16, manete fixa 0,42, ~3 N): inc −0,5° + crat 0,10:
lobos 0,93/0,77 e 0,76/0,80, trim −0,9/+2,4°; inc −0,2° + crat 0,10: 0,97/0,80 e
0,72/0,68, trim −0,2/+2,9 (a incidência não era a causa da queda vs E6);
**inc −0,2° + crat 0,20: 1,24/1,07 (18,8 m/s) e 1,02/0,90 (15,5), Δθ 1,08/0,88,
trim +0,1/+2,6° — ACEITO como v2.** Achado: o crat gravado no PM vale menos que
o mesmo crat por dref (PM 0,20 ≈ dref 0,10) — o dref muda só o fator de
eficácia, o PM reposiciona a charneira; calibrar por dref e gravar no PM
exige re-conferir. **v2 = v1.3-lat + Iyy da Ana (R_pitch 0,75 ft) + CG 0,20 ft
(era 0,07) + h-stab corda 0,37 ft e incidência −0,2° (era 0,31 ft / −6°) +
profundor crat 0,20 (era 0,20… com curso 15 em vez de 25) + cursos ±15° nas
três superfícies.** Assinatura: J 0,052/0,1165/0,0656, m 2,22, elev 15.
Segue a validação completa dos 22 segmentos com `thr_fixo`.

### 16:35–17:30 — VALIDAÇÃO do v2 nos 22 segmentos (`voos/VR_replay_GEMEO_V2_MERGED.mat`, trim `thr_fixo` 0,42, empuxo 2,9–3,8 N em todos)

Blocos `VR_replay_20260908_{163615,164006,164353,164740,165126,1657xx}`; figs
`VR_comp_*_V2.png`, `VR_comp_resumo_V2.png`, `VR_lat_v2_vs_v12.png`.

| Eixo (n) | Métrica | v1.2 corrigido (manhã) | **v2** | real |
|---|---|---|---|---|
| Profundor (7, sem o seg 18) | pico q XP/real | 0,24–0,41 (média 0,30) | **0,86–1,09** (1,00 mediana) | 1 |
| | TIC q (mediana, NL \| XP) | 0,19 \| 0,62 | 0,19 \| **0,21** | |
| | fit q | 15–30 % | 42–73 % | |
| | trim δe (15,5 / 18,8 m/s) | −0,2 / −8,0° | +2,6 / +0,1° | +0,2 / −0,3° |
| Leme (8) | pico r (mediana) | 0,71 | **1,08** [0,73–1,30] | 1 |
| | TIC r (NL \| XP) | 0,46 \| 0,43 | 0,46 \| **0,35** | |
| Aileron (6) | pico p (mediana) | 0,84 (trims c/ motor ~idle) | **1,7** [1,29–1,92] | 1 |
| | TIC p (NL \| XP) | 0,45 \| 0,27 | 0,45 \| 0,26 | |

Seg 18 (12,8 m/s, doublet fraco, TIC 0,68 no NL e no XP) é inutilizável.
**Achado**: a calibração lateral da manhã (v1.3-lat, aileron 0,45) foi feita
com trims de motor quase em marcha lenta (thr0 0,01–0,10, descendo); com
empuxo de nível (3,3 N) o aileron do v2 é 1,7× quente. Varredura por dref
(segs 10/12, 3,3 N): crat 0,45 → 1,71/1,29; 0,25 → 1,49/1,07; 0,15 →
1,40/0,94 (efeito fraco; no PM vale ainda menos). Botão eficaz = envergadura
do aileron (3 → 2 elementos externos) + crat 0,25 — pedido ao Kaue 17:30.

**Mapa de trim do v2** (22 trims com manete fixa 0,42, ḣ ≈ 0…+0,7 m/s ⇒ manete de
nível ≈ 0,40): θ_trim 15,1° @ 12 m/s / 9,1° @ 15 / 4,4° @ 18; δe_trim +4,9° @ 12 /
+2,6 @ 15 / +0,3 @ 18 (inclinação −0,77°/(m/s); real −0,45, nível ≈ −0,3°). As
âncoras de missão do v1.x (pitch0 ≈ 14°, TrimInput manete 0,45 / δe +5,5° a
12 m/s) ficam praticamente válidas — conferir com G5 dblpsi/dblh e OVAL no v2.

### 17:35–18:15 — aileron corrigido no PM e TABELA FINAL do gêmeo v2 (22 segmentos)

PM (Kaue): aileron 1 em **2 elementos externos** (era 3) e **crat 0,25** (era
0,45). Arquivo final `acf_backups_20260902/DH-Lon-REV-03_gemeo_v2_final_20260908.acf`
(md5 63e7783a; = ativo = clones). Segs 9–14 revoados (`_180838`, `_181139`);
`VR_replay_GEMEO_V2_MERGED.mat` atualizado; figs `VR_comp_*_V2.png`,
`VR_comp_resumo_V2.png`, `VR_lat_v2_vs_v12.png`; TIC em `VR_tic_20260908.mat`.

**Gêmeo v2 = v1.3-lat + Iyy da Ana (R_pitch 0,75 ft) + CG 0,20 ft + h-stab
0,37 ft / −0,2° + profundor crat 0,20 + aileron crat 0,25 em 2 elementos +
cursos ±15° nas três superfícies** (yaw 1,12 / roll 0,84 ft, leme 0,30 mantidos).

| Eixo (n) | Métrica | v1.2 corrigido | **v2** | real / Ana |
|---|---|---|---|---|
| Profundor (7) | pico q XP/real | 0,30 | **1,02** [0,86–1,09] | 1 |
| | Δθ XP/real | 0,33 | **1,09** | 1 |
| | TIC q (XP) | 0,62 | **0,21** | Ana 0,19 |
| | fit 2ª ordem q/δe | 79 % | 89–92 % | |
| | trim δe 15,5 / 18,8 m/s | −0,2 / −8,0° | +2,7 / +0,1° | +0,2 / −0,3° |
| Aileron (6) | pico p (mediana [faixa]) | 0,84 | **0,96** [0,72–1,15] | 1 |
| | TIC p (XP) | 0,27 | 0,26 | Ana 0,45 |
| Leme (8) | pico r (mediana [faixa]) | 0,71 | **1,08** [0,73–1,30] | 1 |
| | TIC r (XP) | 0,43 | **0,35** | Ana 0,46 |
| | ζ_DR | 0,22 | 0,24 | 0,14 |
| todos | razão de pico média (VR_plot_comp) | 0,60 | **0,98** | 1 |

Resíduos declarados: dutch roll ainda mais amortecido que o real (ζ 0,24 vs
0,14; knob = braço/área da deriva no PM, não testado); Δφ em malha aberta
(espiral + correções reais replayadas) segue pouco informativo; inclinação
do trim −0,8°/(m/s) vs −0,45 real; seg 18 (12,8 m/s) inutilizável. Protocolo
de comparação: manete fixa 0,42 (≈ nível), trim ≥ 25 s, reload por segmento.

### 18:30 — malha fechada no v2 com as ÂNCORAS ANTIGAS (PID 100 % da dissertação, `XP_auto_reload`)

G5 dblpsi (`voos/XP_voo_20260908_183205.mat`, `G5_dblpsi_SILxXP_v2.png`):
OS 3,4 / 1,3 / 4,2 %, ts5 % 34,3 / 35,0 / 30,0 s, ess −1,2 / +1,8 / −0,9°
(SIL −1,7 / −1,6 / −2,4 %, 24 s), RMS ψ 8,85° (SIL 8,39), h 599,5–601,6 m,
VT 11,0–13,1, φ máx 6,1°, θ 8,5–16,9°, δe +3…+9°. ⇒ o PID engata e fecha no
v2 sem recalibrar âncoras (θ_trim 15° / δe +4,9° a 12 m/s ≈ v1.x).

G5 dblh (`voos/XP_voo_20260908_183438.mat`, `G5_dblh_SILxXP_v2.png`): OS −4,0 /
−4,7 / −7,9 % (SIL −5,1 / −5,0 / −7,8), ts5 % 33,4 / 33,9 / 30,0 s (SIL 35 / 35 /
30), ess −0,65 / +1,27 / −1,07 m (SIL −0,66 / +1,29 / −1,01), **RMS h 6,94 m
(SIL 6,95)** — a malha de altitude do PID no v2 é indistinguível do SIL, como
no v1.1/v1.2 (a malha compensa a autoridade do profundor; agora a autoridade
também é a real).

## SÍNTESE DO DIA (2026-09-08) — gêmeo v2 é o modelo de trabalho

- **Pipeline de validação corrigido**: sinal do leme (+1), escala de curso
  (o .acf agora tem ±15° = drone), trim com manete fixa (empuxo reprodutível),
  reload por handle, salvamento incremental, `VR_drefs`, `VR_tic`,
  `VR_long_metrics`, `vr_fit_q_de`, `VR_analise_lat`, `VR_merge_campanhas`.
- **Gêmeo v2** (`_gemeo_v2_final_20260908.acf`, md5 63e7783a): 22 doublets
  reais em malha aberta — profundor 1,02 / TIC 0,21, aileron 0,96 / 0,26,
  leme 1,08 / 0,35; malha fechada PID sem recalibrar âncoras — dblpsi OS ≤ 4 %,
  dblh igual ao SIL.
- **Pendências**: missões OVAL/AGRESSIVO (PID) e LQRy v3 no v2 (GUI, ~2 min
  cada); regerar slides/TIC de 01/09 (leme espelhado); ζ_DR (deriva no PM);
  commit do repositório; logs ArduPilot brutos de volta à raiz.
