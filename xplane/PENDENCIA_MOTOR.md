# Motor elétrico do DH no X-Plane 9 — diagnóstico e operação (2026-08-29)

## RESOLVIDO (2026-08-31): reload AUTOMÁTICO — `xp_reload_acf.m`

A pendência da endurance está encerrada por AUTOMAÇÃO do reload (a
"bateria infinita" verdadeira não existe no XP9 — ver investigação):

**Investigação (3 sondas em voo, thr 1.0, ~200 s cada):**
1. NÃO é combustível: manter `m_fuel1` cheio por dref (0,03–0,05 kg o
   voo todo; o write pega mas clampa no tanque residual) → motor morre
   igual (TRQ 0,27→0 em ~170 s). `XP_sonda_bateria2.mat`.
2. NÃO é a bateria elétrica do avião: tensão parada em ~24,0 V o voo
   inteiro (−0,03 V), e injetar GPU+APU+gerador (gpu_on/gpu_amps etc.)
   não muda NADA no decaimento. `XP_sonda_bateria3.mat`. (O dref
   `battery_charge_watt_hr` nem existe no XP9 — explica o watt-hours
   do Plane Maker não ter efeito.)
3. O estoque é INTERNO do modelo de motor elétrico do XP9, não exposto
   por dref; a potência decai ~linearmente até 0 em ~90–180 s. Só o
   File → Open Aircraft rearma.
4. UDP legado (porta 49000): `ACFN` (não "VEHN" — doc do XP9 está
   desatualizada) precisa de corpo de 156 bytes (int p + path[150] +
   2 pad) e é ACEITO, mas troca o modelo SEM reiniciar o voo e SEM
   rearmar a energia. `MOUS`/`CHAR` não alcançam os diálogos da UI
   (são do painel). `MENU "600"` FUNCIONA: abre o Open Aircraft
   (números de menu em Resources/menus/English/X-Plane.txt).

**Solução (`xp_reload_acf.m`):** MENU 600 por UDP + 2 cliques de mouse
REAIS (java.awt.Robot) no diálogo — linha `DH-Lon-REV-03.acf` e botão
Open, coordenadas relativas ao centro do client (diálogo centrado,
tamanho fixo; client rect via user32/PowerShell) — e verificação:
t_xp resetou + avião no solo + **TRQ > +0,05 com throttle** (motor
VIVO). Integrado como **OPT-IN** (`XP_auto_reload=true` antes do
lançador — decisão do Kaue 2026-08-31: o padrão continua o reload
MANUAL; a automação serve p/ campanhas sem operador). Requisito: janela do
X-Plane visível; se um arquivo novo ordenar antes de `DH-Lon-REV-03.acf`
na pasta, o clique erra a linha — a verificação acusa.

---

> Substitui o diagnóstico de 2026-08-20 (que culpava hélice sobre-passo e
> "tanque vazio" — ambos FALSOS). Consolidado após a campanha de
> 2026-08-29 com o motor caracterizado em voo.

## O que o motor é

- **Elétrico** (`sim/aircraft/prop/acf_en_type = 3`), 1226 W (1,6 hp),
  redline 1445 rad/s (13.800 RPM). **Fuel = 0 é NORMAL** — não escrever
  drefs de combustível.
- Empuxo real com motor vivo: **+10 a +30 N** (12–18 m/s, thr 1.0) —
  ~2× o T_max do modelo matemático (14 N). Consequência: o C_vel
  (projetado p/ 14 N) trabalha em ciclo-limite de throttle
  (0.4↔1.0, período ~3 s). Voa e regula bem mesmo assim.

## As 3 regras de ouro (aprendidas a caro preço)

1. **O estado do motor é um latch frágil.** Ele MORRE (TRQ=0, hélice só
   em windmill; sem falha registrada em `rel_engfai0`) quando:
   (a) a **bateria esgota** — **~150 s** de voo motorizado no config
   atual (assinatura: thr médio subindo continuamente até saturar em
   1.0, aí o empuxo some); (b) crash + auto-reload ("reset on hard
   crash" — já DESLIGADO nos settings em 2026-08-29); (c) writes de
   drefs `sim/aircraft/*` (pmax etc.) com o sim rodando.
2. **Religamento confiável: SÓ `File → Open Aircraft` na UI** (com
   "Start each flight with engines running" marcado (ok)). Via dref é
   loteria: o power-cycle (battery_array_on/ENGN_running 0→1) religou
   UMA vez (hélice em windmill alto) e nas demais TRAVOU a hélice de
   vez (RPM 0 até em voo). NÃO tentar religar por dref.
3. **Medição de empuxo no solo é INVÁLIDA**: o DH estacionado fica com
   AGL −0,5 m (trem enterrado) e o disco da hélice DENTRO do terreno —
   o X-Plane zera as forças das pás (TRQ aparece, hélice não gira,
   POINT_thrust lixo). Medir empuxo SÓ em voo. O teleporte (sendPOSI)
   derruba o RPM para ~60 rad/s, mas com motor vivo o spool volta em
   ~1–2 s (não mata o motor).

## Resultado com o motor vivo (validação do PID)

G2 de 2026-08-29 22:52 (`voos/XP_missao_20260829_225253_G2.*`), ganhos
100% da dissertação: **t=0–150 s NOMINAL** — h cravada em ~595±3 m,
VT ~12±1, 2 capturas, curvas suaves — até a bateria esgotar (thr médio
0.35→1.0 em 150 s); depois planeio (h 595→415), mas ainda **4/4
capturas** (6,8/21,4/16,3/0,0 m), φ máx 19,6°, sync 0,997.

## Edições no Plane Maker (FEITAS em 2026-08-29 com o Kaue)

1. **Power 1,60 → 0,80 hp** (Standard → Engines → Description,
   "maximum allowable power"): FEITO e validado — pmax lido 613 W,
   empuxo em voo +9..+16,5 N (casou com o T_max=14 N do modelo). (ok)
2. **Battery 1.000 → 4.000 watt-hours** (Standard → Systems →
   Electrical → SOURCES): FEITO — **SEM efeito na endurance** (o motor
   elétrico do XP9 NÃO bebe desses watt-hours).
3. **FADEC "keep within RPM limits"** (Engines → Description): FEITO —
   também sem efeito na endurance.

## PENDÊNCIA REMANESCENTE: endurance do motor (~90–150 s por reload)

Fato empírico (4 missões G2 em 2026-08-29): o empuxo-por-throttle DECAI
desde o engate (o C_vel compensa com thr médio crescendo 0,4→1,0 em
~90–150 s) e o motor então APAGA (TRQ=0) até o próximo Open Aircraft.
Durações observadas: 55 / 150 / 90 / 135 s — não correlaciona com
potência, watt-hours nem FADEC. Hipótese NÃO testada (decisão do Kaue,
2026-08-29: não mexer em combustível num motor elétrico): no XP9 o
elétrico debitar energia do FUEL — este .acf tem fuel capacity = 0
(zerada na edição de peso de 2026-08-19; o `asa5` daquela noite já
mostrava a mesma rampa de throttle). Se um dia quiser testar: Plane
Maker → Weight & Balance → fuel total = 1.0 lb, salvar, recarregar,
voar e ver se a rampa some/escala.

Consequência prática: missões com fase motorizada ≤ ~130 s por reload;
depois o DH degrada graciosamente para planeio (a guiagem segue
capturando — 4/4 em TODAS as missões). Entre corridas: File → Open
Aircraft para "recarregar" o motor.
