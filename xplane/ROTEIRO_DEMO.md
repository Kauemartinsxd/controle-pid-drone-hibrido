# Roteiro de demonstração — guiagem por waypoints do DH no X-Plane

## O que dizer sobre a origem (créditos)

- **Guiagem**: algoritmo LOS de waypoints do trabalho do Julio
  (PIPER-1-6) — linha de visada ao WP atual, círculo de aceitação,
  sequenciamento, hold no último WP. Reimplementado como chart
  (`Guidance_Star`) e adaptado ao DH: referências no referencial da
  proa de engate, interpolação de Δh>20 m, alt/vel por waypoint,
  deltas entrando antes dos pré-filtros da cascata.
- **Controle**: 100% a cascata PID da dissertação (ganhos intocados).
  A guiagem só fabrica referências.
- **Planta**: X-Plane 9 com o .acf "gêmeo v1.1" — calibrado para
  equivaler ao modelo identificado da Ana (trim α 14,50 vs 14,44;
  CLα idêntico; ωn arfagem 6,25 vs 6,3). Ver EQUIVALENCIA_ACF.md.
- **Interface**: XPlaneConnect UDP (padrão do harness do Julio),
  20 Hz, tempo do X-Plane como referência.

## Antes da demo (5 min)

1. Abrir o X-Plane 9 → `File → Open Aircraft` → `DH-Lon-REV-03`
   (Radio Control). Conferir que o avião aparece NA PISTA.
2. Abrir o MATLAB (o path já sobe pelo startup normal; se precisar:
   `addpath('...\controle-pid-drone-hibrido\xplane')`).
3. `XP_gui_waypoints` → abre a GUI de missão.

## Demo ao vivo (o prato principal, ~6 min)

1. Na GUI: mostrar o mapa, clicar 3–4 waypoints (ou botão
   **"Carregar circuito OVAL"** = estádio de 6 WPs (retas 160 m + pontas
   de raio 100, R_accept 60), ~126 s — dimensionado em 2026-08-31 para
   caber nos ~130 s de motor por reload e validado no SIL com 6/6
   capturas; o quadrado histórico de 500 m das campanhas de agosto dava
   265 s e terminava em planeio guiado).
2. **VOAR NO X-PLANE** → confirmar o diálogo. O avião é RECARREGADO
   automaticamente (xp_reload_acf: menu por UDP + cliques
   programáticos — mencionar a bateria de ~90–150 s do motor elétrico
   do XP9 e que a automação substituiu o File → Open Aircraft manual;
   por isso missões ≤ ~130 s ou o final vira planeio guiado). Deixar o
   X-Plane visível: o avião teleporta, engata e voa a missão sozinho,
   em tempo real (1:1).
4. Ao terminar: trajetória azul sobreposta ao plano na GUI,
   resumo de capturas no console, figuras + PNGs salvos em
   `xplane/voos/`.

Se a professora quiser ver de novo: reload + VOAR (1 clique cada).

## Figuras de apoio (já prontas em xplane/voos)

| Figura | História que conta |
|---|---|
| `XP_missao_20260829_225253_G2_traj.png` | Missão G2 do PID: 4/4 capturas, fase nominal |
| `G4_PID_SILxXP_manobras.png` | **G4**: mesmas manobras no modelo NL e no X-Plane — malha de altitude com métricas idênticas (OS −5,1 vs −5,3%; ts 35,0 vs 35,0 s) |
| `G4_LQRY_SILxXP_voo_reto.png` | Contraste LQRY: SIL liso × X-Plane departure em 12,5 s |
| `EQUIV_antes_depois.png` | **A prova do gêmeo em 1 slide**: erro de cada grandeza vs modelo da Ana, ANTES (cinza) × DEPOIS (verde) do Plane Maker — α 65%→0,4%, CLα 60%→0,3%, ωn 56%→0,8% |
| `EQUIV_doublet_malha_aberta.png` | **Mesmo doublet em malha aberta, CI nula**: G_q da Ana × G_q identificada em voo — mesma forma e timing (ωn 6,4×6,3; ζ 0,55×0,61); amplitude −29% = resíduo de eficácia do profundor. (Malha aberta crua não vale: DH instável, trajetórias divergem — por isso via identificação.) |
| Tabelas do `EQUIVALENCIA_ACF.md` | Os números completos por trás da figura (trim/polar/curto período) |
| `LQRY_XPLANE.md` | O arco completo do LQRY (6 adendos): por que 2/4 é o teto |

## Mensagens-chave (se perguntarem)

- PID: 4/4 na missão com os ganhos da dissertação, sem retune; e
  reproduz o SIL com fidelidade de métrica (G4).
- LQRY: funciona como regulador de pequenos sinais na planta de
  projeto (reproduzimos as demos do Mirko); não é implantável para a
  missão — envelope de proa ±15°, margem de atraso <25 ms (o laço
  real tem 50–75 ms), sem anti-windup. Falha até em voo reto no
  X-Plane. A fronteira foi MEDIDA, não especulada.
- O gêmeo transforma o X-Plane em bancada de ensaio válida: mesma
  manobra, mesmos ganhos, métricas casadas.

## Quais scripts rodam (na ordem, ao apertar VOAR)

```
XP_gui_waypoints.m          GUI: coleta WPs -> variaveis XP_* -> chama o lancador
  └─ XP_missao.m            lancador: pre-flight, WPs->NE, ancoras, teleporte, sim, salva
       ├─ XP_inicializacao.m   ambiente XP: socket UDP + ...
       │    └─ DH_inicializacao.m   TRIM da Ana + GANHOS do PID (o MESMO script do SIL)
       ├─ modelo_XP_DH_GUIA.slx    controlador PID + Guidance_Star (LOS) + Planta_XP
       │    ├─ xp_read_dh.m        sensor: 14 canais UDP + teleporte-engate + pacing 1:1
       │    ├─ xp_send_dh.m        atuador: [thr de da dr] c/ limites reais do .acf
       │    └─ XPlaneConnect (lib do Julio)   getDREFs/sendCTRL/sendPOSI
       └─ plot_XP_missao.m        trajetoria + series + PNGs
```

Espelho SIL (G4): `DH_inicializacao` → `manobras/manobra_*.m` →
`sim('modelo_NL_DH_CL')` (planta = equacoes da Ana) → `plot_NL_DH`.
**Entre SIL e X-Plane so a caixa "planta" muda** — controlador e
ganhos identicos por construcao (mesmo DH_inicializacao).

Variantes: `XP_voo.m` (voo sem guiagem + degraus G4 de h/ψ/VT);
`XP_missao_lqry2.m` (LQRY do Mirko: paths isolados, Ganho_hold_*.mat,
modelo_XP_LQRY2_GUIA.slx — mesma Planta_XP, controlador trocado);
`XP_ident_theta.m` (sonda de identificacao usada p/ calibrar o gemeo).

## Plano B — missão SEM X-Plane (se o simulador não colaborar)

```matlab
NL_missao        % G2 na planta da Ana (modelo_NL_DH_GUIA) — roda em segundos
```

Mesma guiagem, mesmo controlador, mesmos plots (`plot_XP_missao`) —
só a planta muda (sfunction_DH em vez do X-Plane). Validado: 4/4 com
capturas a 8–17 m. Aceita `NL_WPs`/`NL_R_accept`/`NL_TimeXP` como o
XP_missao. Também serve de comparação didática ao vivo: "a mesma
missão nos dois mundos".

## Se algo der errado

- **Avião não responde / hélice parada**: `File → Open Aircraft` de
  novo (latch do motor elétrico do XP9). RPM no solo não vale como
  teste (hélice fica enterrada no spawn) — olhar a hélice girando.
- **"tempo do X-Plane CONGELADO"**: tela de crash aberta → Reset
  Flight no X-Plane e rodar de novo.
- **Voo virou planeio no meio**: bateria acabou (normal >130 s) —
  reload e missão mais curta.
- Erro vermelho na GUI: ler o console do MATLAB (mensagem completa).

## Adendo 2026-09-01 — circuito AGRESSIVO (botão novo na GUI)

Segundo botão ao lado do OVAL: **"Circuito AGRESSIVO (6 WPs)"** = quadrado de
160 m com curvas de 90°, degraus de altitude de ±20 m e velocidade alternando
12 ↔ 15 m/s entre waypoints (1,5 voltas, 960 m, R_accept 70, teto 120 s; o voo
termina ~93 s). Lançador por script: `XP_missao_agressiva.m`. Serve para
mostrar onde o X-Plane e o modelo NL DIFEREM (o oval, manso, quase não os
separa): voo de 20:14 → 6/6 nos dois mundos, mas X-Plane h 576..614 m e
θ até 30° contra 600..616 m e 18° no NL; V_T 11,1..15,5 contra 12,0..15,0.
Lição da 1ª tentativa (legs 140 m / R 40): o raio de curva a 15 m/s (~70 m)
passa 55–65 m do canto → WP2 perdido nos dois mundos, e no X-Plane o PID cai
na armadilha de energia (V_T > ref ⇒ throttle 0; h < ref ⇒ δe no batente;
sink ~2 m/s) — usar R_accept ≥ 60 com legs ≥ 160 m.

## Adendo 2026-09-01 (noite) — missões da apresentação a 15 m/s

Decisão do Kaue: apresentar e voar a 15 m/s (folga de α: 10° contra 4° a 12 m/s).
Engate continua a 12 m/s (trim do controlador e do modelo NL); a 1ª perna acelera.
- **OVAL (botão da GUI)** agora = geometria ×1,6: retas 256 m + pontas R160,
  R_accept 100, 1417 m, teto 130 s. X-Plane 6/6 em 95 s (h 595..603, V_T 11,9..15,8,
  φ máx 13,8°), NL 6/6 (h 600..601,5). A ×1,3 com R 80 o X-Plane perdeu o WP3
  por 86 m (o NL passa a 71 m): o X-Plane curva ~10–15 m mais largo que o NL.
- **AGRESSIVO (botão da GUI, `XP_missao_agressiva.m`)** v2 = quadrado 260 m,
  V 18 nas subidas e 15 nas descidas, h 600/620, R_accept 110, 1 volta. X-Plane
  4/4 em 74 s (h 595,5..617, θ 27,6°, α 17,8°), NL 4/4 (h 600..617,5, α 14,5°).
- α máximo dos voos a 15 m/s ocorre no ENGATE (t≈4–5 s, acelerando de 12 com
  θ ~27°): 19,3° no oval e 17,8° no agressivo — transiente breve; em cruzeiro
  a folga é grande. Motor aguentou 120 s a manete média 0,70.
Voos: `XP_missao_20260901_210753_OVAL15b`, `XP_missao_20260901_205852_AGR15`
(+ `NL_missao_*_autoNL` correspondentes e `*_compNL.png`).
