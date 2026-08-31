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
   **"Carregar G2"** = quadrado de 500 m oficial).
2. **Recarregar o avião no X-Plane** (`File → Open Aircraft`) —
   falar da bateria do motor elétrico do XP9 (~90–150 s por reload;
   por isso missões ≤ ~130 s ou o final vira planeio guiado).
3. **VOAR NO X-PLANE** → confirmar o diálogo. Deixar o X-Plane
   visível num canto: o avião teleporta, engata e voa a missão
   sozinho, em tempo real (1:1).
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
