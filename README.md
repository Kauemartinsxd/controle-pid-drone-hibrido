# Controle PID — Drone Híbrido (DH) modo asa-fixa

Modelo da planta + piloto automático em **PID cascata** para o drone híbrido (DH) voando em modo asa-fixa.

> 📽️ **Apresentação:** [`docs/Apresentacao_alinhamento_artigo.html`](docs/Apresentacao_alinhamento_artigo.html) —
> alinhamento do artigo (PID × LQRy do SIL ao HIL, com estudo de robustez). Há também uma versão
> sem a slide de margens ([`docs/Apresentacao_alinhamento_artigo_sem_margens.html`](docs/Apresentacao_alinhamento_artigo_sem_margens.html)).
> Arquivos HTML auto-contidos: baixe e abra no navegador (o GitHub não renderiza HTML no repositório).

> 📄 **Artigo:** [`arquivos/`](arquivos/) contém o draft IEEE Access
> *"Cascade PID Versus Output-Feedback LQR for the Fixed-Wing Mode of a Hybrid VTOL UAV"*
> (fontes LaTeX, figuras, dados e scripts de simulação). Pendências abertas em
> [`arquivos/PENDENCIAS.md`](arquivos/PENDENCIAS.md).

> ✈️ **Gêmeo digital no X-Plane 9:** o mesmo PID voa o DH no simulador, com o `.acf`
> calibrado para equivaler ao modelo identificado — ver a seção
> [X-Plane](#x-plane-gêmeo-digital-e-missões-por-waypoints-xplane) e o relatório
> [`docs/Relatorio_gemeo_XPlane.pdf`](docs/Relatorio_gemeo_XPlane.pdf).

---

## Início rápido

```matlab
% 1. Inicialize (carrega trim, ganhos e configura os paths sozinho —
%    pode rodar de qualquer Current Folder)
DH_inicializacao

% 2. Escolha uma manobra pronta
manobra_long          % (ou manobra_latero | manobra_agressiva)

% 3. Simule
out = sim('modelo_NL_DH_CL');

% 4. Veja os resultados
plot_NL_DH
```

> A `missao_perfil` executa os passos 2–4 sozinha (perfil completo de missão).
> Para um cenário **próprio** em vez de uma manobra pronta, veja [Excitação manual](#excitação-manual-cenário-próprio).

---

## Pré-requisitos

- MATLAB R2025b ou superior
- Simulink
- Optimization Toolbox (para o `fsolve` do trim)

---

## Estrutura

```
controle-pid-drone-hibrido/
├── DH_inicializacao.m      ★ Setup do workspace + paths (rode TODA sessão)
├── plot_NL_DH.m            ★ Plot dos resultados (após uma sim)
├── README.md
│
├── manobras/               ← cenários prontos (rode DEPOIS do init)
│   ├── manobra_long.m         degrau de altitude +10 m @ t=10 s
│   ├── manobra_latero.m       degrau de heading  +15° @ t=5 s
│   ├── manobra_agressiva.m    subida +30 m + curva 40° simultâneas @ t=5 s
│   └── missao_perfil.m        missão completa (= Caso 4 do LQRY); roda sim+plot sozinha
│
├── nao_linear/             ← simulação NL
│   ├── modelo_NL_DH_CL.slx    closed-loop NL com PID (servo + pré-filtro)
│   ├── modelo_NL_DH_OL.slx    planta NL standalone (open-loop)
│   └── sfunction_DH.m         S-Function da planta NL
│
├── linear/                 ← análise linear
│   ├── MATRIZES_DH.m          A/B/C/D + trim (gerado pelo DH_build_model)
│   ├── modelo_linear_DH_CL.slx
│   └── utilitarios/
│       ├── lin_DH.m                  linearização numérica
│       ├── modelo_linear_DH.m        constrói sys_DH / sys_long / sys_lat
│       ├── Comparation_L_and_NL_Long.slx
│       └── Comparation_L_and_NL_Lat.slx
│
├── utilitarios/            ← planta + ferramentas de build
│   ├── DH_build_model.m       trim + linearização (gera MATRIZES_DH)
│   ├── modelo_DH.m            forças e momentos
│   ├── coef_DH.m              coeficientes aerodinâmicos (coef. Ana)
│   ├── dyn_rigidbody_DH.m     derivadas (flag=1)
│   ├── obs_rigidbody_DH.m     saídas (flag=3)
│   └── trimagem_DH.m          trim via fsolve
│
├── xplane/                 ← gêmeo digital no X-Plane 9 (bancada de ensaio)
│   ├── XP_gui_waypoints.m     ★ GUI de missão (mapa com pan/zoom + rastreio ao vivo)
│   ├── XP_missao.m            missão por waypoints no X-Plane (guiagem LOS)
│   ├── NL_missao.m            a MESMA missão na planta da Ana (sem X-Plane)
│   ├── XP_voo.m               voo reto + degraus/doublets (manobras G4/G5)
│   ├── xp_reload_acf.m        reload programático do .acf (opt-in XP_auto_reload)
│   ├── G5_*.m / plot_G5.m     campanha de doublets SIL×X-Plane
│   ├── modelo_XP_DH_*.slx     PID + guiagem + Planta_XP (UDP) / modelos GUIA
│   ├── voos/                  .mat + PNGs de todos os voos e figuras SIL×XP
│   ├── EQUIVALENCIA_ACF.md    calibração do gêmeo + campanhas G4/G5 (números)
│   ├── PENDENCIA_MOTOR.md     motor elétrico do XP9 (energia ~130 s por reload)
│   └── ROTEIRO_DEMO.md        roteiro da demonstração ao vivo
│
├── arquivos/               ← artigo IEEE Access (PID × LQRy de ganho fixo)
│   ├── main.tex / main.pdf    draft do artigo
│   ├── sections/              seções 01–07 (intro … conclusão)
│   ├── figs/                  figuras do artigo (fundo branco, 300 dpi)
│   ├── sims/                  scripts que geram figuras/tabelas (sims_artigo_A…E,
│   │                          pid_recampanha_v2, acoplamento_v2, fig_v1_white, …)
│   ├── dados_*.txt            métricas exportadas p/ as tabelas LaTeX
│   └── PENDENCIAS.md          pontos abertos do draft (marcados com \pendente{})
│
└── docs/                   ← apresentações HTML auto-contidas (deck de alinhamento)
```

---

## Fluxo de uso

### 1. Build do modelo (uma vez, ou se mudou trim/coeficientes)

```matlab
DH_build_model
```

Faz trim com `fsolve` e linearização. Grava `linear/MATRIZES_DH.m`. Re-rodar sempre que mudar:
- coeficientes em `utilitarios/coef_DH.m`
- velocidade/altitude/gama de trim (`Ve`, `he`, `gammae`)
- parâmetros físicos (massa, S, b, c, inércia) em `utilitarios/modelo_DH.m`

Os coeficientes aerodinâmicos atuais vêm da identificação da **Ana Carolina**: Modelo 4 longitudinal + Modelo 2 látero-direcional.

### 2. Inicialização (toda sessão, é rápido)

```matlab
DH_inicializacao
```

Carrega `linear/MATRIZES_DH.m`, define os ganhos do piloto automático, o modelo de servo/motor, o pré-filtro de referência e **configura os paths** (`manobras/`, `nao_linear/`, `linear/`, `utilitarios/`). Por isso as manobras e o `plot_NL_DH` ficam chamáveis pelo nome independente da pasta atual.

> ⚠️ Por default todas as referências ficam **no trim** — sem manobra, o sistema fica em equilíbrio.

### 3. Manobras prontas

Depois do init, rode uma das manobras (`manobras/`). Cada uma zera as excitações da anterior, então dá pra encadear sem re-inicializar:

| Manobra | O que faz |
|---|---|
| `manobra_long` | degrau de altitude **+10 m** em t=10 s (exercita C_alt → clamp → C_theta/Kq) |
| `manobra_latero` | degrau de heading **+15°** em t=5 s (K_heading → C_phi, com yaw damper coordenando) |
| `manobra_agressiva` | **+30 m e curva de 40°** simultâneas em t=5 s (clamp satura ~10 s; caso mais exigente) |
| `missao_perfil` | **missão completa = Caso 4 do LQRY** (VT→15,2; ψ doublet ±5°; h doublet ±5 m). Roda sim+plot sozinha |

```matlab
DH_inicializacao
manobra_long
out = sim('modelo_NL_DH_CL');
plot_NL_DH
```

### Excitação manual (cenário próprio)

Em vez de uma manobra pronta, sobrescreva as referências **depois** do init e **antes** do `sim(...)`:

```matlab
DH_inicializacao

h_ref         = he + 50;          % step de altitude +50 m
VT_ref        = Ve + 6;           % step de velocidade +6 m/s
psi_ref_final = deg2rad(20);      % vira 20° pra direita
psi_ref_t     = 5;                %   em t=5 s

out = sim('modelo_NL_DH_CL');
plot_NL_DH
```

| Quero excitar... | Variáveis | Default (sem step) |
|---|---|---|
| **Altitude** | `h_ref`, ou `h_step_final` + `h_step_t` | `h_ref = he`, `h_step_final = 0` |
| **Velocidade** | `VT_ref`, ou `VT_step_delta` + `VT_step_t` | `VT_ref = Ve`, `VT_step_delta = 0` |
| **Heading** | `psi_ref_final`, `psi_ref_t` | `0`, `5 s` |
| **Pitch direto** (sem malha de altitude) | `att_alt=1` + `theta_step_*` | `att_alt=0`, steps `0` |
| **Bank direto** (malha de rolamento isolada) | `K_heading=0` + `phi_step_*` | `K_heading=0.1975`, steps `0` |

> Cada canal tem ainda 2 steps extras (`*_t2`/`*_t3`, default `1e9` = inertes) que transformam o degrau em **doublet** (0 → +A → −A → 0), igual ao harness do LQRY.

---

## Configurações em `DH_inicializacao.m`

| Variável | Descrição | Default |
|---|---|---|
| `att_alt` | `0` = altitude (cascata) · `1` = θ-step direto | `0` |
| `h_ref` / `VT_ref` | Referências de altitude / velocidade | `he` / `Ve` (trim) |
| `psi_ref_final`, `psi_ref_t` | Step de heading | `0`, `5 s` |
| `theta_step_*` | Step direto de θ (só age com `att_alt=1`) | `0` |
| `phi_step_*` | Step direto de φ (somado ao caminho do heading) | `0` |
| `*_t2` / `*_t3` | 2º/3º steps p/ doublet (`1e9` = inerte) | `1e9` |
| `tau_ref` | Cte de tempo do pré-filtro de referência | `3 s` |
| `theta_ref_clamp` | Limite de θ_ref (proteção de alpha) | `±10°` |
| `C_theta`, `C_vel`, `C_alt`, `C_phi` | Compensadores PI | ver [Ganhos](#ganhos) |
| `Kq`, `Kp`, `Kr`, `K_heading` | SAS dampers + ganho de heading | ver [Ganhos](#ganhos) |
| `act.*`, `eng.*` | Modelo de servo / motor | ver [Atuadores](#atuadores-servo-e-pré-filtro) |

---

## Arquitetura do controle (cascata PID)

### Longitudinal

```
            Altitude Hold (PI lento)
h_ref ─►(+)─► C_alt ─► θ_ref ─[clamp ±10°]─┐
[h] ────┘                                   │
                                            ▼
            Pitch Attitude Hold (PI) + SAS Kq
θ_ref ─►(+)─► C_theta ─► δe        Velocity Hold (PI)
[θ] ───┘                          VT_ref ─►(+)─► C_vel ─► δT
                                  [VT] ───┘
```

### Latero-direcional

```
            Heading Hold (P)
ψ_ref ─►(+)─► K_heading ─► φ_ref ─┐
[ψ] ───┘                          │
                                  ▼
            Bank Angle Hold (PI, ganhos negativos) + SAS Kr (washout)
φ_ref ─►(+)─► C_phi ─► δa
[φ] ───┘
```

> Os compensadores são **PI puros** (Kd = 0); o amortecimento vem dos **dampers explícitos** (Kq no pitch, Kr sobre washout `s/(s+1)` no yaw). O rolamento já é Nível 1 com folga → `Kp = 0`.

---

## Ganhos

Retuning **2026-08-10** para os requisitos da dissertação (ts < 60 s, OS < 10%, ess ≤ 5%, GM > 10 dB, PM > 45°), sobre o modelo linear (coef. Ana, Ve=12 m/s, he=600 m), fechamento sequencial *inner-first*.

| Malha | Função | Tipo | P | I | N |
|---|---|---|---|---|---|
| Pitch Attitude (`C_theta`) | θ_ref → δe | PI | 0.917 | 0.857 | 100 |
| Velocity Hold (`C_vel`) | VT_ref → δT | PI | 0.32 | 0.096 | 100 |
| Altitude Hold (`C_alt`) | h_ref → θ_ref | PI | 0.050 | 0.0043 | 100 |
| Bank Angle (`C_phi`) | φ_ref → δa | PI | −0.45 | −0.036 | 100 |
| Heading (`K_heading`) | ψ_ref → φ_ref | P | 0.1975 | — | — |

**SAS dampers:** `Kq = 0.10` (pitch) · `Kr = 0.1481` (yaw, sobre washout) · `Kp = 0` (roll).

Principais mudanças vs o tuning 2026-06-09 (valores antigos comentados no `DH_inicializacao.m`):
- **Kq** 0.0524 → 0.10 (ζ_SP 0.68 → 0.81, folga p/ fechamento de θ);
- **C_theta** re-sintonizado (`pidtune` wc=3.5, PM=75°, ref-tracking) — o OS de 15% em θ vinha do acoplamento com o throttle, não da malha;
- **C_vel** +10% de ganho (corta OS de VT no NL: degrau +2 m/s dava 20,9%);
- **C_alt** Kp/Ki +74%/+66% (v2: OS de h 5,8%, ess ≈ 0 — a v1 deixava offsets de 1–2 m na missão a 15,2 m/s);
- **C_phi** P forte + I profundo (zero 0.08) — planta φ/δa não é tipo-1; P puro deixava ess de 10–15%, PI clássico dava OS de 24–28%;
- **Kr / Kp / K_heading / tau_psi / tau_ref / clamp:** inalterados.

> Ganhos do roll com sinal negativo pois `Cl_da < 0`.

---

## Atuadores (servo) e pré-filtro

O modelo NL inclui dinâmica de atuador realista (preparação p/ hardware-in-the-loop):

- **Servo (cada superfície):** saturação de posição `±25°` → rate limiter `act.rate = 150°/s` → lag `1/(act.tau·s+1)` com `act.tau = 0.05 s` (`act.bw = 20 rad/s`).
- **Motor/hélice (throttle):** rate `eng.rate = 1/s` → lag `1/(eng.tau·s+1)` com `eng.tau = 0.30 s`. Saturação `[0, 1]`.
- **Pré-filtro de referência:** `1/(tau_ref·s+1)`, `tau_ref = 3 s`, aplicado ao **delta** de cada referência (VT, h, ψ) antes das malhas. Suaviza os degraus e remove o "teleporte" dos atuadores **sem** mexer nos controladores. Controlador de 2 graus de liberdade (resposta ao comando ⟂ estabilidade).

---

## Estabilidade e desempenho da malha fechada (retune 2026-08-10)

Verificação do retuning contra os requisitos da dissertação:

- **Não-linear (overshoot):** VT 8,1% · h 1,4% · ψ 0%.
- **Linear (overshoot):** θ 7,6% · φ 8,5%.
- **Margens:** GM ≥ 21 dB, PM ≥ 86° em todas as malhas (requisito: GM > 10 dB, PM > 45°).
- **Curto-período:** via SAS Kq, ζ_SP ≈ 0.81.

> Para os modos da planta atual, ver a saída de `DH_build_model` / `linear/MATRIZES_DH.m` (os autovalores dependem dos coeficientes carregados). O script de verificação do retune está em `HIL_PID\Matlab\retuning_requisitos_PID.m` (fora deste repositório).

---

## Topologia de comandos no modelo NL

A `S-Function` espera o vetor de entrada nessa ordem:

```
U = [throttle, elevator, aileron, rudder, wind_x, wind_y, wind_z]
     U(1)      U(2)       U(3)     U(4)    U(5..7)
```

O piloto automático produz **delta** (perturbação em torno do trim); a soma com o trim absoluto acontece **antes** dos saturadores:

```
PID_out (delta) ─►(+)─► Sat ─► canal absoluto da S-Function
                  ▲
               TrimInput(k) = Ue(k)
```

**Saturadores (limites físicos):**
- Throttle: `[0, 1]`
- Elevator / Aileron / Rudder: `[−25°, +25°] = [−0.4363, +0.4363] rad`

---

## Validação

Linear × não-linear: para perturbações pequenas em torno do trim a divergência é pequena; para manobras grandes os modelos divergem como esperado (não-linearidades aerodinâmicas + acoplamentos cinemáticos). As figuras estão em `arquivos/figs/` (`fig_nl_lin_*.png`).

---

## X-Plane: gêmeo digital e missões por waypoints (`xplane/`)

O `.acf` do DH no X-Plane 9 foi **calibrado como gêmeo** do modelo identificado
(trim α 14,50° vs 14,44°; CLα 2,87 vs 2,88 /rad; curto período ωn 6,25 vs
6,3 rad/s) e **validado voando as mesmas manobras nos dois mundos** com o mesmo
controlador e ganhos — campanha G5: RMS de altitude **6,95 m (SIL) vs 6,96 m
(X-Plane)**. Relatório completo (método, antes/depois, figuras):
[`docs/Relatorio_gemeo_XPlane.pdf`](docs/Relatorio_gemeo_XPlane.pdf); números e
histórico de calibração: [`xplane/EQUIVALENCIA_ACF.md`](xplane/EQUIVALENCIA_ACF.md).

```matlab
% Demo (X-Plane 9 aberto com o DH carregado):
addpath('xplane')
XP_gui_waypoints     % GUI: clique waypoints (ou "Carregar circuito OVAL") e VOAR
```

- **Guiagem** LOS por waypoints (algoritmo do PIPER-1-6 do Julio, reimplementado)
  fabricando referências para a **mesma cascata PID deste repo** (ganhos intocados);
  só a caixa "planta" muda — X-Plane via UDP/XPlaneConnect a 20 Hz.
- **Missão padrão:** circuito oval de 6 WPs (~126 s), dimensionado para a energia
  do motor elétrico do XP9 (~130 s de voo motorizado por reload — diagnóstico
  completo em [`xplane/PENDENCIA_MOTOR.md`](xplane/PENDENCIA_MOTOR.md)). A
  simulação **encerra sozinha 5 s após capturar o último waypoint**.
- **Antes de cada voo:** `File → Open Aircraft` no X-Plane (religa o motor) — ou
  `XP_auto_reload = true` para o reload programático (`xp_reload_acf.m`).
- **Sem X-Plane:** `NL_missao` voa a mesma missão na planta da Ana em segundos
  (plano B da demo e comparação SIL×XP automática após cada voo).

---

## Artigo (`arquivos/`)

Draft IEEE Access comparando o **PID cascata** deste repositório com o **LQRy de ganho fixo** (entrega embarcada na bancada, campanha de 10/ago), do SIL ao HIL, com estudo de robustez (modelo do Sato, perturbação paramétrica, vento). Narrativa: PID vence desempenho nominal; LQRy vence robustez.

- `main.tex` + `sections/01–07` — fontes LaTeX (classe `ieeeaccess.cls`); `main.pdf` — última compilação.
- `sims/` — scripts que geram as figuras e tabelas do artigo (rodam sobre este repo + os `.mat` das campanhas, fora do repositório).
- `dados_*.txt` — métricas exportadas para as tabelas.
- `PENDENCIAS.md` — pontos abertos do draft, marcados no LaTeX com `\pendente{...}`.

---

## Referências

- STEVENS, B. L.; LEWIS, F. L. *Aircraft Control and Simulation*, 3rd ed. Wiley, 2016.
- SANTOS, M. H. dos. *Projeto e teste em bancada de um piloto automático para veículos aéreos não tripulados*. Dissertação (Mestrado) — ITA, São José dos Campos, 2018.
- SATO, F. C. Y. C. *Modelo matemático completo não-linear DH*, ITA / EEC-D.
