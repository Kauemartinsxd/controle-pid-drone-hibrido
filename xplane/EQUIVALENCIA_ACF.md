# Equivalência .acf ↔ drone real (modelo da Ana)

## STATUS 2026-08-30 — FASE B iterações 1–2 FEITAS: gêmeo ~85–90%

Edições aplicadas (com o Kaue; backup pré-Fase B:
`DH-Lon-REV-03_backup_preFaseB_20260830.acf`):
- **B1**: incidência da asa +5°→0° (6 elementos, tela Wings);
- **B2 iter-1**: aerofólio novo `Aircraft/Radio Control/airfoils/DH asa
  equiv.afl` (base NACA 2412: intercept 0.25→0.20→**0.30**, slope
  0.102→**0.064**, lin range 10→**16**, maximum 1.6→1.4→**1.45**,
  cd-min 0.006→**0.020**, d alpha=10 0.012→0.040→**0.048**), atribuído
  aos 4 slots da WING 1;
- **B2 iter-2**: h-stab incidence −2°→**−5°** (4 elementos).

Resultado medido (trim nivelado sink 0.00, polar 6 pontos, ident de
arfagem, sondas laterais — voos de 2026-08-30 ~14:30–15:00):
| métrica | Ana | .acf iter-2 |
|---|---|---|
| α trim 12 m/s | 14,4° | **14,9°** ✅ |
| CLα | 2,88/rad | **2,87** ✅ |
| α(VT) na polar | — | ±0,5° ✅ |
| sink(VT) | 2,1–3,2 | 2,55–3,04 (+12%) |
| curto período | 6,3 rad/s / ζ0,54 | 9,5 / **ζ0,61** ✅ζ |
| δe trim | +7,6° | +9,7° |
| p_ss/δa | −8,5 | −6,9 (−19%) |
| dutch roll | 5,0 / 0,41 | ~5,6–8,9 / 0,35 |

**Refinamentos restantes** (opcionais, ordem de impacto):
1. ωn do curto período 9,5→6,3: reduzir área/braço do h-stab (~−35% de
   Cmα) — re-conferir trim depois;
2. δe de trim +9,7→+7,6: h-stab incidence −5°→−6°;
3. rolagem −19%: área/corda do aileron +20%;
4. sink +12%: d alpha=10 0.048→0.044.
ATENÇÃO: as âncoras de missão (XP_missao/XP_voo: Xe(8), clamp, trims de
engate) foram calibradas p/ o .acf ANTIGO — recalibrar antes de voar
G2 no gêmeo (trim novo: α≈15°, θ_ref centro ~14–15°, de ≈+10, thr ≈0,47).

---

# Fase A: medição original (2026-08-30, pré-Fase B)

> Objetivo: transformar o `DH-Lon-REV-03.acf` num gêmeo do DRONE REAL,
> cuja referência é o modelo identificado da Ana (coef_DH.m, 24/04/2026).
> Esta página quantifica a distância atual e dá os alvos da Fase B
> (edições no Plane Maker / Airfoil Maker).
> Dados: `voos/XP_sonda_equiv_20260830_141304.mat` (lateral + polar),
> `voos/XP_ident_theta_20260819_223117.mat` (arfagem), voos G2 e
> `voos/XP_voo_20260830_141959.mat` (trim 14 m/s). Régua da Ana:
> trimagem/planeio computados de `dyn_rigidbody_DH` (resíduos ~1e-12).

## 1. Tabela-mestre de comparação

### Trim motorizado nivelado
| VT | Ana: α / δe / thr | .acf: α / δe / thr |
|---|---|---|
| 12 m/s | 14,4° / +7,6° / 0,284 | ≈5° / +2..3° / 0,5–0,85 (fase nominal dos G2) |
| 14 m/s | 9,7° / +3,7° / 0,314 | 2,4° / −2,8° / 0,84 (voo dedicado, sink ~0) |
| (Ana 10→15) | α 21,9→8,0°; δe +13,9→+2,2°; thr ~0,28–0,34 | — |

### Polar de planeio (δe fixo, thr=0 — mesmo experimento nos dois)
| δe | Ana: VT / sink / α | .acf: VT / sink / α |
|---|---|---|
| 0° | 17,0 / −4,13 / 5,4° | 13,4 / −1,94 / 3,5° |
| +2° | 15,2 / −3,21 / 7,8° | 12,3 / −1,87 / 5,3° |
| +4° | 13,9 / −2,66 / 10,1° | 11,5 / −1,95 / 7,0° |
| +6° | 12,9 / −2,32 / 12,5° | 11,2 / −1,76 / 8,6° |
| +8° | 12,0 / −2,10 / 14,9° | 11,5 / −2,12 / 8,9° (eficácia saturando ≈ pré-estol) |

Padrões: α_Ana/α_acf ≈ 1,6 (constante!); VT_acf ≈ VT_Ana − 3~4 m/s por δe;
L/D_acf ≈ 6,4–6,6 vs L/D_Ana ≈ 4,1–5,7 (o .acf plana MELHOR = arrasta menos).

### Dinâmica
| Modo | Ana (linearização 12 m/s) | .acf (identificado em voo) |
|---|---|---|
| Curto período | ωn 6,3 rad/s, ζ 0,54 | ωn ≈9,8 rad/s, ζ ≈1,1 (superamortecido) |
| Autoridade δe (pico q/δe) | ~5,5 rad/s /rad | ~4,8 rad/s /rad ✅ |
| Fugóide | ωn 1,12, ζ 0,085 | não medido formalmente |
| Rolagem: τ | 0,09 s | ≲0,1 s ✅ |
| Rolagem: p_ss/δa | −8,5 rad/s /rad | −5,0 (fator 1,7 menor) |
| Dutch roll | ωn 5,04, ζ 0,41 | ωn ≈2,3, ζ ≈0,33 |
| Espiral | instável (+0,23) | instável (deriva de φ observada) ✅ |
| Limite de α | sem estol no modelo (voa a 14–22°) | estola ≈9–10° |

### Massa/propulsão
| | Ana | .acf |
|---|---|---|
| Massa | 2,2 kg | 2,2 kg ✅ |
| T_max a 12 m/s | 14 N (F=14·δt, linear) | ~14–16 N (0,80 hp) ✅ |
| Lei thr→T | linear | hélice (não-linear; thr de trim difere) ⚠️ |

## 2. Diagnóstico-síntese

O .acf tem **asa "boa demais"**: CLα efetivo ≈1,6× o do drone real
(CLα_Ana = 2,88 /rad — baixo, típico do drone real que voa "pendurado")
e **arrasto menor** (CD0_Ana = 0,100). Por isso o .acf trima com α≈5°
onde o real usa 14,4°. Além disso o aerofólio genérico do .acf estola a
~9–10° — o regime de voo do drone real (α 14–22°) é INACESSÍVEL ao .acf
atual. Cauda: arfagem sobreamortecida (Cmq efetivo alto) e direcional
mole (dutch roll 2× lento); aileron com 60% da autoridade.

## 3. FASE B — alvos de edição (Plane Maker / Airfoil Maker, iterativo)

Referência numérica (coef_DH.m): CLα 2,88 /rad, CL0 0,194, CD0 0,100,
Cmα −0,88, Cmq −9,93, Cmde +1,05, Clp −1,33, Clda −0,71, Cnb +0,153,
Cnr −0,46.

1. **B1 — incidência da asa: +5° → 0°** (reverter a edição de
   2026-08-19). Efeito esperado: α de trim sobe ~5° (5°→~10°).
2. **B2 — aerofólio da asa (Airfoil Maker)**: cl_α menor (alvo: CLα_3D
   ≈ 2,9 /rad), **estol tardio (≥20°)** — sem isso o trim a α=14,4° é
   fisicamente impossível no X-Plane — e cd maior (alvo: polar da
   tabela §1: sink 2,1–4,1 m/s na faixa; L/D_max ≈ 5).
3. **B3 — empenagem horizontal**: reduzir amortecimento de arfagem
   (alvo: curto período ωn 6,3 / ζ 0,54; hoje 9,8 / 1,1) via área/braço
   do h-stab — re-verificar o trim após (mexe em Cmα/Cm0 também).
4. **B4 — aileron**: autoridade ×1,7 (p_ss/δa −5,0 → −8,5 rad/s /rad)
   via área/corda do aileron (curso já é ±25°).
5. **B5 — deriva/direcional**: dutch roll ωn 2,3 → 5,0 rad/s (Cnβ
   efetivo ×~4,8 → deriva maior/braço maior), mantendo ζ ~0,4.
6. **B6 — aceito como diferença**: lei thr→T de hélice (vs linear);
   T_max já casado.

**Processo**: B1+B2 → rodar `XP_sonda_equivalencia` + `XP_ident_theta`
→ comparar com a régua → iterar B2 → B3 → B4/B5 → re-identificação
completa + G2 de validação. Estimativa: 2–4 iterações de ~20 min
(edição do Kaue) + ~5 min (sondas).

## 4. Notas operacionais

- Motor: ver PENDENCIA_MOTOR.md (reload religa; energia ~90–150 s de
  voo motorizado por reload — o mapa de trim fino de 10–15 m/s ficou
  parcial por isso: pontos limpos em 12 e 14 m/s).
- Sinal de rudder/aileron e demais manhas: xp_send_dh.m / xp_read_dh.m.
- A régua da Ana é reproduzível em `scratchpad` (trim_ana_fm/proc_equiv)
  e pode ser recomputada a qualquer VT.
