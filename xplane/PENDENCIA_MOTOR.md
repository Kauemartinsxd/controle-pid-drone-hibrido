# PENDÊNCIA: motor/hélice do .acf sem empuxo em voo (2026-08-20)

> Bloqueia o G2 **nominal** (altitude constante) do PLANO_GUIAGEM. A guiagem
> em si está validada (G1 + G2-planeio, 4/4 capturas — ver `voos/`).

## Sintoma

O DH v3 no X-Plane **nunca produz empuxo líquido positivo em voo**, em
nenhuma condição medida. Todos os voos (inclusive os de 2026-08-19 tidos
como "sustentados") eram planeios: h sempre caindo (sink 0,2–1,7 m/s),
throttle subindo até saturar sem efeito.

## Medições (via `sim/flightmodel/engine/POINT_thrust`, 2026-08-20)

| Condição | Hélice [rad/s] | Empuxo [N] |
|---|---|---|
| thr 1.0, VT 12–13,8 m/s | satura ~520 (~5000 RPM) | **−0,1 a −0,9** |
| thr 1.0, VT 10–12 m/s | 340–440 | −0,1 a −0,8 |
| thr 1.0, VT 17–18,5 m/s | 600–670 | −0,6 a −1,2 |
| Gabarito: Piper 1/6 do Julio | — | **+5 a +19** (mesmo dref) |

Causa provável: **RPM máximo ~5000 com o passo atual → velocidade de passo
≈ 12 m/s ≈ VT de voo** — a hélice "acompanha" o ar, nunca traciona
(windmill leve). O modelo matemático do DH tem T_max = 14 N (F = 14·δt);
o .acf entrega ~0 N.

## Fatos correlatos descobertos

1. **Capacidade de combustível do .acf = 0** (`sim/aircraft/weight/
   acf_m_fuel_tot` = 0) — provável efeito colateral da edição de peso de
   2026-08-19 no Plane Maker (7,0→4,9 lb). Em 2026-08-20 o motor foi
   encontrado MORTO (hélice 12 rad/s, windmill); foi religado pela UI
   durante a sessão (starter — não há starter via UDP/dref no XP9;
   `ENGN_running`, `POINT_tacrad`, DATA rows 34/35 foram testados e o
   modelo do motor sobrescreve tudo).
2. **Teleporte (sendPOSI) derruba o RPM do motor para ~60 rad/s** e o
   spool de volta leva ~15–20 s — mais uma manha do harness: mesmo com
   empuxo consertado, os primeiros ~20 s pós-engate terão tração parcial.
   (Se necessário: engatar e aguardar o spool antes de exigir subida.)
3. Escrever fuel via dref funciona (`sim/aircraft/overflow/acf_tank_rat`
   primeiro [0]=1, depois `sim/flightmodel/weight/m_fuel1`), mas foi
   restaurado a zero — o peso extra piora o planeio e não religa o motor.

## Conserto (Plane Maker, ~5 min — mão do Kaue)

1. `Plane Maker > Standard > Engine Specs`: subir o **prop RPM máximo**
   para ~8000–9000 (ou aumentar o passo da hélice ~+2°/reduzir o
   diâmetro) — alvo: velocidade de passo ≥ 1,5× VT de cruzeiro (≥18 m/s).
2. `Standard > Weight & Balance`: definir **fuel capacity > 0** (ex.
   0,3 kg) para o starvation não voltar (ou confirmar que o XP9 aceita
   capacidade 0 sem matar o motor após reload).
3. Salvar o .acf e **recarregar a aeronave no X-Plane** (Aircraft > Open).
4. Validar ANTES de voar (MATLAB):
   ```matlab
   import XPlaneConnect.*; global GlobalSocket
   GlobalSocket = openUDP('127.0.0.1', 49009, 0, 500);
   sendCTRL([0 0 0 1.0 -998 -998], 0, GlobalSocket); pause(8);
   t = getDREFs({'sim/flightmodel/engine/POINT_thrust'}, GlobalSocket);
   double(t(1))   % alvo: >= +4 N em voo a ~12 m/s (T/W do modelo: 14 N max)
   ```
5. Re-voar o G2 nominal: `XP_missao` (defaults = quadrado 500×500 m em
   h constante). Critérios: 4 capturas, φ<25°, VT 12±1, h ~constante.
