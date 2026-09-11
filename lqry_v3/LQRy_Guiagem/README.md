# LQRy do DH + guiagem por waypoints no modelo não linear

Pacote autocontido (MATLAB R2025b, Simulink) com o LQRy gain-scheduled do
`CL_NL_DH_18_jun_2026` **sem nenhuma alteração de estrutura**, recebendo referências de
uma guiagem LOS por waypoints em vez dos doublets do artigo. Serve para voar as missões
de guiagem da dissertação (circuito oval e circuito agressivo) no modelo NL da Ana, com
os ganhos originais ou com os ganhos re-sintetizados ("v3").

## Como rodar

```matlab
cd <esta pasta>
gui_guiagem_NL      % mapa clicável (ganhos originais por padrão)
guiagem_NL          % ou o script direto: ganhos = 'mirko' por padrão; troque para 'v3' na configuração
```

**`gui_guiagem_NL`** abre um mapa: cada clique marca um waypoint (altitude e velocidade dos
campos ao lado), os botões *Circuito OVAL* e *Circuito AGRESSIVO* carregam as duas missões da
dissertação, a tabela é editável, e *SIMULAR NO NL* roda o `guiagem_NL` com os ganhos escolhidos
no menu (*originais (Mirko)* ou *v3*), a planta do gain scheduling (automática pela mediana das
velocidades dos WPs, ou 12/15/18 m/s) e a constante de tempo do motor. A trajetória simulada é
desenhada por cima do mapa e o resumo sai no console. Arraste para mover o mapa e use a roda do
mouse para zoom.

**`guiagem_NL`** (o que a GUI chama) imprime as capturas (distância mínima a cada WP contra
`R_accept`), os extremos de V_T, h, α, φ, δe e manete, e salva duas figuras + um `.mat` em
`resultados/`. A simulação para sozinha 5 s depois de entrar no círculo do último waypoint.
Qualquer variável da configuração (`ganhos`, `WPs`, `R_accept`, `VT_missao`, `eng_tau`) definida
no workspace antes de rodar o script sobrescreve o padrão; é assim que a GUI o usa.

## O que está no modelo `CL_NL_DH_GUIA.slx`

| Parte | Origem | Observação |
|---|---|---|
| θ Hold, Alt Hold, Vel Hold, φ Hold, ψ Hold, gain scheduling, atuadores de superfície 24/(s+24), lag de manete 0,1 s, `TrimConst*`, `Switch`/`Switch1`/`Switch2` dos modos | `CL_NL_DH_18_jun_2026` | **intactos** (blocos, parâmetros, ligações internas) |
| `Guidance_Star` (chart) + `Goto`/`From` `PsiRefGuia`, `DhGuia`, `DvGuia`, `PhiRefGuia` | novo | guiagem LOS: `psi_ref` → `Sum18` (ψ Hold), `h_ref_rel` → `Sum8` (Alt Hold), `v_ref` → `Sum10` (Vel Hold), `phi_ref` → `Switch2` (só no modo φ Hold) |
| Subsistema `Planta` | novo | `sfunction_DH` original + saturação ±15°, rate limit 150°/s, servo 0,05 s, motor (rate 1/s + lag `eng_tau`); H de controle relativa ao engate, como no seu estimador |
| `theta_hold_ref` (θ de trim) e `beta_ref` (0) | novo | substituem os Steps de θ e β do artigo nos modos θ Hold e no integrador de β |
| `R2D_theta_ref`, `R2D_psi_ref`, `R2D_phi_ref` | novo | só alimentam os seus logs `theta_NL`, `psi_NL`, `phi_NL` com a referência em graus (antes vinha dos `Manual Switch`) |
| `Goto` `Elev`/`Ail`/`Rud`/`throttle` dentro de `Planta` | novo | os seus `From` de log (`elev_NL`, `ail_NL`, `rud_NL`, `Throttle_NL`) passam a ler a deflexão/manete entregue pelos atuadores |
| Fim de missão (`Cond_fim_missao` … `Stop_fim_missao`) | novo | para 5 s após o último WP |

Removidos em relação ao seu modelo: todos os Steps de doublet (ψ, φ, θ, H, V_T), os
`Manual Switch` que os selecionavam e os `Degrees to Radians` dessas referências. Nada
foi removido de dentro das malhas. Os únicos parâmetros alterados em blocos seus são as
condições iniciais de três integradores: `Integrator2` (Alt Hold) parte de
`xi_alt0 = θ_trim / G_int,Alt`, para que θ_ref(0) seja o θ de trim (engate sem transiente);
os outros partem de 0 como no original.

Referências: a guiagem entrega ψ_ref no mesmo referencial (relativo ao engate) que o ψ
Hold realimenta, h_ref relativa ao engate e v_ref absoluta. Nenhuma passa por filtro,
limitador ou termo proporcional: entram **só pelo integrador**, exatamente como os
Steps do artigo.

## Ganhos

- `ganhos_mirko/`: os seus `Ganho_hold_*.mat` e `Dados_Trim.mat`, sem alteração.
- `ganhos_v3/`: mesma estrutura, mesmos nomes de variáveis, ganhos re-sintetizados com pesos
  de Bryson ancorados nos limites do atuador (LQ com ação integral + refino por
  realimentação de saída via Lyapunov, eq. 11–16 do artigo). `relatorio_projeto.txt` traz
  polos, margens e ganhos por planta; `tabela_ganhos_orig_vs_v3.txt` compara com os
  originais; `lqry_v3_projeto.m` regenera tudo
  (`lqry_v3_projeto('raizN', '<ganhos_mirko>', 'outdir', '<destino>')`, precisa do
  Control System Toolbox).

## Resultado esperado (circuito oval, 15 m/s, R_accept 100 m)

| Ganhos | Capturas | Comportamento |
|---|---|---|
| originais | 2/6 | departure logo após o WP1 (φ > 400°, H −1600 m): profundor e manete saturam com 1,8° de erro de θ e 1,3 m/s de erro de V_T |
| v3 | 6/6 | passa a 2–14 m dos WPs, h 599,9–600,1 m, φ ≤ 13°, δe +2,2..+2,5°, manete 0,33–0,34 |

Com `eng_tau = 3.5` (constante de tempo do motor elétrico do X-Plane 9) o v3 repete o
6/6; é a condição em que o gêmeo X-Plane foi voado (oval 6/6 e agressivo 4/4 em 03/09/2026).
