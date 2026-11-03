# railnav-demo

**Autonomous railway navigation with ground-penetrating radars — a runnable demo.**

[![Python](https://img.shields.io/badge/python-3.10%2B-blue.svg)](https://www.python.org/)
[![License: CC BY 4.0](https://img.shields.io/badge/License-CC_BY_4.0-lightgrey.svg)](LICENSE)
[![Tests](https://img.shields.io/badge/tests-24%20passed-brightgreen.svg)](tests/)

Reference demonstration implementation of the algorithms from the paper
*«Автономная навигация на железной дороге с помощью георадаров»*
(Elafarenko, Polyanchenko, Gurov, Grishaev) — see [Citation](#citation):

1. **Algorithm 1 — recurrent route localization.** The train's drive history
   (odometer readings + radar detections of reference objects) is matched
   against a topological railway graph by the update rule

   ```
   x_{k+1} = A_Gᵀ [ x̃_k ⊙ m_k ],    x̃_k = x_k / ‖x_k‖∞
   ```

   where `A_G` is the adjacency matrix of the **pair-node graph** (a node
   `G_ij = S_i S_j` is a *pair* of consecutive track segments, which turns the
   second-order Markov chain of admissible routes into a first-order one), and
   `m_k` is the observation likelihood `μ = μ_symm · μ_dist · μ_emb`.

2. **Map building / updating from drive history (Viterbi).** Each drive
   session is explained by a layered hypothesis graph — *new node*, *known
   node*, *missed detection* (`MERGE`), *false detection* (`PEND`) — decoded
   with Viterbi dynamic programming and gated by a **margin criterion**
   `Δ > ν_margin`. Approved sessions mutate the map; ambiguous ones go to an
   anomaly log.

> ⚠️ **All data is fully synthetic.** The railway network is generated
> procedurally; the "radar" is a simulator with odometer noise, missed and
> false detections. No real УЛАН/ЕГИС data is used.

## Installation

```bash
git clone https://github.com/kenoma/railnav-demo.git
cd railnav-demo
python3 -m pip install -e .    # numpy, pandas, scipy, matplotlib, plotly
```

## Quickstart

```python
import numpy as np
from railnav.synth import station_example, sample_trip, simulate_drive
from railnav.localizer import Localizer, MuParams

g = station_example(seed=1)                       # synthetic station: 7 switches
rng = np.random.default_rng(7)
trip = sample_trip(g, rng, min_segments=6, max_segments=7)
sim = simulate_drive(g, trip, rng, odometer_noise=0.01)
print('truth route:', ' - '.join(s for s, _ in trip))

res = Localizer(g, MuParams(), theta=0.5).run(sim.detections, final=sim.final)
print('localized:', res.localized, '| node:', f'{res.node.prev} -> {res.node.nxt}',
      '| position:', res.position)
```

Example output (the route is sampled randomly, so it may differ):

```text
truth route: p3s - p2s - p1s - p0 - t1 - t2 - t3
localized: True | node: t1 -> t2 | position: (18.0, 0.0)
```

Three possible outcomes of the algorithm are reported honestly:
**route identified** (a single dominant state), **unknown route** (the drive
does not match the map — zero belief vector), **route too short** (several
dominant candidates remain; more switches must be passed).

## Notebooks

Run them in order; each is self-contained. *Notebook prose is in Russian*
(an English translation may follow; this README is the canonical English
entry point).

| Notebook | Contents |
|---|---|
| `01_paper_example` | The track segment from Fig. 4 of the paper; reproduces the adjacency matrix `A_G` from Eq. (5); U-turn restrictions |
| `02_localizer` | Algorithm 1: cold/warm start, all three outcomes, belief trace |
| `03_mu_components` | Likelihood components: bell-shaped `μ_dist` (Eq. 7, Fig. 7 analog), `μ_symm` (inversion-invariant pattern match), `μ_emb`; component ablation |
| `04_station_drive` | A full drive over a 7-switch station: animation of the train and of the belief vector `x_k` |
| `05_map_updating` | Viterbi map building from an empty map; missed / false detections; margin criterion; strict / balanced / radical modes |
| `06_statistics` | Identification rate vs. number of passed switches at cold start (analog of Fig. 8), with and without `μ_emb` |

## Package layout

| Module | Responsibility |
|---|---|
| `graph.py` | Object nodes, track segments, pair nodes `G_ij`, sparse `A_G`, switch routes; `paper_example()` reproduces Eq. (5) |
| `model.py` | `μ_symm`, `μ_dist` (Eq. 7), `μ_emb`, `MuParams` with switchable components |
| `localizer.py` | Algorithm 1: recurrent update, position via `T(K_j, s^c)`, per-step trace |
| `map_updater.py` | Layered Viterbi hypothesis graph, weight filter, margin gate, map mutation, anomaly log |
| `synth.py` | `station_example`, `random_network`, `simulate_drive` (odometer noise, missed/false detections, noisy embeddings) |
| `metrics.py` | Identification-rate curves at cold start |
| `viz.py` | matplotlib schematics and animation; interactive plotly plots |

## Deviations from the paper (deliberate simplifications)

1. **Second-order chain → first order.** The Viterbi map builder runs on
   states "directed edge + entry object" (the same pair-node `G_ij` used by
   Algorithm 1) rather than on pairs of edges; the no-U-turn constraint
   `k ≠ i` is enforced locally.
2. **Single-accumulator `PEND`.** A run of false detections is merged with an
   additive carry; alternative splits of an interval are not enumerated.
3. **`dominance_tol`.** A "dominant" candidate satisfies `x > Θ` **and**
   `x ≥ 0.9·max`. Without this tolerance, embedding noise and the `φ` penalty
   create spurious secondary modes on the true route.
4. **`delta_a_rel`.** Odometer error is multiplicative, so the effective bell
   width is `δ_a = max(δ_a, δ_a_rel·L)`.
5. **Embeddings are emulated** with unit vectors (same object = noisy
   duplicates, different objects ≈ orthogonal). In the paper the Siamese
   ResNet-18 is trained on real ballast-prism images.
6. **Cold start** initializes `x_0` from the **first observation pair**
   (the first interval is partial, `s ≤ L + 3δa`), consistent with the
   paper's requirement to identify from a sequence of detections.

## Testing

```bash
python3 -m pytest tests/ -q      # 24 tests: graph, localizer, map updater
```

## Citation

```bibtex
@article{railnav2026,
  title   = {Автономная навигация на железной дороге с помощью георадаров},
  author  = {Гришаев, Сергей and Гуров, Юрий and Елафаренко, Руслан and Поляниченко, Дмитрий},
  journal = {TODO: venue},   % TODO: add year, volume, DOI / arXiv id
  year    = {TODO}
}
```

## License

© 2026 S. Grishaev, Yu. Gurov, R. Elafarenko, D. Polyanchenko.
This demo is licensed under [CC BY 4.0](LICENSE).

## Языковые версии / Language versions

* English: this file
* Русский: [README.ru.md](README.ru.md)
