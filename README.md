# IBM AI Racing League — Corkscrew driver (63.628 s, zero damage)

Autonomous driver for TORCS (`scr_server`) built for the **AI Racing League — Go FullSpeed with IBM Bob** cohort.

| | |
|---|---|
| Track | Corkscrew, 1 lap, standing start, `scr_server 1`, car `car1-ow1` (dynamics unchanged) |
| Best lap | **63.628 s** (1:03.628) |
| Damage | **0** |
| Top speed | 299 km/h |
| Final driver | [`driver/torcs_jm_par_line_g9a.py`](driver/torcs_jm_par_line_g9a.py) |
| Video | _add link here_ |

The driver is a single Python file based on the kit's `torcs_jm_par.py` client skeleton. It uses **only the Python
standard library** (no numpy / PyTorch), and the full driving plan is embedded in the file.

## Run it

1. Start TORCS → Race → Quick Race → Configure Race: track **Corkscrew**, driver **scr_server 1**, **1 lap** → New Race
   (TORCS waits on port 3001).
2. In a terminal:

```
cd driver
python torcs_jm_par_line_g9a.py -p 3001
```

Close heavy background programs while racing: the fastest driver is tuned for a TORCS step that is answered on time
(see *Limitations*).

## How it works

1. **Track model.** The Corkscrew centre line, width and elevation are read from the track mesh (`corkscrew.acc`).
2. **Racing line.** A minimum-curvature line is computed with iteratively re-weighted least squares (high-curvature
   points get more weight, which pushes the solution towards a minimax-curvature line), keeping a fixed margin to
   the track edge. Final line: weight `1 + 8·κ/κmax`, margin 1.2 m (`source/racelab/planner/line_variants.py`).
3. **Speed profile.** Maximum cornering speed from lateral grip + aerodynamic downforce, then forward (acceleration)
   and backward (braking) passes, including gradient/vertical-curvature terms.
4. **Controller.** Stanley-type steering (heading error, lateral error, curvature feed-forward, yaw damping, low-pass
   filter), speed tracking with brake/throttle, gear selection from driven-wheel rpm, clutch slip control and launch
   traction control for the standing start.
5. **Per-corner tuning.** The lap is divided into 11 zones; each zone has a speed factor, brake factor, apex shift and
   steering-gain multipliers.
6. **Automatic search.** Island-model evolutionary search + coordinate search over all zone and controller
   parameters, running up to 8 headless TORCS instances in parallel. A candidate only counts if the full lap is
   finished with **zero damage**. Leaving the asphalt briefly is allowed by the rules and is bounded by a limit
   on |trackPos|.

## Enhancements and results

| Step | Lap | Notes |
|---|---|---|
| Kit baseline / first RL agent (SAC) | 85.7 s → 73.1 s | pure reinforcement learning, too much zig-zag on straights |
| 1. Precomputed racing line + physics speed profile | 68.7 s | replaced end-to-end RL with a planned line |
| 2. Gearbox, clutch slip control, launch traction control, per-corner gains | 64.9 s | faster standing start and corner exits |
| 3. Evolutionary search on line variants + zone parameters (zero-damage constraint) | **63.628 s** | narrower, curvature-weighted racing line |

Also explored: a **residual RL hybrid** (SAC learns small steering/pedal corrections on top of the planner):
[`driver/torcs_jm_par_hybrid_res3a.py`](driver/torcs_jm_par_hybrid_res3a.py), 64.878 s, damage-free in 30/30 runs
with 10 ms TORCS timing. Drifting (allowing larger slip angles) was tested; the search always turned it off because it
was slower on this car.

Evidence: [`evidence/`](evidence/). It contains the three identical 63.628 s runs with telemetry, plus the final 30-run
check. All benchmark runs are in [`results/leaderboard.csv`](results/leaderboard.csv) and
[`results/LAP_LOG.md`](results/LAP_LOG.md).

## Livery

[`livery/car1-ow1.rgb`](livery/car1-ow1.rgb) (512×512 SGI, preview [`livery/preview.png`](livery/preview.png)) is our
team livery for the mass-start show. It is based on the provided IBM template, with orange-red replacing the blue.
To use it, copy it to `torcs/drivers/scr_server/<slot>/car1-ow1.rgb` and restart TORCS. The official video uses the
unchanged provided livery.

## Limitations

- `torcs_jm_par_line_g9a.py` is tuned for TORCS answering every step on time (GUI race). When steps are missed (10 ms
  timeouts under heavy CPU load), the lap was damage-free in only 3/30 runs. The hybrid driver above is the robust
  alternative (30/30).
- Tuned for Corkscrew only. The plan is track-specific.
- The driver does not react to other cars. In a mass start it drives its own line.

## Repository layout

```
driver/            final submission drivers (standalone, standard library only)
livery/            team livery (.rgb) + preview
evidence/          final runs: summary, telemetry, 30-run check
results/           all benchmark runs (leaderboard.csv, LAP_LOG.md)
source/racelab/    full development code
  planner/         track model, racing line, speed profile, driver, searches, submission builder
  planner/params/  tuned parameter sets (snap_g9det.json = final)
  rl/              SAC / residual RL training code and small snapshots
  run_benchmark.py headless benchmark + telemetry proxy
docs/dev_log_tr/   development log and block reports (Turkish)
```

To rebuild or re-tune, place `source/racelab/` inside the course kit folder next to `torcs/` and `gym_torcs/`.
Those kit components are not redistributed here. Then run, for example:

```
python racelab/planner/build_line_submission.py --params racelab/planner/params/snap_g9det.json --out my_driver.py
python racelab/run_benchmark.py --agent my_driver.py --mode fast
```

The development code expects Python 3.12 with numpy and scipy, plus PyTorch for the RL parts.

## Credits

- TORCS and the `scr_server` / `gym_torcs` client skeleton (`torcs_jm_par.py`, snakeoil) come from the course kit and
  their original authors. We do not claim ownership of the provided code.
- Racing-line, controller, search and RL code: our own work.
