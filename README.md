# AI Racing League final project: Corkscrew driver (1:03.628, zero damage)

Autonomous TORCS driver built for the **AI Racing League — Go FullSpeed with IBM Bob** cohort.

| | |
|---|---|
| Final script | [`driver/torcs_jm_par_line_g9a.py`](driver/torcs_jm_par_line_g9a.py) |
| Track / settings | Corkscrew, 1 lap, standing start, driver `scr_server 1`, car `car1-ow1` (dynamics unchanged) |
| Best lap | **63.628 s** (1:03.628), 3 identical repeat runs |
| Damage | **0** |
| Top speed | 299 km/h |
| Video + screenshot | [Google Drive](https://drive.google.com/drive/folders/10xlkvkG5lfCXRIYFQKuudTFVKJwvKDof?usp=sharing) |

## Author and members

Team:
- Umut Uygur (Computer Engineering, 4th year): team lead, development, testing and submission — umutt.uygurr@gmail.com
- Yusuf Atakan Ünal (Computer Engineering, 4th year) — yusufatakanu@gmail.com
- Kayra Üstten (Electrical and Electronics Engineering) — kayraustten@gmail.com
- Vesile Dila Akhan (Computer Engineering, 2nd year) — dilaakhan05@gmail.com
- Kaan Efe Kocaman (Computer Engineering, 1st year) — kefe.kocaman@tedu.edu.tr, kocamankaanefe@gmail.com

## Problem statement

The goal is the fastest standing-start lap of Corkscrew with **zero damage**, driving the car only through the SCR
client interface. Our driver follows a racing line and speed plan computed in advance from the track geometry and
corrects itself with a feedback controller. Its parameters were tuned by automated search over thousands of
simulated laps.

## Environment and setup

- **OS:** Windows 11
- **Python:** 3.12.10
- **TORCS and challenge kit:** the TORCS Windows build supplied by the organisers, which includes the `scr_server`
  driver. The race config needs Corkscrew, `scr_server 1` and 1 lap. The kit's car and driver files were **not
  modified**: `cars/car1-ow1/car1-ow1.xml` is the original, and the car dynamics are unchanged.
- **Required local files:** only `driver/torcs_jm_par_line_g9a.py`. The racing plan is embedded in the file.
- **External dependencies:** none. The driver uses the Python standard library only (`socket`, `math`, `json`,
  `zlib`, `base64`, ...).
- The development code in `source/racelab/` additionally needs numpy and scipy, plus PyTorch for the RL parts. It is
  **not** needed to run the driver.

## Run

1. Start TORCS **first**: Race → Quick Race → Configure Race → track **Corkscrew**, driver **scr_server 1**,
   **1 lap** → New Race. TORCS waits for a client on UDP port 3001.
2. Then start the client from this repository's `driver/` folder:

Windows:
```
py torcs_jm_par_line_g9a.py -p 3001
```
macOS/Linux:
```
python3 torcs_jm_par_line_g9a.py -p 3001
```

`-p` selects the port (default 3001). Each `scr_server` slot listens on its own port (3001, 3002, ...); for a
multi-car mass start, run one client per slot with that slot's port. Close heavy background programs while racing (see *Limitations*).

## Results

All runs were measured on the same track and settings. Every row in
[`experiment_results.csv`](experiment_results.csv) is a real recorded run.

| Version | Lap | Damage |
|---|---|---|
| Kit baseline `snakeoil3_gym.py` | did not finish | 1863 |
| End-to-end RL (SAC) | 85.736 s | 0 |
| Improved RL + braking assist | 73.098 s | 0 |
| Enhancement 1: racing line + speed profile | 68.746 s | 0 |
| Enhancement 2: drivetrain/launch control + per-corner gains | 64.964 s | 0 |
| Enhancement 3: automated search → **final** | **63.628 s** | 0 |

Evidence:
- [Video and screenshot of the GUI lap](https://drive.google.com/drive/folders/10xlkvkG5lfCXRIYFQKuudTFVKJwvKDof?usp=sharing)
- [`evidence/g9a_runs/`](evidence/g9a_runs/): the three final runs (`summary.json`, full per-step `telemetry.csv`,
  logs).
- [`evidence/final_check_20261016-075220.txt`](evidence/final_check_20261016-075220.txt): 30-run check with 10 ms
  TORCS timing.
- [`results/leaderboard.csv`](results/leaderboard.csv): all 1,684 benchmark runs of the project.

## Enhancements

1. **Precomputed racing line + physics speed profile (85.7 s → 68.7 s).**
   - *Why:* the RL driver zig-zagged on straights and braked late.
   - *What:* track centre line, width and elevation are read from the track mesh. A minimum-curvature line is solved
     with a fixed margin to the edge. The maximum speed at each point comes from lateral grip + downforce, then
     forward/backward passes add acceleration and braking limits.
   - *Steering:* a Stanley-type controller (heading error, lateral error, curvature feed-forward, yaw damping) follows
     the line.
   - *Code:* `source/racelab/planner/`.
2. **Drivetrain and standing-start control + per-corner gains (68.7 s → 65.0 s).**
   - *Why:* comparing telemetry showed time lost at the launch and on corner exits.
   - *What:* gear selection from driven-wheel rpm, clutch slip control and launch traction control. Steering gains
     are now set per corner zone.
3. **Automated search with a zero-damage constraint (65.0 s → 63.628 s).**
   - *Search:* island-model evolutionary search and coordinate search over per-zone speed, braking point, apex shift
     and steering gains, plus racing-line variants. Up to 8 headless simulators run in parallel.
   - *Constraint:* a candidate only counts if it finishes the lap with zero damage.
   - *Result:* the best line weights high-curvature points (`1 + 8·κ/κmax`) and keeps a 1.2 m margin.
   - *Rejected:* drifting (larger slip angles) was tested and rejected; it was always slower.

Also included: a **residual RL hybrid** where SAC learns small corrections on top of the planner,
[`driver/torcs_jm_par_hybrid_res3a.py`](driver/torcs_jm_par_hybrid_res3a.py). It runs 64.878 s and is the most
robust variant: damage-free in 30/30 runs with 10 ms timing.

## Use of IBM Bob

**IBM Bob** was used mainly for understanding, reviewing and checking:
- Making sense of the kit and project structure: what each folder, script and config file does, and how the client
  talks to `scr_server`.
- Understanding the car mechanics and driving control: gears, clutch, braking, steering and the sensor values the
  driver receives.
- Code review and testing: walking through the driver code, finding bugs and weak spots, and suggesting fixes.
- Staying within the competition rules: checking that the driver does not change the car model or its dynamics,
  only uses the allowed client interface, and that the lap counts as clean (zero damage).

Every suggestion was reviewed by us and confirmed with simulator runs before it was kept.

**Claude Code** (Anthropic) was used for the rest of the development: writing the racing-line planner, controller,
parameter-search and RL training code, running the experiments, and analysing telemetry. Every reported result comes
from recorded simulator runs; see `experiment_results.csv` and `evidence/`.

## Demonstration

Video and final-lap screenshot: [Google Drive folder](https://drive.google.com/drive/folders/10xlkvkG5lfCXRIYFQKuudTFVKJwvKDof?usp=sharing)

One continuous standing-start lap on Corkscrew with the provided (unchanged) livery, from launch to the
final lap time and damage readout.

## Livery

[`livery/car1-ow1.rgb`](livery/car1-ow1.rgb) is our team livery for the mass-start show (512×512 SGI texture,
preview: [`livery/preview.png`](livery/preview.png)). It is based on the provided IBM template, with orange-red
replacing the blue. To install it, copy it to `torcs/drivers/scr_server/<slot>/car1-ow1.rgb` and restart TORCS.

## Limitations

- **Sensitive to missed steps.** The final driver is tuned for every TORCS step being answered on time, as in a
  normal GUI race. With 10 ms timeouts under heavy CPU load it stayed damage-free in only 3/30 runs. The hybrid driver
  is the robust alternative.
- **Leaves the asphalt briefly.** The lap uses the run-off area at a few points (max |trackPos| 1.5, no contact, no
  damage). That is allowed by the rules, but it leaves little margin.
- **Corkscrew only.** The plan is track-specific.
- **No opponent awareness.** The driver does not see other cars and simply follows its own line in a mass start.

## Credits and licensing

- TORCS and the `scr_server` / `gym_torcs` client skeleton (`torcs_jm_par.py`, snakeoil) belong to their original
  authors and were provided through the course kit. The submission driver is built on that skeleton; we do not claim
  ownership of the provided code, and the kit itself is not redistributed here.
- The racing-line planner, controller, search, RL training code and livery modification are our own work.

## Repository layout

```
driver/               final drivers (standalone, standard library only)
livery/               team livery (.rgb) + preview
evidence/             final runs (summary, telemetry, logs) and 30-run check
experiment_results.csv  one row per recorded milestone run
results/              all benchmark runs (leaderboard.csv, LAP_LOG.md)
source/racelab/       full development code
  planner/            track model, racing line, speed profile, driver, searches, submission builder
  planner/params/     tuned parameter sets (snap_g9det.json = final)
  rl/                 SAC / residual RL training code, small model snapshots
  submission/         earlier milestone drivers listed in experiment_results.csv
  run_benchmark.py    headless benchmark + telemetry proxy
docs/dev_log_tr/      development log and block reports (Turkish)
SUBMISSION.md         submission e-mail
```

To rebuild the driver, place `source/racelab/` inside the kit folder next to `torcs/` and `gym_torcs/`, then run:
```
python racelab/planner/build_line_submission.py --params racelab/planner/params/snap_g9det.json --out my_driver.py
```
