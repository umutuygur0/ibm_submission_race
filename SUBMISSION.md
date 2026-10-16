To: ibmskillsbuild.emea@skillup.online
Subject: AI Racing League Project Submission - Umut Uygur

Hello Team,

Please find my project submission for evaluation.

Name: Umut Uygur
Project Members: [team member names, or "individual project"]
GitHub Link: https://github.com/umutuygur0/ibm_submission_race
Final snapshot: [commit URL, optional]
Video Demonstration: [video URL]

Project Description / Problem Statement:
An autonomous TORCS driver that sets the fastest possible standing-start lap on Corkscrew with zero damage.
The car follows a precomputed racing line and speed plan with a feedback controller. Its parameters were tuned
by automated search over thousands of headless simulator laps. Best lap: 1:03.628, zero damage.

Enhancements:
1. Precomputed racing line + physics-based speed profile: replaced the zig-zagging end-to-end RL driver (73.1 s)
   with a planned line and a steering/speed controller (68.7 s).
2. Drivetrain and launch control: gear selection from wheel rpm, clutch slip control, launch traction control and
   per-corner controller gains gave a faster standing start and better corner exits (64.9 s).
3. Automated search with a zero-damage constraint: island-model evolution over racing-line variants and per-corner
   speed, brake, apex and gain parameters, 8 simulators in parallel (63.628 s). A residual-RL hybrid (64.878 s,
   30/30 damage-free under 10 ms timing) is included as a robust alternative.

Use of IBM Bob (Free trial version) and AI:
[Describe honestly how IBM Bob and other AI assistants were used, e.g. code generation for the planner and search
scripts, debugging the TORCS/UDP timing issues, analysing telemetry, and writing reports.]

Thank you,
Umut Uygur
