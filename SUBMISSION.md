To: ibmskillsbuild.emea@skillup.online
Subject: AI Racing League Project Submission - Umut Uygur

Hello Team,

Please find my project submission for evaluation.

Name: Umut Uygur
Project Members:
- Umut Uygur (umutt.uygurr@gmail.com)
- Yusuf Atakan Ünal (yusufatakanu@gmail.com)
- Kayra Üstten (kayraustten@gmail.com)
- Vesile Dila Akhan (dilaakhan05@gmail.com)
- Kaan Efe Kocaman (kefe.kocaman@tedu.edu.tr, kocamankaanefe@gmail.com)
GitHub Link: https://github.com/umutuygur0/ibm_submission_race
Final snapshot: [commit URL, optional]
Video Demonstration: https://drive.google.com/drive/folders/10xlkvkG5lfCXRIYFQKuudTFVKJwvKDof?usp=sharing

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
IBM Bob was used mainly to understand the kit and project structure (folders, scripts, configs and the
scr_server client interface) and the car mechanics and driving control. It was also used for code review and
testing: finding bugs and weak spots in the driver code. Finally, it was used to check that the solution stays within
the competition rules (no change to the car model or dynamics, clean zero-damage lap). We reviewed every suggestion
and confirmed it with simulator runs before keeping it. Claude Code was used for the rest of the development:
the racing-line planner, controller, parameter search and RL training code, experiments and telemetry analysis.

Thank you,
Umut Uygur
