# Lap Log — skor tablosu

Otomatik üretilir (`racelab/run_benchmark.py`). Ham veri: `leaderboard.csv`,
her koşunun telemetrisi: `runs/<run_id>/telemetry.csv`.

Sıralama: önce turu bitirenler (hasarsız olan önde, sonra tur süresi),
sonra bitiremeyenler (en uzağa giden önde). Pist: Corkscrew ≈ 3608 m.
`@gui-timing` = TORCS'un GUI'deki 10 ms bekleme süresiyle koşu (gerçek koşula yakın, rastgelelik var).

## Her agent'ın en iyi koşusu

| # | Agent (label) | Bitirme | Tur süresi (s) | Mesafe (m) | Hasar | Çarpma | Pist dışı (kez / s) | Max hız | Ort. hız | Koşu |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | B9_g9a_det1 | 1/1 | 63.628 | 3646.8 | 0 | 0 | 14 / 5.04 | 299.0 | 198.3 | `20261016-075154_B9_g9a_det1` |
| 2 | B9_g9a_det2 | 1/1 | 63.628 | 3646.8 | 0 | 0 | 14 / 5.04 | 299.0 | 198.3 | `20261016-075158_B9_g9a_det2` |
| 3 | B9_g9a_det3 | 1/1 | 63.628 | 3646.8 | 0 | 0 | 14 / 5.04 | 299.0 | 198.3 | `20261016-075203_B9_g9a_det3` |
| 4 | FINAL_torcs_jm_par_line_g9a.py_det | 1/1 | 63.628 | 3646.8 | 0 | 0 | 14 / 5.04 | 299.0 | 198.3 | `20261016-075220_FINAL_torcs_jm_par_line_g9a.py_det` |
| 5 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | 4/30 | 63.658 | 3644.6 | 0 | 0 | 14 / 5.04 | 299.0 | 198.2 | `20261016-075400_FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing` |
| 6 | B9_g7a_det1 | 1/1 | 63.928 | 3624.7 | 0 | 0 | 10 / 2.9 | 298.8 | 197.1 | `20261016-032314_B9_g7a_det1` |
| 7 | FINAL_torcs_jm_par_line_g7a.py_det | 1/1 | 63.928 | 3624.7 | 0 | 0 | 10 / 2.9 | 298.8 | 197.1 | `20261016-075548_FINAL_torcs_jm_par_line_g7a.py_det` |
| 8 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | 13/30 | 63.974 | 3621.2 | 0 | 0 | 9 / 2.64 | 298.8 | 197.0 | `20261016-075730_FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing` |
| 9 | B9_g6a_det1 | 1/1 | 64.026 | 3753.2 | 0 | 0 | 10 / 3.04 | 299.1 | 198.1 | `20261016-003123_B9_g6a_det1` |
| 10 | B9_g6a_det2 | 1/1 | 64.026 | 3753.2 | 0 | 0 | 10 / 3.04 | 299.1 | 198.1 | `20261016-003129_B9_g6a_det2` |
| 11 | FINAL_torcs_jm_par_line_g5r4.py_det | 1/1 | 64.67 | 3712.8 | 0 | 0 | 0 / 0.0 | 299.0 | 196.1 | `20261014-101645_FINAL_torcs_jm_par_line_g5r4.py_det` |
| 12 | FINAL_torcs_jm_par_line_g5r6.py_det | 1/1 | 64.718 | 3709.6 | 0 | 0 | 0 / 0.0 | 299.1 | 195.9 | `20261014-101810_FINAL_torcs_jm_par_line_g5r6.py_det` |
| 13 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 30/30 | 64.718 | 3709.6 | 0 | 0 | 0 / 0.0 | 299.1 | 195.9 | `20261014-101935_FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing` |
| 14 | FINAL_torcs_jm_par_line_t4off.py_det | 1/1 | 64.722 | 3709.3 | 0 | 0 | 5 / 0.94 | 299.1 | 195.7 | `20261015-040321_FINAL_torcs_jm_par_line_t4off.py_det` |
| 15 | FINAL_torcs_jm_par_line_t3off.py_det | 2/2 | 64.73 | 3708.8 | 0 | 0 | 5 / 0.98 | 299.1 | 195.7 | `20261014-233400_FINAL_torcs_jm_par_line_t3off.py_det` |
| 16 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 54/60 | 64.73 | 3708.8 | 0 | 0 | 5 / 0.98 | 299.1 | 195.7 | `20261014-233405_FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing` |
| 17 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 30/30 | 64.75 | 3707.4 | 0 | 0 | 2 / 0.32 | 299.1 | 195.9 | `20261014-101747_FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing` |
| 18 | FINAL_torcs_jm_par_line_g5r0.py_det | 1/1 | 64.778 | 3705.6 | 0 | 0 | 0 / 0.0 | 298.9 | 195.6 | `20261014-101943_FINAL_torcs_jm_par_line_g5r0.py_det` |
| 19 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 30/30 | 64.778 | 3705.6 | 0 | 0 | 0 / 0.0 | 298.9 | 195.6 | `20261014-102048_FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing` |
| 20 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 26/30 | 64.804 | 3703.8 | 0 | 0 | 4 / 0.84 | 299.1 | 195.5 | `20261015-040425_FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing` |
| 21 | FINAL_torcs_jm_par_line_t2.py_det | 1/1 | 64.84 | 3701.6 | 0 | 0 | 0 / 0.0 | 299.1 | 195.4 | `20261014-114000_FINAL_torcs_jm_par_line_t2.py_det` |
| 22 | FINAL_torcs_jm_par_line_t1.py_det | 1/1 | 64.874 | 3699.3 | 0 | 0 | 0 / 0.0 | 299.1 | 195.2 | `20261014-114224_FINAL_torcs_jm_par_line_t1.py_det` |
| 23 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 30/30 | 64.876 | 3699.1 | 0 | 0 | 0 / 0.0 | 299.1 | 195.2 | `20261014-114019_FINAL_torcs_jm_par_line_t2.py_x30@gui-timing` |
| 24 | FINAL_torcs_jm_par_hybrid_res3a.py_det | 4/4 | 64.878 | 3697.4 | 0 | 0 | 7 / 1.42 | 298.6 | 195.1 | `20261014-175653_FINAL_torcs_jm_par_hybrid_res3a.py_det` |
| 25 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 120/120 | 64.878 | 3697.4 | 0 | 0 | 7 / 1.42 | 298.6 | 195.1 | `20261015-040838_FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing` |
| 26 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 30/30 | 64.952 | 3694.1 | 0 | 0 | 0 / 0.0 | 299.1 | 195.0 | `20261014-114358_FINAL_torcs_jm_par_line_t1.py_x30@gui-timing` |
| 27 | line_gr3L@GUI | 2/2 | 64.954 | 3617.7 | 0 | 0 | 0 / 0.0 | 299.1 | 194.0 | `20261014-111505_line_gr3L@GUI` |
| 28 | FINAL_torcs_jm_par_line_gr3L.py_det | 4/4 | 64.964 | 3693.2 | 0 | 0 | 0 / 0.0 | 299.1 | 194.9 | `20261014-002212_FINAL_torcs_jm_par_line_gr3L.py_det` |
| 29 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 119/120 | 64.964 | 3693.2 | 0 | 0 | 0 / 0.0 | 299.1 | 194.9 | `20261014-002218_FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing` |
| 30 | B6_hybrid_parity | 1/1 | 65.002 | 3689.1 | 0 | 0 | 6 / 1.4 | 295.7 | 194.7 | `20261014-164420_B6_hybrid_parity` |
| 31 | FINAL_torcs_jm_par_line_gr2.py_det | 2/2 | 65.156 | 3680.1 | 0 | 0 | 0 / 0.0 | 299.1 | 194.2 | `20261014-001743_FINAL_torcs_jm_par_line_gr2.py_det` |
| 32 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 60/60 | 65.156 | 3680.1 | 0 | 0 | 0 / 0.0 | 299.1 | 194.2 | `20261014-001748_FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing` |
| 33 | FINAL_torcs_jm_par_line_gr3rob.py_det | 1/1 | 65.162 | 3679.8 | 0 | 0 | 0 / 0.0 | 299.1 | 194.2 | `20261014-001528_FINAL_torcs_jm_par_line_gr3rob.py_det` |
| 34 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 30/30 | 65.162 | 3679.8 | 0 | 0 | 0 / 0.0 | 299.1 | 194.2 | `20261014-001611_FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing` |
| 35 | FINAL_torcs_jm_par_line_gr1b.py_det | 1/1 | 65.218 | 3675.9 | 0 | 0 | 0 / 0.0 | 298.6 | 193.9 | `20261013-225323_FINAL_torcs_jm_par_line_gr1b.py_det` |
| 36 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 30/30 | 65.218 | 3675.9 | 0 | 0 | 0 / 0.0 | 298.6 | 193.9 | `20261013-225421_FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing` |
| 37 | B5_line_gr1a_det | 1/1 | 65.498 | 3656.2 | 0 | 0 | 0 / 0.0 | 294.9 | 192.8 | `20261013-223848_B5_line_gr1a_det` |
| 38 | FINAL_torcs_jm_par_line_gr1a.py_det | 1/1 | 65.498 | 3656.2 | 0 | 0 | 0 / 0.0 | 294.9 | 192.8 | `20261013-225523_FINAL_torcs_jm_par_line_gr1a.py_det` |
| 39 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 30/30 | 65.498 | 3656.2 | 0 | 0 | 0 / 0.0 | 294.9 | 192.8 | `20261013-225715_FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing` |
| 40 | B5_parity_clutch | 1/1 | 67.112 | 3683.3 | 0 | 0 | 0 / 0.0 | 291.8 | 189.2 | `20261013-220711_B5_parity_clutch` |
| 41 | FINAL_torcs_jm_par_line_e1c.py_det | 1/1 | 67.768 | 3636.4 | 0 | 0 | 0 / 0.0 | 286.0 | 186.8 | `20261013-051502_FINAL_torcs_jm_par_line_e1c.py_det` |
| 42 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 30/30 | 67.782 | 3635.4 | 0 | 0 | 0 / 0.0 | 286.1 | 186.8 | `20261013-051639_FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing` |
| 43 | B4_line_e1b_det | 1/1 | 67.826 | 3632.2 | 0 | 0 | 0 / 0.0 | 285.9 | 186.6 | `20261013-042830_B4_line_e1b_det` |
| 44 | FINAL_torcs_jm_par_line_e1b.py_det | 1/1 | 67.826 | 3632.2 | 0 | 0 | 0 / 0.0 | 285.9 | 186.6 | `20261013-051643_FINAL_torcs_jm_par_line_e1b.py_det` |
| 45 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 30/30 | 67.968 | 3621.8 | 0 | 0 | 3 / 0.98 | 286.0 | 186.2 | `20261013-051724_FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing` |
| 46 | B4_line_e1a_det | 1/1 | 68.128 | 3744.4 | 0 | 0 | 0 / 0.0 | 285.1 | 187.0 | `20261013-040307_B4_line_e1a_det` |
| 47 | FINAL_torcs_jm_par_line_e3d1.py_det | 1/1 | 68.582 | 3716.3 | 0 | 0 | 0 / 0.0 | 284.0 | 185.7 | `20261013-071246_FINAL_torcs_jm_par_line_e3d1.py_det` |
| 48 | FINAL_torcs_jm_par_line_e2d1.py_det | 2/2 | 68.746 | 3705.9 | 0 | 0 | 0 / 0.0 | 283.6 | 185.3 | `20261013-062037_FINAL_torcs_jm_par_line_e2d1.py_det` |
| 49 | FINAL_torcs_jm_par_line_z12rob.py_det | 1/1 | 68.792 | 3703.1 | 0 | 0 | 0 / 0.0 | 284.0 | 185.2 | `20261013-021609_FINAL_torcs_jm_par_line_z12rob.py_det` |
| 50 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 30/30 | 68.85 | 3698.7 | 0 | 0 | 1 / 0.2 | 284.0 | 184.9 | `20261013-071337_FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing` |
| 51 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 60/60 | 68.922 | 3694.5 | 0 | 0 | 2 / 0.52 | 283.8 | 184.8 | `20261013-062143_FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing` |
| 52 | FINAL_torcs_jm_par_line_z10rob11.py_det | 1/1 | 68.934 | 3693.8 | 0 | 0 | 0 / 0.0 | 283.6 | 184.7 | `20261013-021742_FINAL_torcs_jm_par_line_z10rob11.py_det` |
| 53 | B4_line_z6_det | 1/1 | 68.95 | 3692.7 | 0 | 0 | 0 / 0.0 | 283.5 | 184.6 | `20261013-005058_B4_line_z6_det` |
| 54 | FINAL_torcs_jm_par_line_z6.py_det | 1/1 | 68.95 | 3692.7 | 0 | 0 | 0 / 0.0 | 283.5 | 184.6 | `20261013-022225_FINAL_torcs_jm_par_line_z6.py_det` |
| 55 | B4_line_z9rob_det | 1/1 | 68.956 | 3692.3 | 0 | 0 | 0 / 0.0 | 283.5 | 184.8 | `20261013-012916_B4_line_z9rob_det` |
| 56 | FINAL_torcs_jm_par_line_z9rob.py_det | 2/2 | 68.956 | 3692.3 | 0 | 0 | 0 / 0.0 | 283.5 | 184.8 | `20261013-013954_FINAL_torcs_jm_par_line_z9rob.py_det` |
| 57 | B4_line_z7rob_det | 1/1 | 68.992 | 3689.9 | 0 | 0 | 0 / 0.0 | 283.5 | 184.6 | `20261013-010850_B4_line_z7rob_det` |
| 58 | FINAL_torcs_jm_par_line_z7rob.py_det | 1/1 | 68.992 | 3689.9 | 0 | 0 | 0 / 0.0 | 283.5 | 184.6 | `20261013-022050_FINAL_torcs_jm_par_line_z7rob.py_det` |
| 59 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 30/30 | 69.518 | 3654.0 | 0 | 0 | 3 / 0.9 | 284.1 | 182.9 | `20261013-021652_FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing` |
| 60 | B4_line_z5_det | 1/1 | 69.576 | 3650.2 | 0 | 0 | 0 / 0.0 | 282.9 | 182.9 | `20261013-003919_B4_line_z5_det` |
| 61 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 30/30 | 69.578 | 3649.9 | 0 | 0 | 3 / 0.6 | 283.7 | 182.8 | `20261013-021921_FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing` |
| 62 | FINAL_torcs_jm_par_line_z9rob.py_x2@gui-timing | 2/2 | 69.604 | 3648.0 | 0 | 0 | 3 / 0.52 | 283.8 | 182.7 | `20261013-014000_FINAL_torcs_jm_par_line_z9rob.py_x2@gui-timing` |
| 63 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 30/30 | 69.616 | 3647.2 | 0 | 0 | 3 / 0.64 | 283.7 | 182.5 | `20261013-022306_FINAL_torcs_jm_par_line_z6.py_x30@gui-timing` |
| 64 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 30/30 | 69.682 | 3642.5 | 0 | 0 | 3 / 0.68 | 283.7 | 182.3 | `20261013-022204_FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing` |
| 65 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 30/30 | 69.708 | 3640.7 | 0 | 0 | 2 / 0.34 | 283.6 | 182.2 | `20261013-021749_FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing` |
| 66 | B4_line_z5_x8@gui-timing(yük altında) | 8/8 | 70.006 | 3619.0 | 0 | 0 | 3 / 1.0 | 283.1 | 181.5 | `20261013-003958_B4_line_z5_x8@gui-timing(yük altında)` |
| 67 | B1_v10d_d4.0m20 | 1/1 | 72.94 | 3693.0 | 0 | 0 | 0 / 0.0 | 282.2 | 175.2 | `20261010-004746_B1_v10d_d4.0m20` |
| 68 | B1_v10d_d4.0m20@gui-timing | 10/10 | 72.94 | 3693.0 | 0 | 0 | 0 / 0.0 | 282.2 | 175.2 | `20261010-004816_B1_v10d_d4.0m20@gui-timing` |
| 69 | SUBMISSION_v10d_dist_a4m20 | 1/1 | 72.94 | 3693.0 | 0 | 0 | 0 / 0.0 | 282.2 | 175.2 | `20261010-051200_SUBMISSION_v10d_dist_a4m20` |
| 70 | B1R_a4m20@gui-timing | 14/20 | 72.94 | 3693.0 | 0 | 0 | 0 / 0.0 | 282.2 | 175.2 | `20261010-051640_B1R_a4m20@gui-timing` |
| 71 | B1_v10d_d3.5m20 | 1/1 | 73.032 | 3686.1 | 0 | 0 | 0 / 0.0 | 279.4 | 174.8 | `20261010-004628_B1_v10d_d3.5m20` |
| 72 | B1_v10d_d3.5m20@gui-timing | 10/10 | 73.032 | 3686.1 | 0 | 0 | 0 / 0.0 | 279.4 | 174.8 | `20261010-004651_B1_v10d_d3.5m20@gui-timing` |
| 73 | B1_v10d_d4.0m35@gui-timing | 10/10 | 73.07 | 3683.7 | 0 | 0 | 0 / 0.0 | 277.3 | 174.8 | `20261010-005011_B1_v10d_d4.0m35@gui-timing` |
| 74 | B2_r02@gui-timing | 20/20 | 73.076 | 3683.3 | 0 | 0 | 0 / 0.0 | 277.3 | 174.7 | `20261010-200607_B2_r02@gui-timing` |
| 75 | SUBMISSION_v10d_dist_r02 | 1/1 | 73.098 | 3681.9 | 0 | 0 | 0 / 0.0 | 277.3 | 174.6 | `20261010-202021_SUBMISSION_v10d_dist_r02` |
| 76 | regress_v2builder_r02 | 1/1 | 73.098 | 3681.9 | 0 | 0 | 0 / 0.0 | 277.3 | 174.6 | `20261012-010659_regress_v2builder_r02` |
| 77 | B3_r02_x30@gui-timing | 30/30 | 73.1 | 3681.3 | 0 | 0 | 0 / 0.0 | 277.3 | 174.6 | `20261012-061431_B3_r02_x30@gui-timing` |
| 78 | B2_m0r04@gui-timing | 20/20 | 73.148 | 3678.9 | 0 | 0 | 0 / 0.0 | 279.5 | 174.5 | `20261010-201357_B2_m0r04@gui-timing` |
| 79 | B1_v10d_d4.0m35 | 1/1 | 73.164 | 3677.6 | 0 | 0 | 0 / 0.0 | 277.2 | 174.5 | `20261010-004906_B1_v10d_d4.0m35` |
| 80 | B1_v10d_d3.0m35@gui-timing | 10/10 | 73.25 | 3671.7 | 0 | 0 | 0 / 0.0 | 272.7 | 174.1 | `20261010-005219_B1_v10d_d3.0m35@gui-timing` |
| 81 | B2_r03@gui-timing | 20/20 | 73.25 | 3671.7 | 0 | 0 | 0 / 0.0 | 274.0 | 174.2 | `20261010-200911_B2_r03@gui-timing` |
| 82 | B1_v10d_d2.5m20@gui-timing | 10/10 | 73.298 | 3668.2 | 0 | 0 | 0 / 0.0 | 272.1 | 174.0 | `20261010-005134_B1_v10d_d2.5m20@gui-timing` |
| 83 | B1_v10d_d2.5m20 | 1/1 | 73.306 | 3668.0 | 0 | 0 | 0 / 0.0 | 272.2 | 174.0 | `20261010-005028_B1_v10d_d2.5m20` |
| 84 | B1_v10d_d3.0m35 | 1/1 | 73.306 | 3668.0 | 0 | 0 | 0 / 0.0 | 272.5 | 174.0 | `20261010-005149_B1_v10d_d3.0m35` |
| 85 | B2_r05@gui-timing | 20/20 | 73.37 | 3664.0 | 0 | 0 | 0 / 0.0 | 271.7 | 173.8 | `20261010-201102_B2_r05@gui-timing` |
| 86 | B1_CONTROL@gui-timing | 10/10 | 73.424 | 3658.2 | 0 | 0 | 0 / 0.0 | 270.5 | 173.5 | `20261010-004522_B1_CONTROL@gui-timing` |
| 87 | B2_r0@gui-timing | 13/20 | 73.448 | 3659.2 | 0 | 0 | 1 / 0.6 | 279.4 | 173.8 | `20261010-201759_B2_r0@gui-timing` |
| 88 | B2_m10r03@gui-timing | 20/20 | 73.462 | 3657.7 | 0 | 0 | 0 / 0.0 | 276.6 | 173.6 | `20261010-201237_B2_m10r03@gui-timing` |
| 89 | B4_line_parity | 1/1 | 73.522 | 3654.0 | 0 | 0 | 0 / 0.0 | 277.5 | 173.8 | `20261012-234407_B4_line_parity` |
| 90 | SUBMISSION_v10d_dist_a4m20@gui-timing | 11/20 | 73.542 | 3652.5 | 0 | 0 | 0 / 0.0 | 282.3 | 173.3 | `20261010-051205_SUBMISSION_v10d_dist_a4m20@gui-timing` |
| 91 | SUBMISSION_v10d_dist | 1/1 | 73.582 | 3649.2 | 0 | 0 | 0 / 0.0 | 270.5 | 173.1 | `20261009-061807_SUBMISSION_v10d_dist` |
| 92 | SUBMISSION_v10d_dist@gui-timing | 30/30 | 73.582 | 3649.2 | 0 | 0 | 0 / 0.0 | 270.5 | 173.1 | `20261009-061818_SUBMISSION_v10d_dist@gui-timing` |
| 93 | B1_CONTROL | 1/1 | 73.582 | 3649.2 | 0 | 0 | 0 / 0.0 | 270.5 | 173.1 | `20261010-004454_B1_CONTROL` |
| 94 | SUBMISSION_v10b_best_dist | 1/1 | 73.728 | 3638.6 | 0 | 0 | 0 / 0.0 | 259.7 | 172.7 | `20261009-015401_SUBMISSION_v10b_best_dist` |
| 95 | B2_ctrl3g50@gui-timing | 20/20 | 73.768 | 3636.3 | 0 | 0 | 0 / 0.0 | 270.5 | 172.4 | `20261010-201853_B2_ctrl3g50@gui-timing` |
| 96 | B3_w512a_v08171_a3m50_det | 1/1 | 74.29 | 3712.7 | 0 | 0 | 1 / 0.06 | 267.5 | 172.0 | `20261012-061506_B3_w512a_v08171_a3m50_det` |
| 97 | SUBMISSION_v10b_v04324_dist | 1/1 | 74.416 | 3710.6 | 0 | 0 | 0 / 0.0 | 260.6 | 171.0 | `20261009-015437_SUBMISSION_v10b_v04324_dist` |
| 98 | SUBMISSION_v10b_best_dist@gui-timing | 10/10 | 74.44 | 3717.8 | 0 | 0 | 0 / 0.0 | 259.7 | 171.9 | `20261009-015429_SUBMISSION_v10b_best_dist@gui-timing` |
| 99 | B3_w512a_v08171_a3m50_x30@gui-timing | 30/30 | 74.49 | 3700.6 | 0 | 0 | 1 / 0.06 | 267.5 | 171.5 | `20261012-061548_B3_w512a_v08171_a3m50_x30@gui-timing` |
| 100 | SUBMISSION_v10b_best_plain | 1/1 | 75.156 | 3672.8 | 0 | 0 | 0 / 0.0 | 249.2 | 169.6 | `20261009-015514_SUBMISSION_v10b_best_plain` |
| 101 | SUBMISSION_v10b_best_plain@gui-timing | 10/10 | 75.156 | 3672.8 | 0 | 0 | 0 / 0.0 | 249.2 | 169.6 | `20261009-015523_SUBMISSION_v10b_best_plain@gui-timing` |
| 102 | SUBMISSION_v10b_v04324_dist@gui-timing | 10/10 | 75.162 | 3669.4 | 0 | 0 | 1 / 0.1 | 260.6 | 169.4 | `20261009-015444_SUBMISSION_v10b_v04324_dist@gui-timing` |
| 103 | SUBMISSION_s47_dist | 1/1 | 77.436 | 3650.3 | 0 | 0 | 0 / 0.0 | 260.7 | 164.9 | `20261008-141739_SUBMISSION_s47_dist` |
| 104 | SUBMISSION_s47_dist@gui-timing | 10/10 | 77.764 | 3633.0 | 0 | 0 | 0 / 0.0 | 260.7 | 164.3 | `20261008-141747_SUBMISSION_s47_dist@gui-timing` |
| 105 | SUBMISSION_s47_assist@GUI | 1/1 | 78.154 | 3618.2 | 0 | 0 | 0 / 0.0 | 253.3 | 163.2 | `20261008-140917_SUBMISSION_s47_assist@GUI` |
| 106 | SUBMISSION_s47_assist | 1/1 | 78.158 | 3710.9 | 0 | 0 | 0 / 0.0 | 253.3 | 163.5 | `20261008-071848_SUBMISSION_s47_assist` |
| 107 | SUBMISSION_v6g_assist | 1/1 | 78.934 | 3681.6 | 0 | 0 | 0 / 0.0 | 267.9 | 161.9 | `20261007-214011_SUBMISSION_v6g_assist` |
| 108 | SUBMISSION_v6g_assist@gui-timing | 20/20 | 79.204 | 3667.3 | 0 | 0 | 0 / 0.0 | 267.9 | 161.3 | `20261007-214040_SUBMISSION_v6g_assist@gui-timing` |
| 109 | SUBMISSION_s47_assist@gui-timing | 10/10 | 79.738 | 3633.1 | 0 | 0 | 0 / 0.0 | 253.3 | 160.4 | `20261008-071857_SUBMISSION_s47_assist@gui-timing` |
| 110 | SUBMISSION_v6g@GUI | 1/1 | 79.782 | 3617.7 | 0 | 0 | 0 / 0.0 | 264.2 | 159.6 | `20261007-101531_SUBMISSION_v6g@GUI` |
| 111 | SUBMISSION_v6g@gui-timing | 10/10 | 79.788 | 3632.9 | 0 | 0 | 0 / 0.0 | 264.2 | 159.8 | `20261007-073420_SUBMISSION_v6g@gui-timing` |
| 112 | SUBMISSION_v6g | 1/1 | 79.796 | 3632.4 | 0 | 0 | 0 / 0.0 | 264.2 | 159.8 | `20261007-073415_SUBMISSION_v6g` |
| 113 | SUBMISSION_v6f_fast | 1/1 | 79.806 | 3631.6 | 0 | 0 | 0 / 0.0 | 254.6 | 160.8 | `20261007-064252_SUBMISSION_v6f_fast` |
| 114 | SUBMISSION_v6f_fast@gui-timing | 4/10 | 79.806 | 3631.6 | 0 | 0 | 0 / 0.0 | 254.6 | 160.8 | `20261007-064306_SUBMISSION_v6f_fast@gui-timing` |
| 115 | sac_v6f_best | 1/1 | 79.864 | 3628.1 | 0 | 0 | 0 / 0.0 | 254.7 | 160.6 | `20261007-064208_sac_v6f_best` |
| 116 | sac_v4e_v00248(hızlı-riskli) | 1/1 | 80.668 | 3693.8 | 0 | 0 | 0 / 0.0 | 244.3 | 158.8 | `20261006-060619_sac_v4e_v00248(hızlı-riskli)` |
| 117 | sac_v6f_v00879 | 1/1 | 80.696 | 3690.8 | 0 | 0 | 0 / 0.0 | 249.9 | 158.9 | `20261007-064215_sac_v6f_v00879` |
| 118 | sac_v6f_v01759 | 1/1 | 81.056 | 3670.7 | 0 | 0 | 0 / 0.0 | 247.6 | 158.4 | `20261007-064222_sac_v6f_v01759` |
| 119 | sac_v6b_v00249 | 1/1 | 81.182 | 3664.9 | 0 | 0 | 0 / 0.0 | 242.9 | 157.4 | `20261007-022954_sac_v6b_v00249` |
| 120 | sac_v4e_v00248(hızlı-riskli)@gui-timing | 8/10 | 81.248 | 3662.7 | 0 | 0 | 0 / 0.0 | 245.2 | 157.5 | `20261006-060707_sac_v4e_v00248(hızlı-riskli)@gui-timing` |
| 121 | SUBMISSION_v6c_safe | 1/1 | 81.352 | 3659.0 | 0 | 0 | 0 / 0.0 | 251.1 | 157.5 | `20261007-064400_SUBMISSION_v6c_safe` |
| 122 | SUBMISSION_v6c_safe@gui-timing | 10/10 | 81.352 | 3659.0 | 0 | 0 | 0 / 0.0 | 251.1 | 157.5 | `20261007-064408_SUBMISSION_v6c_safe@gui-timing` |
| 123 | sac_v6c_v01404 | 1/1 | 81.424 | 3654.8 | 0 | 0 | 0 / 0.0 | 251.0 | 157.3 | `20261007-064227_sac_v6c_v01404` |
| 124 | SUBMISSION_v4d@GUI | 2/2 | 81.718 | 3617.7 | 0 | 0 | 0 / 0.0 | 251.0 | 156.1 | `20261007-002313_SUBMISSION_v4d@GUI` |
| 125 | SUBMISSION_v4d(numpy) | 1/1 | 81.736 | 3630.1 | 0 | 0 | 0 / 0.0 | 251.0 | 156.1 | `20261006-015524_SUBMISSION_v4d(numpy)` |
| 126 | SUBMISSION_v4d(numpy)@gui-timing | 10/10 | 81.736 | 3630.1 | 0 | 0 | 0 / 0.0 | 251.0 | 156.1 | `20261006-015536_SUBMISSION_v4d(numpy)@gui-timing` |
| 127 | TEST_sub44_v4d | 1/1 | 81.736 | 3630.1 | 0 | 0 | 0 / 0.0 | 251.0 | 156.1 | `20261008-234338_TEST_sub44_v4d` |
| 128 | sac_v4d_v02537 | 2/2 | 81.904 | 3623.0 | 0 | 0 | 0 / 0.0 | 251.2 | 155.8 | `20261006-015359_sac_v4d_v02537` |
| 129 | sac_v4d_v02537[agent_v2]@gui-timing | 10/10 | 82.476 | 3669.4 | 0 | 0 | 0 / 0.0 | 249.5 | 154.2 | `20261006-015452_sac_v4d_v02537[agent_v2]@gui-timing` |
| 130 | SUBMISSION_v4d(saf-python)@gui-timing | 5/5 | 82.544 | 3668.6 | 0 | 0 | 0 / 0.0 | 244.8 | 154.1 | `20261006-015619_SUBMISSION_v4d(saf-python)@gui-timing` |
| 131 | SUBMISSION_v2(numpy)@gui-timing | 4/5 | 85.694 | 3634.1 | 0 | 0 | 0 / 0.0 | 232.1 | 149.2 | `20261005-224718_SUBMISSION_v2(numpy)@gui-timing` |
| 132 | SUBMISSION_v2(numpy) | 1/1 | 85.704 | 3633.2 | 0 | 0 | 0 / 0.0 | 231.8 | 149.1 | `20261005-224646_SUBMISSION_v2(numpy)` |
| 133 | SUBMISSION_v2(saf-python) | 1/1 | 85.704 | 3633.2 | 0 | 0 | 0 / 0.0 | 231.8 | 149.1 | `20261005-224749_SUBMISSION_v2(saf-python)` |
| 134 | sac_v2_ft_v05581 | 2/2 | 85.736 | 3630.9 | 0 | 0 | 0 / 0.0 | 231.0 | 149.1 | `20261005-153101_sac_v2_ft_v05581` |
| 135 | sac_v2_ft_v05581@gui-timing | 2/10 | 85.736 | 3630.9 | 0 | 0 | 0 / 0.0 | 231.0 | 149.1 | `20261005-153157_sac_v2_ft_v05581@gui-timing` |
| 136 | sac_v2_ft_v05581[agent_v2]@gui-timing | 10/10 | 85.736 | 3630.9 | 0 | 0 | 0 / 0.0 | 231.0 | 149.1 | `20261005-153407_sac_v2_ft_v05581[agent_v2]@gui-timing` |
| 137 | sac_v2_ft_v05581[agent_v2] | 1/1 | 85.736 | 3630.9 | 0 | 0 | 0 / 0.0 | 231.0 | 149.1 | `20261005-155629_sac_v2_ft_v05581[agent_v2]` |
| 138 | sac_g995_best | 2/2 | 87.612 | 3635.4 | 0 | 0 | 0 / 0.0 | 249.1 | 145.3 | `20261005-130829_sac_g995_best` |
| 139 | sac_g995_best@gui-timing | 2/10 | 87.612 | 3635.4 | 0 | 0 | 0 / 0.0 | 249.1 | 145.3 | `20261005-131135_sac_g995_best@gui-timing` |
| 140 | sac_g995_best(53dk)@gui-timing | 2/10 | 103.642 | 3633.1 | 0 | 0 | 0 / 0.0 | 223.2 | 124.1 | `20261004-191543_sac_g995_best(53dk)@gui-timing` |
| 141 | sac_v2_ft_v05581@GUI | 2/2 | 130.346 | 7066.1 | 0 | 0 | 2 / 0.34 | 241.4 | 191.8 | `20261005-154146_sac_v2_ft_v05581@GUI` |
| 142 | sac_v2_ft_v05581[agent_v2]#etrack4 | 2/2 | 130.356 | 7162.4 | 0 | 0 | 2 / 0.34 | 241.4 | 191.9 | `20261005-154834_sac_v2_ft_v05581[agent_v2]#etrack4` |
| 143 | snakeoil3_gym#etrack4@GUI | 2/2 | 113.684 | 7065.6 | 1585 | 7 | 5 / 36.02 | 302.3 | 226.0 | `20261005-151545_snakeoil3_gym#etrack4@GUI` |
| 144 | snakeoil3_gym#etrack4 | 2/2 | 113.698 | 7095.5 | 1585 | 7 | 5 / 36.02 | 302.3 | 226.2 | `20261004-180550_snakeoil3_gym#etrack4` |
| 145 | snakeoil3_gym#etrack4@gui-timing | 4/5 | 117.944 | 7076.0 | 1754 | 8 | 5 / 41.46 | 302.5 | 217.8 | `20261004-180558_snakeoil3_gym#etrack4@gui-timing` |
| 146 | snakeoil3_gym@gui-timing | 4/25 | 134.16 | 3692.8 | 3410 | 7 | 6 / 79.3 | 247.3 | 98.7 | `20261004-180458_snakeoil3_gym@gui-timing` |
| 147 | snakeoil3_jm2@gui-timing | 1/5 | 308.23 | 3708.5 | 2680 | 8 | 8 / 90.08 | 238.8 | 36.8 | `20261004-175923_snakeoil3_jm2@gui-timing` |
| 148 | snakeoil3_jm2 | 0/2 | DNF | 3298.2 | 2921 | 8 | 7 / 399.16 | 232.7 | 26.9 | `20261004-175844_snakeoil3_jm2` |
| 149 | SUBMISSION_v2(saf-python)@gui-timing | 0/5 | DNF | 3231.9 | 1 | 1 | 1 / 359.7 | 221.1 | 25.2 | `20261005-225052_SUBMISSION_v2(saf-python)@gui-timing` |
| 150 | torcs_jm_par | 0/2 | DNF | 2511.4 | 242 | 3 | 5 / 348.44 | 101.8 | 19.7 | `20261004-180024_torcs_jm_par` |
| 151 | torcs_jm_par@gui-timing | 0/5 | DNF | 2483.4 | 247 | 3 | 6 / 360.56 | 102.9 | 19.6 | `20261004-180051_torcs_jm_par@gui-timing` |
| 152 | sac_g995_final(60dk) | 0/2 | DNF | 2471.4 | 399 | 1 | 1 / 387.86 | 253.3 | 19.4 | `20261004-191831_sac_g995_final(60dk)` |
| 153 | snakeoil3_gym | 0/2 | DNF | 2470.6 | 1863 | 5 | 5 / 410.88 | 237.9 | 20.5 | `20261004-175755_snakeoil3_gym` |
| 154 | sac_g995_final(60dk)@gui-timing | 0/10 | DNF | 1727.2 | 0 | 0 | 1 / 394.38 | 225.7 | 13.3 | `20261004-192003_sac_g995_final(60dk)@gui-timing` |
| 155 | sac_g995_best(53dk) | 0/2 | DNF | 498.3 | 217 | 1 | 1 / 427.38 | 231.7 | 4.0 | `20261004-191241_sac_g995_best(53dk)` |
| 156 | jmcncarai | 0/2 | DNF | 497.8 | 274 | 2 | 1 / 426.3 | 161.7 | 3.7 | `20261004-175935_jmcncarai` |
| 157 | jmcncarai@gui-timing | 0/5 | DNF | 497.7 | 274 | 2 | 1 / 426.3 | 161.7 | 3.7 | `20261004-175956_jmcncarai@gui-timing` |
| 158 | sac_smoke2(3dk) | 0/1 | DNF | 489.2 | 133 | 1 | 1 / 427.52 | 243.0 | 4.0 | `20261004-181753_sac_smoke2(3dk)` |

## Tüm koşular (yeniden eskiye)

| Zaman | Label | Tur (s) | Mesafe (m) | Hasar | Çarpma | Pist dışı | Max hız | Mod | Not |
|---|---|---|---|---|---|---|---|---|---|
| 20261016-080016 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.162 | 3678.6 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261016-080013 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.15 | 3679.4 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261016-080009 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.152 | 3679.3 | 0 | 0 | 5 | 298.4 | fast |  |
| 20261016-080006 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.146 | 3679.7 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261016-080003 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.166 | 3678.4 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261016-080000 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.266 | 3671.6 | 0 | 0 | 4 | 298.5 | fast |  |
| 20261016-075957 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.148 | 3679.4 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261016-075954 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.174 | 3675.8 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261016-075951 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.148 | 3679.5 | 0 | 0 | 5 | 298.4 | fast |  |
| 20261016-075948 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.148 | 3679.5 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261016-075944 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.142 | 3679.8 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261016-075941 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.152 | 3679.2 | 0 | 0 | 4 | 298.5 | fast |  |
| 20261016-075938 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.156 | 3679.0 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261016-075935 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.15 | 3679.4 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261016-075932 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.192 | 3676.5 | 0 | 0 | 4 | 298.4 | fast |  |
| 20261016-075929 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.148 | 3679.5 | 0 | 0 | 5 | 298.4 | fast |  |
| 20261016-075926 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.15 | 3679.3 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261016-075922 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.16 | 3678.6 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261016-075919 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.152 | 3679.3 | 0 | 0 | 5 | 298.4 | fast |  |
| 20261016-075916 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.154 | 3679.2 | 0 | 0 | 4 | 298.5 | fast |  |
| 20261016-075913 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.166 | 3678.3 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261016-075910 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.13 | 3680.7 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261016-075906 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.206 | 3675.5 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261016-075903 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.116 | 3681.6 | 0 | 0 | 4 | 298.6 | fast |  |
| 20261016-075900 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.152 | 3679.3 | 0 | 0 | 4 | 298.5 | fast |  |
| 20261016-075857 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.144 | 3679.7 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261016-075852 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.184 | 3677.1 | 0 | 0 | 5 | 298.4 | fast |  |
| 20261016-075849 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.156 | 3678.9 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261016-075845 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.158 | 3678.9 | 0 | 0 | 5 | 298.4 | fast |  |
| 20261016-075842 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.15 | 3679.4 | 0 | 0 | 5 | 298.4 | fast |  |
| 20261016-075837 | FINAL_torcs_jm_par_hybrid_res3a.py_det | 64.878 | 3697.4 | 0 | 0 | 7 | 298.6 | fast |  |
| 20261016-075829 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | DNF | 785.3 | 789 | 4 | 7 | 274.8 | fast |  |
| 20261016-075822 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | DNF | 702.6 | 876 | 3 | 3 | 274.8 | fast |  |
| 20261016-075815 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | DNF | 635.9 | 552 | 3 | 4 | 274.8 | fast |  |
| 20261016-075812 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | 64.896 | 3697.8 | 0 | 0 | 6 | 298.7 | fast |  |
| 20261016-075805 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | DNF | 713.4 | 359 | 4 | 6 | 274.8 | fast |  |
| 20261016-075758 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | DNF | 699.4 | 659 | 6 | 8 | 274.9 | fast |  |
| 20261016-075756 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | 64.378 | 3731.3 | 13 | 1 | 7 | 298.6 | fast |  |
| 20261016-075753 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | 64.896 | 3697.8 | 1 | 1 | 8 | 298.6 | fast |  |
| 20261016-075749 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | 84.296 | 3737.1 | 244 | 5 | 13 | 292.6 | fast |  |
| 20261016-075747 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | 64.542 | 3721.1 | 0 | 0 | 10 | 298.7 | fast |  |
| 20261016-075744 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | 64.61 | 3716.6 | 14 | 1 | 8 | 298.6 | fast |  |
| 20261016-075741 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | 64.236 | 3740.3 | 0 | 0 | 7 | 298.6 | fast |  |
| 20261016-075733 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | DNF | 695.9 | 235 | 6 | 4 | 274.9 | fast |  |
| 20261016-075730 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | 63.974 | 3621.2 | 0 | 0 | 9 | 298.8 | fast |  |
| 20261016-075723 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | DNF | 699.4 | 247 | 3 | 4 | 274.9 | fast |  |
| 20261016-075716 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | DNF | 649.4 | 153 | 2 | 4 | 274.8 | fast |  |
| 20261016-075709 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | DNF | 659.1 | 460 | 3 | 4 | 274.8 | fast |  |
| 20261016-075702 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | DNF | 699.4 | 1290 | 5 | 7 | 274.8 | fast |  |
| 20261016-075654 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | DNF | 652.7 | 901 | 4 | 6 | 274.9 | fast |  |
| 20261016-075651 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | 64.894 | 3698.0 | 0 | 0 | 7 | 298.7 | fast |  |
| 20261016-075644 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | DNF | 719.4 | 1236 | 9 | 11 | 275.0 | fast |  |
| 20261016-075640 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | 64.89 | 3698.3 | 1 | 1 | 7 | 298.7 | fast |  |
| 20261016-075633 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | DNF | 719.4 | 2048 | 8 | 6 | 274.8 | fast |  |
| 20261016-075626 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | DNF | 759.9 | 861 | 3 | 7 | 274.6 | fast |  |
| 20261016-075619 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | DNF | 808.9 | 467 | 4 | 5 | 274.9 | fast |  |
| 20261016-075615 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | 64.682 | 3712.0 | 13 | 1 | 8 | 298.6 | fast |  |
| 20261016-075612 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | 64.3 | 3736.2 | 0 | 0 | 8 | 298.7 | fast |  |
| 20261016-075604 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | DNF | 706.0 | 1594 | 12 | 7 | 274.8 | fast |  |
| 20261016-075601 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | 64.936 | 3695.1 | 14 | 1 | 7 | 298.7 | fast |  |
| 20261016-075554 | FINAL_torcs_jm_par_line_g7a.py_x30@gui-timing | DNF | 687.9 | 51 | 1 | 2 | 274.9 | fast |  |
| 20261016-075548 | FINAL_torcs_jm_par_line_g7a.py_det | 63.928 | 3624.7 | 0 | 0 | 10 | 298.8 | fast |  |
| 20261016-075541 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | DNF | 611.8 | 219 | 5 | 10 | 274.4 | fast |  |
| 20261016-075534 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | DNF | 561.6 | 279 | 8 | 17 | 274.4 | fast |  |
| 20261016-075527 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | DNF | 571.7 | 200 | 4 | 8 | 274.4 | fast |  |
| 20261016-075524 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | 64.338 | 3733.9 | 0 | 0 | 11 | 298.8 | fast |  |
| 20261016-075517 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | DNF | 575.5 | 227 | 4 | 12 | 274.4 | fast |  |
| 20261016-075510 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | DNF | 521.3 | 590 | 9 | 26 | 274.4 | fast |  |
| 20261016-075503 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | DNF | 649.4 | 1381 | 5 | 3 | 274.4 | fast |  |
| 20261016-075455 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | DNF | 530.3 | 217 | 2 | 13 | 274.4 | fast |  |
| 20261016-075447 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | DNF | 672.8 | 247 | 4 | 3 | 274.1 | fast |  |
| 20261016-075440 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | DNF | 585.3 | 277 | 5 | 15 | 274.4 | fast |  |
| 20261016-075433 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | DNF | 619.5 | 247 | 2 | 2 | 274.4 | fast |  |
| 20261016-075426 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | DNF | 545.9 | 63 | 4 | 14 | 274.4 | fast |  |
| 20261016-075419 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | DNF | 557.9 | 238 | 4 | 9 | 274.4 | fast |  |
| 20261016-075411 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | DNF | 664.8 | 1056 | 7 | 10 | 274.4 | fast |  |
| 20261016-075403 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | DNF | 550.7 | 271 | 6 | 23 | 274.4 | fast |  |
| 20261016-075400 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | 63.658 | 3644.6 | 0 | 0 | 14 | 299.0 | fast |  |
| 20261016-075356 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | 63.888 | 3627.6 | 0 | 0 | 12 | 299.1 | fast |  |
| 20261016-075349 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | DNF | 525.0 | 253 | 10 | 38 | 274.4 | fast |  |
| 20261016-075341 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | DNF | 589.4 | 199 | 2 | 2 | 274.4 | fast |  |
| 20261016-075334 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | DNF | 642.7 | 2151 | 3 | 2 | 274.4 | fast |  |
| 20261016-075326 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | DNF | 537.3 | 253 | 8 | 23 | 274.4 | fast |  |
| 20261016-075319 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | DNF | 524.6 | 267 | 13 | 26 | 274.4 | fast |  |
| 20261016-075315 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | 64.144 | 3745.8 | 1 | 1 | 10 | 298.7 | fast |  |
| 20261016-075308 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | DNF | 533.9 | 197 | 2 | 16 | 274.4 | fast |  |
| 20261016-075301 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | DNF | 564.9 | 215 | 3 | 6 | 274.4 | fast |  |
| 20261016-075254 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | DNF | 531.7 | 221 | 5 | 12 | 274.4 | fast |  |
| 20261016-075247 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | DNF | 564.9 | 251 | 6 | 26 | 274.4 | fast |  |
| 20261016-075240 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | DNF | 656.0 | 512 | 5 | 2 | 274.4 | fast |  |
| 20261016-075232 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | DNF | 649.5 | 1206 | 9 | 16 | 274.4 | fast |  |
| 20261016-075225 | FINAL_torcs_jm_par_line_g9a.py_x30@gui-timing | DNF | 551.1 | 261 | 4 | 10 | 274.4 | fast |  |
| 20261016-075220 | FINAL_torcs_jm_par_line_g9a.py_det | 63.628 | 3646.8 | 0 | 0 | 14 | 299.0 | fast |  |
| 20261016-075203 | B9_g9a_det3 | 63.628 | 3646.8 | 0 | 0 | 14 | 299.0 | fast |  |
| 20261016-075158 | B9_g9a_det2 | 63.628 | 3646.8 | 0 | 0 | 14 | 299.0 | fast |  |
| 20261016-075154 | B9_g9a_det1 | 63.628 | 3646.8 | 0 | 0 | 14 | 299.0 | fast |  |
| 20261016-032314 | B9_g7a_det1 | 63.928 | 3624.7 | 0 | 0 | 10 | 298.8 | fast |  |
| 20261016-003129 | B9_g6a_det2 | 64.026 | 3753.2 | 0 | 0 | 10 | 299.1 | fast |  |
| 20261016-003123 | B9_g6a_det1 | 64.026 | 3753.2 | 0 | 0 | 10 | 299.1 | fast |  |
| 20261015-040927 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.148 | 3679.5 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261015-040924 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.154 | 3679.0 | 0 | 0 | 4 | 298.5 | fast |  |
| 20261015-040920 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 64.972 | 3691.2 | 0 | 0 | 6 | 298.6 | fast |  |
| 20261015-040917 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.116 | 3681.7 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261015-040913 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.138 | 3680.2 | 0 | 0 | 5 | 298.6 | fast |  |
| 20261015-040910 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.152 | 3679.2 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261015-040906 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.188 | 3676.8 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261015-040903 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.144 | 3679.8 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261015-040900 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.148 | 3679.5 | 0 | 0 | 5 | 298.4 | fast |  |
| 20261015-040856 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.112 | 3681.9 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261015-040853 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.166 | 3678.4 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261015-040849 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.15 | 3679.4 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261015-040846 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.088 | 3683.5 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261015-040842 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.146 | 3679.6 | 0 | 0 | 5 | 298.4 | fast |  |
| 20261015-040838 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 64.878 | 3697.4 | 0 | 0 | 7 | 298.6 | fast |  |
| 20261015-040835 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.152 | 3679.3 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261015-040831 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.146 | 3679.7 | 0 | 0 | 4 | 298.5 | fast |  |
| 20261015-040828 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.162 | 3678.6 | 0 | 0 | 5 | 298.4 | fast |  |
| 20261015-040824 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.152 | 3679.3 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261015-040820 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.098 | 3682.9 | 0 | 0 | 5 | 298.6 | fast |  |
| 20261015-040817 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.164 | 3678.4 | 0 | 0 | 6 | 298.1 | fast |  |
| 20261015-040813 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.156 | 3679.0 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261015-040809 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.152 | 3679.3 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261015-040806 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.15 | 3679.4 | 0 | 0 | 4 | 298.5 | fast |  |
| 20261015-040803 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.15 | 3679.4 | 0 | 0 | 4 | 298.5 | fast |  |
| 20261015-040759 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.148 | 3679.4 | 0 | 0 | 5 | 298.4 | fast |  |
| 20261015-040756 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.152 | 3679.2 | 0 | 0 | 4 | 298.4 | fast |  |
| 20261015-040753 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.154 | 3679.2 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261015-040749 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.144 | 3679.7 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261015-040745 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.09 | 3683.4 | 0 | 0 | 5 | 298.6 | fast |  |
| 20261015-040740 | FINAL_torcs_jm_par_hybrid_res3a.py_det | 64.878 | 3697.4 | 0 | 0 | 7 | 298.6 | fast |  |
| 20261015-040737 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 66.156 | 3745.2 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261015-040734 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 65.534 | 3653.7 | 1 | 1 | 3 | 299.0 | fast |  |
| 20261015-040731 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.978 | 3692.2 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261015-040727 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 66.174 | 3744.1 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261015-040720 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | DNF | 539.8 | 10 | 1 | 1 | 274.2 | fast |  |
| 20261015-040712 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | DNF | 498.6 | 1 | 1 | 3 | 274.2 | fast |  |
| 20261015-040709 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 65.15 | 3680.5 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261015-040701 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | DNF | 578.6 | 17 | 1 | 10 | 274.2 | fast |  |
| 20261015-040658 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 65.996 | 3619.6 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261015-040654 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 66.294 | 3736.7 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261015-040651 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 65.012 | 3689.9 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261015-040647 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.952 | 3693.9 | 0 | 0 | 4 | 299.1 | fast |  |
| 20261015-040644 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 65.778 | 3635.9 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261015-040640 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.932 | 3695.3 | 0 | 0 | 4 | 299.0 | fast |  |
| 20261015-040632 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | DNF | 840.2 | 43 | 4 | 2 | 274.2 | fast |  |
| 20261015-040629 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 65.534 | 3653.7 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261015-040625 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.802 | 3704.0 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261015-040621 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 65.76 | 3637.3 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261015-040618 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 65.972 | 3621.5 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261015-040615 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 66.01 | 3617.7 | 0 | 0 | 3 | 299.1 | fast |  |
| 20261015-040612 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 65.958 | 3622.4 | 0 | 0 | 3 | 298.1 | fast |  |
| 20261015-040604 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | DNF | 710.5 | 85 | 1 | 5 | 274.2 | fast |  |
| 20261015-040601 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 65.632 | 3646.6 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261015-040557 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 65.976 | 3621.1 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261015-040554 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 66.052 | 3751.5 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261015-040550 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.936 | 3695.1 | 0 | 0 | 4 | 299.0 | fast |  |
| 20261015-040547 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.994 | 3691.2 | 0 | 0 | 4 | 299.0 | fast |  |
| 20261015-040543 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.81 | 3703.4 | 0 | 0 | 4 | 299.1 | fast |  |
| 20261015-040540 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 66.276 | 3737.7 | 0 | 0 | 2 | 299.1 | fast |  |
| 20261015-040533 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | DNF | 902.6 | 322 | 6 | 10 | 274.2 | fast |  |
| 20261015-040528 | FINAL_torcs_jm_par_line_t3off.py_det | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261015-040524 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 65.98 | 3620.8 | 0 | 0 | 2 | 299.1 | fast |  |
| 20261015-040521 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 65.016 | 3689.7 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261015-040517 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 65.976 | 3621.1 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261015-040514 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 66.01 | 3618.3 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261015-040511 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 65.012 | 3689.9 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261015-040507 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 65.672 | 3643.7 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261015-040504 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 65.312 | 3669.4 | 0 | 0 | 2 | 299.1 | fast |  |
| 20261015-040501 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 64.944 | 3694.4 | 0 | 0 | 3 | 299.1 | fast |  |
| 20261015-040457 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 65.974 | 3621.2 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261015-040454 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 65.974 | 3621.2 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261015-040450 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 64.894 | 3697.8 | 0 | 0 | 3 | 299.2 | fast |  |
| 20261015-040447 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 65.824 | 3632.6 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261015-040444 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 65.994 | 3619.7 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261015-040440 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 64.934 | 3695.2 | 0 | 0 | 4 | 299.0 | fast |  |
| 20261015-040432 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | DNF | 653.1 | 761 | 4 | 14 | 274.2 | fast |  |
| 20261015-040429 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 64.954 | 3693.8 | 0 | 0 | 4 | 299.0 | fast |  |
| 20261015-040425 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 64.804 | 3703.8 | 0 | 0 | 4 | 299.1 | fast |  |
| 20261015-040422 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 65.586 | 3650.0 | 0 | 0 | 2 | 299.1 | fast |  |
| 20261015-040414 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | DNF | 653.0 | 486 | 3 | 4 | 274.2 | fast |  |
| 20261015-040410 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 64.892 | 3698.0 | 0 | 0 | 3 | 299.2 | fast |  |
| 20261015-040402 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | DNF | 902.7 | 319 | 7 | 5 | 274.2 | fast |  |
| 20261015-040359 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 66.038 | 3752.3 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261015-040355 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 64.932 | 3695.3 | 0 | 0 | 4 | 299.0 | fast |  |
| 20261015-040352 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 65.872 | 3629.0 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261015-040348 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 65.972 | 3621.4 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261015-040345 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 64.812 | 3703.3 | 0 | 0 | 4 | 299.1 | fast |  |
| 20261015-040337 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | DNF | 516.4 | 33 | 1 | 1 | 274.2 | fast |  |
| 20261015-040334 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 65.816 | 3633.2 | 0 | 0 | 2 | 299.1 | fast |  |
| 20261015-040330 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 64.914 | 3696.6 | 0 | 0 | 3 | 299.1 | fast |  |
| 20261015-040326 | FINAL_torcs_jm_par_line_t4off.py_x30@gui-timing | 65.88 | 3628.4 | 0 | 0 | 2 | 299.1 | fast |  |
| 20261015-040321 | FINAL_torcs_jm_par_line_t4off.py_det | 64.722 | 3709.3 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233606 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233602 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233558 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233554 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233550 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233546 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233542 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233538 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233533 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233529 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233524 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233520 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233516 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233511 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233507 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233503 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233459 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233455 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233451 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233447 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233443 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233439 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233435 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233431 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233427 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233423 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233418 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233414 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233409 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233405 | FINAL_torcs_jm_par_line_t3off.py_x30@gui-timing | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-233400 | FINAL_torcs_jm_par_line_t3off.py_det | 64.73 | 3708.8 | 0 | 0 | 5 | 299.1 | fast |  |
| 20261014-181828 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.838 | 3631.5 | 0 | 0 | 3 | 298.9 | fast |  |
| 20261014-181825 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.578 | 3650.5 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-181822 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.578 | 3650.6 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-181820 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.618 | 3647.7 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-181817 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.554 | 3652.2 | 0 | 0 | 2 | 299.2 | fast |  |
| 20261014-181815 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.578 | 3650.5 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-181812 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.568 | 3651.3 | 0 | 0 | 2 | 298.9 | fast |  |
| 20261014-181809 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.588 | 3649.7 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-181807 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.576 | 3650.7 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-181804 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.588 | 3649.8 | 0 | 0 | 3 | 298.9 | fast |  |
| 20261014-181801 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.718 | 3640.4 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-181758 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.578 | 3650.5 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-181756 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.576 | 3650.7 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-181753 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.596 | 3649.3 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-181750 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.598 | 3649.1 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-181747 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.578 | 3650.5 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-181745 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.564 | 3651.5 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-181742 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.576 | 3650.6 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-181739 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-181736 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.578 | 3650.5 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-181733 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.318 | 3668.9 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-181731 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.578 | 3650.5 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-181728 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.158 | 3680.0 | 0 | 0 | 1 | 299.1 | fast |  |
| 20261014-181725 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.576 | 3650.7 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-181722 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.578 | 3650.5 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-181720 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.612 | 3648.1 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-181717 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.576 | 3650.7 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-181715 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.572 | 3651.0 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-181712 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.578 | 3650.5 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-181709 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.578 | 3650.5 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-181706 | FINAL_torcs_jm_par_line_gr3L.py_det | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-181703 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.146 | 3679.7 | 0 | 0 | 4 | 298.5 | fast |  |
| 20261014-181700 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.166 | 3678.3 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261014-181656 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.156 | 3678.9 | 0 | 0 | 5 | 298.4 | fast |  |
| 20261014-181654 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.152 | 3679.2 | 0 | 0 | 4 | 298.5 | fast |  |
| 20261014-181650 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.106 | 3682.4 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261014-181647 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.152 | 3679.2 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261014-181644 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.15 | 3679.4 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261014-181641 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.18 | 3677.4 | 0 | 0 | 5 | 298.7 | fast |  |
| 20261014-181638 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.132 | 3680.6 | 0 | 0 | 4 | 298.5 | fast |  |
| 20261014-181635 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.176 | 3677.6 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261014-181632 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.158 | 3678.9 | 0 | 0 | 5 | 298.6 | fast |  |
| 20261014-181629 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.15 | 3679.3 | 0 | 0 | 5 | 298.4 | fast |  |
| 20261014-181626 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.232 | 3673.8 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261014-181622 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.152 | 3679.3 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261014-181620 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.142 | 3679.9 | 0 | 0 | 5 | 298.4 | fast |  |
| 20261014-181617 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.144 | 3679.7 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261014-181614 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.254 | 3672.3 | 0 | 0 | 5 | 298.4 | fast |  |
| 20261014-181611 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.162 | 3678.6 | 0 | 0 | 5 | 298.6 | fast |  |
| 20261014-181607 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.154 | 3679.2 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261014-181604 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.164 | 3678.4 | 0 | 0 | 5 | 298.4 | fast |  |
| 20261014-181601 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.16 | 3678.7 | 0 | 0 | 5 | 298.6 | fast |  |
| 20261014-181558 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.248 | 3672.7 | 0 | 0 | 4 | 298.4 | fast |  |
| 20261014-181556 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.148 | 3679.5 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261014-181553 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.154 | 3679.1 | 0 | 0 | 4 | 298.5 | fast |  |
| 20261014-181550 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.164 | 3678.4 | 0 | 0 | 5 | 298.6 | fast |  |
| 20261014-181547 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.19 | 3676.6 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261014-181544 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.148 | 3679.5 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261014-181541 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.168 | 3678.2 | 0 | 0 | 4 | 298.4 | fast |  |
| 20261014-181538 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.15 | 3679.3 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261014-181535 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.164 | 3678.3 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261014-181531 | FINAL_torcs_jm_par_hybrid_res3a.py_det | 64.878 | 3697.4 | 0 | 0 | 7 | 298.6 | fast |  |
| 20261014-180209 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.132 | 3681.8 | 0 | 0 | 1 | 299.1 | fast |  |
| 20261014-180203 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.158 | 3680.1 | 0 | 0 | 2 | 298.9 | fast |  |
| 20261014-180159 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.272 | 3672.1 | 0 | 0 | 1 | 294.8 | fast |  |
| 20261014-180154 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.694 | 3642.1 | 0 | 0 | 2 | 299.2 | fast |  |
| 20261014-180150 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.138 | 3681.3 | 0 | 0 | 1 | 299.1 | fast |  |
| 20261014-180145 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.122 | 3682.5 | 1 | 1 | 3 | 298.9 | fast |  |
| 20261014-180141 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.342 | 3667.3 | 0 | 0 | 1 | 299.2 | fast |  |
| 20261014-180136 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.218 | 3675.9 | 0 | 0 | 3 | 298.8 | fast |  |
| 20261014-180132 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.216 | 3676.1 | 0 | 0 | 2 | 298.9 | fast |  |
| 20261014-180127 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.48 | 3657.5 | 0 | 0 | 0 | 299.0 | fast |  |
| 20261014-180122 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.578 | 3650.5 | 0 | 0 | 1 | 298.9 | fast |  |
| 20261014-180118 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.156 | 3680.2 | 0 | 0 | 1 | 299.1 | fast |  |
| 20261014-180113 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.16 | 3679.8 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-180109 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.538 | 3653.4 | 0 | 0 | 1 | 299.2 | fast |  |
| 20261014-180104 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.132 | 3681.9 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-180100 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.132 | 3681.9 | 0 | 0 | 0 | 299.0 | fast |  |
| 20261014-180055 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.096 | 3684.3 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-180051 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.09 | 3684.8 | 0 | 0 | 1 | 299.1 | fast |  |
| 20261014-180046 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.234 | 3674.8 | 2 | 1 | 1 | 299.0 | fast |  |
| 20261014-180042 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.46 | 3659.0 | 0 | 0 | 1 | 299.2 | fast |  |
| 20261014-180037 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.264 | 3672.2 | 0 | 0 | 2 | 299.2 | fast |  |
| 20261014-180032 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.498 | 3656.2 | 0 | 0 | 2 | 299.1 | fast |  |
| 20261014-180028 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.256 | 3673.2 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-180024 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.256 | 3673.2 | 0 | 0 | 1 | 299.0 | fast |  |
| 20261014-180019 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.524 | 3654.4 | 1 | 1 | 1 | 297.8 | fast |  |
| 20261014-180015 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.274 | 3672.1 | 0 | 0 | 1 | 299.0 | fast |  |
| 20261014-180011 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.106 | 3683.6 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-175950 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | DNF | 696.3 | 903 | 7 | 9 | 273.9 | fast |  |
| 20261014-175946 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.428 | 3661.2 | 0 | 0 | 1 | 293.8 | fast |  |
| 20261014-175942 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.448 | 3659.8 | 0 | 0 | 2 | 298.9 | fast |  |
| 20261014-175937 | FINAL_torcs_jm_par_line_gr3L.py_det | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-175922 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.07 | 3684.8 | 1 | 1 | 6 | 298.4 | fast |  |
| 20261014-175917 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.036 | 3686.8 | 0 | 0 | 6 | 298.7 | fast |  |
| 20261014-175912 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 64.994 | 3690.0 | 0 | 0 | 5 | 298.6 | fast |  |
| 20261014-175907 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.038 | 3686.9 | 10 | 1 | 6 | 298.6 | fast |  |
| 20261014-175902 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.068 | 3685.0 | 0 | 0 | 6 | 298.8 | fast |  |
| 20261014-175857 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.086 | 3683.8 | 0 | 0 | 5 | 298.8 | fast |  |
| 20261014-175852 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.02 | 3688.2 | 0 | 0 | 6 | 298.7 | fast |  |
| 20261014-175847 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.036 | 3687.1 | 0 | 0 | 6 | 298.4 | fast |  |
| 20261014-175842 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.018 | 3688.2 | 0 | 0 | 6 | 298.5 | fast |  |
| 20261014-175837 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.12 | 3681.4 | 0 | 0 | 6 | 298.4 | fast |  |
| 20261014-175832 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.034 | 3687.1 | 0 | 0 | 6 | 298.7 | fast |  |
| 20261014-175827 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.108 | 3682.0 | 0 | 0 | 5 | 298.5 | fast |  |
| 20261014-175822 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.008 | 3688.7 | 0 | 0 | 5 | 298.8 | fast |  |
| 20261014-175818 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.02 | 3688.2 | 0 | 0 | 3 | 298.5 | fast |  |
| 20261014-175813 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.096 | 3683.0 | 0 | 0 | 5 | 298.7 | fast |  |
| 20261014-175809 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.112 | 3681.9 | 0 | 0 | 5 | 298.6 | fast |  |
| 20261014-175804 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.012 | 3688.6 | 12 | 1 | 6 | 298.5 | fast |  |
| 20261014-175759 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.054 | 3685.7 | 0 | 0 | 6 | 298.7 | fast |  |
| 20261014-175754 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 64.986 | 3690.3 | 0 | 0 | 6 | 298.5 | fast |  |
| 20261014-175749 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.094 | 3683.2 | 0 | 0 | 5 | 298.6 | fast |  |
| 20261014-175744 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.1 | 3682.6 | 0 | 0 | 6 | 298.7 | fast |  |
| 20261014-175739 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.178 | 3677.5 | 0 | 0 | 4 | 298.6 | fast |  |
| 20261014-175734 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.158 | 3678.8 | 0 | 0 | 5 | 298.7 | fast |  |
| 20261014-175729 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.05 | 3686.0 | 0 | 0 | 6 | 298.5 | fast |  |
| 20261014-175724 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.156 | 3678.9 | 0 | 0 | 5 | 298.6 | fast |  |
| 20261014-175719 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 64.978 | 3690.9 | 0 | 0 | 6 | 298.7 | fast |  |
| 20261014-175714 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 64.978 | 3691.0 | 0 | 0 | 7 | 298.6 | fast |  |
| 20261014-175709 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.112 | 3681.9 | 0 | 0 | 7 | 298.4 | fast |  |
| 20261014-175704 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.118 | 3681.5 | 0 | 0 | 4 | 298.5 | fast |  |
| 20261014-175659 | FINAL_torcs_jm_par_hybrid_res3a.py_x30@gui-timing | 65.04 | 3686.8 | 0 | 0 | 7 | 298.4 | fast |  |
| 20261014-175653 | FINAL_torcs_jm_par_hybrid_res3a.py_det | 64.878 | 3697.4 | 0 | 0 | 7 | 298.6 | fast |  |
| 20261014-164420 | B6_hybrid_parity | 65.002 | 3689.1 | 0 | 0 | 6 | 295.7 | fast |  |
| 20261014-153513 | line_gr3L@GUI | 64.954 | 3617.7 | 0 | 0 | 0 | 299.1 | gui |  |
| 20261014-114416 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.484 | 3657.3 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-114413 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.466 | 3658.6 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-114409 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.474 | 3658.0 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-114405 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.526 | 3654.3 | 0 | 0 | 3 | 299.1 | fast |  |
| 20261014-114402 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.734 | 3639.3 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-114358 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 64.952 | 3694.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-114355 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.126 | 3682.2 | 0 | 0 | 3 | 299.1 | fast |  |
| 20261014-114351 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.106 | 3683.6 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-114347 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.724 | 3639.9 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-114344 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.082 | 3685.2 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-114339 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.106 | 3683.6 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-114335 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.104 | 3683.8 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-114331 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.106 | 3683.6 | 0 | 0 | 3 | 299.1 | fast |  |
| 20261014-114327 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.608 | 3648.3 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-114324 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.07 | 3686.1 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-114321 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 64.958 | 3693.6 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-114317 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.464 | 3658.7 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-114313 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.38 | 3664.8 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-114309 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.79 | 3635.1 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-114305 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.79 | 3635.1 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-114301 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.454 | 3659.4 | 0 | 0 | 1 | 299.2 | fast |  |
| 20261014-114258 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.466 | 3658.6 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-114254 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.45 | 3659.7 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-114250 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.804 | 3634.1 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-114246 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.73 | 3639.6 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-114243 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.73 | 3639.6 | 0 | 0 | 2 | 299.1 | fast |  |
| 20261014-114239 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.126 | 3682.3 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-114235 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.084 | 3685.1 | 0 | 0 | 2 | 299.2 | fast |  |
| 20261014-114232 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.804 | 3634.0 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-114228 | FINAL_torcs_jm_par_line_t1.py_x30@gui-timing | 65.05 | 3687.6 | 0 | 0 | 3 | 298.9 | fast |  |
| 20261014-114224 | FINAL_torcs_jm_par_line_t1.py_det | 64.874 | 3699.3 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-114151 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 65.404 | 3662.9 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-114147 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 65.604 | 3648.6 | 0 | 0 | 2 | 299.1 | fast |  |
| 20261014-114143 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 65.058 | 3686.8 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-114139 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 64.926 | 3695.7 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-114135 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 65.744 | 3638.4 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-114132 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 65.418 | 3661.9 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-114128 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 64.996 | 3691.2 | 0 | 0 | 2 | 299.1 | fast |  |
| 20261014-114124 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 65.664 | 3644.3 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-114120 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 65.038 | 3688.2 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-114117 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 65.692 | 3642.3 | 0 | 0 | 1 | 299.1 | fast |  |
| 20261014-114113 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 65.42 | 3661.8 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-114110 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 64.904 | 3697.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-114105 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 65.05 | 3687.4 | 0 | 0 | 1 | 299.1 | fast |  |
| 20261014-114101 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 65.724 | 3639.9 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-114058 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 65.192 | 3677.6 | 0 | 0 | 1 | 299.1 | fast |  |
| 20261014-114054 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 65.026 | 3689.0 | 0 | 0 | 1 | 299.2 | fast |  |
| 20261014-114051 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 65.04 | 3688.0 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-114047 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 65.42 | 3661.7 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-114044 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 65.412 | 3662.4 | 0 | 0 | 1 | 299.0 | fast |  |
| 20261014-114040 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 65.018 | 3689.5 | 0 | 0 | 1 | 299.1 | fast |  |
| 20261014-114037 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 65.404 | 3662.9 | 0 | 0 | 1 | 299.0 | fast |  |
| 20261014-114034 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 65.42 | 3661.8 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-114030 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 65.084 | 3685.0 | 0 | 0 | 2 | 299.1 | fast |  |
| 20261014-114026 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 65.422 | 3661.7 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-114023 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 65.692 | 3642.4 | 0 | 0 | 1 | 299.0 | fast |  |
| 20261014-114019 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 64.876 | 3699.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-114015 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 65.082 | 3685.2 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-114011 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 65.7 | 3641.8 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-114008 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 65.724 | 3639.9 | 0 | 0 | 2 | 299.1 | fast |  |
| 20261014-114005 | FINAL_torcs_jm_par_line_t2.py_x30@gui-timing | 65.422 | 3661.7 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-114000 | FINAL_torcs_jm_par_line_t2.py_det | 64.84 | 3701.6 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-111505 | line_gr3L@GUI | 64.954 | 3617.7 | 0 | 0 | 0 | 299.1 | gui |  |
| 20261014-102706 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102702 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102658 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102654 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102650 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102645 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102641 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102637 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102633 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102629 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102626 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102622 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102618 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102614 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102610 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102606 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102602 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102558 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102554 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102550 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102545 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102541 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102537 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102533 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102529 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102525 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102522 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102518 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102514 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102510 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102504 | FINAL_torcs_jm_par_line_gr2.py_det | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102500 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102456 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102452 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102449 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102445 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102441 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102437 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102433 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102429 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102425 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.158 | 3680.0 | 0 | 0 | 3 | 298.9 | fast |  |
| 20261014-102421 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102417 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102413 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102409 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102405 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102401 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102357 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102353 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102349 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102345 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102341 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102336 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102332 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102328 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102324 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102319 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102315 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102311 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102307 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102303 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102257 | FINAL_torcs_jm_par_line_gr3L.py_det | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-102132 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 64.778 | 3705.6 | 0 | 0 | 0 | 298.9 | fast |  |
| 20261014-102128 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 64.778 | 3705.6 | 0 | 0 | 0 | 298.9 | fast |  |
| 20261014-102124 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 64.778 | 3705.6 | 0 | 0 | 0 | 298.9 | fast |  |
| 20261014-102120 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 64.778 | 3705.6 | 0 | 0 | 0 | 298.9 | fast |  |
| 20261014-102116 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 64.778 | 3705.6 | 0 | 0 | 0 | 298.9 | fast |  |
| 20261014-102111 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 64.778 | 3705.6 | 0 | 0 | 0 | 298.9 | fast |  |
| 20261014-102107 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 64.778 | 3705.6 | 0 | 0 | 0 | 298.9 | fast |  |
| 20261014-102104 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 65.022 | 3689.2 | 0 | 0 | 4 | 298.5 | fast |  |
| 20261014-102100 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 64.778 | 3705.6 | 0 | 0 | 0 | 298.9 | fast |  |
| 20261014-102056 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 64.778 | 3705.6 | 0 | 0 | 0 | 298.9 | fast |  |
| 20261014-102052 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 64.778 | 3705.6 | 0 | 0 | 0 | 298.9 | fast |  |
| 20261014-102048 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 64.778 | 3705.6 | 0 | 0 | 0 | 298.9 | fast |  |
| 20261014-102045 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 65.016 | 3689.7 | 0 | 0 | 4 | 298.5 | fast |  |
| 20261014-102041 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 65.066 | 3686.2 | 0 | 0 | 4 | 297.8 | fast |  |
| 20261014-102038 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 64.996 | 3691.0 | 0 | 0 | 4 | 298.4 | fast |  |
| 20261014-102034 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 64.844 | 3701.2 | 0 | 0 | 1 | 298.9 | fast |  |
| 20261014-102031 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 64.998 | 3690.9 | 0 | 0 | 4 | 298.5 | fast |  |
| 20261014-102028 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 64.938 | 3694.9 | 0 | 0 | 3 | 298.5 | fast |  |
| 20261014-102024 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 65.03 | 3688.7 | 0 | 0 | 4 | 298.5 | fast |  |
| 20261014-102021 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 65.026 | 3689.0 | 0 | 0 | 4 | 298.4 | fast |  |
| 20261014-102018 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 65.196 | 3677.4 | 0 | 0 | 4 | 298.5 | fast |  |
| 20261014-102014 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 64.958 | 3693.6 | 0 | 0 | 2 | 298.3 | fast |  |
| 20261014-102011 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 64.998 | 3690.9 | 0 | 0 | 4 | 298.4 | fast |  |
| 20261014-102007 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 65.016 | 3689.7 | 0 | 0 | 4 | 298.5 | fast |  |
| 20261014-102003 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 65.016 | 3689.6 | 0 | 0 | 4 | 298.5 | fast |  |
| 20261014-102000 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 65.018 | 3689.6 | 0 | 0 | 4 | 298.4 | fast |  |
| 20261014-101957 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 65.03 | 3688.7 | 0 | 0 | 4 | 298.4 | fast |  |
| 20261014-101953 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 64.99 | 3691.4 | 1 | 1 | 3 | 297.7 | fast |  |
| 20261014-101950 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 64.938 | 3695.0 | 0 | 0 | 4 | 298.5 | fast |  |
| 20261014-101947 | FINAL_torcs_jm_par_line_g5r0.py_x30@gui-timing | 65.02 | 3689.4 | 0 | 0 | 4 | 298.4 | fast |  |
| 20261014-101943 | FINAL_torcs_jm_par_line_g5r0.py_det | 64.778 | 3705.6 | 0 | 0 | 0 | 298.9 | fast |  |
| 20261014-101939 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.858 | 3700.3 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-101935 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.718 | 3709.6 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-101932 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 2 | 299.1 | fast |  |
| 20261014-101930 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.986 | 3691.8 | 0 | 0 | 3 | 299.1 | fast |  |
| 20261014-101927 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.968 | 3693.0 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-101924 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 2 | 299.1 | fast |  |
| 20261014-101921 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.968 | 3693.1 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-101918 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.966 | 3693.2 | 0 | 0 | 2 | 299.1 | fast |  |
| 20261014-101915 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.836 | 3701.8 | 0 | 0 | 2 | 299.1 | fast |  |
| 20261014-101912 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.98 | 3692.2 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-101909 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.97 | 3692.8 | 0 | 0 | 2 | 299.1 | fast |  |
| 20261014-101906 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.846 | 3701.1 | 0 | 0 | 2 | 299.1 | fast |  |
| 20261014-101904 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.968 | 3692.9 | 0 | 0 | 2 | 299.1 | fast |  |
| 20261014-101901 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.964 | 3693.3 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-101858 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.966 | 3693.2 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-101855 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.928 | 3695.7 | 0 | 0 | 2 | 299.1 | fast |  |
| 20261014-101852 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.906 | 3697.3 | 4 | 1 | 1 | 299.0 | fast |  |
| 20261014-101849 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 65.022 | 3689.5 | 0 | 0 | 2 | 299.1 | fast |  |
| 20261014-101846 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.968 | 3693.1 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-101844 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.984 | 3691.9 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-101841 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.994 | 3691.2 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-101837 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.928 | 3695.7 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-101834 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.968 | 3693.0 | 0 | 0 | 2 | 299.1 | fast |  |
| 20261014-101831 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.848 | 3701.1 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-101828 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.832 | 3702.1 | 0 | 0 | 2 | 298.9 | fast |  |
| 20261014-101825 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.968 | 3693.1 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-101823 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.928 | 3695.6 | 0 | 0 | 2 | 299.1 | fast |  |
| 20261014-101819 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.808 | 3703.7 | 3 | 1 | 0 | 299.0 | fast |  |
| 20261014-101816 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.886 | 3698.5 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-101813 | FINAL_torcs_jm_par_line_g5r6.py_x30@gui-timing | 64.968 | 3693.1 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-101810 | FINAL_torcs_jm_par_line_g5r6.py_det | 64.718 | 3709.6 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-101807 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.792 | 3704.8 | 0 | 0 | 3 | 298.9 | fast |  |
| 20261014-101804 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.782 | 3705.3 | 0 | 0 | 4 | 298.9 | fast |  |
| 20261014-101801 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.964 | 3693.3 | 0 | 0 | 4 | 298.9 | fast |  |
| 20261014-101758 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.828 | 3702.3 | 0 | 0 | 5 | 298.9 | fast |  |
| 20261014-101756 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.83 | 3702.2 | 0 | 0 | 5 | 298.9 | fast |  |
| 20261014-101753 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.782 | 3705.4 | 1 | 1 | 2 | 299.0 | fast |  |
| 20261014-101750 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.828 | 3702.3 | 0 | 0 | 5 | 298.9 | fast |  |
| 20261014-101747 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.75 | 3707.4 | 0 | 0 | 2 | 299.1 | fast |  |
| 20261014-101745 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.824 | 3702.6 | 0 | 0 | 5 | 298.9 | fast |  |
| 20261014-101742 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.826 | 3702.4 | 0 | 0 | 4 | 298.9 | fast |  |
| 20261014-101739 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.824 | 3702.7 | 0 | 0 | 5 | 298.9 | fast |  |
| 20261014-101737 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.848 | 3701.0 | 0 | 0 | 5 | 298.9 | fast |  |
| 20261014-101734 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.834 | 3701.9 | 0 | 0 | 4 | 298.9 | fast |  |
| 20261014-101732 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 65.03 | 3688.7 | 0 | 0 | 4 | 298.9 | fast |  |
| 20261014-101729 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.828 | 3702.4 | 0 | 0 | 5 | 298.9 | fast |  |
| 20261014-101726 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.822 | 3702.7 | 0 | 0 | 5 | 298.9 | fast |  |
| 20261014-101724 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.822 | 3701.0 | 0 | 0 | 5 | 298.9 | fast |  |
| 20261014-101721 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.834 | 3701.9 | 0 | 0 | 4 | 298.5 | fast |  |
| 20261014-101718 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.826 | 3702.4 | 0 | 0 | 5 | 298.9 | fast |  |
| 20261014-101716 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.874 | 3699.3 | 0 | 0 | 4 | 298.9 | fast |  |
| 20261014-101713 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.754 | 3707.2 | 0 | 0 | 3 | 298.9 | fast |  |
| 20261014-101710 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.77 | 3706.1 | 0 | 0 | 4 | 298.9 | fast |  |
| 20261014-101707 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.828 | 3702.4 | 0 | 0 | 4 | 298.9 | fast |  |
| 20261014-101705 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.856 | 3700.5 | 0 | 0 | 4 | 298.9 | fast |  |
| 20261014-101702 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.824 | 3702.7 | 0 | 0 | 5 | 298.9 | fast |  |
| 20261014-101700 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.828 | 3702.4 | 0 | 0 | 5 | 298.9 | fast |  |
| 20261014-101657 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.9 | 3697.5 | 0 | 0 | 5 | 298.9 | fast |  |
| 20261014-101654 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.846 | 3701.1 | 0 | 0 | 4 | 298.9 | fast |  |
| 20261014-101651 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.766 | 3706.4 | 0 | 0 | 3 | 298.9 | fast |  |
| 20261014-101648 | FINAL_torcs_jm_par_line_g5r4.py_x30@gui-timing | 64.78 | 3705.5 | 0 | 0 | 5 | 298.9 | fast |  |
| 20261014-101645 | FINAL_torcs_jm_par_line_g5r4.py_det | 64.67 | 3712.8 | 0 | 0 | 0 | 299.0 | fast |  |
| 20261014-002412 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002408 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002405 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 65.138 | 3681.3 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-002401 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002357 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002353 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002349 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002345 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002341 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002337 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002333 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002329 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002325 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002321 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002317 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002313 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002310 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002306 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002302 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002258 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002254 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002250 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002246 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002242 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002238 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002234 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002230 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002226 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002222 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002218 | FINAL_torcs_jm_par_line_gr3L.py_x30@gui-timing | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-002212 | FINAL_torcs_jm_par_line_gr3L.py_det | 64.964 | 3693.2 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001954 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001949 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001944 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001940 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001936 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001931 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001927 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001923 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001918 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001914 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001909 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001906 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001901 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001856 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001853 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001848 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001844 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001840 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001837 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001832 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001827 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001823 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001818 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001813 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001809 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001805 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001800 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001757 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001752 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001748 | FINAL_torcs_jm_par_line_gr2.py_x30@gui-timing | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001743 | FINAL_torcs_jm_par_line_gr2.py_det | 65.156 | 3680.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001737 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.162 | 3679.8 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001733 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.162 | 3679.8 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001728 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.162 | 3679.8 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001723 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.162 | 3679.8 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001719 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.162 | 3679.8 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001714 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.162 | 3679.8 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001709 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.162 | 3679.8 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001704 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.162 | 3679.8 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001701 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.556 | 3652.1 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-001656 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.452 | 3659.5 | 0 | 0 | 3 | 298.9 | fast |  |
| 20261014-001652 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.162 | 3679.8 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001647 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.162 | 3679.8 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001642 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.162 | 3679.8 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001638 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.162 | 3679.8 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001634 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.162 | 3679.8 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001629 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.162 | 3679.8 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001624 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.162 | 3679.8 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001620 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.162 | 3679.8 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001615 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.162 | 3679.8 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001611 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.162 | 3679.8 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001607 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.568 | 3651.3 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-001603 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.258 | 3673.1 | 0 | 0 | 1 | 299.1 | fast |  |
| 20261014-001559 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.248 | 3673.7 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001555 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.498 | 3656.2 | 0 | 0 | 2 | 299.0 | fast |  |
| 20261014-001551 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.458 | 3659.1 | 0 | 0 | 3 | 298.9 | fast |  |
| 20261014-001548 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.556 | 3652.1 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-001544 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.302 | 3670.1 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261014-001540 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.476 | 3657.8 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-001536 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.374 | 3665.0 | 0 | 0 | 3 | 298.9 | fast |  |
| 20261014-001533 | FINAL_torcs_jm_par_line_gr3rob.py_x30@gui-timing | 65.474 | 3657.9 | 0 | 0 | 3 | 299.0 | fast |  |
| 20261014-001528 | FINAL_torcs_jm_par_line_gr3rob.py_det | 65.162 | 3679.8 | 0 | 0 | 0 | 299.1 | fast |  |
| 20261013-225719 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.898 | 3627.0 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225715 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.498 | 3656.2 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225710 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.64 | 3646.1 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225707 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.86 | 3629.9 | 0 | 0 | 0 | 294.8 | fast |  |
| 20261013-225703 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.84 | 3631.4 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225659 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.756 | 3637.5 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225655 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.88 | 3628.3 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225651 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.7 | 3641.7 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225647 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.842 | 3631.2 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225644 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.758 | 3637.4 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225640 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.896 | 3627.1 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225636 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.702 | 3641.4 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225632 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.836 | 3631.6 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225629 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.878 | 3628.5 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225625 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.74 | 3638.8 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225621 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.734 | 3639.1 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225617 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.816 | 3633.1 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225613 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.65 | 3645.4 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225610 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.924 | 3625.1 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225605 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.678 | 3643.4 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225602 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.84 | 3631.3 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225558 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.798 | 3634.5 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225554 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.856 | 3630.1 | 0 | 0 | 0 | 294.8 | fast |  |
| 20261013-225550 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.718 | 3640.4 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225546 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.798 | 3634.5 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225542 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.792 | 3634.9 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225538 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.842 | 3631.1 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225534 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.816 | 3633.1 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225530 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.586 | 3650.0 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225527 | FINAL_torcs_jm_par_line_gr1a.py_x30@gui-timing | 65.878 | 3628.5 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225523 | FINAL_torcs_jm_par_line_gr1a.py_det | 65.498 | 3656.2 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-225518 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.74 | 3638.8 | 0 | 0 | 2 | 298.5 | fast |  |
| 20261013-225514 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.418 | 3661.8 | 0 | 0 | 0 | 298.5 | fast |  |
| 20261013-225510 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.246 | 3674.0 | 0 | 0 | 0 | 298.6 | fast |  |
| 20261013-225506 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.478 | 3657.7 | 0 | 0 | 0 | 298.5 | fast |  |
| 20261013-225502 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.858 | 3630.1 | 0 | 0 | 2 | 298.5 | fast |  |
| 20261013-225458 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.86 | 3629.8 | 0 | 0 | 2 | 298.5 | fast |  |
| 20261013-225455 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.86 | 3629.8 | 0 | 0 | 2 | 298.4 | fast |  |
| 20261013-225451 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.518 | 3654.8 | 0 | 0 | 1 | 298.5 | fast |  |
| 20261013-225447 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.834 | 3631.8 | 0 | 0 | 2 | 298.5 | fast |  |
| 20261013-225443 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.784 | 3635.6 | 0 | 0 | 2 | 298.5 | fast |  |
| 20261013-225439 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.838 | 3631.5 | 0 | 0 | 2 | 298.5 | fast |  |
| 20261013-225436 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.814 | 3633.3 | 0 | 0 | 2 | 298.5 | fast |  |
| 20261013-225433 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.378 | 3664.7 | 0 | 0 | 0 | 298.6 | fast |  |
| 20261013-225428 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.76 | 3637.4 | 0 | 0 | 2 | 298.5 | fast |  |
| 20261013-225425 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.876 | 3628.6 | 0 | 0 | 2 | 298.5 | fast |  |
| 20261013-225421 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.218 | 3675.9 | 0 | 0 | 0 | 298.6 | fast |  |
| 20261013-225416 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.278 | 3671.7 | 0 | 0 | 0 | 298.6 | fast |  |
| 20261013-225414 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.778 | 3635.9 | 0 | 0 | 2 | 298.4 | fast |  |
| 20261013-225409 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.756 | 3637.7 | 0 | 0 | 2 | 298.5 | fast |  |
| 20261013-225405 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.272 | 3672.2 | 0 | 0 | 0 | 298.6 | fast |  |
| 20261013-225401 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.288 | 3671.1 | 0 | 0 | 0 | 298.6 | fast |  |
| 20261013-225357 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.758 | 3637.5 | 0 | 0 | 2 | 298.5 | fast |  |
| 20261013-225354 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.878 | 3628.5 | 0 | 0 | 2 | 298.5 | fast |  |
| 20261013-225351 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.608 | 3648.4 | 0 | 0 | 2 | 298.4 | fast |  |
| 20261013-225346 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.28 | 3671.5 | 0 | 0 | 0 | 298.6 | fast |  |
| 20261013-225342 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.46 | 3658.9 | 0 | 0 | 1 | 298.5 | fast |  |
| 20261013-225338 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.898 | 3626.9 | 0 | 0 | 2 | 298.5 | fast |  |
| 20261013-225335 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.834 | 3631.8 | 0 | 0 | 2 | 298.5 | fast |  |
| 20261013-225331 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.478 | 3657.6 | 0 | 0 | 0 | 298.5 | fast |  |
| 20261013-225328 | FINAL_torcs_jm_par_line_gr1b.py_x30@gui-timing | 65.876 | 3628.6 | 0 | 0 | 2 | 298.5 | fast |  |
| 20261013-225323 | FINAL_torcs_jm_par_line_gr1b.py_det | 65.218 | 3675.9 | 0 | 0 | 0 | 298.6 | fast |  |
| 20261013-223848 | B5_line_gr1a_det | 65.498 | 3656.2 | 0 | 0 | 0 | 294.9 | fast |  |
| 20261013-220711 | B5_parity_clutch | 67.112 | 3683.3 | 0 | 0 | 0 | 291.8 | fast |  |
| 20261013-071610 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.118 | 3681.1 | 0 | 0 | 1 | 283.6 | fast |  |
| 20261013-071607 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.166 | 3678.0 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-071605 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.492 | 3655.8 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-071602 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.206 | 3675.2 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-071559 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.492 | 3655.8 | 0 | 0 | 3 | 283.8 | fast |  |
| 20261013-071556 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.216 | 3674.7 | 0 | 0 | 3 | 283.8 | fast |  |
| 20261013-071553 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.176 | 3677.3 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-071550 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.134 | 3678.4 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-071547 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.452 | 3658.6 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-071544 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.206 | 3675.2 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-071542 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.432 | 3660.1 | 0 | 0 | 3 | 283.8 | fast |  |
| 20261013-071539 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.478 | 3656.9 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-071536 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.426 | 3660.4 | 0 | 0 | 2 | 283.9 | fast |  |
| 20261013-071533 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.204 | 3675.4 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-071530 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.476 | 3656.8 | 0 | 0 | 3 | 283.8 | fast |  |
| 20261013-071527 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.102 | 3682.3 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-071524 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.248 | 3672.6 | 0 | 0 | 3 | 283.8 | fast |  |
| 20261013-071520 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.456 | 3658.3 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-071517 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.454 | 3658.4 | 0 | 0 | 3 | 283.8 | fast |  |
| 20261013-071514 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.478 | 3656.9 | 0 | 0 | 3 | 283.8 | fast |  |
| 20261013-071511 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.42 | 3660.8 | 0 | 0 | 2 | 283.9 | fast |  |
| 20261013-071507 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.42 | 3660.8 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-071504 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.172 | 3677.6 | 0 | 0 | 4 | 283.9 | fast |  |
| 20261013-071501 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.472 | 3657.2 | 0 | 0 | 3 | 283.8 | fast |  |
| 20261013-071457 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.092 | 3682.9 | 0 | 0 | 0 | 283.7 | fast |  |
| 20261013-071454 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.148 | 3679.2 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-071451 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.516 | 3654.2 | 0 | 0 | 3 | 283.8 | fast |  |
| 20261013-071447 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.494 | 3655.7 | 0 | 0 | 3 | 284.0 | fast |  |
| 20261013-071444 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.42 | 3660.8 | 0 | 0 | 3 | 283.3 | fast |  |
| 20261013-071440 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.198 | 3675.8 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-071435 | FINAL_torcs_jm_par_line_e2d1.py_det | 68.746 | 3705.9 | 0 | 0 | 0 | 283.6 | fast |  |
| 20261013-071425 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 69.082 | 3683.6 | 0 | 0 | 3 | 284.1 | fast |  |
| 20261013-071422 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 68.966 | 3691.3 | 0 | 0 | 3 | 284.2 | fast |  |
| 20261013-071419 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 69.284 | 3670.1 | 0 | 0 | 3 | 284.0 | fast |  |
| 20261013-071416 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 69.262 | 3671.6 | 0 | 0 | 3 | 284.2 | fast |  |
| 20261013-071412 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 68.936 | 3693.2 | 0 | 0 | 2 | 284.0 | fast |  |
| 20261013-071410 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 69.352 | 3665.4 | 0 | 0 | 4 | 284.1 | fast |  |
| 20261013-071406 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 68.966 | 3691.3 | 0 | 0 | 3 | 284.2 | fast |  |
| 20261013-071404 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 69.32 | 3667.7 | 0 | 0 | 3 | 284.2 | fast |  |
| 20261013-071400 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 68.964 | 3691.3 | 0 | 0 | 3 | 284.2 | fast |  |
| 20261013-071357 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 69.24 | 3673.0 | 0 | 0 | 4 | 284.2 | fast |  |
| 20261013-071354 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 69.312 | 3668.1 | 0 | 0 | 3 | 284.2 | fast |  |
| 20261013-071351 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 68.966 | 3691.2 | 0 | 0 | 3 | 284.2 | fast |  |
| 20261013-071347 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 68.926 | 3693.9 | 0 | 0 | 2 | 284.0 | fast |  |
| 20261013-071344 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 68.998 | 3689.2 | 0 | 0 | 3 | 284.2 | fast |  |
| 20261013-071341 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 68.964 | 3691.3 | 0 | 0 | 3 | 284.2 | fast |  |
| 20261013-071337 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 68.85 | 3698.7 | 0 | 0 | 1 | 284.0 | fast |  |
| 20261013-071335 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 69.348 | 3665.6 | 0 | 0 | 3 | 284.1 | fast |  |
| 20261013-071331 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 69.074 | 3683.9 | 0 | 0 | 3 | 284.2 | fast |  |
| 20261013-071328 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 68.986 | 3689.9 | 0 | 0 | 4 | 284.2 | fast |  |
| 20261013-071325 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 69.34 | 3666.3 | 0 | 0 | 3 | 284.1 | fast |  |
| 20261013-071321 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 69.282 | 3670.2 | 0 | 0 | 3 | 284.2 | fast |  |
| 20261013-071318 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 68.908 | 3695.2 | 0 | 0 | 3 | 284.1 | fast |  |
| 20261013-071314 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 68.996 | 3689.2 | 0 | 0 | 3 | 284.2 | fast |  |
| 20261013-071311 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 68.946 | 3692.6 | 0 | 0 | 3 | 284.2 | fast |  |
| 20261013-071307 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 68.986 | 3689.8 | 1 | 1 | 2 | 284.2 | fast |  |
| 20261013-071304 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 68.996 | 3689.2 | 0 | 0 | 3 | 284.2 | fast |  |
| 20261013-071301 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 69.3 | 3668.9 | 0 | 0 | 3 | 284.2 | fast |  |
| 20261013-071258 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 68.948 | 3692.4 | 0 | 0 | 1 | 284.0 | fast |  |
| 20261013-071255 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 68.986 | 3689.9 | 0 | 0 | 3 | 284.2 | fast |  |
| 20261013-071251 | FINAL_torcs_jm_par_line_e3d1.py_x30@gui-timing | 69.32 | 3667.6 | 0 | 0 | 3 | 284.1 | fast |  |
| 20261013-071246 | FINAL_torcs_jm_par_line_e3d1.py_det | 68.582 | 3716.3 | 0 | 0 | 0 | 284.0 | fast |  |
| 20261013-062209 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.494 | 3655.7 | 0 | 0 | 3 | 284.0 | fast |  |
| 20261013-062206 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.226 | 3673.9 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-062203 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.188 | 3676.6 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-062200 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.454 | 3658.4 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-062158 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.498 | 3655.3 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-062155 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.208 | 3675.2 | 0 | 0 | 3 | 283.8 | fast |  |
| 20261013-062152 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.492 | 3655.8 | 0 | 0 | 3 | 283.8 | fast |  |
| 20261013-062149 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.146 | 3679.3 | 0 | 0 | 2 | 283.6 | fast |  |
| 20261013-062147 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.22 | 3674.2 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-062143 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 68.922 | 3694.5 | 0 | 0 | 2 | 283.8 | fast |  |
| 20261013-062141 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.192 | 3676.2 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-062138 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.176 | 3677.4 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-062135 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.156 | 3678.6 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-062131 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.17 | 3677.8 | 0 | 0 | 2 | 283.9 | fast |  |
| 20261013-062129 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.168 | 3677.9 | 0 | 0 | 3 | 284.0 | fast |  |
| 20261013-062126 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.154 | 3678.7 | 0 | 0 | 3 | 283.8 | fast |  |
| 20261013-062123 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.436 | 3659.7 | 0 | 0 | 3 | 284.0 | fast |  |
| 20261013-062120 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.462 | 3657.9 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-062118 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.452 | 3658.5 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-062115 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.204 | 3675.4 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-062112 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.166 | 3678.0 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-062109 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.11 | 3681.6 | 0 | 0 | 2 | 283.6 | fast |  |
| 20261013-062106 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.436 | 3659.7 | 0 | 0 | 3 | 283.8 | fast |  |
| 20261013-062103 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.4 | 3662.2 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-062059 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.19 | 3676.3 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-062055 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.416 | 3661.2 | 0 | 0 | 3 | 283.8 | fast |  |
| 20261013-062053 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.228 | 3673.8 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-062048 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.068 | 3684.5 | 0 | 0 | 1 | 283.6 | fast |  |
| 20261013-062044 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.416 | 3661.0 | 0 | 0 | 3 | 283.9 | fast |  |
| 20261013-062042 | FINAL_torcs_jm_par_line_e2d1.py_x30@gui-timing | 69.198 | 3675.8 | 0 | 0 | 3 | 283.8 | fast |  |
| 20261013-062037 | FINAL_torcs_jm_par_line_e2d1.py_det | 68.746 | 3705.9 | 0 | 0 | 0 | 283.6 | fast |  |
| 20261013-051823 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.676 | 3709.5 | 1 | 1 | 4 | 286.1 | fast |  |
| 20261013-051820 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.318 | 3731.6 | 0 | 0 | 2 | 285.9 | fast |  |
| 20261013-051817 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.26 | 3735.4 | 1 | 1 | 2 | 285.9 | fast |  |
| 20261013-051814 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.676 | 3709.5 | 1 | 1 | 4 | 286.0 | fast |  |
| 20261013-051810 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.326 | 3731.1 | 1 | 1 | 4 | 286.1 | fast |  |
| 20261013-051807 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.356 | 3729.4 | 1 | 1 | 4 | 286.1 | fast |  |
| 20261013-051803 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.438 | 3724.3 | 0 | 0 | 4 | 286.0 | fast |  |
| 20261013-051800 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.358 | 3729.1 | 1 | 1 | 4 | 286.1 | fast |  |
| 20261013-051757 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.7 | 3708.0 | 0 | 0 | 4 | 286.1 | fast |  |
| 20261013-051753 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.416 | 3725.7 | 1 | 1 | 4 | 286.0 | fast |  |
| 20261013-051750 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.336 | 3730.5 | 0 | 0 | 3 | 286.1 | fast |  |
| 20261013-051747 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.376 | 3728.1 | 0 | 0 | 4 | 286.0 | fast |  |
| 20261013-051743 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.268 | 3735.1 | 0 | 0 | 4 | 286.0 | fast |  |
| 20261013-051740 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.276 | 3734.3 | 1 | 1 | 2 | 285.9 | fast |  |
| 20261013-051737 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.648 | 3711.2 | 0 | 0 | 4 | 286.0 | fast |  |
| 20261013-051734 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 67.99 | 3620.1 | 0 | 0 | 4 | 286.0 | fast |  |
| 20261013-051730 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.636 | 3712.0 | 1 | 1 | 3 | 285.9 | fast |  |
| 20261013-051727 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.398 | 3726.7 | 1 | 1 | 4 | 286.0 | fast |  |
| 20261013-051724 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 67.968 | 3621.8 | 0 | 0 | 3 | 286.0 | fast |  |
| 20261013-051720 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.582 | 3715.6 | 0 | 0 | 4 | 286.1 | fast |  |
| 20261013-051718 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.62 | 3713.2 | 1 | 1 | 4 | 286.1 | fast |  |
| 20261013-051715 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.126 | 3743.6 | 0 | 0 | 1 | 285.9 | fast |  |
| 20261013-051711 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.6 | 3714.4 | 0 | 0 | 3 | 286.1 | fast |  |
| 20261013-051709 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.686 | 3708.9 | 1 | 1 | 4 | 286.0 | fast |  |
| 20261013-051705 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.312 | 3732.0 | 0 | 0 | 3 | 286.1 | fast |  |
| 20261013-051702 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.708 | 3707.6 | 0 | 0 | 4 | 286.0 | fast |  |
| 20261013-051658 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.374 | 3728.4 | 1 | 1 | 4 | 286.0 | fast |  |
| 20261013-051655 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.126 | 3743.6 | 0 | 0 | 1 | 285.9 | fast |  |
| 20261013-051651 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.614 | 3713.4 | 1 | 1 | 4 | 286.1 | fast |  |
| 20261013-051648 | FINAL_torcs_jm_par_line_e1b.py_x30@gui-timing | 68.396 | 3727.1 | 0 | 0 | 4 | 286.0 | fast |  |
| 20261013-051643 | FINAL_torcs_jm_par_line_e1b.py_det | 67.826 | 3632.2 | 0 | 0 | 0 | 285.9 | fast |  |
| 20261013-051639 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 67.782 | 3635.4 | 0 | 0 | 0 | 286.1 | fast |  |
| 20261013-051636 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 67.866 | 3629.3 | 1 | 1 | 3 | 286.0 | fast |  |
| 20261013-051633 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.566 | 3716.6 | 0 | 0 | 6 | 286.1 | fast |  |
| 20261013-051630 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.13 | 3743.4 | 0 | 0 | 3 | 286.3 | fast |  |
| 20261013-051627 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.624 | 3712.6 | 1 | 1 | 6 | 286.1 | fast |  |
| 20261013-051623 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.298 | 3731.3 | 0 | 0 | 5 | 286.2 | fast |  |
| 20261013-051621 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.34 | 3730.4 | 0 | 0 | 5 | 286.2 | fast |  |
| 20261013-051617 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.414 | 3726.2 | 0 | 0 | 5 | 286.2 | fast |  |
| 20261013-051614 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.17 | 3741.8 | 0 | 0 | 4 | 286.1 | fast |  |
| 20261013-051611 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.492 | 3721.3 | 0 | 0 | 5 | 286.1 | fast |  |
| 20261013-051608 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.282 | 3734.1 | 0 | 0 | 5 | 286.2 | fast |  |
| 20261013-051605 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.254 | 3735.9 | 0 | 0 | 4 | 286.1 | fast |  |
| 20261013-051602 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.302 | 3732.9 | 1 | 1 | 5 | 286.3 | fast |  |
| 20261013-051558 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.55 | 3717.8 | 1 | 1 | 6 | 286.2 | fast |  |
| 20261013-051556 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.624 | 3712.8 | 0 | 0 | 5 | 286.1 | fast |  |
| 20261013-051552 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.262 | 3735.4 | 1 | 1 | 6 | 286.1 | fast |  |
| 20261013-051549 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.6 | 3714.2 | 1 | 1 | 6 | 286.1 | fast |  |
| 20261013-051546 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.584 | 3715.4 | 0 | 0 | 5 | 286.2 | fast |  |
| 20261013-051543 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.3 | 3732.9 | 0 | 0 | 5 | 286.2 | fast |  |
| 20261013-051539 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.064 | 3747.4 | 0 | 0 | 1 | 286.0 | fast |  |
| 20261013-051536 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.518 | 3719.4 | 1 | 1 | 6 | 286.2 | fast |  |
| 20261013-051533 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.51 | 3720.2 | 0 | 0 | 6 | 286.2 | fast |  |
| 20261013-051529 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.088 | 3745.9 | 0 | 0 | 2 | 286.0 | fast |  |
| 20261013-051526 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.598 | 3714.4 | 1 | 1 | 6 | 286.1 | fast |  |
| 20261013-051523 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.396 | 3726.7 | 0 | 0 | 5 | 286.2 | fast |  |
| 20261013-051519 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.12 | 3743.8 | 1 | 1 | 2 | 286.0 | fast |  |
| 20261013-051517 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.598 | 3714.5 | 1 | 1 | 6 | 286.1 | fast |  |
| 20261013-051513 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.282 | 3734.1 | 0 | 0 | 5 | 286.2 | fast |  |
| 20261013-051510 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.18 | 3740.4 | 0 | 0 | 3 | 286.2 | fast |  |
| 20261013-051507 | FINAL_torcs_jm_par_line_e1c.py_x30@gui-timing | 68.556 | 3717.4 | 0 | 0 | 5 | 286.3 | fast |  |
| 20261013-051502 | FINAL_torcs_jm_par_line_e1c.py_det | 67.768 | 3636.4 | 0 | 0 | 0 | 286.0 | fast |  |
| 20261013-042830 | B4_line_e1b_det | 67.826 | 3632.2 | 0 | 0 | 0 | 285.9 | fast |  |
| 20261013-040307 | B4_line_e1a_det | 68.128 | 3744.4 | 0 | 0 | 0 | 285.1 | fast |  |
| 20261013-022356 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.662 | 3643.9 | 0 | 0 | 3 | 283.7 | fast |  |
| 20261013-022352 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.704 | 3641.0 | 0 | 0 | 4 | 283.7 | fast |  |
| 20261013-022350 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.976 | 3621.3 | 0 | 0 | 3 | 283.8 | fast |  |
| 20261013-022347 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.874 | 3628.7 | 0 | 0 | 3 | 283.6 | fast |  |
| 20261013-022344 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.972 | 3621.6 | 0 | 0 | 4 | 283.7 | fast |  |
| 20261013-022341 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.934 | 3624.4 | 0 | 0 | 3 | 283.7 | fast |  |
| 20261013-022338 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.926 | 3624.9 | 0 | 0 | 3 | 283.8 | fast |  |
| 20261013-022335 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.926 | 3624.9 | 0 | 0 | 3 | 283.8 | fast |  |
| 20261013-022332 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 70.014 | 3618.3 | 0 | 0 | 4 | 283.8 | fast |  |
| 20261013-022329 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.664 | 3643.9 | 0 | 0 | 3 | 283.6 | fast |  |
| 20261013-022326 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 70.048 | 3748.0 | 0 | 0 | 3 | 283.7 | fast |  |
| 20261013-022323 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.838 | 3631.3 | 0 | 0 | 3 | 283.6 | fast |  |
| 20261013-022320 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.904 | 3626.5 | 0 | 0 | 3 | 283.7 | fast |  |
| 20261013-022317 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.734 | 3638.8 | 0 | 0 | 3 | 283.7 | fast |  |
| 20261013-022315 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.91 | 3626.2 | 0 | 0 | 3 | 283.7 | fast |  |
| 20261013-022312 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.826 | 3632.2 | 0 | 0 | 4 | 283.7 | fast |  |
| 20261013-022308 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.66 | 3644.1 | 0 | 0 | 3 | 283.7 | fast |  |
| 20261013-022306 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.616 | 3647.2 | 0 | 0 | 3 | 283.7 | fast |  |
| 20261013-022303 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.946 | 3623.5 | 0 | 0 | 3 | 283.8 | fast |  |
| 20261013-022300 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.796 | 3634.3 | 0 | 0 | 3 | 283.7 | fast |  |
| 20261013-022257 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.73 | 3639.1 | 0 | 0 | 3 | 283.6 | fast |  |
| 20261013-022254 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.644 | 3645.2 | 0 | 0 | 3 | 283.6 | fast |  |
| 20261013-022251 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.752 | 3637.5 | 3 | 1 | 3 | 283.6 | fast |  |
| 20261013-022248 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.968 | 3621.9 | 0 | 0 | 3 | 283.7 | fast |  |
| 20261013-022245 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.632 | 3646.1 | 0 | 0 | 3 | 283.7 | fast |  |
| 20261013-022242 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.71 | 3640.5 | 3 | 1 | 4 | 283.7 | fast |  |
| 20261013-022239 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.662 | 3644.0 | 2 | 1 | 3 | 283.6 | fast |  |
| 20261013-022236 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.63 | 3646.2 | 0 | 0 | 3 | 283.7 | fast |  |
| 20261013-022233 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.976 | 3621.3 | 0 | 0 | 3 | 283.7 | fast |  |
| 20261013-022230 | FINAL_torcs_jm_par_line_z6.py_x30@gui-timing | 69.952 | 3623.1 | 0 | 0 | 4 | 283.7 | fast |  |
| 20261013-022225 | FINAL_torcs_jm_par_line_z6.py_det | 68.95 | 3692.7 | 0 | 0 | 0 | 283.5 | fast |  |
| 20261013-022222 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 69.852 | 3630.4 | 0 | 0 | 4 | 283.7 | fast |  |
| 20261013-022219 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 69.788 | 3634.9 | 0 | 0 | 4 | 283.6 | fast |  |
| 20261013-022216 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 70.0 | 3619.5 | 3 | 1 | 4 | 283.7 | fast |  |
| 20261013-022214 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 70.082 | 3746.1 | 0 | 0 | 3 | 283.7 | fast |  |
| 20261013-022210 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 70.038 | 3748.7 | 0 | 0 | 4 | 283.7 | fast |  |
| 20261013-022207 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 69.96 | 3622.5 | 0 | 0 | 4 | 283.7 | fast |  |
| 20261013-022204 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 69.682 | 3642.5 | 0 | 0 | 3 | 283.7 | fast |  |
| 20261013-022201 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 69.93 | 3624.6 | 0 | 0 | 4 | 283.7 | fast |  |
| 20261013-022158 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 70.144 | 3742.3 | 0 | 0 | 3 | 283.6 | fast |  |
| 20261013-022155 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 70.04 | 3748.6 | 0 | 0 | 4 | 283.6 | fast |  |
| 20261013-022153 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 70.062 | 3747.3 | 0 | 0 | 4 | 283.7 | fast |  |
| 20261013-022150 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 70.166 | 3741.0 | 0 | 0 | 3 | 283.7 | fast |  |
| 20261013-022147 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 70.08 | 3746.2 | 0 | 0 | 5 | 283.8 | fast |  |
| 20261013-022144 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 69.812 | 3633.2 | 0 | 0 | 4 | 283.6 | fast |  |
| 20261013-022141 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 70.144 | 3742.3 | 0 | 0 | 3 | 283.7 | fast |  |
| 20261013-022138 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 69.706 | 3640.8 | 0 | 0 | 4 | 283.6 | fast |  |
| 20261013-022135 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 70.014 | 3618.2 | 3 | 1 | 3 | 283.7 | fast |  |
| 20261013-022133 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 69.978 | 3621.1 | 2 | 1 | 3 | 283.6 | fast |  |
| 20261013-022129 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 69.622 | 3646.8 | 3 | 1 | 3 | 283.7 | fast |  |
| 20261013-022126 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 69.726 | 3639.4 | 3 | 1 | 4 | 283.6 | fast |  |
| 20261013-022123 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 69.828 | 3632.0 | 0 | 0 | 4 | 283.6 | fast |  |
| 20261013-022120 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 69.828 | 3632.1 | 0 | 0 | 4 | 283.7 | fast |  |
| 20261013-022116 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 69.812 | 3633.3 | 0 | 0 | 4 | 283.7 | fast |  |
| 20261013-022114 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 70.098 | 3745.0 | 2 | 1 | 3 | 283.7 | fast |  |
| 20261013-022110 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 69.79 | 3634.8 | 0 | 0 | 4 | 283.6 | fast |  |
| 20261013-022107 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 69.832 | 3631.8 | 2 | 1 | 4 | 283.6 | fast |  |
| 20261013-022104 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 69.664 | 3643.9 | 2 | 1 | 3 | 283.7 | fast |  |
| 20261013-022102 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 70.124 | 3743.6 | 0 | 0 | 3 | 283.7 | fast |  |
| 20261013-022058 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 70.282 | 3734.1 | 2 | 1 | 4 | 283.7 | fast |  |
| 20261013-022055 | FINAL_torcs_jm_par_line_z7rob.py_x30@gui-timing | 70.144 | 3742.3 | 2 | 1 | 3 | 283.7 | fast |  |
| 20261013-022050 | FINAL_torcs_jm_par_line_z7rob.py_det | 68.992 | 3689.9 | 0 | 0 | 0 | 283.5 | fast |  |
| 20261013-022047 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 70.014 | 3618.4 | 0 | 0 | 3 | 283.7 | fast |  |
| 20261013-022044 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.712 | 3640.4 | 3 | 1 | 2 | 283.7 | fast |  |
| 20261013-022041 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.916 | 3625.7 | 0 | 0 | 3 | 283.8 | fast |  |
| 20261013-022038 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.948 | 3623.3 | 3 | 1 | 3 | 283.6 | fast |  |
| 20261013-022035 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.712 | 3640.4 | 0 | 0 | 3 | 283.8 | fast |  |
| 20261013-022032 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.904 | 3626.5 | 0 | 0 | 3 | 283.6 | fast |  |
| 20261013-022029 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.938 | 3624.1 | 0 | 0 | 3 | 283.7 | fast |  |
| 20261013-022026 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.934 | 3624.4 | 0 | 0 | 2 | 283.8 | fast |  |
| 20261013-022023 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.63 | 3646.2 | 2 | 1 | 3 | 283.6 | fast |  |
| 20261013-022020 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.948 | 3623.4 | 0 | 0 | 2 | 283.8 | fast |  |
| 20261013-022017 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.582 | 3649.6 | 0 | 0 | 3 | 283.7 | fast |  |
| 20261013-022014 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.91 | 3626.1 | 0 | 0 | 3 | 283.8 | fast |  |
| 20261013-022012 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.626 | 3646.4 | 0 | 0 | 2 | 283.8 | fast |  |
| 20261013-022008 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.882 | 3628.1 | 0 | 0 | 3 | 283.7 | fast |  |
| 20261013-022006 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.92 | 3625.4 | 0 | 0 | 3 | 283.7 | fast |  |
| 20261013-022002 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.672 | 3643.2 | 0 | 0 | 2 | 283.8 | fast |  |
| 20261013-021959 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.872 | 3629.0 | 0 | 0 | 2 | 283.7 | fast |  |
| 20261013-021957 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.968 | 3621.9 | 3 | 1 | 2 | 283.7 | fast |  |
| 20261013-021953 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.592 | 3648.8 | 0 | 0 | 3 | 283.8 | fast |  |
| 20261013-021951 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.966 | 3622.0 | 2 | 1 | 3 | 283.8 | fast |  |
| 20261013-021947 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.892 | 3627.4 | 2 | 1 | 2 | 283.8 | fast |  |
| 20261013-021945 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.972 | 3621.5 | 0 | 0 | 2 | 283.7 | fast |  |
| 20261013-021942 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.858 | 3629.9 | 0 | 0 | 3 | 283.7 | fast |  |
| 20261013-021939 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.648 | 3644.9 | 0 | 0 | 2 | 283.7 | fast |  |
| 20261013-021937 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.994 | 3619.9 | 0 | 0 | 2 | 283.8 | fast |  |
| 20261013-021933 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.696 | 3641.6 | 0 | 0 | 3 | 283.6 | fast |  |
| 20261013-021930 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.892 | 3627.5 | 0 | 0 | 2 | 283.8 | fast |  |
| 20261013-021928 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.932 | 3624.5 | 2 | 1 | 3 | 283.6 | fast |  |
| 20261013-021924 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.648 | 3644.9 | 3 | 1 | 3 | 283.7 | fast |  |
| 20261013-021921 | FINAL_torcs_jm_par_line_z9rob.py_x30@gui-timing | 69.578 | 3649.9 | 0 | 0 | 3 | 283.7 | fast |  |
| 20261013-021916 | FINAL_torcs_jm_par_line_z9rob.py_det | 68.956 | 3692.3 | 0 | 0 | 0 | 283.5 | fast |  |
| 20261013-021913 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 69.756 | 3637.2 | 0 | 0 | 1 | 283.6 | fast |  |
| 20261013-021910 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 70.08 | 3746.2 | 0 | 0 | 1 | 283.6 | fast |  |
| 20261013-021907 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 70.082 | 3746.1 | 0 | 0 | 2 | 283.6 | fast |  |
| 20261013-021904 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 69.718 | 3640.0 | 0 | 0 | 2 | 283.7 | fast |  |
| 20261013-021901 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 70.096 | 3745.0 | 0 | 0 | 1 | 283.7 | fast |  |
| 20261013-021858 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 70.082 | 3746.1 | 0 | 0 | 1 | 283.6 | fast |  |
| 20261013-021855 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 70.08 | 3746.2 | 0 | 0 | 1 | 283.6 | fast |  |
| 20261013-021851 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 69.972 | 3621.6 | 0 | 0 | 1 | 283.6 | fast |  |
| 20261013-021849 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 70.08 | 3746.2 | 0 | 0 | 2 | 283.8 | fast |  |
| 20261013-021846 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 70.146 | 3742.3 | 0 | 0 | 2 | 283.7 | fast |  |
| 20261013-021843 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 69.974 | 3621.4 | 0 | 0 | 1 | 283.7 | fast |  |
| 20261013-021840 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 70.094 | 3745.3 | 0 | 0 | 1 | 283.4 | fast |  |
| 20261013-021837 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 69.738 | 3638.6 | 0 | 0 | 1 | 283.6 | fast |  |
| 20261013-021834 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 70.014 | 3618.3 | 0 | 0 | 1 | 283.6 | fast |  |
| 20261013-021831 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 69.996 | 3619.8 | 0 | 0 | 1 | 283.6 | fast |  |
| 20261013-021828 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 70.112 | 3744.1 | 0 | 0 | 1 | 283.7 | fast |  |
| 20261013-021825 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 70.038 | 3748.7 | 0 | 0 | 1 | 283.6 | fast |  |
| 20261013-021823 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 69.996 | 3619.9 | 0 | 0 | 1 | 283.7 | fast |  |
| 20261013-021820 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 69.948 | 3623.4 | 0 | 0 | 1 | 283.6 | fast |  |
| 20261013-021817 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 70.082 | 3746.1 | 0 | 0 | 1 | 283.6 | fast |  |
| 20261013-021814 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 69.718 | 3640.0 | 0 | 0 | 2 | 283.7 | fast |  |
| 20261013-021811 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 69.974 | 3621.5 | 0 | 0 | 1 | 283.6 | fast |  |
| 20261013-021807 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 69.72 | 3639.8 | 0 | 0 | 2 | 283.7 | fast |  |
| 20261013-021804 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 70.084 | 3745.9 | 0 | 0 | 2 | 283.6 | fast |  |
| 20261013-021801 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 70.06 | 3747.4 | 0 | 0 | 2 | 283.6 | fast |  |
| 20261013-021759 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 69.81 | 3633.4 | 0 | 0 | 2 | 283.5 | fast |  |
| 20261013-021755 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 70.05 | 3748.1 | 0 | 0 | 1 | 283.6 | fast |  |
| 20261013-021752 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 69.838 | 3629.7 | 0 | 0 | 1 | 283.7 | fast |  |
| 20261013-021749 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 69.708 | 3640.7 | 0 | 0 | 2 | 283.6 | fast |  |
| 20261013-021746 | FINAL_torcs_jm_par_line_z10rob11.py_x30@gui-timing | 69.77 | 3636.3 | 0 | 0 | 2 | 283.7 | fast |  |
| 20261013-021742 | FINAL_torcs_jm_par_line_z10rob11.py_det | 68.934 | 3693.8 | 0 | 0 | 0 | 283.6 | fast |  |
| 20261013-021738 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.77 | 3636.4 | 0 | 0 | 3 | 284.0 | fast |  |
| 20261013-021735 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.518 | 3654.0 | 0 | 0 | 3 | 284.0 | fast |  |
| 20261013-021732 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.926 | 3625.0 | 0 | 0 | 4 | 284.0 | fast |  |
| 20261013-021729 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.872 | 3628.9 | 0 | 0 | 4 | 284.0 | fast |  |
| 20261013-021726 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.818 | 3632.9 | 0 | 0 | 3 | 284.0 | fast |  |
| 20261013-021724 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.864 | 3629.6 | 0 | 0 | 4 | 284.0 | fast |  |
| 20261013-021721 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.586 | 3649.3 | 0 | 0 | 3 | 284.0 | fast |  |
| 20261013-021718 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.82 | 3632.8 | 0 | 0 | 4 | 284.1 | fast |  |
| 20261013-021715 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.838 | 3631.3 | 0 | 0 | 3 | 284.1 | fast |  |
| 20261013-021712 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.582 | 3649.5 | 0 | 0 | 3 | 284.0 | fast |  |
| 20261013-021709 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.892 | 3627.5 | 0 | 0 | 4 | 284.0 | fast |  |
| 20261013-021706 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.576 | 3649.9 | 0 | 0 | 4 | 284.1 | fast |  |
| 20261013-021703 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.832 | 3631.9 | 0 | 0 | 4 | 284.0 | fast |  |
| 20261013-021700 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.89 | 3627.5 | 0 | 0 | 4 | 284.0 | fast |  |
| 20261013-021658 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.874 | 3628.7 | 0 | 0 | 3 | 284.0 | fast |  |
| 20261013-021655 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.828 | 3632.1 | 0 | 0 | 4 | 284.0 | fast |  |
| 20261013-021652 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.518 | 3654.0 | 0 | 0 | 3 | 284.1 | fast |  |
| 20261013-021649 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.894 | 3627.3 | 0 | 0 | 4 | 284.0 | fast |  |
| 20261013-021646 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.616 | 3647.2 | 0 | 0 | 3 | 284.0 | fast |  |
| 20261013-021642 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.884 | 3628.1 | 0 | 0 | 3 | 284.2 | fast |  |
| 20261013-021640 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.652 | 3644.7 | 0 | 0 | 4 | 284.1 | fast |  |
| 20261013-021636 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.6 | 3648.2 | 0 | 0 | 4 | 284.0 | fast |  |
| 20261013-021634 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.864 | 3629.5 | 0 | 0 | 3 | 283.8 | fast |  |
| 20261013-021631 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.876 | 3628.7 | 0 | 0 | 4 | 284.0 | fast |  |
| 20261013-021628 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.816 | 3631.3 | 0 | 0 | 3 | 284.1 | fast |  |
| 20261013-021625 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.922 | 3625.3 | 0 | 0 | 4 | 284.0 | fast |  |
| 20261013-021622 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.87 | 3629.1 | 0 | 0 | 4 | 284.0 | fast |  |
| 20261013-021619 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.882 | 3628.1 | 0 | 0 | 4 | 284.0 | fast |  |
| 20261013-021617 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.918 | 3625.5 | 0 | 0 | 4 | 284.0 | fast |  |
| 20261013-021614 | FINAL_torcs_jm_par_line_z12rob.py_x30@gui-timing | 69.858 | 3629.9 | 0 | 0 | 3 | 284.1 | fast |  |
| 20261013-021609 | FINAL_torcs_jm_par_line_z12rob.py_det | 68.792 | 3703.1 | 0 | 0 | 0 | 284.0 | fast |  |
| 20261013-014004 | FINAL_torcs_jm_par_line_z9rob.py_x2@gui-timing | 69.606 | 3647.9 | 0 | 0 | 1 | 283.8 | fast |  |
| 20261013-014000 | FINAL_torcs_jm_par_line_z9rob.py_x2@gui-timing | 69.604 | 3648.0 | 0 | 0 | 3 | 283.8 | fast |  |
| 20261013-013954 | FINAL_torcs_jm_par_line_z9rob.py_det | 68.956 | 3692.3 | 0 | 0 | 0 | 283.5 | fast |  |
| 20261013-012916 | B4_line_z9rob_det | 68.956 | 3692.3 | 0 | 0 | 0 | 283.5 | fast |  |
| 20261013-010850 | B4_line_z7rob_det | 68.992 | 3689.9 | 0 | 0 | 0 | 283.5 | fast |  |
| 20261013-005058 | B4_line_z6_det | 68.95 | 3692.7 | 0 | 0 | 0 | 283.5 | fast |  |
| 20261013-003958 | B4_line_z5_x8@gui-timing(yük altında) | 70.006 | 3619.0 | 0 | 0 | 3 | 283.1 | fast |  |
| 20261013-003955 | B4_line_z5_x8@gui-timing(yük altında) | 70.298 | 3733.5 | 0 | 0 | 3 | 283.2 | fast |  |
| 20261013-003950 | B4_line_z5_x8@gui-timing(yük altında) | 70.23 | 3737.5 | 0 | 0 | 3 | 283.2 | fast |  |
| 20261013-003946 | B4_line_z5_x8@gui-timing(yük altında) | 70.234 | 3737.2 | 0 | 0 | 3 | 283.2 | fast |  |
| 20261013-003941 | B4_line_z5_x8@gui-timing(yük altında) | 70.014 | 3617.3 | 0 | 0 | 2 | 283.2 | fast |  |
| 20261013-003938 | B4_line_z5_x8@gui-timing(yük altında) | 70.404 | 3726.9 | 0 | 0 | 3 | 283.1 | fast |  |
| 20261013-003934 | B4_line_z5_x8@gui-timing(yük altında) | 70.196 | 3739.4 | 0 | 0 | 2 | 283.2 | fast |  |
| 20261013-003931 | B4_line_z5_x8@gui-timing(yük altında) | 70.37 | 3729.0 | 0 | 0 | 3 | 283.2 | fast |  |
| 20261013-003919 | B4_line_z5_det | 69.576 | 3650.2 | 0 | 0 | 0 | 282.9 | fast |  |
| 20261012-234407 | B4_line_parity | 73.522 | 3654.0 | 0 | 0 | 0 | 277.5 | fast |  |
| 20261012-061653 | B3_w512a_v08171_a3m50_x30@gui-timing | 76.376 | 3704.8 | 0 | 0 | 1 | 267.3 | fast |  |
| 20261012-061650 | B3_w512a_v08171_a3m50_x30@gui-timing | 76.166 | 3715.6 | 0 | 0 | 1 | 267.2 | fast |  |
| 20261012-061646 | B3_w512a_v08171_a3m50_x30@gui-timing | 75.632 | 3640.7 | 0 | 0 | 1 | 267.2 | fast |  |
| 20261012-061642 | B3_w512a_v08171_a3m50_x30@gui-timing | 75.532 | 3645.4 | 0 | 0 | 1 | 267.0 | fast |  |
| 20261012-061639 | B3_w512a_v08171_a3m50_x30@gui-timing | 75.978 | 3620.9 | 0 | 0 | 1 | 267.0 | fast |  |
| 20261012-061636 | B3_w512a_v08171_a3m50_x30@gui-timing | 75.802 | 3631.0 | 0 | 0 | 1 | 267.0 | fast |  |
| 20261012-061632 | B3_w512a_v08171_a3m50_x30@gui-timing | 74.8 | 3685.0 | 0 | 0 | 1 | 267.5 | fast |  |
| 20261012-061628 | B3_w512a_v08171_a3m50_x30@gui-timing | 76.148 | 3717.2 | 0 | 0 | 1 | 267.2 | fast |  |
| 20261012-061625 | B3_w512a_v08171_a3m50_x30@gui-timing | 75.862 | 3628.1 | 0 | 0 | 1 | 267.2 | fast |  |
| 20261012-061621 | B3_w512a_v08171_a3m50_x30@gui-timing | 75.878 | 3626.7 | 0 | 0 | 1 | 267.2 | fast |  |
| 20261012-061618 | B3_w512a_v08171_a3m50_x30@gui-timing | 75.984 | 3620.6 | 0 | 0 | 1 | 267.0 | fast |  |
| 20261012-061614 | B3_w512a_v08171_a3m50_x30@gui-timing | 74.802 | 3687.3 | 0 | 0 | 1 | 267.5 | fast |  |
| 20261012-061611 | B3_w512a_v08171_a3m50_x30@gui-timing | 74.946 | 3677.6 | 0 | 0 | 1 | 267.5 | fast |  |
| 20261012-061607 | B3_w512a_v08171_a3m50_x30@gui-timing | 75.17 | 3665.6 | 0 | 0 | 1 | 266.9 | fast |  |
| 20261012-061603 | B3_w512a_v08171_a3m50_x30@gui-timing | 74.882 | 3681.0 | 0 | 0 | 1 | 267.5 | fast |  |
| 20261012-061600 | B3_w512a_v08171_a3m50_x30@gui-timing | 75.358 | 3657.5 | 0 | 0 | 1 | 267.0 | fast |  |
| 20261012-061556 | B3_w512a_v08171_a3m50_x30@gui-timing | 76.262 | 3711.9 | 0 | 0 | 1 | 266.9 | fast |  |
| 20261012-061553 | B3_w512a_v08171_a3m50_x30@gui-timing | 76.28 | 3713.9 | 0 | 0 | 1 | 266.9 | fast |  |
| 20261012-061548 | B3_w512a_v08171_a3m50_x30@gui-timing | 74.49 | 3700.6 | 0 | 0 | 1 | 267.5 | fast |  |
| 20261012-061545 | B3_w512a_v08171_a3m50_x30@gui-timing | 76.108 | 3722.8 | 0 | 0 | 1 | 267.0 | fast |  |
| 20261012-061541 | B3_w512a_v08171_a3m50_x30@gui-timing | 75.688 | 3637.5 | 0 | 0 | 1 | 267.1 | fast |  |
| 20261012-061537 | B3_w512a_v08171_a3m50_x30@gui-timing | 75.648 | 3639.7 | 0 | 0 | 1 | 267.1 | fast |  |
| 20261012-061534 | B3_w512a_v08171_a3m50_x30@gui-timing | 75.826 | 3629.7 | 0 | 0 | 1 | 267.0 | fast |  |
| 20261012-061531 | B3_w512a_v08171_a3m50_x30@gui-timing | 76.04 | 3721.8 | 0 | 0 | 1 | 266.9 | fast |  |
| 20261012-061527 | B3_w512a_v08171_a3m50_x30@gui-timing | 75.37 | 3655.2 | 0 | 0 | 1 | 267.9 | fast |  |
| 20261012-061524 | B3_w512a_v08171_a3m50_x30@gui-timing | 76.324 | 3709.7 | 0 | 0 | 1 | 267.1 | fast |  |
| 20261012-061521 | B3_w512a_v08171_a3m50_x30@gui-timing | 75.688 | 3637.4 | 0 | 0 | 1 | 267.2 | fast |  |
| 20261012-061518 | B3_w512a_v08171_a3m50_x30@gui-timing | 76.094 | 3719.9 | 0 | 0 | 1 | 267.2 | fast |  |
| 20261012-061514 | B3_w512a_v08171_a3m50_x30@gui-timing | 76.27 | 3711.7 | 0 | 0 | 1 | 267.1 | fast |  |
| 20261012-061511 | B3_w512a_v08171_a3m50_x30@gui-timing | 75.64 | 3640.2 | 0 | 0 | 1 | 267.3 | fast |  |
| 20261012-061506 | B3_w512a_v08171_a3m50_det | 74.29 | 3712.7 | 0 | 0 | 1 | 267.5 | fast |  |
| 20261012-061451 | B3_r02_x30@gui-timing | 73.556 | 3651.2 | 0 | 0 | 0 | 277.3 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061448 | B3_r02_x30@gui-timing | 74.674 | 3706.6 | 0 | 0 | 0 | 275.2 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061445 | B3_r02_x30@gui-timing | 74.77 | 3700.9 | 0 | 0 | 0 | 275.5 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061442 | B3_r02_x30@gui-timing | 73.636 | 3645.8 | 0 | 0 | 0 | 277.3 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061438 | B3_r02_x30@gui-timing | 74.568 | 3713.3 | 0 | 0 | 0 | 274.9 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061435 | B3_r02_x30@gui-timing | 74.62 | 3710.0 | 0 | 0 | 0 | 275.0 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061431 | B3_r02_x30@gui-timing | 73.1 | 3681.3 | 0 | 0 | 0 | 277.3 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061428 | B3_r02_x30@gui-timing | 74.778 | 3700.7 | 0 | 0 | 0 | 274.9 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061425 | B3_r02_x30@gui-timing | 74.57 | 3712.4 | 0 | 0 | 0 | 274.7 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061422 | B3_r02_x30@gui-timing | 73.96 | 3622.7 | 0 | 0 | 0 | 275.0 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061419 | B3_r02_x30@gui-timing | 74.57 | 3712.8 | 0 | 0 | 0 | 274.8 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061416 | B3_r02_x30@gui-timing | 74.446 | 3720.1 | 0 | 0 | 0 | 275.1 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061412 | B3_r02_x30@gui-timing | 73.974 | 3621.7 | 0 | 0 | 0 | 275.0 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061409 | B3_r02_x30@gui-timing | 74.534 | 3714.1 | 0 | 0 | 0 | 275.1 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061406 | B3_r02_x30@gui-timing | 74.748 | 3702.3 | 0 | 0 | 0 | 274.5 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061402 | B3_r02_x30@gui-timing | 74.062 | 3740.0 | 0 | 0 | 0 | 274.7 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061359 | B3_r02_x30@gui-timing | 74.5 | 3717.2 | 0 | 0 | 0 | 275.0 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061356 | B3_r02_x30@gui-timing | 74.6 | 3710.6 | 0 | 0 | 0 | 275.0 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061353 | B3_r02_x30@gui-timing | 74.742 | 3703.1 | 0 | 0 | 0 | 275.3 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061349 | B3_r02_x30@gui-timing | 73.94 | 3624.2 | 0 | 0 | 0 | 274.8 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061346 | B3_r02_x30@gui-timing | 74.506 | 3715.7 | 0 | 0 | 0 | 275.0 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061344 | B3_r02_x30@gui-timing | 74.702 | 3705.3 | 0 | 0 | 0 | 274.7 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061341 | B3_r02_x30@gui-timing | 74.718 | 3704.1 | 0 | 0 | 0 | 275.0 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061337 | B3_r02_x30@gui-timing | 73.618 | 3647.0 | 0 | 0 | 0 | 277.3 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061334 | B3_r02_x30@gui-timing | 74.602 | 3710.6 | 0 | 0 | 0 | 275.4 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061331 | B3_r02_x30@gui-timing | 74.534 | 3715.2 | 0 | 0 | 0 | 275.1 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061328 | B3_r02_x30@gui-timing | 74.182 | 3734.9 | 0 | 0 | 0 | 275.0 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061325 | B3_r02_x30@gui-timing | 74.424 | 3724.0 | 0 | 0 | 0 | 275.2 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061321 | B3_r02_x30@gui-timing | 74.362 | 3724.7 | 0 | 0 | 0 | 275.1 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-061317 | B3_r02_x30@gui-timing | 73.554 | 3651.4 | 0 | 0 | 0 | 277.3 | fast | Blok3 30/30 kuralı, sistem boş |
| 20261012-010659 | regress_v2builder_r02 | 73.098 | 3681.9 | 0 | 0 | 0 | 277.3 | fast |  |
| 20261010-202021 | SUBMISSION_v10d_dist_r02 | 73.098 | 3681.9 | 0 | 0 | 0 | 277.3 | fast | 4g/20m + 0.2s tepki payı |
| 20261010-201949 | B2_ctrl3g50@gui-timing | 73.93 | 3624.9 | 0 | 0 | 0 | 268.9 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201944 | B2_ctrl3g50@gui-timing | 74.066 | 3741.1 | 0 | 0 | 0 | 270.4 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201938 | B2_ctrl3g50@gui-timing | 74.268 | 3731.5 | 0 | 0 | 0 | 270.3 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201933 | B2_ctrl3g50@gui-timing | 74.322 | 3726.3 | 0 | 0 | 0 | 268.8 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201928 | B2_ctrl3g50@gui-timing | 73.938 | 3624.3 | 0 | 0 | 0 | 270.4 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201923 | B2_ctrl3g50@gui-timing | 74.486 | 3718.2 | 0 | 0 | 0 | 269.7 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201918 | B2_ctrl3g50@gui-timing | 74.154 | 3737.4 | 0 | 0 | 0 | 269.8 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201913 | B2_ctrl3g50@gui-timing | 74.41 | 3720.6 | 0 | 0 | 0 | 270.4 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201908 | B2_ctrl3g50@gui-timing | 73.918 | 3625.7 | 0 | 0 | 0 | 269.0 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201903 | B2_ctrl3g50@gui-timing | 74.28 | 3730.3 | 0 | 0 | 0 | 268.8 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201858 | B2_ctrl3g50@gui-timing | 74.004 | 3619.4 | 0 | 0 | 0 | 269.3 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201853 | B2_ctrl3g50@gui-timing | 73.768 | 3636.3 | 0 | 0 | 0 | 270.5 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201848 | B2_ctrl3g50@gui-timing | 74.228 | 3733.8 | 0 | 0 | 0 | 270.5 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201843 | B2_ctrl3g50@gui-timing | 73.966 | 3622.2 | 0 | 0 | 0 | 270.0 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201839 | B2_ctrl3g50@gui-timing | 74.17 | 3736.2 | 0 | 0 | 0 | 270.5 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201834 | B2_ctrl3g50@gui-timing | 74.068 | 3741.5 | 0 | 0 | 0 | 269.3 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201828 | B2_ctrl3g50@gui-timing | 73.818 | 3632.8 | 0 | 0 | 0 | 270.1 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201824 | B2_ctrl3g50@gui-timing | 73.908 | 3626.3 | 0 | 0 | 0 | 269.5 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201819 | B2_ctrl3g50@gui-timing | 74.096 | 3738.3 | 0 | 0 | 0 | 270.6 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201814 | B2_ctrl3g50@gui-timing | 74.366 | 3722.9 | 0 | 0 | 0 | 268.8 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201809 | B2_r0@gui-timing | 74.236 | 3732.9 | 0 | 0 | 0 | 282.1 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201804 | B2_r0@gui-timing | 73.966 | 3622.4 | 0 | 0 | 0 | 282.6 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201759 | B2_r0@gui-timing | 73.448 | 3659.2 | 0 | 0 | 1 | 279.4 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201741 | B2_r0@gui-timing | DNF | 506.5 | 25 | 2 | 1 | 249.3 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201736 | B2_r0@gui-timing | 73.616 | 3647.7 | 0 | 0 | 0 | 282.0 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201717 | B2_r0@gui-timing | DNF | 520.2 | 15 | 1 | 2 | 249.5 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201712 | B2_r0@gui-timing | 73.54 | 3652.9 | 0 | 0 | 0 | 280.8 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201707 | B2_r0@gui-timing | 73.784 | 3635.5 | 0 | 0 | 0 | 281.6 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201648 | B2_r0@gui-timing | DNF | 1597.1 | 10 | 1 | 1 | 276.5 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201644 | B2_r0@gui-timing | 74.098 | 3744.1 | 0 | 0 | 1 | 278.6 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201625 | B2_r0@gui-timing | DNF | 547.4 | 8 | 2 | 2 | 249.1 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201620 | B2_r0@gui-timing | 74.246 | 3734.9 | 0 | 0 | 1 | 279.0 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201559 | B2_r0@gui-timing | DNF | 488.5 | 0 | 0 | 2 | 249.4 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201554 | B2_r0@gui-timing | 73.488 | 3656.7 | 0 | 0 | 0 | 282.3 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201533 | B2_r0@gui-timing | DNF | 498.0 | 1 | 1 | 2 | 249.2 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201527 | B2_r0@gui-timing | 73.672 | 3643.8 | 0 | 0 | 0 | 277.1 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201522 | B2_r0@gui-timing | 73.626 | 3646.9 | 0 | 0 | 0 | 282.3 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201517 | B2_r0@gui-timing | 74.272 | 3730.9 | 0 | 0 | 1 | 278.7 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201513 | B2_r0@gui-timing | 74.072 | 3741.2 | 0 | 0 | 1 | 278.9 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201453 | B2_r0@gui-timing | DNF | 511.8 | 11 | 1 | 2 | 249.5 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201448 | B2_m0r04@gui-timing | 73.352 | 3665.5 | 0 | 0 | 0 | 277.4 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201442 | B2_m0r04@gui-timing | 74.112 | 3737.5 | 0 | 0 | 0 | 277.3 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201438 | B2_m0r04@gui-timing | 73.562 | 3651.2 | 0 | 0 | 0 | 277.3 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201433 | B2_m0r04@gui-timing | 73.828 | 3632.3 | 0 | 0 | 1 | 279.5 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201428 | B2_m0r04@gui-timing | 73.598 | 3648.7 | 0 | 0 | 1 | 277.6 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201423 | B2_m0r04@gui-timing | 73.95 | 3623.5 | 0 | 0 | 0 | 278.0 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201418 | B2_m0r04@gui-timing | 73.328 | 3666.7 | 0 | 0 | 0 | 277.7 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201413 | B2_m0r04@gui-timing | 73.568 | 3650.8 | 0 | 0 | 0 | 279.5 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201408 | B2_m0r04@gui-timing | 73.756 | 3637.4 | 0 | 0 | 0 | 277.6 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201403 | B2_m0r04@gui-timing | 73.856 | 3630.3 | 0 | 0 | 0 | 277.2 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201357 | B2_m0r04@gui-timing | 73.148 | 3678.9 | 0 | 0 | 0 | 279.5 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201352 | B2_m0r04@gui-timing | 73.78 | 3635.7 | 0 | 0 | 0 | 279.3 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201348 | B2_m0r04@gui-timing | 74.418 | 3723.8 | 0 | 0 | 0 | 277.5 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201342 | B2_m0r04@gui-timing | 73.874 | 3629.0 | 0 | 0 | 0 | 277.0 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201337 | B2_m0r04@gui-timing | 73.9 | 3627.1 | 0 | 0 | 0 | 278.9 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201332 | B2_m0r04@gui-timing | 74.17 | 3738.5 | 0 | 0 | 0 | 277.6 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201327 | B2_m0r04@gui-timing | 73.594 | 3649.1 | 0 | 0 | 0 | 277.0 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201322 | B2_m0r04@gui-timing | 73.58 | 3650.1 | 0 | 0 | 0 | 277.2 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201317 | B2_m0r04@gui-timing | 73.624 | 3646.6 | 0 | 0 | 1 | 279.6 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201313 | B2_m0r04@gui-timing | 73.51 | 3654.7 | 0 | 0 | 0 | 279.7 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201307 | B2_m10r03@gui-timing | 73.686 | 3642.5 | 0 | 0 | 1 | 278.6 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201302 | B2_m10r03@gui-timing | 73.49 | 3655.8 | 0 | 0 | 0 | 278.6 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201257 | B2_m10r03@gui-timing | 73.962 | 3622.6 | 0 | 0 | 0 | 275.9 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201252 | B2_m10r03@gui-timing | 73.89 | 3627.8 | 0 | 0 | 0 | 276.7 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201248 | B2_m10r03@gui-timing | 73.656 | 3644.7 | 0 | 0 | 0 | 276.3 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201242 | B2_m10r03@gui-timing | 74.096 | 3737.1 | 0 | 0 | 1 | 278.8 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201237 | B2_m10r03@gui-timing | 73.462 | 3657.7 | 0 | 0 | 0 | 276.6 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201233 | B2_m10r03@gui-timing | 74.914 | 3693.4 | 0 | 0 | 1 | 276.4 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201228 | B2_m10r03@gui-timing | 73.99 | 3620.5 | 0 | 0 | 0 | 276.4 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201223 | B2_m10r03@gui-timing | 74.348 | 3727.0 | 0 | 0 | 0 | 276.5 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201218 | B2_m10r03@gui-timing | 73.816 | 3633.1 | 0 | 0 | 0 | 278.6 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201213 | B2_m10r03@gui-timing | 74.114 | 3741.6 | 0 | 0 | 1 | 278.7 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201208 | B2_m10r03@gui-timing | 73.916 | 3625.9 | 0 | 0 | 1 | 277.1 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201202 | B2_m10r03@gui-timing | 73.604 | 3648.0 | 0 | 0 | 0 | 276.5 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201158 | B2_m10r03@gui-timing | 73.918 | 3625.8 | 0 | 0 | 0 | 278.5 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201153 | B2_m10r03@gui-timing | 73.974 | 3620.0 | 0 | 0 | 0 | 278.6 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201148 | B2_m10r03@gui-timing | 73.99 | 3620.6 | 0 | 0 | 0 | 278.6 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201143 | B2_m10r03@gui-timing | 73.892 | 3627.8 | 0 | 0 | 1 | 275.7 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201138 | B2_m10r03@gui-timing | 73.628 | 3646.8 | 0 | 0 | 1 | 278.2 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201132 | B2_m10r03@gui-timing | 73.962 | 3622.6 | 0 | 0 | 0 | 278.5 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201127 | B2_r05@gui-timing | 74.152 | 3738.2 | 0 | 0 | 0 | 269.1 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201122 | B2_r05@gui-timing | 73.692 | 3641.9 | 0 | 0 | 0 | 272.9 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201117 | B2_r05@gui-timing | 73.558 | 3651.2 | 0 | 0 | 0 | 272.6 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201112 | B2_r05@gui-timing | 73.874 | 3629.0 | 0 | 0 | 0 | 273.0 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201107 | B2_r05@gui-timing | 73.924 | 3625.2 | 0 | 0 | 0 | 272.4 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201102 | B2_r05@gui-timing | 73.37 | 3664.0 | 0 | 0 | 0 | 271.7 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201056 | B2_r05@gui-timing | 73.834 | 3631.7 | 0 | 0 | 0 | 272.7 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201051 | B2_r05@gui-timing | 74.036 | 3742.6 | 0 | 0 | 0 | 272.9 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201046 | B2_r05@gui-timing | 73.91 | 3626.4 | 0 | 0 | 0 | 272.8 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201040 | B2_r05@gui-timing | 74.056 | 3743.1 | 0 | 0 | 0 | 271.7 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201035 | B2_r05@gui-timing | 73.974 | 3621.7 | 0 | 0 | 0 | 272.9 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201029 | B2_r05@gui-timing | 73.376 | 3663.4 | 0 | 0 | 0 | 272.7 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201024 | B2_r05@gui-timing | 74.126 | 3740.1 | 0 | 0 | 0 | 271.5 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201018 | B2_r05@gui-timing | 73.748 | 3637.9 | 0 | 0 | 0 | 272.7 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201013 | B2_r05@gui-timing | 73.84 | 3631.4 | 0 | 0 | 0 | 272.9 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201008 | B2_r05@gui-timing | 73.952 | 3623.2 | 0 | 0 | 0 | 272.7 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-201002 | B2_r05@gui-timing | 73.8 | 3634.3 | 0 | 0 | 0 | 271.7 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200957 | B2_r05@gui-timing | 73.948 | 3623.6 | 0 | 0 | 0 | 271.5 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200952 | B2_r05@gui-timing | 73.912 | 3626.2 | 0 | 0 | 0 | 271.8 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200947 | B2_r05@gui-timing | 73.678 | 3642.8 | 0 | 0 | 0 | 272.7 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200942 | B2_r03@gui-timing | 74.17 | 3737.9 | 0 | 0 | 0 | 274.3 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200937 | B2_r03@gui-timing | 73.91 | 3626.4 | 0 | 0 | 0 | 274.2 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200932 | B2_r03@gui-timing | 73.946 | 3623.8 | 0 | 0 | 0 | 273.6 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200927 | B2_r03@gui-timing | 74.202 | 3733.9 | 0 | 0 | 0 | 273.2 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200922 | B2_r03@gui-timing | 73.608 | 3647.8 | 0 | 0 | 0 | 274.3 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200917 | B2_r03@gui-timing | 73.958 | 3622.9 | 0 | 0 | 0 | 273.7 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200911 | B2_r03@gui-timing | 73.25 | 3671.7 | 0 | 0 | 0 | 274.0 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200906 | B2_r03@gui-timing | 73.834 | 3631.8 | 0 | 0 | 0 | 274.3 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200901 | B2_r03@gui-timing | 73.936 | 3624.5 | 0 | 0 | 0 | 273.7 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200855 | B2_r03@gui-timing | 73.722 | 3639.7 | 0 | 0 | 0 | 274.2 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200849 | B2_r03@gui-timing | 74.238 | 3734.5 | 0 | 0 | 0 | 274.7 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200844 | B2_r03@gui-timing | 73.91 | 3626.4 | 0 | 0 | 0 | 273.8 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200840 | B2_r03@gui-timing | 73.972 | 3621.8 | 0 | 0 | 0 | 274.2 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200834 | B2_r03@gui-timing | 73.674 | 3643.2 | 0 | 0 | 0 | 274.5 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200827 | B2_r03@gui-timing | 73.434 | 3659.6 | 0 | 0 | 0 | 274.4 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200822 | B2_r03@gui-timing | 73.314 | 3667.8 | 0 | 0 | 0 | 275.0 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200815 | B2_r03@gui-timing | 73.578 | 3649.8 | 0 | 0 | 0 | 274.3 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200809 | B2_r03@gui-timing | 73.57 | 3650.3 | 0 | 0 | 0 | 274.7 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200804 | B2_r03@gui-timing | 73.876 | 3628.8 | 0 | 0 | 0 | 274.4 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200758 | B2_r03@gui-timing | 73.81 | 3633.5 | 0 | 0 | 0 | 274.4 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200753 | B2_r02@gui-timing | 74.09 | 3742.5 | 0 | 0 | 0 | 277.4 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200747 | B2_r02@gui-timing | 73.838 | 3631.4 | 0 | 0 | 0 | 277.3 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200742 | B2_r02@gui-timing | 73.532 | 3653.2 | 0 | 0 | 0 | 274.9 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200736 | B2_r02@gui-timing | 73.762 | 3637.1 | 0 | 0 | 0 | 273.8 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200731 | B2_r02@gui-timing | 73.448 | 3658.7 | 0 | 0 | 0 | 275.4 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200726 | B2_r02@gui-timing | 74.018 | 3618.4 | 0 | 0 | 0 | 275.1 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200721 | B2_r02@gui-timing | 73.328 | 3667.0 | 0 | 0 | 0 | 277.5 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200715 | B2_r02@gui-timing | 73.726 | 3639.5 | 0 | 0 | 0 | 274.6 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200709 | B2_r02@gui-timing | 73.674 | 3643.5 | 0 | 0 | 0 | 277.2 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200703 | B2_r02@gui-timing | 74.568 | 3714.9 | 0 | 0 | 0 | 276.4 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200658 | B2_r02@gui-timing | 74.084 | 3743.3 | 0 | 0 | 0 | 275.9 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200651 | B2_r02@gui-timing | 73.492 | 3655.6 | 0 | 0 | 0 | 277.4 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200645 | B2_r02@gui-timing | 73.18 | 3676.6 | 0 | 0 | 0 | 277.3 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200639 | B2_r02@gui-timing | 73.35 | 3665.3 | 0 | 0 | 0 | 277.4 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200632 | B2_r02@gui-timing | 73.174 | 3677.1 | 0 | 0 | 0 | 277.2 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200625 | B2_r02@gui-timing | 73.452 | 3658.6 | 0 | 0 | 0 | 275.0 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200619 | B2_r02@gui-timing | 73.284 | 3669.8 | 0 | 0 | 0 | 277.3 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200613 | B2_r02@gui-timing | 73.17 | 3677.2 | 0 | 0 | 0 | 277.3 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200607 | B2_r02@gui-timing | 73.076 | 3683.3 | 0 | 0 | 0 | 277.3 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-200602 | B2_r02@gui-timing | 74.166 | 3734.6 | 0 | 0 | 0 | 275.1 | fast | blok2 tepki payı, eğitim yükü altında |
| 20261010-051710 | B1R_a4m20@gui-timing | DNF | 497.8 | 22 | 3 | 2 | 249.0 | fast | blok1 tekrar ×20, CPU boş |
| 20261010-051706 | B1R_a4m20@gui-timing | 72.94 | 3693.0 | 0 | 0 | 0 | 282.2 | fast | blok1 tekrar ×20, CPU boş |
| 20261010-051659 | B1R_a4m20@gui-timing | DNF | 1535.2 | 1618 | 4 | 5 | 276.6 | fast | blok1 tekrar ×20, CPU boş |
| 20261010-051655 | B1R_a4m20@gui-timing | 75.084 | 3682.8 | 0 | 0 | 1 | 279.5 | fast | blok1 tekrar ×20, CPU boş |
| 20261010-051652 | B1R_a4m20@gui-timing | 73.482 | 3656.6 | 0 | 0 | 0 | 282.2 | fast | blok1 tekrar ×20, CPU boş |
| 20261010-051644 | B1R_a4m20@gui-timing | DNF | 487.8 | 2 | 1 | 2 | 249.2 | fast | blok1 tekrar ×20, CPU boş |
| 20261010-051640 | B1R_a4m20@gui-timing | 72.94 | 3693.0 | 0 | 0 | 0 | 282.2 | fast | blok1 tekrar ×20, CPU boş |
| 20261010-051636 | B1R_a4m20@gui-timing | 74.838 | 3698.0 | 0 | 0 | 1 | 278.9 | fast | blok1 tekrar ×20, CPU boş |
| 20261010-051632 | B1R_a4m20@gui-timing | 73.514 | 3654.4 | 0 | 0 | 0 | 282.2 | fast | blok1 tekrar ×20, CPU boş |
| 20261010-051629 | B1R_a4m20@gui-timing | 75.682 | 3642.9 | 0 | 0 | 2 | 279.1 | fast | blok1 tekrar ×20, CPU boş |
| 20261010-051626 | B1R_a4m20@gui-timing | 75.754 | 3637.8 | 0 | 0 | 2 | 279.5 | fast | blok1 tekrar ×20, CPU boş |
| 20261010-051618 | B1R_a4m20@gui-timing | DNF | 493.9 | 23 | 7 | 1 | 249.1 | fast | blok1 tekrar ×20, CPU boş |
| 20261010-051616 | B1R_a4m20@gui-timing | 74.626 | 3710.4 | 0 | 0 | 1 | 279.0 | fast | blok1 tekrar ×20, CPU boş |
| 20261010-051613 | B1R_a4m20@gui-timing | 74.816 | 3698.8 | 0 | 0 | 1 | 279.2 | fast | blok1 tekrar ×20, CPU boş |
| 20261010-051605 | B1R_a4m20@gui-timing | DNF | 513.6 | 16 | 2 | 1 | 248.4 | fast | blok1 tekrar ×20, CPU boş |
| 20261010-051602 | B1R_a4m20@gui-timing | 73.828 | 3632.3 | 0 | 0 | 0 | 279.3 | fast | blok1 tekrar ×20, CPU boş |
| 20261010-051559 | B1R_a4m20@gui-timing | 74.778 | 3701.6 | 0 | 0 | 1 | 278.9 | fast | blok1 tekrar ×20, CPU boş |
| 20261010-051556 | B1R_a4m20@gui-timing | 75.15 | 3678.2 | 0 | 0 | 2 | 279.7 | fast | blok1 tekrar ×20, CPU boş |
| 20261010-051548 | B1R_a4m20@gui-timing | DNF | 529.2 | 11 | 1 | 1 | 249.2 | fast | blok1 tekrar ×20, CPU boş |
| 20261010-051545 | B1R_a4m20@gui-timing | 75.258 | 3671.3 | 0 | 0 | 2 | 279.2 | fast | blok1 tekrar ×20, CPU boş |
| 20261010-051507 | SUBMISSION_v10d_dist@gui-timing | 74.152 | 3735.9 | 0 | 0 | 0 | 269.3 | fast | blok1 kontrol ×20, CPU boş |
| 20261010-051503 | SUBMISSION_v10d_dist@gui-timing | 74.244 | 3733.0 | 0 | 0 | 0 | 268.9 | fast | blok1 kontrol ×20, CPU boş |
| 20261010-051459 | SUBMISSION_v10d_dist@gui-timing | 73.582 | 3649.2 | 0 | 0 | 0 | 270.5 | fast | blok1 kontrol ×20, CPU boş |
| 20261010-051455 | SUBMISSION_v10d_dist@gui-timing | 74.744 | 3702.6 | 0 | 0 | 0 | 269.7 | fast | blok1 kontrol ×20, CPU boş |
| 20261010-051451 | SUBMISSION_v10d_dist@gui-timing | 73.612 | 3647.0 | 0 | 0 | 0 | 270.5 | fast | blok1 kontrol ×20, CPU boş |
| 20261010-051448 | SUBMISSION_v10d_dist@gui-timing | 74.804 | 3698.0 | 0 | 0 | 0 | 269.4 | fast | blok1 kontrol ×20, CPU boş |
| 20261010-051444 | SUBMISSION_v10d_dist@gui-timing | 74.132 | 3737.2 | 0 | 0 | 0 | 268.9 | fast | blok1 kontrol ×20, CPU boş |
| 20261010-051441 | SUBMISSION_v10d_dist@gui-timing | 74.33 | 3728.0 | 0 | 0 | 0 | 269.2 | fast | blok1 kontrol ×20, CPU boş |
| 20261010-051437 | SUBMISSION_v10d_dist@gui-timing | 74.776 | 3699.9 | 0 | 0 | 0 | 269.0 | fast | blok1 kontrol ×20, CPU boş |
| 20261010-051434 | SUBMISSION_v10d_dist@gui-timing | 74.782 | 3699.1 | 0 | 0 | 0 | 269.4 | fast | blok1 kontrol ×20, CPU boş |
| 20261010-051431 | SUBMISSION_v10d_dist@gui-timing | 74.894 | 3692.8 | 0 | 0 | 0 | 269.1 | fast | blok1 kontrol ×20, CPU boş |
| 20261010-051427 | SUBMISSION_v10d_dist@gui-timing | 74.136 | 3736.7 | 0 | 0 | 0 | 268.9 | fast | blok1 kontrol ×20, CPU boş |
| 20261010-051424 | SUBMISSION_v10d_dist@gui-timing | 74.484 | 3717.3 | 0 | 0 | 0 | 269.0 | fast | blok1 kontrol ×20, CPU boş |
| 20261010-051421 | SUBMISSION_v10d_dist@gui-timing | 74.65 | 3707.3 | 0 | 0 | 0 | 269.7 | fast | blok1 kontrol ×20, CPU boş |
| 20261010-051417 | SUBMISSION_v10d_dist@gui-timing | 74.168 | 3733.4 | 0 | 0 | 0 | 270.5 | fast | blok1 kontrol ×20, CPU boş |
| 20261010-051414 | SUBMISSION_v10d_dist@gui-timing | 74.786 | 3699.4 | 0 | 0 | 0 | 268.9 | fast | blok1 kontrol ×20, CPU boş |
| 20261010-051410 | SUBMISSION_v10d_dist@gui-timing | 74.938 | 3690.3 | 0 | 0 | 0 | 268.9 | fast | blok1 kontrol ×20, CPU boş |
| 20261010-051406 | SUBMISSION_v10d_dist@gui-timing | 74.144 | 3738.7 | 0 | 0 | 0 | 270.5 | fast | blok1 kontrol ×20, CPU boş |
| 20261010-051403 | SUBMISSION_v10d_dist@gui-timing | 74.856 | 3694.8 | 0 | 0 | 0 | 268.9 | fast | blok1 kontrol ×20, CPU boş |
| 20261010-051400 | SUBMISSION_v10d_dist@gui-timing | 75.0 | 3686.2 | 0 | 0 | 0 | 268.9 | fast | blok1 kontrol ×20, CPU boş |
| 20261010-051348 | SUBMISSION_v10d_dist_a4m20@gui-timing | 75.456 | 3658.4 | 0 | 0 | 2 | 279.1 | fast | blok1 final ×20, CPU boş |
| 20261010-051345 | SUBMISSION_v10d_dist_a4m20@gui-timing | 73.776 | 3636.1 | 0 | 0 | 0 | 278.7 | fast | blok1 final ×20, CPU boş |
| 20261010-051342 | SUBMISSION_v10d_dist_a4m20@gui-timing | 74.408 | 3722.7 | 0 | 0 | 1 | 279.5 | fast | blok1 final ×20, CPU boş |
| 20261010-051339 | SUBMISSION_v10d_dist_a4m20@gui-timing | 75.29 | 3669.4 | 0 | 0 | 2 | 279.7 | fast | blok1 final ×20, CPU boş |
| 20261010-051336 | SUBMISSION_v10d_dist_a4m20@gui-timing | 74.134 | 3742.2 | 0 | 0 | 1 | 281.9 | fast | blok1 final ×20, CPU boş |
| 20261010-051333 | SUBMISSION_v10d_dist_a4m20@gui-timing | 73.554 | 3651.8 | 0 | 0 | 0 | 282.2 | fast | blok1 final ×20, CPU boş |
| 20261010-051324 | SUBMISSION_v10d_dist_a4m20@gui-timing | DNF | 528.6 | 12 | 2 | 2 | 249.2 | fast | blok1 final ×20, CPU boş |
| 20261010-051321 | SUBMISSION_v10d_dist_a4m20@gui-timing | 75.682 | 3642.9 | 0 | 0 | 2 | 279.1 | fast | blok1 final ×20, CPU boş |
| 20261010-051318 | SUBMISSION_v10d_dist_a4m20@gui-timing | 75.61 | 3648.0 | 0 | 0 | 2 | 279.1 | fast | blok1 final ×20, CPU boş |
| 20261010-051310 | SUBMISSION_v10d_dist_a4m20@gui-timing | DNF | 450.3 | 0 | 0 | 2 | 248.4 | fast | blok1 final ×20, CPU boş |
| 20261010-051303 | SUBMISSION_v10d_dist_a4m20@gui-timing | DNF | 1538.8 | 0 | 0 | 2 | 276.6 | fast | blok1 final ×20, CPU boş |
| 20261010-051255 | SUBMISSION_v10d_dist_a4m20@gui-timing | DNF | 493.9 | 14 | 2 | 2 | 249.2 | fast | blok1 final ×20, CPU boş |
| 20261010-051248 | SUBMISSION_v10d_dist_a4m20@gui-timing | DNF | 513.3 | 12 | 2 | 1 | 248.7 | fast | blok1 final ×20, CPU boş |
| 20261010-051239 | SUBMISSION_v10d_dist_a4m20@gui-timing | DNF | 524.0 | 13 | 1 | 2 | 249.4 | fast | blok1 final ×20, CPU boş |
| 20261010-051236 | SUBMISSION_v10d_dist_a4m20@gui-timing | 74.538 | 3715.3 | 0 | 0 | 1 | 279.1 | fast | blok1 final ×20, CPU boş |
| 20261010-051228 | SUBMISSION_v10d_dist_a4m20@gui-timing | DNF | 517.7 | 9 | 1 | 1 | 249.1 | fast | blok1 final ×20, CPU boş |
| 20261010-051220 | SUBMISSION_v10d_dist_a4m20@gui-timing | DNF | 530.9 | 17 | 3 | 2 | 249.1 | fast | blok1 final ×20, CPU boş |
| 20261010-051213 | SUBMISSION_v10d_dist_a4m20@gui-timing | DNF | 516.5 | 16 | 1 | 2 | 249.1 | fast | blok1 final ×20, CPU boş |
| 20261010-051210 | SUBMISSION_v10d_dist_a4m20@gui-timing | 75.718 | 3640.5 | 0 | 0 | 2 | 279.9 | fast | blok1 final ×20, CPU boş |
| 20261010-051205 | SUBMISSION_v10d_dist_a4m20@gui-timing | 73.542 | 3652.5 | 0 | 0 | 0 | 282.3 | fast | blok1 final ×20, CPU boş |
| 20261010-051200 | SUBMISSION_v10d_dist_a4m20 | 72.94 | 3693.0 | 0 | 0 | 0 | 282.2 | fast | blok1 final, CPU boş |
| 20261010-005302 | B1_v10d_d3.0m35@gui-timing | 73.306 | 3668.0 | 0 | 0 | 0 | 272.5 | fast | blok1, eğitim yükü altında |
| 20261010-005254 | B1_v10d_d3.0m35@gui-timing | 73.55 | 3651.8 | 0 | 0 | 0 | 272.5 | fast | blok1, eğitim yükü altında |
| 20261010-005248 | B1_v10d_d3.0m35@gui-timing | 73.27 | 3670.5 | 0 | 0 | 0 | 272.6 | fast | blok1, eğitim yükü altında |
| 20261010-005240 | B1_v10d_d3.0m35@gui-timing | 73.322 | 3666.9 | 0 | 0 | 0 | 272.4 | fast | blok1, eğitim yükü altında |
| 20261010-005233 | B1_v10d_d3.0m35@gui-timing | 73.42 | 3660.6 | 0 | 0 | 0 | 272.4 | fast | blok1, eğitim yükü altında |
| 20261010-005226 | B1_v10d_d3.0m35@gui-timing | 73.706 | 3640.8 | 0 | 0 | 0 | 272.6 | fast | blok1, eğitim yükü altında |
| 20261010-005219 | B1_v10d_d3.0m35@gui-timing | 73.25 | 3671.7 | 0 | 0 | 0 | 272.7 | fast | blok1, eğitim yükü altında |
| 20261010-005212 | B1_v10d_d3.0m35@gui-timing | 73.366 | 3664.1 | 0 | 0 | 0 | 271.1 | fast | blok1, eğitim yükü altında |
| 20261010-005205 | B1_v10d_d3.0m35@gui-timing | 73.47 | 3657.2 | 0 | 0 | 0 | 272.5 | fast | blok1, eğitim yükü altında |
| 20261010-005157 | B1_v10d_d3.0m35@gui-timing | 73.554 | 3651.7 | 0 | 0 | 0 | 272.6 | fast | blok1, eğitim yükü altında |
| 20261010-005149 | B1_v10d_d3.0m35 | 73.306 | 3668.0 | 0 | 0 | 0 | 272.5 | fast | blok1 yardımcı ayarı (eğitim yükü altında) |
| 20261010-005141 | B1_v10d_d2.5m20@gui-timing | 73.382 | 3663.1 | 0 | 0 | 0 | 272.1 | fast | blok1, eğitim yükü altında |
| 20261010-005134 | B1_v10d_d2.5m20@gui-timing | 73.298 | 3668.2 | 0 | 0 | 0 | 272.1 | fast | blok1, eğitim yükü altında |
| 20261010-005127 | B1_v10d_d2.5m20@gui-timing | 73.742 | 3638.2 | 0 | 0 | 0 | 272.2 | fast | blok1, eğitim yükü altında |
| 20261010-005121 | B1_v10d_d2.5m20@gui-timing | 73.394 | 3661.8 | 0 | 0 | 0 | 272.0 | fast | blok1, eğitim yükü altında |
| 20261010-005114 | B1_v10d_d2.5m20@gui-timing | 73.364 | 3664.2 | 0 | 0 | 0 | 271.7 | fast | blok1, eğitim yükü altında |
| 20261010-005106 | B1_v10d_d2.5m20@gui-timing | 73.364 | 3664.1 | 0 | 0 | 0 | 272.1 | fast | blok1, eğitim yükü altında |
| 20261010-005059 | B1_v10d_d2.5m20@gui-timing | 73.308 | 3668.0 | 0 | 0 | 0 | 272.1 | fast | blok1, eğitim yükü altında |
| 20261010-005052 | B1_v10d_d2.5m20@gui-timing | 73.306 | 3668.0 | 0 | 0 | 0 | 272.2 | fast | blok1, eğitim yükü altında |
| 20261010-005044 | B1_v10d_d2.5m20@gui-timing | 73.404 | 3661.6 | 0 | 0 | 0 | 272.1 | fast | blok1, eğitim yükü altında |
| 20261010-005037 | B1_v10d_d2.5m20@gui-timing | 73.306 | 3668.0 | 0 | 0 | 0 | 272.2 | fast | blok1, eğitim yükü altında |
| 20261010-005028 | B1_v10d_d2.5m20 | 73.306 | 3668.0 | 0 | 0 | 0 | 272.2 | fast | blok1 yardımcı ayarı (eğitim yükü altında) |
| 20261010-005019 | B1_v10d_d4.0m35@gui-timing | 73.218 | 3674.1 | 0 | 0 | 0 | 277.2 | fast | blok1, eğitim yükü altında |
| 20261010-005011 | B1_v10d_d4.0m35@gui-timing | 73.07 | 3683.7 | 0 | 0 | 0 | 277.3 | fast | blok1, eğitim yükü altında |
| 20261010-005004 | B1_v10d_d4.0m35@gui-timing | 73.088 | 3682.6 | 0 | 0 | 0 | 277.5 | fast | blok1, eğitim yükü altında |
| 20261010-004958 | B1_v10d_d4.0m35@gui-timing | 74.002 | 3619.6 | 0 | 0 | 0 | 277.1 | fast | blok1, eğitim yükü altında |
| 20261010-004951 | B1_v10d_d4.0m35@gui-timing | 73.198 | 3675.3 | 0 | 0 | 0 | 277.1 | fast | blok1, eğitim yükü altında |
| 20261010-004944 | B1_v10d_d4.0m35@gui-timing | 73.134 | 3679.4 | 0 | 0 | 0 | 277.2 | fast | blok1, eğitim yükü altında |
| 20261010-004937 | B1_v10d_d4.0m35@gui-timing | 73.374 | 3663.8 | 0 | 0 | 0 | 277.1 | fast | blok1, eğitim yükü altında |
| 20261010-004929 | B1_v10d_d4.0m35@gui-timing | 73.102 | 3681.7 | 0 | 0 | 0 | 277.2 | fast | blok1, eğitim yükü altında |
| 20261010-004922 | B1_v10d_d4.0m35@gui-timing | 73.58 | 3649.6 | 0 | 0 | 0 | 277.1 | fast | blok1, eğitim yükü altında |
| 20261010-004914 | B1_v10d_d4.0m35@gui-timing | 73.254 | 3671.8 | 0 | 0 | 0 | 277.3 | fast | blok1, eğitim yükü altında |
| 20261010-004906 | B1_v10d_d4.0m35 | 73.164 | 3677.6 | 0 | 0 | 0 | 277.2 | fast | blok1 yardımcı ayarı (eğitim yükü altında) |
| 20261010-004858 | B1_v10d_d4.0m20@gui-timing | 73.224 | 3674.4 | 0 | 0 | 0 | 281.9 | fast | blok1, eğitim yükü altında |
| 20261010-004851 | B1_v10d_d4.0m20@gui-timing | 73.584 | 3649.6 | 0 | 0 | 0 | 278.8 | fast | blok1, eğitim yükü altında |
| 20261010-004844 | B1_v10d_d4.0m20@gui-timing | 73.004 | 3688.7 | 0 | 0 | 0 | 282.1 | fast | blok1, eğitim yükü altında |
| 20261010-004837 | B1_v10d_d4.0m20@gui-timing | 73.156 | 3678.8 | 0 | 0 | 0 | 281.9 | fast | blok1, eğitim yükü altında |
| 20261010-004830 | B1_v10d_d4.0m20@gui-timing | 73.208 | 3675.2 | 0 | 0 | 0 | 282.2 | fast | blok1, eğitim yükü altında |
| 20261010-004823 | B1_v10d_d4.0m20@gui-timing | 73.112 | 3681.7 | 0 | 0 | 0 | 279.9 | fast | blok1, eğitim yükü altında |
| 20261010-004816 | B1_v10d_d4.0m20@gui-timing | 72.94 | 3693.0 | 0 | 0 | 0 | 282.2 | fast | blok1, eğitim yükü altında |
| 20261010-004808 | B1_v10d_d4.0m20@gui-timing | 73.028 | 3687.0 | 0 | 0 | 0 | 282.0 | fast | blok1, eğitim yükü altında |
| 20261010-004801 | B1_v10d_d4.0m20@gui-timing | 73.354 | 3665.7 | 0 | 0 | 0 | 282.1 | fast | blok1, eğitim yükü altında |
| 20261010-004754 | B1_v10d_d4.0m20@gui-timing | 73.19 | 3676.1 | 0 | 0 | 0 | 282.2 | fast | blok1, eğitim yükü altında |
| 20261010-004746 | B1_v10d_d4.0m20 | 72.94 | 3693.0 | 0 | 0 | 0 | 282.2 | fast | blok1 yardımcı ayarı (eğitim yükü altında) |
| 20261010-004738 | B1_v10d_d3.5m20@gui-timing | 73.394 | 3662.6 | 0 | 0 | 0 | 279.3 | fast | blok1, eğitim yükü altında |
| 20261010-004731 | B1_v10d_d3.5m20@gui-timing | 73.508 | 3654.8 | 0 | 0 | 1 | 279.3 | fast | blok1, eğitim yükü altında |
| 20261010-004724 | B1_v10d_d3.5m20@gui-timing | 73.092 | 3682.3 | 0 | 0 | 0 | 279.3 | fast | blok1, eğitim yükü altında |
| 20261010-004718 | B1_v10d_d3.5m20@gui-timing | 73.76 | 3637.2 | 0 | 0 | 0 | 279.3 | fast | blok1, eğitim yükü altında |
| 20261010-004711 | B1_v10d_d3.5m20@gui-timing | 73.418 | 3660.8 | 0 | 0 | 0 | 277.0 | fast | blok1, eğitim yükü altında |
| 20261010-004704 | B1_v10d_d3.5m20@gui-timing | 73.054 | 3684.9 | 0 | 0 | 0 | 279.3 | fast | blok1, eğitim yükü altında |
| 20261010-004658 | B1_v10d_d3.5m20@gui-timing | 73.8 | 3634.3 | 0 | 0 | 0 | 279.3 | fast | blok1, eğitim yükü altında |
| 20261010-004651 | B1_v10d_d3.5m20@gui-timing | 73.032 | 3686.1 | 0 | 0 | 0 | 279.4 | fast | blok1, eğitim yükü altında |
| 20261010-004644 | B1_v10d_d3.5m20@gui-timing | 73.038 | 3685.8 | 0 | 0 | 0 | 279.3 | fast | blok1, eğitim yükü altında |
| 20261010-004636 | B1_v10d_d3.5m20@gui-timing | 73.052 | 3685.0 | 0 | 0 | 0 | 279.3 | fast | blok1, eğitim yükü altında |
| 20261010-004628 | B1_v10d_d3.5m20 | 73.032 | 3686.1 | 0 | 0 | 0 | 279.4 | fast | blok1 yardımcı ayarı (eğitim yükü altında) |
| 20261010-004603 | B1_CONTROL@gui-timing | 73.504 | 3654.6 | 0 | 0 | 0 | 270.5 | fast | blok1, eğitim yükü altında |
| 20261010-004556 | B1_CONTROL@gui-timing | 73.724 | 3639.3 | 0 | 0 | 0 | 270.4 | fast | blok1, eğitim yükü altında |
| 20261010-004549 | B1_CONTROL@gui-timing | 73.734 | 3638.6 | 0 | 0 | 0 | 269.2 | fast | blok1, eğitim yükü altında |
| 20261010-004543 | B1_CONTROL@gui-timing | 74.324 | 3727.8 | 0 | 0 | 0 | 270.5 | fast | blok1, eğitim yükü altında |
| 20261010-004536 | B1_CONTROL@gui-timing | 73.974 | 3621.7 | 0 | 0 | 0 | 269.4 | fast | blok1, eğitim yükü altında |
| 20261010-004529 | B1_CONTROL@gui-timing | 73.73 | 3639.0 | 0 | 0 | 0 | 270.5 | fast | blok1, eğitim yükü altında |
| 20261010-004522 | B1_CONTROL@gui-timing | 73.424 | 3658.2 | 0 | 0 | 0 | 270.5 | fast | blok1, eğitim yükü altında |
| 20261010-004516 | B1_CONTROL@gui-timing | 73.622 | 3646.7 | 0 | 0 | 0 | 270.6 | fast | blok1, eğitim yükü altında |
| 20261010-004509 | B1_CONTROL@gui-timing | 73.774 | 3635.8 | 0 | 0 | 0 | 270.6 | fast | blok1, eğitim yükü altında |
| 20261010-004501 | B1_CONTROL@gui-timing | 73.726 | 3639.2 | 0 | 0 | 0 | 270.1 | fast | blok1, eğitim yükü altında |
| 20261010-004454 | B1_CONTROL | 73.582 | 3649.2 | 0 | 0 | 0 | 270.5 | fast | blok1 yardımcı ayarı (eğitim yükü altında) |
| 20261009-061840 | SUBMISSION_v10d_dist@gui-timing | 74.018 | 3617.4 | 0 | 0 | 0 | 270.5 | fast | CPU boş |
| 20261009-061837 | SUBMISSION_v10d_dist@gui-timing | 73.97 | 3621.9 | 0 | 0 | 0 | 268.9 | fast | CPU boş |
| 20261009-061834 | SUBMISSION_v10d_dist@gui-timing | 74.802 | 3698.3 | 0 | 0 | 0 | 269.0 | fast | CPU boş |
| 20261009-061831 | SUBMISSION_v10d_dist@gui-timing | 74.802 | 3697.9 | 0 | 0 | 0 | 269.1 | fast | CPU boş |
| 20261009-061828 | SUBMISSION_v10d_dist@gui-timing | 74.188 | 3732.4 | 0 | 0 | 0 | 269.0 | fast | CPU boş |
| 20261009-061825 | SUBMISSION_v10d_dist@gui-timing | 74.802 | 3698.2 | 0 | 0 | 0 | 269.7 | fast | CPU boş |
| 20261009-061822 | SUBMISSION_v10d_dist@gui-timing | 74.894 | 3692.8 | 0 | 0 | 0 | 269.1 | fast | CPU boş |
| 20261009-061818 | SUBMISSION_v10d_dist@gui-timing | 73.582 | 3649.2 | 0 | 0 | 0 | 270.5 | fast | CPU boş |
| 20261009-061814 | SUBMISSION_v10d_dist@gui-timing | 74.822 | 3697.2 | 0 | 0 | 0 | 268.9 | fast | CPU boş |
| 20261009-061811 | SUBMISSION_v10d_dist@gui-timing | 74.076 | 3739.2 | 0 | 0 | 0 | 270.5 | fast | CPU boş |
| 20261009-061807 | SUBMISSION_v10d_dist | 73.582 | 3649.2 | 0 | 0 | 0 | 270.5 | fast | v10d/v04188 + mesafe 3g50m + vites 18650 |
| 20261009-015546 | SUBMISSION_v10b_best_plain@gui-timing | 76.466 | 3711.7 | 0 | 0 | 0 | 249.7 | fast | CPU boş |
| 20261009-015543 | SUBMISSION_v10b_best_plain@gui-timing | 76.42 | 3714.2 | 0 | 0 | 0 | 249.8 | fast | CPU boş |
| 20261009-015539 | SUBMISSION_v10b_best_plain@gui-timing | 75.82 | 3631.6 | 0 | 0 | 0 | 249.2 | fast | CPU boş |
| 20261009-015537 | SUBMISSION_v10b_best_plain@gui-timing | 76.706 | 3698.4 | 0 | 0 | 0 | 249.8 | fast | CPU boş |
| 20261009-015534 | SUBMISSION_v10b_best_plain@gui-timing | 76.542 | 3707.4 | 0 | 0 | 0 | 249.8 | fast | CPU boş |
| 20261009-015530 | SUBMISSION_v10b_best_plain@gui-timing | 76.484 | 3710.6 | 0 | 0 | 0 | 249.8 | fast | CPU boş |
| 20261009-015527 | SUBMISSION_v10b_best_plain@gui-timing | 76.434 | 3713.4 | 0 | 0 | 0 | 249.8 | fast | CPU boş |
| 20261009-015523 | SUBMISSION_v10b_best_plain@gui-timing | 75.156 | 3672.8 | 0 | 0 | 0 | 249.2 | fast | CPU boş |
| 20261009-015520 | SUBMISSION_v10b_best_plain@gui-timing | 76.494 | 3710.3 | 0 | 0 | 0 | 249.8 | fast | CPU boş |
| 20261009-015517 | SUBMISSION_v10b_best_plain@gui-timing | 76.41 | 3714.8 | 0 | 0 | 0 | 249.9 | fast | CPU boş |
| 20261009-015514 | SUBMISSION_v10b_best_plain | 75.156 | 3672.8 | 0 | 0 | 0 | 249.2 | fast | v10 konum bilen |
| 20261009-015509 | SUBMISSION_v10b_v04324_dist@gui-timing | 75.19 | 3668.1 | 0 | 0 | 0 | 260.6 | fast | CPU boş |
| 20261009-015506 | SUBMISSION_v10b_v04324_dist@gui-timing | 75.83 | 3631.4 | 0 | 0 | 1 | 259.7 | fast | CPU boş |
| 20261009-015503 | SUBMISSION_v10b_v04324_dist@gui-timing | 75.932 | 3624.5 | 0 | 0 | 1 | 259.9 | fast | CPU boş |
| 20261009-015500 | SUBMISSION_v10b_v04324_dist@gui-timing | 75.288 | 3662.7 | 0 | 0 | 0 | 259.8 | fast | CPU boş |
| 20261009-015457 | SUBMISSION_v10b_v04324_dist@gui-timing | 75.522 | 3650.2 | 0 | 0 | 0 | 259.7 | fast | CPU boş |
| 20261009-015454 | SUBMISSION_v10b_v04324_dist@gui-timing | 75.66 | 3642.0 | 0 | 0 | 1 | 259.7 | fast | CPU boş |
| 20261009-015451 | SUBMISSION_v10b_v04324_dist@gui-timing | 75.934 | 3624.3 | 0 | 0 | 0 | 259.7 | fast | CPU boş |
| 20261009-015448 | SUBMISSION_v10b_v04324_dist@gui-timing | 75.684 | 3640.6 | 0 | 0 | 0 | 259.8 | fast | CPU boş |
| 20261009-015444 | SUBMISSION_v10b_v04324_dist@gui-timing | 75.162 | 3669.4 | 0 | 0 | 1 | 260.6 | fast | CPU boş |
| 20261009-015441 | SUBMISSION_v10b_v04324_dist@gui-timing | 75.674 | 3641.2 | 0 | 0 | 0 | 259.7 | fast | CPU boş |
| 20261009-015437 | SUBMISSION_v10b_v04324_dist | 74.416 | 3710.6 | 0 | 0 | 0 | 260.6 | fast | v10 konum bilen |
| 20261009-015433 | SUBMISSION_v10b_best_dist@gui-timing | 75.472 | 3655.3 | 0 | 0 | 0 | 260.1 | fast | CPU boş |
| 20261009-015429 | SUBMISSION_v10b_best_dist@gui-timing | 74.44 | 3717.8 | 0 | 0 | 0 | 259.7 | fast | CPU boş |
| 20261009-015426 | SUBMISSION_v10b_best_dist@gui-timing | 75.36 | 3662.4 | 0 | 0 | 0 | 260.1 | fast | CPU boş |
| 20261009-015423 | SUBMISSION_v10b_best_dist@gui-timing | 75.564 | 3649.4 | 0 | 0 | 0 | 260.1 | fast | CPU boş |
| 20261009-015421 | SUBMISSION_v10b_best_dist@gui-timing | 75.674 | 3642.1 | 0 | 0 | 0 | 260.1 | fast | CPU boş |
| 20261009-015418 | SUBMISSION_v10b_best_dist@gui-timing | 75.424 | 3658.4 | 0 | 0 | 0 | 260.1 | fast | CPU boş |
| 20261009-015414 | SUBMISSION_v10b_best_dist@gui-timing | 74.448 | 3717.3 | 0 | 0 | 0 | 259.9 | fast | CPU boş |
| 20261009-015411 | SUBMISSION_v10b_best_dist@gui-timing | 75.526 | 3651.8 | 0 | 0 | 0 | 260.1 | fast | CPU boş |
| 20261009-015408 | SUBMISSION_v10b_best_dist@gui-timing | 75.426 | 3658.2 | 0 | 0 | 0 | 260.0 | fast | CPU boş |
| 20261009-015405 | SUBMISSION_v10b_best_dist@gui-timing | 75.022 | 3683.4 | 0 | 0 | 0 | 260.0 | fast | CPU boş |
| 20261009-015401 | SUBMISSION_v10b_best_dist | 73.728 | 3638.6 | 0 | 0 | 0 | 259.7 | fast | v10 konum bilen |
| 20261008-234338 | TEST_sub44_v4d | 81.736 | 3630.1 | 0 | 0 | 0 | 251.0 | fast | v10 eşdeğerlik testi (silinebilir) |
| 20261008-141811 | SUBMISSION_s47_dist@gui-timing | 78.838 | 3679.8 | 0 | 0 | 0 | 259.4 | fast | CPU boş |
| 20261008-141808 | SUBMISSION_s47_dist@gui-timing | 78.24 | 3707.6 | 0 | 0 | 0 | 259.5 | fast | CPU boş |
| 20261008-141805 | SUBMISSION_s47_dist@gui-timing | 79.014 | 3671.2 | 0 | 0 | 0 | 259.0 | fast | CPU boş |
| 20261008-141802 | SUBMISSION_s47_dist@gui-timing | 79.056 | 3667.9 | 0 | 0 | 0 | 259.3 | fast | CPU boş |
| 20261008-141759 | SUBMISSION_s47_dist@gui-timing | 79.104 | 3666.8 | 0 | 0 | 0 | 259.3 | fast | CPU boş |
| 20261008-141756 | SUBMISSION_s47_dist@gui-timing | 79.09 | 3667.6 | 0 | 0 | 0 | 259.3 | fast | CPU boş |
| 20261008-141754 | SUBMISSION_s47_dist@gui-timing | 79.02 | 3670.9 | 0 | 0 | 0 | 259.0 | fast | CPU boş |
| 20261008-141751 | SUBMISSION_s47_dist@gui-timing | 78.902 | 3676.6 | 0 | 0 | 0 | 259.4 | fast | CPU boş |
| 20261008-141747 | SUBMISSION_s47_dist@gui-timing | 77.764 | 3633.0 | 0 | 0 | 0 | 260.7 | fast | CPU boş |
| 20261008-141744 | SUBMISSION_s47_dist@gui-timing | 79.238 | 3660.1 | 0 | 0 | 0 | 259.0 | fast | CPU boş |
| 20261008-141739 | SUBMISSION_s47_dist | 77.436 | 3650.3 | 0 | 0 | 0 | 260.7 | fast | n3b_s47 + mesafe yardımcısı 3.0g/50m + vites 18650 |
| 20261008-140917 | SUBMISSION_s47_assist@GUI | 78.154 | 3618.2 | 0 | 0 | 0 | 253.3 | gui |  |
| 20261008-071953 | SUBMISSION_v6g_assist@gui-timing | 79.528 | 3648.4 | 0 | 0 | 0 | 267.3 | fast | 07:20 karşılaştırma, CPU boş |
| 20261008-071950 | SUBMISSION_v6g_assist@gui-timing | 79.646 | 3641.5 | 0 | 0 | 0 | 267.6 | fast | 07:20 karşılaştırma, CPU boş |
| 20261008-071947 | SUBMISSION_v6g_assist@gui-timing | 79.856 | 3628.8 | 0 | 0 | 0 | 267.4 | fast | 07:20 karşılaştırma, CPU boş |
| 20261008-071944 | SUBMISSION_v6g_assist@gui-timing | 79.842 | 3629.7 | 0 | 0 | 0 | 267.0 | fast | 07:20 karşılaştırma, CPU boş |
| 20261008-071941 | SUBMISSION_v6g_assist@gui-timing | 79.606 | 3643.6 | 0 | 0 | 0 | 267.7 | fast | 07:20 karşılaştırma, CPU boş |
| 20261008-071938 | SUBMISSION_v6g_assist@gui-timing | 79.42 | 3654.6 | 0 | 0 | 0 | 267.6 | fast | 07:20 karşılaştırma, CPU boş |
| 20261008-071935 | SUBMISSION_v6g_assist@gui-timing | 80.158 | 3721.3 | 0 | 0 | 0 | 267.1 | fast | 07:20 karşılaştırma, CPU boş |
| 20261008-071932 | SUBMISSION_v6g_assist@gui-timing | 80.096 | 3724.5 | 0 | 0 | 0 | 267.2 | fast | 07:20 karşılaştırma, CPU boş |
| 20261008-071928 | SUBMISSION_v6g_assist@gui-timing | 79.39 | 3655.9 | 0 | 0 | 0 | 267.9 | fast | 07:20 karşılaştırma, CPU boş |
| 20261008-071925 | SUBMISSION_v6g_assist@gui-timing | 79.664 | 3640.4 | 0 | 0 | 0 | 267.6 | fast | 07:20 karşılaştırma, CPU boş |
| 20261008-071921 | SUBMISSION_s47_assist@gui-timing | 80.294 | 3704.5 | 0 | 0 | 0 | 253.0 | fast | 07:20 karşılaştırma, CPU boş |
| 20261008-071917 | SUBMISSION_s47_assist@gui-timing | 80.066 | 3714.8 | 0 | 0 | 0 | 253.0 | fast | 07:20 karşılaştırma, CPU boş |
| 20261008-071914 | SUBMISSION_s47_assist@gui-timing | 80.062 | 3715.2 | 0 | 0 | 0 | 252.9 | fast | 07:20 karşılaştırma, CPU boş |
| 20261008-071911 | SUBMISSION_s47_assist@gui-timing | 80.18 | 3709.8 | 0 | 0 | 0 | 253.6 | fast | 07:20 karşılaştırma, CPU boş |
| 20261008-071907 | SUBMISSION_s47_assist@gui-timing | 80.216 | 3708.3 | 0 | 0 | 0 | 253.3 | fast | 07:20 karşılaştırma, CPU boş |
| 20261008-071904 | SUBMISSION_s47_assist@gui-timing | 80.054 | 3715.7 | 0 | 0 | 0 | 253.0 | fast | 07:20 karşılaştırma, CPU boş |
| 20261008-071901 | SUBMISSION_s47_assist@gui-timing | 80.19 | 3709.5 | 0 | 0 | 0 | 253.1 | fast | 07:20 karşılaştırma, CPU boş |
| 20261008-071857 | SUBMISSION_s47_assist@gui-timing | 79.738 | 3633.1 | 0 | 0 | 0 | 253.3 | fast | 07:20 karşılaştırma, CPU boş |
| 20261008-071854 | SUBMISSION_s47_assist@gui-timing | 80.36 | 3701.7 | 0 | 0 | 0 | 253.1 | fast | 07:20 karşılaştırma, CPU boş |
| 20261008-071852 | SUBMISSION_s47_assist@gui-timing | 80.182 | 3709.7 | 0 | 0 | 0 | 253.0 | fast | 07:20 karşılaştırma, CPU boş |
| 20261008-071848 | SUBMISSION_s47_assist | 78.158 | 3710.9 | 0 | 0 | 0 | 253.3 | fast | n3b_s47 + yardımcı 180/18650 |
| 20261007-214043 | SUBMISSION_v6g_assist@gui-timing | 79.824 | 3630.7 | 0 | 0 | 0 | 267.8 | fast | CPU boş |
| 20261007-214040 | SUBMISSION_v6g_assist@gui-timing | 79.204 | 3667.3 | 0 | 0 | 0 | 267.9 | fast | CPU boş |
| 20261007-214037 | SUBMISSION_v6g_assist@gui-timing | 79.93 | 3624.3 | 0 | 0 | 0 | 267.3 | fast | CPU boş |
| 20261007-214034 | SUBMISSION_v6g_assist@gui-timing | 79.978 | 3621.3 | 0 | 0 | 0 | 267.5 | fast | CPU boş |
| 20261007-214031 | SUBMISSION_v6g_assist@gui-timing | 79.268 | 3663.6 | 0 | 0 | 0 | 267.9 | fast | CPU boş |
| 20261007-214028 | SUBMISSION_v6g_assist@gui-timing | 80.044 | 3727.1 | 0 | 0 | 0 | 267.6 | fast | CPU boş |
| 20261007-214025 | SUBMISSION_v6g_assist@gui-timing | 79.338 | 3658.9 | 0 | 0 | 0 | 267.9 | fast | CPU boş |
| 20261007-214022 | SUBMISSION_v6g_assist@gui-timing | 79.826 | 3630.6 | 0 | 0 | 0 | 267.3 | fast | CPU boş |
| 20261007-214019 | SUBMISSION_v6g_assist@gui-timing | 79.942 | 3623.7 | 0 | 0 | 0 | 267.5 | fast | CPU boş |
| 20261007-214016 | SUBMISSION_v6g_assist@gui-timing | 79.762 | 3634.4 | 0 | 0 | 0 | 267.3 | fast | CPU boş |
| 20261007-214011 | SUBMISSION_v6g_assist | 78.934 | 3681.6 | 0 | 0 | 0 | 267.9 | fast | v6g + düzlük yardımcısı 180 m + vites 18650 (eğitimsiz) |
| 20261007-101531 | SUBMISSION_v6g@GUI | 79.782 | 3617.7 | 0 | 0 | 0 | 264.2 | gui |  |
| 20261007-073450 | SUBMISSION_v6g@gui-timing | 80.672 | 3694.6 | 0 | 0 | 0 | 263.9 | fast | CPU boş |
| 20261007-073447 | SUBMISSION_v6g@gui-timing | 80.546 | 3701.0 | 0 | 0 | 0 | 263.8 | fast | CPU boş |
| 20261007-073444 | SUBMISSION_v6g@gui-timing | 80.13 | 3722.0 | 0 | 0 | 0 | 264.3 | fast | CPU boş |
| 20261007-073440 | SUBMISSION_v6g@gui-timing | 80.684 | 3692.6 | 0 | 0 | 0 | 263.7 | fast | CPU boş |
| 20261007-073437 | SUBMISSION_v6g@gui-timing | 80.656 | 3695.5 | 0 | 0 | 0 | 263.7 | fast | CPU boş |
| 20261007-073433 | SUBMISSION_v6g@gui-timing | 80.228 | 3717.3 | 0 | 0 | 0 | 264.5 | fast | CPU boş |
| 20261007-073430 | SUBMISSION_v6g@gui-timing | 80.658 | 3695.3 | 0 | 0 | 0 | 263.7 | fast | CPU boş |
| 20261007-073427 | SUBMISSION_v6g@gui-timing | 80.632 | 3696.6 | 0 | 0 | 0 | 264.1 | fast | CPU boş |
| 20261007-073424 | SUBMISSION_v6g@gui-timing | 80.576 | 3699.5 | 0 | 0 | 0 | 263.9 | fast | CPU boş |
| 20261007-073420 | SUBMISSION_v6g@gui-timing | 79.788 | 3632.9 | 0 | 0 | 0 | 264.2 | fast | CPU boş |
| 20261007-073415 | SUBMISSION_v6g | 79.796 | 3632.4 | 0 | 0 | 0 | 264.2 | fast | gece2 en iyi |
| 20261007-064435 | SUBMISSION_v6c_safe@gui-timing | 82.412 | 3710.1 | 0 | 0 | 0 | 251.5 | fast | CPU boş |
| 20261007-064432 | SUBMISSION_v6c_safe@gui-timing | 82.432 | 3709.2 | 0 | 0 | 0 | 251.5 | fast | CPU boş |
| 20261007-064428 | SUBMISSION_v6c_safe@gui-timing | 82.206 | 3720.7 | 0 | 0 | 0 | 251.2 | fast | CPU boş |
| 20261007-064425 | SUBMISSION_v6c_safe@gui-timing | 82.198 | 3721.1 | 0 | 0 | 0 | 251.5 | fast | CPU boş |
| 20261007-064421 | SUBMISSION_v6c_safe@gui-timing | 81.586 | 3645.4 | 0 | 0 | 0 | 251.1 | fast | CPU boş |
| 20261007-064418 | SUBMISSION_v6c_safe@gui-timing | 81.94 | 3623.9 | 0 | 0 | 0 | 251.1 | fast | CPU boş |
| 20261007-064414 | SUBMISSION_v6c_safe@gui-timing | 82.214 | 3720.3 | 0 | 0 | 0 | 251.5 | fast | CPU boş |
| 20261007-064411 | SUBMISSION_v6c_safe@gui-timing | 82.204 | 3720.7 | 0 | 0 | 0 | 251.5 | fast | CPU boş |
| 20261007-064408 | SUBMISSION_v6c_safe@gui-timing | 81.352 | 3659.0 | 0 | 0 | 0 | 251.1 | fast | CPU boş |
| 20261007-064405 | SUBMISSION_v6c_safe@gui-timing | 82.368 | 3712.4 | 0 | 0 | 0 | 251.5 | fast | CPU boş |
| 20261007-064400 | SUBMISSION_v6c_safe | 81.352 | 3659.0 | 0 | 0 | 0 | 251.1 | fast | gece2 teslim adayı |
| 20261007-064351 | SUBMISSION_v6f_fast@gui-timing | DNF | 467.7 | 0 | 0 | 1 | 203.8 | fast | CPU boş |
| 20261007-064348 | SUBMISSION_v6f_fast@gui-timing | 80.606 | 3696.9 | 0 | 0 | 0 | 254.6 | fast | CPU boş |
| 20261007-064340 | SUBMISSION_v6f_fast@gui-timing | DNF | 468.5 | 0 | 0 | 1 | 203.7 | fast | CPU boş |
| 20261007-064333 | SUBMISSION_v6f_fast@gui-timing | DNF | 464.2 | 0 | 0 | 1 | 205.8 | fast | CPU boş |
| 20261007-064325 | SUBMISSION_v6f_fast@gui-timing | DNF | 464.3 | 0 | 0 | 1 | 205.8 | fast | CPU boş |
| 20261007-064317 | SUBMISSION_v6f_fast@gui-timing | DNF | 466.4 | 0 | 0 | 1 | 204.3 | fast | CPU boş |
| 20261007-064313 | SUBMISSION_v6f_fast@gui-timing | 79.834 | 3629.8 | 0 | 0 | 0 | 254.6 | fast | CPU boş |
| 20261007-064309 | SUBMISSION_v6f_fast@gui-timing | 80.816 | 3685.5 | 0 | 0 | 0 | 254.6 | fast | CPU boş |
| 20261007-064306 | SUBMISSION_v6f_fast@gui-timing | 79.806 | 3631.6 | 0 | 0 | 0 | 254.6 | fast | CPU boş |
| 20261007-064257 | SUBMISSION_v6f_fast@gui-timing | DNF | 468.6 | 0 | 0 | 1 | 204.0 | fast | CPU boş |
| 20261007-064252 | SUBMISSION_v6f_fast | 79.806 | 3631.6 | 0 | 0 | 0 | 254.6 | fast | gece2 teslim adayı |
| 20261007-064227 | sac_v6c_v01404 | 81.424 | 3654.8 | 0 | 0 | 0 | 251.0 | fast | gece2 finalist (TCS 5) |
| 20261007-064222 | sac_v6f_v01759 | 81.056 | 3670.7 | 0 | 0 | 0 | 247.6 | fast | gece2 finalist (TCS 5) |
| 20261007-064215 | sac_v6f_v00879 | 80.696 | 3690.8 | 0 | 0 | 0 | 249.9 | fast | gece2 finalist (TCS 5) |
| 20261007-064208 | sac_v6f_best | 79.864 | 3628.1 | 0 | 0 | 0 | 254.7 | fast | gece2 finalist (TCS 5) |
| 20261007-022954 | sac_v6b_v00249 | 81.182 | 3664.9 | 0 | 0 | 0 | 242.9 | fast | TCS 5 + safe_pos .9; S1 10/10 |
| 20261007-002521 | SUBMISSION_v4d@GUI | 81.718 | 3617.7 | 0 | 0 | 0 | 251.0 | gui |  |
| 20261007-002313 | SUBMISSION_v4d@GUI | 81.718 | 3617.7 | 0 | 0 | 0 | 251.0 | gui |  |
| 20261006-060722 | sac_v4e_v00248(hızlı-riskli)@gui-timing | 81.318 | 3659.1 | 0 | 0 | 0 | 245.0 | fast | CPU boş |
| 20261006-060717 | sac_v4e_v00248(hızlı-riskli)@gui-timing | 81.332 | 3658.3 | 0 | 0 | 0 | 245.1 | fast | CPU boş |
| 20261006-060712 | sac_v4e_v00248(hızlı-riskli)@gui-timing | 81.696 | 3637.9 | 0 | 0 | 0 | 243.6 | fast | CPU boş |
| 20261006-060707 | sac_v4e_v00248(hızlı-riskli)@gui-timing | 81.248 | 3662.7 | 0 | 0 | 0 | 245.2 | fast | CPU boş |
| 20261006-060702 | sac_v4e_v00248(hızlı-riskli)@gui-timing | 81.364 | 3656.7 | 0 | 0 | 0 | 244.8 | fast | CPU boş |
| 20261006-060657 | sac_v4e_v00248(hızlı-riskli)@gui-timing | 81.47 | 3650.7 | 0 | 0 | 0 | 245.1 | fast | CPU boş |
| 20261006-060653 | sac_v4e_v00248(hızlı-riskli)@gui-timing | 81.398 | 3654.7 | 0 | 0 | 0 | 245.3 | fast | CPU boş |
| 20261006-060642 | sac_v4e_v00248(hızlı-riskli)@gui-timing | DNF | 1603.6 | 25 | 1 | 1 | 242.1 | fast | CPU boş |
| 20261006-060631 | sac_v4e_v00248(hızlı-riskli)@gui-timing | DNF | 1628.2 | 377 | 1 | 1 | 242.7 | fast | CPU boş |
| 20261006-060626 | sac_v4e_v00248(hızlı-riskli)@gui-timing | 81.296 | 3660.3 | 0 | 0 | 0 | 245.0 | fast | CPU boş |
| 20261006-060619 | sac_v4e_v00248(hızlı-riskli) | 80.668 | 3693.8 | 0 | 0 | 0 | 244.3 | fast | final turda 17/20 hasarsız |
| 20261006-015636 | SUBMISSION_v4d(saf-python)@gui-timing | 82.832 | 3660.1 | 0 | 0 | 0 | 249.4 | fast | numpy gizli, CPU boş |
| 20261006-015627 | SUBMISSION_v4d(saf-python)@gui-timing | 82.92 | 3654.4 | 0 | 0 | 0 | 249.6 | fast | numpy gizli, CPU boş |
| 20261006-015619 | SUBMISSION_v4d(saf-python)@gui-timing | 82.544 | 3668.6 | 0 | 0 | 0 | 244.8 | fast | numpy gizli, CPU boş |
| 20261006-015610 | SUBMISSION_v4d(saf-python)@gui-timing | 83.118 | 3648.1 | 0 | 0 | 0 | 249.1 | fast | numpy gizli, CPU boş |
| 20261006-015602 | SUBMISSION_v4d(saf-python)@gui-timing | 82.878 | 3656.3 | 0 | 0 | 0 | 249.4 | fast | numpy gizli, CPU boş |
| 20261006-015559 | SUBMISSION_v4d(numpy)@gui-timing | 82.934 | 3654.8 | 0 | 0 | 0 | 249.5 | fast | teslim dosyası, CPU boş |
| 20261006-015556 | SUBMISSION_v4d(numpy)@gui-timing | 83.064 | 3650.6 | 0 | 0 | 0 | 249.5 | fast | teslim dosyası, CPU boş |
| 20261006-015553 | SUBMISSION_v4d(numpy)@gui-timing | 82.774 | 3659.2 | 0 | 0 | 0 | 249.4 | fast | teslim dosyası, CPU boş |
| 20261006-015549 | SUBMISSION_v4d(numpy)@gui-timing | 82.296 | 3673.8 | 0 | 0 | 0 | 251.0 | fast | teslim dosyası, CPU boş |
| 20261006-015546 | SUBMISSION_v4d(numpy)@gui-timing | 82.52 | 3667.2 | 0 | 0 | 0 | 249.0 | fast | teslim dosyası, CPU boş |
| 20261006-015543 | SUBMISSION_v4d(numpy)@gui-timing | 82.886 | 3656.1 | 0 | 0 | 0 | 249.4 | fast | teslim dosyası, CPU boş |
| 20261006-015540 | SUBMISSION_v4d(numpy)@gui-timing | 82.586 | 3665.2 | 0 | 0 | 0 | 249.5 | fast | teslim dosyası, CPU boş |
| 20261006-015536 | SUBMISSION_v4d(numpy)@gui-timing | 81.736 | 3630.1 | 0 | 0 | 0 | 251.0 | fast | teslim dosyası, CPU boş |
| 20261006-015533 | SUBMISSION_v4d(numpy)@gui-timing | 82.596 | 3664.4 | 0 | 0 | 0 | 249.1 | fast | teslim dosyası, CPU boş |
| 20261006-015529 | SUBMISSION_v4d(numpy)@gui-timing | 82.454 | 3672.1 | 0 | 0 | 0 | 251.0 | fast | teslim dosyası, CPU boş |
| 20261006-015524 | SUBMISSION_v4d(numpy) | 81.736 | 3630.1 | 0 | 0 | 0 | 251.0 | fast | teslim dosyası, gömülü v4d_v02537 |
| 20261006-015457 | sac_v4d_v02537[agent_v2]@gui-timing | 82.53 | 3666.5 | 0 | 0 | 0 | 247.2 | fast | gece seçimi #1, CPU boş |
| 20261006-015452 | sac_v4d_v02537[agent_v2]@gui-timing | 82.476 | 3669.4 | 0 | 0 | 0 | 249.5 | fast | gece seçimi #1, CPU boş |
| 20261006-015448 | sac_v4d_v02537[agent_v2]@gui-timing | 82.936 | 3656.4 | 0 | 0 | 0 | 249.5 | fast | gece seçimi #1, CPU boş |
| 20261006-015442 | sac_v4d_v02537[agent_v2]@gui-timing | 82.742 | 3661.2 | 0 | 0 | 0 | 249.5 | fast | gece seçimi #1, CPU boş |
| 20261006-015437 | sac_v4d_v02537[agent_v2]@gui-timing | 82.894 | 3655.6 | 0 | 0 | 0 | 249.8 | fast | gece seçimi #1, CPU boş |
| 20261006-015432 | sac_v4d_v02537[agent_v2]@gui-timing | 82.87 | 3656.6 | 0 | 0 | 0 | 249.5 | fast | gece seçimi #1, CPU boş |
| 20261006-015427 | sac_v4d_v02537[agent_v2]@gui-timing | 82.848 | 3657.9 | 0 | 0 | 0 | 249.5 | fast | gece seçimi #1, CPU boş |
| 20261006-015421 | sac_v4d_v02537[agent_v2]@gui-timing | 82.928 | 3654.8 | 0 | 0 | 0 | 249.9 | fast | gece seçimi #1, CPU boş |
| 20261006-015416 | sac_v4d_v02537[agent_v2]@gui-timing | 82.96 | 3652.8 | 0 | 0 | 0 | 249.5 | fast | gece seçimi #1, CPU boş |
| 20261006-015411 | sac_v4d_v02537[agent_v2]@gui-timing | 82.91 | 3656.5 | 0 | 0 | 0 | 249.5 | fast | gece seçimi #1, CPU boş |
| 20261006-015404 | sac_v4d_v02537 | 81.904 | 3623.0 | 0 | 0 | 0 | 251.2 | fast | gece seçimi #1 (20/20 hasarsız) |
| 20261006-015359 | sac_v4d_v02537 | 81.904 | 3623.0 | 0 | 0 | 0 | 251.2 | fast | gece seçimi #1 (20/20 hasarsız) |
| 20261005-225342 | SUBMISSION_v2(saf-python)@gui-timing | DNF | 814.5 | 0 | 0 | 1 | 177.8 | fast | numpy gizli, eğitim yükü altında |
| 20261005-225219 | SUBMISSION_v2(saf-python)@gui-timing | DNF | 2912.5 | 0 | 0 | 1 | 202.7 | fast | numpy gizli, eğitim yükü altında |
| 20261005-225052 | SUBMISSION_v2(saf-python)@gui-timing | DNF | 3231.9 | 1 | 1 | 1 | 221.1 | fast | numpy gizli, eğitim yükü altında |
| 20261005-224931 | SUBMISSION_v2(saf-python)@gui-timing | DNF | 831.4 | 0 | 0 | 1 | 178.7 | fast | numpy gizli, eğitim yükü altında |
| 20261005-224810 | SUBMISSION_v2(saf-python)@gui-timing | DNF | 2905.4 | 0 | 0 | 1 | 193.5 | fast | numpy gizli, eğitim yükü altında |
| 20261005-224749 | SUBMISSION_v2(saf-python) | 85.704 | 3633.2 | 0 | 0 | 0 | 231.8 | fast | numpy gizli, saf Python MLP |
| 20261005-224741 | SUBMISSION_v2(numpy)@gui-timing | 87.26 | 3648.9 | 0 | 0 | 0 | 231.8 | fast | teslim dosyası, eğitim yükü altında |
| 20261005-224733 | SUBMISSION_v2(numpy)@gui-timing | 87.36 | 3645.6 | 0 | 0 | 0 | 231.7 | fast | teslim dosyası, eğitim yükü altında |
| 20261005-224726 | SUBMISSION_v2(numpy)@gui-timing | 85.704 | 3633.2 | 0 | 0 | 0 | 231.8 | fast | teslim dosyası, eğitim yükü altında |
| 20261005-224718 | SUBMISSION_v2(numpy)@gui-timing | 85.694 | 3634.1 | 0 | 0 | 0 | 232.1 | fast | teslim dosyası, eğitim yükü altında |
| 20261005-224655 | SUBMISSION_v2(numpy)@gui-timing | DNF | 1552.6 | 368 | 1 | 2 | 225.8 | fast | teslim dosyası, eğitim yükü altında |
| 20261005-224646 | SUBMISSION_v2(numpy) | 85.704 | 3633.2 | 0 | 0 | 0 | 231.8 | fast | teslim dosyası, gömülü v2_ft_v05581 |
| 20261005-155629 | sac_v2_ft_v05581[agent_v2] | 85.736 | 3630.9 | 0 | 0 | 0 | 231.0 | fast | canlı telemetri testi |
| 20261005-154841 | sac_v2_ft_v05581[agent_v2]#etrack4 | 130.356 | 7162.4 | 0 | 0 | 2 | 241.4 | fast | Corkscrew'da eğitildi, E-Track 4 hiç görmedi |
| 20261005-154834 | sac_v2_ft_v05581[agent_v2]#etrack4 | 130.356 | 7162.4 | 0 | 0 | 2 | 241.4 | fast | Corkscrew'da eğitildi, E-Track 4 hiç görmedi |
| 20261005-154517 | sac_v2_ft_v05581@GUI | 130.346 | 7066.1 | 0 | 0 | 2 | 241.4 | gui |  |
| 20261005-154146 | sac_v2_ft_v05581@GUI | 130.346 | 7066.1 | 0 | 0 | 2 | 241.4 | gui |  |
| 20261005-153440 | sac_v2_ft_v05581[agent_v2]@gui-timing | 85.736 | 3630.9 | 0 | 0 | 0 | 231.0 | fast | agent v2: ısınma + kuyruk boşaltma |
| 20261005-153434 | sac_v2_ft_v05581[agent_v2]@gui-timing | 85.736 | 3630.9 | 0 | 0 | 0 | 231.0 | fast | agent v2: ısınma + kuyruk boşaltma |
| 20261005-153430 | sac_v2_ft_v05581[agent_v2]@gui-timing | 97.132 | 3649.7 | 0 | 0 | 0 | 202.8 | fast | agent v2: ısınma + kuyruk boşaltma |
| 20261005-153424 | sac_v2_ft_v05581[agent_v2]@gui-timing | 85.736 | 3630.9 | 0 | 0 | 0 | 231.0 | fast | agent v2: ısınma + kuyruk boşaltma |
| 20261005-153419 | sac_v2_ft_v05581[agent_v2]@gui-timing | 96.82 | 3657.1 | 0 | 0 | 0 | 202.0 | fast | agent v2: ısınma + kuyruk boşaltma |
| 20261005-153413 | sac_v2_ft_v05581[agent_v2]@gui-timing | 160.832 | 3659.8 | 24 | 1 | 1 | 202.8 | fast | agent v2: ısınma + kuyruk boşaltma |
| 20261005-153407 | sac_v2_ft_v05581[agent_v2]@gui-timing | 85.736 | 3630.9 | 0 | 0 | 0 | 231.0 | fast | agent v2: ısınma + kuyruk boşaltma |
| 20261005-153400 | sac_v2_ft_v05581[agent_v2]@gui-timing | 87.392 | 3641.5 | 0 | 0 | 0 | 231.0 | fast | agent v2: ısınma + kuyruk boşaltma |
| 20261005-153354 | sac_v2_ft_v05581[agent_v2]@gui-timing | 96.16 | 3675.6 | 0 | 0 | 0 | 203.0 | fast | agent v2: ısınma + kuyruk boşaltma |
| 20261005-153350 | sac_v2_ft_v05581[agent_v2]@gui-timing | 95.386 | 3642.3 | 0 | 0 | 1 | 203.7 | fast | agent v2: ısınma + kuyruk boşaltma |
| 20261005-153247 | sac_v2_ft_v05581@gui-timing | 88.818 | 3658.7 | 0 | 0 | 0 | 231.0 | fast | v2 seçim #1 |
| 20261005-153235 | sac_v2_ft_v05581@gui-timing | DNF | 1535.5 | 1 | 1 | 2 | 202.8 | fast | v2 seçim #1 |
| 20261005-153224 | sac_v2_ft_v05581@gui-timing | DNF | 1527.7 | 305 | 2 | 1 | 202.8 | fast | v2 seçim #1 |
| 20261005-153214 | sac_v2_ft_v05581@gui-timing | DNF | 976.7 | 186 | 1 | 1 | 173.0 | fast | v2 seçim #1 |
| 20261005-153202 | sac_v2_ft_v05581@gui-timing | DNF | 944.6 | 411 | 1 | 1 | 202.8 | fast | v2 seçim #1 |
| 20261005-153157 | sac_v2_ft_v05581@gui-timing | 85.736 | 3630.9 | 0 | 0 | 0 | 231.0 | fast | v2 seçim #1 |
| 20261005-153146 | sac_v2_ft_v05581@gui-timing | DNF | 979.0 | 308 | 1 | 1 | 178.6 | fast | v2 seçim #1 |
| 20261005-153135 | sac_v2_ft_v05581@gui-timing | DNF | 1062.4 | 0 | 0 | 0 | 202.8 | fast | v2 seçim #1 |
| 20261005-153125 | sac_v2_ft_v05581@gui-timing | DNF | 3316.5 | 0 | 0 | 1 | 203.7 | fast | v2 seçim #1 |
| 20261005-153114 | sac_v2_ft_v05581@gui-timing | DNF | 1869.0 | 39 | 1 | 1 | 201.2 | fast | v2 seçim #1 |
| 20261005-153107 | sac_v2_ft_v05581 | 85.736 | 3630.9 | 0 | 0 | 0 | 231.0 | fast | v2 seçim #1 (10/10 hasarsız) |
| 20261005-153101 | sac_v2_ft_v05581 | 85.736 | 3630.9 | 0 | 0 | 0 | 231.0 | fast | v2 seçim #1 (10/10 hasarsız) |
| 20261005-151924 | snakeoil3_gym#etrack4@GUI | 113.684 | 7065.6 | 1585 | 7 | 5 | 302.3 | gui |  |
| 20261005-151545 | snakeoil3_gym#etrack4@GUI | 113.684 | 7065.6 | 1585 | 7 | 5 | 302.3 | gui |  |
| 20261005-131145 | sac_g995_best@gui-timing | 95.276 | 3641.2 | 0 | 0 | 0 | 249.1 | fast | g995 best.pt (60. dk) |
| 20261005-131135 | sac_g995_best@gui-timing | 87.612 | 3635.4 | 0 | 0 | 0 | 249.1 | fast | g995 best.pt (60. dk) |
| 20261005-131108 | sac_g995_best@gui-timing | DNF | 1520.8 | 0 | 0 | 1 | 225.8 | fast | g995 best.pt (60. dk) |
| 20261005-131041 | sac_g995_best@gui-timing | DNF | 1118.2 | 107 | 1 | 1 | 225.8 | fast | g995 best.pt (60. dk) |
| 20261005-131021 | sac_g995_best@gui-timing | DNF | 1599.3 | 24 | 1 | 1 | 225.8 | fast | g995 best.pt (60. dk) |
| 20261005-131002 | sac_g995_best@gui-timing | DNF | 1365.3 | 233 | 1 | 2 | 192.9 | fast | g995 best.pt (60. dk) |
| 20261005-130934 | sac_g995_best@gui-timing | DNF | 1710.9 | 0 | 0 | 2 | 223.2 | fast | g995 best.pt (60. dk) |
| 20261005-130916 | sac_g995_best@gui-timing | DNF | 3554.0 | 4 | 1 | 1 | 202.7 | fast | g995 best.pt (60. dk) |
| 20261005-130901 | sac_g995_best@gui-timing | DNF | 956.8 | 0 | 0 | 1 | 225.8 | fast | g995 best.pt (60. dk) |
| 20261005-130846 | sac_g995_best@gui-timing | DNF | 1194.4 | 0 | 0 | 2 | 181.6 | fast | g995 best.pt (60. dk) |
| 20261005-130837 | sac_g995_best | 87.612 | 3635.4 | 0 | 0 | 0 | 249.1 | fast | g995 best.pt (60. dk), seçimde 5/10 hasarsız |
| 20261005-130829 | sac_g995_best | 87.612 | 3635.4 | 0 | 0 | 0 | 249.1 | fast | g995 best.pt (60. dk), seçimde 5/10 hasarsız |
| 20261004-192033 | sac_g995_final(60dk)@gui-timing | DNF | 185.7 | 816 | 1 | 2 | 129.2 | fast | 1 saat eğitim sonu, gamma .995 |
| 20261004-192023 | sac_g995_final(60dk)@gui-timing | DNF | 993.3 | 0 | 0 | 2 | 225.7 | fast | 1 saat eğitim sonu, gamma .995 |
| 20261004-192013 | sac_g995_final(60dk)@gui-timing | DNF | 117.6 | 8 | 1 | 1 | 88.8 | fast | 1 saat eğitim sonu, gamma .995 |
| 20261004-192003 | sac_g995_final(60dk)@gui-timing | DNF | 1727.2 | 0 | 0 | 1 | 225.7 | fast | 1 saat eğitim sonu, gamma .995 |
| 20261004-191953 | sac_g995_final(60dk)@gui-timing | DNF | 959.7 | 32 | 1 | 1 | 210.0 | fast | 1 saat eğitim sonu, gamma .995 |
| 20261004-191943 | sac_g995_final(60dk)@gui-timing | DNF | 1023.2 | 0 | 0 | 2 | 189.9 | fast | 1 saat eğitim sonu, gamma .995 |
| 20261004-191933 | sac_g995_final(60dk)@gui-timing | DNF | 1111.9 | 60 | 1 | 1 | 226.0 | fast | 1 saat eğitim sonu, gamma .995 |
| 20261004-191922 | sac_g995_final(60dk)@gui-timing | DNF | 412.4 | 283 | 1 | 1 | 157.1 | fast | 1 saat eğitim sonu, gamma .995 |
| 20261004-191912 | sac_g995_final(60dk)@gui-timing | DNF | 997.2 | 1 | 1 | 1 | 225.7 | fast | 1 saat eğitim sonu, gamma .995 |
| 20261004-191902 | sac_g995_final(60dk)@gui-timing | DNF | 1353.9 | 192 | 1 | 2 | 225.7 | fast | 1 saat eğitim sonu, gamma .995 |
| 20261004-191846 | sac_g995_final(60dk) | DNF | 2471.4 | 399 | 1 | 1 | 253.3 | fast | 1 saat eğitim sonu, gamma .995 |
| 20261004-191831 | sac_g995_final(60dk) | DNF | 2471.4 | 399 | 1 | 1 | 253.3 | fast | 1 saat eğitim sonu, gamma .995 |
| 20261004-191611 | sac_g995_best(53dk)@gui-timing | 171.85 | 3619.9 | 0 | 0 | 4 | 221.9 | fast | eğitim sürerken alınan best.pt kopyası |
| 20261004-191551 | sac_g995_best(53dk)@gui-timing | DNF | 498.3 | 217 | 1 | 1 | 231.7 | fast | eğitim sürerken alınan best.pt kopyası |
| 20261004-191543 | sac_g995_best(53dk)@gui-timing | 103.642 | 3633.1 | 0 | 0 | 0 | 223.2 | fast | eğitim sürerken alınan best.pt kopyası |
| 20261004-191526 | sac_g995_best(53dk)@gui-timing | DNF | 1558.5 | 0 | 0 | 1 | 223.6 | fast | eğitim sürerken alınan best.pt kopyası |
| 20261004-191508 | sac_g995_best(53dk)@gui-timing | DNF | 1587.9 | 0 | 0 | 1 | 222.6 | fast | eğitim sürerken alınan best.pt kopyası |
| 20261004-191450 | sac_g995_best(53dk)@gui-timing | DNF | 498.3 | 217 | 1 | 1 | 231.7 | fast | eğitim sürerken alınan best.pt kopyası |
| 20261004-191432 | sac_g995_best(53dk)@gui-timing | DNF | 2921.0 | 0 | 0 | 1 | 228.2 | fast | eğitim sürerken alınan best.pt kopyası |
| 20261004-191415 | sac_g995_best(53dk)@gui-timing | DNF | 499.9 | 224 | 1 | 1 | 231.7 | fast | eğitim sürerken alınan best.pt kopyası |
| 20261004-191357 | sac_g995_best(53dk)@gui-timing | DNF | 1590.4 | 0 | 0 | 1 | 226.8 | fast | eğitim sürerken alınan best.pt kopyası |
| 20261004-191340 | sac_g995_best(53dk)@gui-timing | DNF | 1033.4 | 0 | 0 | 1 | 226.7 | fast | eğitim sürerken alınan best.pt kopyası |
| 20261004-191312 | sac_g995_best(53dk) | DNF | 498.3 | 217 | 1 | 1 | 231.7 | fast | eğitim sürerken alınan best.pt kopyası |
| 20261004-191241 | sac_g995_best(53dk) | DNF | 498.3 | 217 | 1 | 1 | 231.7 | fast | eğitim sürerken alınan best.pt kopyası |
| 20261004-181753 | sac_smoke2(3dk) | DNF | 489.2 | 133 | 1 | 1 | 243.0 | fast | 3 dk smoke eğitim, gamma .99 |
| 20261004-180619 | snakeoil3_gym#etrack4@gui-timing | 122.004 | 7071.5 | 1924 | 7 | 5 | 302.1 | fast | E-Track 4, GUI zamanlaması |
| 20261004-180616 | snakeoil3_gym#etrack4@gui-timing | 187.898 | 7074.9 | 2016 | 12 | 9 | 302.3 | fast | E-Track 4, GUI zamanlaması |
| 20261004-180608 | snakeoil3_gym#etrack4@gui-timing | DNF | 4912.2 | 1465 | 39 | 12 | 281.1 | fast | E-Track 4, GUI zamanlaması |
| 20261004-180602 | snakeoil3_gym#etrack4@gui-timing | 508.938 | 7116.7 | 1680 | 17 | 15 | 276.0 | fast | E-Track 4, GUI zamanlaması |
| 20261004-180558 | snakeoil3_gym#etrack4@gui-timing | 117.944 | 7076.0 | 1754 | 8 | 5 | 302.5 | fast | E-Track 4, GUI zamanlaması |
| 20261004-180553 | snakeoil3_gym#etrack4 | 113.698 | 7095.5 | 1585 | 7 | 5 | 302.3 | fast | E-Track 4, deterministik |
| 20261004-180550 | snakeoil3_gym#etrack4 | 113.698 | 7095.5 | 1585 | 7 | 5 | 302.3 | fast | E-Track 4, deterministik |
| 20261004-180528 | snakeoil3_gym@gui-timing | DNF | 549.0 | 463 | 3 | 1 | 242.9 | fast | baseline, GUI zamanlaması, bitirme oranı testi |
| 20261004-180523 | snakeoil3_gym@gui-timing | DNF | 2061.8 | 757 | 5 | 4 | 245.2 | fast | baseline, GUI zamanlaması, bitirme oranı testi |
| 20261004-180518 | snakeoil3_gym@gui-timing | DNF | 543.5 | 453 | 2 | 1 | 241.7 | fast | baseline, GUI zamanlaması, bitirme oranı testi |
| 20261004-180512 | snakeoil3_gym@gui-timing | DNF | 2472.4 | 1236 | 4 | 4 | 237.1 | fast | baseline, GUI zamanlaması, bitirme oranı testi |
| 20261004-180506 | snakeoil3_gym@gui-timing | DNF | 2048.1 | 791 | 5 | 4 | 245.2 | fast | baseline, GUI zamanlaması, bitirme oranı testi |
| 20261004-180501 | snakeoil3_gym@gui-timing | DNF | 543.5 | 488 | 2 | 1 | 246.6 | fast | baseline, GUI zamanlaması, bitirme oranı testi |
| 20261004-180458 | snakeoil3_gym@gui-timing | 134.16 | 3692.8 | 3410 | 7 | 6 | 247.3 | fast | baseline, GUI zamanlaması, bitirme oranı testi |
| 20261004-180452 | snakeoil3_gym@gui-timing | DNF | 2082.3 | 876 | 5 | 4 | 246.7 | fast | baseline, GUI zamanlaması, bitirme oranı testi |
| 20261004-180445 | snakeoil3_gym@gui-timing | DNF | 3366.5 | 2087 | 8 | 8 | 237.9 | fast | baseline, GUI zamanlaması, bitirme oranı testi |
| 20261004-180439 | snakeoil3_gym@gui-timing | DNF | 2470.7 | 1983 | 5 | 5 | 237.9 | fast | baseline, GUI zamanlaması, bitirme oranı testi |
| 20261004-180436 | snakeoil3_gym@gui-timing | 135.152 | 3651.5 | 2166 | 7 | 8 | 245.6 | fast | baseline, GUI zamanlaması, bitirme oranı testi |
| 20261004-180433 | snakeoil3_gym@gui-timing | 142.234 | 3695.3 | 2201 | 8 | 8 | 245.6 | fast | baseline, GUI zamanlaması, bitirme oranı testi |
| 20261004-180427 | snakeoil3_gym@gui-timing | DNF | 2088.0 | 664 | 6 | 4 | 241.8 | fast | baseline, GUI zamanlaması, bitirme oranı testi |
| 20261004-180421 | snakeoil3_gym@gui-timing | DNF | 2054.8 | 821 | 5 | 4 | 244.4 | fast | baseline, GUI zamanlaması, bitirme oranı testi |
| 20261004-180415 | snakeoil3_gym@gui-timing | DNF | 575.7 | 419 | 2 | 1 | 233.5 | fast | baseline, GUI zamanlaması, bitirme oranı testi |
| 20261004-180409 | snakeoil3_gym@gui-timing | DNF | 2043.0 | 856 | 5 | 3 | 245.6 | fast | baseline, GUI zamanlaması, bitirme oranı testi |
| 20261004-180405 | snakeoil3_gym@gui-timing | 142.02 | 3620.5 | 1950 | 7 | 7 | 237.9 | fast | baseline, GUI zamanlaması, bitirme oranı testi |
| 20261004-180359 | snakeoil3_gym@gui-timing | DNF | 2470.6 | 1973 | 5 | 5 | 237.9 | fast | baseline, GUI zamanlaması, bitirme oranı testi |
| 20261004-180354 | snakeoil3_gym@gui-timing | DNF | 2072.8 | 830 | 6 | 4 | 245.2 | fast | baseline, GUI zamanlaması, bitirme oranı testi |
| 20261004-180348 | snakeoil3_gym@gui-timing | DNF | 2091.6 | 850 | 5 | 3 | 246.7 | fast | baseline, GUI zamanlaması, bitirme oranı testi |
| 20261004-180109 | torcs_jm_par@gui-timing | DNF | 2467.4 | 277 | 3 | 4 | 102.5 | fast | baseline, GUI zamanlaması (-t 10ms) |
| 20261004-180103 | torcs_jm_par@gui-timing | DNF | 532.1 | 50 | 2 | 1 | 103.2 | fast | baseline, GUI zamanlaması (-t 10ms) |
| 20261004-180057 | torcs_jm_par@gui-timing | DNF | 2478.8 | 227 | 2 | 5 | 107.4 | fast | baseline, GUI zamanlaması (-t 10ms) |
| 20261004-180051 | torcs_jm_par@gui-timing | DNF | 2483.4 | 247 | 3 | 6 | 102.9 | fast | baseline, GUI zamanlaması (-t 10ms) |
| 20261004-180045 | torcs_jm_par@gui-timing | DNF | 550.1 | 39 | 2 | 1 | 104.5 | fast | baseline, GUI zamanlaması (-t 10ms) |
| 20261004-180034 | torcs_jm_par | DNF | 2511.4 | 242 | 3 | 5 | 101.8 | fast | baseline, deterministik (-t 1s) |
| 20261004-180024 | torcs_jm_par | DNF | 2511.4 | 242 | 3 | 5 | 101.8 | fast | baseline, deterministik (-t 1s) |
| 20261004-180018 | jmcncarai@gui-timing | DNF | 493.4 | 221 | 2 | 1 | 163.1 | fast | baseline, GUI zamanlaması (-t 10ms) |
| 20261004-180013 | jmcncarai@gui-timing | DNF | 479.1 | 335 | 4 | 1 | 163.7 | fast | baseline, GUI zamanlaması (-t 10ms) |
| 20261004-180007 | jmcncarai@gui-timing | DNF | 495.5 | 199 | 2 | 1 | 163.2 | fast | baseline, GUI zamanlaması (-t 10ms) |
| 20261004-180002 | jmcncarai@gui-timing | DNF | 493.9 | 228 | 3 | 1 | 163.1 | fast | baseline, GUI zamanlaması (-t 10ms) |
| 20261004-175956 | jmcncarai@gui-timing | DNF | 497.7 | 274 | 2 | 1 | 161.7 | fast | baseline, GUI zamanlaması (-t 10ms) |
| 20261004-175945 | jmcncarai | DNF | 497.8 | 274 | 2 | 1 | 161.7 | fast | baseline, deterministik (-t 1s) |
| 20261004-175935 | jmcncarai | DNF | 497.8 | 274 | 2 | 1 | 161.7 | fast | baseline, deterministik (-t 1s) |
| 20261004-175928 | snakeoil3_jm2@gui-timing | DNF | 2468.7 | 2384 | 5 | 5 | 240.7 | fast | baseline, GUI zamanlaması (-t 10ms) |
| 20261004-175923 | snakeoil3_jm2@gui-timing | 308.23 | 3708.5 | 2680 | 8 | 8 | 238.8 | fast | baseline, GUI zamanlaması (-t 10ms) |
| 20261004-175917 | snakeoil3_jm2@gui-timing | DNF | 2120.8 | 813 | 6 | 4 | 251.6 | fast | baseline, GUI zamanlaması (-t 10ms) |
| 20261004-175912 | snakeoil3_jm2@gui-timing | DNF | 2468.7 | 2454 | 6 | 6 | 251.7 | fast | baseline, GUI zamanlaması (-t 10ms) |
| 20261004-175905 | snakeoil3_jm2@gui-timing | DNF | 2468.7 | 2715 | 7 | 6 | 251.5 | fast | baseline, GUI zamanlaması (-t 10ms) |
| 20261004-175854 | snakeoil3_jm2 | DNF | 3298.2 | 2921 | 8 | 7 | 232.7 | fast | baseline, deterministik (-t 1s) |
| 20261004-175844 | snakeoil3_jm2 | DNF | 3298.2 | 2921 | 8 | 7 | 232.7 | fast | baseline, deterministik (-t 1s) |
| 20261004-175839 | snakeoil3_gym@gui-timing | DNF | 543.5 | 603 | 2 | 1 | 247.3 | fast | baseline, GUI zamanlaması (-t 10ms) |
| 20261004-175833 | snakeoil3_gym@gui-timing | DNF | 2084.0 | 772 | 5 | 4 | 238.1 | fast | baseline, GUI zamanlaması (-t 10ms) |
| 20261004-175827 | snakeoil3_gym@gui-timing | DNF | 3377.8 | 3145 | 7 | 8 | 245.6 | fast | baseline, GUI zamanlaması (-t 10ms) |
| 20261004-175822 | snakeoil3_gym@gui-timing | DNF | 2065.8 | 671 | 5 | 4 | 237.9 | fast | baseline, GUI zamanlaması (-t 10ms) |
| 20261004-175816 | snakeoil3_gym@gui-timing | DNF | 2470.7 | 1751 | 5 | 5 | 243.3 | fast | baseline, GUI zamanlaması (-t 10ms) |
| 20261004-175805 | snakeoil3_gym | DNF | 2470.6 | 1863 | 5 | 5 | 237.9 | fast | baseline, deterministik (-t 1s) |
| 20261004-175755 | snakeoil3_gym | DNF | 2470.6 | 1863 | 5 | 5 | 237.9 | fast | baseline, deterministik (-t 1s) |
