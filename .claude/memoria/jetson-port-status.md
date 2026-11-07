---
name: jetson-port-status
description: "Drone_20261013 C2-Explorer port to Jetson Orin Nano — what was changed, how runs are checked, cubicle_office crash findings (Oct 2026)"
metadata:
  node_type: memory
  type: project
  originSessionId: 5699029b-6b7e-46d8-96ae-7e00dc87d4c5
  modified: 2026-10-16T20:43:09.518Z
---

Drone_20261013 (newer archive of the C2-Explorer ROS2/PX4/Gazebo project, already on Jazzy from the dev PC) was ported to the Jetson on 2026-10-14 following Drone_20261004/Drone_fabio/Porting_Jetson_changes.pdf. Changes are listed in the Jetson section of LEGGIMI.md. PX4 .git was copied from the Drone_20261004 port (same v1.17.0 commit) with submodule worktree paths rewritten.

Checks used on a run: /tmp/c2x_run_multi/exploration.log (`visione esterna verso PX4` should grow ~50 per 5 s), `scripts/verifica_senza_gps.sh`, and the PX4 .ulg under PX4-Autopilot/build/px4_sitl_default/rootfs/instance_0/log/ — compare vehicle_visual_odometry vs vehicle_local_position_groundtruth (subtract the constant frame offset, ~[4.0, 13.46] m) to tell collision from FAST-LIO divergence.

cubicle_office LIO on the Jetson: two runs at VEL_MAX 1.0 (one with GUI, one headless) both collided ~500 s in (FAST-LIO accurate until impact); the user's run at VEL_MAX=0.7 headless was clean. office_small LIO test was clean. Archived run: risultati_prova/JETSON_cubicle_lio_v07 (clean, 3D map ok; its replay video still needs a clean re-render). On 2026-10-16 the user had the crashed _v10 run, the jerky v07 video and every punti.npz in risultati_prova deleted (5.9 → 2.5 GB): archived 3D maps can no longer be re-reconstructed, only viewed.

Since 2026-10-16 run_multi_exploration.sh auto-archives every run (user's request) into risultati_prova/<date>_<time>_<scenario>_<odom>_v<VEL>/ in rel_* format, without punti.npz (user keeps that folder lean); ARCHIVIA=0 disables it, reliability_campaign.sh sets it. Not yet exercised by a real flight.

Jetson performance work (2026-10-16), benchmark = cubicle_office lio VISITA=0 VEL_MAX=1.0, takeoff->end of exploration: desktop 234 s. Test 1 (Release -O3 -mcpu=native, path-cache expiry in planning cycles, camera ray-casting gated, registra_mappa without camera + incremental merge): 496 s, clean — kept. Test 2 (+ A*/optimizer time budgets x2 via prestazioni.fattore_tempo_cpu, + parallel connectivity graph 3 threads): 566 s, worse (serial cost matrix A* doubled). Launch now defaults FATTORE_TEMPO_CPU=1, CG_THREADS=1 (= test 1 behaviour); parallel graph alone untested. Remaining gap: path 56% longer than desktop, planning still ~2x. User is testing on their own.

Jetson Open3D 0.20 quirks (fixed in scripts, ARM-only): Poisson must run single-threaded (else "Failed to close loop" abort); off-screen renderer (Filament/Vulkan) returns all-black images for big meshes / 1080p, so render at 1280x720, decimate map to 250k tris, reuse last good frame (video_demo.rendi). Videos without GUI: video_replay3d.py / video_demo.py on an archived run (rel_ulog_1_0.ulg, rel_expl_1.log, rel_mappa3d_1_1/). Don't `pkill -f` a pattern that appears in your own command line — it kills the agent shell.

**Why:** the user is iterating on GPS-free flight on the Jetson and will ask to check run logs.
**How to apply:** when asked to check a run, use the ulog groundtruth comparison above; note `grep -r` in the agent shell skips .git dirs — use find for those.
