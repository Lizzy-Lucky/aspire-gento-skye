# Stage 1 实时进度（turning_on_radio，Gento Skye）

本文件由 `outputs/runs/gento-skye-radio/live_status.py` 每分钟从运行产出自动生成，不要手改。
数据来源：每次运行的 `outcome.json`、`evaluation/protocol.json`，以及各 seed 的 `seed-summary.md`。
「这次改了什么」「结果分析」摘自 seed-summary.md，过长的会被截断；完整内容见对应 seed 目录。

## 总览

_自动更新：2026-10-22 14:05 CST / 10-22 02:05 UTC-4。_

当前没有运行中的 Isaac 进程。
Stage 1 已成功的 seed：无（共 0 个，目标 26–35）。累计运行 88 次，基础设施故障 2 次。

| seed | 分区 | 尝试 | 成功的尝试 | 最近一次 | 状态 |
| --- | --- | --- | --- | --- | --- |
| 36 | 诊断 | 24 (+1 infra) | — | 未达 10 cm | 未成功 |
| 26 | Stage 1 | 13 | — | 未达 10 cm | 未成功 |
| 27 | Stage 1 | 8 | — | 未达 10 cm | 未成功 |
| 28 | Stage 1 | 14 | — | program_completed | 未成功 |
| 29 | Stage 1 | 1 | — | 未达 10 cm | 未成功 |
| 30 | Stage 1 | 28 (+1 infra) | — | no outcome | 未成功 |

## 技能库

| 文件 | 条目数（标题里带 seed 来源的） | 最后修改 |
| --- | --- | --- |
| `isaac-operations.md` | 6 | 10-22 11:39 |
| `mobile-grasp.md` | 16 | 10-22 10:45 |
| `navigation.md` | 1 | 10-22 11:39 |
| `search.md` | 3 | 10-22 10:45 |
| `time-budget.md` | 1 | 10-22 10:45 |

## 每次尝试（最新在上）

### seed 36（诊断）

| # | 结果 | 步数 | 协议 | 这次改了什么（或整条记录） | 结果分析 |
| --- | --- | --- | --- | --- | --- |
| 24 | 未达 10 cm | 542 | held lifted | re-measure the strap from above it up to 3 passes until the shift < 1.5 cm and the box is off the image edge; pinch 2 cm below the strap top. **SUCCESS** (task… |  |
| 23 | program_completed | 488 | lifted | harness fix + lift fallback (IK of the lift pose -> joint path -> execute_trajectory). Lift now plans and executes (EEF 0.658 -> 0.817), but the gripper went 0… |  |
| 22 | program_completed | 454 | lifted | creep only to 0.62 m; then turn in place so the radio sits 15 deg to the left (primary shoulder). Near miss: tilt 90 primary IK ok, TrajOpt x2 fail, joint path… |  |
| 21 | program_completed | 163 | lifted | creep to 0.52 m (<= 9 steps), pregrasp clearance 0.14, after the pregrasp measure the miss and correct with move_to_pose if > 3 cm. FAIL: crept 9 steps to 0.53… |  |
| 20 | program_completed | 646 | lifted | if TrajOpt fails but IK is ok, `plan_motion(ik.joint_positions)` (collision-checked straight joint path) and `execute_trajectory` it for the pregrasp. FAIL: jo… |  |
| 19 | program_completed | 159 | lifted | after the drive, creep toward the radio in 3 cm servo steps to 0.60 m while ik_here passes (step back on failure); tilt 90 gets a second plan_motion try. FAIL:… |  |
| 18 | program_completed | 154 | lifted | torso bend removed; each pregrasp candidate gets up to 3 solve_ik tries. FAIL: tilt 90 primary IK ok but plan_motion TrajOpt Fail; every other (tilt, arm) IK F… |  |
| 17 | program_completed | 468 | lifted | repeat 1 of the identical policy. FAIL: ik_here at_table True, then torso bend (TIMEOUT, residual 0.036) + reset_torso, then all 8 pregrasp IK Fail (as in atte… |  |
| 16 | 未达 10 cm | 742 | held lifted | same policy with the exclusion fix. **SUCCESS** (task_success, protocol held_and_lifted, 742 steps). "red strap" 0.812; handle (-1.145, -1.378, top 0.545); des… |  |
| 15 | program_completed | 729 | lifted | block 6b segments the wrist view with "radio handle"/"red strap"/"handle", rejects boxes > 50 % of the frame, pinches the handle top-down with fingers across i… | "red strap" 0.734 box [111,0,196,233]; handle center (-1.147, -1.378), top z 0.545, extents 0.257 x 0.131 x 0.069, axis (0.83, -0.55). Recenter ok; descend IK Fail with "ignored objects: none". Cause (harness): `_plan_e… |
| 14 | program_completed | 746 | lifted | block 6b re-segments "red radio" in the arm's wrist camera at pregrasp (clearance raised to 0.18 m), re-centers on the wrist cloud median xy and its 95th-perce… | the 0.539 wrist "red radio" was a whole-frame box (table included) -> median shifted 15 cm, top 0.563. Recenter ok, descend IK Fail, closed empty. Keyframe: the radio lies on its back, face (speaker, "Radio") up, a red … |
| 13 | program_completed | 720 | lifted | block 7 descend to grasp pose (fingers 2 cm below radio top), close; block 8 lift 0.15 m in odom z. pregrasp SUCCESS, descend SUCCESS (EEF z 0.736), close -> g… |  |
| 12 | program_completed | 615 | lifted | `ik_here()` probes IK of the current pose at start / after_search / at_table; if at_table fails, servo back 5 cm at a time (<= 25 cm) until it passes. | ik_here True at start/after_search/at_table (no backoff), tilt 90 primary IK+plan ok first try, move_to_pose pregrasp SUCCESS, EEF (-0.988, -1.38, 0.814). Attempt 11's current-pose IK Fail did not reproduce (cuRobo IK i… |
| 11 | program_completed | 468 | lifted | same policy, IK fix in harness; shadow/probe rows now carry `ik_error`. IK no longer crashes; now genuine `IK Fail` for all tilts AND for the current EEF pose … |  |
| 10 | program_completed | 466 | lifted | added an IK reach probe (current EEF pose, then 25/50/75/100 % toward the radio). Even IK of the arm's *current* pose failed -> not reach. Trace `call-*-solve-… |  |
| 9 | program_completed | 467 | lifted | Block 5 tries approach tilts 90/70/55/40 deg (approach in the vertical plane through base->radio, fingers along approach x handle axis), solve_ik first, plan_m… |  |
| 8 | program_completed | 467 | lifted | program_ok, 467 steps, 850 s wall. All 10 straight-down yaw offsets x 2 arms IK Fail (20 plan_motion calls, ~40 s each). Straight-down at 0.71 m is at the edge… |  |
| 7 | program_completed | 466 | lifted | program_ok, 466 steps. move_torso bend -0.4: TIMEOUT "torso did not converge" (residual 0.04 rad); radio still not seen after the bend. Far long axis (-0.67, 0… |  |
| 6 | program_completed | 461 | lifted | program_ok. Block 5: move_torso bend -0.4 returned ok=False (cause printed from attempt 7); radio not seen up close; far measurement kept. Shadow plan_motion t… |  |
| 5 | program_completed | 154 | lifted | program_ok, 154 steps. Block 4: "coffee table" score 0.941; plan_standoff_pose and plan_approach_pose goals both NOT free on the traversability map -> curobo r… |  |
| 4 | infra (disk_full) | 0 |  | program_error NameError: `getattr` does not exist in the policy sandbox. Report lost. |  |
| 4 | program_error | 89 |  | program_error NameError: `getattr` does not exist in the policy sandbox. Report lost. |  |
| 3 | program_completed | 89 |  | program_ok, 89 steps. Pixel-bearing turn 0.737 rad centered the radio (box [240,394,288,440], score 0.855). Cropped cloud (+-0.25 m around the median) 0.34 x 0… |  |
| 2 | program_completed | 75 |  | program_ok, 75 steps. Sweep scores by step (yaw): 0.032 (0.69), 0.004 (1.39), 0.001 (2.08), 0.018 (2.77), 0.691 (-2.82) -> radio found at step 5 (left-rear; pi… |  |
| 1 | program_completed | 0 |  | program_ok, 0 steps, reset 136 s. Radio not in view at reset (head faces fireplace, wall TV, TV stand). SAM3 best score 0.008 (the TV). localize_object still o… |  |

### seed 26（Stage 1）

| # | 结果 | 步数 | 协议 | 这次改了什么（或整条记录） | 结果分析 |
| --- | --- | --- | --- | --- | --- |
| 13 | 未达 10 cm | 594 | held lifted | block 7 closes, and if the gripper reads < 0.03 opens, returns to the pregrasp, turns the hand 90 deg about vertical and tries once more. | **SUCCESS** (task_success, held_and_lifted, 594 steps). near shift 0.417; tilt 90 primary IK+TrajOpt first try; pass0 handle/cable top (1.86, 2.111, 0.646) axis (0.136, 0.987); pass1 rejected (edge, jump 0.118); first o… |
| 12 | program_completed | 638 | lifted | later passes only refine (jump <= 6 cm, score >= 0.3, off edge); recenter accepted if within 3 cm. | radio upright with a black wire bail (two bars) over one end (wrist keyframe). Kept pass0 (cable/bail top 0.645), pass1 jump 0.118 rejected. Descend stopped 3 cm short (EEF z 0.776 vs 0.747), close -> 0.0, lift empty. |
| 11 | program_completed | 615 |  | prompts red strap (>=0.5) / black cable / handle / radio handle; later passes only replace the estimate if score >= 0.3 and off the edge. | pass0 'handle' 0.621 / 'black cable' 0.504 = cable bundle on the radio top (1.851, 2.106, top 0.646, 0.279 x 0.063 x 0.039 -> pinchable); pass1 jumped to the cord on the table (z 0.534) and the recenter ended CONTROLLER… |
| 10 | program_completed | 599 |  | after lowering the torso, re-measure the radio with the head camera (measure_radio) and use that center. | near re-measure shifted the center 0.446 m (block-3 far center was wrong; localize_object agreed with near). Pregrasp SUCCESS. Wrist pass0: 'handle' 0.715 was the colored cable bundle at one end; radio upright (red side… |
| 9 | program_completed | 294 |  | diag also retries with self-collision disabled and prints the self-overlap pairs of that solution. | (lucky) tilt 90 primary IK + TrajOpt ok first try, pregrasp SUCCESS at (1.482, 1.817, 0.842); the wrist camera saw only table top -> the far head estimate of the radio center is >20 cm off. Head view at the start shows … |
| 8 | program_completed | 173 |  | same policy, GENTO_IK_FAIL_DIAG=1: on IK failure re-solve without world collision and report world/self hits and the colliding links. | GENTO_IKFAIL: 39/42 failures 'fail without world collision too'; 3 solvable without world check but then waist_Link_3/head_Link_1/head_Link_2 touch the scene. |
| 7 | program_completed | 171 |  | lower waist_joint_1 to 0.25 before planning (no lean); lift the radio by raising waist_joint_1 +0.15 (arm lift as fallback). | torso lowered to 0.267, ik_here ok, all 8 candidates IK Fail. Offline URDF IK (gento-skye-import/offline_ik.py) for this target (base frame 0.67, 0.12, 0.86): feasible at every lift and every tilt 25-90 (0.0 cm) -> cuRo… |
| 6 | program_completed | 131 |  | lean waist_joint_2 forward by 0.3 rad (move_torso tolerance 0.05) and keep it through the grasp. | lean reached waist_2 -0.28, ik_here ok, but every candidate IK Fail (collision probe flags current config). Offline URDF reach map (gento-skye-import/reach_map.py): shoulder 1.31 m above base_link at lift 0.509; a top-d… |
| 5 | program_completed | 128 |  | accept the pregrasp when the miss is < 5 cm (wrist re-measure refines it). | same base pose as attempt 4 but all 8 candidates IK Fail x3 -> the feasible set at 0.68-0.75 m is tiny (128 IK seeds already). |
| 4 | program_completed | 263 |  | creep only to 0.68; ik_here after the pregrasp execution prints FK_now (cuRobo FK of the live joints) vs sim_now. | at 0.68 no bump (lifted False). tilt 90 primary planned by TrajOpt directly; move_to_pose CONTROLLER_FAILED ~3 cm short -> stage stayed pregrasp_planned. (No GENTO_IKFK FK_now line: the policy's ik_here sits after the m… |
| 3 | program_completed | 130 | lifted | same policy, GENTO_IK_FK_DIAG=1 prints ARM vs DEFAULT FK of every IK solution vs the target. | joint paths collide at every waypoint (start in collision?), radio already lifted 1.7 cm -> creeping to 0.62 m bumps this table. GENTO_IKFK: ARM and DEFAULT FK of every IK solution equal the target -> cuRobo self-consis… |
| 2 | program_completed | 222 | lifted | same policy, harness IK joint mapping fix. | fail. tilt 90 primary IK ok, TrajOpt x2 fail, joint path executed to the IK joints (error 0.0004 rad) but the hand sat 0.20 m above the target; move_to_pose fix TrajOpt Fail. The name-scatter hypothesis was wrong (IK pa… |
| 1 | program_completed | 257 | lifted | seed-36 policy as is (+ strap preference). | radio at (1.48, 1.82), found on sweep step 1 (0.832). Drive to 0.748, creep to ~0.62, bias turn. tilt 90 primary: TrajOpt x2 fail, joint path collides (5/137 waypoints); secondary: joint path ok, executed, but the hand … |

### seed 27（Stage 1）

| # | 结果 | 步数 | 协议 | 这次改了什么（或整条记录） | 结果分析 |
| --- | --- | --- | --- | --- | --- |
| 8 | 未达 10 cm | 710 | held lifted | 6b looks for the handle here, raised 12 cm, then raised +-12 cm forward/left before the refine passes. | **SUCCESS** (task_success, held_and_lifted, 710 steps). Body found from the raised view (0.439); handle look0 'red strap' 0.127 = a red rim edge (top 0.654, 0.125 x 0.037 x 0.046, box at the image edge); pass1 jump 0.12… |
| 7 | program_completed | 318 |  | same, isinstance removed. | body found from the raised view (0.354) and the hand moved over it; wrist view shows the knob/dial face up near the table edge; no handle in frame (nothing above table + 9 cm). |
| 6 | program_error | 267 |  | 6a searches: current view, raised 12 cm, then +-15 cm forward/left offsets raised; moves over the body (twice if it shifts > 4 cm). | program_error NameError 'isinstance' (sandbox has no isinstance) after the search found the body from the raised view (score 0.424, 28634 pts, center (1.78, 0.86), top 0.595, low 0.469). |
| 5 | program_completed | 237 |  | block 6a: at the pregrasp segment 'red radio' in the wrist camera (area-limited), move over the body cloud median (<= 2 passes), then the handle passes. | body pass0 'red radio' 0.119 in the wrist view -> the far center is outside the wrist FOV (attempt 2 EXCL: pregrasp ~0.25 m (world x) beyond the radio). |
| 4 | program_completed | 236 |  | at the table (before lowering) centre the radio in the head view, measure_radio, turn back; take the larger valid of table/near measurements. | table-side head seg 'red radio' only 0.026 -> the radio is below the head view at the table in this seed; wrist at the far center saw a red edge at the image border. |
| 3 | program_completed | 240 |  | handle candidates must top out >= table + 9 cm; weak near measurements (< 1000 pts or < 10 cm tall) ignored. | near failed again; pregrasp at the far center (1.622, 0.962) -> wrist saw only the cord (top 0.482, correctly rejected by the height filter). Far 'measured' and localize_object disagree by 0.54 m in this seed. |
| 2 | program_completed | 705 |  | GENTO_EXCL_DIAG=1 prints the goal/finger points and the 3 nearest object bounds. | GENTO_EXCL: radio_89 AABB lo (3.456, 4.492, 0.423) hi (3.697, 4.768, 0.658) (upright, 0.24 x 0.28 x 0.235). This time near failed, wrist pass0 picked the power cord on the table (top 0.482, table 0.423), descend TIMEOUT… |
| 1 | program_completed | 803 |  | seed-26 policy as is. | radio upright with bail (right arm this time). near measure weak (0.229, 209 pts). Bail found ('black cable' 0.543, top 0.642); both descents PLANNER_FAILED 'IK Fail; ignored objects: none' -> the radio was NOT excluded… |

### seed 28（Stage 1）

| # | 结果 | 步数 | 协议 | 这次改了什么（或整条记录） | 结果分析 |
| --- | --- | --- | --- | --- | --- |
| 14 | program_completed | 307 |  | redrive to 0.85 m instead of 0.68 m (attempts 2-3 planned from 1.07 m). | redrive 0.85 m: tilt 90 primary IK ok, TrajOpt x2 fail; rest IK Fail. |
| 13 | program_completed | 313 |  | same policy with head joints locked. | head locked; still self-collision-limited IK (Link2_L <-> waist_Link_3; 11 of the no-self-collision solutions had no overlap at all -> the solver, not the model, misses them). cuRobo's self_collision_buffer is r1 + r2 +… |
| 12 | program_completed | 315 |  | same policy (no joint-path fallback), buffers Link2 -0.008 / waist_Link_3 -0.01. | tilt 90 primary IK ok but TrajOpt x2 fail; IK fails: 'solvable only without self collision' with Link2_L/R <-> waist_Link_3 touching and head_Link_2 <-> waist_Link_3 3.4 cm. The head joints (Head_J1/J2) were free in the… |
| 11 | program_completed | 408 |  | diagnostic run, GENTO_IK_FAIL_DIAG=1. | GENTO_IKFAIL again 'solvable only without self collision', Link2_R <-> waist_Link_3 touching. Found: CAP_HARNESS_BEHAVIOR_LOCK_TORSO defaults to 0 -> cuRobo IK/plans move the torso too; solve_ik returns arm joints only,… |
| 10 | program_completed | 322 |  | post-redrive measurement accepted only within 0.12 m of the near centre and score >= 0.3. | redrive 0.726 m, side 0.185, ik_here ok; post-redrive measure None; all shadow + fallback IK Fail. |
| 9 | program_completed | 432 |  | same policy with the buffer; GENTO_IK_FAIL_DIAG=1. | post-redrive head measure jumped to another object ((-2.157, -1.937), 0.231, 0.13 tall, by the sofa): pregrasp joint path missed by 0.637 m and the fix was IK Fail (ignored: sofa). No IK diagnostics fired (tilt 90 prima… |
| 8 | program_completed | 407 |  | diagnostic: ik_here after the redrive + GENTO_IK_FAIL_DIAG=1. | ik_here after redrive True; GENTO_IKFAIL: failures are 'solvable only without self collision', the blocking pair Link2_R/L <-> waist_Link_3 at 0.0 cm (touching). Fit report: waist_Link_3 mesh 28.6 x 28 x 46.4 cm, sphere… |
| 7 | program_completed | 317 |  | fallback when nothing plans: lift 0.38 / 0.25 with 8 cm clearance / 0.15 with 8 cm, straight down, both arms. | redrive ok (0.718 m); every fallback (lift 0.38/0.25/0.15, clear 0.14/0.08, both arms) IK Fail. |
| 6 | program_completed | 256 |  | if the near shift > 0.15 m, servo to 0.68 m from the new centre (radio 15 deg left) and re-measure. | redrive ok (1.084 -> 0.728 m), head saw nothing after it (near after redrive None); then all 8 pregrasp candidates IK Fail x3. |
| 5 | program_completed | 788 |  | edge boxes x0.7; candidates tried best-first (<= 3), each with one refine pass; a failed recenter moves on to the next candidate. | candidates tried best-first, all three recenters IK Fail (off 0.12-0.28). Cause: the near re-measure moved the radio 0.46 m away from the far estimate -> radio ~1.07 m from the base, its bail ends 0.86-1.06 m: out of re… |
| 4 | program_completed | 805 |  | handle looks over both ends of the radio's long axis (centre +- L/2, raised 12 cm) before the fwd/left offsets. | (attempt 4, with span penalty + all looks): look1 at one end found a bail-like thin part (0.264 x 0.04 x 0.031, not edge, eff 0.531); look3 (edge, 0.551) won and its recenter was IK Fail (off 0.123); looks 5-7 identical… |
| 3 | program_completed | 1012 |  | handle candidates must be thin: middle extent <= 0.16 m. | chosen 'red strap' 0.201 was the red band on the radio top (part of the body; mid extent 0.094), descents PLANNER_FAILED. Wrist view: upright radio, bail at one end of the long axis (bottom edge of the frame); 'handle' … |
| 2 | program_completed | 1049 |  | search gate 0.35 and the hit's cloud median must be at 0.25-1.0 m with >= 150 points, else keep turning. | search fixed (step 5, 0.852, verdict ok); near 0.414 (shift 0.46); pregrasp SUCCESS (secondary arm); handle look0 was the whole body (0.343 x 0.125 x 0.224, 49k pts) -> both descents PLANNER_FAILED, empty. |
| 1 | program_completed | 2709 |  | seed-27 policy as is. | false target. Sweep step 4 scored 0.248 on a pink object outside a window (gate 0.2) -> 'measured' (-0.555, 1.241, z 1.31), extents 0.07 x 0.066 x 0; drove there, pregrasp TIMEOUT at z 1.46. The radio was not seen in st… |

### seed 29（Stage 1）

| # | 结果 | 步数 | 协议 | 这次改了什么（或整条记录） | 结果分析 |
| --- | --- | --- | --- | --- | --- |
| 1 | 未达 10 cm | 874 | held lifted | as is. | **SUCCESS** first try (task_success, held_and_lifted, 874 steps). Handle looks: 8 evaluated, best look7 eff 0.594 (thin 0.251 x 0.048 x 0.019 at (1.475, 0.268), top 0.648); recenter ok; refine kept; gripper 0.291 after … |

### seed 30（Stage 1）

| # | 结果 | 步数 | 协议 | 这次改了什么（或整条记录） | 结果分析 |
| --- | --- | --- | --- | --- | --- |
| 28 | no outcome |  |  | after the low slide fails, servo the base 8 cm toward the body median so the lowered hand moves with it, then close. |  |
| 27 | program_completed | 1029 |  | smaller waist steps (5 cm, up to 5) and, once the shoulder is down, slide to the body median at that height. | EEF lowered 0.901 → 0.890, 0.840, 0.786, 0.740, 0.706. Slide at that height failed; hand stayed at (1.418, 0.621). Gripper closed to 0.358 but held stayed false and the radio did not rise. After the torso lift the gripp… |
| 26 | program_completed | 1116 |  | keep the hand's current orientation and lower it by dropping waist_joint_1 in 8 cm steps, then close. | waist drops reached EEF 0.903, 0.851, 0.771, then 0.705 (the last move_torso returned false). Fingers were still about 2.5 cm above the radio top. Gripper closed to 0.11, but the radio was not attached and clearance sta… |
| 25 | program_completed | 777 |  | pinch-here keeps the current height and lets the 4 cm descend steps do the lowering. | pregrasp SUCCESS. Body slide TrajOpt Fail. Pinch target EEF z 0.658, hand stayed at 0.903. The first 4 cm arm step (to 0.863) failed to plan, twice, including a 90 deg regrasp. Gripper closed empty. Torso lift then rais… |
| 24 | program_completed | 626 |  | if the handle is already within 6 cm, stay at the current height and descend in 4 cm steps. Stop looking once that near handle is found. | body slide failed, so pinch-here ran at the current hand (1.42, 0.614), top 0.556. It still tried to drop 12 cm in one plan and that IK failed (off 0.124), so the stepped descend never started. |
| 23 | program_completed | 763 |  | a part must rise 12 cm off the table, and a part more than 15 cm from the hand has its score cut to 0.3. | cord rejected (22 cm away, too low). Chose the thin handle 3.9 cm from the hand, top 0.557. Lowering 12 cm to the pregrasp failed IK (off 0.127). Eight identical looks, the offsets never moved the camera. |
| 22 | program_completed | 732 |  | same policy, 8 IK tries on each candidate before giving up. | pregrasp SUCCESS and the slide onto the body succeeded. Table height stayed ~0.38, so straps counted as above the table. The winner was a black cable at (1.168, 0.442), 30 cm from the hand (the cord); recenter IK failed… |
| 21 | program_completed | 255 |  | keep the far table height unless the wrist cloud's low is actually at or below it. The red strap at 0.50 can then pass. | did not reach the wrist. tilt 90 IK failed all 3 tries on both arms, so the table-height change was not exercised. |
| 20 | program_completed | 539 |  | wrist body gate 0.10. Same pinch-in-place fallback. | pregrasp SUCCESS. Wrist body 0.369, 70k points, median (1.329, 0.601), and the slide there succeeded. The cloud low 0.512 replaced the table estimate (real table ~0.38), so every strap at z 0.48-0.56 was rejected as not… |
| 19 | program_completed | 305 |  | accept a wrist body at score 0.15. If the slide to its median fails, pinch at the current hand position. | cartesian plans failed twice, joint path succeeded, pregrasp SUCCESS at (1.421, 0.616, 0.886). Wrist "red radio" scored 0.149 on every view, still under 0.15, so the body was rejected again. The offsets did not move the… |
| 18 | program_completed | 636 |  | after two failed cartesian plans, plan a straight joint path to the IK solution. Torso stays locked, so those arm joints are the pose. | cartesian plan succeeded on the first try (joint path not used). Pregrasp SUCCESS, EEF (1.417, 0.625, 0.898). Wrist "red radio" scored 0.192, just under the 0.2 gate, so the body was rejected and the hand never pinched.… |
| 17 | program_completed | 256 |  | same policy, another draw of the motion planner. | same as 16. tilt 90 primary IK ok, five cartesian plans all IK Fail. Pinch-in-place did not run. |
| 16 | program_completed | 257 |  | if the slide onto the body median fails but the wrist already sees a large radio cloud, lower and pinch at the hand's current xy. | five motion plans all failed (planner IK Fail). The pinch-in-place path did not run. Same pose planned on the first try in attempts 13 and 15. |
| 15 | program_completed | 372 |  | up to 5 motion-plan tries on that tilt-90 pose, torso still locked. Wrist moves stay on plan-then-execute. | plan and pregrasp SUCCESS again, EEF (1.42, 0.612, 0.902). Wrist is directly over the red radio (black center strap, red side handle). Body median is 9 cm off and that slide does not plan. The best handle score (0.828) … |
| 14 | program_completed | 259 |  | wrist shifts (over the body, handle looks, recenter) use plan_motion + execute_trajectory. Torso still locked. | the same tilt-90 pose did not plan this time (2 tries, planner IK Fail). Did not reach the wrist. The plan is stochastic. |
| 13 | program_completed | 619 |  | execute the trajectory plan_motion already returned, still with the torso locked. | pregrasp SUCCESS, EEF (1.417, 0.624, 0.898). Wrist body score 0.21, 70k points, median (1.330, 0.594), top 0.549. move_to_pose onto that median failed, so every handle look stayed on the same frame and picked an edge bo… |
| 12 | program_completed | 115 |  | same policy, CAP_HARNESS_BEHAVIOR_LOCK_TORSO=1 so the motion planner cannot move the waist while searching for a path. | tilt 90 primary IK and plan_motion both ok on the first try. move_to_pose then replanned and failed (IK Fail); the hand stayed at (1.231, 0.672, 0.745). |
| 11 | program_completed | 262 |  | if the center pose will not plan, try the same top-down hand 10 cm and 16 cm back toward the base, 10 cm above the top. | both arms got tilt-90 IK (primary TrajOpt Fail, secondary planner IK Fail). The 10 cm and 16 cm pull-back lost IK. Retreat to 0.97 m, head still blind, both retreat IKs failed. |
| 10 | program_completed | 258 |  | same pose, up to 4 TrajOpt tries when tilt-90 IK succeeds. | again at 0.76 m, tilt 90 primary IK ok, all 4 motion plans failed (planner-internal IK Fail). Retreat did not produce a plan. |
| 9 | program_completed | 252 |  | creep only to 0.85 m (seed 28 tilt-90 IK worked from there). If a retreat still kills ik_here, step back toward the radio in 5 cm steps until the live pose is … | drive stopped at 0.779 m (no creep). tilt 90 primary IK ok first try, TrajOpt failed twice; every other tilt/arm IK Fail. Did not reach the retreat. |
| 8 | program_completed | 249 |  | after the retreat, centre the radio in the head and re-measure before planning. | crept to 0.68 m with ik_here true the whole way; head score at the table 0.014 (radio is on the glass table, below the level head). All 8 shadow, 6 fallback and 6 view-pose IKs failed. Retreat to 0.882 m saw nothing. Af… |
| 7 | program_completed | 249 |  | if nothing plans, servo the base 0.25 m back from the radio and retry a straight-down pregrasp. retreat to 0.885 m, both arms still IK Fail on the far centre. … |  |
| 6 | infra (died_before_logs) |  |  | diag includes zero-radius spheres, lists their links, and the cost of the current configuration. | GENTO_ARMSELF: ARM-embodiment self-collision cost of the no-self-collision solutions is 2-125 while no positive-radius sphere pair overlaps in most. diag includes zero-radius spheres, their links, and the cost of the cu… |
| 6 | program_completed | 222 |  | diag includes zero-radius spheres, lists their links, and the cost of the current configuration. | GENTO_ARMSELF: ARM-embodiment self-collision cost of the no-self-collision solutions is 2-125 while no positive-radius sphere pair overlaps in most. diag includes zero-radius spheres, their links, and the cost of the cu… |
| 5 | program_completed | 225 |  | rerun the diag. | GENTO_ARMSELF crashed (wrong attribute); fixed to mg.kinematics.get_self_collision_config(). rerun the diag. Result: GENTO_ARMSELF: ARM-embodiment self-collision cost of the no-self-collision solutions is 2-125 (positiv… |
| 4 | program_completed | 224 |  | diag also evaluates the solution with the ARM embodiment's own spheres/offsets/collision matrix (GENTO_ARMSELF). | GENTO_ARMSELF crashed (wrong attribute); fixed to mg.kinematics.get_self_collision_config(). |
| 3 | program_completed | 224 |  | diagnostic, GENTO_IK_FAIL_DIAG=1. | GENTO_IKFAIL: 60/60 failures solvable only without self collision; 17 of those solutions have no intersecting pair by the DEFAULT-embodiment sphere check -> the ARM embodiment's self-collision must see something else. |
| 2 | program_completed | 226 |  | block 5c: if nothing plans, move the hand to a reachable top-down view pose 0.45/0.55/0.35 m toward the far centre, 0.30 m above its top, and let the wrist bod… | view poses 0.35-0.55 m ahead, z = top + 0.30, top-down: IK Fail for both arms -> something global blocks IK in this seed. |
| 1 | program_completed | 201 |  | as is. | radio seen at reset (no sweep); far centre (1.42, 0.61), crept to 0.68 m; table/near head measures saw nothing (0.015); all shadow + fallback IK Fail around the far centre. |
