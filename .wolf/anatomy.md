# anatomy.md

> Auto-maintained by OpenWolf. Last scanned: 2026-06-05T08:40:55.190Z
> Files: 580 tracked | Anatomy hits: 0 | Misses: 0

## ../home/vscode/.claude/plans/

- `assess-the-feasibility-of-dreamy-clover.md` — Live scope for the NPS sim → PlotJuggler on the Mac host (~1309 tok)

## ./

- `.claudeignore` — Secrets and credentials (~56 tok)
- `.DS_Store` (~2186 tok)
- `.gitignore` — Git ignore rules (~60 tok)
- `.gitmodules` (~52 tok)
- `build_active.sh` — Build whatever C/C++ configuration is currently selected in VS Code. (~588 tok)
- `build_fw.sh` — Paparazzi firmware build script (~474 tok)
- `CLAUDE.md` — OpenWolf (~2076 tok)
- `Dockerfile.paparazzi` — ── Layer 5: Paparazzi UAV Toolchains ───────────────────────────────────────── (~636 tok)
- `gen_build_log.sh` — Capture a verbose build log for Makefile Tools buildLog IntelliSense. (~602 tok)
- `gen_compile_db.sh` — Generate a per-config compile_commands.json for VS Code IntelliSense. (~623 tok)
- `gen_vscode.sh` — Codegen prebuild step for VS Code / Makefile Tools. (~221 tok)
- `Knowledge/09 - FlightGear 3D Visualization.md` — Setup guide, all gotchas, and verification steps for FG viz with NPS. (~800 tok)
- `README.md` — Project documentation (~1275 tok)
- `sim_anton.py` — Launch ANTON NPS sim + Ivy telemetry monitor. Flags: --mfc --gdb --fg (FlightGear 3D viz via host.docker.internal:5501). (~5414 tok)

## .claude/

- `settings.json` (~565 tok)

## .claude/hooks/

- `check-gitignore.sh` — PreToolUse hook (Edit|Write|MultiEdit) — token-discipline harness. (~461 tok)
- `format-code.sh` — PostToolUse hook (Edit|Write|MultiEdit) — token-discipline harness. (~305 tok)
- `openwolf-refresh.sh` — SessionStart hook — token-discipline harness. (~289 tok)
- `prompt-retro.sh` — Stop hook — token-discipline harness. (~161 tok)

## .claude/rules/

- `openwolf.md` (~313 tok)

## .devcontainer/

- `devcontainer.json` (~1122 tok)
- `docker-compose.override.yml` — To regenerate: re-run `closs init` (picks up changes to ~/.config/claudosseum/config). (~675 tok)
- `docker-compose.yml` — Docker Compose services (~1143 tok)
- `Dockerfile` — Docker container definition (~1170 tok)
- `Dockerfile.local` — ── Layer 5: Paparazzi UAV Toolchains ───────────────────────────────────────── (~636 tok)
- `init-firewall.sh` (~1634 tok)

## .devcontainer/scripts/

- `freeze.sh` — ── freeze.sh ───────────────────────────────────────────────────────────────── (~1667 tok)

## .devcontainer/snapshots/freeze_20260601_105359/

- `apt-packages-manual.txt` (~447 tok)
- `apt-packages.txt` (~6189 tok)
- `HOW-TO-RESTORE.md` — How to restore from this snapshot (~218 tok)
- `npm-global.json` (~84 tok)
- `npm-global.txt` (~21 tok)
- `requirements-freeze.txt` (~143 tok)
- `versions.txt` (~149 tok)

## .devcontainer/snapshots/freeze_20260601_105359/claude-settings/

- `settings.json` (~7 tok)

## .devcontainer/snapshots/freeze_20260601_105359/dotfiles/

- `.bashrc` — ~/.bashrc: executed by bash(1) for non-login shells. (~1568 tok)
- `.profile` — ~/.profile: executed by the command interpreter for login shells. (~216 tok)
- `.zsh_history` (~5880 tok)
- `.zshrc` — If you come from bash you might have to change your $PATH. (~1072 tok)

## .obsidian/

- `app.json` (~1 tok)
- `appearance.json` (~1 tok)
- `core-plugins.json` (~199 tok)
- `workspace.json` (~1622 tok)

## Knowledge/

- `00 - Index.md` — Paparazzi Control System — Knowledge Base (~315 tok)
- `09 - FlightGear 3D Visualization.md` — FlightGear 3D Visualization for NPS (~1076 tok)

## enac_paparazzi/

- `.DS_Store` (~2186 tok)
- `.gdbinit` (~23 tok)
- `.gitignore` — Git ignore rules (~1096 tok)
- `.gitmodules` (~421 tok)
- `.packages` (~131 tok)
- `.travis.yml` — Declares gcc (~517 tok)
- `BUGS` (~31 tok)
- `CHANGELOG.md` — Change log (~21715 tok)
- `CLAUDE.md` — [Satellite name] (~485 tok)
- `CONTRIBUTING.md` — How to contribute (~613 tok)
- `create_module` — load "unix.cma";; (~2809 tok)
- `Doxyfile` — Doxyfile 1.8.6 (~27182 tok)
- `find_confs.py` — URL configuration (~330 tok)
- `fix_code_style.sh` (~380 tok)
- `githelper.sh` (~2178 tok)
- `LICENSE` — Project license (~4798 tok)
- `make-release-tarball.sh` — File:        git-archive-all.sh (~2372 tok)
- `Makefile` — Make build targets (~2893 tok)
- `Makefile.ac` — Hey Emacs, this is a -*- makefile -*- (~2980 tok)
- `Makefile.lpctools` — Hey Emacs, this is a -*- makefile -*- (~301 tok)
- `paparazzi_code_profile_eclipse.xml` (~5036 tok)
- `paparazzi_pkgman.py` — This file is part of Paparazzi. (~1736 tok)
- `paparazzi_version` (~169 tok)
- `paparazzi.sublime-project` (~216 tok)
- `pprz_src_test.sh` (~49 tok)
- `README.md` — Project documentation (~823 tok)
- `start.py` — URL configuration (~4886 tok)
- `Vagrantfile` — -*- mode: ruby -*- (~306 tok)

## enac_paparazzi/conf/

- `abi.dtd` — Declares name (~104 tok)
- `abi.xml` — Declares name (~2343 tok)
- `conf_example.xml` (~2828 tok)
- `conf_tests_coverity.xml` (~5496 tok)
- `conf_tests.xml` (~5884 tok)
- `control_panel_example.xml` (~1958 tok)
- `flash_modes.xml` (~1055 tok)
- `Makefile.ardrone2` — Hey Emacs, this is a -*- makefile -*- (~322 tok)
- `Makefile.arm-embedded-common` — Hey Emacs, this is a -*- makefile -*- (~392 tok)
- `Makefile.arm-embedded-toolchain` — Hey Emacs, this is a -*- makefile -*- (~324 tok)
- `Makefile.arm-linux` — Hey Emacs, this is a -*- makefile -*- (~427 tok)
- `Makefile.arm-linux-toolchain` — Hey Emacs, this is a -*- makefile -*- (~375 tok)
- `Makefile.bbb` — Hey Emacs, this is a -*- makefile -*- (~326 tok)
- `Makefile.bebop` — Hey Emacs, this is a -*- makefile -*- (~318 tok)
- `Makefile.chibios` — This file is part of paparazzi. (~2804 tok)
- `Makefile.disco` — Hey Emacs, this is a -*- makefile -*- (~325 tok)
- `Makefile.hitl` (~39 tok)
- `Makefile.linux` — Hey Emacs, this is a -*- makefile -*- (~1034 tok)
- `Makefile.local` — Hey Emacs, this is a -*- makefile -*- (~67 tok)
- `Makefile.lpc21` — Hey Emacs, this is a -*- makefile -*- (~2116 tok)
- `Makefile.nps` — Hey Emacs, this is a -*- makefile -*- (~875 tok)
- `Makefile.sim` — Hey Emacs, this is a -*- makefile -*- (~1100 tok)
- `Makefile.stm32` — Hey Emacs, this is a -*- makefile -*- (~1786 tok)
- `Makefile.stm32-upload` — Hey Emacs, this is a -*- makefile -*- (~1604 tok)
- `Makefile.swing` — Hey Emacs, this is a -*- makefile -*- (~351 tok)
- `mapGL_help_keys.txt` (~77 tok)
- `maps_example.xml` (~19 tok)
- `maps.dtd` (~33 tok)
- `messages.xml` — Declares name (~43090 tok)
- `mtk.dtd` — Declares name (~135 tok)
- `mtk.xml` — Declares name (~383 tok)
- `ubx.dtd` — Declares name (~136 tok)
- `ubx.xml` — Declares name (~4475 tok)
- `xsens_MTi-G.xml` (~3428 tok)
- `xsens.dtd` (~173 tok)

## enac_paparazzi/conf/airframes/

- `airframe.dtd` — Declares CDATA (~819 tok)
- `README.txt` (~143 tok)
- `test_settings.xml` (~85 tok)

## enac_paparazzi/conf/airframes/AGGIEAIR/

- `aggieair_ark_hexa_1-8.xml` (~3252 tok)
- `aggieair_ark_quad_lisa_mx.xml` (~2689 tok)
- `aggieair_atomic_lia.xml` (~3674 tok)
- `aggieair_blujay_goose.xml` (~3497 tok)
- `aggieair_blujay.xml` (~3458 tok)
- `aggieair_conf.xml` (~1916 tok)
- `aggieair_control_panel.xml` (~5365 tok)
- `aggieair_ecu_example.xml` (~2869 tok)
- `aggieair_el_captitan_lia.xml` (~3653 tok)
- `aggieair_iris_indi.xml` (~4298 tok)
- `aggieair_minion_rp3_lia.xml` (~3526 tok)
- `aggieair_minionsim_lia.xml` (~3654 tok)
- `aggieair_minty_lia.xml` (~3653 tok)
- `El_Captain.xml` (~3966 tok)

## enac_paparazzi/conf/airframes/BR/

- `asctec_br.xml` (~2116 tok)
- `bebop_default.xml` (~2155 tok)
- `bebop_indi_frog_flip.xml` (~2472 tok)
- `bebop_indi_frog.xml` (~2502 tok)
- `bebop_indi.xml` (~2506 tok)
- `conf.xml` (~2007 tok)
- `DelFlyDualPWMservo.xml` — Declares for (~2483 tok)
- `DreamCacher_bart.xml` — Declares for (~2386 tok)
- `ladybird_kit_bart_bluegiga_optitrack.xml` — Declares for (~2438 tok)
- `ladybird_kit_bart_bluegiga.xml` — Declares for (~2394 tok)
- `ladybird_kit_bart.xml` — Declares for (~2431 tok)
- `ladybird_kit_indi_bart.xml` — Declares for (~2565 tok)
- `mavtec4_br.xml` (~2114 tok)
- `quadshot_asp21_FutabaPPMonUart1.xml` (~4754 tok)

## enac_paparazzi/conf/airframes/CDW/

- `cdw_asctec.xml` (~1979 tok)
- `cdw_bebop.xml` (~2472 tok)
- `cdw_conf.xml` (~332 tok)
- `cdw_mavtec.xml` (~2217 tok)
- `cdw_tricopter.xml` (~2029 tok)

## enac_paparazzi/conf/airframes/CRIDEA/

- `cridea_quadsuave.xml` (~2232 tok)

## enac_paparazzi/conf/airframes/ENAC/

- `conf_enac.xml` (~1223 tok)
- `cyfoam.xml` — Declares of (~3618 tok)
- `darko_mfc_voliere.xml` (~3267 tok)
- `hoops_gen_ap.xml` (~3208 tok)
- `rover.xml` (~1217 tok)

## enac_paparazzi/conf/airframes/ENAC/fixed-wing/

- `apogee.xml` (~2306 tok)
- `chimera.xml` (~2428 tok)
- `crrcsim.xml` (~2486 tok)
- `disco.xml` (~2623 tok)
- `eternity1.xml` (~2054 tok)
- `firestorm.xml` (~2410 tok)
- `jp.xml` — Declares radio_control (~3172 tok)
- `twinjet2.xml` (~2594 tok)
- `weasel.xml` (~2486 tok)
- `zagi_mekf_wind.xml` (~2638 tok)
- `zagie_MFC.xml` (~2366 tok)

## enac_paparazzi/conf/airframes/ENAC/quadrotor/

- `ard2_101.xml` (~392 tok)
- `ard2_103.xml` (~402 tok)
- `ard2_104.xml` (~294 tok)
- `ard2_base_control.xml` (~1606 tok)
- `ard2_base_digit.xml` (~836 tok)
- `ard2_base_vision.xml` (~868 tok)
- `ard2_basic_adhoc.xml` (~774 tok)
- `ard2_basic.xml` (~757 tok)
- `booz2_g1.xml` (~2602 tok)
- `x300_optitrack.xml` (~2599 tok)

## enac_paparazzi/conf/airframes/ESDEN/

- `esden_cocto_lm2a2.xml` (~2206 tok)
- `esden_conf.xml` (~2554 tok)
- `esden_gain_scheduling_example.xml` (~2420 tok)
- `esden_hexy_ll11a2pwm.xml` (~2359 tok)
- `esden_hexy_lm2a2pwm.xml` (~2004 tok)
- `esden_lisa2_hex.xml` (~2192 tok)
- `esden_qs_asp22.xml` (~2494 tok)
- `esden_quady_ll11a2pwm.xml` (~2247 tok)
- `esden_quady_lm1a1pwm.xml` (~1867 tok)
- `esden_quady_lm2a2pwm.xml` (~1948 tok)
- `esden_quady_lm2a2pwmppm.xml` (~1871 tok)
- `esden_quady_ls10pwm.xml` (~1917 tok)

## enac_paparazzi/conf/airframes/ESDEN/calib/

- `esden_asp21-000.xml` (~407 tok)
- `esden_asp21-001.xml` (~427 tok)
- `esden_asp21-018.xml` (~414 tok)
- `esden_asp21-026.xml` (~415 tok)
- `esden_asp21-default.xml` (~461 tok)
- `esden_asp22-019.xml` (~220 tok)
- `esden_aspirin_012.xml` (~388 tok)
- `esden_aspirin_jtm.xml` (~350 tok)
- `esden_ls10-default.xml` (~406 tok)

## enac_paparazzi/conf/airframes/FLIXR/

- `flixr_conf.xml` (~1771 tok)
- `flixr_discovery.xml` (~4202 tok)
- `flixr_fraser_lisa_m_rotorcraft.xml` (~2320 tok)
- `flixr_ladybird_lisa_s.xml` — Declares for (~2458 tok)
- `flixr_lisa_mx.xml` (~2695 tok)
- `flixr_lisamx_dual.xml` (~2644 tok)
- `flixr_zmr250_elle0.xml` (~2042 tok)

## enac_paparazzi/conf/airframes/HOOPERFLY/

- `hooperfly_conf.xml` (~2676 tok)
- `hooperfly_control_panel.xml` (~5145 tok)
- `hooperfly_dot_bashrc` — HooperFly Paprarazzi Section (~974 tok)
- `hooperfly_enac_hexa_elle0_v1_2.xml` (~2759 tok)
- `hooperfly_racerpex_hexa_lisa_mx_20.xml` (~2798 tok)
- `hooperfly_racerpex_octo_lisa_mx_20.xml` (~2658 tok)
- `hooperfly_racerpex_quad_elle0_v1_2.xml` (~2648 tok)
- `hooperfly_racerpex_quad_lisa_mx_20.xml` (~2692 tok)
- `hooperfly_slurp.sh` — Declares f (~58 tok)
- `hooperfly_teensyfly_hexa_lisa_mx_20.xml` (~2799 tok)
- `hooperfly_teensyfly_quad_elle0_v1_2_212.xml` (~366 tok)
- `hooperfly_teensyfly_quad_elle0_v1_2_213.xml` (~359 tok)
- `hooperfly_teensyfly_quad_elle0_v1_2_214.xml` (~368 tok)
- `hooperfly_teensyfly_quad_elle0_v1_2_215.xml` (~366 tok)
- `hooperfly_teensyfly_quad_elle0_v1_2_216.xml` (~367 tok)
- `hooperfly_teensyfly_quad_elle0_v1_2.xml` (~2331 tok)
- `hooperfly_teensyfly_quad_elle0.xml` (~2587 tok)
- `hooperfly_teensyfly_quad_lisa_mx_20.xml` (~2693 tok)

## enac_paparazzi/conf/airframes/KS/

- `kirk_conf.xml` (~310 tok)
- `ks_bebop2_stereo.xml` (~2595 tok)
- `ks_mavtec1.xml` (~2584 tok)

## enac_paparazzi/conf/airframes/MM/

- `bebop.xml` (~2319 tok)
- `bebop2_lum1_xbee.xml` (~2794 tok)

## enac_paparazzi/conf/airframes/MTO/

- `test_enac_ptu.xml` (~2208 tok)

## enac_paparazzi/conf/airframes/OPENUAS/

- `openuas_ardrone2.xml` (~2806 tok)
- `openuas_eflite_t28.xml` — Declares radio_control (~11298 tok)
- `openuas_eflite_umx_sbach_342.xml` — Declares radio_control (~8385 tok)
- `openuas_itsybitsy.xml` — Declares for (~2772 tok)
- `openuas_leapfrogeye.xml` (~2604 tok)
- `openuas_mentor.xml` — Declares radio_control (~2930 tok)
- `openuas_moksha.xml` — Declares we (~7860 tok)
- `openuas_parrot_disco.xml` — Declares Comments (~13564 tok)
- `openuas_psi.xml` (~2671 tok)
- `openuas_taxiiii.xml` (~4896 tok)
- `openuas_vivify.xml` (~8687 tok)

## enac_paparazzi/conf/airframes/PPZUAV/fixed-wing/

- `ppzimu_tiny.xml` (~2516 tok)

## enac_paparazzi/conf/airframes/PPZUAV/imu-calibrations/

- `ppzuav_booz2imu_001.xml` (~514 tok)
- `ppzuav_booz2imu_003.xml` (~517 tok)
- `ppzuav_booz2imu_004.xml` (~505 tok)
- `ppzuav_booz2imu_005.xml` (~519 tok)

## enac_paparazzi/conf/airframes/examples/

- `ardrone2_gazebo.xml` (~2351 tok)
- `ardrone2_opticflow_hover.xml` (~2215 tok)
- `ardrone2.xml` (~2168 tok)
- `bebop.xml` (~2300 tok)
- `bebop2_indi.xml` (~2401 tok)
- `bebop2_opticflow.xml` (~2749 tok)
- `bebop2_ukf_magnetometer_calibration.xml` (~2512 tok)
- `bixler_lisa_m_2.xml` (~2031 tok)
- `booz2.xml` (~2015 tok)
- `bumblebee_quad.xml` (~2686 tok)
- `easystar_ets.xml` (~2161 tok)
- `h_hex.xml` (~1924 tok)
- `krooz_sd_quad_mkk.xml` (~2499 tok)
- `ladybird_lisa_s_bluegiga.xml` — Declares for (~2586 tok)
- `ladybird_lisa_s.xml` — Declares for (~2621 tok)
- `lisa_l_chimu.xml` (~1590 tok)
- `logomatic.xml` (~182 tok)
- `MentorEnergy.xml` (~2920 tok)
- `microjet_imu_xsens.xml` (~2182 tok)
- `microjet_lisa_m_xsens.xml` (~1696 tok)
- `microjet_lisa_m.xml` (~2207 tok)
- `microjet_twog_aspirin.xml` (~2051 tok)
- `microjet.xml` (~2356 tok)
- `quadrotor_elle0.xml` (~2172 tok)
- `quadrotor_hbmini.xml` (~2321 tok)
- `quadrotor_lisa_m_2_pwm_spektrum.xml` (~2144 tok)
- `quadrotor_lisa_mx_mavlink.xml` (~2499 tok)
- `quadrotor_lisa_mx.xml` (~2358 tok)
- `quadrotor_lisa_s.xml` (~2300 tok)
- `quadrotor_navgo.xml` (~2694 tok)
- `quadrotor_navstik.xml` (~2550 tok)
- `quadrotor_revo.xml` (~2006 tok)
- `quadshot_178_pylons.xml` (~4454 tok)
- `quadshot_asp21_FutabaPPMonUart1.xml` (~4754 tok)
- `quadshot_asp21_spektrum.xml` (~4684 tok)
- `separate_fbw_ap.xml` (~3143 tok)
- `setup_apogee.xml` (~780 tok)
- `setup_elle0.xml` (~644 tok)
- `setup_lisam2.xml` (~734 tok)
- `swing.xml` (~2290 tok)
- `twinjet.xml` (~1982 tok)
- `Twinstar_energyadaptive.xml` (~3117 tok)
- `twog_analogimu.xml` (~2362 tok)
- `umarim_lite_v2.xml` (~1891 tok)
- `yapaChimuSpi.xml` (~1957 tok)

## enac_paparazzi/conf/airframes/max/

- `microjet_lisa_m.xml` (~2437 tok)

## enac_paparazzi/conf/airframes/testhardware/

- `LisaL_v1.1_aspirin_v1.5_fw.xml` (~2290 tok)
- `LisaL_v1.1_aspirin_v1.5_rc.xml` (~2361 tok)
- `LisaL_v1.1_b2_v1.2_fw.xml` (~2268 tok)
- `LisaL_v1.1_b2_v1.2_rc.xml` (~2603 tok)

## enac_paparazzi/conf/airframes/tudelft/

- `aa_quadplane.xml` (~2527 tok)
- `ardrone2_flip.xml` (~2280 tok)
- `ardrone2_indi.xml` (~2232 tok)
- `ardrone2_OF_hover.xml` (~2736 tok)
- `ardrone2_opticflow_indi.xml` (~2405 tok)
- `ardrone2_opticflow_stereo.xml` (~2188 tok)
- `ardrone2_opticflow.xml` (~2274 tok)
- `ardrone2_optitrack.xml` (~2266 tok)
- `bebop_autonomous_race_2017.xml` (~2229 tok)
- `bebop_autonomous_race_2018.xml` (~2357 tok)
- `bebop_course2018_orangeavoid.xml` (~3038 tok)
- `bebop_flip.xml` (~2473 tok)
- `bebop_frontcam.xml` (~2173 tok)
- `bebop_indi_actuators.xml` (~2060 tok)
- `bebop_indi.xml` (~2267 tok)
- `bebop_mavlink.xml` (~2561 tok)
- `bebop_OF_hover.xml` (~2600 tok)
- `bebop_opticflow.xml` (~2409 tok)
- `bebop_optitrack.xml` (~2277 tok)
- `bebop2_detect_gate_front.xml` (~2970 tok)
- `bebop2_indi_MAVlink.xml` (~2314 tok)
- `bebop2_indi.xml` (~2013 tok)
- `bebop2_no_damping.xml` (~2271 tok)
- `bebop2_opticflow.xml` (~2930 tok)
- `bebop2_optitrack_visionfront.xml` (~2960 tok)
- `bebop2_undistort_front.xml` (~2796 tok)
- `bebop2_vision.xml` (~2359 tok)
- `bs_helidd_indi.xml` (~2523 tok)
- `bs_helidd_pid.xml` (~2340 tok)
- `cx10.xml` (~2300 tok)
- `delfly_lisas.xml` — Declares telemetry (~2842 tok)
- `fan_demo.xml` (~2295 tok)
- `guido_ardrone2_optitrack.xml` (~2331 tok)
- `guido_conf.xml` (~830 tok)
- `guido_control_panel.xml` (~2767 tok)
- `heli450.xml` (~2191 tok)
- `heliGeniusDD.xml` (~2348 tok)
- `iris_indi.xml` (~3424 tok)
- `ladybird_lisa_mxs.xml` (~2352 tok)
- `ladybird_lisamxs_wifi_indi_stereoboard.xml` (~3030 tok)
- `ladylisa_bluegiga_stereoboard.xml` — Declares for (~2564 tok)
- `logo600.xml` (~3621 tok)
- `mavshot.xml` (~3498 tok)
- `mavtec1.xml` (~2582 tok)
- `mavtec4.xml` (~2214 tok)
- `mavtec5.xml` (~2548 tok)
- `origami_lisamxs_wifi_indi_stereoboard.xml` (~3516 tok)
- `outback.xml` (~3972 tok)
- `quadshot_pylons.xml` (~3172 tok)
- `quadthopter.xml` — Declares for (~2363 tok)
- `robird.xml` (~3205 tok)
- `selfie.xml` (~2388 tok)
- `silverlit_lisas.xml` (~2978 tok)
- `splash.xml` (~2893 tok)
- `twoseastwenty.xml` (~3002 tok)
- `walkera_genius_v2.xml` (~2726 tok)
- `xvert.xml` (~2809 tok)
- `yapa_xsens.xml` (~2585 tok)

## enac_paparazzi/conf/airframes/tudelft/IMAV2013/

- `ardrone2.xml` (~1791 tok)
- `chouchou_lisa_s.xml` (~2073 tok)
- `mavrick_lisa_s.xml` (~2027 tok)
- `quadrotor_lisa_s.xml` (~2190 tok)
- `walkera_genius_v1.xml` (~1935 tok)
- `walkera_V120D02S.xml` (~1856 tok)

## enac_paparazzi/conf/airframes/tudelft/IMAV2013/calibrations/

- `182_calib.xml` (~206 tok)

## enac_paparazzi/conf/airframes/tudelft/calibrations/

- `bebop_bart.xml` (~199 tok)
- `bebop2_22.xml` (~212 tok)
- `bebop2_23.xml` (~201 tok)
- `bebop5.xml` (~377 tok)
- `bebop7.xml` (~192 tok)
- `bebop8.xml` (~201 tok)
- `bebopTUBB17.xml` (~202 tok)
- `ladybird1.xml` (~217 tok)
- `ladybird18.xml` (~318 tok)

## enac_paparazzi/conf/airframes/untested/

- `beagle_bone_black.xml` (~468 tok)
- `delta_wing_minimal.xml` (~1596 tok)
- `demo_cc3d.xml` (~576 tok)
- `demo.xml` (~1167 tok)
- `easy_glider.xml` (~2062 tok)
- `easystar.xml` (~1850 tok)
- `funjet_cam.xml` (~2100 tok)
- `funjet.xml` (~2012 tok)
- `hex_naze32.xml` (~2172 tok)
- `krooz_sd_bre_hexa_mkk.xml` (~2509 tok)
- `krooz_sd_fw.xml` (~2177 tok)
- `krooz_sd_hexa_mkk.xml` (~2492 tok)
- `krooz_sd_okto_mkk.xml` (~2586 tok)
- `krooz_sd_quad_pwm.xml` (~2304 tok)
- `lisa_asctec.xml` (~2088 tok)
- `logger_sd.xml` (~300 tok)
- `quad_cc3d.xml` (~2039 tok)
- `quad_cjmcu.xml` (~2293 tok)
- `quad_flip32.xml` (~2219 tok)
- `quad_revo_nano.xml` (~2129 tok)
- `quadrotor_lisa_m_mkk.xml` (~2001 tok)
- `quadrotor_mlkf.xml` (~2024 tok)
- `quadrotor_pixhawk_lite.xml` (~2522 tok)
- `quadrotor_px4fmu.xml` (~2034 tok)
- `stm32f4_discovery_test.xml` (~1854 tok)
- `turntable_usb.xml` (~295 tok)
- `turntable.xml` (~266 tok)
- `twinjet_overo.xml` (~2028 tok)
- `twinstar.xml` (~2208 tok)
- `wind_tunnel.xml` (~500 tok)

## enac_paparazzi/conf/autopilot/

- `autopilot.dtd` — Declares CDATA (~630 tok)
- `fixedwing_autopilot.xml` (~1234 tok)
- `mfc_ap.xml` (~2351 tok)
- `mfc_darko_ap.xml` (~2496 tok)
- `rotorcraft_autopilot.xml` (~2210 tok)
- `rover.xml` (~1012 tok)

## enac_paparazzi/conf/boards/

- `apogee_1.0_chibios.makefile` — Hey Emacs, this is a -*- makefile -*- (~442 tok)
- `apogee_1.0.makefile` — Hey Emacs, this is a -*- makefile -*- (~287 tok)
- `ardrone2_raw.makefile` (~60 tok)
- `ardrone2.makefile` — Hey Emacs, this is a -*- makefile -*- (~435 tok)
- `beagle_bone_black.makefile` — Hey Emacs, this is a -*- makefile -*- (~268 tok)
- `bebop.makefile` — Hey Emacs, this is a -*- makefile -*- (~420 tok)
- `bebop2.makefile` — Hey Emacs, this is a -*- makefile -*- (~440 tok)
- `booz_1.0.makefile` — booz_1.0.makefile (~136 tok)
- `cc3d.makefile` — Hey Emacs, this is a -*- makefile -*- (~458 tok)
- `chimera_1.0.makefile` — Hey Emacs, this is a -*- makefile -*- (~611 tok)
- `cjmcu.makefile` — Hey Emacs, this is a -*- makefile -*- (~508 tok)
- `disco.makefile` — Hey Emacs, this is a -*- makefile -*- (~468 tok)
- `elle0_1.0.makefile` — Hey Emacs, this is a -*- makefile -*- (~493 tok)
- `elle0_1.2.makefile` — Hey Emacs, this is a -*- makefile -*- (~493 tok)
- `hb_1.1.makefile` — hb_1.1.makefile (~152 tok)
- `hbmini_1.0.makefile` — hbmini_1.0.makefile (~158 tok)
- `krooz_sd.makefile` — Hey Emacs, this is a -*- makefile -*- (~266 tok)
- `lia_1.1_chibios.makefile` — Hey Emacs, this is a -*- makefile -*- (~225 tok)
- `lia_1.1.makefile` — Hey Emacs, this is a -*- makefile -*- (~386 tok)
- `lisa_l_1.0.makefile` — Hey Emacs, this is a -*- makefile -*- (~377 tok)
- `lisa_l_1.1.makefile` — Hey Emacs, this is a -*- makefile -*- (~393 tok)
- `lisa_m_1.0.makefile` — Hey Emacs, this is a -*- makefile -*- (~438 tok)
- `lisa_m_2.0_chibios.makefile` — Hey Emacs, this is a -*- makefile -*- (~164 tok)
- `lisa_m_2.0.makefile` — Hey Emacs, this is a -*- makefile -*- (~72 tok)
- `lisa_m_2.1_chibios.makefile` — Hey Emacs, this is a -*- makefile -*- (~64 tok)
- `lisa_m_2.1.makefile` — Hey Emacs, this is a -*- makefile -*- (~72 tok)
- `lisa_m_common_chibios.makefile` — Hey Emacs, this is a -*- makefile -*- (~309 tok)
- `lisa_m_common.makefile` — Hey Emacs, this is a -*- makefile -*- (~480 tok)
- `lisa_m_defaults.makefile` — Hey Emacs, this is a -*- makefile -*- (~205 tok)
- `lisa_mx_2.0.makefile` — Hey Emacs, this is a -*- makefile -*- (~168 tok)
- `lisa_mx_2.1_chibios.makefile` — Hey Emacs, this is a -*- makefile -*- (~302 tok)
- `lisa_mx_2.1.makefile` — Hey Emacs, this is a -*- makefile -*- (~168 tok)
- `lisa_mx_defaults.makefile` — Hey Emacs, this is a -*- makefile -*- (~339 tok)
- `lisa_mxs_1.0.makefile` — Hey Emacs, this is a -*- makefile -*- (~176 tok)
- `lisa_s_1.0.makefile` — Hey Emacs, this is a -*- makefile -*- (~481 tok)
- `logom_2.6.makefile` — logom_2.6.makefile (~86 tok)
- `navgo_1.0.makefile` — navgo_1.0.makefile (~154 tok)
- `navstik_1.0.makefile` — Hey Emacs, this is a -*- makefile -*- (~314 tok)
- `naze32_rev4.makefile` — Hey Emacs, this is a -*- makefile -*- (~479 tok)
- `naze32_rev5.makefile` — Hey Emacs, this is a -*- makefile -*- (~443 tok)
- `opa_ap_1.0.makefile` — Hey Emacs, this is a -*- makefile -*- (~335 tok)
- `opa_ftd_1.0.makefile` — Hey Emacs, this is a -*- makefile -*- (~246 tok)
- `openpilot_revo_1.0.makefile` — Hey Emacs, this is a -*- makefile -*- (~309 tok)
- `openpilot_revo_nano.makefile` — Hey Emacs, this is a -*- makefile -*- (~354 tok)
- `pc.makefile` — pc.makefile (~42 tok)
- `px4fmu_1.7.makefile` — Hey Emacs, this is a -*- makefile -*- (~278 tok)
- `px4fmu_2.4_chibios.makefile` — Hey Emacs, this is a -*- makefile -*- (~601 tok)
- `px4fmu_2.4.makefile` — Hey Emacs, this is a -*- makefile -*- (~391 tok)
- `px4fmu_4.0.makefile` — Hey Emacs, this is a -*- makefile -*- (~448 tok)
- `px4io_2.4.makefile` — Hey Emacs, this is a -*- makefile -*- (~325 tok)
- `sdlog_1.0.makefile` — sdlog_1.0.makefile (~51 tok)
- `stm32f3_discovery_1.0_chibios.makefile` — Hey Emacs, this is a -*- makefile -*- (~437 tok)
- `stm32f4_discovery.makefile` — Hey Emacs, this is a -*- makefile -*- (~286 tok)
- `swing.makefile` — Hey Emacs, this is a -*- makefile -*- (~334 tok)
- `tiny_0.99.makefile` — tiny_0.99.makefile (~245 tok)
- `tiny_1.1.makefile` — tiny_1.1.makefile (~257 tok)
- `tiny_2.1.makefile` — tiny_2.1.makefile (~243 tok)
- `tiny_2.11.makefile` — tiny_2.11.makefile (~244 tok)
- `twog_1.0.makefile` — twog_1.0.makefile (~243 tok)
- `umarim_1.0.makefile` — unami_1.0.makefile (~237 tok)
- `umarim_lite_2.0.makefile` — umarim_lite_2.0.makefile (~241 tok)
- `vms_ecu_1.0_chibios.makefile` — Hey Emacs, this is a -*- makefile -*- (~438 tok)
- `xvert_1.0.makefile` — Hey Emacs, this is a -*- makefile -*- (~423 tok)
- `yapa_2.0.makefile` — yapa_2.0.makefile (~243 tok)

## enac_paparazzi/conf/chibios/

- `chibios_extra_rules.mk` — Extra rules for ChibiOS rules.mk (~246 tok)
- `chibios_rules.mk` — ARM Cortex-Mx common makefile scripts and rules. (~2123 tok)
- `fatfs.mk` — FATFS files. (~98 tok)

## enac_paparazzi/conf/firmwares/

- `demo.makefile` — Hey Emacs, this is a -*- makefile -*- (~918 tok)
- `fixedwing.makefile` — Hey Emacs, this is a -*- makefile -*- (~211 tok)
- `logger.makefile` — setup.makefile (~715 tok)
- `rotorcraft.makefile` — Hey Emacs, this is a -*- makefile -*- (~1442 tok)
- `rover.makefile` — Hey Emacs, this is a -*- makefile -*- (~1133 tok)
- `setup.makefile` — setup.makefile (~991 tok)
- `test_chibios.makefile` — Hey Emacs, this is a -*- makefile -*- (~1272 tok)
- `test_progs.makefile` — Hey Emacs, this is a -*- makefile -*- (~4653 tok)

## enac_paparazzi/conf/firmwares/subsystems/fixedwing/

- `autopilot.makefile` — Hey Emacs, this is a -*- makefile -*- (~1690 tok)
- `navigation_extra.makefile` (~62 tok)

## enac_paparazzi/conf/firmwares/subsystems/rotorcraft/

- `gps_datalink.makefile` (~44 tok)
- `gps_sim_hitl.makefile` (~28 tok)
- `gps_sirf.makefile` (~41 tok)
- `gps_udp.makefile` (~40 tok)

## enac_paparazzi/conf/firmwares/subsystems/shared/

- `actuators_4015.makefile` — for Tiny v1.1 (~38 tok)
- `actuators_4017.makefile` — for Tiny v2 or Twog v1 (~38 tok)
- `actuators_pwm.makefile` (~57 tok)
- `baro_board.makefile` — Hey Emacs, this is a -*- makefile -*- (~3244 tok)
- `gps_furuno.makefile` (~42 tok)
- `gps_mediatek_diy.makefile` (~46 tok)
- `gps_nmea.makefile` (~41 tok)
- `gps_piksi.makefile` (~42 tok)
- `gps_skytraq.makefile` (~43 tok)
- `gps_ublox_utm.makefile` (~28 tok)
- `gps_ublox.makefile` (~42 tok)
- `hitl.makefile` — Hey Emacs, this is a -*- makefile -*- (~171 tok)
- `nps_common.makefile` — Hey Emacs, this is a -*- makefile -*- (~608 tok)
- `nps.makefile` — Hey Emacs, this is a -*- makefile -*- (~64 tok)
- `sdlog.makefile` (~40 tok)
- `spi_master.makefile` — Hey Emacs, this is a -*- makefile -*- (~74 tok)
- `uart.makefile` — Hey Emacs, this is a -*- makefile -*- (~153 tok)
- `udp.makefile` — Hey Emacs, this is a -*- makefile -*- (~130 tok)

## enac_paparazzi/conf/flight_plans/

- `basic_sim.xml` (~1343 tok)
- `basic.xml` (~1214 tok)
- `cube.xml` (~737 tok)
- `demo_gvf.xml` (~1264 tok)
- `demo_module.xml` (~388 tok)
- `dummy.xml` (~93 tok)
- `dynamic_sectors.xml` (~1025 tok)
- `example.xml` (~115 tok)
- `failsafe_modes.xml` (~184 tok)
- `flight_plan.dtd` — Declares CDATA (~1852 tok)
- `huit.xml` (~242 tok)
- `joystick.xml` (~1209 tok)
- `kv_svalbard.xml` (~893 tok)
- `landing.xml` (~421 tok)
- `mission_fw.xml` (~859 tok)
- `nav_modules.xml` (~2118 tok)
- `poles.xml` (~1052 tok)
- `quadshot_delft.xml` (~1224 tok)
- `rotorcraft_basic_geofence.xml` (~1108 tok)
- `rotorcraft_basic_safety.xml` (~1207 tok)
- `rotorcraft_basic_superbitrf_from_hand.xml` (~1766 tok)
- `rotorcraft_basic_superbitrf.xml` (~1311 tok)
- `rotorcraft_basic.xml` (~986 tok)
- `rotorcraft_cam.xml` (~987 tok)
- `rotorcraft_guided_flightplan.xml` (~1859 tok)
- `rotorcraft_guido_optitrack.xml` (~1040 tok)
- `rotorcraft_optitrack_stereoavoid.xml` (~1520 tok)
- `rotorcraft_optitrack.xml` (~1025 tok)
- `rotorcraft_survey.xml` (~1317 tok)
- `rotorcraft_vision.xml` (~360 tok)
- `sectors.dtd` (~148 tok)
- `sectors.xml` (~258 tok)
- `tcas.xml` (~1165 tok)
- `versatile_airspeed.xml` (~2386 tok)
- `versatile_geofence.xml` (~2412 tok)
- `versatile.xml` (~2304 tok)
- `zamboni_survey_test.xml` (~1084 tok)

## enac_paparazzi/conf/flight_plans/AGGIEAIR/

- `aggieair_landing.xml` (~119 tok)
- `BasicTuning_BlueRoom2.xml` (~1164 tok)
- `BasicTuning_Launcher.xml` (~1166 tok)
- `CachJct_Gallo_temp_rect_survey.xml` (~1103 tok)
- `rotorcraft_opticlow_test.xml` (~1043 tok)

## enac_paparazzi/conf/flight_plans/HOOPERFLY/

- `hooperfly_gsa_one.xml` (~1574 tok)
- `hooperfly_rotorcraft_multiflight.xml` (~2350 tok)
- `hooperfly_rotorcraft_nocturnal.xml` (~2321 tok)

## enac_paparazzi/conf/flight_plans/competitions/

- `EMAV2006.xml` (~707 tok)
- `EMAV2008.xml` (~2194 tok)
- `EMAV2009_data.xml` (~568 tok)
- `EMAV2009_safety.xml` (~118 tok)
- `EMAV2009.xml` (~529 tok)
- `IMAV2014_carto.xml` (~2215 tok)
- `IMAV2014_data.xml` (~320 tok)
- `IMAV2014_digit.xml` (~1564 tok)
- `IMAV2014_vision.xml` (~2184 tok)
- `mav05_ccw.xml` (~433 tok)
- `mav05_cw.xml` (~167 tok)
- `mav05.xml` (~151 tok)
- `mav06.xml` (~1285 tok)
- `mav07.xml` (~1840 tok)
- `MAV08_legs.kml` (~1998 tok)
- `MAV08_no_fly_boundaries.kml` (~8399 tok)
- `mav08.xml` (~1424 tok)

## enac_paparazzi/conf/flight_plans/formation/

- `form_follow.xml` — Declares out (~696 tok)
- `form_leader.xml` — Declares out (~790 tok)

## enac_paparazzi/conf/flight_plans/legacy/

- `corsica_sectors.xml` (~419 tok)
- `corsica.xml` (~1257 tok)
- `creidlitz.xml` (~926 tok)
- `fp_tp_auto.xml` (~1816 tok)
- `grosslobke_demo.xml` (~1598 tok)
- `grosslobke_kreise.xml` (~622 tok)
- `grosslobke_start.xml` (~205 tok)
- `ingolfsskali.xml` (~981 tok)
- `kalscott.xml` (~1370 tok)
- `muret_demo_1.xml` (~276 tok)
- `muret_for.xml` (~261 tok)
- `nordlys.xml` (~1376 tok)
- `quadshot_land_takeoff_again.xml` (~1295 tok)
- `rotorcraft_krooz.xml` (~1321 tok)
- `rotorcraft_oa_avoid.xml` (~266 tok)
- `slayer_training.xml` (~1106 tok)

## paparazzi/conf/simulator/flightgear/

- `bebop-set.xml` (~361 tok)
