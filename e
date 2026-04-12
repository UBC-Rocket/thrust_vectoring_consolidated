[33mcommit 78462d065a0f70f9687195c587ba410a05ecbbce[m[33m ([m[1;36mHEAD[m[33m -> [m[1;32membedded-software/esc/stage[m[33m)[m
Author: ethan6138 <ethansun882@gmail.com>
Date:   Tue Apr 7 22:10:36 2026 -0700

    Feat: open loop start up phase fixed

[33mcommit a1efb77d2029be1c0c2535db50d10391233c106b[m
Author: ethan6138 <ethansun882@gmail.com>
Date:   Sun Apr 5 18:05:22 2026 -0700

    Feat: duty cycle set, changed startup time to 10s, corrected target rpm and freq math

[33mcommit a198d7ccb6cf0577931ccb4e61f5f177444c3688[m
Author: ethan6138 <ethansun882@gmail.com>
Date:   Sun Apr 5 15:38:26 2026 -0700

    Update: ADC1 Pins

[33mcommit 355876703319582b5a07b7593ee33113d79d5e7d[m
Author: ethan6138 <ethansun882@gmail.com>
Date:   Sun Apr 5 13:16:09 2026 -0700

    Feat: deadtime set from 0->4

[33mcommit 94fd3caa7235c9b9dab7afbbeec788378e3d172f[m
Author: ethan6138 <ethansun882@gmail.com>
Date:   Sun Apr 5 13:11:54 2026 -0700

    feat: blanking time, feat: open loop

[33mcommit 6693d1fc36b1f5df566d5c35b5a8312b01740528[m
Author: ethan6138 <ethansun882@gmail.com>
Date:   Fri Apr 3 17:24:22 2026 -0700

    feat: ring buffer size 8->32, updated commutation.c to match avionics hardware standup1 slide 4 commutation

[33mcommit 54f7532080febaba70c0af89d16dcb63debc4578[m
Author: ethan6138 <ethansun882@gmail.com>
Date:   Wed Apr 1 22:35:47 2026 -0700

    feat: implemented adc comm thorugh convcpltcallback for 6step commutation

[33mcommit 7986039060a64ea8c863d1f6eb2f83aef30096ea[m
Author: ethan6138 <ethansun882@gmail.com>
Date:   Wed Apr 1 20:44:23 2026 -0700

    initialized ADC1 with continous DMA, setup ring buffer in app.c

[33mcommit 6782166410394035775b161b1722b509894bad28[m
Author: Shadab Mizan <shadabsmizan@gmail.com>
Date:   Sat Mar 28 13:30:03 2026 -0700

    ESC in open-loop code converted into a VS Code project.

[33mcommit 0ebddd304f4d7404a3b8309515785523bbdc36c1[m[33m ([m[1;31morigin/main[m[33m, [m[1;31morigin/embedded-software/bms_firmware[m[33m, [m[1;31morigin/HEAD[m[33m, [m[1;32mmain[m[33m)[m
Merge: ca3e82e b6f3918
Author: benedikthoward <85261373+benedikthoward@users.noreply.github.com>
Date:   Sun Mar 22 15:54:43 2026 -0700

    Merge pull request #20 from UBC-Rocket/embedded-software/stage
    
    Embedded software/stage

[33mcommit b6f3918f590129069fdf30b29988745c1d684e78[m
Merge: d97029e eae992f
Author: benedikthoward <85261373+benedikthoward@users.noreply.github.com>
Date:   Sun Mar 22 15:50:23 2026 -0700

    Merge pull request #19 from UBC-Rocket/embedded-software/feat/ekf-sim-test
    
    Embedded software/feat/ekf sim test

[33mcommit eae992fb265aa5cfb659a3c3e12c71ff23743d14[m
Author: 13carpileup <climie.alex@gmail.com>
Date:   Sun Mar 22 15:44:19 2026 -0700

    Merge !

[33mcommit d97029e0dc50fd065503cde038058bdb630e9283[m
Author: Random121 <59186475+Random121@users.noreply.github.com>
Date:   Sun Mar 22 15:35:24 2026 -0700

    feat: add force to pwm mapping (#18)
    
    * firmware/controls/pwm: add pwm module header
    
    * Create test_pwm.c
    
    * pwm tests
    
    * firmware/controls/pwm: add pwm module source
    
    * firmware/controls/tests: add pwm tests to unity test list
    
    * firmware/controls/tests: fix pwm tests
    
    * firmware/controls/pwm: add additional doc comments
    
    * firmware/controls/pwm: remove unecessary assignment
    
    * firmware/controls/tests: update CMakeLists to include pwm tests
    
    ---------
    
    Co-authored-by: Bren Klein <klein.bren@icloud.com>

[33mcommit e7babd559e020db6cf1e53a9673e671672794f8a[m
Merge: b5dbb3e d63cf60
Author: 13carpileup <climie.alex@gmail.com>
Date:   Sun Mar 22 15:26:47 2026 -0700

    Merge remote-tracking branch 'origin/embedded-software/stage' into embedded-software/feat/ekf-sim-test

[33mcommit b5dbb3ed878194bd7eae09418289eb0423670afb[m
Author: 13carpileup <climie.alex@gmail.com>
Date:   Sun Mar 22 14:46:21 2026 -0700

    remove settings.json from tracking

[33mcommit 87c8ba771c8edb592f45ca080c18b2ae995ba1f5[m
Author: 13carpileup <climie.alex@gmail.com>
Date:   Sun Mar 22 10:47:42 2026 -0700

    Added test data
    
    for gh tests

[33mcommit 592fe286f70ce8c06635b058752bc90e04ea9156[m
Author: 13carpileup <climie.alex@gmail.com>
Date:   Sun Mar 22 10:27:29 2026 -0700

    Updated CMakeLists to set working directory

[33mcommit a6d2f0cdab3c58f52f4a5e487fb6c8102e73f5e2[m
Author: benedikthoward <benedikthoward@gmail.com>
Date:   Thu Mar 19 22:59:00 2026 -0700

    added EKF tests, made EKF library more independent of the firmware, changed it to be a error state kalman filter, fixed a covariance update issue...

[33mcommit d63cf60bffd3a116167affabf09f3c6b69a95726[m
Merge: 79a4d32 57e93e4
Author: benedikthoward <85261373+benedikthoward@users.noreply.github.com>
Date:   Sun Mar 15 16:40:32 2026 -0700

    Merge pull request #16 from UBC-Rocket/embedded-software/feat/updated-ekf
    
    Embedded software/feat/updated ekf

[33mcommit 57e93e40feff29415a01ddb2822929273c431fc8[m[33m ([m[1;31morigin/embedded-software/feat/updated-ekf[m[33m)[m
Author: 13carpileup <climie.alex@gmail.com>
Date:   Sun Mar 15 16:28:22 2026 -0700

    fixed body ekf explosion

[33mcommit 79a4d32ebf36a17b094c76eedbee175e7f67d15c[m
Merge: 9073f6a 1816551
Author: Ivan <134543432+ivan-lyf@users.noreply.github.com>
Date:   Sun Mar 15 16:09:28 2026 -0700

    Merge pull request #15 from UBC-Rocket/feat-(ground-control)-send-reference-and-config-values-via-radio
    
    Feat (ground control) send reference and config values via radio

[33mcommit 1816551d9efe549cc743b07c17cd5b7f664210d0[m[33m ([m[1;31morigin/feat-(ground-control)-send-reference-and-config-values-via-radio[m[33m)[m
Author: danelzhan <164102172+danelzhan@users.noreply.github.com>
Date:   Sun Mar 15 16:02:27 2026 -0700

    ui labels updated

[33mcommit e88d4017b41644852867294c0871b6ed0bb781c2[m
Author: danelzhan <164102172+danelzhan@users.noreply.github.com>
Date:   Sun Mar 15 15:55:22 2026 -0700

    Updated UI and functionality to send PID, Reference, Config packets

[33mcommit 9073f6a6563e46be2c5f9133d1462b316e03c312[m
Merge: 6401690 99bc4dc
Author: Daniel Zhang <164102172+danelzhan@users.noreply.github.com>
Date:   Sun Mar 15 14:07:10 2026 -0700

    Merge pull request #13 from UBC-Rocket/embedded-software/ground-control-pid-panel/stage
    
    fixed UI for pid panel to match actual protobuf

[33mcommit 640169083562206b1d11c9f7406c885b7aa5e9da[m
Merge: ac54174 6b840e5
Author: Steven Chen <109647201+Shengw3n@users.noreply.github.com>
Date:   Sun Mar 15 14:04:10 2026 -0700

    Merge pull request #10 from UBC-Rocket/feature/pid-qt-firmware
    
    feat(flight-controller): apply SetPidGains/SetReference/SetConfig radio commands when armed

[33mcommit 99bc4dc9cbc496a83afe5ea342f9216d7eae14e1[m[33m ([m[1;31morigin/embedded-software/ground-control-pid-panel/stage[m[33m)[m
Author: ivan-lyf <yingfanluo@gmail.com>
Date:   Sun Mar 15 14:00:25 2026 -0700

    fixed UI for pid panel to match actual protobuf

[33mcommit 6b840e5bb7b42a60ff7d44ada4132ca3166f84d3[m
Author: Steven Chen <109647201+Shengw3n@users.noreply.github.com>
Date:   Sun Mar 15 12:46:43 2026 -0700

    feat(flight-controller): better mission logic

[33mcommit ac54174c18308a4efc08e6df072f2941c772ddd2[m
Merge: 1dd9f0e 92eb596
Author: Steven Chen <109647201+Shengw3n@users.noreply.github.com>
Date:   Sun Mar 15 12:23:34 2026 -0700

    Merge pull request #12 from UBC-Rocket/embedded-software/fix-status-no-show/stage
    
    skip telemetry data when status is sent

[33mcommit 92eb596d0c8b04541befd9bc9c5ffd2faeee907d[m[33m ([m[1;31morigin/embedded-software/fix-status-no-show/stage[m[33m)[m
Author: Steven Chen <109647201+Shengw3n@users.noreply.github.com>
Date:   Sun Mar 15 12:14:46 2026 -0700

    Potential fix for pull request finding

[33mcommit 6bd2890c22ebc0fdfa3e2e2d70fd544c996699f8[m
Author: ivan-lyf <yingfanluo@gmail.com>
Date:   Sun Mar 15 12:08:33 2026 -0700

    skip telemetry data when status is sent

[33mcommit efb9ef02173f14f46d232b2fab476556456b7ed3[m
Author: 13carpileup <climie.alex@gmail.com>
Date:   Sun Mar 15 11:53:19 2026 -0700

    merging with benedikts branch

[33mcommit ffacc37ba450c5b2e592237ea8199ce638b2ce5e[m
Merge: 6d0a222 6a20f66
Author: 13carpileup <climie.alex@gmail.com>
Date:   Sun Mar 15 11:31:27 2026 -0700

    Merge remote-tracking branch 'origin/embedded-software/firmware/stage/feat/ring_buffer_macro' into embedded-software/feat/updated-ekf

[33mcommit 1dd9f0ee618010c02d32d40645008374ddbae2c5[m
Merge: b94b034 7a3f896
Author: benedikthoward <85261373+benedikthoward@users.noreply.github.com>
Date:   Sun Mar 15 11:26:44 2026 -0700

    Merge pull request #7 from UBC-Rocket/embedded-software/ground-control-bug-fix/stage
    
    PID values sending

[33mcommit 7a3f896054e36634b7529993ef759df80d3198dd[m
Author: ivan-lyf <yingfanluo@gmail.com>
Date:   Sun Mar 15 11:22:35 2026 -0700

    added qtshadertools

[33mcommit b94b034af23f0d75216518f4ca0e38ff9a8cfa60[m
Merge: ca3e82e 6a20f66
Author: benedikthoward <85261373+benedikthoward@users.noreply.github.com>
Date:   Sun Mar 15 11:20:23 2026 -0700

    Merge pull request #8 from UBC-Rocket/embedded-software/firmware/stage/feat/ring_buffer_macro
    
    Consolidate ring buffers, GPS struct, and timestamp into shared libs

[33mcommit 6d0a222126303c7aa42fdf902c95833c03136479[m
Author: 13carpileup <climie.alex@gmail.com>
Date:   Sun Mar 15 11:19:45 2026 -0700

    updated tests
    
    updating tests for new ekf (adjusting args passed to init_ekf, tick_ekf functions, and removing mat_mul)

[33mcommit 12308fea42b202374bf130000da1815597e61a53[m
Author: 13carpileup <climie.alex@gmail.com>
Date:   Sun Mar 15 11:06:05 2026 -0700

    replace old EKF files with updated EKF

[33mcommit 37e21ba43016e841bdbd87af97f454274a7dcc54[m
Author: 13carpileup <climie.alex@gmail.com>
Date:   Sun Mar 15 11:05:42 2026 -0700

    replace old EKF files with new

[33mcommit 4e67c696ed0df9b676e2d8c40d3e40efad760a8f[m
Author: Steven Chen <109647201+Shengw3n@users.noreply.github.com>
Date:   Sun Mar 15 10:52:39 2026 -0700

    feat(flight-controller): apply SetPidGains/SetReference/SetConfig radio commands when armed

[33mcommit 45ba3ec41125222d83de5146f87b59c06c15b825[m
Author: danelzhan <164102172+danelzhan@users.noreply.github.com>
Date:   Sun Mar 15 10:51:37 2026 -0700

    Bug fixees

[33mcommit 6a20f66ab9b758ed66f43244201fa1f579ca9aa1[m[33m ([m[1;31morigin/embedded-software/firmware/stage/feat/ring_buffer_macro[m[33m)[m
Author: benedikthoward <benedikthoward@gmail.com>
Date:   Tue Mar 10 23:28:24 2026 -0700

    Consolidate ring buffers, GPS struct, and timestamp into shared libs
    
    Replace 6 copy-pasted SPSC ring buffer implementations (accel, gyro,
    ms5611, ms5607, GPS fix, radio msg) with a single generic
    SPSC_RING_DECLARE macro in libs/collections/. Extract the shared
    gps_fix_t struct into libs/gnss-protocol/ so both boards use one
    definition. Move the DWT timestamp utility from the FC into
    libs/timestamp/ for cross-board reuse.

[33mcommit ca3e82e0f335d8c4190ed66371e0252b53389312[m
Author: benedikthoward <benedikthoward@gmail.com>
Date:   Tue Mar 10 12:20:35 2026 -0700

    docs: update CODEOWNERS, branching model, and team leads
    
    Rewrite CODEOWNERS with correct embedded-software/ paths and tvr-* team handles.
    Simplify branching model to two-tier (feature → stage → main).
    Add team leads table to CONTRIBUTING.md.

[33mcommit 7ed00a7ee161d0ec74adec39b2ea00e75925d27d[m[33m ([m[1;31morigin/mechanical-hardware/stage[m[33m, [m[1;31morigin/electrical-hardware/stage[m[33m)[m
Author: benedikthoward <benedikthoward@gmail.com>
Date:   Mon Mar 9 00:07:33 2026 -0700

    Add workflow_dispatch to all CI workflows for manual triggers

[33mcommit d17b3af6da7557099aad725ed5c826ab08c12711[m
Author: benedikthoward <benedikthoward@gmail.com>
Date:   Sun Mar 8 23:57:27 2026 -0700

    Fix GNSS Radio CI: use lowercase preset names (debug/release)

[33mcommit 63234464a8732cce6a00d55f336eb564c3050671[m
Author: benedikthoward <benedikthoward@gmail.com>
Date:   Sun Mar 8 23:53:29 2026 -0700

    Streamline VS Code dev experience and update README
    
    - Add firmware/.vscode/ with build-all tasks, per-board debug configs,
      and cmake.sourceDirectory board selector
    - Use env vars for OpenOCD paths (no hardcoded machine-specific paths)
    - Normalize GNSS CMake presets to lowercase (debug/release)
    - Add CI badges, env var docs, and contributor grid to README

[33mcommit b6c6fb7f4b60b48e083b1dbd0f613fe5e450e427[m
Author: benedikthoward <benedikthoward@gmail.com>
Date:   Sun Mar 8 23:14:13 2026 -0700

    Streamline VS Code dev experience and update README

[33mcommit dedf40d63915009ea4e02343d05772c4cda3fb57[m
Author: benedikthoward <benedikthoward@gmail.com>
Date:   Sun Mar 8 21:52:00 2026 -0700

    Extract bus-agnostic sensor protocol drivers into shared firmware/libs/sensors library

[33mcommit c0a90cc604d31a9135bbbf221e6d5e952dc6723a[m
Author: benedikthoward <benedikthoward@gmail.com>
Date:   Sun Mar 8 17:47:53 2026 -0700

    feat: consolidate TVR mono-repo
    
    Merge ulysses, ulysses-gnss-radio, ulysses-ground-control, and
    rocket-protocol-lib into a single mono-repo.

[33mcommit 53fdf0c6b5e90814978c3c70a61191f0744421de[m
Author: benedikthoward <85261373+benedikthoward@users.noreply.github.com>
Date:   Sun Mar 8 15:32:24 2026 -0700

    Initial commit
