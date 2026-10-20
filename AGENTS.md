# AGENTS.md

## Purpose
- This repo is STM32F407 firmware for the `Infantry_01` target.
- It uses C11, CMake, Ninja, and the STM32 `arm-none-eabi-gcc` toolchain.
- Handwritten logic mostly lives in `Application/`, `ALL_Task/`, `Bsp/`, `Components/`, and `Algorithm/`.
- Generated STM32CubeMX/HAL/USB/FreeRTOS code mostly lives in `Core/`, `USB_DEVICE/`, `Drivers/`, `Middlewares/`, and `cmake/stm32cubemx/`.

## Rules Audit
- Checked `.cursor/rules/`: not present.
- Checked `.cursorrules`: not present.
- Checked `.github/copilot-instructions.md`: not present.
- No prior `AGENTS.md` existed in this repository root.

## Build System
- Root build file: `CMakeLists.txt`.
- Presets file: `CMakePresets.json`.
- Generator: `Ninja`.
- Toolchain file: `cmake/gcc-arm-none-eabi.cmake`.
- Main target: `Infantry_01`.
- Main artifact: `build/<preset>/Infantry_01.elf`.
- `compile_commands.json` is generated in `build/<preset>/`.
- `.clangd` is configured to use `build/Debug` as the compilation database.

## Canonical Commands

### Configure
```bash
cmake --preset Debug
cmake --preset Release
```

### Build
```bash
cmake --build --preset Debug
cmake --build --preset Release
```

### Reconfigure + Build
```bash
cmake --preset Debug && cmake --build --preset Debug
```

### Clean
```bash
cmake --build build/Debug --target clean
cmake --build build/Release --target clean
```

### Discover Presets / Targets
```bash
cmake --list-presets
ninja -C build/Debug -t targets
```

## Test / Lint Reality
- No unit-test framework is configured in this repo.
- `CTest` is not enabled; no `enable_testing()` or `add_test()` calls were found.
- No dedicated lint config was found for `clang-format`, `clang-tidy`, `cppcheck`, or `uncrustify`.
- The practical baseline validation step is a normal Debug build.

## Single Test Guidance
- There is no true "single test" command because no automated tests exist.
- If asked to run one test, say so explicitly and use targeted compilation as the closest substitute.
- The most useful focused check is rebuilding one object file for the changed source.

### Example: Rebuild One Source File
```bash
cmake --build build/Debug --target "CMakeFiles/Infantry_01.dir/Bsp/uart/bsp_uart.c.obj"
```

### How To Find An Object Target
- Look in `build/Debug/build.ninja`.
- Or inspect `build/Debug/compile_commands.json` and convert the output path into a target name.
- Use targeted rebuilds for quick compile checks, not as a replacement for full firmware validation.

## Current Build Status
- Verified 2026-10-19: `cmake --build --preset Debug` compiles and links `Infantry_01.elf` successfully, including `Bsp/uart/bsp_uart.c`.
- The earlier note about `&pData->rxdatas` type errors was stale: active call sites already use `pData->rxdatas`; the `&` variants are commented out.
- Warnings remain and are the baseline, not regressions: signedness comparisons, unused parameters, `-Wtype-limits` dead guards in `DM_MIT_send_ctrl_cmd`, misleading indentation in `M2006_VEL_PID_set_para`.
- No `-Werror`; do not treat these warnings as newly introduced.

## Architecture Landmarks
- `Core/Src/main.c` initializes HAL/peripherals, then starts FreeRTOS.
- `Core/Src/freertos.c` creates task threads.
- Task entry points are implemented in `ALL_Task/*_task.c`.
- Shared system state is centralized in `Application/robot_global.*`.
- Motor abstractions live in `Components/motor/`.
- Remote-control parsing lives in `Components/remote/`.
- Auto-aim parsing/control lives in `Application/auto_ctrl.*`.

## 算法联调上下文 (PID/前馈现状)

> 2026-10-19 会话沉淀。目标：算法联调、加入前馈算法。方向已定并落盘执行文档：`docs/前馈联调说明.md`（轮次总表、代码清单、标定步骤、回退开关）。
> 以下均已读码核实并附 file:line，改控制/前馈代码前可直接引用，无需重读全文件。

### 任务与数据流速记
- 7 任务创建于 `Core/Src/freertos.c:96`：sensor / motor / gimbal / chassis / shoot / recv_cmd / send_info；共享状态 `robot_ctrl`（`Application/robot_global.h:123`）。
- 控制链路：任务层(2ms) `set_target` 写目标 → `motor_task`(1ms) `Motor_All_Update()` 跑 PID → CAN 组帧发送（`Components/motor/motor.c:1489` CAN1 / `motor.c:1544` CAN2）。

### PID 全景（全部在 `Components/motor/motor.c`，离散 PID 无 dt 项，按 1ms 周期整定）
| 电机 | 结构 | 实质 | 关键参数 (行号) |
|---|---|---|---|
| M3508 底盘×4 | 单环速度 PID | 纯 P | Kp=10, Ki=0, Kd=0, Out=10000 (motor.c:1386) |
| M3508 摩擦轮×2 | 单环速度 PID | 纯 P | Kp=20, Ki=0, Out=16384 (motor.c:1423) |
| M2006 拨弹 | 位置+速度串级 | 外环 PI+死区12清积分；内环 Ki=0.1 | motor.c:780/1398；速度覆盖模式 set_target para_num=2 (motor.c:829) |
| GM6020 偏航 | 位置-速度串级+前馈通道 | P+速度阻尼外环 / P 内环 | P_Kp=420, P_Kd=1.45, V_Kp=200, Out=25000 (motor.c:1409/1068) |
| J4310 俯仰 | 达妙 MIT | MCU 无 PID，电机内 PD | kp=25, kd=2, P_MAX=12.5 (motor.c:1372)；`DM_update` 为空 (motor.c:200) |

### 已有前馈点（联调起点）
- yaw 前馈三级链：`chassis_task.c:481` 写 `robot_ctrl.chassis.yaw_speed` → `gimbal_task.c:337-345` 斜率限幅+一阶滤波（`YAW_FF_*` 宏 `gimbal_task.c:37-44`）→ `gimbal_task.c:364` `set_target(yaw_m, 2, yaw_out, ff)` → `motor.c:1114` `v_des * V_Only_Kp(300)` 叠加到电压输出。
- 任务层隐式外环：`gimbal_task.c:332-367` IMU 误差 + 小积分(0.55, 限幅0.1) + pitch 位置映射；底盘跟随纯 P=0.5（`chassis_task.c:466`）。

### 改前馈时的已知风险/观察
- 前馈饱和：`YAW_FF_LIMIT=200 × V_Only_Kp=300 = 60000`，远超 `Out_Max=25000`，快速自转时前馈+P 项会顶满限幅。
- 电机层积分无 dt、无积分分离；失能时清积分清输出（M3508 motor.c:517、GM6020 motor.c:1077 失能期间持续同步 p_des 防甩头）。
- M3508/M2006 `_last_error` 为 int16，极端跳变可截断。
- 视觉链路语义：`recv_cmd_task.c:13` 250ms 软衰减 / 500ms 硬超时；自瞄丢目标 1s 保持后进入扫描（`gimbal_task.c:26` 附近宏）。

### 待办
- 算法联调：执行文档 `docs/前馈联调说明.md` 已落盘。Round 0 已完成（2026-10-19）：`DM_MIT_get_status` VEL/T 反归一化修复、`robot_global.h` 单位注释修正、全量 Debug 构建验证绿。下一步 Round 1a：俯仰重力矩前馈（`motor.c` 4 处，代码见执行文档 §2，待审阅确认后合入）。

## Generated-Code Boundaries
- Treat `Core/`, `USB_DEVICE/`, and `cmake/stm32cubemx/` as generated or generator-adjacent.
- Preserve every `/* USER CODE BEGIN ... */` and `/* USER CODE END ... */` region.
- Prefer adding custom logic inside user blocks instead of reshaping generated control flow.
- Avoid style-only rewrites in generated files.
- Do not rename generated entry points such as `MX_*`, IRQ handlers, or `Error_Handler()`.

## Code Style

### Language And Types
- Use C11-compatible code.
- Prefer fixed-width integer types like `uint8_t`, `uint16_t`, `int32_t`.
- Use project math aliases like `fp32` when touching existing control/math code that already uses them.
- Keep enum typedefs suffixed with `_e`.
- Keep struct typedefs suffixed with `_t`.
- Module-private structs may remain plain `struct name` when that matches local style.

### Naming
- Functions use lower snake case: `motor_get_device`, `parse_target_data`.
- Task entry points use `*_task_func`.
- Macros use upper snake case: `RC_DEADZONE`, `AUTO_SCAN_SPEED_RAD_S`.
- Encode units in constant names when useful: `_MS`, `_TICKS`, `_RAD`, `_RPM`, `_LIMIT`.
- Global/static device instances often use a `g_` prefix.
- Keep names descriptive; avoid cryptic abbreviations outside math/control hot paths.

### Includes
- Include the module's own header first when it has one.
- Prefer `<...>` for standard library headers.
- Prefer `"..."` for project, HAL, CMSIS, and FreeRTOS headers.
- Existing code uses many relative includes; stay consistent with the surrounding directory.
- Do not do broad include-path cleanup unless the task requires it.
- Match filename casing exactly; Windows hides case mistakes that other environments may not.

### Formatting
- Follow the surrounding file instead of imposing a new formatter style.
- Most handwritten files use 4-space indentation.
- Keep opening braces on the same line in handwritten code.
- Group related macros near the top of the file.
- Use blank lines to separate logical sections.
- Avoid formatting-only churn.

### Comments
- The repo already contains many Chinese comments and section banners.
- Preserve existing comments unless they become wrong.
- Add comments only for non-obvious hardware, control, or safety logic.
- Do not add comments that merely restate the code.

### Error Handling
- Follow the existing embedded pattern of early returns for bad pointers and invalid input.
- Check `NULL` aggressively in drivers, parsers, and reusable helpers.
- For HAL init/config failures in startup code, use `Error_Handler()`.
- For runtime control paths, prefer fail-safe state changes over optimistic assumptions.
- When comms drop, follow the existing pattern: clear enables, relax actuators, and reset transient state.
- Avoid infinite blocking in reusable helpers unless the function is explicitly fatal.

### RTOS And Concurrency
- Respect task cadence; several loops run at 2 ms or similarly tight periods.
- Avoid long blocking calls in high-priority tasks.
- Use `osDelay()` and tick-delta timing in the same style as neighboring code.
- Understand ownership of fields in `robot_ctrl` before changing shared state.
- Preserve startup sequencing such as waiting for sensors before enabling motion.

### Safety / Control Logic
- Mechanical limits, dead zones, timeout guards, and offline checks are intentional and safety-critical.
- Keep unit semantics straight; some comments mention degrees while many control constants are radians.
- Preserve clamping and watchdog behavior unless the task explicitly changes system behavior.
- Prefer small, local edits in control logic and verify the surrounding state machine.

## Agent Workflow
- Read the target file and at least one neighboring module before changing control code.
- Reuse existing abstractions such as `motor_device`, `robot_ctrl`, and UART/USB wrappers.
- Keep cross-module dependencies narrow; update headers only when an interface actually changes.
- If you edit generated files, keep changes inside user blocks whenever possible.
- After edits, run a full Debug build or at least a targeted object rebuild.
- If validation fails, distinguish pre-existing failures from failures introduced by your change.
- Be explicit that the repo has no tests instead of inventing unsupported test commands.

## Default Validation
```bash
cmake --preset Debug
cmake --build --preset Debug
```

## Fast Focused Validation
```bash
cmake --build build/Debug --target "CMakeFiles/Infantry_01.dir/path/to/file.c.obj"
```

## When Unsure
- Prefer repo consistency over textbook purity.
- Prefer fail-safe behavior over cleverness.
- Prefer small diffs over broad cleanup.
- Prefer honest reporting over pretending tests or lint steps exist.
