# The Robotics SDK: overview

What the SDK drives, how it works, how to use it safely and what it costs an app. The [README](../README.md) has the quick start and the state of this release; the two headers, [`robotics.h`](../components/robotics/include/robotics.h) and [`robotics_sys.h`](../components/robotics_sys/include/robotics_sys.h), are the contract.

## The module

The Robotics (KDROB001) plugs into the expansion header J3 of the Kode Dot. It carries an AT8833 dual H-bridge, a servo connector (J402), and two small connectors (J403, J404) with the header's free GPIOs and its I2C pair. The SDK covers what the module has, and only that. Whatever an app wires to the module's free lines - an encoder on a motor, a sensor, a second servo - is the app's, through kode-sdk's own `kode_pin_*()` calls (`kode_sys_exp_*()` in an Active Object); `robotics_header_position()` says where those lines are.

## The two layers

kode-sdk has two layers, and so does this SDK. You pick by what is calling.

| | Standard layer | Extended layer |
|---|---|---|
| Header, component | `robotics.h`, `robotics` | `robotics_sys.h`, `robotics_sys` |
| For | `app_main()` and your own tasks | An Active Object (QP/C) |
| A call | Waits for the owner's verdict and answers `true` or `false`; `robotics_error()` says why, in words | Never waits. A **command** posts and returns a `kode_status_t`; a **request** takes `QActive *me` and is answered once with a signal |
| Events | By name: `robotics_event_add_listener("module_found", fn)`, `robotics_event_wait("stepper_done", ms, &e)` | Signals: `robotics_sys_subscribe(&me->super)`, then `ROBOTICS_MODULE_FOUND_SIG` and the rest arrive like any signal |
| From the other kind of caller | An Active Object's call is refused, and `robotics_error()` says "robotics_motor_drive was called from an Active Object (AO_x); robotics.h calls block - call robotics_sys_* instead" | Meant for an Active Object; nothing in it waits |

Both go to the same **owner** (`robotics_aos`, one Active Object that is the only code that drives the module), and share the types of `robotics_types.h` (`robotics_status_t`, `robotics_motor_t`, the stepper's configuration...). The console example uses the extended layer alone.

From a task:

```c
#include "kode.h"
#include "robotics.h"

void app_main(void)
{
    robotics_begin();                          // kode_init() is done inside; the Dot on a USB-C charger
    robotics_watchdog_set(5000);               // off by itself if the app goes quiet for 5 s
    while (!robotics_wait_present(5000)) {
        kode_print("no Robotics on J3 yet");   // J3's supply may take a while
    }
    if (!robotics_motor_drive(ROBOTICS_MOTOR_A, 60)) {      // forward at 60 %, along the ramp
        kode_print("%s", robotics_error());
    }
    robotics_servo_angle(ROBOTICS_SERVO_1, 90);
    kode_delay(3000);                          // under the 5 s, so no call is needed to feed it
    robotics_stop();
}
```

From an Active Object (`examples/outputs` is the whole of it):

```c
#include "robotics_sys.h"

static QState Outputs_initial(Outputs *me, void const *par)
{
    robotics_sys_begin();                      // in the thread that called kode_init()
    robotics_sys_subscribe(&me->super);        // the news, before the module can be found
    return Q_TRAN(&Outputs_waiting);
}
// ROBOTICS_MODULE_FOUND_SIG -> robotics_sys_watchdog_set(5000);           // a command
// the top button -> robotics_sys_outputs_begin(&me->super);              // a request
// ROBOTICS_OUTPUTS_BEGIN_RSP_SIG -> a robotics_sys_answer_t: rc, why; then
//     robotics_sys_output_set(ROBOTICS_OUTPUT_1, 50);                    // a command
```

The extended layer's signals sit at `MAX_SIG + CONFIG_ROBOTICS_SIG_OFFSET` (1100 by default), above an app's own. The answers: `ROBOTICS_STEPPER_BEGIN_RSP_SIG`, `_STEPPER_END_`, `_STEPPER_RESET_POSITION_`, `_OUTPUTS_BEGIN_`, `_OUTPUTS_END_`, `_I2C_LEND_`, `_I2C_RETURN_`, `_I2C_SCAN_RSP_SIG`. The news: `ROBOTICS_MODULE_FOUND_SIG`, `_MODULE_LOST_`, `_WRONG_MODULE_`, `_SETTING_LOST_`, `_SETTING_RESTORED_`, `_FAULT_TRIPPED_`, `_STEPPER_DONE_`, `_WATCHDOG_EXPIRED_SIG`. An answer is a `robotics_sys_answer_t` (`rc`, and `why`, which `robotics_sys_why_text()` says in words); a lend and a scan add their fields. A request holds one **door** per class (stepper, outputs, I2C) for its asker, up to `ROBOTICS_ASKERS_MAX` (4) askers: asking again before the answer is refused, and the door is shut again just before the answer is posted, so its handler may ask the next.

## Safety

The module drives motors from the Dot's 5 V. Read this before you wire one.

- **A crash stops nothing.** A panic, an assertion or the task watchdog restarts the chip and leaves its pins as they were: a motor goes on turning, a stepper's coils stay on and the servo is still held, until kodeOS starts again and lets the header's pins go.
- **The watchdog is off until the app sets it.** `robotics_watchdog_set(5000)` turns everything off when the app asks nothing for 5 s. It covers an app that hangs, not one that crashes: it runs in the same firmware.
- **Leaving with `kode_exit()` puts the bridge to sleep and the servo limp, and nothing more.** The motors coast and the servo's pulses stop. Call `robotics_stop()` first. The bottom button held goes home to kodeOS and stops everything by itself, and nothing that moves is taken again for the 3 s the board may take to go.
- **What is left on stays on.** The four outputs keep their duty, a servo holds its angle and a stepper begun with `ROBOTICS_HOLD_FOREVER` keeps its coils on, until `robotics_stop()`, the watchdog or the bottom button.
- **The current is limited, and the SDK cannot read it.** The bridge chops each channel at about 400 mA by itself. A stalled motor is not a fault: the SDK cannot tell it. A short or a chip too hot stops everything and latches until `robotics_fault_clear()`.
- **The servo connector has no current limit.** It is for one micro servo (SG90 class). A standard servo needs a supply of its own.
- **Power comes from the Dot.** The module has no supply of its own: the motors and the servo take the Dot's 5 V, 2 A in all. Run them with the Dot on a USB-C charger (5 V, 3 A), not on a computer's port; on battery the Dot gives 0.5 A, less than one stalled motor.
- **A load on the outputs goes to the module's own +5V**, never to a higher supply. The free lines are 3.3 V: the Dot's pins are not 5 V tolerant.
- **Change modules with the Dot off**, or wait for `module_lost` before plugging in another.

## Four habits

Every example here has them. They keep an app that hangs, or a person who walks away, from leaving a motor running; they do not cover a crash, which stops this SDK with the app ([Safety](#safety) says what is left then).

1. **Set the watchdog when the module is found, and feed it.** It is off until the app sets it. `robotics_watchdog_set(5000)` at `module_found` (it is accepted before the module is there, and kept while it is gone), then any call that reaches the SDK feeds it; an app with nothing to ask touches it once a second. When it runs out, everything is off and the event `watchdog_expired` says so.
2. **Nothing moves by itself.** The SDK moves nothing until the app asks: not when the module comes back, not when the board wakes. An example starts moving on the top button, not at boot.
3. **A call that cannot be done says so and changes nothing.** No module, a fault, a value out of range, J402 somebody else's: the standard layer answers `false` and `robotics_error()` says why; the extended layer answers `KODE_DENIED` (or `KODE_NO_RESOURCES`: try again). Nothing is ever cut to fit.
4. **A task may wait, an Active Object never.** `robotics.h` is refused from any Active Object's task (all but `robotics_error()`), the app's own or kodeOS's; an AO uses `robotics_sys.h`, whose calls return at once.

The bottom button held goes home to kodeOS and stops everything first. An app that leaves by `kode_exit()` calls `robotics_stop()` before it.

## What it drives

The calls are named for the standard layer (`robotics_<part>_<verb>()`); the extended layer has the same, as `robotics_sys_<part>_<verb>()`.

| Part | Calls | Notes |
|---|---|---|
| Motor A, motor B | `robotics_motor_drive()`, `robotics_motor_brake()`, `robotics_motor_coast()`, `robotics_motor_set_ramp()`, `robotics_motor_set_decay()`, `robotics_motor_set_limit()`, `robotics_motor_set_kick()` | -100..100 %. Every change along a ramp, up and down (500 ms for 0 to 100 % by default). Turning the other way ramps down, brakes 120 ms and ramps up. Slow decay by default - duty x VM, a straight line from 0; fast decay at 20 kHz gives (2 x duty - 1) x VM, nothing below 50 % - and a ceiling below 100 % lowers the stall current. An optional push for a start from still, set per motor (off by default). |
| Tone | `robotics_motor_tone()` | A frequency on one bridge, 100 Hz to 10 kHz, for a while: a speaker on its terminals, or the motor's own winding, sings. Both motors still. |
| Stepper | `robotics_stepper_begin()`, `_move()`, `_move_to()`, `_run()`, `_stop()`, `_halt()`, `_release()`, `_get_position()`, `_reset_position()`, `_wait()`, `_end()` | A bipolar stepper on both bridges: half steps, full steps, or 8 microsteps a full step (each coil's voltage along a sine). Steps from a hardware timer, from internal RAM - with `CONFIG_GPTIMER_ISR_CACHE_SAFE`, `CONFIG_GPTIMER_CTRL_FUNC_IN_IRAM`, `CONFIG_GPIO_CTRL_FUNC_IN_IRAM` and `CONFIG_MCPWM_CTRL_FUNC_IN_IRAM` in the app's sdkconfig, as `examples/stepper` sets them, they go on while the flash is written; without them the steps pause during a flash write, and the log says so. Acceleration planned every 10 ms. A move asked while moving is taken on the way, or the stepper stops and comes back. Coils off after a hold time. |
| Four outputs | `robotics_outputs_begin()`, `robotics_output_set()`, `robotics_outputs_end()` | The bridges as four low-side switches with their own PWM: LEDs, a solenoid, a pump, a vibration motor, each from +5V (J402.2) to J503.1..4. |
| Servo | `robotics_servo_angle()`, `_pulse()`, `_set_speed()`, `_set_range()`, `_release()` | The connector J402, `ROBOTICS_SERVO_1`. Its position is taken the first time it is aimed. Limp until the first angle, which it jumps to; then at its speed. |
| I2C pair | `robotics_i2c_lend(&scl, &sda)`, `robotics_i2c_return()`, `robotics_i2c_scan(addresses, capacity, &found)` | J403's SDA and SCL, handed to the app for its own bus; or scanned for the addresses that answer, with a stuck bus told apart. In the extended layer these are requests, answered with `ROBOTICS_I2C_LEND_RSP_SIG` (the GPIOs) and `ROBOTICS_I2C_SCAN_RSP_SIG` (the addresses; `robotics_sys_scan_has()`, `robotics_sys_scan_list()`). |
| Free lines | `kode_pin_*()` (`kode_sys_exp_*()` in an Active Object) on `robotics_header_position(ROBOTICS_BSP_POSITION_J403_2)`, `robotics_header_position(ROBOTICS_BSP_POSITION_J404_2)`..`_5` | kode-sdk's own calls; `robotics_header_position()` gives the J3 position for the way the module sits. |
| The module | `robotics_begin()`, `robotics_end()`, `robotics_is_present()`, `robotics_wait_present()`, `robotics_status()`, `robotics_error()`, `robotics_stop()`, `robotics_fault_clear()`, `robotics_header_position()` (extended: `robotics_sys_status_read()`, no `_wait_present()`) | Found by its fingerprint, upright or turned round, and followed: unplugged, its positions are handed back. Another module on J3 (a Camera, a Mesh, a F*CK) is named in `robotics_status()`. |
| Events, watchdog | `robotics_event_add_listener()`, `_remove_listener()`, `robotics_event_wait()`; extended: `robotics_sys_subscribe()`. `robotics_watchdog_set()` | Eight events, by name or signal: `module_found`, `module_lost`, `wrong_module`, `setting_lost`, `setting_restored`, `fault_tripped`, `stepper_done`, `watchdog_expired`. The watchdog stops everything when the app asks nothing for a while; it is off until the app sets it. |

## How it works

`robotics_begin()` (or `robotics_sys_begin()`) asks for J3's supply on and starts one Active Object, the **owner** (`AO_Robotics`, of kode-sdk's own kind: `kode_sys.h`). It lends J3's positions 5..10 - the bridge's lines, whichever way the module sits - and keeps them for good, so that no hand-back ever pulses a bridge input. Every half second it reads the module's fingerprint against the P4's pull-downs - seven lines its pull resistors hold in a pattern nothing else has, three readings in a row - both ways round, since J401 fits J3 turned by 180 degrees as well. When it matches, the owner takes the I2C pair and the bridge. From then on it is the only code that drives the module, and it runs every 10 ms: the motors' ramps, the stepper's speed, the servo's approach, the fault check and the check that the module is still there.

The calls of both layers ask the owner. A task's call waits for the owner's verdict, briefly (the owner runs every 10 ms and the call is answered from its next step); an Active Object's command posts and returns, and its request is answered with a signal. What takes the owner longer than a step - the I2C scan, some 30 ms - runs on the owner's worker, so the motors keep moving. A task is told what happened by name, on a task of the SDK's own (`robotics_events`, stack 3072 bytes, at kode-sdk's `KODE_PRIO_STANDARD_EVENTS`, below every Active Object); an Active Object, with a signal posted to its queue with a margin, so news that does not fit is lost and counted (`robotics_status().news_dropped`), never the assertion. Holding the bottom button - kodeOS's way home - stops everything before the board resets.

The bridge wakes by itself when something is asked to move, and sleeps half a second after the last thing driven stops. Its nFAULT line falls for an over-current (a short, 2.2-3 A) or an over-temperature, and its falling edges are counted by the PCNT, so the 1.9 ms blinks of a short are not missed between two looks. Flagged for 200 ms while something is driven, everything coasts and the fault latches: every call that would move something is refused until `robotics_fault_clear()`. `robotics_status()` says which it was: a line that blinked is a short, one that fell and stayed is the chip too hot. `robotics_stop()` goes round the owner's queue, so it is never lost to a full one.

## Things that surprise

- **Power.** The module has no supply of its own: the motors and the servo take the Dot's 5 V, 2 A in all. Run them with the Dot on a USB-C charger (5 V, 3 A), not a computer's port. The SDK never turns `kode_pin_power_5v()` on, and an app should not for the motors: on battery it gives 0.5 A, less than one stalled motor. The servo connector is for one micro servo (SG90 class).
- **The current is limited, and the SDK cannot read it.** R503/R504 (0.5 ohm) make the AT8833 chop each channel at about 400 mA. Nothing on the module brings the current to the Dot, and a stalled motor is not a fault: the chopper holds it at its limit and nFAULT stays high, so the SDK cannot tell a stall. `robotics_motor_set_limit()` lowers the ceiling.
- **A geared motor needs more to start than to keep turning.** A small TT gear motor at 5 V can stay still from rest at 30 % and keep turning down to 25 % once it moves: static friction, not the SDK. The number belongs to each motor, so the SDK has none: with `robotics_motor_set_kick(motor, kick_pct, duration_ms)`, a start from still to a speed below `kick_pct` gets `kick_pct` for `duration_ms` first, then the speed asked. An app sets it for its own motor. Off by default.
- **Microsteps are voltage microsteps.** The AT8833 has no current reference, so `ROBOTICS_STEP_MICRO` sets each coil's voltage, not its current. `micro_pct` near 0.4 A x the coil's ohms / 5 V keeps the sine under the chopper's limit; at 100 % a low-ohm motor gets its tops cut flat at ~400 mA.
- **Either way round.** The module works plugged in upright or turned by 180 degrees; `robotics_status()` says which. The free lines are named by their connector pin, and `robotics_header_position()` turns them into J3 positions.
- **Two motors, one stepper or four outputs.** The AT8833's two bridges drive motor A and motor B; or, between `robotics_stepper_begin()` and `robotics_stepper_end()`, the stepper's two coils; or, between `robotics_outputs_begin()` and `robotics_outputs_end()`, four loads to +5V. A load on the outputs must go to the module's own +5V, never to a higher supply.
- **A tone plays on one bridge while both motors are still.** It has a timer of its own, but over a turning motor it would not be heard; a motor asked to move ends it.
- **One module's SDK per app.** `robotics_begin()` holds positions 5..10 for as long as the app runs, module or not, and looks at the I2C pairs every half second while no Robotics is there. An app that drives another module on J3 should not start this SDK.
- **Change modules with the Dot off**, or wait for `module_lost` (2 s at the least) before plugging in another: one plugged in faster, with an I2C pair where the Robotics has its, would be taken for the Robotics still there.
- **Positions 5..10 are the SDK's** from `robotics_begin()` on, module or not. The I2C pair and the free lines of both orientations are lent only for a look while the module is absent; once found, the free lines are the app's. The servo connector is lent the first time the servo is aimed.
- **With its pins let go, the bridge is awake, not asleep.** nSLEEP has a pull-up on the module (R502), so a bridge nobody drives coasts with its inputs on their pull-downs. The SDK holds nSLEEP low while nothing moves.
- **`kode_pin_write_pwm()` keeps all its outputs.** The motors, the tone and the servo run on the P4's MCPWM, group 1, with exact 0 % and 100 %; the LEDC stays kode-sdk's. An app that wants MCPWM of its own takes group 0. `robotics_bsp.h` explains the plan.

## The examples

One ESP-IDF project per part, each an app for kodeOS with a header that says what it teaches, what to wire, what to try and the rule it keeps. Three use the standard layer and four the extended one, so there is one of each kind to copy from. A task is one file (`<name>_main.c`, `app_main()` and a few functions); an Active Object is `<name>_main.c` that starts it plus `<name>_ao.c/.h`. Nothing moves until a person presses the top button.

| Example | Layer | What it shows | Wire |
|---|---|---|---|
| [`examples/module`](../examples/module) | Standard | The module found on J3, which way round it sits and what the owner knows of it (`robotics_status()`), told by listeners when it leaves or trips. Drives nothing: run it first. | The module on J3 |
| [`examples/console_actions`](../examples/console_actions) | Extended | The module on kodeOS's console: nine functions and six variables, and the news on the log | Whatever the rows are used for |
| [`examples/motors`](../examples/motors) | Standard | Two DC motors in one sequence of calls and `kode_delay()`: a tone, then forward, reverse, brake and coast | A small DC motor (N20, TT) on MOTOR A and another on MOTOR B |
| [`examples/stepper`](../examples/stepper) | Standard | A bipolar stepper, by moves and by speed, waiting for each move with `robotics_stepper_wait()` | One coil on MOTOR A, the other on MOTOR B (NEMA 17 up to ~400 mA a phase) |
| [`examples/servos`](../examples/servos) | Extended | The servo on J402 as an Active Object: commands and a `QTimeEvt` for the pauses, the module's news as signals | A micro servo (SG90, MG90S) on J402 |
| [`examples/outputs`](../examples/outputs) | Extended | The two bridges as four switched outputs: a request (`robotics_sys_outputs_begin()`) and its answer, then a command a tick | Up to four loads from +5V (J402.2) to J503.1..4 |
| [`examples/i2c`](../examples/i2c) | Extended | The I2C pair: scan, lend and return as three requests in a chain, the app's own bus on a worker | A 3.3 V I2C device on J403 |

Run the ones that drive something with the Dot on a USB-C charger (5 V, 3 A); `module` and `i2c` drive nothing, and a computer's port is enough for them. Their `sdkconfig.defaults` set `CONFIG_KODEOS_RADIO=y` with Wi-Fi off, and leave Bluetooth off, as kode-sdk has it for an app. Copy those lines: an app of the module needs the radio in the image, because the module's supply is switched over the radio's link.

## Options

| Kconfig | Default | What it does |
|---|---|---|
| `CONFIG_ROBOTICS_AO_PRIORITY` | 0 (`KODE_PRIO_USER_HIGH`, 16) | The QP/C priority of `AO_Robotics`. Two Active Objects at one level stop the board when the second starts: an app whose own AO needs the high level moves this one. `robotics_ao.c` says why it sits there. With `CONFIG_KODE_SCHEDULER`, set it: 0 then leaves the owner no level. Level 3 is kode-sdk's standard layer's and is refused too. |
| `CONFIG_ROBOTICS_SIG_OFFSET` | 1100 | Where `robotics_sys.h`'s 16 signals start above `MAX_SIG` (1000..60000). An app's own signals are `MAX_SIG` to `MAX_SIG + 999`; the module SDKs take 1000, 1100, 1200 and 1300 so an app that uses two has no signal twice. |
| `CONFIG_KODEOS_RADIO` | `y` in every example | J3's supply is switched by the C5 over the radio link. Without the radio in the image the SDK still builds, but J3 is always powered (`power` says `ROBOTICS_POWER_ALWAYS`) and `robotics_end()` cannot switch it off. |
| `CONFIG_KODE_LOWPOWER` | off | kode-sdk's low power. With it, the owner turns everything off when the board goes to sleep and starts nothing until it is awake. No example sets it; `tests/target` builds it as a variant (below). kode-sdk v0.1.5 compiles it only with Bluetooth and its console on, as `tests/target/sdkconfig.lowpower` sets them. |

## The console

[`examples/console_actions`](../examples/console_actions) puts the module on kodeOS's console as rows a person or a script can type: `function name="rob_motor_a" arg=60`, `variable name="rob_motors" watch=200`. The rows call the extended layer, and an Active Object subscribed to the module's news logs what it does. As in kode-sdk, the rows are the app's: one that wants them copies the example's tables, or the rows it needs. They keep a short `rob_` prefix: a row's name has 15 characters at most and shares one name space with the app's own rows.

| Functions | Variables |
|---|---|
| `rob_stop`, `rob_fault_clear`, `rob_motor_a`, `rob_motor_b`, `rob_brake`, `rob_servo`, `rob_step_run`, `rob_step_move`, `rob_watchdog` | `rob_present`, `rob_orientation`, `rob_fault`, `rob_module`, `rob_motors`, `rob_position` |

## What it costs an app

One Active Object (`AO_Robotics`, its 8 KB stack in internal RAM, its queue of 16 events), one worker (`robotics_work`, its stack in PSRAM, for the I2C scan), one small task for the standard layer's events (`robotics_events`, 3 KB of stack, made when the owner starts), J3's positions 5..10 for as long as the app runs, and MCPWM group 1's three timers and three operators, named in `robotics_bsp.h`, a pulse counter unit for nFAULT's edges, a GPTimer while the stepper is begun, and an I2C controller booked for the pair under the module's name. `tools/check_ram.sh` holds the whole image - kode-sdk and the module - to `tests/target/ram_budget.txt`, which says how much of the P4's 640 KB of internal RAM the image takes at link time. Nearly all of it is kode-sdk's; the module's own statics come to about 11 KB.

## Layout

The tree is kode-sdk's: one component per layer, each part in a folder of its own with its header in `include/` and its source in `src/`.

| Path | What it holds |
|---|---|
| `components/robotics` | The standard layer (`kode` in kode-sdk): `include/robotics.h`, what a task calls, and `src/robotics.c`. |
| `components/robotics_sys` | The extended layer (`kode_sys` in kode-sdk): `include/robotics_sys.h`, what an Active Object calls, and `src/robotics_sys.c` (the doors, the subscribers, the signals). Its Kconfig is `CONFIG_ROBOTICS_SIG_OFFSET`. |
| `components/robotics_aos` | The owner (`kaos` in kode-sdk), serving both layers: what they include in `core/include` (`robotics_aos.h`, `robotics_check.h`), and the one Active Object in `robotics/src` - `AO_Robotics` (`robotics_ao.c`), what its signals do (`robotics_aos.c`), the events for tasks (`robotics_events.c`), and in plain C its arithmetic (`robotics_plan.c`), which request the state allows (`robotics_check.c`) and the words for a reason (`robotics_why.c`). |
| `components/robotics_drv` | The drivers (`kdrv` in kode-sdk), a folder each: `bridge`, `stepper` (the step pulses), `servo_pwm`, `i2c_scan`, `pins`, and `at8833`, the chip's truth table in plain C. |
| `components/robotics_bsp` | The module described (`kbsp` in kode-sdk), in `board/include`: J3 positions, timings, the PWM plan, and the types both layers share (`robotics_types.h`). Only describes. |
| `examples/` | One project per part. Standard layer: `module` (presence, status and events: run it first), `motors` (with a tone) and `stepper`. Extended layer: `console_actions`, `servos` (the one on J402), `outputs` and `i2c`. Each says what to wire. |
| `tests/host` | The rules that are plain C, with Unity and nothing else, built for ESP-IDF's `linux` target: `robotics_plan`, `robotics_check`, `robotics_why` and `robotics_drv_at8833`. |
| `tests/host_drv` | `robotics_drv_stepper.c`, the stepper's alarm, over a timer the cases fire by hand: the phase each step writes, the count, a move that ends on its own last step, a retarget that loses no step. `robotics_drv_bridge.c` and `robotics_drv_servo_pwm.c` over a faked MCPWM that says what each pin does across a period: exact 0 % and 100 %, the servo's pulse, a tone and the way into it and out. And `robotics_drv_i2c_scan.c` over a faked I2C bus. |
| `tests/host_owner` | The real owner (`robotics_aos.c`) on FreeRTOS for the `linux` target, over faked drivers and a stand-in for `AO_Robotics` behind `robotics_ao.h` (`main/fakes/`): the module found, gone or the wrong one, the ramps, faults, the watchdog, the stepper, the servo, the outputs, the I2C pair, sleep and home, the answers and the news of both layers. |
| `tests/host_ao` | `AO_Robotics` itself on kode-sdk's real QP/C and worker, at the pinned release (a clone beside it, in `.gitignore`): its queue's and pool's margins, the tick, the signals the system publishes, the worker's answer. |
| `tests/host_standard` | `robotics.c` over an owner that writes down what it is asked (`main/fakes/`): a case for every call and every refusal of the standard layer, and the call made in any state. |
| `tests/host_sys` | `robotics_sys.c` on kode-sdk's real QP/C over that same kind of owner: the commands, the requests with their doors and answers, the news, the immediates, the pool and the queues run dry, and the call made in any state. |
| `tests/host_console` | `examples/console_actions` over the real extended layer and that owner, its tables held to kodeOS's registry rules: every row taken, every function posting what its name says, every variable a word the console sends, and the news it logs. |
| `tests/target` | The SDK built for the esp32p4 against the pinned `kode-sdk`, every call of `robotics.h` and `robotics_sys.h` named in `ALL[]` and linked, and its RAM budget (`ram_budget.txt`). Its `lowpower` variant is the same build with `CONFIG_KODE_LOWPOWER=y`. |
| `tests/sanitize.cmake` | ASan and UBSan for the host suites, under `tools/host_tests.sh --sanitize`. |
| `tools/` | `host_tests.sh`, `check_layers.sh` (thirteen rules), `check_ram.sh`, `check_example_pins.sh`, and the checks' own tests. `check_misra.sh` (cppcheck's MISRA C check, held to a baseline file given to it) and `build_cppcheck.sh` (the cppcheck that baseline names). |

## Building and checking

The SDK builds with ESP-IDF v6.1 against kode-sdk v0.1.5. The components never pin kode-sdk: the app does. The first configure of a project fetches kode-sdk over https, so it needs the network.

```sh
. <path-to-esp-idf>/export.sh
tools/host_tests.sh                    # the host suites; --sanitize for ASan and UBSan
tools/check_layers.sh
tools/check_example_pins.sh
idf.py -C tests/target build           # a few minutes
tools/check_ram.sh tests/target/build tests/target/ram_budget.txt
ROBOTICS_TARGET_VARIANT=lowpower idf.py -C tests/target -B tests/target/build_lowpower build
```
