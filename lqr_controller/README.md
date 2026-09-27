# LQR controller (out-of-tree app) for MAE 345/549 Lab 2

This is a port of the course's 2019 `src/modules/src/controller_lqr.c` (on `master`
in this repo) to the out-of-tree (OOT) controller API of current bitcraze
firmware. It builds for the Crazyflie 2.1 Brushless (`cf21bl`) and for the brushed
Crazyflie 2.x (`cf2`).

What stays the same as in 2019:

- The control law `u = K (x - x_ref) + [m g, 0, 0, 0]`, with the same state
  ordering and sign conventions as `quad_utils`.
- The reference: x, y and yaw are 0 (the point where the Kalman filter was reset),
  and z comes from the setpoint.
- The parameters `ctrlLQR.k11` … `ctrlLQR.k412`.
- The log variables `ctrlLQR.e_*`, `u1`–`u4` and `u2_pwm`–`u4_pwm`.

What changed:

- **The controller number is now 6 (`ControllerTypeOot`), not 3.** In current
  firmware, 3 is INDI.
- `u` goes to the firmware in SI units (`controlModeForceTorque`). The platform's
  power distribution then maps it to motors using the right arm length and thrust
  constants for brushed or brushless. The `ctrlLQR.k_f` / `ctrlLQR.k_m` parameters
  are gone.
- The hover feed-forward mass is the parameter `ctrlLQR.mass`. It defaults to the
  platform's `CF_MASS`. Weigh the drone with its decks and battery, then set it.
- A stop setpoint (`send_stop_setpoint`) cuts the motors while LQR is active.
- `setpoint->thrust` is no longer added to `u1`. It's 0 for hover/position
  setpoints anyway, and it's in PWM units, not newtons.

## Build

You need the ARM toolchain (`brew install --cask gcc-arm-embedded` on macOS) and
the submodules (`git submodule update --init --recursive`). Then:

```
cd lqr_controller
make cf21bl_defconfig      # or cf2_defconfig for the brushed Crazyflie 2.x
make -j
```

The binary ends up in `lqr_controller/build/` (`cf21bl.bin` or `cf2.bin`).

## Flash

### 1. Install the official brushless release (once per drone)

This installs current firmware on both chips (STM32 and nRF51), so the drone flies
on stock firmware before you add the LQR build. It also recovers a drone that had
the old 2019 `cf2.bin` flashed onto it.

1. Update cfclient: `pip install --upgrade cfclient cflib`. Versions from 2024 and
   earlier don't list the brushless platform. Newer versions may need Python 3.10
   or later; if pip won't upgrade in a 3.9 environment, make a new venv with a
   newer Python.
2. Plug in the Crazyradio, run `cfclient`, and open **Connect → Bootloader**. Don't
   connect to the drone first.
3. Put the drone in bootloader mode. With it off, hold the power button about 3 s
   until the blue LEDs blink, then let go. Alternatively, use "Cold boot
   (recovery)" in the dialog.
4. Pick the latest release for **Crazyflie 2.1 Brushless (cf21bl)**, not
   "Crazyflie 2.1", and click **Program**. Keep other powered-on drones away.
5. After it restarts, connect in cfclient and check that it arms and hovers
   normally.

### 2. Flash the LQR build

Put the drone in bootloader mode again and run:

```
python3 -m cfloader flash build/cf21bl.bin stm32-fw
```

This replaces only the STM32 firmware and keeps the nRF51 firmware from step 1.
`make cload` in this folder also works if the radio is set up.

## Lab 2 notebook changes

- `cf.param.set_value('stabilizer.controller', '3')` → `'6'`.
- The brushless drone won't spin its motors until it's armed. Arm it once after
  connecting: `cf.platform.send_arming_request(True)`.
- Update the system parameters (below) and retune Q/R before trusting the
  simulation.

## System parameters

### Python model (`F2024-dev/quad_utils/quadrotor.py`, class `Crazyflie`)

This is what K is designed from. Its values describe the brushed Crazyflie 2.1.
Rather than editing the class in place, add a `CrazyflieBrushless(Crazyflie)`
subclass that overrides these properties, so the other labs and the brushed drones
keep working.

| Parameter | Brushed 2.1 (current) | Brushless 2.1 |
|---|---|---|
| inertia `_I` (kg m²) | diag(1.4194e-5, 1.4089e-5, 2.9741e-5) | not measured yet, see below |
| `mass` (kg) | 0.03 | weigh it with flow deck and battery (`CF_MASS` default: 0.0393) |
| `max_thrust` | 60 × 9.8 | not used anywhere; 4 × 0.2 N = 0.8 N (≈ 82 g) if you want it accurate |
| `lift_coeff` kf (N per motor command) | 1.938952e-6 | 0.2 / 65535 ≈ 3.05e-6 |
| `drag_coeff` km (Nm per motor command) | 4.760115e-8 | kf × 0.00569 ≈ 1.74e-8 |
| `arm_length` (m) | 0.046 | 0.050 |

- `mass` and `_I` go into the A and B matrices, so they change K. These are the
  ones to get right.
- `lift_coeff`, `drag_coeff` and `arm_length` are only used by `clipped_controller`
  to model motor saturation in the simulation. The brushless values come from the
  firmware's `THRUST_MAX` (0.2 N per motor, full scale 65535) and `THRUST2TORQUE`
  (0.00569 m), so simulated saturation matches the firmware.
- Where the brushless numbers come from:
  - Read directly from `platform_defaults_cf21bl.h` (release 2026.08): `CF_MASS`,
    `ARM_LENGTH`, `THRUST_MAX` and `THRUST2TORQUE`. The last two are the defaults
    with `CONFIG_ENABLE_THRUST_BAT_COMPENSATED=y`, which is on by default.
  - Derived from how the firmware's power distribution works: kf =
    `THRUST_MAX` / 65535 and km = kf × `THRUST2TORQUE`.
  - `CF_MASS` (39.3 g) is Bitcraze's figure for the brushless with propeller
    guards, 350 mAh battery and Lighthouse deck, not our flow-deck setup, so weigh
    yours.
- The brushed `drag_coeff` gives km/kf = 0.0245 m, about 3.5× Bitcraze's brushed
  `THRUST2TORQUE`. The 2019 firmware converted torque with its own km, so it was
  self-consistent. The new firmware uses `THRUST2TORQUE`, so use kf ×
  `THRUST2TORQUE` in the simulation. This only affects simulated yaw saturation.
- There's no reliable published inertia for the brushless drone yet. Check Bitcraze's
  docs or forum, or estimate it. It's heavier with a longer arm, so expect somewhat
  larger values than the brushed numbers.

### Firmware

- **Hover mass.** This is the only physical parameter the controller itself uses,
  for the `m g` feed-forward term. It defaults to `CF_MASS` (`src/lqr_controller.c`).
  To change it at runtime without reflashing, set it next to where the notebook
  loads the gains:
  ```python
  cf.param.set_value('ctrlLQR.mass', '0.041')
  ```
- **Motor and geometry constants.** `ARM_LENGTH`, `THRUST_MAX` and `THRUST2TORQUE`
  in `src/platform/interface/platform_defaults_cf21bl.h` are Bitcraze's own
  measurements for this drone. The firmware's power distribution uses them to turn
  force and torque into motor commands. Leave them alone.
