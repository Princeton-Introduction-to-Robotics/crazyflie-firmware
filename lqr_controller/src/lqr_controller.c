/**
 * lqr_controller.c - Out-of-tree LQR controller for MAE 345/549 Lab 2.
 *
 * Port of the course's 2019 in-tree controller_lqr.c (Princeton fork of
 * crazyflie-firmware 2019.02) to the out-of-tree controller API, so it runs
 * on current firmware and on the Crazyflie 2.1 Brushless (cf21bl) platform.
 *
 * Control law (same as the Python model in quad_utils):
 *
 *   u = K (x - x_ref) + [m g, 0, 0, 0]
 *
 *   x = [x, y, z, phi, theta, psi, vx, vy, vz, p, q, r]
 *   u = [net thrust (N), roll torque, pitch torque, yaw torque (Nm)]
 *
 * The quad_utils model uses a sign convention where y, pitch and pitch rate
 * are flipped relative to the firmware's right-handed body frame. The state
 * mapping below is the one the 2019 firmware used, and the output mapping is
 * the force/torque equivalent of its legacy PWM output. Unlike the 2019
 * version, u is sent to the platform's power distribution in SI units, so
 * motor constants and arm length come from the platform (brushed or
 * brushless) instead of being hard-coded here.
 *
 * Select the controller with stabilizer.controller = 6 (ControllerTypeOot).
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "LQR"
#include "debug.h"

#include "controller.h"
#include "param.h"
#include "log.h"
#include "math3d.h"
#include "physicalConstants.h"
#include "platform_defaults.h"

// The OOT app layer needs an appMain(), but the controller is called from the
// stabilizer loop, so this task only sleeps.
void appMain() {
  DEBUG_PRINT("LQR controller available (stabilizer.controller = %d)\n", ControllerTypeOot);

  while(1) {
    vTaskDelay(M2T(2000));
  }
}

static float K[4][12];
static float mass = CF_MASS;  // kg, override with ctrlLQR.mass if decks change the weight

static float state_vec[12];
static float setpt_vec[12];
static float state_error_vec[12];
static float input_vec[4];

// Per-motor differential commands, in the same "PWM" units the 2019 firmware
// logged (int16, full scale +/- 2^15), so the Lab 2 plotting cell still works.
static int16_t pwm_int[3];

static inline int16_t saturateSignedInt16(float in)
{
  if (in > INT16_MAX) {
    return INT16_MAX;
  } else if (in < -INT16_MAX) {
    return -INT16_MAX;
  } else {
    return (int16_t)in;
  }
}

void controllerOutOfTreeInit(void)
{
  memset(state_error_vec, 0, sizeof(state_error_vec));
  memset(input_vec, 0, sizeof(input_vec));
  memset(pwm_int, 0, sizeof(pwm_int));
}

bool controllerOutOfTreeTest(void)
{
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors,
                         const state_t *state, const stabilizerStep_t stabilizerStep)
{
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
    return;
  }

  control->controlMode = controlModeForceTorque;

  // Stop setpoint: cut the motors.
  if (setpoint->mode.z == modeDisable) {
    control->thrustSi = 0.0f;
    control->torqueX = 0.0f;
    control->torqueY = 0.0f;
    control->torqueZ = 0.0f;
    return;
  }

  state_vec[0]  = state->position.x;
  state_vec[1]  = -state->position.y;
  state_vec[2]  = state->position.z;

  // state->attitude.pitch is already inverted (legacy convention), which is
  // what the model's theta expects.
  state_vec[3]  = radians(state->attitude.roll);
  state_vec[4]  = radians(state->attitude.pitch);
  state_vec[5]  = radians(state->attitude.yaw);

  state_vec[6]  = state->velocity.x;
  state_vec[7]  = -state->velocity.y;
  state_vec[8]  = state->velocity.z;

  state_vec[9]  = radians(sensors->gyro.x);
  state_vec[10] = -radians(sensors->gyro.y);
  state_vec[11] = radians(sensors->gyro.z);

  // Hover about the point where the Kalman filter was reset, at the
  // commanded height.
  memset(setpt_vec, 0, sizeof(setpt_vec));
  setpt_vec[2] = setpoint->position.z;

  for (int i = 0; i < 12; i++) {
    state_error_vec[i] = state_vec[i] - setpt_vec[i];
  }

  for (int i = 0; i < 4; i++) {
    input_vec[i] = 0.0f;

    for (int j = 0; j < 12; j++) {
      input_vec[i] += K[i][j] * state_error_vec[j];
    }
  }

  input_vec[0] += mass * GRAVITY_MAGNITUDE;

  // Model inputs -> right-handed body-frame force/torque.
  control->thrustSi = fmaxf(input_vec[0], 0.0f);
  control->torqueX  = input_vec[1];
  control->torqueY  = -input_vec[2];
  control->torqueZ  = input_vec[3];

  // Same per-motor split as powerDistributionForceTorque(), scaled so that the
  // values match the 2019 firmware's control->roll / pitch / yaw.
  const float arm = 0.707106781f * ARM_LENGTH;
  const float forceToCmd = UINT16_MAX / THRUST_MAX;
  pwm_int[0] = saturateSignedInt16(2.0f * forceToCmd * 0.25f / arm * input_vec[1]);
  pwm_int[1] = saturateSignedInt16(2.0f * forceToCmd * 0.25f / arm * input_vec[2]);
  pwm_int[2] = saturateSignedInt16(-forceToCmd * 0.25f * input_vec[3] / THRUST2TORQUE);
}

PARAM_GROUP_START(ctrlLQR)
PARAM_ADD(PARAM_FLOAT, mass, &mass)

PARAM_ADD(PARAM_FLOAT, k11, &K[0][0])
PARAM_ADD(PARAM_FLOAT, k21, &K[1][0])
PARAM_ADD(PARAM_FLOAT, k31, &K[2][0])
PARAM_ADD(PARAM_FLOAT, k41, &K[3][0])

PARAM_ADD(PARAM_FLOAT, k12, &K[0][1])
PARAM_ADD(PARAM_FLOAT, k22, &K[1][1])
PARAM_ADD(PARAM_FLOAT, k32, &K[2][1])
PARAM_ADD(PARAM_FLOAT, k42, &K[3][1])

PARAM_ADD(PARAM_FLOAT, k13, &K[0][2])
PARAM_ADD(PARAM_FLOAT, k23, &K[1][2])
PARAM_ADD(PARAM_FLOAT, k33, &K[2][2])
PARAM_ADD(PARAM_FLOAT, k43, &K[3][2])

PARAM_ADD(PARAM_FLOAT, k14, &K[0][3])
PARAM_ADD(PARAM_FLOAT, k24, &K[1][3])
PARAM_ADD(PARAM_FLOAT, k34, &K[2][3])
PARAM_ADD(PARAM_FLOAT, k44, &K[3][3])

PARAM_ADD(PARAM_FLOAT, k15, &K[0][4])
PARAM_ADD(PARAM_FLOAT, k25, &K[1][4])
PARAM_ADD(PARAM_FLOAT, k35, &K[2][4])
PARAM_ADD(PARAM_FLOAT, k45, &K[3][4])

PARAM_ADD(PARAM_FLOAT, k16, &K[0][5])
PARAM_ADD(PARAM_FLOAT, k26, &K[1][5])
PARAM_ADD(PARAM_FLOAT, k36, &K[2][5])
PARAM_ADD(PARAM_FLOAT, k46, &K[3][5])

PARAM_ADD(PARAM_FLOAT, k17, &K[0][6])
PARAM_ADD(PARAM_FLOAT, k27, &K[1][6])
PARAM_ADD(PARAM_FLOAT, k37, &K[2][6])
PARAM_ADD(PARAM_FLOAT, k47, &K[3][6])

PARAM_ADD(PARAM_FLOAT, k18, &K[0][7])
PARAM_ADD(PARAM_FLOAT, k28, &K[1][7])
PARAM_ADD(PARAM_FLOAT, k38, &K[2][7])
PARAM_ADD(PARAM_FLOAT, k48, &K[3][7])

PARAM_ADD(PARAM_FLOAT, k19, &K[0][8])
PARAM_ADD(PARAM_FLOAT, k29, &K[1][8])
PARAM_ADD(PARAM_FLOAT, k39, &K[2][8])
PARAM_ADD(PARAM_FLOAT, k49, &K[3][8])

PARAM_ADD(PARAM_FLOAT, k110, &K[0][9])
PARAM_ADD(PARAM_FLOAT, k210, &K[1][9])
PARAM_ADD(PARAM_FLOAT, k310, &K[2][9])
PARAM_ADD(PARAM_FLOAT, k410, &K[3][9])

PARAM_ADD(PARAM_FLOAT, k111, &K[0][10])
PARAM_ADD(PARAM_FLOAT, k211, &K[1][10])
PARAM_ADD(PARAM_FLOAT, k311, &K[2][10])
PARAM_ADD(PARAM_FLOAT, k411, &K[3][10])

PARAM_ADD(PARAM_FLOAT, k112, &K[0][11])
PARAM_ADD(PARAM_FLOAT, k212, &K[1][11])
PARAM_ADD(PARAM_FLOAT, k312, &K[2][11])
PARAM_ADD(PARAM_FLOAT, k412, &K[3][11])
PARAM_GROUP_STOP(ctrlLQR)

LOG_GROUP_START(ctrlLQR)
LOG_ADD(LOG_FLOAT, e_x, &state_error_vec[0])
LOG_ADD(LOG_FLOAT, e_y, &state_error_vec[1])
LOG_ADD(LOG_FLOAT, e_z, &state_error_vec[2])

LOG_ADD(LOG_FLOAT, e_roll,  &state_error_vec[3])
LOG_ADD(LOG_FLOAT, e_pitch, &state_error_vec[4])
LOG_ADD(LOG_FLOAT, e_yaw,   &state_error_vec[5])

LOG_ADD(LOG_FLOAT, e_vx, &state_error_vec[6])
LOG_ADD(LOG_FLOAT, e_vy, &state_error_vec[7])
LOG_ADD(LOG_FLOAT, e_vz, &state_error_vec[8])

LOG_ADD(LOG_FLOAT, e_vroll,  &state_error_vec[9])
LOG_ADD(LOG_FLOAT, e_vpitch, &state_error_vec[10])
LOG_ADD(LOG_FLOAT, e_vyaw,   &state_error_vec[11])

LOG_ADD(LOG_FLOAT, u1, &input_vec[0])
LOG_ADD(LOG_FLOAT, u2, &input_vec[1])
LOG_ADD(LOG_FLOAT, u3, &input_vec[2])
LOG_ADD(LOG_FLOAT, u4, &input_vec[3])

LOG_ADD(LOG_INT16, u2_pwm, &pwm_int[0])
LOG_ADD(LOG_INT16, u3_pwm, &pwm_int[1])
LOG_ADD(LOG_INT16, u4_pwm, &pwm_int[2])
LOG_GROUP_STOP(ctrlLQR)
