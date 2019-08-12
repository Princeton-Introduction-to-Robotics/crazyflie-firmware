#include "controller_lqr.h"
#include "param.h"
#include "log.h"
#include "math3d.h"

#include <string.h>

static float K[4][12];
static float state_vec[12];
static float setpt_vec[12];
static float state_error_vec[12];
static float input_vec[4];
static float pwm_vec[4];

static const float MASS    = 0.03f;     // kg
static const float GRAVITY = 9.81f;     // m / s^2
static const float K_F     = 7.754e-6f; // N / "pwm"
static const float K_M     = 1.938e-7f; // Nm / "pwm"
static const float L       = 0.046f;    // m

void forcesToPwm(float *control_vec, float *pwm_vec) {
  pwm_vec[0] = control_vec[0] / (4 * K_F);
  pwm_vec[1] = control_vec[1] / (2 * L * K_F);
  pwm_vec[2] = control_vec[2] / (2 * L * K_F);
  pwm_vec[3] = control_vec[3] / (4 * K_M);
}

void controllerLQRInit(void)
{

}

bool controllerLQRTest(void)
{
  return true;
}

void controllerLQR(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }

  state_vec[0]  = state->position.x;
  state_vec[1]  = state->position.y;
  state_vec[2]  = state->position.z;

  state_vec[3]  = radians(state->attitude.roll);
  state_vec[4]  = -radians(state->attitude.pitch);
  state_vec[5]  = radians(state->attitude.yaw);

  state_vec[6]  = state->velocity.x;
  state_vec[7]  = state->velocity.y;
  state_vec[8]  = state->velocity.z;

  state_vec[9]  = radians(sensors->gyro.x);
  state_vec[10] = -radians(sensors->gyro.y);
  state_vec[11] = radians(sensors->gyro.z);

  setpt_vec[0]  = setpoint->position.x;
  setpt_vec[1]  = setpoint->position.y;
  setpt_vec[2]  = setpoint->position.z;

  setpt_vec[3]  = radians(setpoint->attitude.roll);
  setpt_vec[4]  = -radians(setpoint->attitude.pitch);
  setpt_vec[5]  = radians(setpoint->attitude.yaw);

  setpt_vec[6]  = setpoint->velocity.x;
  setpt_vec[7]  = setpoint->velocity.y;
  setpt_vec[8]  = setpoint->velocity.z;

  setpt_vec[9]  = radians(setpoint->attitudeRate.roll);
  setpt_vec[10] = -radians(setpoint->attitudeRate.pitch);
  setpt_vec[11] = radians(setpoint->attitudeRate.yaw);

  for (int i = 0; i < 12; i++) {
    state_error_vec[i] = state_vec[i] - setpt_vec[i];
  }

  // Matrix multiplication!
  for (int i = 0; i < 4; i++){
    input_vec[i] = 0;

    for (int j = 0; j < 12; j++) {
      input_vec[i] += K[i][j] * state_error_vec[j];
    }
  }

  input_vec[0] += MASS * GRAVITY;

  forcesToPwm(input_vec, pwm_vec);

  control->thrust = pwm_vec[0];
  control->roll   = (int) pwm_vec[1];
  control->pitch  = (int) pwm_vec[2];
  control->yaw    = (int) pwm_vec[3];
}


PARAM_GROUP_START(ctrlLQR)
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

LOG_ADD(LOG_FLOAT, u1,   &input_vec[0])
LOG_ADD(LOG_FLOAT, u2,   &input_vec[1])
LOG_ADD(LOG_FLOAT, u3,   &input_vec[2])
LOG_ADD(LOG_FLOAT, u4,   &input_vec[3])
LOG_GROUP_STOP(ctrlLQR)
