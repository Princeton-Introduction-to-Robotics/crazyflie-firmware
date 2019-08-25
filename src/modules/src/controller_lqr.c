#include "controller_lqr.h"
#include "param.h"
#include "log.h"
#include "math3d.h"
#include "num.h"
#include "eeprom.h"
#include "console.h"

#include <string.h>
#include <math.h>

static const float MASS    = 0.03f;     // kg
static const float GRAVITY = 9.81f;     // m / s^2
static const float K_F     = 1.938952e-6f; // N / "PWM"
static const float K_M     = 4.760115E-08f; // Nm / "PWM"
static const float L       = 0.046f;    // m


static float K[4][12];

static float state_vec[12];
static float setpt_vec[12];
static float state_error_vec[12];
static float input_vec[4];
static float pwm_vec[4];
static int16_t pwm_total;
static int16_t pwm_int[3];
static float force_total;
static float thrust;
static bool saved_state = false;

/**
* Converts forces to "PWM" units.
*
* This function converts from forces (specified in Newtons) to PWM units, which
* are proportional to motor RPM ^ 2, and ranges from 0 to 65536. The PWM units
* corresponds to the motor duty cycle.
*
* Essentially, this function is half of the inversion of (2) in [1].
* Specifically, the entries of `pwm_vec` returned are those of `control_vec`
* multiplied by the coefficient from the inversion of (2). The actual summing
* of these values is done in lines 84-90 of `power_distribution_stock.c`.
*
* [1]: "Minimum  Snap  Trajectory  Generation  and  Control  for  Quadrotor"
*      by Mellinger and Kumar.
*
* @param[in] control_vec The desired inputs in Newtons and Newton-meters. Order
*            is the same as in [1].
*
* @param[out] pwm_vec The inputs converted into PWM units as described above.
*/
static inline void forcesToPwm(float *control_vec, float *pwm_vec) {
  // May need to negate pwm_vec[1], pwm_vec[2]. Sums in
  // power_distribution_stock.c seem to have negatives cf. Mellinger paper.
  pwm_vec[0] = control_vec[0] / (4 * K_F);
  pwm_vec[1] = sqrtf(2) * control_vec[1] / (4 * L * K_F);
  pwm_vec[2] = sqrtf(2) * control_vec[2] / (4 * L * K_F);
  pwm_vec[3] = control_vec[3] / (4 * K_M);
}

static inline int16_t saturateSignedInt16(float in)
{
  // Don't use INT16_MIN, because later we may negate it,
  // which won't work for that value.
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t) in;
}


/**
* Initialize the LQR controller.
*
* Called once when switching to the the LQR controller. Useful for initializing
* static variables used by the module.
*/
void controllerLQRInit(void)
{
  saved_state = false;
  consolePrintf("butts\n");
}

/**
* Test the LQR controller initialization.
*
* Called by the system to check that the controller is properly initialized.
*/
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

  //Depending on the setpoint types, we zero out certain enties here.
  if (setpoint->mode.x == modeVelocity) {
    setpt_vec[0] = 0.0f;
  }

  if (setpoint->mode.y == modeVelocity) {
    setpt_vec[1] = 0.0f;
  }

  if (setpoint->mode.z == modeVelocity) {
    setpt_vec[2] = 0.0f;
  }

  if (setpoint->mode.roll == modeVelocity) {
    setpt_vec[3] = 0.0f;
  }

  if (setpoint->mode.pitch == modeVelocity) {
    setpt_vec[4] = 0.0f;
  }

  if (setpoint->mode.yaw == modeVelocity) {
    setpt_vec[5] = 0.0f;
  }

  if (!saved_state) {
    saved_state = true;

    union {
      float f;
      uint8_t bytes[4];
    } dummy;

    for (int i = 0; i < 12; i++) {
      dummy.f = state_error_vec[i];
      uint8_t *ptr = &(dummy.bytes[0]);
      eepromWriteBuffer(ptr, 8182 + i, sizeof(uint8_t) * 4);
    }
  }

  // Matrix multiplication!
  for (int i = 0; i < 4; i++){
    input_vec[i] = 0;

    for (int j = 0; j < 12; j++) {
      input_vec[i] += K[i][j] * state_error_vec[j];
    }
  }

  input_vec[0] += setpoint->thrust + MASS * GRAVITY;
  force_total = input_vec[0];
  forcesToPwm(input_vec, pwm_vec);

  control->thrust = pwm_vec[0];
  control->roll   = saturateSignedInt16(2 * pwm_vec[1]);
  control->pitch  = saturateSignedInt16(2 * pwm_vec[2]);
  control->yaw    = saturateSignedInt16(pwm_vec[3]);

  pwm_int[0] = control->roll;
  pwm_int[1] = control->pitch;
  pwm_int[2] = control->yaw;

  //memset(control, 0, sizeof(control_t));

  pwm_total = (int16_t) (force_total / (4 * K_F));
  thrust = setpoint->thrust;
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

LOG_ADD(LOG_INT16, u2_pwm,   &pwm_int[0])
LOG_ADD(LOG_INT16, u3_pwm,   &pwm_int[1])
LOG_ADD(LOG_INT16, u4_pwm,   &pwm_int[2])



LOG_ADD(LOG_FLOAT, thrust, &thrust)
LOG_GROUP_STOP(ctrlLQR)
