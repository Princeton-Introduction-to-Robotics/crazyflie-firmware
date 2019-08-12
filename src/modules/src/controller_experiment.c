#include "controller_experiment.h"
#include "param.h"
#include "log.h"
#include "math3d.h"

#include <string.h>

static float thrust;

void controllerExperimentInit(void)
{
  thrust = 0.0f;
}

bool controllerExperimentTest(void)
{
  return true;
}

void controllerExperiment(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }

  control->thrust = 65535 * thrust;
  control->roll   = 0;
  control->pitch  = 0;
  control->yaw    = 0;
}


PARAM_GROUP_START(ctrlExp)
PARAM_ADD(PARAM_FLOAT, thrust, &thrust)
PARAM_GROUP_STOP(ctrlExp)
