/**
 * controller_lqr.h - LQR Controller Interface
 */
#ifndef __CONTROLLER_EXPERIMENT_H__
#define __CONTROLLER_EXPERIMENT_H__

#include "stabilizer_types.h"

void controllerExperimentInit(void);
bool controllerExperimentTest(void);
void controllerExperiment(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);

#endif //__CONTROLLER_EXPERIMENT_H__
