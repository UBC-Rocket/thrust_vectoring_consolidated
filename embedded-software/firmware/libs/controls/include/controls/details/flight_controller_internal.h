#ifndef FLIGHT_CONTROLLER_INTERNAL_H
#define FLIGHT_CONTROLLER_INTERNAL_H

#include "controls/flight_controller.h"

void compute_axis_angle_err(const quaternion_t *q_ref,
                            const quaternion_t *q_meas,
                            float phi[3]);

void update_torque_pid(const float phi[3],
                       const float dt_s,
                       float torque_cmd[3]);

void decompose_torque(const float torque_cmd[3],
                      const float thrust_dir[3],
                      float torque_gimbal[3],
                      float *torque_thrust_mag);

void compute_thrust_dir(const flight_controller_gimbal_config_t *gcfg,
                        const float torque_cmd[3],
                        const float thrust_dir[3],
                        const float thrust_mag,
                        float new_thrust_dir[3]);

void compute_gimbal_angles(const flight_controller_gimbal_config_t *gcfg,
                           const float thrust_dir[3],
                           float *theta_x_cmd,
                           float *theta_y_cmd);

void update_thrust_pid(const flight_controller_thrust_config_t *tcfg,
                       const float z_ref,
                       const float vz_ref,
                       const float z_meas,
                       const float vz_meas,
                       const float dt_s,
                       float *T_cmd);


#endif /* FLIGHT_CONTROLLER_INTERNAL_H */ 
