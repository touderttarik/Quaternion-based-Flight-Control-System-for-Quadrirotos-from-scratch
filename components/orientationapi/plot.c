#include "plot.h"

#include <math.h>
#include <stdio.h>

void plot_init(plot_state_t *state, int decimation, float rad_to_deg) {
    if (!state) {
        return;
    }
    state->decimation = decimation > 0 ? decimation : 1;
    state->counter = 0;
    state->rad_to_deg = rad_to_deg;

    printf("PLOT_G_HEADER,va_x[u],va_y[u],va_z[u],va_hat_x[u],va_hat_y[u],va_hat_z[u]\n");
    printf("PLOT_BIAS_HEADER,b_hat_x[rad/s],b_hat_y[rad/s],b_hat_z[rad/s],b_hat_dot_x[rad/s^2],b_hat_dot_y[rad/s^2],b_hat_dot_z[rad/s^2]\n");
    printf("PLOT_Q_HEADER,q_w[u],q_x[u],q_y[u],q_z[u],q_norm[u]\n");
    printf("PLOT_SENS_HEADER,accel_x[g],accel_y[g],accel_z[g],gyro_x[rad/s],gyro_y[rad/s],gyro_z[rad/s]\n");
    printf("PLOT_ERR_HEADER,err_angle_deg[deg],omega_mes_x[arb],omega_mes_y[arb],omega_mes_z[arb]\n");
}

void plot_emit(plot_state_t *state, const mahony_t *mahony, const sensor_data_t *sensors) {
    if (!state || !mahony || !sensors) {
        return;
    }
    if ((state->counter++ % state->decimation) != 0) {
        return;
    }

    float q_norm = sqrtf(
        mahony->q_hat.w * mahony->q_hat.w +
        mahony->q_hat.x * mahony->q_hat.x +
        mahony->q_hat.y * mahony->q_hat.y +
        mahony->q_hat.z * mahony->q_hat.z);

    float va_dot = mahony->va.x * mahony->va_hat.x +
                   mahony->va.y * mahony->va_hat.y +
                   mahony->va.z * mahony->va_hat.z;
    if (va_dot > 1.0f) va_dot = 1.0f;
    if (va_dot < -1.0f) va_dot = -1.0f;

    float err_angle_deg = acosf(va_dot) * state->rad_to_deg;
    float b_hat_dot_x = mahony->b_hat_dot.x;
    float b_hat_dot_y = mahony->b_hat_dot.y;
    float b_hat_dot_z = mahony->b_hat_dot.z;

    printf("PLOT_G,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
           mahony->va.x, mahony->va.y, mahony->va.z,
           mahony->va_hat.x, mahony->va_hat.y, mahony->va_hat.z);

    printf("PLOT_BIAS,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
           mahony->b_hat.x, mahony->b_hat.y, mahony->b_hat.z,
           b_hat_dot_x, b_hat_dot_y, b_hat_dot_z);

    printf("PLOT_Q,%.6f,%.6f,%.6f,%.6f,%.6f\n",
           mahony->q_hat.w, mahony->q_hat.x, mahony->q_hat.y,
           mahony->q_hat.z, q_norm);

    printf("PLOT_SENS,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
           sensors->accel.x, sensors->accel.y, sensors->accel.z,
           sensors->gyro.x, sensors->gyro.y, sensors->gyro.z);

    printf("PLOT_ERR,%.6f,%.6f,%.6f,%.6f\n",
           err_angle_deg, mahony->omega_mes.x, mahony->omega_mes.y, mahony->omega_mes.z);
}

void plot_ctrl_init(plot_state_t *state, int decimation, float rad_to_deg) {
    if (!state) {
        return;
    }
    state->decimation = decimation > 0 ? decimation : 1;
    state->counter = 0;
    state->rad_to_deg = rad_to_deg;

    printf("PLOT_PID_Q_HEADER,q_err_w[u],q_err_x[u],q_err_y[u],q_err_z[u]\n");
    printf("PLOT_PID_V_HEADER,v_err_x[deg],v_err_y[deg],v_err_z[deg]\n");
    printf("PLOT_PID_RATE_HEADER,omega_sp_x[rad/s],omega_sp_y[rad/s],omega_sp_z[rad/s],omega_err_x[rad/s],omega_err_y[rad/s],omega_err_z[rad/s]\n");
    printf("PLOT_PID_I_HEADER,iomega_err_x[rad],iomega_err_y[rad],iomega_err_z[rad]\n");
    printf("PLOT_PID_U_HEADER,u_x[arb],u_y[arb],u_z[arb]\n");
    printf("PLOT_MIX_HEADER,f1[u],f2[u],f3[u],f4[u]\n");
    printf("PLOT_PWM_HEADER,pwm_1[us],pwm_2[us],pwm_3[us],pwm_4[us]\n");
}

void plot_ctrl_emit(plot_state_t *state, const pid_struct_t *pid, const mix_t *mix) {
    if (!state || !pid || !mix) {
        return;
    }
    if ((state->counter++ % state->decimation) != 0) {
        return;
    }

    float v_err_x_deg = pid->v_err.x * state->rad_to_deg;
    float v_err_y_deg = pid->v_err.y * state->rad_to_deg;
    float v_err_z_deg = pid->v_err.z * state->rad_to_deg;

    printf("PLOT_PID_Q,%.6f,%.6f,%.6f,%.6f\n",
           pid->q_err.w, pid->q_err.x, pid->q_err.y, pid->q_err.z);
    printf("PLOT_PID_V,%.6f,%.6f,%.6f\n",
           v_err_x_deg, v_err_y_deg, v_err_z_deg);
    printf("PLOT_PID_RATE,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
           pid->omega_sp.x, pid->omega_sp.y, pid->omega_sp.z,
           pid->omega_err.x, pid->omega_err.y, pid->omega_err.z);
    printf("PLOT_PID_I,%.6f,%.6f,%.6f\n",
           pid->Iomega_err.x, pid->Iomega_err.y, pid->Iomega_err.z);
    printf("PLOT_PID_U,%.6f,%.6f,%.6f\n",
           pid->u.x, pid->u.y, pid->u.z);
    printf("PLOT_MIX,%.6f,%.6f,%.6f,%.6f\n",
           mix->f1, mix->f2, mix->f3, mix->f4);
    printf("PLOT_PWM,%.0f,%.0f,%.0f,%.0f\n",
           (double)mix->pwm_1, (double)mix->pwm_2,
           (double)mix->pwm_3, (double)mix->pwm_4);
}
