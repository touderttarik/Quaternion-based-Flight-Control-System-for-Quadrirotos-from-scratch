#ifndef ORIENTATIONAPI_PLOT_H
#define ORIENTATIONAPI_PLOT_H

#include "mahony.h"

typedef struct {
    int decimation;
    int counter;
    float rad_to_deg;
} plot_state_t;

void plot_init(plot_state_t *state, int decimation, float rad_to_deg);
void plot_emit(plot_state_t *state, const mahony_t *mahony, const sensor_data_t *sensors);

#endif /* ORIENTATIONAPI_PLOT_H */
