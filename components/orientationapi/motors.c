#include "motors.h"
#include "freertos/FreeRTOS.h"
static mcpwm_cmpr_handle_t s_comparator_1 = NULL;
static mcpwm_cmpr_handle_t s_comparator_2 = NULL;
static mcpwm_cmpr_handle_t s_comparator_3 = NULL;
static mcpwm_cmpr_handle_t s_comparator_4 = NULL;

void motors_init(uint8_t motor1_pin, uint8_t motor2_pin, uint8_t motor3_pin, uint8_t motor4_pin){
    //We will synchronize our four motors with a single timer of 400Hz
    mcpwm_timer_config_t timer_config = {
    .group_id = 0,
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    .resolution_hz = 1000000,
    .period_ticks = 2500,
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_new_timer(&timer_config, &timer);



    //We create our two operators
    //An operator is a unit that regroups a comparator and a generator
    mcpwm_operator_config_t operator_config_1 = {
        .group_id = 0,
    };
    mcpwm_oper_handle_t oper_1 = NULL;
    mcpwm_new_operator(&operator_config_1, &oper_1);

    mcpwm_operator_config_t operator_config_2 = {
        .group_id = 0,
    };
    mcpwm_oper_handle_t oper_2 = NULL;
    mcpwm_new_operator(&operator_config_2, &oper_2);

    //Now we connect timer1 to oper1 and timer2 to oper2
    mcpwm_operator_connect_timer(oper_1, timer) ;//for the left motors
    mcpwm_operator_connect_timer(oper_2, timer) ;//for the right motors
    
    mcpwm_generator_config_t generator_config_1 = {
        .gen_gpio_num = motor1_pin,
    };
    mcpwm_gen_handle_t generator_1 = NULL;
    mcpwm_new_generator(oper_1, &generator_config_1, &generator_1);

    mcpwm_generator_config_t generator_config_2 = {
        .gen_gpio_num = motor2_pin,
    };
    mcpwm_gen_handle_t generator_2 = NULL;
    mcpwm_new_generator(oper_2, &generator_config_2, &generator_2);

    mcpwm_generator_config_t generator_config_3 = {
        .gen_gpio_num = motor3_pin,
    };
    mcpwm_gen_handle_t generator_3 = NULL;
    mcpwm_new_generator(oper_2, &generator_config_3, &generator_3);

    mcpwm_generator_config_t generator_config_4 = {
        .gen_gpio_num = motor4_pin,
    };
    mcpwm_gen_handle_t generator_4 = NULL;
    mcpwm_new_generator(oper_1, &generator_config_4, &generator_4);

    //Creating four comparators : 
    //The comparator will compare the actual timer value with a target value, the target value is the total up time of the period.
    mcpwm_comparator_config_t comparator_config_1 = {
        .flags.update_cmp_on_tez = true
    };
    mcpwm_new_comparator(oper_1, &comparator_config_1, &s_comparator_1) ;//The operator receives the time from the timer and distributes it to the comparator
    
    mcpwm_comparator_config_t comparator_config_2 = {
        .flags.update_cmp_on_tez = true
    };
    mcpwm_new_comparator(oper_2, &comparator_config_2, &s_comparator_2) ;
    
    mcpwm_comparator_config_t comparator_config_3 = {
        .flags.update_cmp_on_tez = true
    };
    mcpwm_new_comparator(oper_2, &comparator_config_3, &s_comparator_3) ;
    
    mcpwm_comparator_config_t comparator_config_4 = {
        .flags.update_cmp_on_tez = true
    };
    mcpwm_new_comparator(oper_1, &comparator_config_4, &s_comparator_4) ;
    
    //Set the actions on timer and comparator events :
    mcpwm_generator_set_action_on_timer_event(
        generator_1,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH));

    mcpwm_generator_set_action_on_compare_event(
        generator_1,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       s_comparator_1,
                                       MCPWM_GEN_ACTION_LOW));
                                        //Set the actions on comparator events :
    mcpwm_generator_set_action_on_timer_event(
        generator_2,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH));

    mcpwm_generator_set_action_on_compare_event(
        generator_2,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       s_comparator_2,
                                       MCPWM_GEN_ACTION_LOW));
                                        //Set the actions on comparator events :
    mcpwm_generator_set_action_on_timer_event(
        generator_3,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH));

    mcpwm_generator_set_action_on_compare_event(
        generator_3,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       s_comparator_3,
                                       MCPWM_GEN_ACTION_LOW));
                                        //Set the actions on comparator events :
    mcpwm_generator_set_action_on_timer_event(
        generator_4,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH));

    mcpwm_generator_set_action_on_compare_event(
        generator_4,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       s_comparator_4,
                                       MCPWM_GEN_ACTION_LOW));
    
    //Arm the ESCs
    mcpwm_comparator_set_compare_value(s_comparator_1, 1000);
    mcpwm_comparator_set_compare_value(s_comparator_2, 1000);
    mcpwm_comparator_set_compare_value(s_comparator_3, 1000);
    mcpwm_comparator_set_compare_value(s_comparator_4, 1000);

    //Start the timer
    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
    
    vTaskDelay(pdMS_TO_TICKS(3000));
}



void write_motors(mix_t *mix){
    mix->pwm_1 = 1000u + (uint16_t)(mix->f1 * 1000.0f);
    mix->pwm_2 = 1000u + (uint16_t)(mix->f2 * 1000.0f);
    mix->pwm_3 = 1000u + (uint16_t)(mix->f3 * 1000.0f);
    mix->pwm_4 = 1000u + (uint16_t)(mix->f4 * 1000.0f);
    
    mcpwm_comparator_set_compare_value(s_comparator_1, mix->pwm_1);
    mcpwm_comparator_set_compare_value(s_comparator_2, mix->pwm_2);
    mcpwm_comparator_set_compare_value(s_comparator_3, mix->pwm_3);
    mcpwm_comparator_set_compare_value(s_comparator_4, mix->pwm_4);
}
