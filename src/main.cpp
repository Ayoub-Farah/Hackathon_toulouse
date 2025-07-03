/*
 * Copyright (c) 2021-present LAAS-CNRS
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @brief  This example demonstrates how to deploy a Buck converter with
 *         voltage mode control on the Twist power shield.
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 * @author Ayoub Farah Hassan <ayoub.farah-hassan@laas.fr>
 */

/*--------------Zephyr---------------------------------------- */
#include <zephyr/console/console.h>

/*--------------OWNTECH APIs---------------------------------- */
#include "SpinAPI.h"
#include "ShieldAPI.h"
#include "TaskAPI.h"

/*--------------OWNTECH Libraries----------------------------- */
#include "pid.h"
#include "arm_math_types.h"
#include <ScopeMimicry.h>

/*--------------SETUP FUNCTIONS DECLARATION------------------- */
/* Setups the hardware and software of the system */
void setup_routine();

/*--------------LOOP FUNCTIONS DECLARATION-------------------- */
/* Code to be executed in the slow communication task */
void loop_communication_task();
/* Code to be executed in the background task */
void loop_application_task();
/* Code to be executed in real time in the critical task */
void loop_critical_task();

/*--------------USER VARIABLES DECLARATIONS------------------- */

/* [us] period of the control task */
static uint32_t control_task_period = 100;
/* [bool] state of the PWM (ctrl task) */
static bool pwm_enable = false;

uint8_t received_serial_char;

/* Measure variables */

static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I_high;
static float32_t V_high;

static float32_t temp_1_value;
static float32_t temp_2_value;

/* Temporary storage fore measured value (ctrl task) */
static float meas_data;

float32_t duty_cycle = 0.3;

/* Voltage reference */
static float32_t voltage_reference = 15;

/* PID coefficients for a 8.6ms step response*/
static float32_t kp = 0.000215;
static float32_t Ti = 7.5175e-5;
static float32_t Td = 0.0;
static float32_t N = 0.0;
static float32_t upper_bound = 1.0F;
static float32_t lower_bound = 0.0F;
static float32_t Ts = control_task_period * 1e-6;
static PidParams pid_params(Ts, kp, Ti, Td, N, lower_bound, upper_bound);
static Pid pid;

/* Scope variables */

static bool enable_acq;
static uint32_t num_trig_ratio_point = 512;
static const uint16_t NB_DATAS = 2048; //Number of data acquired
static const float32_t minimal_step = 1.0F / (float32_t) NB_DATAS;
static uint16_t number_of_cycle = 2;
static ScopeMimicry scope(NB_DATAS, 2);
static bool is_downloading;
static bool trigger = false;

/* SM switching variables */

static float32_t N_u = 0;
static float32_t N_l = 3;
static bool master = true;
static uint8_t g = 0;
static uint8_t direction = 1;
static uint32_t counter = 0;

/*--------------------------------------------------------------- */

/* LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER */
enum serial_interface_menu_mode
{
    IDLEMODE = 0,
    POWERMODE
};

uint8_t mode = IDLEMODE;

/*--------------SCOPE FUNCTIONS------------------------------- */

/* Trigger function for scope manager */
bool a_trigger() {
    return trigger;
}

void dump_scope_datas(ScopeMimicry &scope)  {
    uint8_t *buffer = scope.get_buffer();
    /* We divide by 4 (4 bytes per float data) */
    uint16_t buffer_size = scope.get_buffer_size() >> 2;
    printk("begin record\n");
    printk("#");
    for (uint16_t k=0;k < scope.get_nb_channel(); k++) {
        printk("%s,", scope.get_channel_name(k));
    }
    printk("\n");
    printk("# %d\n", scope.get_final_idx());
    for (uint16_t k=0;k < buffer_size; k++) {
        printk("%08x\n", *((uint32_t *)buffer + k));
        task.suspendBackgroundUs(100);
    }
    printk("end record\n");
}



/*--------------SETUP FUNCTIONS------------------------------- */

/**
 * This is the setup routine.
 * Here the setup :
 *  - Initializes the power shield in Buck mode
 *  - Initializes the power shield sensors
 *  - Initializes the PID controller
 *  - Spawns three tasks.
 */
void setup_routine()
{
    /* Buck voltage mode */
    //shield.power.initBuck(LEG1);
    //shield.power.initBoost(LEG2);

    //shield.sensors.enableDefaultTwistSensors();

    /* Enable switch control with max and min duty cycle*/
    //shield.power.setDutyCycleMax(ALL,1.0);
    //shield.power.setDutyCycleMin(ALL,0.0);

    /* Configure scope channels, what measurelents do you want to acquire? */
    if (master == true)
    {
        scope.connectChannel(N_u, "N_u");
        scope.connectChannel(N_l, "N_l");
        scope.set_trigger(&a_trigger);
        scope.set_delay(0.2F);
        scope.start();
    }
    //Vc_BTS indicates the bootstrap capacitor charge level, we have to measure it externally

    pid.init(pid_params);

    /* Then declare tasks */
    uint32_t app_task_number = task.createBackground(loop_application_task);
    uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.createCritical(loop_critical_task, 10000);

    /* Finally, start tasks */
    task.startBackground(app_task_number);
    task.startBackground(com_task_number);
    task.startCritical();
}

/*--------------LOOP FUNCTIONS-------------------------------- */

/**
 * This tasks implements a minimalistic USB serial interface to control
 * the buck converter.
 */
void loop_communication_task()
{
    if (master == false)
    {
        received_serial_char = console_getchar();
        switch (received_serial_char)
        {
        case 'h':
            /*----------SERIAL INTERFACE MENU----------------------- */
            printk(" ________________________________________ \n"
                "|     ---- MENU buck voltage mode ----   |\n"
                "|     press i : idle mode                |\n"
                "|     press p : power mode               |\n"
                "|     press o : SM is ON                 |\n"
                "|     press f : SM is OFF                |\n"
                "|________________________________________|\n\n");
            /*------------------------------------------------------ */
            break;
        case 'i':
            printk("idle mode\n");
            mode = IDLEMODE;
            break;
        case 'p':
            printk("power mode\n");
            mode = POWERMODE;
            break;
        case 'o':
            printk("SM ON\n");
            g = 1;
            break;
        case 'f':
            printk("SM OFF\n");
            g = 0;
            break;
        default:
            break;
        }
    }
}

/**
 * This is the code loop of the background task
 * This task mostly logs back measurements to the USB serial interface.
 */
void loop_application_task()
{
    if (mode == IDLEMODE)
    {
        //spin.led.turnOff();
    }
    else if (mode == POWERMODE)
    {
        //spin.led.turnOn();

        /*
        shield.sensors.triggerTwistTempMeas(TEMP_SENSOR_1);
        shield.sensors.triggerTwistTempMeas(TEMP_SENSOR_2);

        meas_data = shield.sensors.getLatestValue(TEMP_SENSOR_1);
        if (meas_data != NO_VALUE) temp_1_value = meas_data;

        meas_data = shield.sensors.getLatestValue(TEMP_SENSOR_2);
        if (meas_data != NO_VALUE) temp_2_value = meas_data;


        printk("%.3f:", (double)I1_low_value);
        printk("%.3f:", (double)V1_low_value);
        printk("%.3f:", (double)voltage_reference);
        printk("%.3f:", (double)I2_low_value);
        printk("%.3f:", (double)V2_low_value);
        printk("%.3f:", (double)voltage_reference);
        printk("%.3f:", (double)I_high);
        printk("%.3f:", (double)V_high);
        printk("%.3f:", (double)temp_1_value);
        printk("%.3f:", (double)temp_2_value);
        printk("\n");
        */
    }
    task.suspendBackgroundMs(100);
}

/**
 * This is the code loop of the critical task
 * This task runs at 10kHz.
 *  - It retrieves sensors values
 *  - It runs the PID controller
 *  - It update the PWM signals
 */
void loop_critical_task()
{
    /*
    meas_data = shield.sensors.getLatestValue(I1_LOW);
    if (meas_data != NO_VALUE) I1_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V1_LOW);
    if (meas_data != NO_VALUE) V1_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V2_LOW);
    if (meas_data != NO_VALUE) V2_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(I2_LOW);
    if (meas_data != NO_VALUE) I2_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(I_HIGH);
    if (meas_data != NO_VALUE) I_high = meas_data;

    meas_data = shield.sensors.getLatestValue(V_HIGH);
    if (meas_data != NO_VALUE) V_high = meas_data;
    */

    if (master = true)
    {
        spin.led.toggle(); //indicates it is working

        if (N_u == 3)
        {
            direction = 0;
        }
        else if (N_u == 0)
        {
            direction = 1;
        }
        if (counter == 100)
        {
            if (direction == 1)
            {
                N_u++;
                N_l--;
            }
            if (direction == 0)
            {
                N_u--;
                N_l++;
            }
        }
        scope.acquire();
    }
    if (master = false)
    {

        if (mode == IDLEMODE)
        {
            spin.led.toggle();

            if (pwm_enable == true)
            {
                shield.power.stop(ALL);
            }
            pwm_enable = false;
        }
        else if (mode == POWERMODE)
        {
            //scope.acquire();
            if(g == 0) // SM is off
            {
                //LED OFF
                spin.led.turnOff();
            }
            if(g == 1) // SM is on
            {
                //LED ON
                spin.led.turnOn();
            }
        }
    
    }

}

/**
 * This is the main function of this example
 * This function is generic and does not need editing.
 */
int main(void)
{
    setup_routine();

    return 0;
}
