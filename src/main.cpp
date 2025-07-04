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
static bool trigger = true;

/* SM switching variables */

static uint8_t N_u;
static uint8_t N_l;
static float32_t scope_1;
static float32_t scope_2;
static bool master = true;
static uint8_t g = 0;
static uint8_t seq_u[6] = {0, 1, 2, 3, 2, 1};
static uint8_t seq_l[6] = {3, 2, 1, 0, 1, 2};
static uint8_t counter = 0;
static uint32_t sw_timer = 0;
static uint32_t scope_timer = 0;
static uint32_t f_sw = 2; // 2 Hz = 0.5 s to transition;
static uint32_t scope_period = 1000; // acquire every 1000 * 100 µs;


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
    shield.power.initBuck(LEG1);
    shield.power.initBoost(LEG2);

    shield.sensors.enableDefaultTwistSensors();

    /* Enable switch control with max and min duty cycle*/
    shield.power.setDutyCycleMax(ALL,1.0);
    shield.power.setDutyCycleMin(ALL,0.0);

    /* Configure scope channels, what measurements do you want to acquire? */
    if (master == true)
    {
        scope.connectChannel(scope_1, "N_u");
        scope.connectChannel(scope_2, "N_l");
        scope.set_trigger(&a_trigger);
        scope.set_delay(0.2F);
        scope.start();
    }

    pid.init(pid_params);

    /* Then declare tasks */
    uint32_t app_task_number = task.createBackground(loop_application_task);
    uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.createCritical(loop_critical_task, 100);

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
            "|     press r : record data              |\n"
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
        trigger = true;
        break;
    case 'o':
        printk("SM ON\n");
        g = 1;
        break;
    case 'f':
        printk("SM OFF\n");
        g = 0;
        break;
    case 'r':
        is_downloading = true;
        trigger = false;
        break;
    default:
        break;
    }
}

/**
 * This is the code loop of the background task
 * This task mostly logs back measurements to the USB serial interface.
 */
void loop_application_task()
{   
    if (master == true)
    {
        if (mode == IDLEMODE)
        {
            spin.led.turnOff();
            if (is_downloading)
            {
                dump_scope_datas(scope);
                is_downloading = false;
            }
        }
        if (mode == POWERMODE)
        {
            spin.led.toggle();

            /*
            if (counter >= 6) {
                counter = 0;
            }
            N_u = seq_u[counter];  // recuperate
            N_l = seq_l[counter];  // recuperate
            counter++;
            */

            printk("%1.f:", scope_1);
            printk("%1.f:", scope_2);
            printk("%u:", counter);
            printk("%u:", sw_timer);
            printk("\n");

        }

        task.suspendBackgroundMs(100);
    }

    if (master == false)
    {
        if (mode == IDLEMODE)
        {
            spin.led.toggle(); //indicates it is working
        }
        else if (mode == POWERMODE)
        {
            if (g == 1)
            {
                spin.led.turnOn();
            }
            else if (g == 1)
            {
                spin.led.turnOff();
            }
        }
        task.suspendBackgroundMs(500000);
    }
}

/**
 * This is the code loop of the critical task
 * This task runs at 10kHz.
 * - It update main N_u and N_l
 * - It update sets follower logic -> SM on or off
 */
void loop_critical_task()
{
    if (master == true)
    {
        if (mode == IDLEMODE)
        {

        }

        if (mode == POWERMODE)
        {
            if (sw_timer == 1/f_sw)
            {
                if (counter >= 6) {
                counter = 0;
                }
                scope_1 = (float)seq_u[counter];  // recuperate
                scope_2 = (float)seq_l[counter];  // recuperate
                counter++;
                sw_timer = 0;
            }

            if (scope_timer == scope_period)
            {
                scope.acquire();
            }
            sw_timer++;
            scope_timer++;
        }
    
    }
    else
    {

        if (mode == IDLEMODE)
        {

            if (pwm_enable == true)
            {
                shield.power.stop(ALL);
            }
            pwm_enable = false;
        }
        else if (mode == POWERMODE)
        {
            if(g == 0) // SM is off
            {
                //LED OFF
            }
            if(g == 1) // SM is on
            {
                //LED ON
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
