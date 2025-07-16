/*
 * Copyright (c) 2023-present LAAS-CNRS
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
 * @author Régis Ruelland <regis.ruelland@laas.fr>
 */

/*--------------Zephyr---------------------------------------- */
#include <zephyr/console/console.h>

/*--------------OWNTECH APIs---------------------------------- */
#include "SpinAPI.h"
#include "ShieldAPI.h"
#include "TaskAPI.h"

/*--------------OWNTECH Libraries----------------------------- */
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

void sorting();

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

/* Temporary storage fore measured value (ctrl task) */
static float meas_data;

float32_t duty_cycle = 0.3;

/* Scope variables */

static bool enable_acq; //trigger variable
static const uint16_t NB_DATAS = 2048; //Number of data acquired
static const float32_t minimal_step = 1.0F / (float32_t) NB_DATAS;
static ScopeMimicry scope(NB_DATAS, 5);
static bool is_downloading;

/* SM switching variables */

static float32_t number_of_connected_submodules_upper_arm;
static float32_t number_of_connected_submodules_lower_arm;

static bool master = true;
static uint8_t seq_u[6] = {1, 2, 3, 2, 1, 0};
static uint8_t seq_l[6] = {2, 1, 0, 1, 2, 3};
static uint8_t counter_seq = 0;
static uint32_t sw_timer = 0;
static uint32_t scope_timer = 0;
static uint32_t sw_period = 10000; // 2 Hz = 0.5 s to transition;
static uint32_t scope_period = 100; // acquire every 1000 * 100 µs;

/* CVB variables */
static float32_t modules_capacitor_voltages[3] = {3.0,5.0,4.0}; // Example values to be sorted
static uint8_t modules_indexes[3] = {0,1,2}; // Example indexes to be sorted
static uint8_t total_number_of_modules_arm= 3;
static float32_t index_1;
static float32_t index_2;
static float32_t index_3;

/* Gate logic */
uint8_t g[3] = {0,0,0}; // Example gate signals to send
static uint8_t g_SM;
static float32_t g_u_1;
static float32_t g_u_2;
static float32_t g_u_3;
static float32_t g_l_1;
static float32_t g_l_2;
static float32_t g_l_3;


/*--------------------------------------------------------------- */

/* LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER */
enum serial_interface_menu_mode
{
    IDLEMODE = 0,
    POWERMODE
};

uint8_t mode = IDLEMODE;

/* Trigger function for scope manager */
bool a_trigger() {
    return enable_acq;
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
        scope.connectChannel(number_of_connected_submodules_upper_arm, "N_u");
        scope.connectChannel(number_of_connected_submodules_lower_arm, "N_l");
        //scope.connectChannel(values[0], "value 1");
        //scope.connectChannel(values[1], "value 2");
        //scope.connectChannel(values[2], "value 3");
        // scope.connectChannel(index_1, "index 1");
        // scope.connectChannel(index_2, "index 2");
        // scope.connectChannel(index_3, "index 3");
        scope.connectChannel(g_u_1, "g_u_1");
        scope.connectChannel(g_u_2, "g_u_2");
        scope.connectChannel(g_u_3, "g_u_3");
        scope.set_trigger(&a_trigger);
        scope.set_delay(0.0F);
        scope.start();
    }

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
    if (master == true)
    {
        switch (received_serial_char)
        {
        case 'h':
            /*----------SERIAL INTERFACE MENU----------------------- */
            printk(" ________________________________________ \n"
                "|     ---- MENU buck voltage mode ----   |\n"
                "|     press i : idle mode                |\n"
                "|     press p : power mode               |\n"
                "|     press r : record data              |\n"
                "|     press a : toggle enable_acq var    |\n"
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
        case 'r':
            is_downloading = true;
            break;
        case 'a':
            enable_acq = !(enable_acq);
            break;
        default:
            break;
        }
    }
    else{
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
                "|     press a : toggle enable_acq var    |\n"
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
            g_SM = 1;
            break;
        case 'f':
            printk("SM OFF\n");
            g_SM = 0;
            break;
        case 'r':
            is_downloading = true;
            break;
        case 'a':
            enable_acq = !(enable_acq);
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
            printk("%1.f:", number_of_connected_submodules_upper_arm);
            printk("%1.f:", number_of_connected_submodules_lower_arm);
            printk("%u:", counter_seq);
            printk("%u:", sw_timer);
            printk("%1.f:", index_1);
            printk("%1.f:", index_2);
            printk("%1.f:", index_3);
            printk("%u:", g_u_1);
            printk("%u:", g_u_2);
            printk("%u:", g_u_3);
            printk("\n");
        }
        task.suspendBackgroundMs(1000);
    }

    if (master == false)
    {
        if (mode == IDLEMODE)
        {
            spin.led.toggle(); //indicates it is working
        }
        else if (mode == POWERMODE)
        {
            if (g_SM == 1)
            {
                spin.led.turnOn();
                
            }
            else if (g_SM == 0)
            {
                spin.led.turnOff();
                
            }
        }
        task.suspendBackgroundMs(2000);
    }
}


void sorting()
{
    uint8_t counter_loops_sorting = 0;
    while(counter_loops_sorting < 10){
            g[0] = 0;
            g[1] = 0;
            g[2] = 0;
            for(uint8_t counter = 0; counter < total_number_of_modules_arm-1; counter++)
            {
                if(modules_capacitor_voltages[counter] > modules_capacitor_voltages[counter + 1]) //Sorting modules indexes according to capacitor voltage
                {
                    float32_t temp = modules_capacitor_voltages[counter];
                    modules_capacitor_voltages[counter] = modules_capacitor_voltages[counter + 1];
                    modules_capacitor_voltages[counter + 1] = temp;
                    float32_t temp2 = modules_indexes[counter];
                    modules_indexes[counter] = modules_indexes[counter + 1];
                    modules_indexes[counter + 1] = temp2;
                }
                
                
            }

            counter_loops_sorting++;
        }
    for(uint8_t counter = 0; counter < total_number_of_modules_arm; counter++)
        {
            if(counter < number_of_connected_submodules_upper_arm)
                {
                     uint8_t index_smallest_voltage_capacitor = modules_indexes[counter];
                     g[index_smallest_voltage_capacitor] = 1;
                     //g[counter] = 1;
                }
        }

}

void gate_signals()
{


}
/**
 * This is the code loop of the critical task
 * This task runs at 10kHz.
 * - It update main N_u and N_l
 * - It update sets follower logic -> SM on or off
 * - It does the voltage sorting
 * - It creates the gate signals to the SM
 */
void loop_critical_task()
{
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

    if (master == true)
    {
        if (mode == IDLEMODE)
        {
            if (pwm_enable == true)
            {
                shield.power.stop(ALL);
            }
            pwm_enable = false;
        }

        if (mode == POWERMODE)
        {
            if (sw_timer == sw_period)
            {
                if (counter_seq >= 6) {
                    counter_seq = 0;
                }
                number_of_connected_submodules_upper_arm = (float)seq_u[counter_seq];  // recuperate
                number_of_connected_submodules_lower_arm = (float)seq_l[counter_seq];  // recuperate
                counter_seq++;
                sw_timer = 0;
            }

            sorting();

            index_1 = (float)modules_indexes[0];  // recuperate
            index_2 = (float)modules_indexes[1];  // recuperate
            index_3 = (float)modules_indexes[2];  // recuperate

            g_u_1 = (float)g[0];  // recuperate
            g_u_2 = (float)g[1];  // recuperate
            g_u_3 = (float)g[2];  // recuperate
            
            if (scope_timer == scope_period)
            {
                scope.acquire();
                scope_timer = 0;
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
            if(g_SM == 0) // SM is off
            {
                /*
                shield.power.setDutyCycle(LEG1,0.0); // Switches Q1 off
                if (!pwm_enable)
                {
                    pwm_enable = true;
                    shield.power.start(LEG1);
                }
                */
            }
            if(g_SM == 1) // SM is on
            {
                /*
                shield.power.setDutyCycle(LEG1,1.0); // Switches Q1 on
                if (!pwm_enable)
                {
                    pwm_enable = true;
                    shield.power.start(LEG1);
                }
                */
                
            }
            //shield.power.stop(LEG2);
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
