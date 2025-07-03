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
 * @brief  This example shows how to blink the onboard LED of the Spin board.
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 * @author Ayoub Farah Hassan <ayoub.farah-hassan@laas.fr>
 */

/* --------------OWNTECH APIs---------------------------------- */
#include "SpinAPI.h"
#include "TaskAPI.h"
#include "CommunicationAPI.h"

#define MMC_LEAD 0
#define MMC_SM1 1
#define MMC_SM2 2
#define MMC_SM3 3
#define MMC_SM4 4
#define MMC_SM5 5
#define MMC_SM6 6

/**
 * @brief This function is considering a byte called 'cmd' 
 *        which can turn on or off signals.
 *        The signals are identified by their 'id'.
 *        The value 'val' is used to set the signal:
 *        - if val is true, the signal is set to 1
 *        - if val is false, the signal is set to 0
 */
#define SET_SIGNAL(cmd, id, val) \
    do { \
        if (val) (cmd) |= (1 << (id)); \
        else (cmd) &= ~(1 << (id)); \
    } while(0)

/**
 * @brief This function is to get the turn on/off state of a signal
 *        identified by its 'id' from a byte called 'cmd'.
 */
#define GET_SIGNAL(cmd, id) (((cmd) >> (id)) & 0x01)

/* --------------SETUP FUNCTIONS DECLARATION------------------- */

/* Setups the hardware and software of the system */
void setup_routine();

/* --------------LOOP FUNCTIONS DECLARATION-------------------- */

/* Code to be executed in the background task */
void loop_background_task();
/* Code to be executed in real time in the critical task */
void loop_critical_task();

/* --------------USER VARIABLES DECLARATIONS------------------- */

/* TODO : Define module_ID depending on the ID of the board */
uint8_t module_ID = MMC_LEAD;

static uint8_t module_comand; // The command ethe slave needs to apply

/**
 * This is a structure that defines the frame 
 * that will be sent and received through the RS485 communication.
 * command is a byte that contains the state of the signals
 * Capacitor_Voltage is the voltage of the capacitor
 * ID is the ID of the module
 */
struct MMC_frame {
	uint8_t command;
	float32_t Capacitor_Voltage;
	uint8_t ID;
}__packed;

typedef MMC_frame MMC_frame_t;
static MMC_frame_t dataTX_mmc;
static MMC_frame_t dataRX_mmc;

float32_t MMC_capacitor_voltage[6];


uint8_t buffer_tx[6];
uint8_t buffer_rx[6];

/* --------------SETUP FUNCTIONS------------------------------- */

void reception_function(void)
{
	dataRX_mmc = *(MMC_frame_t *) buffer_rx;

	if(Module_ID == MMC_LEAD)
	{
		MMC_capacitor_voltage[dataRX_mmc.ID-1] = dataRX_mmc.Capacitor_Voltage;
	}

	else
		{
            if(dataRX_mmc.ID == MMC_LEAD)
            {
                module_comand = GET_SIGNAL(dataRX_mmc.command, dataRX_mmc.ID);	
            }
			
			if((dataRX_mmc.ID == module_ID-1))
			{
			    dataTX_mmc.ID = module_ID;
			   // dataTX_mmc.Capacitor_Voltage = MMC_voltage; /* TODO :uncomment when we get the voltage */
			    memcpy(buffer_tx, &dataTX_mmc, sizeof(dataTX_mmc));
			   communication.rs485.startTransmission();
			}
	
		}
}

/**
 * This is the setup routine.
 * It is used to call functions that will initialize your spin, power shields
 * and tasks.
 *
 * In this example, we spawn a background task.
 * An optional critical task can be initialized by uncommenting the two
 * commented lines.
 */
void setup_routine()
{
    /* Declare task */
    uint32_t background_task_number =
                            task.createBackground(loop_background_task);

    /* Uncomment following line if you use the critical task */
    task.createCritical(loop_critical_task, 100);

    /* Finally, start tasks */
    task.startBackground(background_task_number);
    /* Uncomment following line if you use the critical task */
    task.startCritical();

    communication.rs485.configure(buffer_tx, buffer_rx, sizeof(buffer_rx),
				      reception_function,
				      SPEED_10M); // custom configuration for RS485
}

/* --------------LOOP FUNCTIONS-------------------------------- */

/**
 * This is the code loop of the background task
 * It runs perpetually. Here a `suspendBackgroundMs` is used to pause during
 * 1000ms between each LED toggles.
 * Hence we expect the LED to blink each second.
 */
void loop_background_task()
{
    /* Task content */
    spin.led.toggle();

    /* Pause between two runs of the task */
    task.suspendBackgroundMs(1000);
}

/**
 * Uncomment lines in setup_routine() to use critical task.
 *
 * This is the code loop of the critical task
 * It is executed every 500 micro-seconds defined in the setup_software
 * function. You can use it to execute an ultra-fast code with
 * the highest priority which cannot be interrupted by the background tasks.
 *
 * In the critical task, you can implement your control algorithm that will
 * run in Real Time and control your power flow.
 */
void loop_critical_task()
{
	SET_SIGNAL(data_mmc.command, data_mmc.MMC_SM1, 0);
	SET_SIGNAL(data_mmc.command, data_mmc.MMC_SM2, 1);
	SET_SIGNAL(data_mmc.command, data_mmc.MMC_SM3, 1);
	SET_SIGNAL(data_mmc.command, data_mmc.MMC_SM4, 1);
	SET_SIGNAL(data_mmc.command, data_mmc.MMC_SM5, 1);
	SET_SIGNAL(data_mmc.command, data_mmc.MMC_SM6, 1);

    memcpy(buffer_tx, &data_mmc, sizeof(data_mmc));
    communication.rs485.startTransmission();
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
