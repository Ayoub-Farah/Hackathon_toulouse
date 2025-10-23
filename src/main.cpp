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
#include "ShieldAPI.h"
#include "CommunicationAPI.h"

/*--------------OWNTECH Libraries----------------------------- */
#include "pid.h"
#include "arm_math_types.h"
#include <ScopeMimicry.h>
#include <cstddef>
#include <cstdint>

/*-- Zephyr includes --*/
#include "zephyr/console/console.h"

#define MMC_LEAD 0
#define MMC_SM1 1
#define MMC_SM2 2
#define MMC_SM3 3
#define MMC_SM4 4
#define MMC_SM5 5
#define MMC_SM6 6
#define MMC_SM7 7
#define MMC_SM8 8
#define MMC_SM9 9
#define MMC_SM10 10

constexpr uint8_t MMC_SM_COUNT = 10;
constexpr uint8_t MMC_SM_FIRST = MMC_SM1;
constexpr uint8_t MMC_SM_LAST = MMC_SM10;

/* -------------- BOARD IDENTIFICATION ----------------------- */

/* TODO: Replace dummy UIDs with actual board identifiers when available. */
constexpr uint32_t UID_MMC_LEAD_BOARD = 0x11112222;
constexpr uint32_t UID_MMC_SM1_BOARD = 0x11113333;
constexpr uint32_t UID_MMC_SM2_BOARD = 0x11114444;
constexpr uint32_t UID_MMC_SM3_BOARD = 0x11115555;
constexpr uint32_t UID_MMC_SM4_BOARD = 0x11116666;
constexpr uint32_t UID_MMC_SM5_BOARD = 0x11117777;
constexpr uint32_t UID_MMC_SM6_BOARD = 0x11118888;
constexpr uint32_t UID_MMC_SM7_BOARD = 0x11119999;
constexpr uint32_t UID_MMC_SM8_BOARD = 0x1111AAA0;
constexpr uint32_t UID_MMC_SM9_BOARD = 0x1111BBB1;
constexpr uint32_t UID_MMC_SM10_BOARD = 0x1111CCC2;

static uint32_t read_board_uid()
{
    static volatile uint32_t *const uid0 =
        reinterpret_cast<volatile uint32_t *>(0x1FFF7590UL);
    return *uid0;
}

static uint8_t detect_module_id()
{
    switch (read_board_uid())
    {
    case UID_MMC_LEAD_BOARD:
        return MMC_LEAD;
    case UID_MMC_SM1_BOARD:
        return MMC_SM1;
    case UID_MMC_SM2_BOARD:
        return MMC_SM2;
    case UID_MMC_SM3_BOARD:
        return MMC_SM3;
    case UID_MMC_SM4_BOARD:
        return MMC_SM4;
    case UID_MMC_SM5_BOARD:
        return MMC_SM5;
    case UID_MMC_SM6_BOARD:
        return MMC_SM6;
    case UID_MMC_SM7_BOARD:
        return MMC_SM7;
    case UID_MMC_SM8_BOARD:
        return MMC_SM8;
    case UID_MMC_SM9_BOARD:
        return MMC_SM9;
    case UID_MMC_SM10_BOARD:
        return MMC_SM10;
    default:
        return MMC_SM1;
    }
}

/* -------------- DATA PACKING HELPERS ----------------------- */

constexpr float32_t MMC_VOLTAGE_SCALE = 50.0F;

static inline uint16_t mmc_encode_voltage(float32_t voltage)
{
    int32_t raw = static_cast<int32_t>((voltage * 4095.0F) / MMC_VOLTAGE_SCALE);
    if (raw < 0)
    {
        raw = 0;
    }
    if (raw > 0x0FFF)
    {
        raw = 0x0FFF;
    }
    return static_cast<uint16_t>(raw);
}

static inline float32_t mmc_decode_voltage(uint16_t raw)
{
    return (MMC_VOLTAGE_SCALE * static_cast<float32_t>(raw & 0x0FFF)) / 4095.0F;
}

/* --------------SETUP FUNCTIONS DECLARATION------------------- */

/* Setups the hardware and software of the system */
void setup_routine();

/* --------------LOOP FUNCTIONS DECLARATION-------------------- */

/* Code to be executed in the background task */
void loop_background_task();
/* Code to be executed in real time in the critical task */
void loop_critical_task();

/* --------------USER VARIABLES DECLARATIONS------------------- */

/* Auto-detected module ID (uses dummy UIDs for now). */
uint8_t module_ID = detect_module_id(); // The ID of the module, can be set to MMC_LEAD or any other SMx

static uint8_t module_comand; // The command the followers needs to apply
static uint8_t module_command_past;
static bool change_state_command = false; // Flag to change the state of the command
static bool send_idle = false;            // Flag to send idle command from master to followers

/**
 * This is a structure that defines the frame
 * that will be sent and received through the RS485 communication.
 * sm_insertion packs 10 individual insertion flags (1 bit each).
 * capacitor_voltage_raw stores the capacitor voltage encoded on 12 bits.
 * arm_current_raw stores the arm current encoded on 12 bits.
 * status packs 10 individual 3-bit error codes and the arm selection flag.
 * sm_id identifies the source submodule (byte-sized).
 */
struct MMC_frame
{
    union
    {
        uint16_t raw;
        struct
        {
            uint16_t sm1_inserted : 1;
            uint16_t sm2_inserted : 1;
            uint16_t sm3_inserted : 1;
            uint16_t sm4_inserted : 1;
            uint16_t sm5_inserted : 1;
            uint16_t sm6_inserted : 1;
            uint16_t sm7_inserted : 1;
            uint16_t sm8_inserted : 1;
            uint16_t sm9_inserted : 1;
            uint16_t sm10_inserted : 1;
        } bits;
    } sm_insertion;
    uint16_t capacitor_voltage_raw : 12;
    uint16_t arm_current_raw : 12;
    union
    {
        uint32_t raw;
        struct
        {
            uint32_t sm1_error_code : 3;
            uint32_t sm2_error_code : 3;
            uint32_t sm3_error_code : 3;
            uint32_t sm4_error_code : 3;
            uint32_t sm5_error_code : 3;
            uint32_t sm6_error_code : 3;
            uint32_t sm7_error_code : 3;
            uint32_t sm8_error_code : 3;
            uint32_t sm9_error_code : 3;
            uint32_t sm10_error_code : 3;
            uint32_t upper_arm_frame : 1;
        } bits;
    } status;
    uint8_t sm_id;
} __packed;

typedef MMC_frame MMC_frame_t;

constexpr uint8_t MMC_STATUS_BITS_PER_SM = 3;
constexpr uint8_t MMC_STATUS_UPPER_ARM_SHIFT = MMC_STATUS_BITS_PER_SM * MMC_SM_COUNT;
constexpr uint32_t MMC_STATUS_UPPER_ARM_MASK = (1UL << MMC_STATUS_UPPER_ARM_SHIFT);

static inline void mmc_frame_set_voltage_raw(MMC_frame_t &frame, uint16_t raw)
{
    frame.capacitor_voltage_raw = static_cast<uint16_t>(raw & 0x0FFFU);
}

static inline uint16_t mmc_frame_get_voltage_raw(const MMC_frame_t &frame)
{
    return static_cast<uint16_t>(frame.capacitor_voltage_raw & 0x0FFFU);
}

static inline void mmc_frame_set_current_raw(MMC_frame_t &frame, uint16_t raw)
{
    frame.arm_current_raw = static_cast<uint16_t>(raw & 0x0FFFU);
}

static inline uint16_t mmc_frame_get_current_raw(const MMC_frame_t &frame)
{
    return static_cast<uint16_t>(frame.arm_current_raw & 0x0FFFU);
}

static inline void mmc_frame_set_sm_identifier(MMC_frame_t &frame, uint8_t id)
{
    frame.sm_id = id;
}

static inline uint8_t mmc_frame_get_sm_identifier(const MMC_frame_t &frame)
{
    return frame.sm_id;
}

static inline void mmc_frame_set_sm_inserted(MMC_frame_t &frame, uint8_t sm_index, bool inserted)
{
    if (sm_index < MMC_SM_FIRST || sm_index > MMC_SM_LAST)
    {
        return;
    }
    uint8_t shift = static_cast<uint8_t>(sm_index - MMC_SM_FIRST);
    uint16_t mask = static_cast<uint16_t>(1U << shift);
    if (inserted)
    {
        frame.sm_insertion.raw |= mask;
    }
    else
    {
        frame.sm_insertion.raw &= static_cast<uint16_t>(~mask);
    }
}

static inline bool mmc_frame_get_sm_inserted(const MMC_frame_t &frame, uint8_t sm_index)
{
    if (sm_index < MMC_SM_FIRST || sm_index > MMC_SM_LAST)
    {
        return false;
    }
    uint8_t shift = static_cast<uint8_t>(sm_index - MMC_SM_FIRST);
    uint16_t mask = static_cast<uint16_t>(1U << shift);
    return (frame.sm_insertion.raw & mask) != 0U;
}

static inline void mmc_frame_set_sm_error_code(MMC_frame_t &frame, uint8_t sm_index, uint8_t code)
{
    if (sm_index < MMC_SM_FIRST || sm_index > MMC_SM_LAST)
    {
        return;
    }
    uint8_t shift = static_cast<uint8_t>((sm_index - MMC_SM_FIRST) * MMC_STATUS_BITS_PER_SM);
    uint32_t mask = static_cast<uint32_t>(0x7U) << shift;
    frame.status.raw &= ~mask;
    frame.status.raw |= (static_cast<uint32_t>(code & 0x7U) << shift);
}

static inline uint8_t mmc_frame_get_sm_error_code(const MMC_frame_t &frame, uint8_t sm_index)
{
    if (sm_index < MMC_SM_FIRST || sm_index > MMC_SM_LAST)
    {
        return 0U;
    }
    uint8_t shift = static_cast<uint8_t>((sm_index - MMC_SM_FIRST) * MMC_STATUS_BITS_PER_SM);
    uint32_t mask = static_cast<uint32_t>(0x7U) << shift;
    return static_cast<uint8_t>((frame.status.raw & mask) >> shift);
}

static inline void mmc_frame_set_upper_arm_flag(MMC_frame_t &frame, bool is_upper_arm)
{
    if (is_upper_arm)
    {
        frame.status.raw |= MMC_STATUS_UPPER_ARM_MASK;
    }
    else
    {
        frame.status.raw &= ~MMC_STATUS_UPPER_ARM_MASK;
    }
}

static inline bool mmc_frame_is_upper_arm(const MMC_frame_t &frame)
{
    return (frame.status.raw & MMC_STATUS_UPPER_ARM_MASK) != 0U;
}

static inline bool mmc_is_upper_arm_module(uint8_t id)
{
    if (id == MMC_LEAD)
    {
        return true;
    }
    if (id < MMC_SM_FIRST || id > MMC_SM_LAST)
    {
        return false;
    }
    uint8_t offset = static_cast<uint8_t>(id - MMC_SM_FIRST);
    return offset < (MMC_SM_COUNT / 2);
}

static MMC_frame_t dataTX_mmc;
static MMC_frame_t dataRX_mmc;

float32_t MMC_capacitor_voltage[MMC_SM_COUNT];

constexpr size_t MMC_FRAME_SIZE = sizeof(MMC_frame_t);

uint8_t buffer_tx[MMC_FRAME_SIZE];
uint8_t buffer_rx[MMC_FRAME_SIZE];

float32_t MMC_voltage = 0.0f;

uint32_t counter_timer = 0;
uint32_t counter_receive = 0;

uint8_t received_serial_char; // Variable to store the received character from the serial interface
int8_t CommTask_num;

enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    POWERMODE = 1,
};

serial_interface_menu_mode mode = IDLEMODE;

void loop_communication_task(); // Code to be executed in the communication task

/* --------------- Firmware CVB variables ------------------*/

/* [us] period of the control task */
static uint32_t control_task_period = 100;
/* [bool] state of the PWM (ctrl task) */
static bool pwm_enable = false;

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

static bool enable_acq; // trigger variable
static float32_t trig_ratio;
static float32_t begin_trig_ratio = 0.05;
static float32_t end_trig_ratio = 0.95;
static uint32_t num_trig_ratio_point = 1024;
static const uint16_t NB_DATAS = 2048; // Number of data acquired
static const float32_t minimal_step = 1.0F / (float32_t)NB_DATAS;
static uint16_t number_of_cycle = 2;
static ScopeMimicry scope(NB_DATAS, 5);
static bool is_downloading;

/* SM switching variables */

static uint32_t critical_period = 100; // 100 µs;
static uint8_t N_u;
static uint8_t N_l;
static float32_t scope_1;
static float32_t number_of_connected_submodules_upper_arm;
static float32_t number_of_connected_submodules_lower_arm;

static float32_t scope_2;
static bool master = true;
static uint8_t seq_u[6] = {1, 2, 3, 2, 1, 0};
static uint8_t seq_l[6] = {2, 1, 0, 1, 2, 3};
static uint8_t counter_seq = 0;
static uint32_t sw_timer = 0;
static uint32_t scope_timer = 0;
static uint32_t f_sw = 2; // 2 Hz = 0.5 s to transition;
// static uint32_t sw_period = 1/(f_sw*critical_period)*1000000; // 2 Hz = 0.5 s to transition;
static uint32_t sw_period = 1000; // 2 Hz = 0.5 s to transition;
static uint32_t scope_period = 1; // acquire every 1000 * 100 µs;

/* CVB variables */
static float32_t values[3] = {3.0, 5.0, 4.0}; // Example values to be sorted
static uint8_t indexes[3] = {1, 2, 3};        // Example indexes to be sorted
static uint8_t counter = 0;
static uint8_t N_modules = 3;
static float32_t index_1;
static float32_t index_2;
static float32_t index_3;

/* Gate logic */
static uint8_t gate_index;
uint8_t g[3] = {0, 0, 0}; // Example gate signals to send
static uint8_t g_SM;
static float32_t g_u_1;
static float32_t g_u_2;
static float32_t g_u_3;
static float32_t g_l_1;
static float32_t g_l_2;
static float32_t g_l_3;

/* --------------SETUP FUNCTIONS------------------------------- */

void config_led_LL()
{
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_5, LL_GPIO_PULL_NO);
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
}

inline void Led_turnON_LL()
{
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
}

inline void Led_turnOFF_LL()
{
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
}

/* Trigger function for scope manager */
bool a_trigger()
{
    return enable_acq;
}

void dump_scope_datas(ScopeMimicry &scope)
{
    uint8_t *buffer = scope.get_buffer();
    /* We divide by 4 (4 bytes per float data) */
    uint16_t buffer_size = scope.get_buffer_size() >> 2;
    printk("begin record\n");
    printk("#");
    for (uint16_t k = 0; k < scope.get_nb_channel(); k++)
    {
        printk("%s,", scope.get_channel_name(k));
    }
    printk("\n");
    printk("# %d\n", scope.get_final_idx());
    for (uint16_t k = 0; k < buffer_size; k++)
    {
        printk("%08x\n", *((uint32_t *)buffer + k));
        task.suspendBackgroundUs(100);
    }
    printk("end record\n");
}

void reception_function(void)
{
    dataRX_mmc = *(MMC_frame_t *)buffer_rx;
    uint8_t sender_id = mmc_frame_get_sm_identifier(dataRX_mmc);

    if (module_ID == MMC_LEAD)
    {
        if ((sender_id >= MMC_SM_FIRST) && (sender_id <= MMC_SM_LAST))
        {
            MMC_capacitor_voltage[sender_id - MMC_SM_FIRST] =
                mmc_decode_voltage(mmc_frame_get_voltage_raw(dataRX_mmc));
        }
    }

    else
    {
        if (sender_id == MMC_LEAD)
        {
            /* retrievig command from lead message*/
            module_comand = static_cast<uint8_t>(
                mmc_frame_get_sm_inserted(dataRX_mmc, module_ID));
            /* retrieving status */
            if (mmc_frame_get_sm_error_code(dataRX_mmc, module_ID) != 0U)
            {
                mode = POWERMODE;
            }
            else
            {
                mode = IDLEMODE;
            }
        }

        /* The board following the ID of the one who sent will start sending
            the next message */
        if (sender_id == static_cast<uint8_t>(module_ID - 1))
        {
            dataTX_mmc = dataRX_mmc; // Copy the received data to the transmission data
            mmc_frame_set_sm_identifier(dataTX_mmc, module_ID);
            mmc_frame_set_upper_arm_flag(dataTX_mmc, mmc_is_upper_arm_module(module_ID));
            mmc_frame_set_voltage_raw(dataTX_mmc,
                                      mmc_encode_voltage(MMC_voltage)); /* TODO: replace MMC_voltage with measured value */
            if (mode == POWERMODE)
            {
                memcpy(buffer_tx, &dataTX_mmc, sizeof(dataTX_mmc));
                communication.rs485.startTransmission();
            }
        }
    }
    counter_receive++;
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
    master = (module_ID == MMC_LEAD);

    config_led_LL(); // Configure the LED pin in Low Level

    shield.power.initBuck(ALL);
    /* Declare task */
    uint32_t background_task_number =
        task.createBackground(loop_background_task);

    /* Uncomment following line if you use the critical task */
    task.createCritical(loop_critical_task, 100);

    shield.sensors.enableDefaultTwistSensors();

    /* Finally, start tasks */
    task.startBackground(background_task_number);
    /* Uncomment following line if you use the critical task */
    task.startCritical();

    CommTask_num = task.createBackground(loop_communication_task);
    task.startBackground(CommTask_num);

    communication.rs485.configure(buffer_tx, buffer_rx, sizeof(buffer_rx),
                                  reception_function,
                                  SPEED_20M); // custom configuration for RS485
                                              /* Configure scope channels, what measurements do you want to acquire? */
    if (master == true)
    {
        scope.connectChannel(number_of_connected_submodules_upper_arm, "N_u");
        scope.connectChannel(number_of_connected_submodules_lower_arm, "N_l");
        // scope.connectChannel(values[0], "value 1");
        // scope.connectChannel(values[1], "value 2");
        // scope.connectChannel(values[2], "value 3");
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
}

/* --------------LOOP FUNCTIONS-------------------------------- */

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
        send_idle = false; 
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

/**
 * This is the code loop of the background task
 * It runs perpetually. Here a `suspendBackgroundMs` is used to pause during
 * 1000ms between each LED toggles.
 * Hence we expect the LED to blink each second.
 */
void loop_background_task()
{
    if (module_ID == MMC_LEAD)
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
    }
    else
    {
        if (mode == IDLEMODE)
        {
            spin.led.turnOff();
        }
    }

    task.suspendBackgroundMs(2000);
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
    if (mode == POWERMODE)
    {
        /* The lead sends commands to the followers */
        if (module_ID == MMC_LEAD)
        {
            if (sw_timer == sw_period)
            {
                if (counter_seq >= 6)
                {
                    counter_seq = 0;
                }
                number_of_connected_submodules_upper_arm = (float)seq_u[counter_seq]; // recuperate
                number_of_connected_submodules_lower_arm = (float)seq_l[counter_seq]; // recuperate
                counter_seq++;
                sw_timer = 0;
            }

            if (number_of_connected_submodules_upper_arm == 0)
            {
                g[0] = 0;
                g[1] = 0;
                g[2] = 0;
            }
            if (number_of_connected_submodules_upper_arm == 1)
            {
                g[0] = 1;
                g[1] = 0;
                g[2] = 0;
            }
            if (number_of_connected_submodules_upper_arm == 2)
            {
                g[0] = 1;
                g[1] = 1;
                g[2] = 0;
            }
            if (number_of_connected_submodules_upper_arm == 3)
            {
                g[0] = 1;
                g[1] = 1;
                g[2] = 1;
            }

            index_1 = (float)indexes[0]; // recuperate
            index_2 = (float)indexes[1]; // recuperate
            index_3 = (float)indexes[2]; // recuperate

            g_u_1 = (float)g[0]; // recuperate
            g_u_2 = (float)g[1]; // recuperate
            g_u_3 = (float)g[2]; // recuperate

            if (scope_timer == scope_period)
            {
                scope.acquire();
                scope_timer = 0;
            }
            sw_timer++;
            scope_timer++;

            dataTX_mmc.sm_insertion.raw = 0U;
            mmc_frame_set_sm_inserted(dataTX_mmc, MMC_SM1, g[0] != 0U);
            mmc_frame_set_sm_inserted(dataTX_mmc, MMC_SM2, g[1] != 0U);
            mmc_frame_set_sm_inserted(dataTX_mmc, MMC_SM3, g[2] != 0U);

            dataTX_mmc.status.raw = 0U;
            for (uint8_t sm = MMC_SM_FIRST; sm <= MMC_SM_LAST; ++sm)
            {
                mmc_frame_set_sm_error_code(dataTX_mmc, sm, 1U);
            }
            mmc_frame_set_upper_arm_flag(dataTX_mmc, mmc_is_upper_arm_module(module_ID));
            mmc_frame_set_sm_identifier(dataTX_mmc, module_ID);
            mmc_frame_set_voltage_raw(dataTX_mmc, mmc_encode_voltage(MMC_voltage));
            memcpy(buffer_tx, &dataTX_mmc, sizeof(dataTX_mmc));
            communication.rs485.startTransmission();
        }
        else
        {
            if (module_comand != module_command_past)
            {
                change_state_command = true; // Set the flag to change the state
            }

            if (module_comand)
            {
                if (change_state_command)
                {
                    Led_turnON_LL();
                    change_state_command = false; // Reset the flag
                }
            }
            else
            {
                if (change_state_command)
                {
                    Led_turnOFF_LL();
                    change_state_command = false; // Reset the flag
                }
            }
        }
        module_command_past = module_comand; // Update the past command
    }
    else if (mode == IDLEMODE)
    {
        /* Made to send IDLE flag only once */
        if (!send_idle)
        {
            dataTX_mmc.sm_insertion.raw = 0U;
            dataTX_mmc.status.raw = 0U;
            for (uint8_t sm = MMC_SM_FIRST; sm <= MMC_SM_LAST; ++sm)
            {
                mmc_frame_set_sm_error_code(dataTX_mmc, sm, 0U);
            }
            mmc_frame_set_upper_arm_flag(dataTX_mmc, mmc_is_upper_arm_module(module_ID));
            mmc_frame_set_sm_identifier(dataTX_mmc, module_ID);
            mmc_frame_set_voltage_raw(dataTX_mmc, mmc_encode_voltage(MMC_voltage));
            memcpy(buffer_tx, &dataTX_mmc, sizeof(dataTX_mmc));
            communication.rs485.startTransmission();
            send_idle = true; // Set the flag to send idle command
        }
    }
    counter_timer++;
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
