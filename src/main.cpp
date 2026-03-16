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
 * @brief RS485 communication test for one lead and ten MMC submodules.
 *
 * The frame size stays unchanged. Power control is disabled: no buck start,
 * no duty cycle management and no gate command. Each submodule forwards the
 * chain frame with a fixed capacitor-voltage value. The lead checks the
 * received value against the expected one and reports all debug counters only
 * in idle mode.
 */

#include "CommunicationAPI.h"
#include "SpinAPI.h"
#include "TaskAPI.h"

#include <zephyr/console/console.h>
#include <zephyr/sys/printk.h>

#include <inttypes.h>
#include <string.h>

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

#define IDLE 0
#define POWER 1
#define LEAD_ERROR 2
#define OVER_VOLTAGE 3
#define UNDER_VOLTAGE 4
#define OVER_CURRENT 5

constexpr uint8_t MMC_SM_COUNT = 10;
constexpr uint8_t MMC_SM_FIRST = MMC_SM1;
constexpr uint8_t MMC_SM_LAST = MMC_SM10;

constexpr uint32_t CONTROL_TASK_PERIOD_US = 100U;
constexpr uint32_t BACKGROUND_TASK_PERIOD_MS = 200U;
constexpr uint8_t CONTROL_TASK_DEBUG_GPIO = PC8;

constexpr float Vcap_expected = 80.0F;
constexpr float i_expected = 10.0F;
constexpr float Cap_voltage_SCALE = Vcap_expected * 2.0F;
constexpr float Arm_current_SCALE = i_expected * 2.0F;
constexpr float Arm_current_OFFSET = i_expected;

constexpr float LEAD_TEST_CAP_VOLTAGE = 5.0F;
constexpr float MODULE_TEST_CAP_VOLTAGES[MMC_SM_COUNT] = {
    10.0F, 20.0F, 30.0F, 40.0F, 50.0F,
    60.0F, 70.0F, 80.0F, 90.0F, 100.0F};

constexpr uint32_t UID_MMC_LEAD_BOARD = 0x002B002A;
constexpr uint32_t UID_MMC_SM1_BOARD = 0x0031001B;
constexpr uint32_t UID_MMC_SM2_BOARD = 0x0033004B;
constexpr uint32_t UID_MMC_SM3_BOARD = 0x00330049;
constexpr uint32_t UID_MMC_SM4_BOARD = 0x0033004C;
constexpr uint32_t UID_MMC_SM5_BOARD = 0x00330054;
constexpr uint32_t UID_MMC_SM6_BOARD = 0x11119999;
constexpr uint32_t UID_MMC_SM7_BOARD = 0x1111AAA0;
constexpr uint32_t UID_MMC_SM8_BOARD = 0x1111BBB1;
constexpr uint32_t UID_MMC_SM9_BOARD = 0x1111CCC2;
constexpr uint32_t UID_MMC_SM10_BOARD = 0x1111CCC3;

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

enum serial_interface_menu_mode
{
    IDLEMODE = 0,
    POWERMODE = 1,
};

constexpr uint8_t MMC_STATUS_CODE_BITS = 3;
constexpr uint32_t MMC_STATUS_CODE_MASK = (1UL << MMC_STATUS_CODE_BITS) - 1U;
constexpr uint32_t MMC_STATUS_UPPER_ARM_MASK = (1UL << MMC_STATUS_CODE_BITS);

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
        uint8_t raw;
        struct
        {
            uint8_t status_code : MMC_STATUS_CODE_BITS;
            uint8_t upper_arm_frame : 1;
        } bits;
    } status;
    uint8_t sm_id;
} __packed;

typedef MMC_frame MMC_frame_t;

constexpr size_t MMC_FRAME_SIZE = sizeof(MMC_frame_t);

static uint8_t buffer_tx[MMC_FRAME_SIZE];
static uint8_t buffer_rx[MMC_FRAME_SIZE];
static MMC_frame_t dataTX_mmc;
static MMC_frame_t dataRX_mmc;

static uint8_t module_ID = MMC_SM1;
static bool master = false;
static bool send_idle = true;
static bool idle_debug_pending = false;
static serial_interface_menu_mode mode = IDLEMODE;

static volatile uint32_t counter_timer = 0;
static volatile uint32_t counter_receive = 0;
static volatile uint32_t rx_count_by_module[MMC_SM_COUNT] = {};
static volatile uint32_t voltage_error_count[MMC_SM_COUNT] = {};
static volatile uint16_t last_voltage_raw_by_module[MMC_SM_COUNT] = {};
static volatile uint32_t unexpected_sender_count = 0;

static uint8_t received_serial_char = 0;

static inline uint16_t mmc_encode_voltage(float voltage)
{
    int32_t raw = static_cast<int32_t>((voltage * 4095.0F) / Cap_voltage_SCALE);
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

static inline float mmc_decode_voltage(uint16_t raw)
{
    return (Cap_voltage_SCALE * static_cast<float>(raw & 0x0FFFU)) / 4095.0F;
}

static inline uint16_t mmc_encode_current(float current)
{
    float shifted = current + Arm_current_OFFSET;
    int32_t raw = static_cast<int32_t>((shifted * 4095.0F) / Arm_current_SCALE);
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

static inline void mmc_frame_set_sm_identifier(MMC_frame_t &frame, uint8_t id)
{
    frame.sm_id = id;
}

static inline uint8_t mmc_frame_get_sm_identifier(const MMC_frame_t &frame)
{
    return frame.sm_id;
}

static inline void mmc_frame_set_status_code(MMC_frame_t &frame, uint8_t status_code)
{
    frame.status.raw &= ~MMC_STATUS_CODE_MASK;
    frame.status.raw |= static_cast<uint32_t>(status_code & MMC_STATUS_CODE_MASK);
}

static inline uint8_t mmc_frame_get_status_code(const MMC_frame_t &frame)
{
    return static_cast<uint8_t>(frame.status.raw & MMC_STATUS_CODE_MASK);
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
    return static_cast<uint8_t>(id - MMC_SM_FIRST) < (MMC_SM_COUNT / 2U);
}

static float mmc_expected_voltage_for_module(uint8_t id)
{
    if (id < MMC_SM_FIRST || id > MMC_SM_LAST)
    {
        return 0.0F;
    }
    return MODULE_TEST_CAP_VOLTAGES[id - MMC_SM_FIRST];
}

static float mmc_local_test_voltage(void)
{
    if (module_ID == MMC_LEAD)
    {
        return LEAD_TEST_CAP_VOLTAGE;
    }
    return mmc_expected_voltage_for_module(module_ID);
}

static uint32_t voltage_to_centi_volts(float voltage)
{
    if (voltage <= 0.0F)
    {
        return 0U;
    }
    return static_cast<uint32_t>(voltage * 100.0F + 0.5F);
}

static void reset_debug_counters(void)
{
    counter_timer = 0;
    counter_receive = 0;
    unexpected_sender_count = 0;

    for (uint8_t i = 0; i < MMC_SM_COUNT; ++i)
    {
        rx_count_by_module[i] = 0;
        voltage_error_count[i] = 0;
        last_voltage_raw_by_module[i] = 0;
    }
}

static void apply_mode_outputs(void)
{
    if (mode == POWERMODE)
    {
        if (!master)
        {
            spin.led.turnOn();
        }
    }
    else
    {
        spin.led.turnOff();
        spin.gpio.resetPin(CONTROL_TASK_DEBUG_GPIO);
    }
}

static void set_mode(serial_interface_menu_mode new_mode)
{
    if (mode == new_mode)
    {
        return;
    }

    if (new_mode == POWERMODE)
    {
        reset_debug_counters();
        send_idle = false;
        idle_debug_pending = false;
    }
    else
    {
        idle_debug_pending = true;
    }

    mode = new_mode;
    apply_mode_outputs();
}

static void fill_local_payload(MMC_frame_t &frame, uint8_t sender_id, uint8_t status_code)
{
    mmc_frame_set_sm_identifier(frame, sender_id);
    mmc_frame_set_upper_arm_flag(frame, mmc_is_upper_arm_module(sender_id));
    mmc_frame_set_status_code(frame, status_code);
    mmc_frame_set_voltage_raw(frame, mmc_encode_voltage(mmc_local_test_voltage()));
    mmc_frame_set_current_raw(frame, mmc_encode_current(0.0F));
}

static void print_menu(void)
{
    printk(" ______________________________________________ \n"
           "|        ---- MENU test RS485 MMC ----         |\n"
           "|     press h : help                           |\n"
           "|     press i : idle mode                      |\n"
           "|     press p : power mode / start test        |\n"
           "|______________________________________________|\n\n");
}

static void print_follower_debug_report(void)
{
    const uint32_t cycles = counter_timer;
    const uint32_t rx = counter_receive;
    const uint32_t missing = (cycles > rx) ? (cycles - rx) : 0U;
    const uint32_t local_voltage_centi = voltage_to_centi_volts(mmc_local_test_voltage());

    printk("\n=== RS485 test module SM%u ===\n", module_ID);
    printk("frame=%u bytes, gpio_debug=Spin %u, tension_fixee=%" PRIu32 ".%02" PRIu32 " V\n",
           static_cast<unsigned int>(MMC_FRAME_SIZE),
           CONTROL_TASK_DEBUG_GPIO,
           local_voltage_centi / 100U,
           local_voltage_centi % 100U);
    printk("control_counter=%" PRIu32 ", rx_counter=%" PRIu32 ", misses=%" PRIu32 "\n",
           cycles,
           rx,
           missing);
}

static void print_lead_debug_report(void)
{
    const uint32_t cycles = counter_timer;
    const uint32_t rx = counter_receive;
    const uint32_t expected_rx_total = cycles * MMC_SM_COUNT;
    const uint32_t missing_rx_total =
        (expected_rx_total > rx) ? (expected_rx_total - rx) : 0U;
    const uint32_t extra_rx_total =
        (rx > expected_rx_total) ? (rx - expected_rx_total) : 0U;

    printk("\n=== RS485 test lead ===\n");
    printk("frame=%u bytes, gpio_debug=Spin %u\n",
           static_cast<unsigned int>(MMC_FRAME_SIZE),
           CONTROL_TASK_DEBUG_GPIO);
    printk("control_counter=%" PRIu32 ", rx_counter=%" PRIu32
           ", expected_rx=%" PRIu32 ", misses=%" PRIu32 ", extra=%" PRIu32
           ", unexpected_sender=%" PRIu32 "\n",
           cycles,
           rx,
           expected_rx_total,
           missing_rx_total,
           extra_rx_total,
           unexpected_sender_count);

    bool any_fault = (missing_rx_total != 0U) || (extra_rx_total != 0U) ||
                     (unexpected_sender_count != 0U);

    printk("Cartes en defaut:");
    for (uint8_t sm_id = MMC_SM_FIRST; sm_id <= MMC_SM_LAST; ++sm_id)
    {
        const uint8_t index = sm_id - MMC_SM_FIRST;
        const uint32_t rx_by_module = rx_count_by_module[index];
        const uint32_t missing_by_module =
            (cycles > rx_by_module) ? (cycles - rx_by_module) : 0U;
        const uint32_t value_errors = voltage_error_count[index];
        const bool module_fault =
            (missing_by_module != 0U) || (value_errors != 0U);

        if (module_fault)
        {
            any_fault = true;
            printk(" SM%u", sm_id);
        }
    }
    if (!any_fault)
    {
        printk(" aucune");
    }
    printk("\n");

    for (uint8_t sm_id = MMC_SM_FIRST; sm_id <= MMC_SM_LAST; ++sm_id)
    {
        const uint8_t index = sm_id - MMC_SM_FIRST;
        const uint32_t expected_voltage_centi =
            voltage_to_centi_volts(mmc_expected_voltage_for_module(sm_id));
        const uint32_t received_voltage_centi =
            voltage_to_centi_volts(mmc_decode_voltage(last_voltage_raw_by_module[index]));
        const uint32_t rx_by_module = rx_count_by_module[index];
        const uint32_t missing_by_module =
            (cycles > rx_by_module) ? (cycles - rx_by_module) : 0U;
        const uint32_t value_errors = voltage_error_count[index];
        const bool ok = (missing_by_module == 0U) && (value_errors == 0U);

        printk("SM%u %s: rx=%" PRIu32 "/%" PRIu32
               ", miss=%" PRIu32 ", attendu=%" PRIu32 ".%02" PRIu32
               " V, recu=%" PRIu32 ".%02" PRIu32 " V, erreurs_val=%" PRIu32 "\n",
               sm_id,
               ok ? "OK" : "KO",
               rx_by_module,
               cycles,
               missing_by_module,
               expected_voltage_centi / 100U,
               expected_voltage_centi % 100U,
               received_voltage_centi / 100U,
               received_voltage_centi % 100U,
               value_errors);
    }
}

void reception_function(void)
{
    memcpy(&dataRX_mmc, buffer_rx, sizeof(dataRX_mmc));

    const uint8_t sender_id = mmc_frame_get_sm_identifier(dataRX_mmc);
    const uint8_t status_code = mmc_frame_get_status_code(dataRX_mmc);
    const bool power_frame = (status_code == POWER);

    if (master)
    {
        if ((sender_id >= MMC_SM_FIRST) && (sender_id <= MMC_SM_LAST))
        {
            const uint8_t index = sender_id - MMC_SM_FIRST;
            const uint16_t voltage_raw = mmc_frame_get_voltage_raw(dataRX_mmc);

            last_voltage_raw_by_module[index] = voltage_raw;

            if (power_frame)
            {
                counter_receive++;
                rx_count_by_module[index]++;

                if (voltage_raw != mmc_encode_voltage(mmc_expected_voltage_for_module(sender_id)))
                {
                    voltage_error_count[index]++;
                }
            }
        }
        else if (sender_id != MMC_LEAD)
        {
            unexpected_sender_count++;
        }

        return;
    }

    if (sender_id == MMC_LEAD)
    {
        set_mode(power_frame ? POWERMODE : IDLEMODE);
    }

    if (sender_id == static_cast<uint8_t>(module_ID - 1U))
    {
        if (power_frame)
        {
            counter_receive++;
        }

        dataTX_mmc = dataRX_mmc;
        fill_local_payload(dataTX_mmc, module_ID, status_code);
        memcpy(buffer_tx, &dataTX_mmc, sizeof(dataTX_mmc));
        communication.rs485.startTransmission();
    }
}

void setup_routine(void);
void loop_background_task(void);
void loop_communication_task(void);
void loop_critical_task(void);

void setup_routine(void)
{
    const uint32_t board_uid = read_board_uid();

    module_ID = detect_module_id();
    master = (module_ID == MMC_LEAD);

    printk("Board UID: 0x%08" PRIX32 " | role: %s | module_id: %u | frame: %u bytes\n",
           board_uid,
           master ? "LEAD" : "MODULE",
           module_ID,
           static_cast<unsigned int>(MMC_FRAME_SIZE));

    spin.gpio.configurePin(CONTROL_TASK_DEBUG_GPIO, OUTPUT);
    spin.gpio.resetPin(CONTROL_TASK_DEBUG_GPIO);
    spin.led.turnOff();

    communication.rs485.configure(buffer_tx,
                                  buffer_rx,
                                  sizeof(buffer_rx),
                                  reception_function,
                                  SPEED_20M);

    const uint32_t background_task_number = task.createBackground(loop_background_task);
    task.createCritical(loop_critical_task, CONTROL_TASK_PERIOD_US);
    const uint32_t communication_task_number = task.createBackground(loop_communication_task);

    task.startBackground(background_task_number);
    task.startCritical();
    task.startBackground(communication_task_number);

    print_menu();
}

void loop_communication_task(void)
{
    received_serial_char = console_getchar();

    switch (received_serial_char)
    {
    case 'h':
        print_menu();
        break;
    case 'i':
        printk("idle mode\n");
        set_mode(IDLEMODE);
        break;
    case 'p':
        printk("power mode\n");
        set_mode(POWERMODE);
        break;
    default:
        break;
    }
}

void loop_background_task(void)
{
    if ((mode == IDLEMODE) && idle_debug_pending)
    {
        if (master)
        {
            print_lead_debug_report();
        }
        else
        {
            print_follower_debug_report();
        }
        idle_debug_pending = false;
    }

    task.suspendBackgroundMs(BACKGROUND_TASK_PERIOD_MS);
}

void loop_critical_task(void)
{
    if (mode == POWERMODE)
    {
        counter_timer++;
        spin.gpio.togglePin(CONTROL_TASK_DEBUG_GPIO);

        if (master)
        {
            dataTX_mmc.sm_insertion.raw = 0U;
            dataTX_mmc.status.raw = 0U;
            fill_local_payload(dataTX_mmc, MMC_LEAD, POWER);
            memcpy(buffer_tx, &dataTX_mmc, sizeof(dataTX_mmc));
            communication.rs485.startTransmission();
        }
        else{
            spin.led.turnOn();
        }
    }
    else if (master && !send_idle)
    {
        dataTX_mmc.sm_insertion.raw = 0U;
        dataTX_mmc.status.raw = 0U;
        fill_local_payload(dataTX_mmc, MMC_LEAD, IDLE);
        memcpy(buffer_tx, &dataTX_mmc, sizeof(dataTX_mmc));
        communication.rs485.startTransmission();
        send_idle = true;
    }
    else if (!master && mode == IDLEMODE)
    {
        spin.led.turnOff();
    }
}

int main(void)
{
    setup_routine();
    return 0;
}
