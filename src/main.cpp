/*
 * MMC Local Consensus Control - OwnTech/Twist firmware prototype
 *
 * Copyright (c) 2026 Zaid Jabbar
 * SPDX-License-Identifier: MIT
 *
 * This file keeps the OwnTech-style single-entry layout used in the reference
 * MMC firmware while integrating the Simulink neighbor-consensus / Local
 * Consensus balancing idea developed in this repository.
 */

/**
 * @brief MMC arm firmware prototype for Spin + Twist.
 *
 * Architecture:
 * - one lead board generates the NLM insertion command,
 * - follower boards relay RS485 frames in a ring,
 * - every frame carries capacitor voltage, arm current, status, and insertion bits,
 * - the lead computes Local Consensus priorities from the measured neighbor voltages,
 * - each follower applies the insertion bit assigned to its module ID.
 *
 * This is a bridge between the Simulink model and the real OwnTech/Twist setup.
 */

#if !defined(MMC_HOST_BUILD)
/* -------------- OWNTECH APIs ---------------------------------- */
#include "SpinAPI.h"
#include "TaskAPI.h"
#include "ShieldAPI.h"
#include "CommunicationAPI.h"

/* -------------- OWNTECH Libraries ----------------------------- */
#include "filters.h"
#include "trigo.h"
#include "pid.h"
#include "pr.h"
#include "arm_math_types.h"
#include <ScopeMimicry.h>

/* -------------- Zephyr includes ------------------------------- */
#include "zephyr/console/console.h"
#else
#include <cstdio>
using float32_t = float;
#endif

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>

#include "mmc_config.hpp"
#include "mmc_frame.hpp"
#include "mmc_local_consensus.hpp"

/* -------------- Board identification placeholders ------------- */
constexpr uint32_t UID_MMC_LEAD_BOARD = 0x002B002A;
constexpr uint32_t UID_MMC_SM1_BOARD  = 0x00330054;
constexpr uint32_t UID_MMC_SM2_BOARD  = 0x0033004B;
constexpr uint32_t UID_MMC_SM3_BOARD  = 0x00330049;
constexpr uint32_t UID_MMC_SM4_BOARD  = 0x0033004C;
constexpr uint32_t UID_MMC_SM5_BOARD  = 0x0031001B;
constexpr uint32_t UID_MMC_SM6_BOARD  = 0x11118888;
constexpr uint32_t UID_MMC_SM7_BOARD  = 0x11119999;
constexpr uint32_t UID_MMC_SM8_BOARD  = 0x1111AAA0;
constexpr uint32_t UID_MMC_SM9_BOARD  = 0x1111BBB1;
constexpr uint32_t UID_MMC_SM10_BOARD = 0x1111CCC2;

static const float32_t Ts = static_cast<float32_t>(MMC_CONTROL_TASK_PERIOD_US) * 1e-6F;
static const float32_t w0 = 2.0F * MMC_PI_F * MMC_OUTPUT_FREQUENCY_HZ;

static uint32_t read_board_uid()
{
#if defined(MMC_HOST_BUILD)
    return UID_MMC_SM1_BOARD;
#else
    static volatile uint32_t *const uid0 =
        reinterpret_cast<volatile uint32_t *>(0x1FFF7590UL);
    return *uid0;
#endif
}

static uint8_t detect_module_id()
{
    switch (read_board_uid())
    {
    case UID_MMC_LEAD_BOARD: return MMC_LEAD;
    case UID_MMC_SM1_BOARD:  return MMC_SM1;
    case UID_MMC_SM2_BOARD:  return MMC_SM2;
    case UID_MMC_SM3_BOARD:  return MMC_SM3;
    case UID_MMC_SM4_BOARD:  return MMC_SM4;
    case UID_MMC_SM5_BOARD:  return MMC_SM5;
    case UID_MMC_SM6_BOARD:  return MMC_SM6;
    case UID_MMC_SM7_BOARD:  return MMC_SM7;
    case UID_MMC_SM8_BOARD:  return MMC_SM8;
    case UID_MMC_SM9_BOARD:  return MMC_SM9;
    case UID_MMC_SM10_BOARD: return MMC_SM10;
    default:                return MMC_SM1;
    }
}

/* -------------- Runtime variables ----------------------------- */
uint8_t module_ID = detect_module_id();
static bool master = false;
static bool send_idle = false;
static bool pwm_enable = false;
static bool enable_acq = false;
static bool is_downloading = false;

static uint8_t module_command = 0U;
static uint8_t module_command_past = 0U;
static uint8_t received_serial_char = 0U;
static int8_t CommTask_num = -1;

enum serial_interface_menu_mode
{
    IDLEMODE = 0,
    POWERMODE = 1,
};

static serial_interface_menu_mode mode = IDLEMODE;

static MMC_frame_t dataTX_mmc{};
static MMC_frame_t dataRX_mmc{};

constexpr size_t MMC_FRAME_SIZE = sizeof(MMC_frame_t);
static uint8_t buffer_tx[MMC_FRAME_SIZE] = {0U};
static uint8_t buffer_rx[MMC_FRAME_SIZE] = {0U};

static float32_t MMC_capacitor_voltage[MMC_SM_COUNT] = {0.0F};
static float32_t MMC_arm_current[MMC_SM_COUNT] = {0.0F};

static float32_t Cap_voltage = 0.0F;
static float32_t Arm_current = 0.0F;
static float32_t V_high = 0.0F;
static float32_t I1_low_value = 0.0F;
static uint32_t counter_timer = 0U;
static uint32_t counter_receive = 0U;
static uint32_t critical_task_timer = 0U;

/* NLM / Local Consensus balancing state. */
static float32_t modulation_index = 1.0F;
static float32_t amplitude_offset = 1.0F;
static float32_t angle = 0.0F;
static float32_t modulation_signal_upper = 0.5F;
static float32_t modulation_signal_lower = 0.5F;
static float32_t number_of_connected_submodules_upper_arm = 0.0F;
static float32_t number_of_connected_submodules_lower_arm = 0.0F;
static float32_t i_upper_arm = 0.0F;
static float32_t i_lowfilter_value = 0.0F;
static float32_t local_consensus_score[MMC_ARM_MODULES] = {0.0F};
static uint8_t gate_upper[MMC_ARM_MODULES] = {0U};

static float32_t g_u_1 = 0.0F;
static float32_t g_u_2 = 0.0F;
static float32_t g_u_3 = 0.0F;
static float32_t g_u_4 = 0.0F;
static float32_t g_u_5 = 0.0F;

#if !defined(MMC_HOST_BUILD)
static LowPassFirstOrderFilter i_low_filter(Ts, 180e-6F);
static const uint16_t NB_DATAS = 1028;
static ScopeMimicry scope(NB_DATAS, 12);
static uint32_t scope_timer = 0U;
static uint32_t scope_period = 1U;
#endif

/* -------------- Function declarations ------------------------- */
void setup_routine();
void loop_background_task();
void loop_critical_task();
void loop_communication_task();
void reception_function(void);

/* -------------- Math wrappers --------------------------------- */
static inline float32_t mmc_modulo_2pi(float32_t value)
{
#if defined(MMC_HOST_BUILD)
    float32_t out = fmodf(value, 2.0F * MMC_PI_F);
    if (out < 0.0F) out += 2.0F * MMC_PI_F;
    return out;
#else
    return ot_modulo_2pi(value);
#endif
}

static inline float32_t mmc_sin(float32_t value)
{
#if defined(MMC_HOST_BUILD)
    return sinf(value);
#else
    return ot_sin(value);
#endif
}

static inline uint8_t mmc_round_to_uint8(float32_t value)
{
    if (value <= 0.0F) return 0U;
    if (value >= static_cast<float32_t>(MMC_ARM_MODULES)) return MMC_ARM_MODULES;
    return static_cast<uint8_t>(value + 0.5F);
}

/* -------------- Local Consensus gate assignment --------------- */
static void mmc_assign_upper_arm_gates_local_consensus(uint8_t n_insert)
{
    float32_t vc_upper[MMC_ARM_MODULES] = {0.0F};

    for (uint8_t i = 0; i < MMC_ARM_MODULES; ++i)
    {
        vc_upper[i] = MMC_capacitor_voltage[i];
    }

    for (uint8_t i = 0; i < MMC_ARM_MODULES; ++i)
    {
        local_consensus_score[i] = mmc_local_consensus_priority(
            i,
            vc_upper,
            MMC_ARM_MODULES,
            i_upper_arm);
    }

    mmc_select_top_consensus_priorities(
        local_consensus_score,
        MMC_ARM_MODULES,
        n_insert,
        gate_upper);

    g_u_1 = static_cast<float32_t>(gate_upper[0]);
    g_u_2 = static_cast<float32_t>(gate_upper[1]);
    g_u_3 = static_cast<float32_t>(gate_upper[2]);
    g_u_4 = static_cast<float32_t>(gate_upper[3]);
    g_u_5 = static_cast<float32_t>(gate_upper[4]);
}

/* -------------- Measurement and safety ------------------------ */
static uint8_t mmc_local_status_code()
{
    if (Cap_voltage > MMC_OVERVOLTAGE_LIMIT_V) return OVER_VOLTAGE;
    if (Cap_voltage < MMC_UNDERVOLTAGE_LIMIT_V) return UNDER_VOLTAGE;
    if (fabsf(Arm_current) > MMC_OVERCURRENT_LIMIT_A) return OVER_CURRENT;
    return (mode == POWERMODE) ? POWER : IDLE;
}

static void update_measurements(void)
{
#if !defined(MMC_HOST_BUILD)
    float32_t latest = shield.sensors.getLatestValue(V_HIGH);
    if (latest != NO_VALUE)
    {
        V_high = latest;
        Cap_voltage = V_high;
    }

    latest = shield.sensors.getLatestValue(I1_LOW);
    if (latest != NO_VALUE)
    {
        I1_low_value = latest;
        Arm_current = -I1_low_value;
    }
#else
    Cap_voltage = MMC_capacitor_voltage[0];
    Arm_current = 1.0F;
#endif
}

/* -------------- Scope helpers --------------------------------- */
#if !defined(MMC_HOST_BUILD)
static bool scope_trigger()
{
    return enable_acq;
}

static void dump_scope_datas(ScopeMimicry &scope_ref)
{
    uint8_t *buffer = scope_ref.get_buffer();
    uint16_t buffer_size = scope_ref.get_buffer_size() >> 2;

    printk("begin record\n");
    printk("#");
    for (uint16_t k = 0; k < scope_ref.get_nb_channel(); ++k)
    {
        printk("%s,", scope_ref.get_channel_name(k));
    }
    printk("\n");
    printk("# %d\n", scope_ref.get_final_idx());

    for (uint16_t k = 0; k < buffer_size; ++k)
    {
        printk("%08x\n", *((uint32_t *)buffer + k));
        task.suspendBackgroundUs(100);
    }

    printk("end record\n");
}
#endif

/* -------------- RS485 reception and relay --------------------- */
void reception_function(void)
{
    dataRX_mmc = *(MMC_frame_t *)buffer_rx;
    const uint8_t sender_id = mmc_frame_get_sm_identifier(dataRX_mmc);
    const uint8_t status_code = mmc_frame_get_status_code(dataRX_mmc);

    if (module_ID == MMC_LEAD)
    {
        if ((sender_id >= MMC_SM_FIRST) && (sender_id <= MMC_SM_LAST))
        {
            const uint8_t index = static_cast<uint8_t>(sender_id - MMC_SM_FIRST);
            MMC_capacitor_voltage[index] =
                mmc_decode_voltage(mmc_frame_get_voltage_raw(dataRX_mmc));
            MMC_arm_current[index] =
                mmc_decode_current(mmc_frame_get_current_raw(dataRX_mmc));

            if ((status_code >= LEAD_ERROR) && (mode != IDLEMODE))
            {
                mode = IDLEMODE;
                send_idle = false;
            }
        }
    }
    else
    {
        if (sender_id == MMC_LEAD)
        {
            module_command = static_cast<uint8_t>(
                mmc_frame_get_sm_inserted(dataRX_mmc, module_ID));
            module_command_past = module_command;

            mode = (status_code == POWER) ? POWERMODE : IDLEMODE;
            pwm_enable = (mode == POWERMODE) && (module_command != 0U);
        }

        if (sender_id == static_cast<uint8_t>(module_ID - 1U))
        {
            dataTX_mmc = dataRX_mmc;
            mmc_frame_set_sm_identifier(dataTX_mmc, module_ID);
            mmc_frame_set_upper_arm_flag(dataTX_mmc, mmc_is_upper_arm_module(module_ID));
            mmc_frame_set_voltage_raw(dataTX_mmc, mmc_encode_voltage(Cap_voltage));
            mmc_frame_set_current_raw(dataTX_mmc, mmc_encode_current(Arm_current));
            mmc_frame_set_status_code(dataTX_mmc, mmc_local_status_code());

            memcpy(buffer_tx, &dataTX_mmc, sizeof(dataTX_mmc));
#if !defined(MMC_HOST_BUILD)
            communication.rs485.startTransmission();
#endif
        }
    }

    counter_receive++;
}

/* -------------- Setup routine --------------------------------- */
void setup_routine()
{
    const uint32_t board_uid = read_board_uid();
    master = (module_ID == MMC_LEAD);

#if !defined(MMC_HOST_BUILD)
    printk("Board UID: 0x%08" PRIX32 "\n", board_uid);

    shield.power.initBuck(ALL);
    shield.sensors.enableDefaultTwistSensors();

    shield.power.disconnectCapacitor(LEG1);
    shield.power.disconnectCapacitor(LEG2);
    shield.power.setDutyCycleMax(ALL, 1.0F);
    shield.power.setDutyCycleMin(ALL, 0.0F);

    uint32_t background_task_number = task.createBackground(loop_background_task);
    task.createCritical(loop_critical_task, MMC_CONTROL_TASK_PERIOD_US);

    task.startBackground(background_task_number);
    task.startCritical();

    CommTask_num = task.createBackground(loop_communication_task);
    task.startBackground(CommTask_num);

    communication.rs485.configure(buffer_tx, buffer_rx, sizeof(buffer_rx),
                                  reception_function,
                                  SPEED_20M);

    if (master)
    {
        communication.sync.initMaster();
        scope.connectChannel(modulation_signal_upper, "m_u");
        scope.connectChannel(number_of_connected_submodules_upper_arm, "N_u");
        scope.connectChannel(g_u_1, "g_u_1");
        scope.connectChannel(g_u_2, "g_u_2");
        scope.connectChannel(g_u_3, "g_u_3");
        scope.connectChannel(g_u_4, "g_u_4");
        scope.connectChannel(g_u_5, "g_u_5");
        scope.connectChannel(MMC_capacitor_voltage[0], "v_c_1");
        scope.connectChannel(MMC_capacitor_voltage[1], "v_c_2");
        scope.connectChannel(MMC_capacitor_voltage[2], "v_c_3");
        scope.connectChannel(MMC_capacitor_voltage[3], "v_c_4");
        scope.connectChannel(MMC_capacitor_voltage[4], "v_c_5");
        scope.connectChannel(i_lowfilter_value, "i_u_f");
        scope.set_trigger(&scope_trigger);
        scope.set_delay(0.0F);
        scope.start();
    }
    else
    {
        communication.sync.initSlave();
    }
#endif

    (void)board_uid;
}

/* -------------- Serial command task --------------------------- */
void loop_communication_task()
{
#if !defined(MMC_HOST_BUILD)
    received_serial_char = console_getchar();

    switch (received_serial_char)
    {
    case 'h':
        printk(" ________________________________________ \n"
               "| ---- MMC Local Consensus menu ------- |\n"
               "| press i : idle mode                   |\n"
               "| press p : power mode                  |\n"
               "| press r : record scope data           |\n"
               "| press a : toggle scope trigger        |\n"
               "|_______________________________________|\n\n");
        break;
    case 'i':
        printk("idle mode\n");
        mode = IDLEMODE;
        send_idle = true;
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
        enable_acq = !enable_acq;
        break;
    default:
        break;
    }
#endif
}

/* -------------- Background task ------------------------------- */
void loop_background_task()
{
#if !defined(MMC_HOST_BUILD)
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
        else if (mode == POWERMODE)
        {
            spin.led.toggle();
        }
    }

    task.suspendBackgroundMs(2000);
#endif
}

/* -------------- Lead command-frame construction --------------- */
static void mmc_build_lead_command_frame(uint8_t n_insert)
{
    mmc_assign_upper_arm_gates_local_consensus(n_insert);

    dataTX_mmc.sm_insertion.raw = 0U;
    dataTX_mmc.status.raw = 0U;

    for (uint8_t counter = 0; counter < MMC_ARM_MODULES; ++counter)
    {
        mmc_frame_set_sm_inserted(dataTX_mmc,
                                  static_cast<uint8_t>(MMC_SM1 + counter),
                                  gate_upper[counter] != 0U);
    }

    mmc_frame_set_status_code(dataTX_mmc, (mode == POWERMODE) ? POWER : IDLE);
    mmc_frame_set_upper_arm_flag(dataTX_mmc, true);
    mmc_frame_set_sm_identifier(dataTX_mmc, MMC_LEAD);
    mmc_frame_set_voltage_raw(dataTX_mmc, mmc_encode_voltage(Cap_voltage));
    mmc_frame_set_current_raw(dataTX_mmc, mmc_encode_current(Arm_current));
    memcpy(buffer_tx, &dataTX_mmc, sizeof(dataTX_mmc));
}

/* -------------- Critical control task ------------------------- */
void loop_critical_task()
{
    update_measurements();
    critical_task_timer++;

    if (mode == POWERMODE)
    {
        if (module_ID == MMC_LEAD)
        {
            angle += w0 * Ts;
            angle = mmc_modulo_2pi(angle);

            modulation_index = 1.0F;
            modulation_signal_upper =
                (amplitude_offset + modulation_index * mmc_sin(angle)) / 2.0F;
            modulation_signal_lower =
                (amplitude_offset - modulation_index * mmc_sin(angle)) / 2.0F;

            modulation_signal_upper = mmc_clamp01(modulation_signal_upper);
            modulation_signal_lower = mmc_clamp01(modulation_signal_lower);

            const uint8_t n_insert_upper = mmc_round_to_uint8(
                static_cast<float32_t>(MMC_ARM_MODULES) * modulation_signal_upper);
            const uint8_t n_insert_lower = mmc_round_to_uint8(
                static_cast<float32_t>(MMC_ARM_MODULES) * modulation_signal_lower);

            number_of_connected_submodules_upper_arm = static_cast<float32_t>(n_insert_upper);
            number_of_connected_submodules_lower_arm = static_cast<float32_t>(n_insert_lower);

            i_upper_arm = MMC_arm_current[0];
#if !defined(MMC_HOST_BUILD)
            i_lowfilter_value = i_low_filter.calculateWithReturn(i_upper_arm);
#else
            i_lowfilter_value = i_upper_arm;
#endif
            i_upper_arm = i_lowfilter_value;

            mmc_build_lead_command_frame(n_insert_upper);

#if !defined(MMC_HOST_BUILD)
            communication.rs485.startTransmission();

            if (scope_timer >= scope_period)
            {
                scope.acquire();
                scope_timer = 0U;
            }
            else
            {
                scope_timer++;
            }
#endif

            (void)n_insert_lower;
        }
        else
        {
#if !defined(MMC_HOST_BUILD)
            const float32_t follower_duty = (module_command != 0U) ? 1.0F : 0.0F;
            shield.power.setDutyCycle(LEG1, follower_duty);
#endif
        }
    }
    else
    {
        pwm_enable = false;
        module_command = 0U;
#if !defined(MMC_HOST_BUILD)
        shield.power.setDutyCycle(LEG1, 0.0F);
#endif
    }

    counter_timer++;
}

#if defined(MMC_HOST_BUILD)
int main()
{
    module_ID = MMC_LEAD;
    mode = POWERMODE;

    for (uint8_t i = 0; i < MMC_ARM_MODULES; ++i)
    {
        MMC_capacitor_voltage[i] = 78.0F + static_cast<float32_t>(i);
        MMC_arm_current[i] = 1.0F;
    }

    setup_routine();
    loop_critical_task();

    std::printf("Local Consensus gates: %u %u %u %u %u\n",
                gate_upper[0], gate_upper[1], gate_upper[2], gate_upper[3], gate_upper[4]);
    return 0;
}
#endif
