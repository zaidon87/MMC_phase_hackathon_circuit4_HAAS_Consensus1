/*
 * MMC Local-Rank Neighbor Control - OwnTech-style embedded entry point
 *
 * Copyright (c) 2026 Zaid Jabbar
 * SPDX-License-Identifier: MIT
 *
 * This file is a hardware-oriented skeleton prepared from the Simulink
 * neighbor-consensus/local-rank MMC controller. It follows the layout used in
 * OwnTech Power API projects: OwnTech API includes, module definitions,
 * compact communication helpers, setup routine, background task, and critical
 * task.
 */

/**
 * @brief OwnTech/Twist-oriented MMC arm controller skeleton.
 *
 * The current Simulink controller uses neighbor capacitor voltages to correct
 * the submodule duty/modulation index. This C++ file prepares the same logic
 * for future embedded validation on Spin + Twist hardware.
 */

#if !defined(MMC_HOST_BUILD)
/* -------------- OWNTECH APIs ---------------------------------- */
#include "SpinAPI.h"
#include "TaskAPI.h"
#include "ShieldAPI.h"
#include "CommunicationAPI.h"

/* -------------- OWNTECH Libraries ----------------------------- */
#include "arm_math_types.h"
#include <ScopeMimicry.h>

/* -------------- Zephyr includes ------------------------------- */
#include "zephyr/console/console.h"
#else
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
using float32_t = float;
#endif

/* -------------- MMC module identifiers ------------------------ */
#define MMC_LEAD 0
#define MMC_SM1  1
#define MMC_SM2  2
#define MMC_SM3  3
#define MMC_SM4  4
#define MMC_SM5  5
#define MMC_SM6  6
#define MMC_SM7  7
#define MMC_SM8  8
#define MMC_SM9  9
#define MMC_SM10 10

/* -------------- MMC status values ----------------------------- */
#define IDLE          0
#define POWER         1
#define LEAD_ERROR    2
#define OVER_VOLTAGE  3
#define UNDER_VOLTAGE 4
#define OVER_CURRENT  5

constexpr uint8_t MMC_SM_COUNT = 10;
constexpr uint8_t MMC_SM_FIRST = MMC_SM1;
constexpr uint8_t MMC_SM_LAST  = MMC_SM10;

/* -------------- General MMC definitions ----------------------- */
static const float32_t f0 = 50.0F;                   /* Output frequency [Hz] */
static const uint8_t total_number_of_modules_arm = 5;/* Submodules per arm */
constexpr float32_t Vcap_expected = 80.0F;           /* Expected capacitor voltage [V] */
constexpr float32_t i_expected = 10.0F;              /* Expected arm-current amplitude [A] */
constexpr float32_t overvoltage_tolerance = 95.0F;   /* Hardware safety threshold [V] */
constexpr float32_t overcurrent_tolerance = 8.0F;    /* Hardware safety threshold [A] */

/* -------------- Local-rank controller parameters -------------- */
constexpr float32_t k_v = 0.2F;
constexpr float32_t k_rank = 0.02F;
constexpr float32_t voltage_deadband = 0.1F;
constexpr float32_t current_scale = 1.0F;

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

/* -------------- Data packing helpers -------------------------- */
constexpr float32_t Cap_voltage_SCALE = Vcap_expected * 2.0F;
constexpr float32_t Arm_current_SCALE = i_expected * 2.0F;
constexpr float32_t Arm_current_OFFSET = i_expected;

static inline uint16_t mmc_encode_voltage(float32_t voltage)
{
    int32_t raw = static_cast<int32_t>((voltage * 4095.0F) / Cap_voltage_SCALE);
    if (raw < 0) raw = 0;
    if (raw > 0x0FFF) raw = 0x0FFF;
    return static_cast<uint16_t>(raw);
}

static inline float32_t mmc_decode_voltage(uint16_t raw)
{
    return (Cap_voltage_SCALE * static_cast<float32_t>(raw & 0x0FFF)) / 4095.0F;
}

static inline uint16_t mmc_encode_current(float32_t current)
{
    float32_t shifted = current + Arm_current_OFFSET;
    int32_t raw = static_cast<int32_t>((shifted * 4095.0F) / Arm_current_SCALE);
    if (raw < 0) raw = 0;
    if (raw > 0x0FFF) raw = 0x0FFF;
    return static_cast<uint16_t>(raw);
}

static inline float32_t mmc_decode_current(uint16_t raw)
{
    return ((Arm_current_SCALE * static_cast<float32_t>(raw & 0x0FFF)) / 4095.0F)
           - Arm_current_OFFSET;
}

/* -------------- MMC communication frame ----------------------- */
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
            uint16_t sm1_inserted  : 1;
            uint16_t sm2_inserted  : 1;
            uint16_t sm3_inserted  : 1;
            uint16_t sm4_inserted  : 1;
            uint16_t sm5_inserted  : 1;
            uint16_t sm6_inserted  : 1;
            uint16_t sm7_inserted  : 1;
            uint16_t sm8_inserted  : 1;
            uint16_t sm9_inserted  : 1;
            uint16_t sm10_inserted : 1;
        } bits;
    } sm_insertion;

    uint16_t capacitor_voltage_raw : 12;
    uint16_t arm_current_raw       : 12;

    union
    {
        uint8_t raw;
        struct
        {
            uint8_t status_code     : MMC_STATUS_CODE_BITS;
            uint8_t upper_arm_frame : 1;
        } bits;
    } status;

    uint8_t sm_id;
} __attribute__((packed));

using MMC_frame_t = MMC_frame;

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

static inline void mmc_frame_set_status_code(MMC_frame_t &frame, uint8_t status_code)
{
    frame.status.raw &= ~MMC_STATUS_CODE_MASK;
    frame.status.raw |= static_cast<uint8_t>(status_code & MMC_STATUS_CODE_MASK);
}

static inline uint8_t mmc_frame_get_status_code(const MMC_frame_t &frame)
{
    return static_cast<uint8_t>(frame.status.raw & MMC_STATUS_CODE_MASK);
}

static inline void mmc_frame_set_sm_inserted(MMC_frame_t &frame, uint8_t sm_index, bool inserted)
{
    if (sm_index < MMC_SM_FIRST || sm_index > MMC_SM_LAST) return;
    uint8_t shift = static_cast<uint8_t>(sm_index - MMC_SM_FIRST);
    uint16_t mask = static_cast<uint16_t>(1U << shift);
    if (inserted) frame.sm_insertion.raw |= mask;
    else frame.sm_insertion.raw &= static_cast<uint16_t>(~mask);
}

static inline bool mmc_frame_get_sm_inserted(const MMC_frame_t &frame, uint8_t sm_index)
{
    if (sm_index < MMC_SM_FIRST || sm_index > MMC_SM_LAST) return false;
    uint8_t shift = static_cast<uint8_t>(sm_index - MMC_SM_FIRST);
    uint16_t mask = static_cast<uint16_t>(1U << shift);
    return (frame.sm_insertion.raw & mask) != 0U;
}

static inline void mmc_frame_set_upper_arm_flag(MMC_frame_t &frame, bool is_upper_arm)
{
    if (is_upper_arm) frame.status.raw |= MMC_STATUS_UPPER_ARM_MASK;
    else frame.status.raw &= static_cast<uint8_t>(~MMC_STATUS_UPPER_ARM_MASK);
}

static inline bool mmc_is_upper_arm_module(uint8_t id)
{
    if (id == MMC_LEAD) return true;
    if (id < MMC_SM_FIRST || id > MMC_SM_LAST) return false;
    uint8_t offset = static_cast<uint8_t>(id - MMC_SM_FIRST);
    return offset < (MMC_SM_COUNT / 2);
}

/* -------------- Local neighbor / local-rank control ----------- */
static inline float32_t clamp01(float32_t value)
{
    if (value < 0.0F) return 0.0F;
    if (value > 1.0F) return 1.0F;
    return value;
}

static inline float32_t mmc_neighbor_consensus_duty(float32_t m_arm,
                                                    float32_t i_arm,
                                                    float32_t vc_i,
                                                    float32_t vc_prev,
                                                    float32_t vc_next)
{
    float32_t err = 0.5F * (vc_prev + vc_next) - vc_i;
    if (std::fabs(err) < voltage_deadband) err = 0.0F;

    const float32_t dir = std::tanh(i_arm / current_scale);
    const float32_t dm = k_v * err * dir;

    return clamp01(m_arm + dm);
}

static inline float32_t mmc_local_rank_duty(float32_t m_arm,
                                            float32_t i_arm,
                                            float32_t vc_i,
                                            float32_t vc_prev,
                                            float32_t vc_next)
{
    float32_t duty_consensus = mmc_neighbor_consensus_duty(m_arm, i_arm, vc_i, vc_prev, vc_next);

    float32_t rank_score = 0.0F;
    rank_score += (vc_i > vc_prev) ? 1.0F : 0.0F;
    rank_score += (vc_i > vc_next) ? 1.0F : 0.0F;

    const float32_t rank_centered = rank_score - 1.0F;
    const float32_t dir = std::tanh(i_arm / current_scale);
    const float32_t dm_rank = -k_rank * rank_centered * dir;

    return clamp01(duty_consensus + dm_rank);
}

/* -------------- Runtime variables ----------------------------- */
uint8_t module_ID = detect_module_id();
static bool master = false;
static bool pwm_enable = false;

static MMC_frame_t dataTX_mmc{};
static MMC_frame_t dataRX_mmc{};
static float32_t MMC_capacitor_voltage[MMC_SM_COUNT] = {0.0F};
static float32_t MMC_arm_current[MMC_SM_COUNT] = {0.0F};

static float32_t Cap_voltage = 0.0F;
static float32_t Arm_current = 0.0F;
static float32_t duty_command = 0.0F;
static float32_t m_arm_reference = 0.5F;

/* -------------- Function declarations ------------------------- */
void setup_routine();
void loop_background_task();
void loop_critical_task();
void loop_communication_task();

/* -------------- Setup routine --------------------------------- */
void setup_routine()
{
    master = (module_ID == MMC_LEAD);

#if !defined(MMC_HOST_BUILD)
    shield.power.initBuck(ALL);
    shield.sensors.enableDefaultTwistSensors();
    shield.power.disconnectCapacitor(LEG1);
    shield.power.disconnectCapacitor(LEG2);
    shield.power.setDutyCycleMax(ALL, 1.0F);
    shield.power.setDutyCycleMin(ALL, 0.0F);

    uint32_t background_task_number = task.createBackground(loop_background_task);
    task.createCritical(loop_critical_task, 100);

    task.startBackground(background_task_number);
    task.startCritical();
#endif
}

/* -------------- Background task ------------------------------- */
void loop_background_task()
{
#if !defined(MMC_HOST_BUILD)
    if (master) spin.led.toggle();
    task.suspendBackgroundMs(1000);
#endif
}

void loop_communication_task()
{
    /* Placeholder: configure RS485 ring exchange here. */
}

/* -------------- Critical task --------------------------------- */
void loop_critical_task()
{
#if !defined(MMC_HOST_BUILD)
    float32_t latest = shield.sensors.getLatestValue(V_HIGH);
    if (latest != NO_VALUE) Cap_voltage = latest;

    latest = shield.sensors.getLatestValue(I1_LOW);
    if (latest != NO_VALUE) Arm_current = -latest;
#endif

    const uint8_t local_index = (module_ID > 0) ? static_cast<uint8_t>(module_ID - 1) : 0;
    const uint8_t prev_index = (local_index + MMC_SM_COUNT - 1U) % MMC_SM_COUNT;
    const uint8_t next_index = (local_index + 1U) % MMC_SM_COUNT;

    const float32_t vc_prev = MMC_capacitor_voltage[prev_index];
    const float32_t vc_next = MMC_capacitor_voltage[next_index];
    const float32_t vc_i = Cap_voltage;

    duty_command = mmc_local_rank_duty(m_arm_reference, Arm_current, vc_i, vc_prev, vc_next);

#if !defined(MMC_HOST_BUILD)
    if (pwm_enable)
    {
        shield.power.setDutyCycle(LEG1, duty_command);
    }
#endif
}

#if defined(MMC_HOST_BUILD)
int main()
{
    setup_routine();
    MMC_capacitor_voltage[0] = 80.0F;
    MMC_capacitor_voltage[1] = 81.0F;
    MMC_capacitor_voltage[9] = 79.0F;
    Cap_voltage = 80.0F;
    Arm_current = 1.0F;
    loop_critical_task();
    std::printf("MMC host smoke test duty = %.4f\n", duty_command);
    return 0;
}
#endif
