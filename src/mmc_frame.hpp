/*
 * MMC RS485 frame helpers.
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "mmc_config.hpp"

#ifndef __packed
#define __packed __attribute__((packed))
#endif

constexpr uint8_t MMC_STATUS_CODE_BITS = 3;
constexpr uint32_t MMC_STATUS_CODE_MASK = (1UL << MMC_STATUS_CODE_BITS) - 1U;
constexpr uint32_t MMC_STATUS_UPPER_ARM_MASK = (1UL << MMC_STATUS_CODE_BITS);

constexpr float MMC_CAP_VOLTAGE_SCALE = MMC_VCAP_EXPECTED_V * 2.0F;
constexpr float MMC_ARM_CURRENT_SCALE = MMC_I_EXPECTED_A * 2.0F;
constexpr float MMC_ARM_CURRENT_OFFSET = MMC_I_EXPECTED_A;

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
} __packed;

typedef MMC_frame MMC_frame_t;

static inline uint16_t mmc_encode_voltage(float voltage)
{
    int32_t raw = static_cast<int32_t>((voltage * 4095.0F) / MMC_CAP_VOLTAGE_SCALE);
    if (raw < 0) raw = 0;
    if (raw > 0x0FFF) raw = 0x0FFF;
    return static_cast<uint16_t>(raw);
}

static inline float mmc_decode_voltage(uint16_t raw)
{
    return (MMC_CAP_VOLTAGE_SCALE * static_cast<float>(raw & 0x0FFFU)) / 4095.0F;
}

static inline uint16_t mmc_encode_current(float current)
{
    const float shifted = current + MMC_ARM_CURRENT_OFFSET;
    int32_t raw = static_cast<int32_t>((shifted * 4095.0F) / MMC_ARM_CURRENT_SCALE);
    if (raw < 0) raw = 0;
    if (raw > 0x0FFF) raw = 0x0FFF;
    return static_cast<uint16_t>(raw);
}

static inline float mmc_decode_current(uint16_t raw)
{
    return ((MMC_ARM_CURRENT_SCALE * static_cast<float>(raw & 0x0FFFU)) / 4095.0F)
           - MMC_ARM_CURRENT_OFFSET;
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
    frame.status.raw &= static_cast<uint8_t>(~MMC_STATUS_CODE_MASK);
    frame.status.raw |= static_cast<uint8_t>(status_code & MMC_STATUS_CODE_MASK);
}

static inline uint8_t mmc_frame_get_status_code(const MMC_frame_t &frame)
{
    return static_cast<uint8_t>(frame.status.raw & MMC_STATUS_CODE_MASK);
}

static inline void mmc_frame_set_upper_arm_flag(MMC_frame_t &frame, bool is_upper_arm)
{
    if (is_upper_arm) frame.status.raw |= static_cast<uint8_t>(MMC_STATUS_UPPER_ARM_MASK);
    else frame.status.raw &= static_cast<uint8_t>(~MMC_STATUS_UPPER_ARM_MASK);
}

static inline bool mmc_frame_is_upper_arm(const MMC_frame_t &frame)
{
    return (frame.status.raw & MMC_STATUS_UPPER_ARM_MASK) != 0U;
}

static inline void mmc_frame_set_sm_inserted(MMC_frame_t &frame, uint8_t sm_index, bool inserted)
{
    if (sm_index < MMC_SM_FIRST || sm_index > MMC_SM_LAST) return;
    const uint8_t shift = static_cast<uint8_t>(sm_index - MMC_SM_FIRST);
    const uint16_t mask = static_cast<uint16_t>(1U << shift);
    if (inserted) frame.sm_insertion.raw |= mask;
    else frame.sm_insertion.raw &= static_cast<uint16_t>(~mask);
}

static inline bool mmc_frame_get_sm_inserted(const MMC_frame_t &frame, uint8_t sm_index)
{
    if (sm_index < MMC_SM_FIRST || sm_index > MMC_SM_LAST) return false;
    const uint8_t shift = static_cast<uint8_t>(sm_index - MMC_SM_FIRST);
    const uint16_t mask = static_cast<uint16_t>(1U << shift);
    return (frame.sm_insertion.raw & mask) != 0U;
}

static inline bool mmc_is_upper_arm_module(uint8_t id)
{
    if (id == MMC_LEAD) return true;
    if (id < MMC_SM_FIRST || id > MMC_SM_LAST) return false;
    const uint8_t offset = static_cast<uint8_t>(id - MMC_SM_FIRST);
    return offset < MMC_ARM_MODULES;
}
