/*
 * MMC configuration constants for OwnTech/Twist firmware prototypes.
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>

#ifndef MMC_CONFIG_HPP
#define MMC_CONFIG_HPP

/* Module identifiers */
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

/* Runtime status values */
#define IDLE          0
#define POWER         1
#define LEAD_ERROR    2
#define OVER_VOLTAGE  3
#define UNDER_VOLTAGE 4
#define OVER_CURRENT  5

constexpr uint8_t MMC_SM_COUNT = 10;
constexpr uint8_t MMC_SM_FIRST = MMC_SM1;
constexpr uint8_t MMC_SM_LAST  = MMC_SM10;
constexpr uint8_t MMC_ARM_MODULES = 5;

constexpr float MMC_OUTPUT_FREQUENCY_HZ = 50.0F;
constexpr float MMC_VCAP_EXPECTED_V = 80.0F;
constexpr float MMC_I_EXPECTED_A = 10.0F;
constexpr float MMC_OVERVOLTAGE_LIMIT_V = 95.0F;
constexpr float MMC_UNDERVOLTAGE_LIMIT_V = 5.0F;
constexpr float MMC_OVERCURRENT_LIMIT_A = 8.0F;
constexpr float MMC_PI_F = 3.14159265358979323846F;
constexpr uint32_t MMC_CONTROL_TASK_PERIOD_US = 100U;

/* Local-rank / neighbor-consensus gains */
constexpr float MMC_K_V = 0.2F;
constexpr float MMC_K_RANK = 0.02F;
constexpr float MMC_VOLTAGE_DEADBAND_V = 0.1F;
constexpr float MMC_CURRENT_SCALE_A = 1.0F;

#endif /* MMC_CONFIG_HPP */
