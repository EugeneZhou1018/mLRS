//*******************************************************
// STM32 DroneCAN Library
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// DroneCAN Driver Library for STM32 using HAL
// for use with libcanard
//*******************************************************
// This driver follows very closely the original UAVCAN/DroneCAN driver
// for use with libcanard to make it essentially a drop in, i.e., to
// allow use without too much changes in existing code.
//*******************************************************
#ifndef STM32_DRONECAN_DRIVER_H
#define STM32_DRONECAN_DRIVER_H

#include "libcanard/canard.h"


#ifdef __cplusplus
extern "C"
{
#endif


typedef enum
{
    DC_HAL_ERROR_INVALID_ARGUMENT             = 1000,
    DC_HAL_ERROR_UNSUPPORTED_BIT_RATE         = 1001,
    DC_HAL_ERROR_UNSUPPORTED_FRAME_FORMAT     = 1002,

    DC_HAL_ERROR_CAN_INIT                     = 2000,
    DC_HAL_ERROR_CAN_CONFIG_FILTER            = 2001,
    DC_HAL_ERROR_CAN_CONFIG_GLOBAL_FILTER     = 2002,
    DC_HAL_ERROR_CAN_START                    = 2003,
    DC_HAL_ERROR_CAN_ADD_TX_MESSAGE           = 2004,
    DC_HAL_ERROR_CAN_GET_RX_MESSAGE           = 2005,

    DC_HAL_ERROR_UNSUPPORTED_CLOCK_FREQUENCY  = 3000,
    DC_HAL_ERROR_TIMING                       = 3001,
} DC_HAL_ERROR_ENUM;


typedef enum
{
    DC_HAL_IFACE_MODE_NORMAL = 0,                   // Normal mode
    DC_HAL_IFACE_MODE_SILENT,                       // Do not affect the bus, only listen
    DC_HAL_IFACE_MODE_AUTOMATIC_TX_ABORT_ON_ERROR   // Abort pending TX if a bus error has occurred
} DC_HAL_IFACE_MODE_ENUM;


typedef struct
{
    uint64_t rx_overflow_count;
    uint64_t error_count;
} tDcHalStatistics;


typedef struct
{
    uint32_t id;
    uint32_t mask;
} tDcHalAcceptanceFilterConfiguration;


typedef struct
{
    uint16_t bit_rate_prescaler;
    uint8_t bit_segment_1;
    uint8_t bit_segment_2;
    uint8_t sync_jump_width;
} tDcHalCanTimings;


int16_t dc_hal_init(
    const tDcHalCanTimings* const timings,
    const DC_HAL_IFACE_MODE_ENUM iface_mode);


int16_t dc_hal_start(void);


int16_t dc_hal_transmit(const CanardCANFrame* const frame);


int16_t dc_hal_receive(CanardCANFrame* const frame);


int16_t dc_hal_config_acceptance_filters(
    const tDcHalAcceptanceFilterConfiguration* const filter_configs,
    const uint8_t num_filter_configs);


tDcHalStatistics dc_hal_get_stats(void);


int16_t dc_hal_compute_timings(
    const uint32_t peripheral_clock_rate,
    const uint32_t target_bitrate,
    tDcHalCanTimings* const timings);


#ifdef __cplusplus
}
#endif
#endif // STM32_DRONECAN_DRIVER_H
