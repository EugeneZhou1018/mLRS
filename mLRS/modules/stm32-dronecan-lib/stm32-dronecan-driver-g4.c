//*******************************************************
// STM32 DroneCAN Library
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// DroneCAN Driver Library for STM32 using HAL
// for use with libcanard
//*******************************************************
#if defined STM32G491xx || defined STM32G441xx  || defined STM32G431xx

#include "stm32g4xx_hal.h"

#ifdef HAL_FDCAN_MODULE_ENABLED
#include "stm32-dronecan-driver.h"
#include <string.h>


#define DC_HAL_ACCEPTANCE_FILTERS_NUM_MAX  8 // must be equal to SRAMCAN_FLE_NBR, for FDCAN_EXTENDED_ID

// we can use IS_FDCAN_NOMINAL_PRESCALER(), IS_FDCAN_NOMINAL_TSEG1(), IS_FDCAN_NOMINAL_TSEG2(), IS_FDCAN_NOMINAL_SJW() !
#define DC_HAL_PRESCALER_MIN        1
#define DC_HAL_PRESCALER_MAX        512

#define DC_HAL_NOMINAL_BS1_MIN      1     // datasheet: bit_time must be 4 .. 81 tq
#define DC_HAL_NOMINAL_BS1_MAX      256
#define DC_HAL_NOMINAL_BS2_MIN      1
#define DC_HAL_NOMINAL_BS2_MAX      128
#define DC_HAL_NOMINAL_SJW_MIN      1
#define DC_HAL_NOMINAL_SJW_MAX      128

#define DC_HAL_DATA_PRESCALER_MIN   1
#define DC_HAL_DATA_PRESCALER_MAX   32
#define DC_HAL_DATA_BS1_MIN         1
#define DC_HAL_DATA_BS1_MAX         32
#define DC_HAL_DATA_BS2_MIN         1
#define DC_HAL_DATA_BS2_MAX         16
#define DC_HAL_DATA_SJW_MIN         1
#define DC_HAL_DATA_SJW_MAX         16    // datasheet: must always be smaller than BS2


static tDcHalStatistics dc_hal_stats;

static bool dc_hal_abort_tx_on_error;

static FDCAN_HandleTypeDef hfdcan;


//-------------------------------------------------------
// Helper
//-------------------------------------------------------

static void _process_error_status(void)
{
    // TODO
}


//-------------------------------------------------------
// Init
//-------------------------------------------------------

int16_t dc_hal_init(
    const tDcHalCanTimings* const timings,
    const DC_HAL_IFACE_MODE_ENUM iface_mode)
{
    if ((iface_mode != DC_HAL_IFACE_MODE_NORMAL) &&
        (iface_mode != DC_HAL_IFACE_MODE_SILENT) &&
        (iface_mode != DC_HAL_IFACE_MODE_AUTOMATIC_TX_ABORT_ON_ERROR)) {
        return -DC_HAL_ERROR_INVALID_ARGUMENT;
    }

    if ((timings == NULL) ||
        !IS_FDCAN_NOMINAL_PRESCALER(timings->bit_rate_prescaler) ||
        !IS_FDCAN_NOMINAL_TSEG1(timings->bit_segment_1) ||
        !IS_FDCAN_NOMINAL_TSEG2(timings->bit_segment_2) ||
        !IS_FDCAN_NOMINAL_SJW(timings->sync_jump_width)) {
        return -DC_HAL_ERROR_INVALID_ARGUMENT;
    }

    memset(&dc_hal_stats, 0, sizeof(dc_hal_stats));
    dc_hal_abort_tx_on_error = (iface_mode == DC_HAL_IFACE_MODE_AUTOMATIC_TX_ABORT_ON_ERROR);

    hfdcan.Instance = FDCAN1;
    __HAL_FDCAN_DISABLE_IT(&hfdcan, 0);

    hfdcan.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan.Init.FrameFormat = FDCAN_FRAME_CLASSIC; // FDCAN_FRAME_FD_NO_BRS, FDCAN_FRAME_FD_BRS
    hfdcan.Init.Mode = FDCAN_MODE_NORMAL;

    hfdcan.Init.AutoRetransmission = DISABLE;
    hfdcan.Init.TransmitPause = DISABLE;
    hfdcan.Init.ProtocolException = DISABLE;

    hfdcan.Init.NominalPrescaler = timings->bit_rate_prescaler;
    hfdcan.Init.NominalTimeSeg1 = timings->bit_segment_1;
    hfdcan.Init.NominalTimeSeg2 = timings->bit_segment_2;
    hfdcan.Init.NominalSyncJumpWidth = timings->sync_jump_width;

    hfdcan.Init.DataPrescaler = 1; // irrelevant if FrameFormat != FDCAN_FRAME_FD_BRS
    hfdcan.Init.DataTimeSeg1 = 1;
    hfdcan.Init.DataTimeSeg2 = 1;
    hfdcan.Init.DataSyncJumpWidth = 1;

    hfdcan.Init.StdFiltersNbr = 0; // these are used in FDCAN_CalcultateRamBlockAddresses() called in HAL_FDCAN_Init()
    hfdcan.Init.ExtFiltersNbr = DC_HAL_ACCEPTANCE_FILTERS_NUM_MAX;

    hfdcan.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION; // FDCAN_TX_QUEUE_OPERATION

    HAL_StatusTypeDef hres = HAL_FDCAN_Init(&hfdcan);
    if (hres != HAL_OK) { return -DC_HAL_ERROR_CAN_INIT; }

    // configure reception filter
    //   follow licanard's default filter setup in spirit
    //   here it's really needed since the FDCAN RAM was already set up for max num filters
    //   so we fill it with some default
    //   HAL_FDCAN_ConfigFilter() sets the filter registers even when it is disabled
    // at least one filter must be enabled for receive to work
    FDCAN_FilterTypeDef sFilterConfig = {};
    sFilterConfig.IdType = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK; // classic filter: FilterID1 = id, FilterID2 = mask
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // FDCAN_FILTER_DISABLE, FDCAN_FILTER_TO_RXFIFO1
    sFilterConfig.FilterID1 = 0;
    sFilterConfig.FilterID2 = 0;

    for (uint8_t n = 0; n < DC_HAL_ACCEPTANCE_FILTERS_NUM_MAX; n++) {
        //sFilterConfig.FilterConfig = ((n & 0x01) == 0) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
        sFilterConfig.FilterConfig = (n == 0) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_DISABLE;
        sFilterConfig.FilterIndex = n;
        hres = HAL_FDCAN_ConfigFilter(&hfdcan, &sFilterConfig);
        if (hres != HAL_OK) { return -DC_HAL_ERROR_CAN_CONFIG_FILTER; }
    }

    // configure global filter
    //  reject non matching frames with STD and EXT ID
    //  filter all remote frames with STD and EXT ID
    hres = HAL_FDCAN_ConfigGlobalFilter(&hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    if (hres != HAL_OK) { return -DC_HAL_ERROR_CAN_CONFIG_GLOBAL_FILTER; }

    // FDCAN_RX_FIFO_BLOCKING, FDCAN_RX_FIFO_OVERWRITE ???
    //hres = HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE);
    //hres = HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan, FDCAN_RX_FIFO1, FDCAN_RX_FIFO_OVERWRITE);

    //HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan, 3, 0);
    //HAL_FDCAN_EnableTxDelayCompensation(&hfdcan);

    // CCE bit in FDCAN_CCCR register is automatically cleared when INIT bit in FDCAN_CCCR is cleared.
    //CLEAR_BIT(hfdcan.Instance->CCCR, FDCAN_CCCR_CCE);

    return 0;
}


int16_t dc_hal_start(void)
{
    HAL_StatusTypeDef hres = HAL_FDCAN_Start(&hfdcan);
    if (hres != HAL_OK) { return -DC_HAL_ERROR_CAN_START; }

    return 0;
}


//-------------------------------------------------------
// Transmit
//-------------------------------------------------------

int16_t dc_hal_transmit(const CanardCANFrame* const frame)
{
    if (frame == NULL) {
        return -DC_HAL_ERROR_INVALID_ARGUMENT;
    }

    if (frame->id & CANARD_CAN_FRAME_ERR) {
        return -DC_HAL_ERROR_UNSUPPORTED_FRAME_FORMAT;
    }
    if (frame->id & CANARD_CAN_FRAME_RTR) { // DroneCAN does not use REMOTE frames
        return -DC_HAL_ERROR_UNSUPPORTED_FRAME_FORMAT;
    }
    if (!(frame->id & CANARD_CAN_FRAME_EFF)) { // DroneCAN does not use STD ID, uses only EXT frames
        return -DC_HAL_ERROR_UNSUPPORTED_FRAME_FORMAT;
    }

    if (frame->data_len > 8) { // don't do FD
        return -DC_HAL_ERROR_UNSUPPORTED_FRAME_FORMAT;
    }

    _process_error_status();

    // thx to the TxFiFo in the G4 we can do the crude method and just put the message into the fifo if there is space
    // check for space in fifo
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan) == 0) {
        return 0; // no space, postpone
    }
    // this check is done in HAL_FDCAN_AddMessageToTxFifoQ(), so better do it here too
    if ((hfdcan.Instance->TXFQS & FDCAN_TXFQS_TFQF) != 0) {
        return 0; // postpone
    }

    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; // FDCAN_ESI_ACTIVE, FDCAN_ESI_PASSIVE ???
    pTxHeader.BitRateSwitch = FDCAN_BRS_OFF; // FDCAN_BRS_ON
    pTxHeader.FDFormat = FDCAN_CLASSIC_CAN; // FDCAN_FD_CAN
    pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // FDCAN_STORE_TX_EVENTS
    pTxHeader.MessageMarker = 0; // This parameter must be a number between 0 and 0xFF

    pTxHeader.Identifier = (frame->id & CANARD_CAN_EXT_ID_MASK);
    pTxHeader.IdType = FDCAN_EXTENDED_ID;
    pTxHeader.TxFrameType = FDCAN_DATA_FRAME;

    pTxHeader.DataLength = (uint32_t)frame->data_len << 16;

    HAL_StatusTypeDef hres = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &pTxHeader, frame->data);
    if (hres != HAL_OK) { return -DC_HAL_ERROR_CAN_ADD_TX_MESSAGE; }

    return 1;
}


//-------------------------------------------------------
// Receive
//-------------------------------------------------------

int16_t dc_hal_receive(CanardCANFrame* const frame)
{
    if (frame == NULL) {
        return -DC_HAL_ERROR_INVALID_ARGUMENT;
    }

    _process_error_status();

    uint32_t rx_fifo[2] = { FDCAN_RX_FIFO0, FDCAN_RX_FIFO1 };
    uint8_t data[64];

    for (uint8_t i = 0; i < 2; i++) {
        FDCAN_RxHeaderTypeDef pRxHeader;
        HAL_StatusTypeDef hres = HAL_FDCAN_GetRxMessage(&hfdcan, rx_fifo[i], &pRxHeader, data); //frame->data);
        if (hres != HAL_OK) {
            continue; // return -DC_HAL_ERROR_CAN_GET_RX_MESSAGE;
        }

        if (pRxHeader.BitRateSwitch != FDCAN_BRS_OFF) {
            continue; // return -DC_HAL_ERROR_CAN_GET_RX_MESSAGE; // something is wrong here
        }
        if (pRxHeader.FDFormat != FDCAN_CLASSIC_CAN) {
            continue; // return -DC_HAL_ERROR_CAN_GET_RX_MESSAGE; // something is wrong here
        }
        if (pRxHeader.IsFilterMatchingFrame != 0) {
            // TODO
        }
        // DroneCAN uses only EXT frames, so these should be errors
        if (pRxHeader.IdType != FDCAN_EXTENDED_ID) {
            continue; // return -DC_HAL_ERROR_CAN_GET_RX_MESSAGE;
        }
        if (pRxHeader.RxFrameType != FDCAN_DATA_FRAME) {
            continue; // return -DC_HAL_ERROR_CAN_GET_RX_MESSAGE;
        }

        frame->data_len = pRxHeader.DataLength >> 16;
        if (frame->data_len > 8) {
            continue; // return -DC_HAL_ERROR_CAN_GET_RX_MESSAGE; // argh something is terribly wrong
        }

        frame->id = (pRxHeader.Identifier & CANARD_CAN_EXT_ID_MASK);
        frame->id |= CANARD_CAN_FRAME_EFF;

        memset(frame->data, 0, 8);
        memcpy(frame->data, data, frame->data_len);

        frame->iface_id = 0;

        return 1;
    }

    return 0;
}


//-------------------------------------------------------
// Filter
//-------------------------------------------------------

// num_filter_configs = 0 rejects all frames
int16_t dc_hal_config_acceptance_filters(
    const tDcHalAcceptanceFilterConfiguration* const filter_configs,
    const uint8_t num_filter_configs)
{
    if ((filter_configs == NULL) || (num_filter_configs > DC_HAL_ACCEPTANCE_FILTERS_NUM_MAX)) {
        return -DC_HAL_ERROR_INVALID_ARGUMENT;
    }

    FDCAN_FilterTypeDef sFilterConfig = {};
    sFilterConfig.IdType = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK; // classic filter: FilterID1 = id, FilterID2 = mask
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // FDCAN_FILTER_DISABLE, FDCAN_FILTER_TO_RXFIFO1
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterID1 = 0;
    sFilterConfig.FilterID2 = 0;

    for (uint8_t n = 0; n < num_filter_configs; n++) {
        sFilterConfig.FilterConfig = ((n & 0x01) == 0) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
        sFilterConfig.FilterIndex = n;
        sFilterConfig.FilterID1 = (filter_configs[n].id & CANARD_CAN_EXT_ID_MASK);
        sFilterConfig.FilterID2 = (filter_configs[n].mask & CANARD_CAN_EXT_ID_MASK);
        HAL_StatusTypeDef hres = HAL_FDCAN_ConfigFilter(&hfdcan, &sFilterConfig);
        if (hres != HAL_OK) { return -DC_HAL_ERROR_CAN_CONFIG_FILTER; }
    }

    // fill remaining filters with default
    sFilterConfig.FilterID1 = 0;
    sFilterConfig.FilterID2 = 0;
    for (uint8_t n = num_filter_configs; n < DC_HAL_ACCEPTANCE_FILTERS_NUM_MAX; n++) {
        //sFilterConfig.FilterConfig = ((n & 0x01) == 0) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
        //sFilterConfig.FilterConfig = (n == 0) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_DISABLE;
        sFilterConfig.FilterConfig = FDCAN_FILTER_DISABLE;
        sFilterConfig.FilterIndex = n;
        HAL_StatusTypeDef hres = HAL_FDCAN_ConfigFilter(&hfdcan, &sFilterConfig);
        if (hres != HAL_OK) { return -DC_HAL_ERROR_CAN_CONFIG_FILTER; }
    }

    return 0;
}


//-------------------------------------------------------
// More Helper
//-------------------------------------------------------

tDcHalStatistics dc_hal_get_stats(void)
{
    return dc_hal_stats;
}


int16_t dc_hal_compute_timings(
    const uint32_t peripheral_clock_rate,
    const uint32_t target_bitrate,
    tDcHalCanTimings* const timings)
{
    if (target_bitrate != 1000000) {
        return -DC_HAL_ERROR_UNSUPPORTED_BIT_RATE;
    }

    // timings generated by phryniszak for 75%
#if 1
    if (peripheral_clock_rate == 170000000) { // 170 MHz
        timings->bit_rate_prescaler = 1;
        timings->bit_segment_1 = 127;
        timings->bit_segment_2 = 42;
        timings->sync_jump_width = 42;
    } else if (peripheral_clock_rate == 160000000) { // 160 MHz
        timings->bit_rate_prescaler = 1;
        timings->bit_segment_1 = 119;
        timings->bit_segment_2 = 40;
        timings->sync_jump_width = 40;
    } else if (peripheral_clock_rate == 80000000) { // 80 MHz
        timings->bit_rate_prescaler = 1;
        timings->bit_segment_1 = 59;
        timings->bit_segment_2 = 20;
        timings->sync_jump_width = 20;
    } else {
        return -DC_HAL_ERROR_UNSUPPORTED_CLOCK_FREQUENCY;
    }
#endif

    // timings generated by somewhat following mbed's method
    // it favors small prescaler
    // 170 MHz -> 1, 127, 42, 42
#if 0
    const uint32_t tq = peripheral_clock_rate / 1000000;
    uint32_t prescaler = 1;
    uint32_t bs1 = tq/2; // just something, to silence compiler
    while (1) {
        // see if we can use that prescaler
        // 75%
        // => 1 + BS1 = 3 * BS2 => BS2 will never reach it's upper limit
        // => 4/3 * (1 + BS1) * prescaler = tq
        // => BS1 = 3/4 * tq/prescaler - 1
        bs1 = 3 * tq / (4 * prescaler); // is 3/4 * tq/prescaler - 1, but drop the -1
        if (IS_FDCAN_NOMINAL_TSEG1(bs1)) break; // we found a good value
        prescaler++;
        if (!IS_FDCAN_NOMINAL_PRESCALER(prescaler)) {
            return -DC_HAL_ERROR_TIMING;
        }
    }

    timings->bit_rate_prescaler = prescaler;
    timings->bit_segment_1 = bs1;
    timings->bit_segment_2 = tq - 1 - bs1; // to ensure (1 + BS1 + BS2) = f_clk_MHz / 1000000
    timings->sync_jump_width = timings->bit_segment_2;
#endif

    // timings generated by kvaser
#if 0
    timings->bit_rate_prescaler = 1;
    timings->bit_segment_1 = 127;
    timings->bit_segment_2 = 42;
    timings->sync_jump_width = 43; // not allowed according to datasheet !!
#endif


    // timings generated by ACANF
#if 0
    return ACANFD_STM32_Settings(
        peripheral_clock_rate,
        timings,
        target_bitrate, 75,  1, 75,   50);
        // ppm = 500: 5, 24, 9, 9
        // ppm = 100: 5, 24, 9, 9
        // ppm =  50: 5, 24, 9, 9
        // => (1 + 24 + 9) * 5 = 170 OK!
#endif

    // let's do a check
    // datasheet:
    //   tq = prescaler * 1/f_clk
    //   bit_time = (1 + BS1 + BS2) * tq
    // =>
    //   bit_time = (1 + BS1 + BS2) * prescaler * 1/f_clk
    // we want bit_time = 1/1000000
    // =>
    //   (1 + BS1 + BS2) * prescaler = f_clk / 1000000
    const uint32_t bit_time = (1 + timings->bit_segment_1 + timings->bit_segment_2) * timings->bit_rate_prescaler;
    const uint32_t f_clk_MHz = peripheral_clock_rate / 1000000;
    if (bit_time != f_clk_MHz) {
        return -DC_HAL_ERROR_TIMING;
    }

    return 0;
}



/*
https://phryniszak.github.io/stm32g-fdcan/
https://github.com/phryniszak/stm32g-fdcan?tab=readme-ov-file
nominal:  TSeg1 = 127,  Tseg2 = 42, SJW = 42, Prescale = 1     75%
data:     TSeg1 = 7,    Tseg2 = 2,  SJW = 2, Prescale = 17     80%

https://kvaser.com/support/calculators/can-fd-bit-timing-calculator/
170 MHz, 500 ppm, 100ns
1MB/s 1MB/s
for ca 75%
nominal:  TSeg1 = 85 + 42 = 127,  Tseg2 = 42, SJW = 43, Prescale = 1
data:     TSeg1 = 126,            Tseg2 = 43, SJW = 43, Prescale = 1

STM32G4 examples
https://github.com/STMicroelectronics/STM32CubeG4/tree/master/Projects/STM32G474E-EVAL/Examples/FDCAN

https://github.com/pierremolinaro/acanfd-stm32

https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/CANFDIface.cpp

https://github.com/am32-firmware/AM32/pull/36/files

https://github.com/ARMmbed/mbed-os/pull/13565/files
*/


#endif // HAL_PCD_MODULE_ENABLED
#endif // STM32G4
