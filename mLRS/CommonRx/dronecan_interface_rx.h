//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// DroneCAN Interface RX Side
//*******************************************************
#ifndef DRONECAN_INTERFACE_RX_H
#define DRONECAN_INTERFACE_RX_H
#pragma once


#include "dronecan_interface_rx_types.h"

#ifdef DEVICE_HAS_DRONECAN

#if FDCAN_IRQ_PRIORITY != DRONECAN_IRQ_PRIORITY
#error FDCAN_IRQ_PRIORITY not eq DRONECAN_IRQ_PRIORITY !
#endif

extern tRxDroneCan dronecan;

extern uint16_t micros16(void);
extern volatile uint32_t millis32(void);
extern bool connected(void);
extern tStats stats;
extern tGlobalConfig Config;

#ifndef DRONECAN_PREFERRED_NODE_ID
#define DRONECAN_PREFERRED_NODE_ID  68
#endif

#ifndef CANARD_POOL_SIZE
#define CANARD_POOL_SIZE  4096
#endif

#define DRONECAN_BUF_SIZE  512 // needs to be larger than max DroneCAN frame

CanardInstance canard;
uint8_t canard_memory_pool[CANARD_POOL_SIZE]; // doing this static leads to crash in full mLRS code !?


uint64_t micros64(void)
{
static uint64_t base_us = 0;
static uint16_t last_cnt;

    uint16_t cnt = micros16();
    if (cnt < last_cnt) {
        base_us += 0x10000;
    }
    last_cnt = cnt;
    return base_us + cnt;
}


void stm32_uid(uint8_t uid[12])
{
    // this is shorter than using LL_GetUID_Word0(), LL_GetUID_Word1(), LL_GetUID_Word2()
    uint8_t* uid_ptr = (uint8_t*)UID_BASE;
    memcpy(uid, uid_ptr, 12);
}


uint32_t stm32_cpu_id(void)
{
    // easier and more complete that using LL_CPUID_Getxxxx() functions
    return SCB->CPUID;
}


void dronecan_uid(uint8_t uid[DC_UNIQUE_ID_LEN])
{
    stm32_uid(uid); // fill first 12 bytes with UID
    uint32_t cpu_id = stm32_cpu_id();
    memcpy(&uid[12], &cpu_id, 4);
}


// that's the same as used in fhss lib, is Microsoft Visual/Quick C/C++'s
uint16_t dronecan_prng(void)
{
static uint32_t _seed = 1234;
const uint32_t a = 214013;
const uint32_t c = 2531011;
const uint32_t m = 2147483648;

    _seed = (a * _seed + c) % m;

    return _seed >> 16;
}


// receive one frame, only called from isr context
void dronecan_receive_frames(void)
{
CanardCANFrame frame;

    while (1) {
        int16_t res = dc_hal_receive(&frame); // 0: no receive, 1: receive, <0: error
        if (res < 0) {
            dbg.puts("\nERR: rec ");dbg.puts(s16toBCD_s(res));
        }
        if (res <= 0) break; // no receive or error
//dbg.puts("\nrx ");dbg.puts(u32toHEX_s(frame.id & CANARD_CAN_EXT_ID_MASK));
//dbg.puts("\nrx");
        res = canardHandleRxFrame(&canard, &frame, micros64()); // 0: ok, <0: error
        return; // only do one
    }
}


// transmits all frames from the TX queue, for calling from non-isr context
void dronecan_process_tx_queue(void)
{
const CanardCANFrame* frame;

    while (1) {
        frame = canardPeekTxQueue(&canard);
        if (!frame) break; // no frame in tx queue
        int16_t res = dc_hal_transmit(frame, millis32());
//dbg.puts("\ntx ");dbg.puts(s16toBCD_s(res));
        if (res != 0) { // successfully submitted or error, so drop the frame
            canardPopTxQueue(&canard);
        }
        return; // only do one
    }
}


// DroneCAN/Libcanard call back, forward declaration
bool dronecan_should_accept_transfer(
    const CanardInstance* const ins,
    uint64_t* const out_data_type_signature,
    uint16_t data_type_id,
    CanardTransferType transfer_type,
    uint8_t source_node_id);


// DroneCAN/Libcanard call back, forward declaration
void dronecan_on_transfer_received(CanardInstance* const ins, CanardRxTransfer* const transfer);


// CAN peripheral init forward declaration
void can_init(void);


//-------------------------------------------------------
// RxDroneCan class implementation
//-------------------------------------------------------

void tRxDroneCan::Init(bool ser_over_can_enable_flag)
{
    tick_1Hz = 0;
    node_status_transfer_id = 0;
    rc_input_transfer_id = 0;
    rc_input_tlast_ms = 0;
    node_id_allocation_transfer_id = 0;
    node_id_allocation = {};
    node_id_allocation_running = false;
    tunnel_targetted_transfer_id = 0;
    tunnel_targetted.to_fc_tlast_ms = 0;
    tunnel_targetted.server_node_id = 0;
    fifo_fc_to_ser.Flush();
    fifo_ser_to_fc.Flush();
    tunnel_targetted_fc_to_ser_rate = 0;
    tunnel_targetted_ser_to_fc_rate = 0;
    fifo_fc_to_ser_tx_full_error_cnt = 0;

    ser_over_can_enabled = ser_over_can_enable_flag;

    dbg.puts("\n\n\nCAN init");

    can_init();

    canardInit(
        &canard,                          // uninitialized library instance
        canard_memory_pool,               // raw memory chunk used for dynamic allocation
        sizeof(canard_memory_pool),       // size of the above, in bytes
        dronecan_on_transfer_received,    // callback, see CanardOnTransferReception
        dronecan_should_accept_transfer,  // callback, see CanardShouldAcceptTransfer
        nullptr);                         // user_reference, unused

    if (!ser_over_can_enabled) {
        // canardSetLocalNodeID(&canard, DRONECAN_PREFERRED_NODE_ID);
        node_id_allocation_running = true;
    } else {
        // ArduPilot's MAVLink via CAN seems to need a fixed node id
        canardSetLocalNodeID(&canard, DRONECAN_PREFERRED_NODE_ID);
        node_id_allocation_running = false;
    }

    int16_t res = set_can_filters();
    if (res < 0) {
        dbg.puts("\nERROR: filter config failed");
    }

    // it appears to not matter if first isr enable and then start, or vice versa
#ifdef DRONECAN_USE_RX_ISR
    res = dc_hal_enable_isr();
    if (res < 0) {
        dbg.puts("\nERROR: can isr config failed");
    }
#endif

    res = dc_hal_start();
    if (res < 0) {
        dbg.puts("\nERROR: can start failed");
    }

    dbg.puts("\nCAN inited");
}


void tRxDroneCan::Start(void)
{
#ifdef DRONECAN_USE_RX_ISR
    // Hum?? it somehow does not work to call dc_hal_enable_isr() here ??
    dc_hal_rx_flush();
#endif
    dbg.puts("\nCAN started");
}


bool tRxDroneCan::id_is_allcoated(void)
{
    return (canardGetLocalNodeID(&canard) != CANARD_BROADCAST_NODE_ID);
}


int16_t tRxDroneCan::set_can_filters(void)
{
tDcHalAcceptanceFilterConfiguration filter_configs[2];
uint8_t filter_num = 0;

    if (!id_is_allcoated()) {
        // initialize filters as needed for node id allocation at startup, only accept
        // - DYNAMIC_NODE_ID_ALLOCATION broadcasts
        filter_configs[0].rx_fifo = DC_HAL_RX_FIFO0;
        filter_configs[0].id =
            DC_MESSAGE_TYPE_TO_CAN_ID(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID) |
            DC_SERVICE_NOT_MESSAGE_TO_CAN_ID(0x00);
        filter_configs[0].mask =
            DC_MESSAGE_TYPE_MASK | DC_SERVICE_NOT_MESSAGE_MASK;
        filter_num = 1;

    } else {
        // set reduced filters, only accept
        // - GETNODEINFO requests
        // - TUNNEL_TARGETTED broadcasts
        filter_configs[0].rx_fifo = DC_HAL_RX_FIFO0;
        filter_configs[0].id =
            DC_SERVICE_TYPE_TO_CAN_ID(UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_ID) |
            DC_REQUEST_NOT_RESPONSE_TO_CAN_ID(0x01) |
            DC_DESTINATION_ID_TO_CAN_ID(canardGetLocalNodeID(&canard)) |
            DC_SERVICE_NOT_MESSAGE_TO_CAN_ID(0x01);
        filter_configs[0].mask =
            DC_SERVICE_TYPE_MASK | DC_REQUEST_NOT_RESPONSE_MASK | DC_DESTINATION_ID_MASK | DC_SERVICE_NOT_MESSAGE_MASK;
        filter_num = 1;
        if (ser_over_can_enabled) {
            filter_configs[0].rx_fifo = DC_HAL_RX_FIFO1;
            filter_configs[1].id =
                DC_MESSAGE_TYPE_TO_CAN_ID(UAVCAN_TUNNEL_TARGETTED_ID) |
                DC_SERVICE_NOT_MESSAGE_TO_CAN_ID(0x00);
            filter_configs[1].mask =
                DC_MESSAGE_TYPE_MASK | DC_SERVICE_NOT_MESSAGE_MASK;
            filter_num = 2;
        }
    }

    dbg.puts("\nFilter");
    for (uint8_t n = 0; n < filter_num; n++) {
        dbg.puts("\n  id:   ");dbg.puts(u32toHEX_s(filter_configs[n].id));
        dbg.puts("\n  mask: ");dbg.puts(u32toHEX_s(filter_configs[n].mask));
    }

    return dc_hal_config_acceptance_filters(filter_configs, filter_num);
}


// This function is called at 1 ms rate from the main loop
void tRxDroneCan::Tick_ms(void)
{
//    dronecan_receive_frames();
//    dronecan_process_tx_queue();

    uint64_t tnow_us = micros64(); // call it every ms to ensure it is updated

    // do dynamic node allocation if still needed
    if (!dronecan.id_is_allcoated()) {
        node_id_allocation_running = true;
        if (millis32() > node_id_allocation.send_next_request_at_ms) {
            send_dynamic_node_id_allocation_request();
        }
        return;
    }
    if (node_id_allocation_running) {
        node_id_allocation_running = false;
        set_can_filters();
        return;
    }

    uint32_t tnow_ms = millis32();
    if (tunnel_targetted.server_node_id) { // don't send before we haven't gotten a tunnel.Targetted from the fc
        if (fifo_ser_to_fc.Available() > 0 || (tnow_ms - tunnel_targetted.to_fc_tlast_ms) > 500) {
            tunnel_targetted.to_fc_tlast_ms = tnow_ms;
            send_tunnel_targetted();
        }
    } else {
        tunnel_targetted.to_fc_tlast_ms = tnow_ms;
        // this is important
        // otherwise the fifo is pretty full and many CAN messages would be send, and the rx crashes
        // behaving badly, ok, but why does it crash ??
        fifo_ser_to_fc.Flush();
    }

    DECc(tick_1Hz, SYSTICK_DELAY_MS(1000));
    if (!tick_1Hz) {
        // purge transfers that are no longer transmitted. This can free up some memory
        canardCleanupStaleTransfers(&canard, tnow_us);

        // emit node status message
        send_node_status();

dbg.puts("\n fc->ser:   ");dbg.puts(u16toBCD_s(tunnel_targetted_fc_to_ser_rate));
dbg.puts("\n ser->fc:   ");dbg.puts(u16toBCD_s(tunnel_targetted_ser_to_fc_rate));
tunnel_targetted_fc_to_ser_rate = 0;
tunnel_targetted_ser_to_fc_rate = 0;
dbg.puts("\n   tx_fifo err: ");dbg.puts(u16toBCD_s(fifo_fc_to_ser_tx_full_error_cnt));
dbg.puts("\n       err sum: ");dbg.puts(u16toBCD_s(dc_hal_get_stats().error_sum_count));
    }
}


void tRxDroneCan::Do(void)
{
    dronecan_receive_frames();
    dronecan_process_tx_queue();
}


void tRxDroneCan::SendRcData(tRcData* const rc_out, bool failsafe)
{
    if (!dronecan.id_is_allcoated()) return;

    uint32_t tnow_ms = millis32();
    if ((tnow_ms - rc_input_tlast_ms) < 19) return; // don't send too fast, DroneCAN is not for racing ...
    rc_input_tlast_ms = tnow_ms;

    uint8_t failsafe_mode = Setup.Rx.FailsafeMode;
    if (failsafe) {
        switch (failsafe_mode) {
        case FAILSAFE_MODE_NO_SIGNAL:
            // do not output anything, so jump out
            return;
        case FAILSAFE_MODE_CH1CH4_CENTER:
            // in this mode do not report bad signal
            failsafe = false;
            break;
        }
    }

    _p.rc_input.id = 0;
    _p.rc_input.status = 0;
    if (failsafe) _p.rc_input.status |= DRONECAN_SENSORS_RC_RCINPUT_STATUS_FAILSAFE;

    // this message's quality is used by ArduPilot for setting rssi (not LQ)
    // it goes from 0 ... 255
    // so we use the same conversion as in e.g. RADIO_STATUS, so that ArduPilot shows us (nearly) the same value
    _p.rc_input.quality = (connected()) ? rssi_i8_to_ap(stats.GetLastRssi()) : 0; // stats.GetLQ_rc()
    if (connected()) {
        _p.rc_input.status |= DRONECAN_SENSORS_RC_RCINPUT_STATUS_QUALITY_VALID;
    }

    _p.rc_input.rcin.len = 16;
    for (uint8_t i = 0; i < 16; i++) {
        // to get the same as mavlink rc we have
        // pwm = [ (rc-1024)*15/4 ] * 5/32 + 1500 = (rc - 1024) * 75 / 128 + 1500
        // in order to get the full range we x8 sso we can add +1 to the multiplier
        _p.rc_input.rcin.data[i] = (((int32_t)(rc_out->ch[i]) - 1024) * 601) / 1024 + 1500;
    }

    uint32_t len = dronecan_sensors_rc_RCInput_encode(&_p.rc_input, _buf);

    canardBroadcast(
        &canard,
        DRONECAN_SENSORS_RC_RCINPUT_SIGNATURE,
        DRONECAN_SENSORS_RC_RCINPUT_ID,
        &rc_input_transfer_id,
        CANARD_TRANSFER_PRIORITY_HIGH,
        _buf,
        len);
}


//-------------------------------------------------------
// serial interface
//-------------------------------------------------------

void tRxDroneCan::putbuf(uint8_t* const buf, uint16_t len)
{
    fifo_ser_to_fc.PutBuf(buf, len);
    tunnel_targetted_ser_to_fc_rate += len;
}


bool tRxDroneCan::available(void)
{
    return (fifo_fc_to_ser.Available() > 0);
    return false;
}


uint8_t tRxDroneCan::getc(void)
{
    return fifo_fc_to_ser.Get();
    return 0;
}


void tRxDroneCan::flush(void)
{
    fifo_fc_to_ser.Flush();
    fifo_ser_to_fc.Flush();
}


uint16_t tRxDroneCan::bytes_available(void)
{
    return fifo_fc_to_ser.Available();
}


//-------------------------------------------------------
// DroneCAN default services
//-------------------------------------------------------

// Send NodeStatus message
// This allows node to show up in the DroneCAN GUI tool and in the flight controller logs
void tRxDroneCan::send_node_status(void)
{
    _p.node_status.uptime_sec = millis32() / 1000;
    _p.node_status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    _p.node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    _p.node_status.sub_mode = 0;

    // put something in vendor specific status
    static uint16_t cnt = 0;
    _p.node_status.vendor_specific_status_code = cnt;
    cnt++;

    uint32_t len = uavcan_protocol_NodeStatus_encode(&_p.node_status, _buf);

    canardBroadcast(
        &canard,
        UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
        UAVCAN_PROTOCOL_NODESTATUS_ID,
        &node_status_transfer_id,
        CANARD_TRANSFER_PRIORITY_LOW,
        _buf,
        len);
}


// Handle a GetNodeInfo request, and send a response
void tRxDroneCan::handle_get_node_info_request(CanardInstance* const ins, CanardRxTransfer* const transfer)
{
    _p.node_info_resp.status.uptime_sec = millis32() / 1000;
    _p.node_info_resp.status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    _p.node_info_resp.status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    _p.node_info_resp.status.sub_mode = 0;
    _p.node_info_resp.status.vendor_specific_status_code = 0;

    uint32_t version = VERSION;
    uint32_t major = version / 10000;
    version -= major * 10000;
    uint32_t minor = version / 100;
    version -= minor * 100;
    uint32_t patch = version;

    _p.node_info_resp.software_version.major = major;
    _p.node_info_resp.software_version.minor = minor;
    _p.node_info_resp.software_version.optional_field_flags = patch;
    _p.node_info_resp.software_version.vcs_commit = 0; // should put git hash in here

    _p.node_info_resp.hardware_version.major = 0;
    _p.node_info_resp.hardware_version.minor = 0;

    dronecan_uid(_p.node_info_resp.hardware_version.unique_id);

    // can be 80 chars, which is always larger than our device name, so no need to worry about too long string
    strcpy((char*)_p.node_info_resp.name.data, "mlrs.");
    strcat((char*)_p.node_info_resp.name.data, DEVICE_NAME);
    for (uint8_t n = 0; n < strlen((char*)_p.node_info_resp.name.data); n ++) {
        if (_p.node_info_resp.name.data[n] == ' ') _p.node_info_resp.name.data[n] = '_';
        if (_p.node_info_resp.name.data[n] >= 'A' && _p.node_info_resp.name.data[n] <= 'Z') {
          _p.node_info_resp.name.data[n] = _p.node_info_resp.name.data[n] - 'A' + 'a';
        }
    }
    _p.node_info_resp.name.len = strlen((char*)_p.node_info_resp.name.data);

    uint16_t len = uavcan_protocol_GetNodeInfoResponse_encode(&_p.node_info_resp, _buf);

    canardRequestOrRespond(
        ins,
        transfer->source_node_id,
        UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
        UAVCAN_PROTOCOL_GETNODEINFO_ID,
        &transfer->transfer_id,
        transfer->priority,
        CanardResponse,
        _buf,
        len);
}


// The following two functions for dynamic node id allocation VERY closely follow an example source which
// was provided by the UAVACN and libcanard projects in around 2017. The original sources seem to not be
// available anymore. The licence was almost surely  permissive (MIT?) and the author Pavel Kirienko. We
// apologize for not giving more appropriate credit.

// Handle a dynamic node allocation message
void tRxDroneCan::handle_dynamic_node_id_allocation_broadcast(CanardInstance* const ins, CanardRxTransfer* const transfer)
{
    // Dynamic node ID allocation protocol.
    // Taking this branch only if we don't have a node ID, ignoring otherwise.

    // Rule C - updating the randomized time interval
    node_id_allocation.send_next_request_at_ms =
        millis32() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        (dronecan_prng() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID) {
        node_id_allocation.unique_id_offset = 0;
        return;
    }

    // Copying the unique ID from the message
    struct uavcan_protocol_dynamic_node_id_Allocation payload;
    if (uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &payload)) {
        return; // something went wrong
    }

    // Obtaining the local unique ID
    uint8_t my_uid[DC_UNIQUE_ID_LEN];
    dronecan_uid(my_uid);

    // Matching the received UID against the local one
    if (memcmp(payload.unique_id.data, my_uid, payload.unique_id.len) != 0) {
        node_id_allocation.unique_id_offset = 0;
        return; // No match, return
    }

    if (payload.unique_id.len < DC_UNIQUE_ID_LEN) {
        // The allocator has confirmed part of unique ID, switching to the next stage and updating the timeout.
        node_id_allocation.unique_id_offset = payload.unique_id.len;
        node_id_allocation.send_next_request_at_ms -= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;
    } else {
        // Allocation complete - copying the allocated node ID from the message
        canardSetLocalNodeID(ins, payload.node_id);
    }
}


// Request a dynamic node allocation
void tRxDroneCan::send_dynamic_node_id_allocation_request(void)
{
    node_id_allocation.send_next_request_at_ms =
        millis32() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        (dronecan_prng() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    // Structure of the request is documented in the DSDL definition
    // See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
    uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1];
    allocation_request[0] = DRONECAN_PREFERRED_NODE_ID << 1;

    if (node_id_allocation.unique_id_offset == 0) {
        allocation_request[0] |= 1; // First part of unique ID
    }

    uint8_t my_uid[DC_UNIQUE_ID_LEN];
    dronecan_uid(my_uid);

    uint8_t uid_size = (uint8_t)(DC_UNIQUE_ID_LEN - node_id_allocation.unique_id_offset);
    if (uid_size > UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST) {
        uid_size = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST;
    }

    memmove(&allocation_request[1], &my_uid[node_id_allocation.unique_id_offset], uid_size);

    // Broadcasting the request
    canardBroadcast(&canard,
        UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
        UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
        &node_id_allocation_transfer_id,
        CANARD_TRANSFER_PRIORITY_LOW,
        allocation_request,
        uid_size + 1);

    // Preparing for timeout; if response is received, this value will be updated from the callback.
    node_id_allocation.unique_id_offset = 0;
}


// Handle a tunnel.Targetted message, check if it's for us and proper
void tRxDroneCan::handle_tunnel_targetted_broadcast(CanardInstance* const ins, CanardRxTransfer* const transfer)
{
    if (uavcan_tunnel_Targetted_decode(transfer, &_p.tunnel_targetted)) {
        return; // something went wrong
    }

    // must be targeted at us
    if (_p.tunnel_targetted.target_node != canardGetLocalNodeID(&canard)) return;

    // ArduPilot unfortunately does not set this correctly, so can't check
    //if (_p.tunnel_targetted.protocol.protocol != UAVCAN_TUNNEL_PROTOCOL_MAVLINK2) return;

    // serial_id must be set to 0
    // TODO: should we just store it and reuse when sending?
    if (_p.tunnel_targetted.serial_id != 0) return;

    tunnel_targetted_fc_to_ser_rate += _p.tunnel_targetted.buffer.len;

    // memorize the node_id of the sender, this is most likely our fc (hopefully true)
    if (!tunnel_targetted.server_node_id) {
        tunnel_targetted.server_node_id = transfer->source_node_id;
    }

    if (_p.tunnel_targetted.buffer.len == 0) return; // a short cut

    if (fifo_fc_to_ser.IsFull() || !fifo_fc_to_ser.HasSpace(_p.tunnel_targetted.buffer.len)) {
        fifo_fc_to_ser_tx_full_error_cnt++;
    }

    fifo_fc_to_ser.PutBuf(_p.tunnel_targetted.buffer.data, _p.tunnel_targetted.buffer.len);
}


// Send a tunnel.Targetted message to the node which hopefully is the flight controller
void tRxDroneCan::send_tunnel_targetted(void)
{
    _p.tunnel_targetted.target_node = tunnel_targetted.server_node_id;
    _p.tunnel_targetted.protocol.protocol = UAVCAN_TUNNEL_PROTOCOL_MAVLINK2;
    _p.tunnel_targetted.serial_id = 0;
    _p.tunnel_targetted.options = UAVCAN_TUNNEL_TARGETTED_OPTION_LOCK_PORT;
    _p.tunnel_targetted.baudrate = Config.SerialBaudrate; // this is ignored by ArduPilot (as it should)

    uint16_t data_len = fifo_ser_to_fc.Available();
    _p.tunnel_targetted.buffer.len = (data_len < 120) ? data_len : 120;
    for (uint8_t n = 0; n < data_len; n++) _p.tunnel_targetted.buffer.data[n] = fifo_ser_to_fc.Get();

    uint16_t len = uavcan_tunnel_Targetted_encode(&_p.tunnel_targetted, _buf);

    canardBroadcast(
        &canard,
        UAVCAN_TUNNEL_TARGETTED_SIGNATURE,
        UAVCAN_TUNNEL_TARGETTED_ID,
        &tunnel_targetted_transfer_id,
        CANARD_TRANSFER_PRIORITY_MEDIUM,
        _buf,
        len);
}


//-------------------------------------------------------
// DroneCAN/Libcanard call backs
//-------------------------------------------------------

// this callback is invoked when it detects beginning of a new transfer on the bus that can be received
// by the local node.
// Return value
//  true: library will receive the transfer
//  false: library will ignore the transfer.
// This function must fill in the out_data_type_signature to be the signature of the message.
bool dronecan_should_accept_transfer(
    const CanardInstance* const ins,
    uint64_t* const out_data_type_signature,
    uint16_t data_type_id,
    CanardTransferType transfer_type,
    uint8_t source_node_id)
{
    // handle service requests
    if (transfer_type == CanardTransferTypeRequest) {
        switch (data_type_id) {
            case UAVCAN_PROTOCOL_GETNODEINFO_ID:
                if (!dronecan.id_is_allcoated()) return false;
                *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
                return true;
        }
    }
    // handle broadcast
    if (transfer_type == CanardTransferTypeBroadcast) {
        switch (data_type_id) {
            case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID:
                if (dronecan.id_is_allcoated()) return false; // we are already done with node id allocation
                *out_data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
                return true;
            case UAVCAN_TUNNEL_TARGETTED_ID:
                if (!dronecan.id_is_allcoated()) return false;
                if (!dronecan.ser_over_can_enabled) return false;
                *out_data_type_signature = UAVCAN_TUNNEL_TARGETTED_SIGNATURE;
                return true;
        }
    }
    return false;
}


// this callback is invoked when a new message or request or response is received
void dronecan_on_transfer_received(CanardInstance* const ins, CanardRxTransfer* const transfer)
{
    // handle service requests
    if (transfer->transfer_type == CanardTransferTypeRequest) {
        switch (transfer->data_type_id) {
            case UAVCAN_PROTOCOL_GETNODEINFO_ID:
                dronecan.handle_get_node_info_request(ins, transfer);
                return;
        }
    }
    // handle broadcasts
    if (transfer->transfer_type == CanardTransferTypeBroadcast) {
        switch (transfer->data_type_id) {
            case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID:
                dronecan.handle_dynamic_node_id_allocation_broadcast(ins, transfer);
                return;
            case UAVCAN_TUNNEL_TARGETTED_ID:
                dronecan.handle_tunnel_targetted_broadcast(ins, transfer);
                return;
        }
    }
}


//-------------------------------------------------------
// CAN init
//-------------------------------------------------------

#ifdef STM32F1
#ifndef HAL_CAN_MODULE_ENABLED
  #error HAL_CAN_MODULE_ENABLED not defined, enable it in Core\Inc\stm32f1xx_hal_conf.h!
#endif

void can_init(void)
{
    // CAN peripheral initialization

    gpio_init(IO_PA11, IO_MODE_INPUT_PU, IO_SPEED_VERYFAST);
    gpio_init_af(IO_PA12, IO_MODE_OUTPUT_ALTERNATE_PP, IO_AF_9, IO_SPEED_VERYFAST);

    //__HAL_RCC_CAN1_CLK_ENABLE();
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_CAN1);

    // DroneCAN Hal initialization

    tDcHalCanTimings timings;
    int16_t res = dc_hal_compute_timings(HAL_RCC_GetPCLK1Freq(), 1000000, &timings); // = 36000000, CAN is on slow APB1 bus
    if (res < 0) {
        dbg.puts("\nERROR: Solution for CAN timings could not be found");
        return;
    }
    dbg.puts("\n  PCLK1: ");dbg.puts(u32toBCD_s(HAL_RCC_GetPCLK1Freq()));
    dbg.puts("\n  Prescaler: ");dbg.puts(u16toBCD_s(timings.bit_rate_prescaler));
    dbg.puts("\n  BS1: ");dbg.puts(u8toBCD_s(timings.bit_segment_1));
    dbg.puts("\n  BS2: ");dbg.puts(u8toBCD_s(timings.bit_segment_2));
    dbg.puts("\n  SJW: ");dbg.puts(u8toBCD_s(timings.sync_jump_width));
    // 4, 7, 1, 1

    //res = canardSTM32Init(&timings, CanardSTM32IfaceModeNormal);
    res = dc_hal_init(&timings, DC_HAL_IFACE_MODE_AUTOMATIC_TX_ABORT_ON_ERROR);
    if (res < 0) {
        dbg.puts("\nERROR: Failed to open CAN iface ");dbg.puts(s16toBCD_s(res));
        return;
    }
}

#endif // STM32F1
#ifdef STM32G4
#ifndef HAL_FDCAN_MODULE_ENABLED
  #error HAL_FDCAN_MODULE_ENABLED not defined, enable it in Core\Inc\stm32g4xx_hal_conf.h!
#endif

//#define USE_HAL_NOT_LL
//#include "stm32g4xx_hal.h"

void can_init(void)
{
    // GPIO initialization
    // PA11 = FDCAN1_RX
    // PA12 = FDCAN1_TX

#ifdef USE_HAL_NOT_LL
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {};
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#else
    gpio_init_af(IO_PA11, IO_MODE_OUTPUT_ALTERNATE_PP, IO_AF_9, IO_SPEED_VERYFAST);
    gpio_init_af(IO_PA12, IO_MODE_OUTPUT_ALTERNATE_PP, IO_AF_9, IO_SPEED_VERYFAST);
/*    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

    LL_GPIO_InitTypeDef GPIO_InitStruct = {};
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_9;

    GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct); */
#endif

    // FDCAN clock initialization
/*
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {}
*/
#ifdef USE_HAL_NOT_LL
    __HAL_RCC_FDCAN_CONFIG(RCC_FDCANCLKSOURCE_PCLK1); // RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_FDCANSEL) | RCC_CCIPR_FDCANSEL_1;

    __HAL_RCC_FDCAN_CLK_ENABLE(); // RCC->APB1ENR1  |= RCC_APB1ENR1_FDCANEN;
    //__HAL_RCC_FDCAN_FORCE_RESET(); // SET_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_FDCANRST) // RCC->APB1RSTR1 |= RCC_APB1RSTR1_FDCANRST;
    //__HAL_RCC_FDCAN_RELEASE_RESET(); // CLEAR_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_FDCANRST) // RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_FDCANRST;
#else
    LL_RCC_SetFDCANClockSource(LL_RCC_FDCAN_CLKSOURCE_PCLK1);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_FDCAN);
    //LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_FDCAN);
    //LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_FDCAN);
#endif

    // DroneCAN HAL initialization

    tDcHalCanTimings timings;
    //int16_t res = dc_hal_compute_timings(HAL_RCC_GetPCLK1Freq(), 1000000, &timings);
    int16_t res = dc_hal_compute_timings(HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN), 1000000, &timings);
    if (res < 0) {
        dbg.puts("\nERROR: Solution for CAN timings could not be found");
        return;
    }
    dbg.puts("\n  PCLK1: ");dbg.puts(u32toBCD_s(HAL_RCC_GetPCLK1Freq()));
    dbg.puts("\n  FDCAN CLK: ");dbg.puts(u32toBCD_s(HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN)));
    dbg.puts("\n  Prescaler: ");dbg.puts(u16toBCD_s(timings.bit_rate_prescaler));
    dbg.puts("\n  BS1: ");dbg.puts(u8toBCD_s(timings.bit_segment_1));
    dbg.puts("\n  BS2: ");dbg.puts(u8toBCD_s(timings.bit_segment_2));
    dbg.puts("\n  SJW: ");dbg.puts(u8toBCD_s(timings.sync_jump_width));

    res = dc_hal_init(&timings, DC_HAL_IFACE_MODE_AUTOMATIC_TX_ABORT_ON_ERROR);
    if (res < 0) {
        dbg.puts("\nERROR: Failed to open CAN iface ");dbg.puts(s16toBCD_s(res));
        return;
    }
}

#endif // STM32G4


//-------------------------------------------------------
//-- check some sizes
//-------------------------------------------------------

STATIC_ASSERT(DRONECAN_BUF_SIZE >= UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE, "DRONECAN_BUF_SIZE too small")
STATIC_ASSERT(DRONECAN_BUF_SIZE >= UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE, "DRONECAN_BUF_SIZE too small")
STATIC_ASSERT(DRONECAN_BUF_SIZE >= DRONECAN_SENSORS_RC_RCINPUT_MAX_SIZE, "DRONECAN_BUF_SIZE too small")
STATIC_ASSERT(DRONECAN_BUF_SIZE >= UAVCAN_TUNNEL_TARGETTED_MAX_SIZE, "DRONECAN_BUF_SIZE too small")


#endif // DEVICE_HAS_DRONECAN

#endif // DRONECAN_INTERFACE_RX_H
