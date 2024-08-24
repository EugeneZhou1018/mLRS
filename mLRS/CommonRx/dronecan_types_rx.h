//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// DroneCAN Interface RX Side
//*******************************************************
#ifndef DRONECAN_TYPES_RX_H
#define DRONECAN_TYPES_RX_H
#pragma once

#if defined DEVICE_HAS_DRONECAN || defined DEVICE_HAS_DRONECAN_W_MAV_OVER_CAN

#include "../Common/libs/fifo.h"

#include "../../../modules/stm32-dronecan-lib/stm32-dronecan-driver.h"
#include "../../../modules/stm32-dronecan-lib/stm32-dronecan-protocol.h"
#include "../Common/dronecan/out/include/uavcan.protocol.NodeStatus.h"
#include "../Common/dronecan/out/include/uavcan.protocol.GetNodeInfo.h"
#include "../Common/dronecan/out/include/dronecan.sensors.rc.RCInput.h"
#include "../Common/dronecan/out/include/uavcan.protocol.dynamic_node_id.Allocation.h"
#include "../Common/dronecan/out/include/uavcan.tunnel.Targetted.h"
#include "../Common/dronecan/out/include/uavcan.tunnel.Protocol.h"


#define DRONECAN_BUF_SIZE  512 // needs to be larger than the largest DroneCAN frame


//-------------------------------------------------------
// RxDroneCan class
//-------------------------------------------------------

class tRxDroneCan
{
  public:
    void Init(void);
    void Tick_ms(void);
    void Do(void);
    void SendRcData(tRcData* const rc_out, bool failsafe);

    void send_node_status(void);
    void handle_get_node_info_request(CanardInstance* const ins, CanardRxTransfer* const transfer);
    void handle_dynamic_node_id_allocation_broadcast(CanardInstance* const ins, CanardRxTransfer* const transfer);
    void send_dynamic_node_id_allocation_request(void);

  private:
    int16_t set_can_filters(void);

    uint16_t tick_1Hz;

    uint8_t node_status_transfer_id; // is this per message ? it seems so ...

    uint8_t rc_input_transfer_id;
    uint32_t rc_input_tlast_ms;

    uint8_t node_id_allocation_transfer_id;
    struct {
        uint32_t send_next_request_at_ms;
        uint32_t unique_id_offset;
    } node_id_allocation;
    bool node_id_allocation_running;

    // to not burden the stack
    union {
        struct uavcan_protocol_NodeStatus node_status;
        struct uavcan_protocol_GetNodeInfoResponse node_info_resp;
        struct dronecan_sensors_rc_RCInput rc_input;
        struct uavcan_tunnel_Targetted tunnel_targetted;
    } _p;
    uint8_t _buf[DRONECAN_BUF_SIZE];
};


#else

class tRxDroneCan
{
  public:
    void Init(void) {}
    void Tick_ms(void) {}
    void Do(void) {}
    void SendRcData(tRcData* const rc_out, bool failsafe) {}
};


#endif // DEVICE_HAS_DRONECAN

#endif // DRONECAN_TYPES_RX_H
