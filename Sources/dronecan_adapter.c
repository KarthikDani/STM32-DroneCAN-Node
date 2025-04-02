/*
 * dronecan_adapter.c
 *
 *  Created on: Apr 2, 2025
 *      Author: karthik
 */

// --------------------------
//#include <canard.h>
//#include <canard_stm32.h>
//#include <dronecan_msgs.h>
# include "dronecan_adapter.h"
// --------------------------

// some helper functions:
uint64_t micros64(void) {
  return HAL_GetTick() * 1000;
}

uint32_t millis32(void) {
  return HAL_GetTick();
}

void getUniqueID(uint8_t id[16]) {
  memset(id, 0, 16);
  // use stm32 unique id
  uint32_t uid[3];
  uid[0] = HAL_GetUIDw0();  // 0-3
  uid[1] = HAL_GetUIDw1();  // 4-7
  uid[2] = HAL_GetUIDw2();  // 8-11
  memcpy(id, uid, sizeof(uid));
  // TODO 12-15 is empty, you can add something here if you want
}

void processTxRxOnce() {
  // Transmitting
  for (const CanardCANFrame *txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;) {
    const int16_t tx_res = canardSTM32Transmit(txf);
    if (tx_res < 0) {  // Failure - drop the frame
      canardPopTxQueue(&canard);
    } else if (tx_res > 0) {  // Success - just drop the frame
      canardPopTxQueue(&canard);
    } else {  // Timeout - just exit and try again later
      break;
    }
  }

  // Receiving
  CanardCANFrame rx_frame;

  const uint64_t timestamp = micros64();
  const int16_t  rx_res    = canardSTM32Receive(&rx_frame);
  if (rx_res < 0) {
    CNT_ReceiveErrors++;
  } else if (rx_res > 0) {  // Success - process the frame
    canardHandleRxFrame(&canard, &rx_frame, timestamp);
  }
}

bool shouldAcceptTransfer(const CanardInstance *ins __attribute__((unused)),
                          uint64_t             *out_data_type_signature,
                          uint16_t              data_type_id,
                          CanardTransferType    transfer_type,
                          uint8_t               source_node_id __attribute__((unused))) {
  if (transfer_type == CanardTransferTypeRequest) {
    if (!connected) {
      return false;  // can not get message if not connected (and therefore no node id)
    }
    switch (data_type_id) {
      case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
        *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
        return true;
      }
      case UAVCAN_PROTOCOL_PARAM_GETSET_ID: {
        *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
        return true;
      }
      case UAVCAN_PROTOCOL_RESTARTNODE_ID: {
        *out_data_type_signature = UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE;
        return true;
      }
    }
  } else if (transfer_type == CanardTransferTypeBroadcast) {
    switch (data_type_id) {
      if (!connected) {
        *out_data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
        return true;
      } else {  // ignore if already connected
        return false;
      }
      case UAVCAN_PROTOCOL_NODESTATUS_ID: {
        return false;
      }
    }
  }
  return false;
}

//TODO you can copy the handle_* functions from the examples in the libcanard project

void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer) {
  if (transfer->transfer_type == CanardTransferTypeRequest) {
    switch (transfer->data_type_id) {
      case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
        handle_GetNodeInfo(ins, transfer);
        break;
      }
      case UAVCAN_PROTOCOL_RESTARTNODE_ID: {
        handle_RestartNode();
        break;
      }
    }
  } else if (transfer->transfer_type == CanardTransferTypeBroadcast) {
    switch (transfer->data_type_id) {
      case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID: {
        handle_DNA_Allocation(ins, transfer); //TODO you need to set connected = true when DNA is finished
        break;
      }
    }
  }
}

void init_dronecan() {
  canardSTM32Init(&timings, CanardSTM32IfaceModeNormal);
  canardInit(&canard,
             memory_pool,
             sizeof(memory_pool),
             onTransferReceived,
             shouldAcceptTransfer,
             NULL);

  next_1hz_service_at = micros64();
}

void tick_dronecan() {
  processTxRxOnce();
  CNT.stats = canardSTM32GetStats();

  if (canardGetLocalNodeID(&canard) == CANARD_BROADCAST_NODE_ID) {
    connected = false;
    if (millis32() >= send_next_node_id_allocation_request_at_ms) {
      request_DNA();
    }
  }

  const uint64_t ts = micros64();
  if (ts >= next_1hz_service_at) {
    next_1hz_service_at += 1000000ULL;
    canardCleanupStaleTransfers(&canard, ts);
    if (connected) {
      send_NodeStatus();
    }
  }
}
