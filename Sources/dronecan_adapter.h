/*
 * dronecan_adapter.h
 *
 *  Created on: Apr 2, 2025
 *      Author: karthik
 */

#ifndef SOURCES_DRONECAN_ADAPTER_H_
#define SOURCES_DRONECAN_ADAPTER_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <canard.h>
#include <canard_stm32.h>
#include <dronecan_msgs.h>

// Function Prototypes
uint64_t micros64(void);
uint32_t millis32(void);
void getUniqueID(uint8_t id[16]);
void processTxRxOnce(void);
bool shouldAcceptTransfer(const CanardInstance *ins,
                          uint64_t *out_data_type_signature,
                          uint16_t data_type_id,
                          CanardTransferType transfer_type,
                          uint8_t source_node_id);
void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer);
void init_dronecan(void);
void tick_dronecan(void);

// External Variables
extern CanardInstance canard;
extern bool connected;
extern uint64_t next_1hz_service_at;
extern uint32_t send_next_node_id_allocation_request_at_ms;
extern CanardPoolAllocatorBlock memory_pool[];
extern CanardSTM32Stats CNT;
extern CanardSTM32CANTimings timings;

#endif /* SOURCES_DRONECAN_ADAPTER_H_ */
