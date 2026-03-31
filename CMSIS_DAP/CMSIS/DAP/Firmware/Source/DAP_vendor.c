/*
 * Copyright (c) 2013-2017 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law/agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * ----------------------------------------------------------------------
 *
 * $Date:        1. December 2017
 * $Revision:    V2.0.0
 *
 * Project:      CMSIS-DAP Source
 * Title:        DAP_vendor.c CMSIS-DAP Vendor Commands
 *
 *---------------------------------------------------------------------------*/

#include "DAP_config.h"
#include "DAP.h"

//**************************************************************************************************
/**
\defgroup DAP_Vendor_Adapt_gr Adapt Vendor Commands
\ingroup DAP_Vendor_gr
@{

The file DAP_vendor.c provides template source code for extension of a Debug Unit with
Vendor Commands. Copy this file to the project folder of theDebug Unit and add the
file to the MDK-ARM project under the file group Configuration.
*/

// Custom DAP Vendor Command IDs for TraceRip
#define ID_DAP_TraceRip_DelayBusRead      0x80U   // Vendor0: Delay and Bus Read

// Additional constants for bus read operations
#ifndef DP_READ
#define DP_READ                           0x83U   // SWD/JTAG Read access (APnDP=0, RnW=1, A2=0, A3=0)
#endif

/**
 * @brief Execute precise delay followed by a bus read operation
 *
 * This command waits for a specified delay time and then issues a bus read
 * to capture target state during fault injection attacks.
 *
 * Request format:
 *   Byte 0: Command ID (ID_DAP_TraceRip_DelayBusRead)
 *   Byte 1-2: Delay time in microseconds (16-bit little-endian)
 *
 * Response format:
 *   Byte 0: Status (0 = success, 0xFF = error)
 *   Byte 1-4: Bus read value (32-bit little-endian)
 *   Byte 5-8: Timestamp (32-bit little-endian)
 */
static uint64_t TraceRip_DelayBusRead(uint64_t delay_us) {
  uint64_t result = 0ULL;
  uint32_t status = DAP_OK;
  uint32_t bus_value = 0U;
  uint32_t timestamp = 0U;

#if (DAP_SWD != 0) || (DAP_JTAG != 0)
  // Convert microseconds to clock cycles based on current clock speed
  // Assuming typical DAP clock of 1MHz (1 cycle = 1 microsecond)
  uint64_t delay_cycles = delay_us * DAP_Data.clock_delay;

  // Precise delay using PIN_DELAY_SLOW
  PIN_DELAY_SLOW((uint32_t)delay_cycles);

  // Issue bus read to DP_RDBUFF register
  #if (DAP_SWD != 0)
    SWD_Transfer(DP_READ | DP_RDBUFF, &bus_value);
  #else
    JTAG_Transfer(DP_READ | DP_RDBUFF, &bus_value);
  #endif

  // Capture timestamp
  timestamp = TIMESTAMP_GET();
#else
  // Fallback delay without hardware support
  PIN_DELAY_SLOW((uint32_t)(delay_us * 10U));
  bus_value = 0xDEADBEEF;
  timestamp = TIMESTAMP_GET();
#endif

  // Build result: [Status(8)] [BusValue(32)] [Timestamp(32)]
  // Using uint64_t to avoid shift overflow when extracting individual fields
  result |= ((uint64_t)status << 56);             // Status in upper 8 bits
  result |= ((uint64_t)bus_value << 24);          // Bus value in next 32 bits
  result |= ((uint64_t)timestamp);                // Timestamp in lower 32 bits

  return result;
}

/** Process DAP Vendor Command and prepare Response Data
\param request   pointer to request data
\param response  pointer to response data
\return          number of bytes in response (lower 16 bits)
                 number of bytes in request (upper 16 bits)
*/
uint32_t DAP_ProcessVendorCommand(const uint8_t *request, uint8_t *response) {
  uint32_t num = (1U << 16) | 1U;

  *response++ = *request;        // copy Command ID

  switch (*request++) {          // first byte in request is Command ID
    case ID_DAP_TraceRip_DelayBusRead:
      num += 1U << 16;           // increment request count

      // Read delay parameter (16-bit little-endian)
      uint32_t delay_us = ((uint32_t)(*request)) | (((uint32_t)(*request + 1U)) << 8U);

      // Execute delay and bus read operation
      // Use uint64_t for result to support shifts > 32 bits
      uint64_t result = TraceRip_DelayBusRead(delay_us);

      // Write response: status, bus_value[3], timestamp[3]
      *response++ = (uint8_t)(result >> 56);                    // Status [7:0]
      *response++ = (uint8_t)((result >> 48) & 0xFFU);          // Bus Value [31:24]
      *response++ = (uint8_t)((result >> 40) & 0xFFU);          // Bus Value [23:16]
      *response++ = (uint8_t)((result >> 32) & 0xFFU);          // Bus Value [15:8]
      *response++ = (uint8_t)((result >> 24) & 0xFFU);          // Timestamp [31:24]
      *response++ = (uint8_t)((result >> 16) & 0xFFU);          // Timestamp [23:16]
      *response++ = (uint8_t)((result >> 8) & 0xFFU);           // Timestamp [15:8]
      *response++ = (uint8_t)(result & 0xFFU);                  // Timestamp [7:0]

      num += 7U;                // increment response count by 7 bytes

      break;

    case ID_DAP_Vendor1:  break;
    case ID_DAP_Vendor2:  break;
    case ID_DAP_Vendor3:  break;
    case ID_DAP_Vendor4:  break;
    case ID_DAP_Vendor5:  break;
    case ID_DAP_Vendor6:  break;
    case ID_DAP_Vendor7:  break;
    case ID_DAP_Vendor8:  break;
    case ID_DAP_Vendor9:  break;
    case ID_DAP_Vendor10: break;
    case ID_DAP_Vendor11: break;
    case ID_DAP_Vendor12: break;
    case ID_DAP_Vendor13: break;
    case ID_DAP_Vendor14: break;
    case ID_DAP_Vendor15: break;
    case ID_DAP_Vendor16: break;
    case ID_DAP_Vendor17: break;
    case ID_DAP_Vendor18: break;
    case ID_DAP_Vendor19: break;
    case ID_DAP_Vendor20: break;
    case ID_DAP_Vendor21: break;
    case ID_DAP_Vendor22: break;
    case ID_DAP_Vendor23: break;
    case ID_DAP_Vendor24: break;
    case ID_DAP_Vendor25: break;
    case ID_DAP_Vendor26: break;
    case ID_DAP_Vendor27: break;
    case ID_DAP_Vendor28: break;
    case ID_DAP_Vendor29: break;
    case ID_DAP_Vendor30: break;
    case ID_DAP_Vendor31: break;
  }

  return (num);
}

///@}
