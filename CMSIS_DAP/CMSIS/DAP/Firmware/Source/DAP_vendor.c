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

/**
 * \brief Read a target register at a specific address
 *
 * Reads a 32-bit register value from the target device at the given address.
 * Supports both AP (Access Port) and DP (Debug Port) accesses via SWD.
 *
 * \param addr      Target register address (AP or DP register address)
 * \param data      Pointer to store the read register value
 * \param is_ap     Flag indicating if accessing an AP register (1) or DP register (0)
 *
 * \return          DAP_OK on success, DAP_ERROR on failure
 *
 * \note            For AP accesses, addr should be in the range 0x0-0xFFF
 *                  For DP accesses, addr typically uses standard DP addresses
 */
uint8_t DAP_ReadTargetRegister(uint32_t addr, uint32_t *data) {
    uint8_t response;
    /* Validate input parameters */
    if (data == NULL) {
        return DAP_ERROR;
    }

    // CSW: Control/Status Word
    // TAR: Transfer Address Register
    // DRW: Data Read/Write Register
    response = SWD_Transfer(DAP_TRANSFER_APnDP | 0x04, &addr); // AP Write TAR, WDATA 0x08000000 reg TAR
    if (response != DAP_TRANSFER_OK)
        return DAP_ERROR;

    uint32_t nll = 0LL;
    response = SWD_Transfer(DAP_TRANSFER_APnDP | 0xFF, &nll); // AP Read DRW, WDATA 0x00 reg DRW
    if (response != DAP_TRANSFER_OK)
        return DAP_ERROR;
    
    response = SWD_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, data); // DP Read RDBUFF, WDATA <response>
    if (response != DAP_TRANSFER_OK)
        return DAP_ERROR;

    return DAP_OK;
}

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
 * Result format:
 *   Byte 0-3: Bus read value (32-bit little-endian)
 *   Byte 4-7: Timestamp (32-bit little-endian)
 *
 * Return value: Error code.
 */
static uint8_t TraceRip_DelayBusRead(uint32_t delay_us, uint32_t address, uint64_t *result) {
    *result = 0ULL;
    uint8_t status = DAP_OK;
    
    // reset target
    //RESET_TARGET();

    // Precise delay using PIN_DELAY_SLOW
    // Convert microseconds to clock cycles based on current clock speed
    uint64_t delay_cycles = delay_us * ((CPU_CLOCK/1000000U) + (DELAY_SLOW_CYCLES-1U)) / DELAY_SLOW_CYCLES;
    if (delay_cycles > 0)
        PIN_DELAY_SLOW((uint32_t)delay_cycles);
    
    // read from given address
    uint32_t reg_value = 0U;
    status = DAP_ReadTargetRegister(address, &reg_value);

    // Capture timestamp
    uint32_t timestamp = TIMESTAMP_GET();

    // Build result: [Status(8)] [RegValue(32)]
    // Using uint64_t to avoid shift overflow when extracting individual fields
    *result |= ((uint64_t)reg_value << 32);  // register value in the upper 32 bits
    *result |= ((uint64_t)timestamp);        // Timestamp in lower 32 bits

    return status;
}

/** Process DAP Vendor Command and prepare Response Data
\param request   pointer to request data
\param response  pointer to response data
\return          number of bytes in response (lower 16 bits)
                 number of bytes in request (upper 16 bits)
*/
uint32_t DAP_ProcessVendorCommand(const uint8_t *request, uint8_t *response) {
    // at least one byte in request -> increase request count by one
    uint32_t num = (1U << 16) | 1U;

    // copy Command ID
    *response++ = *request;

    // first byte in request is Command ID
    switch (*request++) {
        case ID_DAP_Vendor0:
            // six bytes in request -> increment request count by six
            num += 6U << 16;

            // Read delay parameter (16-bit)
            uint32_t delay_us = (((uint32_t)(*request)) << 8U) | (((uint32_t)(*(request + 1))) << 0U);

            // read address parameter
            uint32_t address  = (((uint32_t)(*(request + 2))) << 24U)
                              | (((uint32_t)(*(request + 3))) << 16U)
                              | (((uint32_t)(*(request + 4))) << 8U) 
                              | (((uint32_t)(*(request + 5))) << 0U);
            
            // perform read
            uint64_t result;
            uint8_t status = TraceRip_DelayBusRead(delay_us, address, &result);

            // Write response: status, bus_value[3], timestamp[3]
            *response++ = (uint8_t)(status);
            *response++ = (uint8_t)((result >> 56) & 0xFFU);          // register value [31:24]
            *response++ = (uint8_t)((result >> 48) & 0xFFU);          // register value [23:16]
            *response++ = (uint8_t)((result >> 40) & 0xFFU);          // register value [15:8]
            *response++ = (uint8_t)((result >> 32) & 0xFFU);          // register value [7:0]
            *response++ = (uint8_t)((result >> 24) & 0xFFU);          // Timestamp [31:24]
            *response++ = (uint8_t)((result >> 16) & 0xFFU);          // Timestamp [23:16]
            *response++ = (uint8_t)((result >> 8) & 0xFFU);           // Timestamp [15:8]
            *response++ = (uint8_t)(result & 0xFFU);                  // Timestamp [7:0]

            num += 9U;// increment response count by 9 bytes
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
