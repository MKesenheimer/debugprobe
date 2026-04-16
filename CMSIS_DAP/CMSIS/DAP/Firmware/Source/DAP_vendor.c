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
#include <malloc.h>

//**************************************************************************************************
/**
\defgroup DAP_Vendor_Adapt_gr Adapt Vendor Commands
\ingroup DAP_Vendor_gr
@{

The file DAP_vendor.c provides template source code for extension of a Debug Unit with
Vendor Commands. Copy this file to the project folder of theDebug Unit and add the
file to the MDK-ARM project under the file group Configuration.
*/

#define BANK_REG(bank, reg)    (((bank) << 4) | (reg))
#define DP_DPIDR        BANK_REG(0x0, 0x0) /* DPv1+: ro */
//#define DP_ABORT        BANK_REG(0x0, 0x0) /* DPv1+: SWD: wo */
#define DP_DPIDR1       BANK_REG(0x1, 0x0) /* DPv3: ro */
#define DP_BASEPTR0     BANK_REG(0x2, 0x0) /* DPv3: ro */
#define DP_BASEPTR1     BANK_REG(0x3, 0x0) /* DPv3: ro */
//#define DP_CTRL_STAT    BANK_REG(0x0, 0x4) /* DPv0+: rw */
#define DP_DLCR         BANK_REG(0x1, 0x4) /* DPv1+: SWD: rw */
#define DP_TARGETID     BANK_REG(0x2, 0x4) /* DPv2: ro */
#define DP_DLPIDR       BANK_REG(0x3, 0x4) /* DPv2: ro */
#define DP_EVENTSTAT    BANK_REG(0x4, 0x4) /* DPv2: ro */
#define DP_SELECT1      BANK_REG(0x5, 0x4) /* DPv3: ro */
//#define DP_RESEND       BANK_REG(0x0, 0x8) /* DPv1+: SWD: ro */
//#define DP_SELECT       BANK_REG(0x0, 0x8) /* DPv0+: JTAG: rw; SWD: wo */
//#define DP_RDBUFF       BANK_REG(0x0, 0xC) /* DPv0+: ro */
#define DP_TARGETSEL    BANK_REG(0x0, 0xC) /* DPv2: SWD: wo */

/* Fields of the DP's AP ABORT register */
#define DAPABORT        (1UL << 0)
#define STKCMPCLR       (1UL << 1)
#define STKERRCLR       (1UL << 2)
#define WDERRCLR        (1UL << 3)
#define ORUNERRCLR      (1UL << 4)

#define SWD_CMD_START   (1 << 0)
#define SWD_CMD_APNDP   (1 << 1)
#define SWD_CMD_RNW     (1 << 2)
#define SWD_CMD_A32     (3 << 3)
#define SWD_CMD_PARITY  (1 << 5)
#define SWD_CMD_STOP    (0 << 6)
#define SWD_CMD_PARK    (1 << 7)

/* Fields of the DP's CTRL/STAT register */
#define CORUNDETECT     (1UL << 0)
#define SSTICKYORUN     (1UL << 1)
/* 3:2 - transaction mode (e.g. pushed compare) */
#define SSTICKYCMP      (1UL << 4)
#define SSTICKYERR      (1UL << 5)
#define READOK          (1UL << 6) /* SWD-only */
#define WDATAERR        (1UL << 7) /* SWD-only */
#define CDBGRSTREQ      (1UL << 26)
#define CDBGRSTACK      (1UL << 27)
#define CDBGPWRUPREQ    (1UL << 28)
#define CDBGPWRUPACK    (1UL << 29)
#define CSYSPWRUPREQ    (1UL << 30)
#define CSYSPWRUPACK    (1UL << 31)

#define ADIV5_AP_REG_IDR        (0xFC)		/* RO: Identification Register */
#define AP_REG_IDR(dap)         (ADIV5_AP_REG_IDR)
#define ADIV5_DP_SELECT_APSEL	0xFF000000
#define ADIV5_DP_SELECT_APBANK	0x000000F0
#define DP_SELECT_DPBANK		0x0000000F

#define MEM_AP_REG_CSW    (0x00)
#define MEM_AP_REG_TAR    (0x04)
#define MEM_AP_REG_TAR64  (0x08)		/* RW: Large Physical Address Extension */
#define MEM_AP_REG_DRW    (0x0C)		/* RW: Data Read/Write register */
#define MEM_AP_REG_BD0    (0x10)		/* RW: Banked Data register 0-3 */
#define MEM_AP_REG_BD1    (0x14)
#define MEM_AP_REG_BD2    (0x18)
#define MEM_AP_REG_BD3    (0x1C)
#define MEM_AP_REG_MBT    (0x20)		/* --: Memory Barrier Transfer register */
#define MEM_AP_REG_BASE64 (0xF0)		/* RO: Debug Base Address (LA) register */
#define MEM_AP_REG_CFG    (0xF4)		/* RO: Configuration register */
#define MEM_AP_REG_BASE   (0xF8)		/* RO: Debug Base Address register */

/**
 * JTAG-to-SWD sequence.
 *
 * The JTAG-to-SWD sequence is at least 50 TCK/SWCLK cycles with TMS/SWDIO
 * high, putting either interface logic into reset state, followed by a
 * specific 16-bit sequence and finally a line reset in case the SWJ-DP was
 * already in SWD mode.
 * Bits are stored (and transmitted) LSB-first.
 */
static const uint8_t swd_seq_jtag_to_swd[] = {
    /* At least 50 TCK/SWCLK cycles with TMS/SWDIO high */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    /* Switching sequence from JTAG to SWD */
    0x9e, 0xe7,
    /* At least 50 TCK/SWCLK cycles with TMS/SWDIO high */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    /* At least 2 idle (low) cycles */
    0x00,
};
static const unsigned int swd_seq_jtag_to_swd_len = 136;

static inline int parity_u32(uint32_t x) {
    x ^= x >> 16;
    x ^= x >> 8;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    return x & 1;
}

static inline uint8_t swd_cmd(bool is_read, bool is_ap, uint8_t regnum) {
    uint8_t cmd = (is_ap ? SWD_CMD_APNDP : 0)
        | (is_read ? SWD_CMD_RNW : 0)
        | ((regnum & 0xc) << 1);

    /* 8 cmd bits 4:1 may be set */
    if (parity_u32(cmd))
        cmd |= SWD_CMD_PARITY;
    return cmd;
}

void *buf_set_buf(const void *_src, unsigned int src_start, void *_dst, unsigned int dst_start, unsigned int len) {
    const uint8_t *src = _src;
    uint8_t *dst = _dst;
    unsigned int i, sb, db, sq, dq, lb, lq;

    sb = src_start / 8;
    db = dst_start / 8;
    sq = src_start % 8;
    dq = dst_start % 8;
    lb = len / 8;
    lq = len % 8;

    src += sb;
    dst += db;

    /* check if both buffers are on byte boundary and
     * len is a multiple of 8bit so we can simple copy
     * the buffer */
    if ((sq == 0) && (dq == 0) &&  (lq == 0)) {
        for (i = 0; i < lb; i++)
            *dst++ = *src++;
        return _dst;
    }

    /* fallback to slow bit copy */
    for (i = 0; i < len; i++) {
        if (((*src >> (sq&7)) & 1) == 1)
            *dst |= 1 << (dq&7);
        else
            *dst &= ~(1 << (dq&7));
        if (sq++ == 7) {
            sq = 0;
            src++;
        }
        if (dq++ == 7) {
            dq = 0;
            dst++;
        }
    }

    return _dst;
}

static inline void bit_copy(uint8_t *dst, unsigned int dst_offset, const uint8_t *src, unsigned int src_offset, unsigned int bit_count) {
    buf_set_buf(src, src_offset, dst, dst_offset, bit_count);
}

/* clock a sequence of bits out on TMS, to change JTAG states */
static uint32_t DAP_SendSequence(uint8_t s_len, const uint8_t *sequence) {
    uint8_t *request = (uint8_t *)malloc((s_len + 2)*sizeof(uint8_t));
    request[0] = ID_DAP_SWJ_Sequence;
    request[1] = s_len;
    bit_copy(&request[2], 0, sequence, 0, s_len);
    uint8_t response[2];
    uint32_t status = DAP_ProcessCommand(request, &response[0]);
    free(request);
    return status;
}

/* Get Chip IDCode */
static uint32_t DAP_GetIDCode() {
    uint32_t dpidr = 0xdeadbeef;
    // details:
    // openocd::adi_v5_swd.c::swd_connect_single
    // openocd::adi_v5_swd.c::swd_queue_dp_read_inner
    // openocd::cmsis_dap.c::cmsis_dap_swd_write_from_queue
    uint8_t cmd = (swd_cmd(true, false, DP_DPIDR) >> 1) & 0x0f; 
    SWD_Transfer(cmd, &dpidr);
    return dpidr;
}

/* Clear sticky errors */
static void DAP_ClearStickyErrors() {
    // details:
    // openocd::adi_v5_swd.c::swd_connect_single
    // openocd::adi_v5_swd.c::swd_clear_sticky_error
    // openocd::cmsis_dap.c::cmsis_dap_swd_write_from_queue
    uint8_t cmd = (swd_cmd(false, false, DP_ABORT) >> 1) & 0x0f;
    uint32_t data = STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR;
    SWD_Transfer(cmd, &data);

    cmd = (swd_cmd(true, false, DP_RDBUFF) >> 1) & 0x0f;
    uint32_t data2 = 0LL;
    SWD_Transfer(cmd, &data2);
}

/* Power-up system and debug domains, if not done before. */
static void DAP_Select() {
    // details:
    // <TODO>
    uint8_t cmd = (swd_cmd(false, false, DP_SELECT) >> 1) & 0x0f;
    uint32_t data = 0LL;
    SWD_Transfer(cmd, &data);
}

/* Power-up system and debug domains, if not done before. */
static void DAP_PowerUpDebugDomain() {
    // details:
    // openocd::arm_adi_v5.c::dap_dp_init
    // write: W CTRL/STAT -> OK -> 0x50000022
    uint8_t cmd = (swd_cmd(false, false, DP_CTRL_STAT) >> 1) & 0x0f;
    uint32_t data = CDBGPWRUPREQ | CSYSPWRUPREQ | SSTICKYERR | SSTICKYORUN;
    SWD_Transfer(cmd, &data);

    // read: R CTRL/STAT -> OK -> 0xf0000040
    cmd = (swd_cmd(true, false, DP_CTRL_STAT) >> 1) & 0x0f;
    uint32_t data2 = 0LL;
    SWD_Transfer(cmd, &data2);

    // write: W CTRL/STAT -> OK -> 0x50000000
    cmd = (swd_cmd(false, false, DP_CTRL_STAT) >> 1) & 0x0f;
    data = CDBGPWRUPREQ | CSYSPWRUPREQ;
    SWD_Transfer(cmd, &data);

    // read: R CTRL/STAT -> OK -> 0xf0000040
    cmd = (swd_cmd(true, false, DP_CTRL_STAT) >> 1) & 0x0f;
    data2 = 0LL;
    SWD_Transfer(cmd, &data2);

    // read: R CTRL/STAT -> OK -> 0xf0000040
    cmd = (swd_cmd(true, false, DP_CTRL_STAT) >> 1) & 0x0f;
    data2 = 0LL;
    SWD_Transfer(cmd, &data2);

    // read: R CTRL/STAT -> OK -> 0xf0000040
    cmd = (swd_cmd(true, false, DP_CTRL_STAT) >> 1) & 0x0f;
    data2 = 0LL;
    SWD_Transfer(cmd, &data2);

    // write: W CTRL/STAT -> OK -> 0x50000000
    cmd = (swd_cmd(false, false, DP_CTRL_STAT) >> 1) & 0x0f;
    data = CDBGPWRUPREQ | CSYSPWRUPREQ;
    SWD_Transfer(cmd, &data);

    // read: R CTRL/STAT -> OK -> 0xf0000040
    cmd = (swd_cmd(true, false, DP_CTRL_STAT) >> 1) & 0x0f;
    data2 = 0LL;
    SWD_Transfer(cmd, &data2);
}

static void swd_queue_ap_read(uint32_t reg) {
    // read: R APx -> OK -> 0x00000000
    uint8_t cmd = (swd_cmd(true, true, reg) >> 1) & 0x0f;
    uint32_t data = 0LL;
    SWD_Transfer(cmd, &data);

    // read: RDBUFF -> OK -> 0x04770031
    cmd = (swd_cmd(true, false, DP_RDBUFF) >> 1) & 0x0f;
    uint32_t data2 = 0LL;
    SWD_Transfer(cmd, &data2);

    // read: RDBUFF -> OK -> 0x00000000
    cmd = (swd_cmd(true, false, DP_RDBUFF) >> 1) & 0x0f;
    data2 = 0LL;
    SWD_Transfer(cmd, &data2);
}

/*
 * This function checks the ID for each access port to find the requested Access Port type
 */
static void DAP_SearchMemAP() {
    // details:
    // Prepare DP bank for DP_SELECT1 now to save one write
    // cortex_m.c::cortex_m_examine
    // cortex_m.c::cortex_m_find_mem_ap
    // arm_adi_v5.c::dap_find_by_types_get_ap
    // arm_adi_v5.h::dap_queue_ap_read
    // adi_v5_swd.c::swd_queue_ap_read
    // adi_v5_swd.c::swd_queue_ap_read -> swd_check_reconnect -> does nothing in this case
    // adi_v5_swd.c::swd_queue_ap_read -> swd_multidrop_select -> is not called in this case
    // adi_v5_swd.c::swd_queue_ap_read -> swd_queue_ap_bankselect -> 
    // adi_v5_swd.c::swd_queue_ap_read -> read_reg -> 
    // adi_v5_swd.c::swd_queue_ap_bankselect
    // write: W SELECT -> OK -> 0x000000f0
    uint32_t ap_num = 0;
    uint32_t dap_select = 0;
    uint32_t reg = AP_REG_IDR(0);
    uint32_t sel = (ap_num << 24) | (reg & ADIV5_DP_SELECT_APBANK);
    sel |= dap_select & DP_SELECT_DPBANK;
    uint8_t cmd = (swd_cmd(false, false, DP_SELECT) >> 1) & 0x0f;
    SWD_Transfer(cmd, &sel);

    swd_queue_ap_read(reg);
}    

/*
 * This function checks the ID for each access port to find the requested Access Port type
 */
static void DAP_MemAPInit() {
    // details:
    // cortex_m.c::cortex_m_examine
    // cortex_m.c::cortex_m_find_mem_ap
    // arm_adi_v5.c::mem_ap_init
    
    swd_queue_ap_read(MEM_AP_REG_CFG);
}

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

    //  AP: Access Port
    //  DP: Debug Port
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

    // store swd state
    uint8_t swdio_state = PIN_SWDIO_IN();
    uint32_t saved_clock_delay = DAP_Data.clock_delay;
    uint8_t saved_fast_clock = DAP_Data.fast_clock;
    uint8_t saved_idle_cycles = DAP_Data.transfer.idle_cycles;
    uint16_t saved_retry_count = DAP_Data.transfer.retry_count;
    uint16_t saved_match_retry = DAP_Data.transfer.match_retry;
    uint8_t saved_turnaround = DAP_Data.swd_conf.turnaround;
    uint8_t saved_data_phase = DAP_Data.swd_conf.data_phase;
    PIN_SWDIO_OUT_DISABLE(); // Switch to SWD Input mode

    // power off and reset target
    probe_assert_vtarget(0);
    probe_assert_reset(0);
    Delayus(100000); // delay until power down

    // power on target
    probe_assert_vtarget(1);
    Delayus(10000); // delay until power up

    // configure debug block
    DAP_Data.transfer.idle_cycles = 0; // orig 0
    DAP_Data.transfer.retry_count = 1; // orig 64
    DAP_Data.transfer.match_retry = 0; // orig 0
    DAP_Data.swd_conf.turnaround  = 1;  // orig 1
    DAP_Data.swd_conf.data_phase  = 0;  // orig 0

    // JTAG -> SWD
    DAP_SendSequence(swd_seq_jtag_to_swd_len, swd_seq_jtag_to_swd);

    // get IDCODE
    // IDCODE -> OK -> 0x0bc11477
    DAP_GetIDCode();

    // clear errors
    // W ABORT -> OK -> 0x0000001e
    // RDBUFF  -> OK -> 0x00000000
    DAP_ClearStickyErrors();

    // bank select
    // W SELECT -> OK -> 0x00000000
    DAP_Select();

    // power up debug domain
    // W CTRL/STAT -> OK -> 0x50000022
    // R CTRL/STAT -> OK -> 0xf0000040
    // W CTRL/STAT -> OK -> 0x50000000
    // R CTRL/STAT -> OK -> 0xf0000040
    // R CTRL/STAT -> OK -> 0xf0000040
    // R CTRL/STAT -> OK -> 0xf0000040
    // W CTRL/STAT -> OK -> 0x50000001
    // R CTRL/STAT -> OK -> 0xf0000041
    DAP_PowerUpDebugDomain();

    // Search Mem-AP
    // W SELECT -> OK -> 0x000000f0
    // R APc -> OK -> 0x00000000
    // RDBUFF -> OK -> 0x04770031
    // RDBUFF -> OK -> 0x00000000
    //DAP_SearchMemAP();

    // Mem init
    // R AP4 -> OK -> 0x00000000
    // RDBUFF -> OK -> 0x00000000
    // RDBUFF -> OK -> 0x00000000
    //DAP_MemAPInit();

    // release reset
    probe_assert_reset(1);

    // Precise delay using PIN_DELAY_SLOW
    if (delay_us > 0)
        Delayus(delay_us);

    // read from given address and create a hardware fault if rdp 1 is set
    uint32_t reg_value = 0U;
    status = DAP_ReadTargetRegister(address, &reg_value);

    // Capture timestamp
    uint32_t timestamp = TIMESTAMP_GET();

    // Build result: [Status(8)] [RegValue(32)]
    // Using uint64_t to avoid shift overflow when extracting individual fields
    *result |= ((uint64_t)reg_value << 32);  // register value in the upper 32 bits
    *result |= ((uint64_t)timestamp);        // Timestamp in lower 32 bits

    // restore swd state
    DAP_Data.clock_delay = saved_clock_delay;
    DAP_Data.fast_clock = saved_fast_clock;
    DAP_Data.transfer.idle_cycles = saved_idle_cycles;
    DAP_Data.transfer.retry_count = saved_retry_count;
    DAP_Data.transfer.match_retry = saved_match_retry;
    DAP_Data.swd_conf.turnaround = saved_turnaround;
    DAP_Data.swd_conf.data_phase = saved_data_phase;
    PIN_SWDIO_OUT_ENABLE();
    PIN_SWDIO_OUT(swdio_state);

    // clear errors
    // W ABORT -> OK -> 0x0000001e
    // RDBUFF  -> OK -> 0x00000000
    //DAP_ClearStickyErrors();

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
            num += 8U << 16;

            // Read delay parameter (16-bit)
            uint32_t delay_us =   (((uint32_t)(*(request + 0))) << 24U) 
                                | (((uint32_t)(*(request + 1))) << 16U) 
                                | (((uint32_t)(*(request + 2))) << 8U) 
                                | (((uint32_t)(*(request + 3))) << 0U);

            // read address parameter
            uint32_t address  = (((uint32_t)(*(request + 4))) << 24U)
                              | (((uint32_t)(*(request + 5))) << 16U)
                              | (((uint32_t)(*(request + 6))) << 8U) 
                              | (((uint32_t)(*(request + 7))) << 0U);
            
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
