# DW3000 Driver API Reference Guide

**Driver:** DW3XXX Device Driver Version 06.00.14
**Chip Family:** DW3000, DW3700, DW3720 (QM33110, QM33120)
**Copyright:** Decawave Ltd, Dublin, Ireland (2013–2021)

---

## Table of Contents

1. [Overview](#1-overview)
2. [Types & Constants](#2-types--constants)
3. [Key Data Structures](#3-key-data-structures)
4. [Enumerations](#4-enumerations)
5. [API Functions by Category](#5-api-functions-by-category)
   - [Device Init & Management](#51-device-init--management)
   - [Configuration](#52-configuration)
   - [TX / RX Frame Operations](#53-tx--rx-frame-operations)
   - [Timestamps & Timing](#54-timestamps--timing)
   - [Ranging & Diagnostics](#55-ranging--diagnostics)
   - [Interrupt Management & Callbacks](#56-interrupt-management--callbacks)
   - [Sleep & Power Management](#57-sleep--power-management)
   - [Temperature & Voltage](#58-temperature--voltage)
   - [GPIO & Antenna](#59-gpio--antenna)
   - [OTP Operations](#510-otp-operations)
   - [Calibration](#511-calibration)
   - [AES / Encryption](#512-aes--encryption)
   - [Continuous / CW Mode](#513-continuous--cw-mode)
   - [Sniff Mode](#514-sniff-mode)
   - [Double RX Buffer](#515-double-rx-buffer)
   - [IEEE 802.15.4 Addressing & Frame Filtering](#516-ieee-802154-addressing--frame-filtering)
   - [SPI CRC](#517-spi-crc)
   - [System Status Registers](#518-system-status-registers)
   - [Timers (DW3720)](#519-timers-dw3720)
   - [Dual SPI / Semaphore (DW37xx)](#520-dual-spi--semaphore-dw37xx)
   - [WiFi Coexistence](#521-wifi-coexistence)
   - [NLOS Diagnostics](#522-nlos-diagnostics)
   - [Utility / Miscellaneous](#523-utility--miscellaneous)
6. [HAL Interface](#6-hal-interface)
7. [Register Map Summary](#7-register-map-summary)

---

## 1. Overview

The DW3XXX driver provides a C API for the Decawave/Qorvo DW3000 family of Ultra-Wideband (UWB) transceivers. The driver abstracts register-level SPI access and exposes high-level functions for ranging, time-stamping, AES encryption, and MAC-layer operations.

### Supported Devices

| Device ID | Part Number | Notes |
|-----------|-------------|-------|
| `0xDECA0302` | DW3000 | Non-PDOA |
| `0xDECA0304` | QM33110 | Non-PDOA |
| `0xDECA0312` | DW3000 PDOA | With PDOA |
| `0xDECA0313` | DW3700 PDOA | Dual-SPI, semaphore |
| `0xDECA0314` | QM33120 / DW3720 | Timers, ARIB pulse shape |

### File Layout

| File | Purpose |
|------|---------|
| `deca_device_api.h` | Main public API: enums, structs, ~130 function prototypes |
| `deca_interface.h` | HAL/driver interface: `dwchip_s`, `dwt_spi_s`, `dwt_ops_s`, ioctl enum |
| `deca_regs.h` | Register address/bit-field `#define` macros (no functions) |
| `deca_types.h` | Portable integer typedefs (`uint8_t` … `int64_t`) |
| `deca_version.h` | Version constants |

### Timing Constants

| Macro | Value | Description |
|-------|-------|-------------|
| `DWT_TIME_UNITS` | `1/(499.2e6 × 128)` ≈ 15.65 ps | One DW timestamp unit |
| `DW3000_CHIP_FREQ` | 499 200 000 Hz | RF chip clock |
| `DW3000_DTU_FREQ` | 249 600 000 Hz | Device Time Unit frequency |
| `DW3000_CHIP_PER_DTU` | 2 | Chips per DTU |
| `DW3000_CHIP_PER_DLY` | 512 | Chips per antenna-delay unit |
| `DTU_TO_US(x)` | — | Convert DTU → microseconds |
| `US_TO_DTU(x)` | — | Convert microseconds → DTU |
| `FREQ_OFFSET_MULTIPLIER` | `998.4e6/2/1024/131072` | Carrier integrator → Hz |
| `HERTZ_TO_PPM_MULTIPLIER_CHAN_5` | `-1e6/6489.6e6` | Hz → PPM for CH5 |
| `HERTZ_TO_PPM_MULTIPLIER_CHAN_9` | `-1e6/7987.2e6` | Hz → PPM for CH9 |
| `CLOCK_OFFSET_PPM_TO_RATIO` | `1/(1<<26)` | PPM → ratio |
| `RX_BUFFER_MAX_LEN` | 1023 | Maximum RX frame bytes |
| `FCS_LEN` | 2 | Frame check sequence bytes |
| `DWT_SFDTOC_DEF` | 129 | Default SFD timeout (symbols) |
| `DEFAULT_XTAL_TRIM` | `0x2E` | Default crystal trim (2 pF load) |
| `XTAL_TRIM_BIT_MASK` | `0x3F` | Max XTAL trim value mask |

---

## 2. Types & Constants

### `deca_types.h`

Defines portable integer types via `<stdint.h>` (and fallback typedefs for STM32F429xx):

| Type | Description |
|------|-------------|
| `uint8_t` | Unsigned 8-bit |
| `uint16_t` | Unsigned 16-bit |
| `uint32_t` | Unsigned 32-bit |
| `uint64_t` | Unsigned 64-bit |
| `int8_t` | Signed 8-bit |
| `int16_t` | Signed 16-bit |
| `int32_t` | Signed 32-bit |
| `int64_t` | Signed 64-bit |

### `deca_version.h`

| Macro | Value | Description |
|-------|-------|-------------|
| `DRIVER_NAME` | `"DW3XXX"` | Driver family name |
| `DRIVER_VERSION_HEX` | `0x060014` | Version as hex (AA.BB.CC) |
| `DRIVER_VERSION_STR` | `"DW3XXX Device Driver Version 06.00.14"` | Version string |

Version format: `0xAABBCC` where CC = minor fix, BB = may need code changes, AA = major.

---

## 3. Key Data Structures

### `dwt_config_t` — Channel / PHY Configuration

```c
typedef struct {
    uint8_t chan;                  // Channel number: 5 or 9
    dwt_tx_plen_e txPreambLength;  // TX preamble length (DWT_PLEN_64 .. DWT_PLEN_4096)
    dwt_pac_size_e rxPAC;          // RX PAC size
    uint8_t txCode;                // TX preamble code (code selects PRF, e.g. 9 → 64 MHz PRF)
    uint8_t rxCode;                // RX preamble code
    dwt_sfd_type_e sfdType;        // SFD type (IEEE 4a/DW-8/DW-16/IEEE 4z)
    dwt_uwb_bit_rate_e dataRate;   // Data rate (850K or 6M8)
    dwt_phr_mode_e phrMode;        // PHR mode (standard / extended frames)
    dwt_phr_rate_e phrRate;        // PHR rate (standard / data rate)
    uint16_t sfdTO;                // SFD timeout in symbols (default 129)
    dwt_sts_mode_e stsMode;        // STS mode (off / mode 1 / mode 2 / no-data)
    dwt_sts_lengths_e stsLength;   // STS length
    dwt_pdoa_mode_e pdoaMode;      // PDOA mode (M0 / M1 / M3)
} dwt_config_t;
```

Pass to `dwt_configure()`.

---

### `dwt_txconfig_t` — TX Spectrum Configuration

```c
typedef struct {
    uint8_t  PGdly;    // Pulse generator delay (bandwidth adjustment)
    uint32_t power;    // TX power: [31:24] CP, [23:16] SHR, [15:8] PHR, [7:0] DATA
    uint16_t PGcount;  // PG count reference for bandwidth calibration
} dwt_txconfig_t;
```

Pass to `dwt_configuretxrf()`.

---

### `dwt_rxdiag_t` — RX Diagnostics

Packed structure (filled by `dwt_readdiagnostics()`). Key fields:

| Field | Type | Description |
|-------|------|-------------|
| `ipatovRxTime[5]` | `uint8_t[]` | RX timestamp from Ipatov sequence |
| `ipatovRxStatus` | `uint8_t` | Ipatov RX status |
| `ipatovPOA` | `uint16_t` | Phase of Arrival – Ipatov |
| `stsRxTime[5]` | `uint8_t[]` | RX timestamp from STS |
| `stsRxStatus` | `uint16_t` | STS RX status |
| `stsPOA` | `uint16_t` | POA of STS block 1 |
| `sts2RxTime[5]` | `uint8_t[]` | RX timestamp from STS block 2 |
| `sts2POA` | `uint16_t` | POA of STS block 2 |
| `tdoa[6]` | `uint8_t[]` | TDOA from two STS timestamps |
| `pdoa` | `int16_t` | PDOA signed [1:-11] radians |
| `xtalOffset` | `int16_t` | Estimated remote xtal offset |
| `ciaDiag1` | `uint32_t` | CIA diagnostics common to both sequences |
| `ipatovPeak` | `uint32_t` | Peak sample index/amplitude – Ipatov CIR |
| `ipatovPower` | `uint32_t` | Channel power – Ipatov |
| `ipatovF1/F2/F3` | `uint32_t` | First path amplitudes – Ipatov |
| `ipatovFpIndex` | `uint16_t` | First path index – Ipatov |
| `stsPeak/stsPower/stsF1..F3` | `uint32_t` | CIR diagnostics for STS |
| `sts2Peak/…` | `uint32_t` | CIR diagnostics for STS block 2 |

---

### `dwt_deviceentcnts_t` — Event Counters

```c
typedef struct {
    uint16_t PHE;   // RX header errors
    uint16_t RSL;   // Frame sync loss events
    uint16_t CRCG;  // Good CRC frames
    uint16_t CRCB;  // Bad CRC frames
    uint8_t  ARFE;  // Address filter errors
    uint8_t  OVER;  // RX buffer overruns (double-buffer mode)
    uint16_t SFDTO; // SFD timeout events
    uint16_t PTO;   // Preamble timeout events
    uint8_t  RTO;   // RX frame-wait timeout events
    uint16_t TXF;   // Transmitted frames
    uint8_t  HPW;   // Half period warnings
    uint8_t  CRCE;  // SPI CRC errors
    uint16_t PREJ;  // Preamble rejection events
    uint16_t SFDD;  // SFD detection events (DW3720 only)
    uint8_t  STSE;  // STS error/warning events
} dwt_deviceentcnts_t;
```

---

### `dwt_cb_data_t` — Callback Data

```c
typedef struct {
    uint32_t status;      // SYS_STATUS_LO snapshot on ISR entry
    uint16_t status_hi;   // SYS_STATUS_HI snapshot
    uint16_t datalength;  // Received frame length
    uint8_t  rx_flags;    // RX flags (DWT_CB_DATA_RX_FLAG_*)
    uint8_t  dss_stat;    // Dual SPI status (DW37xx)
    struct dwchip_s *dw;  // Chip handle
} dwt_cb_data_t;
```

Passed to all TX/RX callbacks.

---

### `dwt_aes_config_t` — AES Configuration

```c
typedef struct {
    dwt_aes_otp_sel_key_block_e aes_otp_sel_key_block; // First or second 128-bit OTP key
    dwt_aes_key_otp_type_e aes_key_otp_type;           // Key from OTP or RAM
    dwt_aes_core_type_e aes_core_type;                 // GCM or CCM*
    dwt_mic_size_e mic;                                // MIC size (0, 4, 6, 8, 10, 12, 14, 16 bytes)
    dwt_aes_key_src_e key_src;                         // Key source: registers or RAM/OTP
    dwt_aes_key_load_e key_load;                       // Load key from RAM
    uint8_t key_addr;                                  // RAM address offset of key
    dwt_aes_key_size_e key_size;                       // Key size: 128/192/256 bit
    dwt_aes_mode_e mode;                               // Encrypt or decrypt
} dwt_aes_config_t;
```

---

### `dwt_aes_job_t` — AES Job Descriptor

```c
typedef struct {
    uint8_t *nonce;              // Pointer to nonce (unique per transaction)
    uint8_t *header;             // AAD header (not encrypted/decrypted)
    uint8_t *payload;            // Data to encrypt/decrypt
    uint8_t  header_len;
    uint16_t payload_len;
    dwt_aes_src_port_e src_port; // Source: scratch/RX buf 0/RX buf 1/TX buf
    dwt_aes_dst_port_e dst_port; // Dest: scratch/RX buf 0/RX buf 1/TX buf/STS key
    dwt_aes_mode_e mode;         // Encrypt or decrypt
    uint8_t  mic_size;           // Tag size in bytes
} dwt_aes_job_t;
```

---

### `dwt_aes_key_t` — 128–256-bit AES Key Storage

```c
typedef struct {
    uint32_t key0, key1, key2, key3;  // 128-bit key
    uint32_t key4, key5, key6, key7;  // Extended to 256-bit
} dwt_aes_key_t;
```

---

### `dwt_sts_cp_key_t` — 128-bit STS Key

```c
typedef struct { uint32_t key0, key1, key2, key3; } dwt_sts_cp_key_t;
```

---

### `dwt_sts_cp_iv_t` — 128-bit STS IV (Nonce)

```c
typedef struct { uint32_t iv0, iv1, iv2, iv3; } dwt_sts_cp_iv_t;
```

---

### `dwt_timer_cfg_t` — Timer Configuration (DW3720)

```c
typedef struct {
    dwt_timers_e      timer;           // TIMER0 or TIMER1
    dwt_timer_period_e timer_div;      // Clock divider (38.4 MHz / 1..128)
    dwt_timer_mode_e  timer_mode;      // Single-shot or repeat
    uint8_t           timer_gpio_stop; // 1 = halt GPIO on interrupt
    uint8_t           timer_coexout;   // Configure GPIO for WiFi co-ex output
} dwt_timer_cfg_t;
```

---

### `dwt_nlos_alldiag_t` — NLOS Diagnostic

```c
typedef struct {
    uint32_t accumCount;    // Accumulated preamble/STS symbols
    uint32_t F1, F2, F3;   // First path amplitude points
    uint32_t cir_power;     // CIR power
    uint8_t  D;             // DGC decision (0–7)
    dwt_diag_type_e diag_type; // IPATOV / STS1 / STS2
    uint8_t  result;        // LOS/NLOS result
} dwt_nlos_alldiag_t;
```

---

### `dwt_nlos_ipdiag_t` — NLOS Ipatov Index Diagnostic

```c
typedef struct {
    uint32_t index_fp_u32;  // First Path Index
    uint32_t index_pp_u32;  // Peak Path Index
} dwt_nlos_ipdiag_t;
```

---

## 4. Enumerations

### Error Codes — `dwt_error_e`

| Value | Constant | Description |
|-------|----------|-------------|
| 0 | `DWT_SUCCESS` | Operation succeeded |
| -1 | `DWT_ERROR` | Generic error |
| -2 | `DWT_ERR_PLL_LOCK` | PLL failed to lock |
| -3 | `DWT_ERR_RX_CAL_PGF` | PGF calibration error |
| -4 | `DWT_ERR_RX_CAL_RESI` | RX cal RESI error |
| -5 | `DWT_ERR_RX_CAL_RESQ` | RX cal RESQ error |
| -6 | `DWT_ERR_RX_ADC_CAL` | ADC calibration error |

### Chip IDs — `dw_chip_id_e`

| Constant | Value | Part |
|----------|-------|------|
| `DWT_DW3000_DEV_ID` | `0xDECA0302` | DW3000 |
| `DWT_QM33110_DEV_ID` | `0xDECA0304` | QM33110 |
| `DWT_DW3000_PDOA_DEV_ID` | `0xDECA0312` | DW3000 PDOA |
| `DWT_DW3700_PDOA_DEV_ID` | `0xDECA0313` | DW3700 PDOA |
| `DWT_QM33120_PDOA_DEV_ID` | `0xDECA0314` | QM33120 / DW3720 |

### Bit Rate — `dwt_uwb_bit_rate_e`

| Constant | Description |
|----------|-------------|
| `DWT_BR_850K` | 850 kbits/s |
| `DWT_BR_6M8` | 6.8 Mbits/s |
| `DWT_BR_NODATA` | No data (SP3 packet) |

### PRF — `dwt_prf_e`

| Constant | Description |
|----------|-------------|
| `DWT_PRF_16M` | 16 MHz PRF |
| `DWT_PRF_64M` | 64 MHz PRF |
| `DWT_PRF_SCP` | SCP ~100 MHz PRF |

### PAC Size — `dwt_pac_size_e`

| Constant | Symbols | Recommended for preamble |
|----------|---------|--------------------------|
| `DWT_PAC4` | 4 | < 127 symbols |
| `DWT_PAC8` | 8 | ≤ 128 symbols |
| `DWT_PAC16` | 16 | 256 symbols |
| `DWT_PAC32` | 32 | 512 symbols |

### SFD Type — `dwt_sfd_type_e`

| Constant | Description |
|----------|-------------|
| `DWT_SFD_IEEE_4A` | IEEE 8-bit ternary (length 8) |
| `DWT_SFD_DW_8` | DW 8-bit (length 8) |
| `DWT_SFD_DW_16` | DW 16-bit (length 16) |
| `DWT_SFD_IEEE_4Z` | IEEE 8-bit binary / 4z BPRF (length 8) |

### TX Preamble Length — `dwt_tx_plen_e`

| Constant | Symbols |
|----------|---------|
| `DWT_PLEN_32` | 32 |
| `DWT_PLEN_64` | 64 (standard) |
| `DWT_PLEN_72` | 72 |
| `DWT_PLEN_128` | 128 |
| `DWT_PLEN_256` | 256 |
| `DWT_PLEN_512` | 512 |
| `DWT_PLEN_1024` | 1024 (standard) |
| `DWT_PLEN_1536` | 1536 |
| `DWT_PLEN_2048` | 2048 |
| `DWT_PLEN_4096` | 4096 (standard) |

### STS Mode — `dwt_sts_mode_e`

| Constant | Description |
|----------|-------------|
| `DWT_STS_MODE_OFF` | STS disabled |
| `DWT_STS_MODE_1` | STS mode 1 |
| `DWT_STS_MODE_2` | STS mode 2 |
| `DWT_STS_MODE_ND` | STS with no data |
| `DWT_STS_MODE_SDC` | Super Deterministic Codes |

### STS Length — `dwt_sts_lengths_e`

`DWT_STS_LEN_32` (0) through `DWT_STS_LEN_2048` (6). Use `GET_STS_REG_SET_VALUE(x)` to get the register value.

### PDOA Mode — `dwt_pdoa_mode_e`

| Constant | Description |
|----------|-------------|
| `DWT_PDOA_M0` | PDOA off |
| `DWT_PDOA_M1` | PDOA mode 1 |
| `DWT_PDOA_M3` | PDOA mode 3 |

### PHR Mode / Rate

| Constant | Description |
|----------|-------------|
| `DWT_PHRMODE_STD` | Standard PHR (up to 127 B) |
| `DWT_PHRMODE_EXT` | Extended PHR (up to 1023 B) |
| `DWT_PHRRATE_STD` | PHR at standard rate |
| `DWT_PHRRATE_DTA` | PHR at data rate (6M81) |

### TX Start Mode — `dwt_starttx_mode_e`

| Constant | Description |
|----------|-------------|
| `DWT_START_TX_IMMEDIATE` | Send immediately |
| `DWT_START_TX_DELAYED` | Send at DX_TIME |
| `DWT_RESPONSE_EXPECTED` | Enable RX after TX |
| `DWT_START_TX_DLY_REF` | Delayed relative to DREF_TIME |
| `DWT_START_TX_DLY_RS` | Delayed relative to RX_TIME_0 |
| `DWT_START_TX_DLY_TS` | Delayed relative to TX_TIME_LO |
| `DWT_START_TX_CCA` | CCA: send if no preamble in PTO |

### RX Enable Mode — `dwt_startrx_mode_e`

| Constant | Description |
|----------|-------------|
| `DWT_START_RX_IMMEDIATE` | Enable RX now |
| `DWT_START_RX_DELAYED` | Delayed RX (auto-fallback on late error) |
| `DWT_IDLE_ON_DLY_ERR` | Stay IDLE if delayed RX fails |
| `DWT_START_RX_DLY_REF` | Relative to DREF_TIME |
| `DWT_START_RX_DLY_RS` | Relative to RX_TIME_0 |
| `DWT_START_RX_DLY_TS` | Relative to TX_TIME_LO |

### Interrupt Masks — `dwt_int_conf_e` (selected)

| Constant | Description |
|----------|-------------|
| `DWT_INT_TXFRS_BIT_MASK` | Frame sent |
| `DWT_INT_RXFCG_BIT_MASK` | RX frame CRC good |
| `DWT_INT_RXFCE_BIT_MASK` | RX frame CRC error |
| `DWT_INT_RXFTO_BIT_MASK` | RX frame wait timeout |
| `DWT_INT_RXPTO_BIT_MASK` | Preamble timeout |
| `DWT_INT_RXSTO_BIT_MASK` | SFD timeout |
| `DWT_INT_RXPHE_BIT_MASK` | PHY header error |
| `DWT_INT_CIADONE_BIT_MASK` | CIA done |
| `DWT_INT_SPIRDY_BIT_MASK` | SPI ready |
| `DWT_INT_RCINIT_BIT_MASK` | Device entered IDLE_RC |
| `DWT_INT_CPERR_BIT_MASK` | STS quality warning/error |
| `DWT_INT_TIMER0_BIT_MASK` | TIMER0 expiry (DW3720) |
| `DWT_INT_TIMER1_BIT_MASK` | TIMER1 expiry (DW3720) |
| `DWT_INT_RX` | All RX-related interrupts |
| `DWT_INT_ALL_LO` / `DWT_INT_ALL_HI` | All interrupts |
| `SYS_STATUS_ALL_RX_TO` | All RX timeout events |
| `SYS_STATUS_ALL_RX_ERR` | All RX error events |
| `SYS_STATUS_ALL_RX_GOOD` | All good RX events |

### Sleep / Wake Parameters

**On-wake (`dwt_on_wake_param_e`):** `DWT_PGFCAL`, `DWT_GOTORX`, `DWT_GOTOIDLE`, `DWT_SEL_OPS0..3`, `DWT_LOADLDO`, `DWT_LOADDGC`, `DWT_LOADBIAS`, `DWT_RUNSAR`, `DWT_CONFIG`

**Wake-up source (`dwt_wkup_param_e`):** `DWT_SLP_EN`, `DWT_SLEEP`, `DWT_BROUT_EN`, `DWT_WAKE_CSN`, `DWT_WAKE_WUP`, `DWT_PRES_SLEEP`

**Auto-sleep trigger (`dwt_sleep_after_param_e`):** `DWT_TX_COMPLETE`, `DWT_RX_COMPLETE`

### Frame Filter Options — `dwt_ff_conf_options_e`

`DWT_FF_ENABLE_802_15_4`, `DWT_FF_DISABLE`, `DWT_FF_BEACON_EN`, `DWT_FF_DATA_EN`, `DWT_FF_ACK_EN`, `DWT_FF_MAC_EN`, `DWT_FF_RSVD_EN`, `DWT_FF_MULTI_EN`, `DWT_FF_FRAG_EN`, `DWT_FF_EXTEND_EN`, `DWT_FF_COORD_EN`, `DWT_FF_IMPBRCAST_EN`, `DWT_FF_MAC_LE0..3_EN`

### Reset Options — `dwt_reset_options_e`

| Constant | Value | Description |
|----------|-------|-------------|
| `DWT_RESET_ALL` | `0x00` | Full reset |
| `DWT_RESET_CTRX` | `0x0F` | Clear TX/RX |
| `DWT_RESET_RX` | `0xEF` | Reset RX only |
| `DWT_RESET_CLEAR` | `0xFF` | Clear reset |

### Interrupt Options — `dwt_INT_options_e`

| Constant | Description |
|----------|-------------|
| `DWT_DISABLE_INT` | Disable specified interrupts |
| `DWT_ENABLE_INT` | Enable additional interrupts |
| `DWT_ENABLE_INT_ONLY` | Enable only specified interrupts |
| `DWT_ENABLE_INT_DUAL_SPI` | Enable + dual SPI mode |
| `DWT_ENABLE_INT_ONLY_DUAL_SPI` | Enable only + dual SPI mode |

---

## 5. API Functions by Category

> **Convention:** All functions return `DWT_SUCCESS` (0) or `DWT_ERROR` (-1) unless another return type is shown. `void` functions have no return value.

---

### 5.1 Device Init & Management

| Signature | Description |
|-----------|-------------|
| `int dwt_probe(struct dwt_probe_s *probe_interf)` | Select the correct driver from the list based on device ID. Must be called first. |
| `int dwt_initialise(int mode)` | Initialise DW3xxx: read OTP (LDO, bias, xtal trim), configure device. SPI must be < 7 MHz. |
| `int32_t dwt_apiversion(void)` | Return driver API version number. |
| `char *dwt_version_string(void)` | Return driver version string. |
| `int dwt_setlocaldataptr(unsigned int index)` | Set local data pointer to array element `index` (multi-device support). |
| `struct dwchip_s* dwt_update_dw(struct dwchip_s *new_dw)` | Update the interrupt-handler's `dw` pointer; returns old pointer. |
| `void dwt_setdwstate(int state)` | Force device into IDLE/IDLE_PLL (`DWT_DW_IDLE`) or IDLE_RC (`DWT_DW_IDLE_RC`). |
| `uint8_t dwt_checkidlerc(void)` | Check if device is in IDLE_RC; returns 1 if yes. Wait after reset before increasing SPI rate. |
| `uint8_t dwt_checkirq(void)` | Returns 1 if IRQ line is active (IRQS bit set). |
| `void dwt_softreset(int reset_semaphore)` | Soft-reset the device. SPI must be ≤ 7 MHz. `reset_semaphore=1` also resets semaphore (DW3720). |
| `int dwt_check_dev_id(void)` | Read device ID and verify it matches a known DW3xxx ID. |
| `uint32_t dwt_readdevid(void)` | Read and return the silicon device ID. |
| `uint32_t dwt_getpartid(void)` | Return 32-bit part ID (read during `dwt_initialise`). |
| `uint32_t dwt_getlotid(void)` | Return 32-bit lot ID (read during `dwt_initialise`). |
| `uint8_t dwt_otprevision(void)` | Return OTP revision (read during `dwt_initialise`). |
| `void dwt_restoreconfig(void)` | Restore configuration after wake from deep sleep. |
| `void dwt_wakeup_ic(void)` | Wake device via IO pin (platform-dependent). |

---

### 5.2 Configuration

| Signature | Description |
|-----------|-------------|
| `int dwt_configure(dwt_config_t *config)` | Apply full PHY configuration (channel, preamble, PRF, data rate, STS, PDOA). Returns `DWT_ERROR` if PLL fails. |
| `void dwt_configuretxrf(dwt_txconfig_t *config)` | Configure TX spectrum: power levels and PG delay. |
| `void dwt_configurestsmode(uint8_t stsMode)` | Set STS mode independently (call after `dwt_configure`). |
| `void dwt_configurestskey(dwt_sts_cp_key_t *pStsKey)` | Set 128-bit AES key for STS generation. |
| `void dwt_configurestsiv(dwt_sts_cp_iv_t *pStsIv)` | Set 128-bit initial value (IV/nonce) for STS. |
| `void dwt_configurestsloadiv(void)` | Reload the STS IV from registers into hardware. |
| `void dwt_configmrxlut(int channel)` | Set MRX LUT defaults for given channel (5 or 9). |
| `void dwt_configuresfdtype(uint8_t sfdType)` | Configure SFD type only (call after `dwt_configure`). |
| `void dwt_setrxantennadelay(uint16_t antennaDly)` | Write RX antenna delay to CIA_CONF register. |
| `uint16_t dwt_getrxantennadelay(void)` | Read current RX antenna delay from CIA_CONF. |
| `void dwt_settxantennadelay(uint16_t antennaDly)` | Write TX antenna delay to TX_ANTD register. |
| `uint16_t dwt_gettxantennadelay(void)` | Read current TX antenna delay from TX_ANTD. |
| `void dwt_setfinegraintxseq(int enable)` | Enable (1) / disable (0) fine-grain TX sequencing. Disable before enabling PA. |
| `void dwt_setlnapamode(int lna_pa)` | Enable GPIO for external LNA (bit 0) / PA (bit 1) control. |
| `void dwt_setxtaltrim(uint8_t value)` | Set crystal trim (0x00–0x3F, ~1.65 ppm/step). |
| `uint8_t dwt_getxtaltrim(void)` | Return initial XTAL trim applied at init. |
| `void dwt_setplenfine(uint8_t preambleLength)` | Set fine preamble length in steps of 8 (16–2048); 0 = use TXPSR_PE. |
| `void dwt_configure_rf_port(dwt_rf_port_selection_e rfPort, dwt_rf_port_ctrl_e enable)` | Select RF port (1 or 2) and enable/disable manual antenna control. |
| `void dwt_configure_and_set_antenna_selection_gpio(uint8_t antenna_config)` | Drive GPIO6/7 for antenna selection. |
| `void dwt_set_fixedsts(uint8_t set)` | Enable Fixed STS mode – same STS in every packet (DW3720). |
| `void dwt_set_alternative_pulse_shape(uint8_t set_alternate)` | Enable ARIB alternative pulse shape (DW3720). |

---

### 5.3 TX / RX Frame Operations

| Signature | Description |
|-----------|-------------|
| `int dwt_writetxdata(uint16_t txDataLength, uint8_t *txDataBytes, uint16_t txBufferOffset)` | Write up to 1023 bytes of TX data into the IC's TX buffer. |
| `void dwt_writetxfctrl(uint16_t txFrameLength, uint16_t txBufferOffset, uint8_t ranging)` | Set TX frame control register (length, buffer offset, ranging flag). |
| `int dwt_starttx(uint8_t mode)` | Start transmission. `mode` is a `dwt_starttx_mode_e` bitmask. |
| `int dwt_rxenable(int mode)` | Enable receiver. `mode` is a `dwt_startrx_mode_e` bitmask. |
| `void dwt_forcetrxoff(void)` | Immediately turn off TX and RX. |
| `void dwt_readrxdata(uint8_t *buffer, uint16_t length, uint16_t rxBufferOffset)` | Read received data from RX buffer. |
| `void dwt_write_rx_scratch_data(uint8_t *buffer, uint16_t length, uint16_t bufferOffset)` | Write data to RX scratch buffer. |
| `void dwt_read_rx_scratch_data(uint8_t *buffer, uint16_t length, uint16_t bufferOffset)` | Read data from RX scratch buffer. |
| `void dwt_readaccdata(uint8_t *buffer, uint16_t len, uint16_t accOffset)` | Read CIR accumulator data (18-bit complex samples, 6 bytes each). First byte returned is a dummy. |
| `uint16_t dwt_getframelength(void)` | Return byte count of last received frame. |
| `void dwt_setrxaftertxdelay(uint32_t rxDelayTime)` | Set RX-after-TX turn-on delay (20-bit, UWB µs units). |
| `void dwt_setdelayedtrxtime(uint32_t starttime)` | Set delayed TX/RX start time (high 32 bits of system time). |
| `void dwt_setreferencetrxtime(uint32_t reftime)` | Set reference time for `DWT_START_TX_DLY_REF` / `DWT_START_RX_DLY_REF` (8 ns resolution). |
| `void dwt_enableautoack(uint8_t responseDelayTime, int enable)` | Enable auto-ACK with optional delay in symbols (requires frame filtering). |
| `void dwt_configureframefilter(uint16_t enabletype, uint16_t filtermode)` | Configure IEEE 802.15.4 frame filtering. |
| `void dwt_configciadiag(uint8_t enable_mask)` | Enable CIA diagnostic logging (`DW_CIA_DIAG_LOG_*`). |

---

### 5.4 Timestamps & Timing

| Signature | Description |
|-----------|-------------|
| `void dwt_readtxtimestamp(uint8_t *timestamp)` | Read 40-bit TX timestamp (adjusted, antenna delay included) into 5-byte buffer. |
| `uint32_t dwt_readtxtimestamphi32(void)` | Read high 32 bits of TX timestamp. |
| `uint32_t dwt_readtxtimestamplo32(void)` | Read low 32 bits of TX timestamp. |
| `void dwt_readrxtimestamp(uint8_t *timestamp)` | Read 40-bit RX timestamp (adjusted) into 5-byte buffer. |
| `void dwt_readrxtimestampunadj(uint8_t *timestamp)` | Read unadjusted RX timestamp into 5-byte buffer. |
| `void dwt_readrxtimestamp_ipatov(uint8_t *timestamp)` | Read RX timestamp w.r.t. Ipatov CIR (adjusted). |
| `void dwt_readrxtimestamp_sts(uint8_t *timestamp)` | Read RX timestamp w.r.t. STS CIR (adjusted). |
| `uint32_t dwt_readrxtimestamphi32(void)` | Read high 32 bits of RX timestamp. (Not for double-buffer mode.) |
| `uint32_t dwt_readrxtimestamplo32(void)` | Read low 32 bits of RX timestamp. (Not for double-buffer mode.) |
| `uint32_t dwt_readsystimestamphi32(void)` | Read high 32 bits of system time counter. |
| `void dwt_readsystime(uint8_t *timestamp)` | Read 32-bit system time into 4-byte buffer. |
| `void dwt_reset_system_counter(void)` | Momentarily reset system time counter to 0. |

---

### 5.5 Ranging & Diagnostics

| Signature | Description |
|-----------|-------------|
| `int16_t dwt_readpdoa(void)` | Read PDOA result (signed [1:-11] radians). Convert to degrees: `pdoa/2048 * 180/π`. |
| `void dwt_readtdoa(uint8_t *tdoa)` | Read 41-bit TDOA value into 6-byte buffer. Valid only if `abs(TDOA) ≤ DWT_VALID_TDOA_LIMIT` (100). |
| `int16_t dwt_readclockoffset(void)` | Read clock offset (s[6:-4] format; divide by 16 for ppm). Positive = local RX faster than remote TX. |
| `int32_t dwt_readcarrierintegrator(void)` | Read carrier integrator value. Multiply by `FREQ_OFFSET_MULTIPLIER` for Hz. |
| `void dwt_readdiagnostics(dwt_rxdiag_t *diagnostics)` | Read full RX signal quality diagnostic data. |
| `int dwt_readstsquality(int16_t *rxStsQualityIndex)` | Read STS quality index. ≥ 0 is good; for 64 MHz PRF, ≥ 90% of STS length is good. |
| `int dwt_readstsstatus(uint16_t *stsStatus, int sts_num)` | Read STS status for STS 0 or 1. Returns 0 for good/valid. |
| `void dwt_configeventcounters(int enable)` | Enable (1, also resets) / disable (0) event counters. |
| `void dwt_readeventcounters(dwt_deviceentcnts_t *counters)` | Read all event counters into structure. |
| `uint8_t dwt_get_dgcdecision(void)` | Read DGC_DECISION index for RX/FP level estimation. |
| `uint32_t dwt_readctrdbg(void)` | Read CTR_DBG register (low 32 bits of STS IV counter). |
| `uint32_t dwt_readdgcdbg(void)` | Read DGC_DBG register. |
| `uint32_t dwt_readCIAversion(void)` | Read CIA hardware version. |
| `uint32_t dwt_getcirregaddress(void)` | Return base address of ACC_MEM (CIR accumulator). |
| `register_name_add_t* dwt_get_reg_names(void)` | Return pointer to register name/address array (for debug logging). |
| `void dwt_setpdoaoffset(uint16_t offset)` | Write PDOA offset to CIA_ADJUST register. |
| `uint32_t dwt_readpdoaoffset(void)` | Read PDOA offset from CIA_ADJUST register. |
| `uint8_t dwt_nlos_alldiag(dwt_nlos_alldiag_t *all_diag)` | Read IPATOV/STS1/STS2 diagnostic registers for LOS/NLOS estimation. CIA diag must be `DW_CIA_DIAG_LOG_ALL`. |
| `void dwt_nlos_ipdiag(dwt_nlos_ipdiag_t *index)` | Read Ipatov first-path and peak-path indices for low-signal NLOS estimation. |

---

### 5.6 Interrupt Management & Callbacks

| Signature | Description |
|-----------|-------------|
| `void dwt_setinterrupt(uint32_t bitmask_lo, uint32_t bitmask_hi, dwt_INT_options_e INT_options)` | Enable/disable/replace interrupt mask in SYS_ENABLE_LO and SYS_ENABLE_HI. |
| `void dwt_setcallbacks(dwt_cb_t cbTxDone, dwt_cb_t cbRxOk, dwt_cb_t cbRxTo, dwt_cb_t cbRxErr, dwt_cb_t cbSPIErr, dwt_cb_t cbSPIRdy, dwt_cb_t cbDualSPIEv)` | Register event callback functions. NULL = ignore event. |
| `void dwt_isr(void)` | ISR: processes RXFCG, TXFRS, timeouts, errors; fires registered callbacks. Supports double-buffer. |
| `decaIrqStatus_t decamutexon(void)` | Disable IRQ and return saved state (platform-specific). |
| `void decamutexoff(decaIrqStatus_t s)` | Re-enable IRQ from saved state (platform-specific). |

**Callback types:**
```c
typedef void (*dwt_cb_t)(const dwt_cb_data_t *);    // TX/RX/timeout/error callbacks
typedef void (*dwt_spierrcb_t)(void);                // SPI CRC error callback
```

---

### 5.7 Sleep & Power Management

| Signature | Description |
|-----------|-------------|
| `void dwt_configuresleep(uint16_t mode, uint8_t wake)` | Configure on-wake actions (`mode`) and wake-up sources (`wake`). |
| `void dwt_entersleep(int idle_rc)` | Enter sleep/deep sleep. If `idle_rc = DWT_DW_IDLE_RC`, auto INIT2IDLE is cleared. |
| `void dwt_entersleepafter(int event_mask)` | Auto-enter sleep after TX (`DWT_TX_COMPLETE`) and/or RX (`DWT_RX_COMPLETE`). |
| `void dwt_entersleepaftertx(int enable)` | Deprecated: auto-sleep after TX only. Use `dwt_entersleepafter` instead. |
| `void dwt_clearaonconfig(void)` | Clear AON configuration block. |
| `uint16_t dwt_calibratesleepcnt(void)` | Calibrate LP oscillator. Returns XTAL cycles per LP OSC cycle. Call before `dwt_configuresleepcnt`. |
| `void dwt_configuresleepcnt(uint16_t sleepcnt)` | Program high 16 bits of 28-bit sleep counter. SPI must be < 3 MHz. |
| `uint8_t dwt_aon_read(uint16_t aon_address)` | Read 1 byte from AON memory at given address. |
| `void dwt_aon_write(uint16_t aon_address, uint8_t aon_write_data)` | Write 1 byte to AON memory. |

---

### 5.8 Temperature & Voltage

| Signature | Description |
|-----------|-------------|
| `uint16_t dwt_readtempvbat(void)` | Read raw temperature (bits [15:8]) and Vbat (bits [7:0]) from ADC. |
| `float dwt_convertrawtemperature(uint8_t raw_temp)` | Convert raw temperature to °C. Requires `dwt_initialise()` first. |
| `float dwt_convertrawvoltage(uint8_t raw_voltage)` | Convert raw voltage to V. Requires `dwt_initialise()` first. |
| `uint8_t dwt_readwakeuptemp(void)` | Return raw temperature sampled at last wake-from-sleep. |
| `uint8_t dwt_readwakeupvbat(void)` | Return raw Vbat sampled at last wake-from-sleep. |
| `uint8_t dwt_geticrefvolt(void)` | Return factory reference voltage (from OTP, 3.0 V measurement). |
| `uint8_t dwt_geticreftemp(void)` | Return factory reference temperature (from OTP, 22 °C measurement). |

---

### 5.9 GPIO & Antenna

| Signature | Description |
|-----------|-------------|
| `void dwt_setgpiomode(uint32_t gpio_mask, uint32_t gpio_modes)` | Configure GPIO function using `dwt_gpio_mask_e` mask and `dwt_gpio_pin_e` modes. |
| `void dwt_setgpiodir(uint16_t in_out)` | Set GPIO direction: bit=1 → input, bit=0 → output. |
| `void dwt_setgpiovalue(uint16_t gpio, int value)` | Drive output GPIO(s) high (1) or low (0). |
| `uint16_t dwt_readgpiovalue(void)` | Read raw GPIO pin values. |
| `void dwt_enablegpioclocks(void)` | Enable GPIO clocks required for correct GPIO operation. |
| `void dwt_setleds(uint8_t mode)` | Configure LED GPIOs: bit 0 = enable LEDs, bit 1 = blink on init. |
| `void dwt_configure_and_set_antenna_selection_gpio(uint8_t antenna_config)` | Drive GPIO6/7 for antenna switching (see `ANT_GPIO*` bitfield macros). |

**GPIO Pin Modes (selected from `dwt_gpio_pin_e`):**

| Constant | Device | Description |
|----------|--------|-------------|
| `DW3000_GPIO_PIN2_RXLED` | DW3000 | RX LED |
| `DW3000_GPIO_PIN3_TXLED` | DW3000 | TX LED |
| `DW3000_GPIO_PIN5_EXTTXE` | DW3000 | External TX enable |
| `DW3000_GPIO_PIN6_EXTRXE` | DW3000 | External RX enable |
| `DW37XX_GPIO_PIN4_COEX_IN` | DW37xx | WiFi coex input |
| `DW37XX_GPIO_PIN5_COEX_OUT` | DW37xx | WiFi coex output |
| `DW3000_GPIO_PIN0_PDOA_SW_TX` | DW3000 | PDOA antenna switch TX |

---

### 5.10 OTP Operations

| Signature | Description |
|-----------|-------------|
| `void dwt_otpread(uint16_t address, uint32_t *array, uint8_t length)` | Read `length` 32-bit words from OTP starting at `address`. |
| `int dwt_otpwriteandverify(uint32_t value, uint16_t address)` | Write and verify a 32-bit word to OTP. |
| `int dwt_otpwrite(uint32_t value, uint16_t address)` | Write a 32-bit word to OTP without verification (DW3700). |

**OTP read mode flags (`dwt_read_otp_modes_e`):**

| Constant | Description |
|----------|-------------|
| `DWT_READ_OTP_PID` | Read part ID |
| `DWT_READ_OTP_LID` | Read lot ID |
| `DWT_READ_OTP_BAT` | Read reference voltage |
| `DWT_READ_OTP_TMP` | Read reference temperature |

---

### 5.11 Calibration

| Signature | Description |
|-----------|-------------|
| `int dwt_run_pgfcal(void)` | Run PGF (pre-gain filter) calibration required before RX. Returns `DWT_ERROR` on failure. |
| `int dwt_pgf_cal(int ldoen)` | Run PGF calibration; `ldoen=1` enables/disables LDOs around the call. |
| `int dwt_pll_cal(void)` | Re-calibrate and re-lock PLL. |
| `uint8_t dwt_calcbandwidthadj(uint16_t target_count)` | Find PG_DELAY setting that achieves the target PG count (bandwidth calibration). |
| `uint16_t dwt_calcpgcount(uint8_t pgdly)` | Calculate PGC_STATUS count for a given PG_DELAY value (reference for temperature compensation). |
| `uint8_t dwt_readpgdelay(void)` | Return current PG delay value. |

---

### 5.12 AES / Encryption

| Signature | Description |
|-----------|-------------|
| `void dwt_set_keyreg_128(const dwt_aes_key_t *key)` | Load 128-bit AES key into key registers. |
| `void dwt_configure_aes(const dwt_aes_config_t *pCfg)` | Configure AES block (mode, key source, MIC size, core type). |
| `int8_t dwt_do_aes(dwt_aes_job_t *job, dwt_aes_core_type_e core_type)` | Execute AES encrypt/decrypt job. Returns AES_STS_ID status. |

**AES Error codes (return from `dwt_do_aes`):**

| Macro | Value | Description |
|-------|-------|-------------|
| `ERROR_DATA_SIZE` | -1 | Data too large |
| `ERROR_WRONG_MODE` | -2 | Invalid mode |
| `ERROR_WRONG_MIC_SIZE` | -3 | Invalid MIC size |
| `ERROR_PAYLOAD_SIZE` | -4 | Payload too large |
| `MIC_ERROR` | 0xFF | MIC authentication failed |

---

### 5.13 Continuous / CW Mode

| Signature | Description |
|-----------|-------------|
| `void dwt_configcwmode(void)` | Set device to continuous wave (CW) mode at channel frequency. |
| `void dwt_repeated_cw(int cw_enable, int cw_mode_config)` | Enable/disable repeated CW transmission. |
| `void dwt_configcontinuousframemode(uint32_t framerepetitionrate)` | Enable continuous TX frame mode (regulatory testing). Rate in ~4 ns units, minimum 2. |
| `void dwt_disablecontinuousframemode(void)` | Stop continuous TX frame mode. |
| `void dwt_repeated_frames(uint32_t framerepetitionrate)` | Enable repeated frame transmission at given rate. |
| `void dwt_stop_repeated_frames(void)` | Stop repeated frame transmission. |

---

### 5.14 Sniff Mode

| Signature | Description |
|-----------|-------------|
| `void dwt_setsniffmode(int enable, uint8_t timeOn, uint8_t timeOff)` | Enable/disable SNIFF low-power RX mode. `timeOn`: 1–15 (× PAC size + 1 PAC). `timeOff`: 0–255 (× ~1 µs). |

---

### 5.15 Double RX Buffer

| Signature | Description |
|-----------|-------------|
| `void dwt_setdblrxbuffmode(dwt_dbl_buff_state_e dbl_buff_state, dwt_dbl_buff_mode_e dbl_buff_mode)` | Enable/disable double RX buffer and set auto or manual mode. |
| `void dwt_signal_rx_buff_free(void)` | Signal to device that current RX buffer has been processed and is free. |
| `void dwt_writerdbstatus(uint8_t mask)` | Write to RDB_STATUS register to clear double-buffer events. |
| `uint8_t dwt_readrdbstatus(void)` | Read RDB_STATUS register. |
| `void dwt_setinterrupt_db(uint8_t bitmask, dwt_INT_options_e INT_options)` | Enable/disable double-buffer interrupt events (DW37xx). |

**Double-buffer status bits (`dwt_rdb_e`):**

`DWT_RDB_STATUS_RXFCG0/1_BIT_MASK`, `DWT_RDB_STATUS_RXFR0/1_BIT_MASK`, `DWT_RDB_STATUS_CIADONE0/1_BIT_MASK`, `DWT_RDB_STATUS_CP_ERR0/1_BIT_MASK`

---

### 5.16 IEEE 802.15.4 Addressing & Frame Filtering

| Signature | Description |
|-----------|-------------|
| `void dwt_setpanid(uint16_t panID)` | Set PAN ID. |
| `void dwt_setaddress16(uint16_t shortAddress)` | Set 16-bit short address. |
| `void dwt_seteui(uint8_t *eui64)` | Set 64-bit EUI (8-byte buffer). |
| `void dwt_geteui(uint8_t *eui64)` | Read 64-bit EUI into 8-byte buffer. |
| `void dwt_configure_le_address(uint16_t addr, int leIndex)` | Write 16-bit address to Low-Energy (LE) register 0–3 for frame-pending. |
| `void dwt_configureframefilter(uint16_t enabletype, uint16_t filtermode)` | Configure 802.15.4 frame filtering (enable type + allowed frame types). |

---

### 5.17 SPI CRC

| Signature | Description |
|-----------|-------------|
| `void dwt_enablespicrccheck(dwt_spi_crc_mode_e crc_mode, dwt_spierrcb_t spireaderr_cb)` | Enable SPI CRC checking (`DWT_SPI_CRC_MODE_WR` or `_WRRD`). Provide callback for read-CRC mismatch. |
| `uint8_t dwt_generatecrc8(const uint8_t *byteArray, int flen, uint8_t crcInit)` | Calculate CRC-8 (polynomial `x⁸+x²+x+1`) for SPI write transactions. |

---

### 5.18 System Status Registers

| Signature | Description |
|-----------|-------------|
| `void dwt_writesysstatuslo(uint32_t mask)` | Write (clear) bits in SYS_STATUS lower 32-bit register. |
| `void dwt_writesysstatushi(uint32_t mask)` | Write (clear) bits in SYS_STATUS higher register. |
| `uint32_t dwt_readsysstatuslo(void)` | Read lower 32 bits of SYS_STATUS. |
| `uint32_t dwt_readsysstatushi(void)` | Read higher bits of SYS_STATUS (16-bit on DW3000/3700, 32-bit on DW3720). |

---

### 5.19 Timers (DW3720)

| Signature | Description |
|-----------|-------------|
| `void dwt_configure_timer(dwt_timer_cfg_t *tim_cfg)` | Configure TIMER0 or TIMER1 with divider, mode, and GPIO options. |
| `void dwt_set_timer_expiration(dwt_timers_e timer_name, uint32_t exp)` | Set 22-bit expiration count. E.g., with XTAL/64 (1.66 µs units), `exp=1024` → ~1.7 ms. |
| `void dwt_timer_enable(dwt_timers_e timer_name)` | Enable a timer (0→1 transition of enable bit). |
| `void dwt_timers_reset(void)` | Reset both timers (stops repeat mode). |
| `uint16_t dwt_timers_read_and_clear_events(void)` | Read and clear timer event counts: bits [7:0] = TIMER0, [15:8] = TIMER1. |
| `void dwt_configure_wificoex_gpio(uint8_t timer_coexout, uint8_t coex_swap)` | Configure GPIO4/5 for COEX_OUT driven by timer. |

**Timer clock options (`dwt_timer_period_e`):**

| Constant | Frequency | Period |
|----------|-----------|--------|
| `DWT_XTAL` | 38.4 MHz | 26 ns |
| `DWT_XTAL_DIV2` | 19.2 MHz | 52 ns |
| `DWT_XTAL_DIV4` | 9.6 MHz | 104 ns |
| `DWT_XTAL_DIV8` | 4.8 MHz | 208 ns |
| `DWT_XTAL_DIV16` | 2.4 MHz | 417 ns |
| `DWT_XTAL_DIV32` | 1.2 MHz | 833 ns |
| `DWT_XTAL_DIV64` | 600 kHz | 1.67 µs |
| `DWT_XTAL_DIV128` | 300 kHz | 3.33 µs |

---

### 5.20 Dual SPI / Semaphore (DW37xx)

| Signature | Description |
|-----------|-------------|
| `void dwt_softreset_fcmd(void)` | Reset DW3700 including semaphore (fast command). |
| `void dwt_softreset_no_sema_fcmd(void)` | Reset DW3700 without resetting semaphore. |
| `void dwt_ds_sema_request(void)` | Request dual-SPI semaphore (non-blocking). |
| `void dwt_ds_sema_release(void)` | Release dual-SPI semaphore. |
| `void dwt_ds_sema_force(void)` | Force-take semaphore (SPI2 host only). |
| `uint8_t dwt_ds_sema_status(void)` | Return semaphore status low byte. |
| `uint8_t dwt_ds_sema_status_hi(void)` | Return semaphore status high byte. |
| `void dwt_ds_en_sleep(dwt_host_sleep_en_e host_sleep_en)` | Allow (`HOST_EN_SLEEP`) or prevent (`HOST_DIS_SLEEP`) the device from entering sleep. |
| `int dwt_ds_setinterrupt_SPIxavailable(dwt_spi_host_e spi_num, dwt_INT_options_e int_set)` | Enable/disable SPI1MAVAIL or SPI2MAVAIL interrupt. |
| `void dwt_enable_disable_eq(uint8_t en)` | Enable/disable CIA equaliser (use when peer uses SRRC pulse shape). Default: disabled. |

---

### 5.21 WiFi Coexistence

| Signature | Description |
|-----------|-------------|
| `void dwt_wifi_coex_set(dwt_wifi_coex_e enable, int coex_io_swap)` | Drive GPIO5 (or GPIO4 if `coex_io_swap=1`) to signal WiFi chip. |
| `void dwt_config_ostr_mode(uint8_t enable, uint16_t wait_time)` | Enable One-Shot Timebase Reset mode: system counter resets on SYNC pin. Recommended `wait_time=33`. |

---

### 5.22 NLOS Diagnostics

| Signature | Description |
|-----------|-------------|
| `uint8_t dwt_nlos_alldiag(dwt_nlos_alldiag_t *all_diag)` | Read IPATOV, STS1, or STS2 diagnostics for LOS/NLOS classification. Requires `DW_CIA_DIAG_LOG_ALL`. |
| `void dwt_nlos_ipdiag(dwt_nlos_ipdiag_t *index)` | Read first-path index and peak-path index from Ipatov (low-signal NLOS estimate). |
| `int dwt_adjust_tx_power(uint16_t boost, uint32_t ref_tx_power, uint8_t channel, uint32_t *adj_tx_power, uint16_t *applied_boost)` | Calculate adjusted TX power for short frames. `boost` in 0.1 dB units relative to 1 ms frame (0 dB baseline). |

---

### 5.23 Utility / Miscellaneous

| Signature | Description |
|-----------|-------------|
| `void dwt_setrxtimeout(uint32_t time)` | Set RX frame-wait timeout (units: 512/499.2 MHz ≈ 1.026 µs). 0 = disable. |
| `void dwt_setpreambledetecttimeout(uint16_t timeout)` | Set preamble detection timeout (multiples of PAC size; counter adds 1 PAC). |
| `void deca_sleep(unsigned int time_ms)` | Platform-specific millisecond delay. |
| `void deca_usleep(unsigned long time_us)` | Platform-specific microsecond delay. |
| `dwt_mic_size_e dwt_micsizefrombytes(uint8_t mic_size_in_bytes)` | Convert MIC byte count to `dwt_mic_size_e` enum. |

---

## 6. HAL Interface

Defined in `deca_interface.h`.

### `dwt_spi_s` — SPI Abstraction

The implementer must provide these function pointers for the target hardware:

| Function Pointer | Signature | Description |
|------------------|-----------|-------------|
| `readfromspi` | `int (*)(uint16_t headerLen, uint8_t *headerBuf, uint16_t readLen, uint8_t *readBuf)` | SPI read: write header then read data |
| `writetospi` | `int (*)(uint16_t headerLen, const uint8_t *headerBuf, uint16_t bodyLen, const uint8_t *bodyBuf)` | SPI write: header + data |
| `writetospiwithcrc` | `int (*)(uint16_t headerLen, const uint8_t *headerBuf, uint16_t bodyLen, const uint8_t *bodyBuf, uint8_t crc8)` | SPI write with CRC-8 appended |
| `setslowrate` | `void (*)(void)` | Switch SPI to slow rate (< 7 MHz, for init/reset) |
| `setfastrate` | `void (*)(void)` | Switch SPI to fast rate |

---

### `dwchip_s` — Chip Descriptor

Central handle passed through the HAL layer:

```c
struct dwchip_s {
    struct dwt_spi_s       *SPI;                  // SPI interface (must be first)
    void                 (*wakeup_device_with_io)(void); // IO wakeup function
    struct dwt_driver_s    *dwt_driver;            // Matched driver
    struct dwt_callbacks_s  callbacks;             // TX/RX/error callbacks
    struct dwt_mcps_config_s *config;              // MCPS configuration
    struct mcps802154_llhw  *llhw;                 // 802.15.4 LLHW
    struct mcps802154_ops   *mcps_ops;
    struct dw3000_calibration_data *calib_data;
    struct dwt_mcps_runtime_s *mcps_runtime;
    struct dwt_mcps_rx_s    *rx;
    int8_t coex_gpio_pin;                          // WiFi co-ex GPIO pin
    int8_t coex_gpio_active_state;
    void                   *priv;                  // Driver private data (must be last)
};
```

---

### `dwt_probe_s` — Probe Structure

```c
struct dwt_probe_s {
    void *dw;                            // Optional external dwchip_s (NULL = use internal)
    void *spi;                           // Pointer to dwt_spi_s
    void (*wakeup_device_with_io)(void); // Platform IO wakeup function
};
```

Pass to `dwt_probe()` to select the driver.

---

### `dwt_callbacks_s` — Event Callbacks

```c
struct dwt_callbacks_s {
    dwt_spierrcb_t cbSPIRDErr;  // SPI read error (CRC mismatch)
    dwt_cb_t cbTxDone;          // TX confirmation
    dwt_cb_t cbRxOk;            // RX good frame
    dwt_cb_t cbRxTo;            // RX timeout
    dwt_cb_t cbRxErr;           // RX error
    dwt_cb_t cbSPIErr;          // SPI error
    dwt_cb_t cbSPIRdy;          // SPI ready (after reset)
    dwt_cb_t cbDualSPIEv;       // Dual SPI event (SPI1/SPI2 available)
    int (*rx_frame)(struct dwchip_s *, struct dw_rx_s *);   // MCPS RX frame
    int (*rx_error)(struct dwchip_s *, int);                // MCPS RX error
};
```

---

### `dwt_ops_s` — Driver Operations Table

Low-level operations dispatched per chip variant:

| Function | Description |
|----------|-------------|
| `configure` | Apply `dwt_config_t` |
| `write_tx_data` | Write TX buffer |
| `write_tx_fctrl` | Set TX frame control |
| `read_rx_data` | Read RX buffer |
| `read_acc_data` | Read CIR accumulator |
| `read_rx_timestamp` | Read RX timestamp |
| `configure_tx_rf` | Apply `dwt_txconfig_t` |
| `set_interrupt` | Set interrupt mask |
| `rx_enable` | Enable receiver |
| `initialize` | Initialize device |
| `xfer` | Raw SPI transfer |
| `ioctl` | Generic ioctl dispatch |
| `isr` | Interrupt service routine |
| `dbg_fn` | Debug function |

---

### `dwt_mcps_ops_s` — MCPS Operations Table

Higher-level MAC operations used by the 802.15.4 MCPS layer:

| Function | Description |
|----------|-------------|
| `init` / `deinit` | MCPS layer init/deinit |
| `tx_frame` | Transmit frame with timing info |
| `rx_enable` / `rx_disable` | Enable/disable receiver with timing |
| `get_timestamp` | Get 64-bit timestamp |
| `get_rx_frame` | Read received frame |
| `set_hrp_uwb_params` | Set HRP UWB PHY parameters |
| `set_channel` | Set channel and preamble code |
| `set_hw_addr_filt` | Configure hardware address filter |
| `ioctl` | Generic ioctl |
| `isr` | ISR |

---

### `dwt_ioctl_e` — IOCTL Command Enum

The ioctl enum provides a unified way to call driver operations. Key commands:

| Command | Description |
|---------|-------------|
| `DWT_CONFIGURE` → `dwt_configure()` | Full PHY config |
| `DWT_RXENABLE` | Enable RX |
| `DWT_STARTTX` | Start TX |
| `DWT_READRXTIMESTAMP` | Read RX timestamp |
| `DWT_READTXTIMESTAMP` | Read TX timestamp |
| `DWT_READPDOA` / `DWT_READTDOA` | Read PDOA/TDOA |
| `DWT_READCLOCKOFFSET` | Read clock offset |
| `DWT_ENTERSLEEP` / `DWT_CONFIGURESLEEP` | Sleep management |
| `DWT_DOAES` | AES operation |
| `DWT_OTPREAD` / `DWT_OTPWRITE` | OTP access |
| `DWT_SETINTERUPTDB` | Double-buffer interrupt (DW37xx) |
| `DWT_TIMERSRST` / `DWT_CONFIGTIMER` / `DWT_TIMERENABLE` | Timers (DW3720) |
| `DWT_DSSEMAREQUEST` / `DWT_DSSEMARELEASE` | Semaphore (DW37xx) |

---

### STD Interface Functions

```c
int      interface_init(struct dwchip_s *p);
void     interface_deinit(struct dwchip_s *p);
int      interface_tx_frame(struct dwchip_s *dw, uint8_t *data, size_t len, struct dw_tx_frame_info_s *info);
int      interface_rx_enable(struct dwchip_s *dw, struct dw_rx_frame_info_s *info);
int      interface_rx_disable(struct dwchip_s *dw);
uint64_t interface_get_timestamp(struct dwchip_s *dw);
void     interface_read_rx_frame(struct dwchip_s *dw, uint8_t *ptr, size_t len);
```

---

## 7. Register Map Summary

Defined in `deca_regs.h` as `#define` macros (format: `<REG>_ID` for address, `<REG>_<FIELD>_BIT_MASK` for bit fields).

### Key Registers

| Register | Address | Description |
|----------|---------|-------------|
| `DEV_ID` | `0x000` | Device ID (RIDTAG[31:16], MODEL[15:8], VER[7:4], REV[3:0]) |
| `EUI_64_LO` | `0x004` | EUI-64 lower 32 bits |
| `EUI_64_HI` | `0x008` | EUI-64 upper 32 bits |
| `PANADR` | `0x00C` | PAN ID [31:16] + Short Address [15:0] |
| `SYS_CFG` | `0x010` | System config (PHR mode, SPI CRC, PDOA mode, auto-ACK, etc.) |
| `ADR_FILT_CFG` | `0x014` | Address filter config (LE pending bits, coordinator mode) |
| `SYS_TIME` | `0x01C` | Current system time counter (40-bit) |
| `TX_FCTRL` | `0x024` | TX frame control (length, buffer offset, ranging, PRF, preamble) |
| `DX_TIME` | `0x02C` | Delayed TX/RX time |
| `DREF_TIME` | `0x030` | Reference time for `DLY_REF` mode |
| `RX_FWTO` | `0x034` | RX frame wait timeout |
| `SYS_CTRL` | `0x038` | System control (TXSTRT, RXENAB, TRXOFF) |
| `SYS_ENABLE_LO` | `0x03C` | IRQ enable mask low |
| `SYS_ENABLE_HI` | `0x040` | IRQ enable mask high |
| `SYS_STATUS_LO` | `0x044` | System status low (TXFRS, RXFCG, RXFCE, timeouts, errors) |
| `SYS_STATUS_HI` | `0x048` | System status high |
| `RX_FINFO` | `0x04C` | RX frame info (length, PRF, preamble, data rate) |
| `RX_BUFFER0` | `0x120` | RX data buffer 0 |
| `RX_BUFFER1` | `0x170` | RX data buffer 1 (double-buffer mode) |
| `TX_BUFFER` | `0x1C0` | TX data buffer |
| `ACC_MEM` | — | CIR accumulator memory (base from `dwt_getcirregaddress()`) |
| `CIA_CONF` | — | CIA config (RX antenna delay, STS/Ipatov control) |
| `IP_TOA_LO/HI` | — | Ipatov time of arrival (40-bit timestamp) |
| `STS_TOA_LO/HI` | — | STS time of arrival block 1 |
| `STS1_TOA_LO/HI` | — | STS time of arrival block 2 |
| `CIA_TDOA_0` | — | TDOA (41-bit) |
| `CIA_TDOA_1_PDOA` | — | PDOA (16-bit signed) + TDOA high bits |
| `CIA_DIAG_0/1` | — | CIA diagnostic data |
| `TX_ANTD` | — | TX antenna delay |
| `CP_CFG0` | — | STS length config |
| `CP_KEY0..3` | — | STS CP key (128-bit) |
| `CP_IV0..3` | — | STS CP IV/nonce (128-bit) |
| `AES_CFG` | — | AES block configuration |
| `AES_IV0..3` | `0x10034` | AES IV entry |
| `AES_KEY0..7` | — | AES key registers |
| `SYS_STATE` | — | Current device state |
| `AON_CTRL` | — | AON control (sleep/wake) |
| `AON_CFG` | — | AON config (on-wake actions) |
| `RDB_STATUS` | — | Double RX buffer status |
| `RDB_STAT_EN` | — | Double RX buffer interrupt enable |
| `OTP_ADDR` | — | OTP address register |
| `OTP_DATA` | — | OTP data register |
| `OTP_CTRL` | — | OTP control register |
| `GPIO_MODE` | — | GPIO function select |
| `GPIO_DIR` | — | GPIO direction |
| `GPIO_OUT` | — | GPIO output values |
| `GPIO_RAW` | — | GPIO raw input values |
| `LED_CTRL` | — | LED control register |
| `XTAL` | — | Crystal oscillator trim |
| `TIMER_CTRL` | — | Timer control (DW3720) |
| `TIMER0_CNF/CNT` | — | TIMER0 config and count (DW3720) |
| `TIMER1_CNF/CNT` | — | TIMER1 config and count (DW3720) |
| `DUALSPISTATUS` | — | Dual SPI semaphore status (DW37xx) |

### Register Convention

Each register entry in `deca_regs.h` follows the pattern:

```c
#define <REG>_ID          <address>       // Base address
#define <REG>_LEN         (N)             // Length in bytes
#define <REG>_MASK        0x...           // Full register mask
#define <REG>_<FIELD>_BIT_OFFSET  (N)     // Bit position
#define <REG>_<FIELD>_BIT_LEN     (N)     // Field width in bits
#define <REG>_<FIELD>_BIT_MASK    0x...   // Field mask
```

Example — `SYS_CFG` (`0x010`):

| Field | Offset | Width | Mask | Description |
|-------|--------|-------|------|-------------|
| `FFEN` | 0 | 1 | `0x1` | Frame filtering enable |
| `DIS_FCS_TX` | 1 | 1 | `0x2` | Disable FCS on TX |
| `DIS_FCE` | 2 | 1 | `0x4` | Disable FCS check on RX |
| `DIS_DRXB` | 3 | 1 | `0x8` | Disable double RX buffer |
| `PHR_MODE` | 4 | 1 | `0x10` | PHR mode (std/ext) |
| `PHR_6M8` | 5 | 1 | `0x20` | PHR at 6.8 Mbps |
| `SPI_CRC` | 6 | 1 | `0x40` | SPI CRC enable |
| `CIA_IPATOV` | 7 | 1 | `0x80` | CIA use Ipatov |
| `CIA_STS` | 8 | 1 | `0x100` | CIA use STS |
| `RXWTOE` | 9 | 1 | `0x200` | RX wait timeout enable |
| `RXAUTR` | 10 | 1 | `0x400` | RX auto re-enable |
| `AUTO_ACK` | 11 | 1 | `0x800` | Auto ACK enable |
| `PDOA_MODE` | 16 | 2 | `0x30000` | PDOA mode |
| `FAST_AAT_EN` | 18 | 1 | `0x40000` | Fast auto ACK timing |

---

*Guide generated from driver version 06.00.14 header files.*
