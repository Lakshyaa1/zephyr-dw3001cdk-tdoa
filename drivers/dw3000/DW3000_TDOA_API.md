# DW3000 API Reference: TDOA System

This file extracts every API element from `DW3000_API_GUIDE.md` that is directly relevant to building a Time Difference of Arrival (TDOA) positioning system with the DW3000.

TDOA requires: precise RX timestamps at multiple anchors for the same transmission, a shared or calibrated time base, and the ability to compute the arrival-time differences and relate them to position.

---

## Table of Contents

1. [TDOA Concept & How the API Supports It](#1-tdoa-concept--how-the-api-supports-it)
2. [Device Initialisation Sequence](#2-device-initialisation-sequence)
3. [PHY Configuration for TDOA](#3-phy-configuration-for-tdoa)
4. [Timestamp API](#4-timestamp-api)
5. [Clock Offset & Drift Correction](#5-clock-offset--drift-correction)
6. [Transmit Path (Tag / Blink Source)](#6-transmit-path-tag--blink-source)
7. [Receive Path (Anchor)](#7-receive-path-anchor)
8. [Interrupt & Callback Handling](#8-interrupt--callback-handling)
9. [System Status Registers](#9-system-status-registers)
10. [Antenna Delay Calibration](#10-antenna-delay-calibration)
11. [Time Base Synchronisation (OSTR / SYNC)](#11-time-base-synchronisation-ostr--sync)
12. [Diagnostics & Signal Quality](#12-diagnostics--signal-quality)
13. [Key Enumerations & Constants Quick-Reference](#13-key-enumerations--constants-quick-reference)
14. [Typical TDOA Code Patterns](#14-typical-tdoa-code-patterns)

---

## 1. TDOA Concept & How the API Supports It

In TDOA a **tag** (or blink transmitter) sends a single UWB packet. Multiple **anchors** each record the time of arrival independently. The position is solved from the differences `TDOA_ij = t_i - t_j` of arrival times at anchor pairs.

### API building blocks

| Need | Key API |
|------|---------|
| Tag transmits blink | `dwt_writetxdata`, `dwt_writetxfctrl`, `dwt_starttx` |
| Anchor records RX timestamp | `dwt_readrxtimestamp` / `dwt_readrxtimestamp_ipatov` |
| Convert timestamp to seconds | multiply by `DWT_TIME_UNITS` (15.65 ps) |
| Correct for inter-anchor clock drift | `dwt_readclockoffset`, `dwt_readcarrierintegrator` |
| Reject NLOS arrivals | `dwt_readdiagnostics`, `dwt_nlos_alldiag`, `dwt_readstsquality` |
| Synchronise anchor time bases | OSTR mode (`dwt_config_ostr_mode`) or DS-TWR clock-offset correction |
| Hardware-reported TDOA (PDOA devices) | `dwt_readtdoa`, `dwt_readpdoa` |

### Timestamp resolution

```
1 DW timestamp unit = DWT_TIME_UNITS = 1 / (499.2e6 × 128) ≈ 15.65 ps
1 DTU (Device Time Unit) = 2 chips = 1 / 249.6e6 ≈ 4.006 ns
40-bit counter wraps at 2^40 × 15.65 ps ≈ 17.2 seconds
```

---

## 2. Device Initialisation Sequence

Every anchor and tag must follow this sequence. The current project uses it in `sit_init()` and `uwb_init()`.

```c
// 1. Bring up hardware, hold WAKEUP pin low
dw3000_hw_wakeup_pin_low();
Sleep(5);

// 2. Set SPI slow before probe/init (< 7 MHz)
port_set_dw_ic_spi_slowrate();

// 3. Probe: match driver to device ID
if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS)
    // handle error

// 4. Wait for device to reach IDLE_RC (reads SYS_STATUS RCINIT bit)
while (!dwt_checkidlerc()) {}

// 5. Increase SPI to fast rate (up to 36 MHz)
port_set_dw_ic_spi_fastrate();

// 6. Initialise: reads OTP (LDO, bias, xtal trim), applies defaults
if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS)
    // handle error

// 7. Apply PHY config
if (dwt_configure(&config) != DWT_SUCCESS)
    // handle error (PLL lock fail -> reset and retry)

// 8. Apply TX spectrum config
dwt_configuretxrf(&txconfig);

// 9. Set antenna delays (calibrated per board)
dwt_setrxantennadelay(rx_ant_dly);
dwt_settxantennadelay(tx_ant_dly);

// 10. Enable CIA diagnostics (needed for NLOS filtering)
dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);

// 11. Enable LNA/PA if external front-end fitted
dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
```

### Relevant functions

| Function | Description |
|----------|-------------|
| `int dwt_probe(struct dwt_probe_s *probe_interf)` | Select driver; must be first. |
| `uint8_t dwt_checkidlerc(void)` | Returns 1 when device is in IDLE_RC (safe to increase SPI rate). |
| `int dwt_initialise(int mode)` | Init, read OTP. SPI < 7 MHz. `mode = DWT_DW_INIT`. |
| `int dwt_configure(dwt_config_t *config)` | Apply PHY config. Returns `DWT_ERROR` if PLL fails. |
| `void dwt_configuretxrf(dwt_txconfig_t *config)` | Set TX power and PG delay. |
| `void dwt_softreset(int reset_semaphore)` | Hard-reset if init fails. SPI < 7 MHz. |
| `void dwt_restoreconfig(void)` | Restore config after wake from deep sleep. |

---

## 3. PHY Configuration for TDOA

### `dwt_config_t` — recommended TDOA settings

The project currently uses channel 9, 128-symbol preamble, STS off, PDOA off. For TDOA:

```c
static dwt_config_t tdoa_config = {
    .chan           = 9,               // Channel 9 (7987.2 MHz centre)
    .txPreambLength = DWT_PLEN_128,    // Longer preamble → better first-path estimate
    .rxPAC          = DWT_PAC8,        // PAC 8 for preamble ≤ 128 symbols
    .txCode         = 9,               // Preamble code 9 (64 MHz PRF)
    .rxCode         = 9,
    .sfdType        = DWT_SFD_DW_8,   // DW 8-bit SFD (non-standard, better performance)
    .dataRate       = DWT_BR_6M8,      // 6.8 Mbps for short on-air time
    .phrMode        = DWT_PHRMODE_STD, // Standard PHR (≤ 127 B payload)
    .phrRate        = DWT_PHRRATE_STD,
    .sfdTO          = (129 + 8 - 8),   // = preamble_syms + 1 + sfd_len - pac_size
    .stsMode        = DWT_STS_MODE_OFF,// For basic TDOA; use STS_MODE_1 for security
    .stsLength      = DWT_STS_LEN_64,
    .pdoaMode       = DWT_PDOA_M0,    // Off for pure TDOA; M1/M3 for AoA+TDOA
};
```

### SFD timeout formula

```
sfdTO = txPreambLength_symbols + 1 + sfdLen_symbols - rxPAC_symbols
      = 128 + 1 + 8 - 8 = 129   (as used in project)
```

### Key PHY constants for TDOA timing budgets

| Macro | Value | Use |
|-------|-------|-----|
| `DWT_TIME_UNITS` | ~15.65 ps | Convert raw timestamps to seconds |
| `DW3000_DTU_FREQ` | 249.6 MHz | DTU tick rate |
| `UUS_TO_DWT_TIME` | 65536 (= 512×128) | µs → DW time units (used in project) |
| `DWT_SFDTOC_DEF` | 129 | Default SFD timeout |
| `RX_BUFFER_MAX_LEN` | 1023 | Max frame size |

### Preamble length trade-off for TDOA

| Preamble | First-path SNR | On-air time | Use when |
|----------|---------------|-------------|----------|
| `DWT_PLEN_64` | Lower | ~70 µs | Short range, high rate blinks |
| `DWT_PLEN_128` | Good | ~130 µs | General TDOA (used in project) |
| `DWT_PLEN_512` | High | ~500 µs | Long range, noisy environment |
| `DWT_PLEN_1024` | Very high | ~1 ms | Maximum range |

---

## 4. Timestamp API

This is the core of TDOA. Every anchor records the RX timestamp of the same tag blink.

### Reading the RX timestamp (anchor side)

```c
uint8_t rx_ts[5];
dwt_readrxtimestamp(rx_ts);       // Adjusted (antenna delay subtracted), recommended

// Or read specific sequences:
dwt_readrxtimestamp_ipatov(rx_ts); // From Ipatov CIR (standard preamble)
dwt_readrxtimestamp_sts(rx_ts);    // From STS CIR (when STS mode enabled)
dwt_readrxtimestampunadj(rx_ts);   // Raw, no antenna delay correction

// Convert 5-byte little-endian to uint64
uint64_t ts = ((uint64_t)rx_ts[4] << 32) |
              ((uint64_t)rx_ts[3] << 24) |
              ((uint64_t)rx_ts[2] << 16) |
              ((uint64_t)rx_ts[1] <<  8) |
               (uint64_t)rx_ts[0];

// Convert to seconds
double ts_sec = ts * DWT_TIME_UNITS;  // 15.65e-12 s per unit
```

### Reading the TX timestamp (tag / DS-TWR initiator side)

```c
uint8_t tx_ts[5];
dwt_readtxtimestamp(tx_ts);       // Adjusted TX timestamp (antenna delay included)

uint32_t tx_hi = dwt_readtxtimestamphi32();  // High 32 bits only
uint32_t tx_lo = dwt_readtxtimestamplo32();  // Low 32 bits only
```

### System time (free-running counter)

```c
uint8_t sys_ts[4];
dwt_readsystime(sys_ts);           // 32-bit snapshot of free-running counter

uint32_t sys_hi = dwt_readsystimestamphi32();
```

### Timestamp function reference

| Function | Bits | Adjusted? | Use for |
|----------|------|-----------|---------|
| `dwt_readrxtimestamp(buf)` | 40 | Yes (ant. delay subtracted) | Standard TDOA anchor RX timestamp |
| `dwt_readrxtimestamp_ipatov(buf)` | 40 | Yes | Ipatov-specific; use in double-buffer mode |
| `dwt_readrxtimestamp_sts(buf)` | 40 | Yes | STS-specific timestamp |
| `dwt_readrxtimestampunadj(buf)` | 40 | No | Calibration, debugging |
| `dwt_readrxtimestamphi32()` | 32 hi | Yes | Quick read; **not for double-buffer mode** |
| `dwt_readrxtimestamplo32()` | 32 lo | Yes | Quick read; **not for double-buffer mode** |
| `dwt_readtxtimestamp(buf)` | 40 | Yes | Tag TX timestamp for DS-TWR clock sync |
| `dwt_readtxtimestamphi32()` | 32 hi | Yes | High bits of TX timestamp |
| `dwt_readtxtimestamplo32()` | 32 lo | Yes | Low bits of TX timestamp |
| `dwt_readsystime(buf)` | 32 | n/a | Current system counter value |
| `dwt_readsystimestamphi32()` | 32 hi | n/a | Snapshot for scheduling |
| `dwt_reset_system_counter()` | — | — | Reset counter to 0 (use with OSTR for sync) |

### Hardware TDOA register (`dwt_readtdoa`)

On PDOA-capable devices (DW3000 PDOA, DW3700, DW3720) the CIA hardware computes the TDOA between the two STS blocks:

```c
uint8_t tdoa[6];
dwt_readtdoa(tdoa);   // 41-bit signed value, units = DWT_TIME_UNITS

// Validity check (from API header):
// if abs(TDOA_raw) > DWT_VALID_TDOA_LIMIT (100), PDOA is not valid
```

This is *intra-packet* TDOA (between STS1 and STS2 within one frame), not inter-anchor TDOA — but it is the raw material for PDOA-based AoA which can complement TDOA positioning.

---

## 5. Clock Offset & Drift Correction

TDOA accuracy is destroyed by unsynchronised clocks. The DW3000 provides two measurements to correct for this.

### Clock offset (crystal frequency error)

```c
int16_t offset = dwt_readclockoffset();
// Format: s[6:-4], divide by 16 to get ppm
// Positive = local RX clock faster than remote TX
double ppm = (double)offset / 16.0;
```

### Carrier integrator (fine frequency offset)

```c
int32_t ci = dwt_readcarrierintegrator();
// Multiply to get Hz:
double freq_offset_hz = ci * FREQ_OFFSET_MULTIPLIER;
//   FREQ_OFFSET_MULTIPLIER = 998.4e6 / 2.0 / 1024.0 / 131072.0

// Convert to ppm for the channel in use:
double ppm_ch9 = freq_offset_hz * HERTZ_TO_PPM_MULTIPLIER_CHAN_9;
double ppm_ch5 = freq_offset_hz * HERTZ_TO_PPM_MULTIPLIER_CHAN_5;
//   HERTZ_TO_PPM_MULTIPLIER_CHAN_9 = -1.0e6 / 7987.2e6
//   HERTZ_TO_PPM_MULTIPLIER_CHAN_5 = -1.0e6 / 6489.6e6
```

### How the project applies clock offset correction (from `sit.c`)

```c
double clockOffsetRatio = dwt_readcarrierintegrator() *
    (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_9 / 1.0e6);

// Applied in SS-TWR TOF formula to correct for remote clock rate:
double tof = ((time_round - time_reply * (1.0 - clockOffsetRatio)) / 2.0)
             * DWT_TIME_UNITS;
```

For TDOA with multiple anchors, each anchor measures the carrier integrator value from the received tag blink and uses it to map its local timestamp onto a common reference clock before computing differences.

### Anchor-to-anchor clock synchronisation approaches

| Method | API | Notes |
|--------|-----|-------|
| **Passive listening + offset correction** | `dwt_readcarrierintegrator()` per packet | Each anchor corrects independently per blink |
| **DS-TWR clock calibration** | Full TX/RX timestamp exchange | Used in `sit_two_device_calibration_*` |
| **Hardware OSTR sync** | `dwt_config_ostr_mode()` | External SYNC pulse resets all anchor counters simultaneously |
| **Crystal trim** | `dwt_setxtaltrim()` | Trim to reduce raw drift before correction |

---

## 6. Transmit Path (Tag / Blink Source)

The tag sends periodic blinks. It needs no anchor-side intelligence.

### Minimal blink transmit

```c
// Write payload into TX buffer
int ret = dwt_writetxdata(payload_len, payload_bytes, 0 /*offset*/);

// Set frame control: length (including 2-byte FCS), offset=0, ranging=0
dwt_writetxfctrl(payload_len, 0, 0);

// Start TX immediately
dwt_starttx(DWT_START_TX_IMMEDIATE);

// Wait for TXFRS (frame sent) in sys status
uint32_t status;
while (!((status = dwt_readsysstatuslo()) & DWT_INT_TXFRS_BIT_MASK)) {}
dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);  // clear
```

### Delayed TX (for scheduled blinks or two-way assist messages)

```c
// Compute future TX time from current system time
uint32_t tx_time = (dwt_readsystimestamphi32() + delay_dtu) & 0xFFFFFFFEUL;
dwt_setdelayedtrxtime(tx_time);

// The exact 40-bit TX timestamp including antenna delay:
uint64_t tx_ts_40 = ((uint64_t)(tx_time & 0xFFFFFFFEUL) << 8) + tx_ant_dly;

dwt_starttx(DWT_START_TX_DELAYED);
```

### TX function reference

| Function | Description |
|----------|-------------|
| `int dwt_writetxdata(uint16_t len, uint8_t *data, uint16_t offset)` | Load up to 1023 bytes into TX buffer. |
| `void dwt_writetxfctrl(uint16_t frameLen, uint16_t offset, uint8_t ranging)` | Set TX frame control register. `ranging=1` sets ranging bit in frame. |
| `int dwt_starttx(uint8_t mode)` | Start TX. Returns `DWT_ERROR` if delayed time has passed. |
| `void dwt_setdelayedtrxtime(uint32_t starttime)` | Set high-32-bit delayed TX/RX start time. |
| `void dwt_setreferencetrxtime(uint32_t reftime)` | Set reference time for `DWT_START_TX_DLY_REF` mode. |
| `void dwt_setrxaftertxdelay(uint32_t rxDelayTime)` | After TX, turn on RX after this delay (for response-expected patterns). |
| `void dwt_setplenfine(uint8_t preambleLength)` | Override preamble length in 8-symbol steps (16–2048). |

---

## 7. Receive Path (Anchor)

Each anchor listens continuously and records the RX timestamp of every tag blink.

### Minimal always-on anchor RX loop (polling)

```c
dwt_rxenable(DWT_START_RX_IMMEDIATE);

while (1) {
    uint32_t status;
    // Wait for RX good frame or error
    while (!((status = dwt_readsysstatuslo()) &
             (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO))) {}

    if (status & DWT_INT_RXFCG_BIT_MASK) {
        // Good frame — read timestamp immediately before re-enabling RX
        uint8_t rx_ts[5];
        dwt_readrxtimestamp(rx_ts);

        uint16_t frame_len = dwt_getframelength();
        uint8_t buf[128];
        dwt_readrxdata(buf, frame_len - 2, 0);  // -2 for FCS

        dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | DWT_INT_RXFR_BIT_MASK);
    } else {
        // Error or timeout — clear and re-enable
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
    }
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}
```

### RX function reference

| Function | Description |
|----------|-------------|
| `int dwt_rxenable(int mode)` | Enable receiver. Returns `DWT_ERROR` if delayed start is already late. |
| `void dwt_readrxdata(uint8_t *buf, uint16_t len, uint16_t offset)` | Read received frame bytes. |
| `uint16_t dwt_getframelength(void)` | Return byte count of last received frame (includes FCS). |
| `void dwt_setrxtimeout(uint32_t time)` | Frame-wait timeout in 512/499.2 MHz ≈ 1.026 µs units. 0 = infinite. |
| `void dwt_setpreambledetecttimeout(uint16_t timeout)` | Preamble timeout in multiples of PAC size. |
| `void dwt_forcetrxoff(void)` | Immediately abort RX (e.g. on command to stop). |

### RX mode flags for TDOA anchors

| Mode | Use |
|------|-----|
| `DWT_START_RX_IMMEDIATE` | Continuous listening anchor |
| `DWT_START_RX_DELAYED` | Anchor pre-schedules RX window around expected blink time |
| `DWT_IDLE_ON_DLY_ERR` | Combined with delayed: stay idle if blink missed (power saving) |

---

## 8. Interrupt & Callback Handling

For a real-time anchor, use interrupt-driven RX rather than polling.

### Register callbacks

```c
void rx_ok_cb(const dwt_cb_data_t *cb_data) {
    // cb_data->datalength = frame length
    // cb_data->rx_flags   = DWT_CB_DATA_RX_FLAG_* bitmask
    uint8_t rx_ts[5];
    dwt_readrxtimestamp(rx_ts);
    // process / store timestamp ...
}

void rx_err_cb(const dwt_cb_data_t *cb_data) {
    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

dwt_setcallbacks(NULL,      // TX done - not needed on anchor
                 rx_ok_cb,  // RX good
                 rx_err_cb, // RX timeout
                 rx_err_cb, // RX error
                 NULL, NULL, NULL);
```

### Enable interrupts

```c
// Enable RX good frame + all RX errors + RX timeouts
dwt_setinterrupt(DWT_INT_RXFCG_BIT_MASK
               | SYS_STATUS_ALL_RX_ERR
               | SYS_STATUS_ALL_RX_TO,
               0,
               DWT_ENABLE_INT_ONLY);
```

### Call ISR from hardware IRQ handler

```c
// In your MCU IRQ handler:
void dw3000_irq_handler(void) {
    dwt_isr();  // Reads SYS_STATUS, clears flags, fires registered callbacks
}
```

### Callback and interrupt function reference

| Function | Description |
|----------|-------------|
| `void dwt_setcallbacks(cbTxDone, cbRxOk, cbRxTo, cbRxErr, cbSPIErr, cbSPIRdy, cbDualSPIEv)` | Register all event callbacks. NULL to ignore. |
| `void dwt_setinterrupt(uint32_t bitmask_lo, uint32_t bitmask_hi, dwt_INT_options_e op)` | Set interrupt enable mask. |
| `void dwt_isr(void)` | ISR: reads status, clears, fires callbacks. Call from hardware IRQ. |
| `uint8_t dwt_checkirq(void)` | Polled alternative: returns 1 if IRQ line active. |

### `dwt_cb_data_t` fields used in TDOA

| Field | Description |
|-------|-------------|
| `status` | `SYS_STATUS_LO` snapshot — check `DWT_INT_RXFCG_BIT_MASK` for good frame |
| `datalength` | Frame length in bytes (includes FCS) |
| `rx_flags` | `DWT_CB_DATA_RX_FLAG_RNG` (ranging bit), `DWT_CB_DATA_RX_FLAG_CIA` (CIA done) |

---

## 9. System Status Registers

Used to check for RX events when polling, and to clear events after handling.

### Useful status masks for TDOA

| Mask | Description |
|------|-------------|
| `DWT_INT_RXFCG_BIT_MASK` | RX frame CRC good — primary "good packet" flag |
| `DWT_INT_CIADONE_BIT_MASK` | CIA done — timestamp is ready |
| `DWT_INT_RXFR_BIT_MASK` | RX frame ready (use together with RXFCG) |
| `DWT_INT_TXFRS_BIT_MASK` | TX frame sent — read TX timestamp after this |
| `SYS_STATUS_ALL_RX_ERR` | Ored mask of all RX error flags |
| `SYS_STATUS_ALL_RX_TO` | Ored mask of all RX timeout flags |
| `SYS_STATUS_ALL_RX_GOOD` | Ored mask of all good RX flags |
| `DWT_INT_RXPTO_BIT_MASK` | Preamble timeout |
| `DWT_INT_RXFTO_BIT_MASK` | Frame wait timeout |
| `DWT_INT_CPERR_BIT_MASK` | STS quality error |
| `DWT_INT_RCINIT_BIT_MASK` | Device entered IDLE_RC (check after reset) |

### Status register functions

| Function | Description |
|----------|-------------|
| `uint32_t dwt_readsysstatuslo(void)` | Read lower 32 bits of SYS_STATUS |
| `uint32_t dwt_readsysstatushi(void)` | Read upper bits (16-bit on DW3000, 32-bit on DW3720) |
| `void dwt_writesysstatuslo(uint32_t mask)` | Clear status bits (write-1-to-clear) |
| `void dwt_writesysstatushi(uint32_t mask)` | Clear upper status bits |

---

## 10. Antenna Delay Calibration

Antenna delay directly adds a fixed offset to every RX and TX timestamp. Incorrect delays are the main source of systematic TDOA error — all anchors must have their delays calibrated.

```c
// Set during init (project default: 16385 = ~256 ns for both)
dwt_setrxantennadelay(rx_ant_dly);
dwt_settxantennadelay(tx_ant_dly);

// Read back:
uint16_t rx_dly = dwt_getrxantennadelay();
uint16_t tx_dly = dwt_gettxantennadelay();
```

### Unit conversion

```
antenna_delay_units = antenna_delay_seconds / DWT_TIME_UNITS
                    = antenna_delay_seconds / 15.65e-12

Example: 257.0 ns / 15.65e-12 = 16425 units
```

### Calibration loop pattern (from project)

In `sit_sstwr_responder()` the antenna delay is used to predict the exact TX timestamp:

```c
uint32_t resp_tx_time = (poll_rx_ts + (1800 * UUS_TO_DWT_TIME)) >> 8;
// Predicted 40-bit TX timestamp (for embedding in response message):
uint32_t resp_tx_ts_40lsb = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8)
                            + dwt_gettxantennadelay();
```

For TDOA, both RX and TX antenna delays must be equal across all anchors, or each anchor's delay must be measured individually and stored.

### Bandwidth calibration (affects timestamp accuracy)

```c
// After dwt_configure(), recalibrate bandwidth at current temperature:
uint8_t pg_delay = dwt_calcbandwidthadj(target_count);
// Then update TX config with new pg_delay and re-call dwt_configuretxrf()

// Reference count measurement:
uint16_t pg_count = dwt_calcpgcount(pg_delay);
```

### Crystal trim (reduces raw drift between anchors)

```c
// Read OTP-programmed value at init (done automatically by dwt_initialise):
uint8_t trim = dwt_getxtaltrim();   // value applied at init

// Manual adjustment (0x00–0x3F, ~1.65 ppm/step, default = DEFAULT_XTAL_TRIM = 0x2E):
dwt_setxtaltrim(trim);
```

---

## 11. Time Base Synchronisation (OSTR / SYNC)

For cable-free anchor synchronisation the DW3700/DW3720 supports OSTR (One-Shot Timebase Reset): all anchors share a SYNC GPIO line; a single rising edge causes every anchor to reset its internal system counter at a deterministic phase.

```c
// Enable OSTR on each anchor before synchronisation pulse:
dwt_config_ostr_mode(1, 33);  // enable=1, wait_time=33 (recommended, must be ≡1 mod 4)
// After the SYNC pulse fires, counter is reset.

// To disable:
dwt_config_ostr_mode(0, 0);

// Manual reset of counter (software triggered):
dwt_reset_system_counter();
```

### OSTR timing note

`wait_time` must satisfy `wait_time % 4 == 1` (e.g. 29, 33, 37…). The value 33 is recommended in the DW3700 User Manual. Setting an incorrect modulo changes the phase relationship between the 38.4 MHz external clock and the 125 MHz system clock.

---

## 12. Diagnostics & Signal Quality

Good TDOA requires detecting and rejecting NLOS (non-line-of-sight) arrivals where the first path is blocked and a reflected path is timestamped instead.

### CIA diagnostics (enable at init)

```c
dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);  // Log all: Ipatov + STS diagnostics
```

### Read full diagnostics

```c
dwt_rxdiag_t diag;
dwt_readdiagnostics(&diag);

// Key fields for NLOS detection:
// diag.ipatovFpIndex    - first path index in Ipatov CIR
// diag.ipatovPeak       - peak index and amplitude
// diag.ipatovPower      - channel power (Ipatov)
// diag.ipatovF1/F2/F3   - first path amplitude samples
// diag.xtalOffset       - remote xtal offset (use for clock correction)
```

### NLOS detection functions

```c
// Quick NLOS classification from first/peak path index separation:
dwt_nlos_ipdiag_t ip_index;
dwt_nlos_ipdiag(&ip_index);
// ip_index.index_fp_u32  - first path index
// ip_index.index_pp_u32  - peak path index
// Large separation -> suspected NLOS

// Full NLOS diagnostic including Ipatov/STS1/STS2 signal metrics:
dwt_nlos_alldiag_t nlos;
nlos.diag_type = IPATOV;
uint8_t ok = dwt_nlos_alldiag(&nlos);
// nlos.result indicates LOS/NLOS classification
// nlos.F1/F2/F3 = first path amplitudes
// nlos.cir_power = channel power
// nlos.D = DGC decision (0-7)
```

### STS quality (when STS mode enabled)

```c
int16_t sts_idx;
int sts_ok = dwt_readstsquality(&sts_idx);
// sts_ok >= 0: good STS (for 64 MHz PRF, expect sts_idx >= 0.9 * sts_length_symbols)
// sts_ok < 0:  poor STS quality — timestamp may be unreliable
```

### DGC decision

```c
uint8_t dgc = dwt_get_dgcdecision();
// Used in the received signal level estimation formulas (see User Manual)
```

### Diagnostic function reference

| Function | Description |
|----------|-------------|
| `void dwt_configciadiag(uint8_t enable_mask)` | Enable CIA diagnostic logging. Use `DW_CIA_DIAG_LOG_ALL` for NLOS. |
| `void dwt_readdiagnostics(dwt_rxdiag_t *diag)` | Read full RX diagnostic data (CIR peaks, powers, PDOA, TDOA, xtalOffset). |
| `void dwt_nlos_ipdiag(dwt_nlos_ipdiag_t *idx)` | Read Ipatov first/peak path indices for NLOS detection. |
| `uint8_t dwt_nlos_alldiag(dwt_nlos_alldiag_t *all_diag)` | Full LOS/NLOS classification using Ipatov or STS. |
| `int dwt_readstsquality(int16_t *rxStsQualityIndex)` | STS quality; < 0 = unreliable timestamp. |
| `int dwt_readstsstatus(uint16_t *stsStatus, int sts_num)` | STS status for STS0 or STS1. |
| `uint8_t dwt_get_dgcdecision(void)` | Read DGC decision index for signal level computation. |
| `int16_t dwt_readclockoffset(void)` | Remote clock offset in ppm (divide raw by 16). |
| `int32_t dwt_readcarrierintegrator(void)` | Carrier integrator → multiply by `FREQ_OFFSET_MULTIPLIER` for Hz. |

---

## 13. Key Enumerations & Constants Quick-Reference

### For `dwt_starttx()` mode

| Constant | TDOA use |
|----------|----------|
| `DWT_START_TX_IMMEDIATE` | Tag blink, no response needed |
| `DWT_START_TX_DELAYED` | Scheduled blink at precise time |
| `DWT_START_TX_DLY_RS` | Delayed from last RX timestamp (DS-TWR style assist) |
| `DWT_RESPONSE_EXPECTED` | TX then immediately enable RX (TWR assist on anchor) |

### For `dwt_rxenable()` mode

| Constant | TDOA use |
|----------|----------|
| `DWT_START_RX_IMMEDIATE` | Continuous anchor listening |
| `DWT_START_RX_DELAYED` | Pre-schedule RX around expected blink arrival |
| `DWT_IDLE_ON_DLY_ERR` | Save power if blink not received |

### For `dwt_setinterrupt()` operation

| Constant | Use |
|----------|-----|
| `DWT_ENABLE_INT_ONLY` | Replace mask completely (safest for init) |
| `DWT_ENABLE_INT` | Add bits to existing mask |
| `DWT_DISABLE_INT` | Remove bits from existing mask |

### Useful `dwt_on_wake_param_e` bits (sleep config)

| Constant | TDOA use |
|----------|----------|
| `DWT_CONFIG` | Reload AON config on wake (essential) |
| `DWT_GOTOIDLE` | Wake into IDLE (not RX) — anchor re-enables RX explicitly |
| `DWT_GOTORX` | Wake directly into RX (for always-listening anchors) |
| `DWT_PGFCAL` | Re-run PGF calibration on wake (clear on DW3720) |
| `DWT_LOADDGC` | Reload DGC table on wake |

### Double-buffer status bits (anchor with continuous RX)

| Constant | Description |
|----------|-------------|
| `DWT_RDB_STATUS_RXFCG0_BIT_MASK` | Good frame in buffer 0 |
| `DWT_RDB_STATUS_RXFCG1_BIT_MASK` | Good frame in buffer 1 |
| `DWT_RDB_STATUS_CIADONE0_BIT_MASK` | CIA done for buffer 0 (timestamp ready) |
| `DWT_RDB_STATUS_CIADONE1_BIT_MASK` | CIA done for buffer 1 |

---

## 14. Typical TDOA Code Patterns

### Pattern A: Simple anchor — receive blink, record timestamp

```c
void anchor_run(void) {
    /* Init as above, then: */
    dwt_setrxtimeout(0);              // Disable timeout: listen forever
    dwt_setpreambledetecttimeout(0);  // No preamble timeout
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    while (1) {
        uint32_t status;
        while (!((status = dwt_readsysstatuslo()) &
                 (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {}

        if (status & DWT_INT_RXFCG_BIT_MASK) {
            /* Read timestamp before anything else */
            uint8_t rx_ts[5];
            dwt_readrxtimestamp(rx_ts);
            uint64_t ts = 0;
            for (int i = 4; i >= 0; i--) ts = (ts << 8) | rx_ts[i];

            /* Read frame */
            uint16_t len = dwt_getframelength();
            uint8_t buf[128];
            dwt_readrxdata(buf, len - 2, 0);

            /* Read clock offset for correction */
            int16_t clk_off = dwt_readclockoffset();
            double ppm = (double)clk_off / 16.0;

            /* Optional NLOS check */
            dwt_nlos_ipdiag_t nlos_idx;
            dwt_nlos_ipdiag(&nlos_idx);
            bool nlos = (nlos_idx.index_pp_u32 - nlos_idx.index_fp_u32) > NLOS_THRESHOLD;

            /* Report ts, ppm, nlos to TDOA solver */
            tdoa_report_anchor(anchor_id, ts, ppm, nlos);

            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD);
        } else {
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR);
        }
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
}
```

### Pattern B: Tag — transmit periodic blink with sequence number

```c
void tag_run(void) {
    uint8_t seq = 0;
    while (1) {
        uint8_t blink[12] = {0xC5, seq++}; // IEEE 802.15.4 blink frame
        dwt_writetxdata(sizeof(blink), blink, 0);
        dwt_writetxfctrl(sizeof(blink) + 2 /*FCS*/, 0, 0 /*not ranging*/);
        dwt_starttx(DWT_START_TX_IMMEDIATE);

        /* Wait for TX done */
        while (!(dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK)) {}
        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

        k_msleep(100); /* Blink rate: 10 Hz */
    }
}
```

### Pattern C: DS-TWR-based clock calibration between two anchors

This is what the project already does in `sit_two_device_calibration_*`. The result gives the inter-anchor clock offset and drift, which enables converting per-anchor timestamps to a common time base:

```c
// On anchor A (initiator role):
time_round_1 = (double)(sensing_2_rx - sensing_1_tx);  // raw DW units
time_reply_1 = (double)(sensing_2_tx - sensing_1_rx);  // anchor B's reply time

// On anchor C (passive observer):
time_b21 = (double)(sensing_2_rx_c - sensing_1_rx_c);  // C's view of same interval
time_m21 = (double)(sensing_2_rx_a - sensing_1_tx_a);  // A's view

// Clock ratio between C and A:
double ratio_CA = time_b21 / time_m21;
// Use ratio_CA to scale C's timestamps onto A's clock before computing TDOA
```

### Pattern D: Computing TDOA from two anchor timestamps

```c
// Both timestamps already in A's time reference after clock correction:
double ts_A_sec = ts_anchor_A * DWT_TIME_UNITS;
double ts_B_sec = ts_anchor_B_corrected * DWT_TIME_UNITS;

double tdoa_sec = ts_A_sec - ts_B_sec;
double tdoa_m   = tdoa_sec * SPEED_OF_LIGHT;  // SPEED_OF_LIGHT = 299702547.0 m/s

// tdoa_m is the TDOA in metres — input to hyperbolic position solver
```

### Pattern E: Enabling STS for secure/authenticated TDOA

```c
/* Configure STS key and IV (unique per deployment) */
dwt_sts_cp_key_t sts_key = {0xc9a375fa, 0x8df43a20, 0xb5e5a4ed, 0x0738123b};
dwt_sts_cp_iv_t  sts_iv  = {0x00000001, 0x00000000, 0x00000000, 0x00000000};

dwt_configurestskey(&sts_key);
dwt_configurestsiv(&sts_iv);
dwt_configurestsloadiv();  // Load IV into hardware

/* Use STS_MODE_1 in config */
config.stsMode   = DWT_STS_MODE_1;
config.stsLength = DWT_STS_LEN_64;
dwt_configure(&config);

/* After RX, check STS quality before accepting timestamp */
int16_t sts_idx;
if (dwt_readstsquality(&sts_idx) < 0) {
    /* Reject this measurement — STS quality too poor */
}

/* Use STS-specific timestamp for best accuracy when STS is enabled */
uint8_t rx_ts[5];
dwt_readrxtimestamp_sts(rx_ts);
```

---

## Summary: Minimum API Set for a TDOA Anchor Node

```
INIT:     dwt_probe → dwt_checkidlerc → dwt_initialise → dwt_configure
          → dwt_configuretxrf → dwt_setrxantennadelay → dwt_settxantennadelay
          → dwt_configciadiag(DW_CIA_DIAG_LOG_ALL)

RX LOOP:  dwt_rxenable → [wait] → dwt_readsysstatuslo
          → dwt_readrxtimestamp (timestamp ASAP)
          → dwt_getframelength → dwt_readrxdata
          → dwt_readclockoffset (or dwt_readcarrierintegrator)
          → dwt_nlos_ipdiag (reject NLOS)
          → dwt_writesysstatuslo (clear flags)
          → [back to dwt_rxenable]

OPTIONAL: dwt_readstsquality (if STS enabled)
          dwt_readdiagnostics (full CIR data)
          dwt_config_ostr_mode (hardware time sync)
          dwt_setxtaltrim (crystal frequency calibration)
```
