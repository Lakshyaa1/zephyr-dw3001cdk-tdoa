#include <zephyr/kernel.h>
#include <stdio.h>
#include <string.h>

#include "deca_device_api.h"
#include "deca_probe_interface.h"
#include "dw3000_hw.h"
#include "port.h"

#define ROLE_INITIATOR 1

#define SPEED_OF_LIGHT 299702547.0
#define UUS_TO_DWT_TIME 63898

#define POLL_TX_TO_RESP_RX_DLY_UUS 900
#define RESP_RX_TO_FINAL_TX_DLY_UUS 900

#define MSG_POLL  0x01
#define MSG_RESP  0x02
#define MSG_FINAL 0x03

#define AVG_WINDOW 15

static uint16_t antenna_delay = 26194;

static double dist_buffer[AVG_WINDOW];
static int dist_index = 0;
static int dist_count = 0;

static dwt_config_t config = {
    .chan = 9,
    .txPreambLength = DWT_PLEN_128,
    .rxPAC = DWT_PAC8,
    .txCode = 9,
    .rxCode = 9,
    .sfdType = DWT_SFD_DW_8,
    .dataRate = DWT_BR_6M8,
    .phrMode = DWT_PHRMODE_STD,
    .phrRate = DWT_PHRRATE_STD,
    .sfdTO = (129 + 8 - 8),
    .stsMode = DWT_STS_MODE_OFF,
    .stsLength = DWT_STS_LEN_64,
    .pdoaMode = DWT_PDOA_M0,
};

static uint64_t get_tx_timestamp()
{
    uint8_t ts[5];
    uint64_t value = 0;

    dwt_readtxtimestamp(ts);

    for (int i = 4; i >= 0; i--)
        value = (value << 8) | ts[i];

    return value;
}

static uint64_t get_rx_timestamp()
{
    uint8_t ts[5];
    uint64_t value = 0;

    dwt_readrxtimestamp(ts);

    for (int i = 4; i >= 0; i--)
        value = (value << 8) | ts[i];

    return value;
}

static int uwb_init()
{
    dw_device_init();

    dw3000_hw_wakeup_pin_low();
    Sleep(5);

    port_set_dw_ic_spi_slowrate();

    if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS)
        return -1;

    port_set_dw_ic_spi_fastrate();

    if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS)
        return -1;

    if (dwt_configure(&config) != DWT_SUCCESS)
        return -1;

    dwt_setrxantennadelay(antenna_delay);
    dwt_settxantennadelay(antenna_delay);

    return 0;
}

void process_uart_command(char *cmd)
{
    if (strncmp(cmd, "SET_DELAY", 9) == 0)
    {
        int val;
        sscanf(cmd, "SET_DELAY %d", &val);

        antenna_delay = val;

        dwt_setrxantennadelay(antenna_delay);
        dwt_settxantennadelay(antenna_delay);

        printf("DELAY_SET %d\n", antenna_delay);
    }
}

#if ROLE_INITIATOR

static void initiator_loop()
{
    uint8_t poll_msg[2] = {MSG_POLL, 0};
    uint8_t resp_msg[32];
    uint8_t final_msg[32];

    uint8_t seq = 0;

    while (1)
    {
        poll_msg[1] = seq;

        dwt_writetxdata(sizeof(poll_msg), poll_msg, 0);
        dwt_writetxfctrl(sizeof(poll_msg) + FCS_LEN, 0, 0);

        dwt_starttx(DWT_START_TX_IMMEDIATE);

        while (!(dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK));

        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

        uint64_t t1 = get_tx_timestamp();

        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        uint32_t status;

        while (!((status = dwt_readsysstatuslo()) &
                 (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)));

        if (status & DWT_INT_RXFCG_BIT_MASK)
        {
            uint16_t len = dwt_getframelength();

            dwt_readrxdata(resp_msg, len - FCS_LEN, 0);

            uint64_t t4 = get_rx_timestamp();

            uint64_t t2 = 0;
            uint64_t t3 = 0;

            for (int i = 0; i < 5; i++)
            {
                t2 |= ((uint64_t)resp_msg[2 + i]) << (8 * i);
                t3 |= ((uint64_t)resp_msg[7 + i]) << (8 * i);
            }

            uint32_t final_tx_time =
                (t4 + RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME) >> 8;

            dwt_setdelayedtrxtime(final_tx_time);

            uint64_t t5 =
                (((uint64_t)(final_tx_time & 0xFFFFFFFE)) << 8);

            final_msg[0] = MSG_FINAL;
            final_msg[1] = seq;

            for (int i = 0; i < 5; i++)
                final_msg[2 + i] = (t1 >> (8 * i));

            for (int i = 0; i < 5; i++)
                final_msg[7 + i] = (t4 >> (8 * i));

            for (int i = 0; i < 5; i++)
                final_msg[12 + i] = (t5 >> (8 * i));

            dwt_writetxdata(17, final_msg, 0);
            dwt_writetxfctrl(17 + FCS_LEN, 0, 0);

            dwt_starttx(DWT_START_TX_DELAYED);

            while (!(dwt_readsysstatuslo() &
                     DWT_INT_TXFRS_BIT_MASK));

            dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
        }

        dwt_writesysstatuslo(
            DWT_INT_RXFCG_BIT_MASK |
            SYS_STATUS_ALL_RX_ERR);

        seq++;

        Sleep(200);
    }
}

#else

static void responder_loop()
{
    uint8_t rx_buf[32];
    uint8_t resp_msg[32];

    while (1)
    {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        uint32_t status;

        while (!((status = dwt_readsysstatuslo()) &
                 (DWT_INT_RXFCG_BIT_MASK |
                  SYS_STATUS_ALL_RX_ERR)));

        if (status & DWT_INT_RXFCG_BIT_MASK)
        {
            uint16_t len = dwt_getframelength();

            dwt_readrxdata(rx_buf, len - FCS_LEN, 0);

            if (rx_buf[0] == MSG_POLL)
            {
                uint8_t seq = rx_buf[1];

                uint64_t t2 = get_rx_timestamp();

                uint32_t resp_tx_time =
                    (t2 + POLL_TX_TO_RESP_RX_DLY_UUS *
                              UUS_TO_DWT_TIME) >>
                    8;

                dwt_setdelayedtrxtime(resp_tx_time);

                uint64_t t3 =
                    (((uint64_t)(resp_tx_time &
                                 0xFFFFFFFE))
                     << 8);

                resp_msg[0] = MSG_RESP;
                resp_msg[1] = seq;

                for (int i = 0; i < 5; i++)
                    resp_msg[2 + i] = (t2 >> (8 * i));

                for (int i = 0; i < 5; i++)
                    resp_msg[7 + i] = (t3 >> (8 * i));

                dwt_writetxdata(12, resp_msg, 0);
                dwt_writetxfctrl(12 + FCS_LEN, 0, 0);

                dwt_starttx(DWT_START_TX_DELAYED);

                while (!(dwt_readsysstatuslo() &
                         DWT_INT_TXFRS_BIT_MASK));

                dwt_writesysstatuslo(
                    DWT_INT_TXFRS_BIT_MASK);

                dwt_rxenable(DWT_START_RX_IMMEDIATE);

                while (!((status =
                              dwt_readsysstatuslo()) &
                         (DWT_INT_RXFCG_BIT_MASK |
                          SYS_STATUS_ALL_RX_ERR)));

                if (status &
                    DWT_INT_RXFCG_BIT_MASK)
                {
                    dwt_readrxdata(rx_buf, 17, 0);

                    uint64_t t1 = 0, t4 = 0, t5 = 0;

                    for (int i = 0; i < 5; i++)
                        t1 |= ((uint64_t)rx_buf[2 + i]) << (8 * i);

                    for (int i = 0; i < 5; i++)
                        t4 |= ((uint64_t)rx_buf[7 + i]) << (8 * i);

                    for (int i = 0; i < 5; i++)
                        t5 |= ((uint64_t)rx_buf[12 + i]) << (8 * i);

                    uint64_t t6 = get_rx_timestamp();

                    double Ra = (double)(t4 - t1);
                    double Rb = (double)(t6 - t3);
                    double Da = (double)(t5 - t4);
                    double Db = (double)(t3 - t2);

                    double tof =
                        (Ra * Rb - Da * Db) /
                        (Ra + Rb + Da + Db);

                    tof *= DWT_TIME_UNITS;

                    double dist =
                        tof * SPEED_OF_LIGHT;

                    dist_buffer[dist_index] = dist;
                    dist_index = (dist_index + 1) % AVG_WINDOW;

                    if (dist_count < AVG_WINDOW)
                        dist_count++;

                    if (dist_count == AVG_WINDOW)
                    {
                        double sum = 0;

                        for (int i = 0; i < AVG_WINDOW; i++)
                            sum += dist_buffer[i];

                        double avg = sum / AVG_WINDOW;

                        printf("DIST %.4f\n", avg);
                    }
                }
            }
        }

        dwt_writesysstatuslo(
            DWT_INT_RXFCG_BIT_MASK |
            SYS_STATUS_ALL_RX_ERR);
    }
}

#endif

int main(void)
{
    printf("UWB Calibration Firmware Started\n");

    if (uwb_init() != 0)
    {
        printf("UWB INIT FAILED\n");
        return -1;
    }

#if ROLE_INITIATOR
    initiator_loop();
#else
    responder_loop();
#endif

    return 0;
}