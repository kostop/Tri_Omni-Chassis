#include "dw1000.h"
#include <stdlib.h>
#include <string.h>
#include "math.h"
//光速
#define SPEED_OF_LIGHT 299702547

/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define ALL_MSG_TAG_IDX 3
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4
#define ANGLE_MSG_IDX 10
#define LOCATION_FLAG_IDX 11
#define LOCATION_INFO_LEN_IDX 12
#define LOCATION_INFO_START_IDX 13
#define ANGLE_MSG_MAX_LEN 30

//中断标志位
static uint32 status_reg = 0;
//接收数据
static int8 frame_len = 0;
#define RX_BUF_LEN 30
static uint8 rx_buffer[RX_BUF_LEN];
//累加数据数据位
static uint8 frame_seq_nb = 0;
static uint8 frame_seq_nb_semaphore = 0;
//接收数据id索引
static uint8 anthor_index = 0;
static uint8 tag_index = 0;
static uint8 tag_index1 = 0;


#define TX_ANT_DLY 0
#define RX_ANT_DLY 32950

#define POLL_RX_TO_RESP_TX_DLY_UUS 4000
#define RESP_TX_TO_FINAL_RX_DLY_UUS 60
#define FINAL_RX_TIMEOUT_UUS 12000

//通信时间戳
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;
static uint64 final_rx_ts;

//通信数据包
static uint8 rx_poll_msg[] =  {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 tx_resp_msg[] =  {0x41, 0x88, 0, 0x0, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 rx_final_msg[] = {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 distance_msg[] = {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xAA, 0, 0,0, 0, 0, 0};

static dwt_config_t config =
{
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* Use non-standard SFD (Boolean) */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

#define CAR_NUM	3
#define ANCHOR
#define ANCHOR_MAX_NUM 3
#define ANCHOR_IND 0  // 0 1 2
	
//测距
static double tof;
static double distance;


static int Anthordistance[ANCHOR_MAX_NUM];
static int Anthordistance_count[ANCHOR_MAX_NUM];

static void Anchor_Array_Init(void);
static void reset_DW1000(void);
static void spi_set_rate_low(void);
static void spi_set_rate_high(void);

static uint64 get_rx_timestamp_u64(void);
static uint64 get_tx_timestamp_u64(void);
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);


static int temp;
void dw_task(void const * argument)
{
	reset_DW1000();

    spi_set_rate_low();
    while(dwt_initialise(DWT_LOADUCODE) == -1)
	{
		//init fail
	};
    spi_set_rate_high();
    dwt_configure(&config);
    dwt_setleds(1);
	
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    Anchor_Array_Init();
	while(1)
	{
        dwt_setrxtimeout(0);
        dwt_rxenable(0);
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
        { };
        if (status_reg & SYS_STATUS_RXFCG)
        {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
            if (frame_len <= RX_BUFFER_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }
            if(rx_buffer[ALL_MSG_SN_IDX]%ANCHOR_MAX_NUM != ANCHOR_IND)
                continue;

            anthor_index = rx_buffer[ALL_MSG_SN_IDX]%ANCHOR_MAX_NUM;
            tag_index = rx_buffer[ALL_MSG_TAG_IDX];

            rx_buffer[ALL_MSG_SN_IDX] = 0;
            rx_buffer[ALL_MSG_TAG_IDX] = 0;

            if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                /* Retrieve poll reception timestamp. */
                poll_rx_ts = get_rx_timestamp_u64();

                /* Set expected delay and timeout for final message reception. */
                dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
                dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

                /* Write and send the response message. See NOTE 9 below.*/
                tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                tx_resp_msg[ALL_MSG_TAG_IDX] = tag_index;
                dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
                dwt_writetxfctrl(sizeof(tx_resp_msg), 0);
                dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

                while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
                { };

                if (status_reg & SYS_STATUS_RXFCG)
                {
                    /* Clear good RX frame event and TX frame sent in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

                    /* A frame has been received, read it into the local buffer. */
                    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                    if (frame_len <= RX_BUF_LEN)
                    {
                        dwt_readrxdata(rx_buffer, frame_len, 0);
                    }

                    rx_buffer[ALL_MSG_SN_IDX] = 0;
                    if(tag_index != rx_buffer[ALL_MSG_TAG_IDX])
                        continue;
                    rx_buffer[ALL_MSG_TAG_IDX] = 0;
                    if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0)
                    {
                        uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
                        uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                        double Ra, Rb, Da, Db;
                        int64 tof_dtu;

                        /* Retrieve response transmission and final reception timestamps. */
                        resp_tx_ts = get_tx_timestamp_u64();
                        final_rx_ts = get_rx_timestamp_u64();

                        /* Get timestamps embedded in the final message. */
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

                        /* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 10 below. */
                        poll_rx_ts_32 = (uint32)poll_rx_ts;
                        resp_tx_ts_32 = (uint32)resp_tx_ts;
                        final_rx_ts_32 = (uint32)final_rx_ts;
                        Ra = (double)(resp_rx_ts - poll_tx_ts);								//174509902		176560819
                        Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);						//183500240		185383428
                        Da = (double)(final_tx_ts - resp_rx_ts);           					//183500466		185500621
                        Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);      					//174509773		176561074
                        tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));		//-22.02126944	460360.6895

                        tof = tof_dtu * DWT_TIME_UNITS;
                        distance = tof * SPEED_OF_LIGHT;
                        distance = distance - dwt_getrangebias(config.chan,(float)distance, config.prf);
                        temp = (int)(distance*100);
                        distance_msg[10] = temp/100;
                        distance_msg[11] = temp%100;
                        distance_msg[12] = anthor_index;

                        distance_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                        distance_msg[ALL_MSG_TAG_IDX] = tag_index;
                        dwt_writetxdata(sizeof(distance_msg), distance_msg, 0);
                        dwt_writetxfctrl(sizeof(distance_msg), 0);

                        /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
                         * set by dwt_setrxaftertxdelay() has elapsed. */
                        dwt_starttx(DWT_START_TX_IMMEDIATE );
						while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
						{ };
                    }
                }
                else
                {
                    /* Clear RX error events in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                }
            }
		}
//		vTaskDelay(10);
	}
}

static void Anchor_Array_Init(void)
{
    int anchor_index = 0;
    for(anchor_index = 0; anchor_index < ANCHOR_MAX_NUM; anchor_index++)
    {
        Anthordistance[anchor_index] = 0;
        Anthordistance_count[anchor_index] = 0;
    }
}

void reset_DW1000(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Enable GPIO used for DW1000 reset
	// done by main.c(MX_GPIO_Init) 

	//drive the RSTn pin low
	HAL_GPIO_WritePin(DW1000_RST_PORT, DW1000_RST_PIN, GPIO_PIN_RESET);
	
	//put the pin back to tri-state ... as input
	GPIO_InitStruct.Pin = DW1000_RST_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DW1000_RST_PORT, &GPIO_InitStruct);

  	vTaskDelay(2);
}

static void SPI_ChangeRate(SPI_TypeDef *SPIx, uint16_t scalingfactor)
{
	uint16_t tmpreg = 0;

	/* Get the SPIx CR1 value */
	tmpreg = SPIx->CR1;

	/*clear the scaling bits*/
	tmpreg &= 0xFFC7;

	/*set the scaling bits*/
	tmpreg |= scalingfactor;

	/* Write to SPIx CR1 */
	SPIx->CR1 = tmpreg;
}

void spi_set_rate_low(void)
{
    SPI_ChangeRate(DW1000_SPI_Handle.Instance, SPI_BAUDRATEPRESCALER_32);
}

void spi_set_rate_high(void)
{
    SPI_ChangeRate(DW1000_SPI_Handle.Instance, SPI_BAUDRATEPRESCALER_4);
}

static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}
