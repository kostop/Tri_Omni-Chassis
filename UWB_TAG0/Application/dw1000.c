#include "dw1000.h"
#include <stdlib.h>
#include <string.h>
#include "math.h"
#include "usart.h"

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

#define RNG_DELAY_MS 2

#define TX_ANT_DLY 0
#define RX_ANT_DLY 32950

#define POLL_TX_TO_RESP_RX_DLY_UUS 190
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3700
#define RESP_RX_TIMEOUT_UUS 8000


#define UUS_TO_DWT_TIME 65536

#define ANCHOR_REFRESH_COUNT 5

#define TAG
#define TAG_MAX_NUM 3
#define TAG_ID 0x0F
#define MASTER_TAG 0x0F
#define MAX_SLAVE_TAG 0x02
#define SLAVE_TAG_START_INDEX 0x01

#define ANCHOR_MAX_NUM 3

//主从空闲判断
static uint8 Semaphore_Enable = 0 ;
static uint8 Waiting_TAG_Release_Semaphore = 0;
static  uint8 Semaphore[MAX_SLAVE_TAG];

//接收标志位
static uint32 status_reg = 0;

//标签id索引
static uint8 tag_index = 0;
//数据包累加位
static uint8 frame_seq_nb = 0;
static int8 frame_len = 0;
//接收数据包
#define RX_BUF_LEN 30
static uint8 rx_buffer[RX_BUF_LEN];

//通信时间戳
static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;

static uint16_t dis_tasndsfsdf;

//距离信息
static int Anthordistance[ANCHOR_MAX_NUM];
static int Anthordistance_count[ANCHOR_MAX_NUM];

//通信数据包
static uint8 tx_poll_msg[] =  {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 rx_resp_msg[] =  {0x41, 0x88, 0, 0x0, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 tx_final_msg[] = {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 distance_msg[] = {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xAA, 0, 0,0, 0, 0, 0};
/*
controller_msg		遥控器数据包
id					3
state				10
x<<8				11
x		m			12
y<<8				13
y		m			14
yaw<<8				15
yaw		rad			16
*/
static uint8 controller_msg[] = {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xAE, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 car_msg[] = {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xAF, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static uint8 Semaphore_Release[] = {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0, 0};
static uint8 Tag_Statistics[] = {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE1, 0, 0, 0};
static uint8 Master_Release_Semaphore[] = {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE2, 0, 0, 0};
static uint8 Tag_Statistics_response[] = {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE3, 0, 0, 0};
static uint8 Master_Release_Semaphore_comfirm[] = {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE4, 0, 0, 0};

typedef struct
{
	uint8_t state;
	float x;
	float y;
	float yaw;
}car_measure_t;
static car_measure_t target_car_measure;
static car_measure_t car_measure;

typedef struct
{
	float x;
	float y;
	float yaw;
}pos_t;

typedef struct
{
	float vx;
	float vy;
	float vyaw;
}line_t;


typedef struct
{
	pos_t anthor_pos[2];			//AB
	pos_t anthor_tag_pos;			//C
	pos_t tag_pos[TAG_MAX_NUM];		//D
}pos_para_t;

typedef struct
{
	float anthor_dis;						//b
	float anthor_tag_dis;					//a
	float tag_dis[TAG_MAX_NUM][3];			//edc
}dis_para_t;

typedef struct
{
	dis_para_t	dis;
	pos_para_t	pos;
}dw1000_para_t;

dw1000_para_t dw1000_para;

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



static void car_task(void);
static void Tag_Measure_Dis(void);
static void Semaphore_Init(void);
static int Sum_Tag_Semaphore_request(void);
static void distance_mange(void);

static void reset_DW1000(void);
static void spi_set_rate_low(void);
static void spi_set_rate_high(void);

static int filter(int input, int fliter_idx );
static uint64 get_rx_timestamp_u64(void);
static uint64 get_tx_timestamp_u64(void);
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);
static void final_msg_set_ts(uint8 *ts_field, uint64 ts);
static float parseSignedFloat(uint8_t highByte, uint8_t lowByte, float scale);
static void floatToSignedBytes(float value, float scale, uint8_t* highByte, uint8_t* lowByte);

void dw_task(void const * argument)
{
    reset_DW1000();
    spi_set_rate_low();
    while(dwt_initialise(DWT_LOADUCODE) == -1)
	{
		//init fail
	}
    spi_set_rate_high();
    dwt_configure(&config);
    dwt_setleds(1);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
	
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    if(TAG_ID ==  MASTER_TAG)
    {
        Semaphore_Enable = 1 ;
        Semaphore_Init();
        Waiting_TAG_Release_Semaphore = 0;
    }
    else
    {
        Semaphore_Enable = 0 ;
    }
	while(1)
	{
		if(Semaphore_Enable == 1)
        {
            Tag_Measure_Dis();
			car_task();
            Semaphore_Enable = 0 ;

            if(TAG_ID != MASTER_TAG)
            {
                //send release semaphore to master tag
                Semaphore_Release[ALL_MSG_SN_IDX] = frame_seq_nb;
                Semaphore_Release[ALL_MSG_TAG_IDX] = TAG_ID;
                dwt_writetxdata(sizeof(Semaphore_Release), Semaphore_Release, 0);
                dwt_writetxfctrl(sizeof(Semaphore_Release), 0);

                dwt_starttx(DWT_START_TX_IMMEDIATE );
                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                { };
            }
        }

        if(TAG_ID == MASTER_TAG)//master  tag
        {
            //statistics tag
            if(Sum_Tag_Semaphore_request() == 0)
            {
                for(tag_index = SLAVE_TAG_START_INDEX; tag_index <MAX_SLAVE_TAG; tag_index++)
                {
                    Tag_Statistics[ALL_MSG_SN_IDX] = 0;
                    Tag_Statistics[ALL_MSG_TAG_IDX] = tag_index;
                    dwt_writetxdata(sizeof(Tag_Statistics), Tag_Statistics, 0);
                    dwt_writetxfctrl(sizeof(Tag_Statistics), 0);
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

                        if(rx_buffer[ALL_MSG_TAG_IDX] == tag_index)
                        {
                            uint8 temp = rx_buffer[ALL_MSG_TAG_IDX] ;
                            rx_buffer[ALL_MSG_TAG_IDX] =0;
                            if (memcmp(rx_buffer, Tag_Statistics_response, ALL_MSG_COMMON_LEN) == 0)
                            {
                                Semaphore[temp] = 1;
                            }
                        }
                    }
                    else
                    {
                        /* Clear RX error events in the DW1000 status register. */
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                        //GPIO_SetBits(GPIOA,GPIO_Pin_1);
                    }
                }
                //print all the tags in network
                for(tag_index = SLAVE_TAG_START_INDEX; tag_index <MAX_SLAVE_TAG; tag_index++)
                {
                    if(Semaphore[tag_index] == 1)
                    {
                        // printf("Tag%d In NetWork!\r\n",tag_index);
                    }
                }
            }
            //pick one tag ,send Semaphore message
            //release to specific tag(TAG ID)
            //master tag send release signal,and the specific tag send comfirm message
            if(Waiting_TAG_Release_Semaphore == 0 && Sum_Tag_Semaphore_request() != 0)
            {
                Semaphore[0] = 0;//slave tag must not use tag_id = 0x00!!
                for(tag_index = SLAVE_TAG_START_INDEX; tag_index <MAX_SLAVE_TAG; tag_index++)
                {
                    if(Semaphore[tag_index] == 1)
                    {
                        // printf("Release Semaphore to Tag%d!\r\n",tag_index);
                        // dwt_setrxtimeout(0);

                        dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
                        dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
                        Master_Release_Semaphore[ALL_MSG_SN_IDX] = 0;
                        Master_Release_Semaphore[ALL_MSG_TAG_IDX] = tag_index;
                        dwt_writetxdata(sizeof(Master_Release_Semaphore), Master_Release_Semaphore, 0);
                        dwt_writetxfctrl(sizeof(Master_Release_Semaphore), 0);
                        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

                        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
                        { };

                        if (status_reg & SYS_STATUS_RXFCG)
                        {
                            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
                            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                            if (frame_len <= RX_BUF_LEN)
                            {
                                dwt_readrxdata(rx_buffer, frame_len, 0);
                            }
                            rx_buffer[ALL_MSG_SN_IDX] = 0;

                            if(rx_buffer[ALL_MSG_TAG_IDX] == tag_index)
                            {
                                rx_buffer[ALL_MSG_TAG_IDX] = 0;
                                // USART_puts(rx_buffer,frame_len);
                                if (memcmp(rx_buffer, Master_Release_Semaphore_comfirm, ALL_MSG_COMMON_LEN) == 0)
                                {
                                    //if the tag recive a semaphore, wait release remaphore
                                    Waiting_TAG_Release_Semaphore ++;
                                    break;//only release one semphore once
                                }
                            }
                        }
                        else//the tag may leave net,clear semaphore
                        {
                            Semaphore[tag_index] = 0 ;
                            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                        }
                    }
                }
            }

            if(Waiting_TAG_Release_Semaphore == 0 )
            {
                // GPIO_SetBits(GPIOA,GPIO_Pin_2);GPIO_SetBits(GPIOA,GPIO_Pin_1);
            }
            //Master tag waitting for specific tag Semaphore Release message
            if( Waiting_TAG_Release_Semaphore >0)
            {
                //  printf("Waiting for Release Semaphore!\r\n");
                dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS*5);//about 10ms,need adjust!!
                dwt_rxenable(0);
                while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
                { };

                if (status_reg & SYS_STATUS_RXFCG)
                {
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
                    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
                    if (frame_len <= RX_BUFFER_LEN)
                    {
                        dwt_readrxdata(rx_buffer, frame_len, 0);
                    }

                    rx_buffer[ALL_MSG_SN_IDX] = 0;
                    uint8 temp=rx_buffer[ALL_MSG_TAG_IDX] ;
                    rx_buffer[ALL_MSG_TAG_IDX] = 0;
                    if (memcmp(rx_buffer, Semaphore_Release, ALL_MSG_COMMON_LEN) == 0)
                    {
                        if(Semaphore[temp] == 1)
                        {
                            Semaphore[temp] = 0 ;
                            if(Waiting_TAG_Release_Semaphore > 0 )
                            {
                                Waiting_TAG_Release_Semaphore --;
                            }
                        }
                    }
                }
                else
                {
                    //maybe the tag leave network
                    if(Waiting_TAG_Release_Semaphore > 0)
                    {
                        Waiting_TAG_Release_Semaphore--;
                        Semaphore[tag_index] = 0 ;
                    }
                    /* Clear RX error events in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                }
            }
            if(Sum_Tag_Semaphore_request() == 0)
            {
                Semaphore_Enable = 1 ;
                Waiting_TAG_Release_Semaphore= 0;
            }
        }
        else  //slave tags
        {
            dwt_setrxtimeout(0);
            dwt_rxenable(0);

            /* Poll for reception of a frame or error/timeout. See NOTE 7 below. */
            while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
            { };

            if (status_reg & SYS_STATUS_RXFCG)
            {
                /* Clear good RX frame event in the DW1000 status register. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG|SYS_STATUS_TXFRS);//clear rx & tx flag at the same time

                /* A frame has been received, read it into the local buffer. */
                frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
                if (frame_len <= RX_BUFFER_LEN)
                {
                    dwt_readrxdata(rx_buffer, frame_len, 0);
                }
                rx_buffer[ALL_MSG_SN_IDX] = 0;
                if(rx_buffer[ALL_MSG_TAG_IDX] == TAG_ID)
                {
                    rx_buffer[ALL_MSG_TAG_IDX] = 0;
                    if (memcmp(rx_buffer, Tag_Statistics, ALL_MSG_COMMON_LEN) == 0)
                    {
                        //GPIO_SetBits(GPIOA,GPIO_Pin_3);
                        Tag_Statistics_response[ALL_MSG_SN_IDX] = frame_seq_nb;
                        Tag_Statistics_response[ALL_MSG_TAG_IDX] = TAG_ID;
                        dwt_writetxdata(sizeof(Tag_Statistics_response), Tag_Statistics_response, 0);
                        dwt_writetxfctrl(sizeof(Tag_Statistics_response), 0);
                        dwt_starttx(DWT_START_TX_IMMEDIATE );
                    }

                    if (memcmp(rx_buffer, Master_Release_Semaphore, ALL_MSG_COMMON_LEN) == 0)
                    {
                        Master_Release_Semaphore_comfirm[ALL_MSG_SN_IDX] = frame_seq_nb;
                        Master_Release_Semaphore_comfirm[ALL_MSG_TAG_IDX] = TAG_ID;
                        dwt_writetxdata(sizeof(Master_Release_Semaphore_comfirm), Master_Release_Semaphore_comfirm, 0);
                        dwt_writetxfctrl(sizeof(Master_Release_Semaphore_comfirm), 0);

                        dwt_starttx(DWT_START_TX_IMMEDIATE);
                        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                        { };

                        Semaphore_Enable = 1;
                    }
                }
            }
            else
            {
                /* Clear RX error events in the DW1000 status register. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
            }
        }
		
//		vTaskDelay(5);
    }
}

static void car_task(void)
{
	uint8_t frame_len;
	dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
	dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
	
	car_msg[3] = TAG_ID;
    floatToSignedBytes(car_measure.x, 100.0f, &car_msg[11], &car_msg[12]);
    floatToSignedBytes(car_measure.y, 100.0f, &car_msg[13], &car_msg[14]);
    floatToSignedBytes(car_measure.yaw, 100.0f, &car_msg[15], &car_msg[16]);
	
	dwt_writetxdata(sizeof(car_msg), car_msg, 0);
	dwt_writetxfctrl(sizeof(car_msg), 0);
	dwt_starttx(DWT_START_TX_IMMEDIATE| DWT_RESPONSE_EXPECTED);
	
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
		
		if(rx_buffer[3] == TAG_ID)
		{
            rx_buffer[2] = 0;
            rx_buffer[3] = 0;
			if (memcmp(rx_buffer, controller_msg, ALL_MSG_COMMON_LEN) == 0)
			{
				target_car_measure.state = rx_buffer[10];
				target_car_measure.x = parseSignedFloat(rx_buffer[11], rx_buffer[12], 100.0f);
				target_car_measure.y = parseSignedFloat(rx_buffer[13], rx_buffer[14], 100.0f);
				target_car_measure.yaw = parseSignedFloat(rx_buffer[15], rx_buffer[16], 100.0f);
			}
		}
	}
}

void Tag_Measure_Dis(void)
{
    uint8 dest_anthor = 0,frame_len = 0;
    float final_distance = 0;
    for(dest_anthor = 0 ;  dest_anthor<ANCHOR_MAX_NUM; dest_anthor++)
    {
        dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
        dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
        /* Write frame data to DW1000 and prepare transmission. See NOTE 7 below. */
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        tx_poll_msg[ALL_MSG_TAG_IDX] = TAG_ID;//基站收到标签的信息，里面有TAG_ID,在基站回复标签的时候，也需要指定TAG_ID,只有TAG_ID一致才做处理

        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0);

        /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
         * set by dwt_setrxaftertxdelay() has elapsed. */
        dwt_starttx(DWT_START_TX_IMMEDIATE| DWT_RESPONSE_EXPECTED);

        dwt_rxenable(0);//这个后加的，默认tx后应该自动切换rx，但是目前debug 发现并没有自动打开，这里强制打开rx

        /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
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

            if(rx_buffer[ALL_MSG_TAG_IDX] != TAG_ID)//检测TAG_ID
                continue;
            rx_buffer[ALL_MSG_TAG_IDX] = 0;

            /* As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
            rx_buffer[ALL_MSG_SN_IDX] = 0;

            if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                uint32 final_tx_time;

                /* Retrieve poll transmission and response reception timestamp. */
                poll_tx_ts = get_tx_timestamp_u64();
                resp_rx_ts = get_rx_timestamp_u64();

                /* Compute final message transmission time. See NOTE 9 below. */
                final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(final_tx_time);

                /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
                final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY;

                /* Write all timestamps in the final message. See NOTE 10 below. */
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

                /* Write and send final message. See NOTE 7 below. */
                tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                tx_final_msg[ALL_MSG_TAG_IDX] = TAG_ID;
                dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
                dwt_writetxfctrl(sizeof(tx_final_msg), 0);

                dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED );
				dwt_rxenable(0);

                while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
                { };

                /* Increment frame sequence number after transmission of the poll message (modulo 256). */
                
				if (status_reg & SYS_STATUS_RXFCG)
                {
                    /* Clear good/fail RX frame event in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
                    /* A frame has been received, read it into the local buffer. */
                    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                    if (frame_len <= RX_BUF_LEN)
                    {
                        dwt_readrxdata(rx_buffer, frame_len, 0);
                    }

                    if(rx_buffer[ALL_MSG_TAG_IDX] != TAG_ID)
                        continue;
                    rx_buffer[ALL_MSG_TAG_IDX] = 0;

                    /*As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
                    rx_buffer[ALL_MSG_SN_IDX] = 0;

                    if (memcmp(rx_buffer, distance_msg, ALL_MSG_COMMON_LEN) == 0)
                    {
						if(rx_buffer[12]==0x6F)
							dis_tasndsfsdf = (rx_buffer[10]<<8|rx_buffer[11]);
						else
							Anthordistance[rx_buffer[12]] +=(rx_buffer[10]*1000 + rx_buffer[11]*10);
                        // final_distance = rx_buffer[10] + (float)rx_buffer[11]/100;
                        Anthordistance_count[rx_buffer[12]] ++;
                        {
                            int Anchor_Index = 0;
                            while(Anchor_Index < ANCHOR_MAX_NUM)
                            {
                                if(Anthordistance_count[Anchor_Index] >= ANCHOR_REFRESH_COUNT)
                                {
                                    distance_mange();
                                    Anchor_Index = 0;
                                    //clear all
                                    while(Anchor_Index < ANCHOR_MAX_NUM)
                                    {
                                        Anthordistance_count[Anchor_Index] = 0;
                                        Anthordistance[Anchor_Index] = 0;
                                        Anchor_Index++;
                                    }
                                    break;
                                }
                                Anchor_Index++;
                            }
                        }
                    }
                }
                else
                {
                    /* Clear RX error events in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                }
            }
        }
        else
        {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
        /* Execute a delay between ranging exchanges. */
        deca_sleep(RNG_DELAY_MS);
        frame_seq_nb++;
    }

}

static float y_before_filter, x_before_filter;
static float cos_data;
static void tag_pos_count(dw1000_para_t *dw_para_point)
{
	float a, b, c, d, e;
	a = dw_para_point->dis.anthor_tag_dis;
	b = dw_para_point->dis.anthor_dis;
	c = dw_para_point->dis.tag_dis[0][2];
	d = dw_para_point->dis.tag_dis[0][1];
	e = dw_para_point->dis.tag_dis[0][0];
	
	if(a==0&&b==0&&c==0&&d==0&&e==0)
		return;
	cos_data = (pow(a,2)+pow(e,2)-pow(d,2))/(2.0f*a*e);
	x_before_filter = e*sqrt(1.0f-pow(cos_data,2));
	y_before_filter = e*cos_data;
	
//	first_order_filter_cali(&dw_para_point->pos_filter[0], x_before_filter);
//	first_order_filter_cali(&dw_para_point->pos_filter[1], y_before_filter);
	dw_para_point->pos.tag_pos[0].x = x_before_filter;
	dw_para_point->pos.tag_pos[0].y = y_before_filter;
}

static uint8_t send_msg1[11];
static uint8_t position_data_flag = 0;
static void compute_angle_send_to_anthor0(int distance1, int distance2,int distance3, int distance4)
{
	dw1000_para.dis.tag_dis[0][0] = (float)distance1/1000.0f;
	dw1000_para.dis.tag_dis[0][1] = (float)distance2/1000.0f;
	dw1000_para.dis.tag_dis[0][2] = (float)distance3/1000.0f;
	dw1000_para.dis.anthor_tag_dis = (float)distance4/1000.0f;
	//计算当前底盘位置数据
	tag_pos_count(&dw1000_para);

	//头帧数据
	send_msg1[0] = 'm';
	send_msg1[1] = 'r';
	send_msg1[2] = target_car_measure.state;		//状态标志
	send_msg1[9] = 0x01;
	send_msg1[10] = 0x10;
	//速度控制模式
	if(	target_car_measure.state == CAR_NO_MOVE)
	{
		
		floatToSignedBytes(0, 100.0f, &send_msg1[3], &send_msg1[4]);
		floatToSignedBytes(0, 100.0f, &send_msg1[5], &send_msg1[6]);
		floatToSignedBytes(0, 100.0f, &send_msg1[7], &send_msg1[8]);
	}
	//位置控制模式
	else if(target_car_measure.state == CAR_POSITION_MOVE||
			target_car_measure.state == CAR_CALIBRATE||
			target_car_measure.state == CAR_NORMAL_MOVE)
	{
		if(position_data_flag == 0)
		{
			floatToSignedBytes(target_car_measure.x, 100.0f, &send_msg1[3], &send_msg1[4]);
			floatToSignedBytes(target_car_measure.y, 100.0f, &send_msg1[5], &send_msg1[6]);
			floatToSignedBytes(target_car_measure.yaw, 100.0f, &send_msg1[7], &send_msg1[8]);
			if(target_car_measure.state == CAR_POSITION_MOVE)
			{
				send_msg1[2]=3;
				position_data_flag = 1;
			}
		}
		else if(position_data_flag == 1)
		{
			floatToSignedBytes(dw1000_para.pos.tag_pos[0].x, 100.0f, &send_msg1[3], &send_msg1[4]);
			floatToSignedBytes(dw1000_para.pos.tag_pos[0].y, 100.0f, &send_msg1[5], &send_msg1[6]);
			floatToSignedBytes(dw1000_para.pos.tag_pos[0].yaw, 100.0f, &send_msg1[7], &send_msg1[8]);
			position_data_flag = 0;
			send_msg1[2]=4;
		}
	}
//	usart_transmit_data(send_msg1, 11);
//	USART_puts((char*)send_msg1,11);
	HAL_UART_Transmit(&huart1, send_msg1, 11, 10);
}


static void distance_mange(void)
{
    {
        int Anchor_Index = 0;
        while(Anchor_Index < ANCHOR_MAX_NUM)
        {
            if(Anthordistance_count[Anchor_Index] > 0 )
            {
                Anthordistance[Anchor_Index] =filter((int)(Anthordistance[Anchor_Index]/Anthordistance_count[Anchor_Index]),Anchor_Index);
                // Anthordistance[Anchor_Index] =(int)(Anthordistance[Anchor_Index]/Anthordistance_count[Anchor_Index]);
            }
            Anchor_Index++;
        }
    }
    compute_angle_send_to_anthor0(Anthordistance[0], Anthordistance[1],Anthordistance[2], 0);
}

#define Filter_N 5  //max filter use in this system
#define Filter_D 5  //each filter contain "Filter_D" data
int Value_Buf[Filter_N][Filter_D]= {0};
int filter_index[Filter_N] = {0};
static int filter(int input, int fliter_idx )
{
    char count = 0;
    int sum = 0;
    if(input > 0)
    {
        Value_Buf[fliter_idx][filter_index[fliter_idx]++]=input;
        if(filter_index[fliter_idx] == Filter_D) filter_index[fliter_idx] = 0;

        for(count = 0; count<Filter_D; count++)
        {
            sum += Value_Buf[fliter_idx][count];
        }
        return (int)(sum/Filter_D);
    }
    else
    {
        for(count = 0; count<Filter_D; count++)
        {
            sum += Value_Buf[fliter_idx][count];
        }
        return (int)(sum/Filter_D);
    }

}

static int Sum_Tag_Semaphore_request(void)
{
    int tag_index = 0 ;
    int sum_request = 0;
    for(tag_index = SLAVE_TAG_START_INDEX; tag_index <MAX_SLAVE_TAG; tag_index++)
    {
        sum_request+=Semaphore[tag_index];
    }
    return sum_request;
}

static void Semaphore_Init(void)
{
    int tag_index = 0 ;
    for(tag_index = 0; tag_index <MAX_SLAVE_TAG; tag_index++)
    {
        Semaphore[tag_index]  = 0;
    }
}

static void reset_DW1000(void)
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

static void spi_set_rate_low(void)
{
    SPI_ChangeRate(DW1000_SPI_Handle.Instance, SPI_BAUDRATEPRESCALER_32);
}

static void spi_set_rate_high(void)
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

static void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}

static float parseSignedFloat(uint8_t highByte, uint8_t lowByte, float scale) {
    int16_t value = (highByte << 8) | lowByte;
    return (float)value / scale;
}

static void floatToSignedBytes(float value, float scale, uint8_t* highByte, uint8_t* lowByte) {
    int16_t fixedPoint = (int16_t)(value * scale);
    *highByte = (fixedPoint >> 8) & 0xFF;
    *lowByte = fixedPoint & 0xFF;
}

