/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/mm.h> 

#include <zephyr/logging/log.h>
#define LOG_MODULE_NAME tests
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#if defined(NRF54L15_XXAA) || defined(NRF54L10_XXAA) || defined(NRF54L05_XXAA)
#include <hal/nrf_clock.h>
#endif /* defined(NRF54L15_XXAA) */

#if defined(CONFIG_CLOCK_CONTROL_NRF2)
#include <hal/nrf_lrcconf.h>
#endif

#include "hal/nrf_gpio.h"
#include "hal/nrf_radio.h"

#if defined(CONFIG_CLOCK_CONTROL_NRF)
int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		LOG_ERR("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do {
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res) {
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
	} while (err);

#if defined(NRF54L15_XXAA) || defined(NRF54L10_XXAA) || defined(NRF54L05_XXAA)
	/* MLTPAN-20 */
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
#endif /* defined(NRF54L15_XXAA) */

	LOG_DBG("HF clock started");
	return 0;
}
#endif
static uint32_t bytewise_bitswap(uint32_t inp);

static uint32_t swap_bits(uint32_t inp)
{
    uint32_t i;
    uint32_t retval = 0;

    inp = (inp & 0x000000FFUL);

    for (i = 0; i < 8; i++)
    {
        retval |= ((inp >> i) & 0x01) << (7 - i);
    }

    return retval;
}


static uint32_t bytewise_bitswap(uint32_t inp)
{
      return (swap_bits(inp >> 24) << 24)
           | (swap_bits(inp >> 16) << 16)
           | (swap_bits(inp >> 8) << 8)
           | (swap_bits(inp));
}

#define RADIO_PKT_LEN 8 // Radio packet length in bytes
// Radio configuration for 2.4GHz proprietary protocol
static uint8_t radio_packet[RADIO_PKT_LEN] = {0}; // Example packet data
static volatile bool radio_ready = false;
static bool m_is_tx_dev = false; // Flag to indicate if we are in TX mode

uint32_t m_tx_cnts = 0; // Counter for TX packets
uint32_t m_rx_cnts = 0; // Counter for RX packets

uint32_t m_1min_rx_cnts = 0; // 1 minute RX packet counter
uint32_t m_1min_rx_cnts_theory = 0; // 1 minute RX packet counter

uint32_t m_rx_total = 0; // Total RX packet counter
uint32_t m_rx_total_theory = 0; // Total RX packet counter theoretical value

// 跳频功能相关变量
#define HOP_CHANNELS_NUM 8
static const uint8_t hop_channels[HOP_CHANNELS_NUM] = {
    2,   
    14,  
    36,  
    56,  
    66,  
    78,  
    0,  
    22,  
    // 1,   
    // 11,  
    // 21,  
    // 31,  
    // 41,  
    // 51,  
    // 61,  
    // 71   
};

static uint8_t current_channel_index = 0;
static bool rx_synchronized = false;
static volatile bool rx_channel_timeout = false;
static volatile bool m_rx_ok = false;

void radio_start_rx(void);

// 跳频功能函数
static inline void radio_hop_to_next_channel(void)
{
	current_channel_index++;
    current_channel_index = current_channel_index % HOP_CHANNELS_NUM; // 确保索引循环在范围内
	// printf("Hop to channel %d (freq: %d MHz)\n", current_channel_index, 2400 + hop_channels[current_channel_index]);

	*(volatile uint32_t *)((uint8_t *)(NRF_RADIO) +  0x70C) &= ~(1 << 31);
	 nrf_radio_frequency_set(NRF_RADIO, 2400 + hop_channels[current_channel_index]);
	*(volatile uint32_t *)((uint8_t *)(NRF_RADIO) +  0x07C) = 1;
   
}


// RX信道扫描定时器 - 用于未同步时的10ms信道切换
#define RX_SCAN_TIMER  NRF_TIMER10
#define RX_SCAN_TIMER_IRQ  TIMER10_IRQHandler
#define RX_SCAN_TIMER_IRQ_IDX  TIMER10_IRQn

ISR_DIRECT_DECLARE(RX_SCAN_TIMER_IRQ)
{
    RX_SCAN_TIMER->EVENTS_COMPARE[0] = 0;
    // printf("RX Scan Timer IRQ: Switching channel\n");
	rx_synchronized = false;
    if (!m_is_tx_dev && !rx_synchronized) {
        // 未同步的RX设备，切换到下一个信道
        rx_channel_timeout = true;
		
    }
    
    return 0;
}

void rx_scan_timer_init(void)
{
    RX_SCAN_TIMER->TASKS_CLEAR = 1;
    RX_SCAN_TIMER->MODE = 0;          // Timer mode
    RX_SCAN_TIMER->BITMODE = 3;       // 32-bit mode  
    RX_SCAN_TIMER->PRESCALER = 5;     // 1MHz (32MHz / 2^5)
    RX_SCAN_TIMER->CC[0] = 1000;     // 1000us = 1ms at 1MHz
    RX_SCAN_TIMER->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
	RX_SCAN_TIMER->SHORTS = TIMER_SHORTS_COMPARE0_STOP_Msk; // Auto-clear on compare
    
    IRQ_DIRECT_CONNECT(RX_SCAN_TIMER_IRQ_IDX, 0, RX_SCAN_TIMER_IRQ, 0);
    NVIC_ClearPendingIRQ(RX_SCAN_TIMER_IRQ_IDX);
    irq_enable(RX_SCAN_TIMER_IRQ_IDX);
}

static inline void rx_scan_timer_start(uint32_t cc_value)
{
    RX_SCAN_TIMER->EVENTS_COMPARE[0] = 0;
    RX_SCAN_TIMER->TASKS_CLEAR = 1;
    RX_SCAN_TIMER->CC[0] = cc_value;
    RX_SCAN_TIMER->TASKS_START = 1;
}

void rx_scan_timer_stop(void)
{
    RX_SCAN_TIMER->TASKS_STOP = 1;
}
// Radio interrupt service routine
ISR_DIRECT_DECLARE(radio_isr)
{
    // Handle READY event
    if (NRF_RADIO->EVENTS_READY) {
        NRF_RADIO->EVENTS_READY = 0;
		// printf("EVENTS_READY\n");
		m_rx_ok = false;
    }
    
    // Handle END event
    if (NRF_RADIO->EVENTS_END) {
        NRF_RADIO->EVENTS_END = 0;
        // printf("EVENTS_END\n");
        if (!m_is_tx_dev) {
            // RX completed
            m_rx_cnts++;

			rx_scan_timer_start(160);

            if (NRF_RADIO->CRCSTATUS == 1) {
				m_rx_ok = true;
                if (!rx_synchronized) {
                    rx_synchronized = true;
                }               
            }

			// printf("RX Packet: ");
			// for (int i = 0; i < 8; i++) {
			// 	printf("%02X ", radio_packet[i]);
			// }
			// printf("\n");

        } else {
            // TX completed - prepare next packet
            m_tx_cnts++;
            
            // TX设备每次发送后跳频
            radio_hop_to_next_channel();
            
            // Update packet data for next transmission (DPPI will trigger next TX automatically)
            radio_packet[0]++; // 
            radio_packet[1]++; // 
            radio_packet[2]++; // 
            radio_packet[3]++; //
            
            // printf("TX hop to channel %d (freq: %d MHz), packet #%d\n", 
            //        current_channel_index, 2400 + hop_channels[current_channel_index], packet_counter);
        }
        
        // Disable radio
        // NRF_RADIO->TASKS_DISABLE = 1;
    }
    
    // Handle DISABLED event
    if (NRF_RADIO->EVENTS_DISABLED) {
        NRF_RADIO->EVENTS_DISABLED = 0;
		// printf("EVENTS_DISABLED\n");
        
		if (!m_is_tx_dev) {
			if(m_rx_ok){
				radio_hop_to_next_channel();
				// printf("RX hop to channel %d (freq: %d MHz)\n", 
                //        current_channel_index, 2400 + hop_channels[current_channel_index]);
			}
		}
    }

	if(NRF_RADIO->EVENTS_RXREADY) {
		NRF_RADIO->EVENTS_RXREADY = 0;		
		// printf("EVENTS_RXREADY\n");
	}
    
    return 0;
}

void radio_init(void)
{
    // Configure radio for 2.4GHz proprietary mode
    nrf_radio_mode_set(NRF_RADIO, NRF_RADIO_MODE_NRF_4MBIT_BT_0_4);
    
    // 初始化频道为第0个信道
    current_channel_index = 0;
    nrf_radio_frequency_set(NRF_RADIO, 2400 + hop_channels[current_channel_index]);
    nrf_radio_txpower_set(NRF_RADIO, NRF_RADIO_TXPOWER_POS8DBM);

	// Radio address config
    NRF_RADIO->PREFIX0 =
        ((uint32_t)swap_bits(0xC3) << 24) // Prefix byte of address 3 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC2) << 16) // Prefix byte of address 2 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC1) << 8)  // Prefix byte of address 1 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC0) << 0); // Prefix byte of address 0 converted to nRF24L series format

    NRF_RADIO->PREFIX1 =
        ((uint32_t)swap_bits(0xC7) << 24) // Prefix byte of address 7 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC6) << 16) // Prefix byte of address 6 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC4) << 0); // Prefix byte of address 4 converted to nRF24L series format

    NRF_RADIO->BASE0 = bytewise_bitswap(0x01234567UL);  // Base address for prefix 0 converted to nRF24L series format
    NRF_RADIO->BASE1 = bytewise_bitswap(0x89ABCDEFUL);  // Base address for prefix 1-7 converted to nRF24L series format

    NRF_RADIO->TXADDRESS   = 0x00UL;  // Set device address 0 to use when transmitting
    NRF_RADIO->RXADDRESSES = 0x01UL;  // Enable device address 0 to use to select which addresses to receive

    // Packet configuration
    NRF_RADIO->PCNF0 = (0     << RADIO_PCNF0_S1LEN_Pos) |
                       (0     << RADIO_PCNF0_S0LEN_Pos) |
                       (0 << RADIO_PCNF0_LFLEN_Pos); //lint !e845 "The right argument to operator '|' is certain to be 0"

    // Packet configuration
    NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos) |
                       (RADIO_PCNF1_ENDIAN_Big       << RADIO_PCNF1_ENDIAN_Pos)  |
                       (4   << RADIO_PCNF1_BALEN_Pos)   |
                       (RADIO_PKT_LEN         << RADIO_PCNF1_STATLEN_Pos) |  // 
                       (RADIO_PKT_LEN       << RADIO_PCNF1_MAXLEN_Pos); //lint !e845 "The right argument to operator '|' is certain to be 0"

    // CRC Config
    NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos); // Number of checksum bits
    if ((NRF_RADIO->CRCCNF & RADIO_CRCCNF_LEN_Msk) == (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos))
    {
        NRF_RADIO->CRCINIT = 0xFFFFUL;   // Initial value
        NRF_RADIO->CRCPOLY = 0x11021UL;  // CRC poly: x^16 + x^12^x^5 + 1
    }
    else if ((NRF_RADIO->CRCCNF & RADIO_CRCCNF_LEN_Msk) == (RADIO_CRCCNF_LEN_One << RADIO_CRCCNF_LEN_Pos))
    {
        NRF_RADIO->CRCINIT = 0xFFUL;   // Initial value
        NRF_RADIO->CRCPOLY = 0x107UL;  // CRC poly: x^8 + x^2^x^1 + 1
    }
    
    // Enable radio interrupts - use INTENSET00 for nRF54L
    NRF_RADIO->INTENSET00 = RADIO_INTENSET00_READY_Msk | 
                            RADIO_INTENSET00_END_Msk | 
                            RADIO_INTENSET00_DISABLED_Msk|
							RADIO_INTENSET00_RXREADY_Msk;
    
    // Set up radio interrupt
    IRQ_DIRECT_CONNECT(RADIO_0_IRQn, 0, radio_isr, 0);
    NVIC_ClearPendingIRQ(RADIO_0_IRQn);
    irq_enable(RADIO_0_IRQn);
    
    // Set packet pointer to TX buffer initially
    NRF_RADIO->PACKETPTR = (uint32_t)radio_packet;
    
	nrf_radio_fast_ramp_up_enable_set(NRF_RADIO, true); // Enable fast ramp-up for better performance
    radio_ready = true;
    printf("Radio initialized for 2.4GHz proprietary mode with interrupts\n");
}

// New timer interrupt for radio TX every 100ms - 使用TIMER10域与RADIO在同一域
#define RADIO_TIMER  NRF_TIMER10
#define RADIO_TIMER_IRQ  TIMER10_IRQHandler
#define RADIO_TIMER_IRQ_IDX  TIMER10_IRQn

//============================================================================================================================//
#define USR_SPIM  NRF_SPIM21
static void spi20_init(void){
	USR_SPIM->CONFIG = (SPIM_CONFIG_CPHA_Leading << SPIM_CONFIG_CPHA_Pos) |
						 (SPIM_CONFIG_CPOL_ActiveHigh << SPIM_CONFIG_CPOL_Pos) |
						 (SPIM_CONFIG_ORDER_MsbFirst << SPIM_CONFIG_ORDER_Pos) ;
    USR_SPIM->PRESCALER = 4; //
	
	USR_SPIM->PSEL.CSN = (32 + 10); // P1.11
	USR_SPIM->PSEL.SCK = (32 + 11); // P1.15
	USR_SPIM->PSEL.MOSI = (32 + 12); // P1.14
	USR_SPIM->PSEL.MISO = (32 + 13); // P1.13
	
	USR_SPIM->DMA.TX.PTR = (uint32_t)radio_packet; // TX buffer address
	USR_SPIM->DMA.TX.MAXCNT = 8;

	USR_SPIM->ENABLE = SPIM_ENABLE_ENABLE_Enabled; // Enable SPIM20
}

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

	clocks_start();

	radio_init();
	if(NRF_UICR->OTP[0] == 0xABaBa )
	{
		// Initialize radio first
		m_is_tx_dev = true;
	}
	
	
	if(m_is_tx_dev){
		printf("Radio timer started - TX every 100ms via DPPI (hardware trigger, same domain)\n");
		printf("Running as TX device - DPPI triggered transmissions\n");
		// 配置 TIMER10 用于 100ms 周期的无线电发送 - 使用 DPPI 硬件触发，与RADIO在同一域
		#define RADIO_TIMER2TX_CH 0  // 使用 DPPI 通道 2 连接定时器和无线电
		#define RADIO_DISABLE_CH 1  // 使用 DPPI 通道 3 连接无线电禁用事件
		// 初始化第一个数据包
		
		RADIO_TIMER->TASKS_CLEAR = 1;
		RADIO_TIMER->MODE = 0;          // Timer mode
		RADIO_TIMER->BITMODE = 3;       // 32-bit mode  
		RADIO_TIMER->PRESCALER = 5;     // 1MHz (32MHz / 2^4)
		RADIO_TIMER->CC[0] = 125;    // 100ms = 100000 us at 1MHz
		RADIO_TIMER->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;  // Auto-clear on compare
		
		// 配置 TIMER10 发布 COMPARE[0] 事件到 DPPI 通道 2
		RADIO_TIMER->PUBLISH_COMPARE[0] = ((1ul<<31)|RADIO_TIMER2TX_CH); // 发布比较事件到 DPPI

		// 配置无线电订阅 DPPI 通道 2 的事件来触发 TXEN
		NRF_RADIO->SUBSCRIBE_TXEN = ((1ul<<31)|RADIO_TIMER2TX_CH);  // 订阅 DPPI 事件触发 TXEN

		NRF_RADIO->PUBLISH_DISABLED = ((1ul<<31)|RADIO_DISABLE_CH); // 发布 DISABLED 事件到 DPPI 通道 3

		// 启动定时器 - 无需中断处理，纯硬件触发
		RADIO_TIMER->EVENTS_COMPARE[0] = 0; // 清除比较事件

		// PPIB10-PPIB20
		NRF_DPPIC10->CHEN = (1<<RADIO_TIMER2TX_CH) | (1<<RADIO_DISABLE_CH);  // RADIO和TIMER10都在DPPI10域

		NRF_PPIB11->SUBSCRIBE_SEND[0] = ((1ul<<31)|RADIO_TIMER2TX_CH); // 订阅 DPPI 通道 2 的发送事件
		NRF_PPIB11->SUBSCRIBE_SEND[1] = ((1ul<<31)|RADIO_DISABLE_CH); // 订阅 DPPI 通道 2 的发送事件

		#define DPPI20_TX_IND_CH 0
		#define DPPI20_TX_DISABLE_CH 1

		NRF_PPIB21->PUBLISH_RECEIVE[0] = ((1ul<<31)|DPPI20_TX_IND_CH); // 发布 DPPI 通道 2 的接收事件
		NRF_PPIB21->PUBLISH_RECEIVE[1] = ((1ul<<31)|DPPI20_TX_DISABLE_CH); // 发布 DPPI 通道 2 的接收事件
		NRF_DPPIC20->CHEN = (1<<DPPI20_TX_IND_CH) | (1<<DPPI20_TX_DISABLE_CH);  // 启用 DPPI 通道

		#define TX_IND_PIN (32+9)  // P1.09
		#define TX_DISABLE_PIN (32+11)  // P1.11
		// 配置为输出
		nrf_gpio_cfg_output(TX_IND_PIN);
		nrf_gpio_pin_clear(TX_IND_PIN);  // 初始状态为低电平

		nrf_gpio_cfg_output(TX_DISABLE_PIN);
		nrf_gpio_pin_clear(TX_DISABLE_PIN);  // 初始状态为低电平

		// 配置 GPIOTE 通道用于切换输出
		NRF_GPIOTE20->CONFIG[0] = (3|((TX_IND_PIN)<<4)|(3<<16)); // TASK MODE, P1.09, TOGGLE
		NRF_GPIOTE20->SUBSCRIBE_OUT[0] = ((1<<31)|DPPI20_TX_IND_CH);      // 订阅 DPPI 通道事件

		NRF_GPIOTE20->CONFIG[1] = (3|((TX_DISABLE_PIN)<<4)|(3<<16)); // TASK MODE, P1.11, TOGGLE
		NRF_GPIOTE20->SUBSCRIBE_OUT[1] = ((1<<31)|DPPI20_TX_DISABLE_CH);      // 订阅 DPPI 通道事件

		// 注意：不启用中断，因为使用DPPI硬件触发
		RADIO_TIMER->TASKS_START = 1;   // 启动定时器

		NRF_RADIO->SHORTS = RADIO_SHORTS_TXREADY_START_Msk | RADIO_SHORTS_PHYEND_DISABLE_Msk; // 自动在READY时开始，PHYEND时禁用
	

	}else{
		// Start receiving mode for RX device
		printf("Starting RX mode - scanning channels for synchronization\n");
		printf("Running as RX device - Channel scanning mode\n");
		
		// 初始化RX扫描定时器
		rx_scan_timer_init();
		
		NRF_RADIO->SHORTS = RADIO_SHORTS_RXREADY_START_Msk | RADIO_SHORTS_PHYEND_DISABLE_Msk | RADIO_SHORTS_DISABLED_RXEN_Msk; // 连续接收循环

		spi20_init();
		#define RADIO_RX2SPI_CH 0 // 

		NRF_RADIO->PUBLISH_PHYEND = ((1ul<<31)|RADIO_RX2SPI_CH); // 
		NRF_DPPIC10->CHEN = (1<<RADIO_RX2SPI_CH);  // 

		NRF_PPIB11->SUBSCRIBE_SEND[0] = ((1ul<<31)|RADIO_RX2SPI_CH); // 

		// dppi10 and dppi20 can be different ch, this demo use the same ch for simplicity
		NRF_PPIB21->PUBLISH_RECEIVE[0] = ((1ul<<31)|RADIO_RX2SPI_CH); // 
		USR_SPIM->SUBSCRIBE_START = ((1ul<<31)|RADIO_RX2SPI_CH); // 订阅 DPPI 通道事件
		NRF_DPPIC20->CHEN = (1<<RADIO_RX2SPI_CH);  

		// Set packet pointer to RX buffer
		NRF_RADIO->PACKETPTR = (uint32_t)radio_packet;
		
		// Clear events
		NRF_RADIO->EVENTS_READY = 0;
		NRF_RADIO->EVENTS_END = 0;
		NRF_RADIO->EVENTS_DISABLED = 0;
		
		// 从第一个信道开始扫描
		printf("RX scanning channel %d (freq: %d MHz)\n", 
		       current_channel_index, 2400 + hop_channels[current_channel_index]);
		
		// 启动10ms扫描定时器
		rx_scan_timer_start(1000);
		
		// Enable RX - interrupt will handle the rest
		NRF_RADIO->TASKS_RXEN = 1;
		
	}

	uint32_t ticks = 0;
	uint32_t sync_ignore = 0;
	while(1){
		// 处理RX信道扫描超时
		if (!m_is_tx_dev && !rx_synchronized && rx_channel_timeout) {
			rx_channel_timeout = false;
			m_rx_ok = false; // 重置接收状态
			NRF_RADIO->EVENTS_DISABLED = 0; // 清除禁用事件
			NRF_RADIO->TASKS_DISABLE = 1;
			while (NRF_RADIO->EVENTS_DISABLED == 0) {
				// 等待无线电禁用完成
			}

			// 切换到下一个信道
			radio_hop_to_next_channel();
			// printf("RX scanning channel %d (freq: %d MHz)\n", 
			    //    current_channel_index, 2400 + hop_channels[current_channel_index]);
			
			rx_scan_timer_start(1000); // 重新启动扫描定时器

			// 重新启动接收
			NRF_RADIO->TASKS_RXEN = 1;
		}

		if(rx_synchronized){
			sync_ignore++;
		}

		if(sync_ignore<2){

			k_sleep(K_SECONDS(1));
			m_rx_cnts = 0;
			continue;
		}
		
		k_sleep(K_SECONDS(1));
		ticks++;

		if (m_is_tx_dev) {
			printf("TX packets: %d (DPPI triggered, current freq: %d MHz),%X,%X\n", 
			       m_tx_cnts, 2400 + hop_channels[current_channel_index], NRF_PPIB10->OVERFLOW.SEND, NRF_PPIB20->OVERFLOW.SEND);
		} else {
			m_1min_rx_cnts += m_rx_cnts; // 重置计数器
			m_1min_rx_cnts_theory += 8000; // 重置理论计数器
			m_rx_total += m_rx_cnts; // 累计总接收包数
			m_rx_total_theory += 8000; // 累计理论总接收
			printf("RX packets: %d,total: %d,total percentage: %.2f\n", 
				       m_rx_cnts, m_rx_total,
				       m_rx_total * 100.0/ m_rx_total_theory);
		}
		
		if (ticks >= 60) {
			ticks = 0;
			// 每分钟打印一次统计信息
			printf("1 minute RX packets: %d, theoretical: %d, percentage: %.2f\n", 
			       m_1min_rx_cnts, m_1min_rx_cnts_theory,
			       m_1min_rx_cnts * 100.0/ m_1min_rx_cnts_theory);
			m_1min_rx_cnts = 0; // 重置计数器
			m_1min_rx_cnts_theory = 0; // 重置理论计数器
		}
		m_rx_cnts = 0;
		m_tx_cnts = 0;
	}
	return 0;
}
