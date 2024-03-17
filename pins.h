
// This is a replacement firmware for the Corona RP8D1 RC receiver.
//
// It has only been used on the 8-channel 35MHz version (RP8D1). If you want to use it
// on other frequency versions of the receiver then you need to modify the
// CH_STEP, CH0_FREQ, START_CHAN, END_CHAN #defines accordingly.
//
// Their are NO guarantees or warranties what so ever with this code, use at your own risk!

#ifndef _PINS_H_
#define _PINS_H_

#define BUT_BIT						PD3
#define BUT_DIR						DDRD
#define BUT_PIN						PIND
#define BUT_PORT					PORTD

#define LED_BIT						PD4
#define LED_DIR						DDRD
#define LED_PIN						PIND
#define LED_PORT					PORTD

#define UART_RXD_BIT				PD0
#define UART_RXD_DIR				DDRD
#define UART_RXD_PIN				PIND
#define UART_RXD_PORT				PORTD

#define UART_TXD_BIT				PD1
#define UART_TXD_DIR				DDRD
#define UART_TXD_PIN				PIND
#define UART_TXD_PORT				PORTD

#define DC_REFERENCE_IN_BIT			PD7
#define DC_REFERENCE_IN_DIR			DDRD
#define DC_REFERENCE_IN_PIN			PIND
#define DC_REFERENCE_IN_PORT		PORTD

#define RSSI_IN_BIT					PC0
#define RSSI_IN_DIR					DDRC
#define RSSI_IN_PIN					PINC
#define RSSI_IN_PORT				PORTC

#define PPM_FM_THRES_IN_BIT			PC1
#define PPM_FM_THRES_IN_DIR			DDRC
#define PPM_FM_THRES_IN_PIN			PINC
#define PPM_FM_THRES_IN_PORT		PORTC

#define PPM_FM_SIG_IN_BIT			PD6
#define PPM_FM_SIG_IN_DIR			DDRD
#define PPM_FM_SIG_IN_PIN			PIND
#define PPM_FM_SIG_IN_PORT			PORTD

#define PLL_CE_OUT_BIT				PC2
#define PLL_CE_OUT_DIR				DDRC
#define PLL_CE_OUT_PIN				PINC
#define PLL_CE_OUT_PORT				PORTC

#define PLL_DATA_OUT_BIT			PC3
#define PLL_DATA_OUT_DIR			DDRC
#define PLL_DATA_OUT_PIN			PINC
#define PLL_DATA_OUT_PORT			PORTC

#define PLL_CLK_OUT_BIT				PC4
#define PLL_CLK_OUT_DIR				DDRC
#define PLL_CLK_OUT_PIN				PINC
#define PLL_CLK_OUT_PORT			PORTC

#define OC1A_BIT					PB1
#define OC1A_DIR					DDRB
#define OC1A_PIN					PINB
#define OC1A_PORT					PORTB

#define CHAN_1_OUT_BIT				PB0
#define CHAN_1_OUT_DIR				DDRB
#define CHAN_1_OUT_PIN				PINB
#define CHAN_1_OUT_PORT				PORTB

#define CHAN_2_OUT_BIT				PB1
#define CHAN_2_OUT_DIR				DDRB
#define CHAN_2_OUT_PIN				PINB
#define CHAN_2_OUT_PORT				PORTB

#define CHAN_3_OUT_BIT				PB2
#define CHAN_3_OUT_DIR				DDRB
#define CHAN_3_OUT_PIN				PINB
#define CHAN_3_OUT_PORT				PORTB

#define CHAN_4_OUT_BIT				PB3
#define CHAN_4_OUT_DIR				DDRB
#define CHAN_4_OUT_PIN				PINB
#define CHAN_4_OUT_PORT				PORTB

#define CHAN_5_OUT_BIT				PB4
#define CHAN_5_OUT_DIR				DDRB
#define CHAN_5_OUT_PIN				PINB
#define CHAN_5_OUT_PORT				PORTB

#define CHAN_6_OUT_BIT				PB5
#define CHAN_6_OUT_DIR				DDRB
#define CHAN_6_OUT_PIN				PINB
#define CHAN_6_OUT_PORT				PORTB

// also TXD
#define CHAN_7_OUT_BIT				PD1
#define CHAN_7_OUT_DIR				DDRD
#define CHAN_7_OUT_PIN				PIND
#define CHAN_7_OUT_PORT				PORTD

// also RXD
#define CHAN_8_OUT_BIT				PD0
#define CHAN_8_OUT_DIR				DDRD
#define CHAN_8_OUT_PIN				PIND
#define CHAN_8_OUT_PORT				PORTD

#endif
