/*
* CO2_mini.c
* Send over nRF24 CO2 level
*
* Created: 24.01.2023
*  Author: Vadim Kulakov, vad7@yahoo.com
*
* ATtiny44A
*
* Radio nRF24L01+
*/ 
#define F_CPU 4000000UL
// Fuses: BODLEVEL = 2.7V (BODLEVEL[2:0] = 101)

//#define DEBUG_PROTEUS

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/atomic.h>

const char ProgramID[] PROGMEM = "CO2 mini v1";

#define MAX_FANS					10 // Maximum fans

#define LED1_ANODE_TO_VCC
#define LED1_PORT					PORTA		// Info led
#define LED1						(1<<PORTA1)
#ifdef LED1_ANODE_TO_VCC
#define LED1_ON						LED1_PORT &= ~LED1
#define LED1_OFF					LED1_PORT |= LED1
#else
#define LED1_ON						LED1_PORT |= LED1
#define LED1_OFF					LED1_PORT &= ~LED1
#endif

#define KEY1						(1<<PORTB0) // = IR Receiver out
#define KEY1_PRESSING				!(PINB & KEY1)
#define KEYS_SETUP					PORTB |= KEY1

#define CO2SensorState				(PINA & (1<<PORTA7)) // SenseAir S8 LP - 1kHz PWM out, using Input Capture
#define CO2SensorMax				2000 // ppm
#define CO2_PWM_Add					1

#define UNUSED_PINS_SETUP			PORTA |= (1<<PORTA0); PORTB |= (1<<PORTB1) | (1<<PORTB2)	// Pullup: not used
#define SETUP_WATCHDOG WDTCSR = (1<<WDCE) | (1<<WDE); WDTCSR = (1<<WDE) | (0<<WDIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0);	//  Watchdog 1 s

//											// 0xF0 mask - Number of long flashes, 0x0F mask - Number of short flashes
#define WRN_SETUP					0x01
#define WRN_SETUP_INFO				0x02
#define WRN_SETUP_ERR				0x10
#define WRN_RF_Send					0x10	// Send failure, after short bursts = fan offset (mask 0x0F)
#define WRN_RF_SetAddr				0x20	// Set addresses failure,
#define WRN_RF_NotResp				0x30	// RF module not response,
#define WRN_CO2Sensor				0x40	// CO2 Sensor reading failure

struct _EEPROM {
	uint8_t _OSCCAL;
	uint8_t RF_RxAddress;		// nRF24 receive address LSB
	uint8_t RF_Channel;			// nRF24 channel
	uint8_t RF_REG_SETUP_RETR;	//(0bxxxx<<NRF24_BIT_ARD) | (0bxxxx<<NRF24_BIT_ARC), // Auto Retransmit Delay, Number Re-Transmit on fail
	uint8_t SendPeriod;			// sec
	int16_t CO2_Threshold;		// Threshold for speed = 1
	int16_t CO2_correct;		// for correct CO2 measure - add to sensor value
	uint8_t RF_FanAddr[MAX_FANS]; // Fans address LSB
} __attribute__ ((packed));
struct _EEPROM EEMEM EEPROM;

#define fSetup_Write				0x80 // EEPROM[Type] = Data
#define fSetup_WriteStart			0x8F
#define fSetup_Read					0xC0 // read MEM[Data] => Data, MEM type: EEPROM, MAIN, PROGRAM
#define fSetup_1b					0x01
#define fSetup_2b					0x02
#define Type_EEPROM					0	// EEPROM
#define Type_RAM					1	// main memory (OSCCAL=0x51)
#define Type_PROGMEM				2	// Program FLASH
#define Type_PGM_ID					3	// ProgramID(cstring)
#define Type_RESET					4	// Restart program

struct SETUP_DATA { // the same size as SEND_DATA!
	uint16_t Data;	// Read/Write byte or word
	uint8_t Type;	// Read data type(SetupType.*) or Write EEPROM address
	uint8_t Flags;	// Setup command: fSetup_*
} __attribute__ ((packed));

struct SEND_DATA {
	uint16_t CO2level;
	uint8_t FanSpeed;
	uint8_t Flags;
} __attribute__ ((packed));
struct SEND_DATA data;
uint8_t WriteTimeout = 0;	// > 0 - Write starting

volatile uint8_t LED_Warning		= 0; // Flashes, long (mask 0xF0):	0 - all ok, or ERR_*
uint8_t LED_WarningOnCnt = 0, LED_WarningOffCnt = 0, LED_Warning_WorkLong = 0, LED_Warning_WorkShort = 0, LED_Warning_NoRepeat = 0;
volatile uint8_t Timer				= 0;
uint8_t Key1Pressing				= 0;
int16_t CO2Level					= 0;
int16_t CO2Level_correct			= 0;
uint8_t CO2Level_new				= 0;	// if > 0 then decrement on timer overflow, if = 0 then CO2 = MAX

#if(1)
void Delay10us(uint8_t ms) {
	while(ms-- > 0) _delay_us(10); 
	wdt_reset();
}
//void Delay1ms(uint8_t ms) {
//	while(ms-- > 0) { 
//		_delay_ms(1); 
//		wdt_reset(); 
//	}
//}
void Delay100ms(unsigned int ms) {
	while(ms-- > 0) { 
		_delay_ms(100); 
		wdt_reset();
	}
}

void FlashLED(uint8_t num, uint8_t toff, uint8_t ton) {
	while (num-- > 0) {
		LED1_OFF;
		Delay100ms(toff);
		LED1_ON;
		Delay100ms(ton);
	}
	LED1_OFF;
}

void FlashNumber(uint8_t Number)
{ // HEX
	FlashLED(Number / 16, 5, 15);
	Delay100ms(20);
	FlashLED(Number % 16, 5, 5);
	Delay100ms(20);
}

#endif

#include "nRF24L01.h"

uint16_t TCNT1_prev = 0;
uint16_t TCNT1_last = 0;
ISR(TIM1_CAPT_vect)
{
	TCNT1_prev = TCNT1_last;
	TCNT1_last = ICR1;
	if(TCCR1B & (1<<ICES1)) {
		TCCR1B &= ~(1<<ICES1);
	} else {
		TCCR1B |= (1<<ICES1);
		uint16_t n = (TCNT1_last - TCNT1_prev + CO2_PWM_Add) / 2;  // tick = 0.5us, 2000 ppm = 1ms
		if(n > CO2SensorMax) n = CO2SensorMax; else n += CO2Level_correct;
		CO2Level = (CO2Level + n) / 2;
		CO2Level_new = 2;
	}
}

ISR(TIM1_OVF_vect) // there is no pulses - max value
{
	if(CO2Level_new) CO2Level_new--; else CO2Level = CO2SensorMax;
}

uint8_t TimerCnt = 0;
ISR(TIM0_OVF_vect, ISR_NOBLOCK) // 21Hz, 0.0476 s
{
	if(++TimerCnt == 21) { // 1 sec
		TimerCnt = 0;
		if(Timer) Timer--;
	}
	if(KEY1_PRESSING) {
		if(Key1Pressing != 255) Key1Pressing++;
	} else Key1Pressing = 0;
	if(WriteTimeout) WriteTimeout--;
	// LED_Warning: 0xF0 mask - Number of long flashes, 0x0F mask - Number of short flashes, LED_Warning_NoRepeat = no repeat
	if(LED_WarningOnCnt) {
		LED1_ON;
		LED_WarningOnCnt--;
	} else if(LED_WarningOffCnt) {
		LED1_OFF;
		LED_WarningOffCnt--;
	} else if(LED_Warning_WorkLong) { // long flashes
		LED_Warning_WorkLong--;
		LED_WarningOnCnt = 26;			// 1.25s (*0.0476s)
		if(LED_Warning_WorkLong == 0) {
			LED_WarningOffCnt = 12;		// 0.6s
			goto xSetPause;
		} else LED_WarningOffCnt = 8;	// 0.38s
	} else if(LED_Warning_WorkShort) { // short flashes
		LED_Warning_WorkShort--;
		LED_WarningOnCnt = 5;	// 0.3s
		LED_WarningOffCnt = 7;	// 0.3s
xSetPause:	
		if(LED_Warning_WorkShort == 0) LED_WarningOffCnt = 50; // 2.4s
	} else if(LED_Warning) {
		LED_Warning_WorkLong = (LED_Warning & 0xF0) >> 4;
		LED_Warning_WorkShort = LED_Warning & 0x0F;
		if(LED_Warning_NoRepeat) {
			LED_Warning_NoRepeat = 0;
			LED_Warning = 0;
		}
	}
}

void Set_LED_Warning(uint8_t d)
{
	if(LED_Warning == 0) {
		LED_Warning_NoRepeat = 1;
		LED_Warning = d;
	}
}

void Set_LED_Warning_New(uint8_t d)
{
	LED_Warning_WorkLong = LED_Warning_WorkShort = LED_WarningOnCnt = LED_WarningOffCnt = 0;
	LED_Warning = d;
	if(d) LED_Warning_NoRepeat = 1;
	LED1_OFF;
}

void GetSettings(void)
{
	uint8_t b = eeprom_read_byte(&EEPROM._OSCCAL);
	if(b != 0xFF) OSCCAL = b;
	CO2Level_correct = eeprom_read_word((uint16_t*)&EEPROM.CO2_correct);
}

void ResetSettings(void)
{
	//eeprom_update_byte(&EEPROM._OSCCAL, OSCCAL);
	eeprom_update_byte(&EEPROM.RF_RxAddress, 0xCF);
	eeprom_update_byte(&EEPROM.RF_Channel, 122);
	eeprom_update_byte(&EEPROM.RF_REG_SETUP_RETR, (0b0000<<NRF24_BIT_ARD) | (0b0011<<NRF24_BIT_ARC)); // Auto Retransmit Delay,  Re-Transmit on fail
	eeprom_update_byte(&EEPROM.SendPeriod, 30); // sec
	eeprom_update_word((uint16_t*)&EEPROM.CO2_Threshold, 1000); // ppm
	eeprom_update_word((uint16_t*)&EEPROM.CO2_correct, 0);
	eeprom_update_byte(&EEPROM.RF_FanAddr[0], 0xC1);
	eeprom_update_byte(&EEPROM.RF_FanAddr[1], 0); // after last => 0
}

int main(void)
{
	CLKPR = (1<<CLKPCE); CLKPR = (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (1<<CLKPS0); // Clock prescaler division factor: 2
	MCUCR = (1<<SE) | (0<<SM1) | (0<<SM0); // Idle sleep enable
	DDRA = LED1; // Out
	NRF24_DDR |= NRF24_CE | NRF24_CSN | NRF24_SCK | NRF24_MOSI; // Out
	KEYS_SETUP;
	UNUSED_PINS_SETUP;
	// Timer 8 bit		NRF24L01_Buffer	Unknown identifier	Error
	TCCR0A = (1<<WGM01) | (1<<WGM00);  // Timer0: Fast PWM OCRA
	TCCR0B = (1<<WGM02) | (1 << CS02) | (0 << CS01) | (1 << CS00); // Timer0 prescaller: 1024
	OCR0A = 185; // OC0A - Fclk/(prescaller*(1+TOP)) = 21hz
	//OCR0B = 0; // Half Duty cycle ((TOP+1)/2-1)
	TIMSK0 = (1<<TOIE0); // Timer/Counter0, Overflow Interrupt Enable
	// Timer 16 bit
	TCCR1A = (1<<WGM11) | (1<<WGM10);  // Timer1: Fast PWM Top OCR1A (15)
	TCCR1B = (1<<ICES1) | (0<<ICNC1) | (1<<WGM13) | (1<<WGM12) | (0<<CS12) | (0<<CS11) | (1<<CS10); // Timer1: /1, Input Capture Rising Edge
	OCR1A = 0xFFFF; // =Fclk/prescaller/Freq - 1; Freq=Fclk/(prescaller*(1+TOP)). resolution 0.5us/2
	TIMSK1 = (1<<ICIE1) | (1<<TOIE1); // Timer/Counter1: Input Capture Interrupt Enable
	// ADC
 	//ADMUX = (0<<REFS1) | (1<<MUX2)|(1<<MUX1)|(1<<MUX0); // ADC7 (PA7)
 	//ADCSRA = (1<<ADEN) | (0<<ADATE) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // ADC enable, Free Running mode, Interrupt, ADC 128 divider
 	//ADCSRB = (1<<ADLAR) | (0<<ADTS2) | (0<<ADTS1) | (0<<ADTS0); // ADC Left Adjust Result
	// Pin change
 	//GIMSK = (1<<PCIE0) | (1<<PCIE1); // Pin Change Interrupt Enable 0, 1
 	//PCMSK0 = (1<<PCINT0); // Pin Change Mask Register 0 - Sensor
 	//PCMSK1 = (1<<PCINT8) | (1<<PCINT9) | (1<<PCINT10); // Pin Change Mask Register 0 - Keys
	if(eeprom_read_byte(&EEPROM.RF_RxAddress) == 0xFF) ResetSettings();
	GetSettings();
	//Delay100ms(1);
	SETUP_WATCHDOG;
	sei();
	FlashLED(2,1,1);
	NRF24_init(eeprom_read_byte(&EEPROM.RF_Channel)); // After init transmit must be delayed
	NRF24_WriteByte(NRF24_CMD_W_REGISTER | NRF24_REG_SETUP_RETR, eeprom_read_byte(&EEPROM.RF_REG_SETUP_RETR));
	while(!NRF24_SetAddresses(eeprom_read_byte(&EEPROM.RF_RxAddress))) {
		FlashLED(5,1,1);
		#ifdef DEBUG_PROTEUS
			break;
		#endif
	}
	NRF24_SetMode(NRF24_ReceiveMode);
	Timer = 15; // sec
	do { // Setup
		__asm__ volatile ("" ::: "memory"); // Need memory barrier
		//wdt_reset();
		Delay10us(10);
		struct SETUP_DATA *p = (struct SETUP_DATA*)&data;
		if(NRF24_Receive((uint8_t*)&data)) {
			Set_LED_Warning(1);
			if(p->Flags == fSetup_WriteStart) WriteTimeout = 20;  // *0.047 sec
			else if((p->Flags == fSetup_Write + fSetup_1b || p->Flags == fSetup_Write + fSetup_2b)) { // setup WRITE command
				if(p->Type < sizeof(struct _EEPROM) && WriteTimeout) {
					if(p->Flags == fSetup_Write + fSetup_1b) {
						eeprom_update_byte((uint8_t*)&EEPROM + p->Type, p->Data);
					} else {
						eeprom_update_word((uint16_t*)((uint8_t*)&EEPROM + p->Type), p->Data);
					}
					GetSettings();
					Set_LED_Warning(2);
					WriteTimeout = 20; // *0.047 sec
				}
				Timer = 255; // sec
			} else if((p->Flags == fSetup_Read + fSetup_1b || p->Flags == fSetup_Read + fSetup_2b)) { // setup READ command
				if(p->Type == Type_EEPROM) {
					if(p->Data < sizeof(struct _EEPROM)) {
						if(p->Flags == fSetup_Read + fSetup_1b) p->Data = eeprom_read_byte((uint8_t*)&EEPROM + p->Data);
						else p->Data = eeprom_read_word((uint16_t*)((uint8_t*)&EEPROM + p->Data));
						p->Type = p->Flags = 0;
					} else continue;
				} else if(p->Type == Type_RAM) {
					ATOMIC_BLOCK(ATOMIC_FORCEON) {
						if(p->Flags == fSetup_Read + fSetup_1b) p->Data = *((uint8_t *)p->Data);
						else p->Data = *((uint16_t *)p->Data);
					}
					p->Type = p->Flags = 0;
				} else if(p->Type == Type_PROGMEM) {
					if(p->Flags == fSetup_Read + fSetup_1b) p->Data = pgm_read_byte(p->Data);
					else p->Data = pgm_read_word(p->Data);
					p->Type = p->Flags = 0;
				} else if(p->Type == Type_RESET) {
					LED1_ON;
					cli(); while(1) ; // restart
				}
				Delay100ms(1);
				NRF24_SetMode(NRF24_TransmitMode);
				uint8_t err = 0;
				if(p->Type == Type_PGM_ID) {
					for(uint8_t i = 0; i < sizeof(ProgramID); i++) {
						p->Data = pgm_read_byte(&ProgramID[i]);
						err = NRF24_Transmit((uint8_t *)&data);
						if(err) break;
						Delay10us(255);
					}
				} else err = NRF24_Transmit((uint8_t *)&data);
				NRF24_SetMode(NRF24_ReceiveMode);
				if(err) Set_LED_Warning_New(WRN_SETUP_ERR + err);
				Timer = 255; // sec
			}
		}
		if(Key1Pressing > 200) { // reset settings after 10 sec
			FlashLED(50, 1, 1);
			ResetSettings();
			cli(); while(1) ; // restart
		}
	} while(Timer);
	FlashLED(3,1,1);
	{  // adjust OSCAL on 1kHz PWM form CO2 sensor
		int16_t n, delta, prev_delta = 32767;
		uint16_t prev = 0;
		uint8_t skip = 1;
		uint8_t prev_OSCCAL = OSCCAL;
		Timer = 2; // sec
		do {
			__asm__ volatile ("" ::: "memory"); // Need memory barrier
			sleep_cpu();
			wdt_reset();
			if((TCCR1B & (1<<ICES1)) == 0) { // New
				ATOMIC_BLOCK(ATOMIC_FORCEON) { 
					n = TCNT1_prev - prev; 
					prev = TCNT1_prev;
				}
				if(skip) {
					skip = 0;
					continue;
				}
				delta = n - CO2SensorMax * 2 + CO2_PWM_Add;
				if(prev_delta < 4 && prev_delta < abs(delta)) {
					OSCCAL = prev_OSCCAL;
					eeprom_update_byte(&EEPROM._OSCCAL, OSCCAL);
					break;
				}
				prev_OSCCAL = OSCCAL;
				if(delta < 0) {
					OSCCAL = prev_OSCCAL + 1;
				} else if(delta > 0) {
					OSCCAL = prev_OSCCAL - 1;
				} else break;
				prev_delta = abs(delta);
				skip = 1;
			}
		} while(Timer);
	}
	NRF24_SetMode(NRF24_TransmitMode);
	uint8_t n = eeprom_read_byte(&EEPROM.SendPeriod);
	if(n < 128) n *= 2;
	Timer = n;
	n = 0;
	while(1)
	{
		__asm__ volatile ("" ::: "memory"); // Need memory barrier
		sleep_cpu();
		wdt_reset();
		if(Timer == 0) {
			//NRF24_SetMode(NRF24_TransmitMode);
			data.FanSpeed = data.Flags = 0;
			ATOMIC_BLOCK(ATOMIC_FORCEON) { data.CO2level = CO2Level; }
			if(data.CO2level > (int16_t)eeprom_read_word((uint16_t*)&EEPROM.CO2_Threshold)) data.FanSpeed = 1;
			uint8_t fan = 0;
			for(; fan < MAX_FANS; fan++) {
				uint8_t addr = eeprom_read_byte(&EEPROM.RF_FanAddr[fan]);
				if(addr == 0) break;
				if(n != 1) { // if only 1 fan - skip:
					if(!NRF24_SetAddresses(addr)) {
						Set_LED_Warning(WRN_RF_SetAddr);
						break;
					}
				}
				uint8_t err = NRF24_Transmit((uint8_t *)&data);
				uint8_t err2 = err == 2 ? WRN_RF_NotResp : (WRN_RF_Send + fan + 1);
				if(err)	Set_LED_Warning(err2);
			}
			n = fan;
			//NRF24_Powerdown();
			Timer = eeprom_read_byte(&EEPROM.SendPeriod);
		}
	}
}
