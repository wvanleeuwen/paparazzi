#ifndef CONFIG_TINY_H
#define CONFIG_TINY_H

#ifdef SITL
/* Dummy definitions: adc are unused anyway */
#define AdcBank0(x) (x)
#define AdcBank1(x) (x)
#endif /* SITL */

/* Master oscillator freq.       */
#define FOSC (12000000)

/* PLL multiplier                */
#define PLL_MUL (5)

/* CPU clock freq.               */
#define CCLK (FOSC * PLL_MUL)

/* Peripheral bus speed mask 0x00->4, 0x01-> 1, 0x02 -> 2   */
#define PBSD_BITS 0x00
#define PBSD_VAL 4

/* Peripheral bus clock freq. */
#define PCLK (CCLK / PBSD_VAL)

#define LED_1_BANK 1
#define LED_1_PIN 23

#define LED_2_BANK 1
#define LED_2_PIN 17

#define LED_3_BANK 1
#define LED_3_PIN 21

#define POWER_SWITCH_LED 3

#define LED_4_BANK 1
#define LED_4_PIN 18

#define MODEM_POWER_LED 4

#define LED_5_BANK 1
#define LED_5_PIN 16

#define LED_GPS_RESET_BANK 1
#define LED_GPS_RESET_PIN 19

#define Configure_GPS_RESET_Pin() LED_INIT(GPS_RESET)
#define Set_GPS_RESET_Pin_LOW() LED_ON(GPS_RESET)
#define Open_GPS_RESET_Pin() ClearBit(LED_DIR(GPS_RESET), LED_PIN(GPS_RESET))

/* P0.5 aka MAT0.1  */
#define SERVO_CLOCK_PIN  5
#define SERVO_CLOCK_PINSEL PINSEL0
#define SERVO_CLOCK_PINSEL_VAL 0x02
#define SERVO_CLOCK_PINSEL_BIT 10
/* p1.20          */
#define SERVO_RESET_PIN 20

/* PPM : rc rx on P0.6*/
#define PPM_PINSEL PINSEL0
#define PPM_PINSEL_VAL 0x02
#define PPM_PINSEL_BIT 12
#define PPM_CRI TIR_CR2I
#define PPM_CCR_CRF TCCR_CR2_F
#define PPM_CCR_CRR TCCR_CR2_R
#define PPM_CCR_CRI TCCR_CR2_I
#define PPM_CR T0CR2

/* ADC */

// IRV
#define ADC_0 AdcBank1(6)
#ifdef USE_ADC_0
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_6
#endif

// IRH1
#define ADC_1 AdcBank1(7)
#ifdef USE_ADC_1
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_7
#endif

// IRH2
#define ADC_2 AdcBank0(4)
#ifdef USE_ADC_2
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_4
#endif

// ONBOARDCAM
#define ADC_3 AdcBank0(6)
#ifdef USE_ADC_3
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_6
#endif

#define ADC_14 AdcBank1(4)
#ifdef USE_ADC_14
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_4
#endif

// GYRO Y
#define ADC_4 AdcBank0(3)
#ifdef USE_ADC_4
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_3
#endif

// GYRO X
#define ADC_5 AdcBank0(2)
#ifdef USE_ADC_5
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_2
#endif

/*
// Not connected
#define ADC_6 AdcBank0(1)
#ifdef USE_ADC_6
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_1
#endif
*/

// CURRENT
#ifndef ADC_CHANNEL_CURRENT
#define ADC_CHANNEL_CURRENT AdcBank1(3)
#endif
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_3

#define DefaultMilliAmpereOfAdc(adc) ((adc-506)*51)


// VBAT
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY AdcBank1(5)
#endif
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_5


#define DefaultVoltageOfAdc(adc) (0.0247311828*adc)


#endif /* CONFIG_TINY_H */
