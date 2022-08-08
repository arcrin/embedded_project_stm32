//
// Created by andy- on 2022-08-06.
//

#ifndef MCU1_STM32F407XX_ADC_DRIVER_H
#define MCU1_STM32F407XX_ADC_DRIVER_H

// Bit position for Status Register (SR)
#define ADC_SR_AWD      0
#define ADC_SR_EOC      1
#define ADC_SR_JEOC     2
#define ADC_SR_JSTRT    3
#define ADC_SR_STRT     4
#define ADC_SR_OVR      5

// Bit position for Control Register 1 (CR1)
#define ADC_CR1_AWDCH   0
#define ADC_CR1_EOCIE   5
#define ADC_CR1_AWDIE   6
#define ADC_CR1_JEOCIE  7
#define ADC_CR1_SCAN    8
#define ADC_CR1_AWDSG   9
#define ADC_CR1_JAUTO   10
#define ADC_CR1_DISCEN  11
#define ADC_CR1_JDISCE  12
#define ADC_CR1_DISCNUM 13
#define ADC_CR1_JAWDEN  22
#define ADC_CR1_AWDEN   23
#define ADC_CR1_RES     24
#define ADC_CR1_OVRIE   26

// Bit position for Control Register 2 (CR2)
#define ADC_CR2_ADON    0
#define ADC_CR2_CONT    1
#define ADC_CR2_DMA     8
#define ADC_CR2_DDS     9
#define ADC_CR2_EOCS    10
#define ADC_CR2_ALIGN   11
#define ADC_CR2_JEXTSEL 16
#define ADC_CR2_JEXTEN  20
#define ADC_CR2_JSWST   22
#define ADC_CR2_EXTSEL  25
#define ADC_CR2_EXTEN   28
#define ADC_CR2_SWSTART 30

// Bit position for Sample Time Register 1 (SMPR1)
#define ADC_SMPR1_10    0
#define ADC_SMPR1_11    3
#define ADC_SMPR1_12    6
#define ADC_SMPR1_13    9
#define ADC_SMPR1_14    12
#define ADC_SMPR1_15    15
#define ADC_SMPR1_16    18
#define ADC_SMPR1_17    21
#define ADC_SMPR1_18    24

// Bit position for Sample Time Register 2 (SMPR2)
#define ADC_SMPR2_0     0
#define ADC_SMPR2_1     3
#define ADC_SMPR2_2     6
#define ADC_SMPR2_3     9
#define ADC_SMPR2_4     12
#define ADC_SMPR2_5     15
#define ADC_SMPR2_6     18
#define ADC_SMPR2_7     21

// Bit position for Injected Channel Data Offset Register 1 (JOFER1)
#define ADC_JOFR1_JOFFSET   0


// Bit position for Injected Channel Data Offset Register 2 (2)
#define ADC_JOFR1_JOFFSET   0


// Bit position for Injected Channel Data Offset Register 3 (JOFER3)
#define ADC_JOFR1_JOFFSET   0


// Bit position for Injected Channel Data Offset Register 4 (JOFER4)
#define ADC_JOFR1_JOFFSET   0

// Bit position for Higher Threshold Register (HTR)
#define ADC_HTR_HT          0

// Bit position for Lower Threshold Register (LTR)
#define ADC_LTR_LT          0

// Bit position for Sequence Register 1 (SQR1)
#define ADC_SQR1_SQ13       0
#define ADC_SQR1_SQ14       5
#define ADC_SQR1_SQ15       10
#define ADC_SQR1_SQ16       15
#define ADC_SQR1_L          20

// Bit position for Sequence Register 2
#define ADC_SQR2_SQ7        0
#define ADC_SQR2_SQ8        5
#define ADC_SQR2_SQ9        10
#define ADC_SQR2_SQ10       15
#define ADC_SQR2_SQ11       20
#define ADC_SQR2_SQ12       25

// Bit position for Sequence Register 3
#define ADC_SQR3_SQ1    0
#define ADC_SQR3_SQ2    5
#define ADC_SQR3_SQ3    10
#define ADC_SQR3_SQ4    15
#define ADC_SQR3_JL     20

// Bit position for Common Status Register (CSR)
#define ADC_CSR_AWD1    0
#define ADC_CSR_EOC1    1
#define ADC_CSR_JEOC1   2
#define ADC_CSR_JSTRT1  3
#define ADC_CSR_STRT1   4
#define ADC_CSR_OVR1    5

#define ADC_CSR_AWD2    8
#define ADC_CSR_EOC2    9
#define ADC_CSR_JEOC2   10
#define ADC_CSR_JSTRT2  11
#define ADC_CSR_STRT2   12
#define ADC_CSR_OVR2    13

#define ADC_CSR_AWD3    16
#define ADC_CSR_EOC3    17
#define ADC_CSR_JEOC3   18
#define ADC_CSR_JSTRT3  19
#define ADC_CSR_STRT3   20
#define ADC_CSR_OVR3    21


// Bit position for Common Control Register (CCR)
#define ADC_CCR_MULT    0
#define ADC_CCR_DELAY   8
#define ADC_CCR_DDS     13
#define ADC_CCR_DMA     14
#define ADC_CCR_ADCPRE  16
#define ADC_CCR_VBATE   22
#define ADC_CCR_TSVREFE 23


void ADC_Init();

void Start_Conversion();

uint32_t ADC_Read();

#endif //MCU1_STM32F407XX_ADC_DRIVER_H
