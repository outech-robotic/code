//
// Created by tic-tac on 03/08/2020.
//
#include "adc.h"
#include "peripheral/stm32f0/gpio.h"
#include <stm32f0xx_ll_bus.h>
#include "config.h"

const uint32_t adc_channels[C_NB_ADC_CHANNELS] = {
        LL_ADC_CHANNEL_0,
        LL_ADC_CHANNEL_1,
        LL_ADC_CHANNEL_2,
        LL_ADC_CHANNEL_3,
        LL_ADC_CHANNEL_4,
        LL_ADC_CHANNEL_5,
        LL_ADC_CHANNEL_6,
        LL_ADC_CHANNEL_7,
        LL_ADC_CHANNEL_8,
};

uint16_t adc_data[C_NB_ADC_CHANNELS] = {0};
uint8_t adc_current = 0;
bool adc_active = false;


void ADC_start_conversion(){
    if(!adc_active) {
        LL_ADC_REG_StartConversion(ADC1);
        adc_active = true;
    }
}


bool ADC_is_ready(){
    return !adc_active;
}


int16_t ADC_get_measurement(uint8_t index){
    if(index<C_NB_ADC_CHANNELS) {
        return adc_data[index];
    }
    else{
        return -1;
    }
}


int ADC_init() {
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);

    // Set channels to use
    LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_0);
    LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_1);
    LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_2);
    LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_3);
    LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_4);
    LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_5);
    LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_6);
    LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_7);
    LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_8);
    LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_TEMPSENSOR);
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_TEMPSENSOR);

    // Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    LL_ADC_InitTypeDef ADC_InitStruct;
    LL_ADC_StructInit(&ADC_InitStruct);
    ADC_InitStruct.Clock = LL_ADC_CLOCK_ASYNC;
    ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
    ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
    ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
    LL_ADC_Init(ADC1, &ADC_InitStruct);

    LL_ADC_REG_InitTypeDef ADC_REG_InitStruct;
    LL_ADC_REG_StructInit(&ADC_REG_InitStruct);
    ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
    ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
    ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
    ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
    ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
    LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
    LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
    LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_239CYCLES_5);
    LL_ADC_EnableIT_EOC(ADC1);
    LL_ADC_EnableIT_EOS(ADC1);
    NVIC_EnableIRQ(ADC1_IRQn);
    LL_ADC_Enable(ADC1);

    for(uint8_t i = 0; i < C_NB_ADC_CHANNELS; i++){
        adc_data[i] = 0;
    }
    adc_current = 0;
    adc_active = false;

    return 0;
}


#ifdef __cplusplus
extern "C" {
#endif

void ADC1_IRQHandler(void) {
    if(LL_ADC_IsActiveFlag_EOC(ADC1)){
        // A new conversion is complete.
        adc_data[adc_current] = LL_ADC_REG_ReadConversionData12(ADC1);
        adc_current++;
        if(LL_ADC_IsActiveFlag_EOS(ADC1)){
            // End of sampling sequence, all measurements are complete.
            togglePin(PB7);
            LL_ADC_ClearFlag_EOS(ADC1);
            adc_current = 0;
            adc_active = false;
        }
    }
}

#ifdef __cplusplus
}
#endif
