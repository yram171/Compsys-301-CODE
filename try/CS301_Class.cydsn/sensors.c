#include <project.h>
#include <sensors.h>
#include <stdint.h>

/* Peak-to-peak acquisition — keep as-is (example) */
uint16_t Sensor_ComputePeakToPeak(uint8_t channel) {
    uint16_t min_val = 4095;
    uint16_t max_val = 0;

    ADC_StartConvert();
    for (uint16_t i=0;i<N_SAMPLES;i++){
        while(!ADC_IsEndConversion(ADC_RETURN_STATUS)){}
        uint16_t sample = ADC_GetResult16(channel) & 0x0FFF;
        if(sample < min_val) min_val = sample;
        if(sample > max_val) max_val = sample; 
    } 
    ADC_StopConvert();
    return (max_val - min_val);
}