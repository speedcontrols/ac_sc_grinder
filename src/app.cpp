#include "app.h"

#include "adc.h"

#include "speed_controller.h"
#include "sensors.h"
#include "triac_driver.h"
#include "calibrator.h"

SpeedController speedController;
Sensors sensors;
TriacDriver triacDriver(sensors);
Calibrator calibrator;

////////////////////////////////////////////////////////////////////////////////

// ADC data is transfered to double size DMA buffer. Interrupts happen on half
// transfer and full transfer. So, we can process received data without risk
// of override. While half of buffer is processed, another half os used to
// collect next data.

uint16_t ADCBuffer[ADC_FETCH_PER_TICK * ADC_CHANNELS_COUNT * 2];

// Polling flag, set to true every time when new ADC data is ready.
volatile bool adc_data_ready = false;
// Offset of actual ADC data in DMA buffer,
volatile uint32_t adc_data_offset = 0;

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
  adc_data_ready = true;
  adc_data_offset = 0;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
  adc_data_ready = true;
  adc_data_offset = ADC_FETCH_PER_TICK * ADC_CHANNELS_COUNT;
}


// Main app loop.
void app_start()
{
  eeprom_float_init();
  // Load config info from emulated EEPROM
  speedController.configure();
  sensors.configure();

  // Final hardware start: calibrate ADC & run cyclic DMA ops.
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADCBuffer, ADC_FETCH_PER_TICK * ADC_CHANNELS_COUNT * 2);

  // Override loop in main.c to reduce patching
  while (1) {
    // Polling for flag which indicates that ADC data is ready
    while (!adc_data_ready) {}

    // Reset flag
    adc_data_ready = false;

    // Load samples from actual half of ADC buffer to sensors buffers
    sensors.adc_raw_data_load(ADCBuffer, adc_data_offset);

    sensors.tick();

    // Detect calibration mode & run calibration procedure if needed.
    // If calibration in progress - skip other steps.
    if (calibrator.tick()) continue;

    // Normal processing

    speedController.in_knob = sensors.knob;
    speedController.in_speed = sensors.speed;

    speedController.tick();

    triacDriver.setpoint = speedController.out_power;

    triacDriver.tick();
  }
}
