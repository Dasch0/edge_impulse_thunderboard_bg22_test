/***************************************************************************//**
 * @file
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/

#include "util.h"
#include <stdio.h>
#include "sl_status.h"
#include "sl_imu.h"

#include "model_metadata.h"
#include "ei_run_classifier.h"
#include "numpy.hpp"

// Implementation Note: Make sure this matches your models training data
#define IMU_SAMPLE_RATE 50.0f //Hz
#define CONVERT_G_TO_MS2 9.80665f


// Statically declared input buffer
static float accelerometer_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
static uint32_t accelerometer_buffer_offset;

/// Helper function to sample accelerometer and place samples in float buffer
/// This method will do nothing if the buffer is full or if the accelerometer has
/// no new data.
///
sl_status_t sample_accelerometer_to_buffer(uint32_t* offset, float* buf, bool normalize) {
  sl_status_t sc = SL_STATUS_NOT_READY;
  // Only sample if accelerometer is ready and buffer is not full
  if (sl_imu_is_data_ready() && (*offset <= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME)) {

    // Implementation note: This matches the sequence done by ei_inertial_read_data
    //   To process accelerometer data and output an acceleration value. This means
    //   any networks trained on edge-impulse supported platforms (TB sense 2, phones)
    //   will work properly. However data collected from other sources may use a
    //   different scaling, and this function will need to be modified to work
    //   properly with models trained on such data

    sl_imu_get_acceleration_raw_data(&buf[*offset]);

    buf[*offset] *= CONVERT_G_TO_MS2;
    buf[*offset + 1] *= CONVERT_G_TO_MS2;
    buf[*offset + 2] *= CONVERT_G_TO_MS2;

    *offset += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;

    sc = SL_STATUS_OK;
  }
    return sc;
}

void app_init(void)
{
  // Initialize the IMU driver
  sl_status_t sc;
  sc = sl_imu_init();
  sl_imu_configure(IMU_SAMPLE_RATE);

  // Initialize input buffer
  accelerometer_buffer_offset = 0;

  // Initialize EI classifier
  run_classifier_init();
}

/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void)
{

  // Check accelerometer for samples on each iteration of the application loop
  sample_accelerometer_to_buffer(&accelerometer_buffer_offset, accelerometer_buffer, false);

  // Once buffer is filled, run inference
  if (accelerometer_buffer_offset >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
    EI_IMPULSE_ERROR err;
    ei_impulse_result_t result;
    signal_t signal;

    accelerometer_buffer_offset = 0;

    // Create signal from buffer and run inference

    // Implementation note: There are many ways to generate the signal_t. Creating
    //  from a buffer as shown here is mainly useful if data from accelerometer
    //  is already being stored for other purposes or is being streaming in via
    //  DMA.
    numpy::signal_from_buffer(accelerometer_buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    err = run_classifier(&signal, &result);


    // Print the results (replace this with whatever application logic is needed
    // to respond to the inference result)

    // Implementation note: This minimal port to BG22 thunderboard does not have
    //   proper timekeeping enabled for EI. All timing measurements in the result_t
    //   appear as 0.
    ei_printf("Prediction: \r\n");
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
      ei_printf("    %s: \t", result.classification[ix].label);
      // Convert to integer percentage before printing, since floating point prints
      //  aren't set up
      ei_printf("%d%%", (int) (result.classification[ix].value * 100));
      ei_printf("\r\n");
    }
  }
}
