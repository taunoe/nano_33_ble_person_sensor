/*************************************************************
 * Copyright 2022-2024 Tauno Erik
 * 
 * File: main.cpp
 * 
 * Second
 * Project: Nano 33 BLE Camera Person Sensor
 * Board:   Nano 33 BLE
 * Started: 11.02.2024
 * Edited:  11.02.2024
 * Edge Impulse: https://studio.edgeimpulse.com/studio/348147/
 * 
**************************************************************/
#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>

#include <Nano_33_BLE_Person_Sensor_inferencing.h>

#include <Arduino_OV767X.h>    // https://github.com/arduino-libraries/Arduino_OV767X
                               // https://github.com/tinyMLx/arduino-library

#include "Tauno_LEDs.h"        // LEDs for the camera, on left and right

/* Constant variables ------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS     160
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS     120

#define DWORD_ALIGN_PTR(a)   ((a & 0x3) ?(((uintptr_t)a + 0x4) & ~(uintptr_t)0x3) : a)


/* Pins */
#define LEFT_LED_PIN   7  // D7
#define RIGHT_LED_PIN A7  // A7


/* LEDs: Status Colours
   DARK_RED, RED, ORANGE, YELLOW, GREEN, DARK_GREEN, BLUE
*/
#define SYSTEM_LED  GREEN
#define NIGHT_LED   DARK_GREEN
#define CAMERA_LED  RED
#define BELL_LED    DARK_RED
#define MOBILE_LED  YELLOW
#define HUMAN_LED   BLUE
#define DATA_LED    ORANGE


/* Data print */
const int DATA_PRINT_INTERVAl = 2000;  // 2seconds
uint32_t Prev_data_print = 0;
bool Is_data_print = false;


/* ML model*/
#define MOB_GAT_ID       1
#define MOBILE_THRESHOLD 0.75
#define INFERENCE_DELAY  500
bool Is_run_inferencing = true;

/* Init objects */
Tauno_LEDs Light(LEFT_LED_PIN, RIGHT_LED_PIN);


/* Edge Impulse ------------------------------------------------------------- */
class OV7675 : public OV767X {
    public:
        int begin(int resolution, int format, int fps);
        void readFrame(void* buffer);

    private:
        int vsyncPin;
        int hrefPin;
        int pclkPin;
        int xclkPin;

        volatile uint32_t* vsyncPort;
        uint32_t vsyncMask;
        volatile uint32_t* hrefPort;
        uint32_t hrefMask;
        volatile uint32_t* pclkPort;
        uint32_t pclkMask;

        uint16_t width;
        uint16_t height;
        uint8_t bytes_per_pixel;
        uint16_t bytes_per_row;
        uint8_t buf_rows;
        uint16_t buf_size;
        uint8_t resize_height;
        uint8_t *raw_buf;
        void *buf_mem;
        uint8_t *intrp_buf;
        uint8_t *buf_limit;

        void readBuf();
        int allocate_scratch_buffs();
        int deallocate_scratch_buffs();
};

typedef struct {
	size_t width;
	size_t height;
} ei_device_resize_resolutions_t;

/**
 * @brief      Check if new serial data is available
 *
 * @return     Returns number of available bytes
 */
int ei_get_serial_available(void) {
    return Serial.available();
}

/**
 * @brief      Get next available byte
 *
 * @return     byte
 */
char ei_get_serial_byte(void) {
    return Serial.read();
}

/* Private variables ------------------------------------------------------- */
static OV7675 Cam;
static bool is_initialised = false;

/*
** @brief points to the output of the capture
*/
static uint8_t *ei_camera_capture_out = NULL;
uint32_t resize_col_sz;
uint32_t resize_row_sz;
bool do_resize = false;
bool do_crop = false;

static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) ;
int calculate_resize_dimensions(uint32_t out_width, uint32_t out_height, uint32_t *resize_col_sz, uint32_t *resize_row_sz, bool *do_resize);
void resizeImage(int srcWidth, int srcHeight, uint8_t *srcImage, int dstWidth, int dstHeight, uint8_t *dstImage, int iBpp);
void cropImage(int srcWidth, int srcHeight, uint8_t *srcImage, int startX, int startY, int dstWidth, int dstHeight, uint8_t *dstImage, int iBpp);


/*
Function to test hardware elements one by one
*/
void tests() {
  Light.test();
}


void setup() {
  Serial.begin(115200);

  // Setup Pins
  Light.begin();

  // comment out the below line to cancel the wait for USB connection (needed for native USB)
  while (!Serial);
  Serial.println("Edge Impulse Inferencing Demo");

 // summary of inferencing settings (from model_metadata.h)
  ei_printf("Inferencing settings:\n");
  ei_printf("\tImage resolution: %dx%d\n", EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);
  ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
  ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));

  tests();  // Test hardware elements
}  // setup end


void loop() {
  uint32_t time_now = millis();

  while (Is_run_inferencing) {
    time_now = millis();

    ei_printf("\nStarting inferencing...\n");
    // instead of wait_ms, we'll wait on the signal,
    // this allows threads to cancel us
    if (ei_sleep(INFERENCE_DELAY) != EI_IMPULSE_OK) {
      break;
    }

    ei_printf("Taking photo...\n");

    if (ei_camera_init() == false) {
      ei_printf("ERR: Failed to initialize image sensor\r\n");
      break;
    }

    // choose resize dimensions
    uint32_t resize_col_sz;
    uint32_t resize_row_sz;
    bool do_resize = false;
    int res = calculate_resize_dimensions(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, &resize_col_sz, &resize_row_sz, &do_resize);
    if (res) {
      ei_printf("ERR: Failed to calculate resize dimensions (%d)\r\n", res);
      break;
    }

    void *snapshot_mem = NULL;
    uint8_t *snapshot_buf = NULL;
    snapshot_mem = ei_malloc(resize_col_sz*resize_row_sz*2);
    if (snapshot_mem == NULL) {
      ei_printf("failed to create snapshot_mem\r\n");
      break;
    }
    snapshot_buf = (uint8_t *)DWORD_ALIGN_PTR((uintptr_t)snapshot_mem);

    if (ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
      ei_printf("Failed to capture image\r\n");
      if (snapshot_mem) ei_free(snapshot_mem);
      break;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_cutout_get_data;

    // run the impulse: DSP, neural network and the Anomaly algorithm
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR ei_error = run_classifier(&signal, &result, debug_nn);
    if (ei_error != EI_IMPULSE_OK) {
      ei_printf("Failed to run impulse (%d)\n", ei_error);
      ei_free(snapshot_mem);
      break;
    }

    // print the predictions
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
              result.timing.dsp, result.timing.classification, result.timing.anomaly);
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    bool bb_found = result.bounding_boxes[0].value > 0;
    for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
      auto bb = result.bounding_boxes[ix];
      if (bb.value == 0) {
        continue;
      }

      ei_printf("    %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\n", bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
    }

    if (!bb_found) {
      ei_printf("    No objects found\n");
    }
#else
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
      ei_printf("    %s: %.5f\n", result.classification[ix].label,
                                  result.classification[ix].value);
    }

#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif
#endif
    /* // Tahan, et töötaks ka kui serial ei ole ühendatud
    while (ei_get_serial_available() > 0) {
      if (ei_get_serial_byte() == 'b') {
        ei_printf("Inferencing stopped by user\r\n");
        Is_run_inferencing = false; 
      }
    }
    */
    if (snapshot_mem) ei_free(snapshot_mem);
  }
  ei_camera_deinit();

}  // Loop 0 end




