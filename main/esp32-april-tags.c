// Demo of how to use AT on the ESP
// More cpp examples of how to use the camera here:
// https://github.com/alanesq/esp32cam-demo/blob/master/esp32cam-demo.ino
#include <stdio.h>

// ESP32 includes
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_camera.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "sys/param.h"
#include "string.h"

// AprilTag includes
#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"

// Detect tags with up to this many bit errors.
#define MAX_BIT_ERRORS 1

#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

static const char *TAG = "example:take_picture";

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_GRAYSCALE, //YUV422,PIXFORMAT_RGB565,RGB565,JPEG
    .frame_size = FRAMESIZE_VGA,         // QVGA is 400x240
    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

static esp_err_t init_camera()
{
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "ERROR: Camera init failed with error 0x%x", err);
        return err;
    }

    return ESP_OK;
}

void get_and_detect_tags(apriltag_detector_t *td) {
    int total_quads = 0;
    const int hamm_hist_max = 10;

    int total_hamm_hist[hamm_hist_max];
    memset(total_hamm_hist, 0, sizeof(total_hamm_hist));
    double total_time = 0;

    int hamm_hist[hamm_hist_max];
    memset(hamm_hist, 0, sizeof(hamm_hist));

    ESP_LOGI(TAG, "Taking picture...");

    camera_fb_t *pic = esp_camera_fb_get();
    if (pic == NULL) {
        printf("Got NULL image");
        return;
    }

    ESP_LOGI(TAG, "Picture taken");

    image_u8_t at_image_data = {
        .width  = pic->width,
        .height = pic->height,
        .stride = pic->width,
        .buf    = pic->buf
    };

    image_u8_t *im = &at_image_data;

    zarray_t *detections = apriltag_detector_detect(td, im);
    ESP_LOGI(TAG, "Detected %d tags\n", zarray_size(detections));

    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
                i, det->family->nbits, det->family->h, det->id, det->hamming, det->decision_margin);

        hamm_hist[det->hamming]++;
        total_hamm_hist[det->hamming]++;
    }

    apriltag_detections_destroy(detections);

    ESP_LOGI(TAG, "Time profile: ");
    timeprofile_display(td->tp);

    total_quads += td->nquads;

    double t =  timeprofile_total_utime(td->tp) / 1.0E3;
    total_time += t;
    ESP_LOGI(TAG, "\nTotal Time: %f\n", total_time);

    esp_camera_fb_return(pic);  // TODO: is everything freed from im struct
}

void april_tags_run(void) {
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();

    apriltag_detector_add_family_bits(td, tf, MAX_BIT_ERRORS);

    td->quad_decimate = 3;
    td->quad_sigma = 0.0;
    td->nthreads = 2;
    td->debug = 0;
    td->refine_edges = 1;

    while (1) {
        get_and_detect_tags(td);
    }

    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);

}

void power_on_msg(void) {
    printf("Running ESP AprilTags Demo! :)\n");
    return;
}

void app_main()
{
    power_on_msg();

    if(ESP_OK != init_camera()) {
        return;
    }

    april_tags_run();

    return;
}
