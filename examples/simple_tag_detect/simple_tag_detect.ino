/*
 * AprilTag detector demo on AI Thinker ESP32-CAM
 * Created by gvl610
 * Based on https://github.com/AprilRobotics/apriltag
 * with some modifications (for adaption and performance)
 */

//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
//            You must select partition scheme from the board menu that has at least 3MB APP space.
//            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15 
//            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well

// ===================
// Select camera model
// ===================
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
//#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
// ** Espressif Internal Boards **
//#define CAMERA_MODEL_ESP32_CAM_BOARD
//#define CAMERA_MODEL_ESP32S2_CAM_BOARD
//#define CAMERA_MODEL_ESP32S3_CAM_LCD
//#define CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3 // Has PSRAM
//#define CAMERA_MODEL_DFRobot_Romeo_ESP32S3 // Has PSRAM

// Hardware-specific headers
#include "esp_camera.h"
#include "camera_pins.h"

// Apriltag headers
#include "apriltag.h"
#include "tag16h5.h"
#include "common/image_u8.h"
#include "common/zarray.h"

/*
 * Define this macro to enable debug mode
 * Level 0: Absolutely no debug at all. Suitable for
 *  production use.
 * Level 1: Simple debug messages , like simple events
 *  notifications.
 * Level 2: Low level debug messages
 * Level 3: Debug messages that in a loop
 */
#define DEBUG 2

void setup() {
  // Init serial
  Serial.begin(115200);
#if DEBUG >= 2
  Serial.setDebugOutput(true);
#endif

  // Show some signs of life
#if DEBUG >= 1
  Serial.println("AprilTag detector demo on ESP32-CAM");
  Serial.println("Created by gvl610");
  Serial.println("Based on https://github.com/AprilRobotics/apriltag");
  Serial.println("with some modifications (for adaption and performance)");
  Serial.print("Init PSRAM... ");
#endif

  // Init PSRAM
  psramInit();
#if DEBUG >= 1
  Serial.println("done");
  Serial.print("Memory available in PSRAM: ");
  Serial.println(ESP.getFreePsram());
#endif

  // Create camera config structure
  camera_config_t config;

  // Set camera pins (as defined in cam_pins.h)
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  // Set clock frequency
  config.xclk_freq_hz = 20000000;

  // Set frame config
  config.frame_size = FRAMESIZE_HVGA;
  config.pixel_format = PIXFORMAT_GRAYSCALE; // Required for AprilTag processing
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // Init camera
#if DEBUG >= 1
    Serial.print("Init camera... ");
#endif
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    // Cannot init camera
#if DEBUG >= 1
    Serial.printf("Camera init failed with error 0x%x\n", err);
#endif

    // Reset (cannot let it die here)
    ESP.restart();
  }

  // Specific configs for different camera
  sensor_t * s = esp_camera_sensor_get();
  // Initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM) || defined(CAMERA_MODEL_AI_THINKER)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

  // Custom camera configs should go here
  //s->set_framesize(s, FRAMESIZE_QVGA);

  // Done init camera
#if DEBUG >= 1
  Serial.println("done");
  Serial.print("Memory available in PSRAM: ");
  Serial.println(ESP.getFreePsram());
#endif

  // Setup AprilTag detection
#if DEBUG >= 1
  Serial.print("Init AprilTag detector... ");
#endif

  // Create tag family object
  apriltag_family_t *tf = tag16h5_create();

  // Create AprilTag detector object
  apriltag_detector_t *td = apriltag_detector_create();

  // Add tag family to the detector
  apriltag_detector_add_family(td, tf);
  
  // Tag detector configs
  // quad_sigma is Gaussian blur's sigma
  // quad_decimate: small number = faster but cannot detect small tags
  //                big number = slower but can detect small tags (or tag far away)
  // With quad_sigma = 1.0 and quad_decimate = 4.0, ESP32-CAM can detect 16h5 tag
  // from the distance of about 1 meter (tested with tag on screen. not on paper)
  td->quad_sigma = 1.0;
  td->quad_decimate = 4.0;
  td->refine_edges = 0;
  td->decode_sharpening = 0;
  td->nthreads = 1;
  td->debug = 0;

  // Done init AprilTag detector
#if DEBUG >= 1
  Serial.println("done");
  Serial.print("Memory available in PSRAM: ");
  Serial.println(ESP.getFreePsram());
  Serial.println("Start detecting...");
#endif

  // Main detecting loop (we just ignore the loop() function)
  while (true) {
    // Get a frame from camera
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
#if DEBUG >= 3
      Serial.println("Failed to get frame! Retrying...");
#endif
      continue;
    }

    // Convert our framebuffer to detector's input format
#if DEBUG >= 3
    Serial.println("Got a frame, converting it to detector's input format... ");
#endif
    image_u8_t im = {
      .width = fb->width,
      .height = fb->height,
      .stride = fb->width,
      .buf = fb->buf
    };
#if DEBUG >= 3
    Serial.println("done");
    Serial.println("Detecting... ");
#endif

    // Detect
    zarray_t *detections = apriltag_detector_detect(td, &im);
#if DEBUG >= 3
    Serial.println("done. Result:");
#endif

    // Print result
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        Serial.print(det->id);
        Serial.print(", ");
    }
    
    Serial.println("");

    // Cleaning up
#if DEBUG >= 3
    Serial.print("Memory available in PSRAM: ");
    Serial.println(ESP.getFreePsram());
    Serial.println("Cleaning up... ");
#endif
    // Free detection result object
    apriltag_detections_destroy(detections);

    // Return camera framebuffer to the camera driver
    esp_camera_fb_return(fb);

    // Display time needed per frame
    double t =  timeprofile_total_utime(td->tp) / 1.0E3;
    Serial.printf("t: %12.3f\n", t);
  }
}

void loop() {
  // Nothing here, the real loop is already at the end of setup())
}
