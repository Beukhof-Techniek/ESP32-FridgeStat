/**
 * ESP32 Refrigerator Thermostat with Signal K support
 *
 * Reads the temperature via a Dallas DS18B20 sensor and outputs the actual temperature in K
 * to a SignalK server as "environment.inside.refrigerator.temperature"
 * Meanwhile displaying the temperature in degrees C on the OLED display and driving
 * an opto coupler to control (on/off) the refrigerator's compressor.
 * The on-board LED indicates the on/off state of the compressor.
 * Setting the desired temperature by pressing the encoder button for at least 1 sec to get into edit mode (square is
 * drawn around the display's edge),
 * choosing the desired temperature by rotating the encoder (not too fast due to the limited processing capabilities
 * of the esp32) and pressing the buton for at least 1 sec to save the new value. Now the display returns to
 * displaying the actual temperature.
 * Edit mode automatically times out after 10 seconds and the display returns to displaying the actual temperature
 * without saving any changed.
 *
 * You can use this source file as a basis for your own projects.
 * Remove the parts that are not relevant to you, and add your own code
 * for external hardware libraries.
 */

#include "sensesp_app_builder.h"
#include "sensesp/signalk/signalk_output.h"
#include <Preferences.h>
#include "sensesp_onewire/onewire_temperature.h"
#include <U8g2lib.h>

/*************************************************************************************
 ************************************ PARAMETERS *************************************
 *************************************************************************************/

/**
 * Temperature max and min settings (limits the selectable range)
 */
const int temperature_min       = 5;
const int temperature_max       = 99;
const int temperatureHysteresis = 1;   // Hysteresis of the ON/OFF switching (gives .5 times this value in degrees below/above the desired temp to switch on/off)

/**
 * Compressor ON and OFF times are there to limit the time the compressor runs before taking a time-out
 * e.g. cools down for a bit regardless whether the desired temperature has been reached. The OFF time
 * represents the minimum amount of time the compressor cools down before starting up again. 
 */
const int compressorOFFMinimum  = 120; // Minimum time in seconds the compressor will be switched OFF (cooling down period)
const int compressorONMaximum   = 600; // Maximum time in seconds the compressor will be switched ON (maximum run time)

/**
 * Network settings.
 * Replace with your own network credentials
 */
const char *wifi_ssid = "your-wifi-devices-ssid";
const char *wifi_password = "your-wifi-devices-password";
const char *hostname = "esp32-fridgestat"; // ESP32 Refrigerator Thermostat (or whatever name you think is valid)

/**
 * Signal K client config
 * Important note: when changing the Signal K server address/connecting to a different Signal K server
 * and therefore changing the sk_server_address value below, you need to CLEAR THE FLASH from the ESP
 * as well because SensESP stores this value in its config path and prefers that value over whatever
 * you feed the builder constructor below.
 */
const char *sk_server_address = "192.168.20.1";
const uint16_t sk_server_port = 3000;

/*************************************************************************************
 ********************************** END PARAMETERS ***********************************
 *************************************************************************************/

using namespace sensesp;
using namespace sensesp::onewire;

#define CLK_PIN     0 // ESP32 pin GPIO0 connected to the rotary encoder's CLK pin
#define DT_PIN      1 // ESP32 pin GPIO1 connected to the rotary encoder's DT  pin
#define SW_PIN      2 // ESP32 pin GPIO2 connected to the rotary encoder's SW  pin
#define DS18B20_PIN 3 // Temperature sensor
#define OPTO_PIN    4 // Optocoupler LED
#define SDA_PIN     5 // I2C SDA for the display
#define SCL_PIN     6 // I2C SCL for the display
#define LED_PIN     8 // LOW == ON / HIGH == OFF

int   temperatureValueEdited;
int   temperatureValueSaved  = 0;
long  temperatureValueActual;
int   compressorStatus       = 1;   // Starting at the OFF position
ulong compressorTimeON;
ulong compressorTimeOFF      = 0;

int buttonState;
int buttonStatePrevious = HIGH;
volatile int encoderCLKState;
volatile int encoderDTState;
int encoderCLKStatePrevious = HIGH;

bool EDIT_mode = false;
ulong EDIT_timeout;

Preferences preferences;

// Instantiate the oled display
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, SCL_PIN, SDA_PIN);
// Width & Height of the 0.42 inch display
int oledWidth = 72;
int oledHeight = 40;
// Offsets to get the output at the correct position in the 128x64 buffer
// Horizontally the output needs to be centered (28+72+28=128)
// Vertically the output needs to be at the bottom line (24+40=64)
int oledXOffset = 28;
int oledYOffset = 24;

/**
 * Enter (enable) EDIT mode.
 * 
 * Takes the previously saved temperature value and puts it into
 * temperatureValueEdited.
 * Sets the EDIT timeout value to the current millisecond.
 */
void IRAM_ATTR editSettings() {
  EDIT_mode = true;
  temperatureValueEdited = temperatureValueSaved;
  ESP_LOGI(__FILE__, "EDIT mode ON\n");
  EDIT_timeout = millis();
}

/**
 * Store the EDITed value.
 * 
 * Writes the EDITed value into flash under "ins_refr_temp"
 * and exits (disables) EDIT mode.
 */
void IRAM_ATTR saveSettings() {
  EDIT_mode = false;
  temperatureValueSaved = temperatureValueEdited;
  preferences.putInt("ins_refr_temp", temperatureValueSaved);
  ESP_LOGI(__FILE__, "Saved settings\n");
}

/**
 * Encoder button state handler.
 * 
 * While the button is pressed (state == LOW) this function
 * counts to 1 second to enter EDIT mode or (when already in
 * EDIT mode) calls for the function which saves the adjusted
 * settings.
 */
void IRAM_ATTR onButtonPressed() {
  ESP_LOGI(__FILE__, "Button pressed\n");

  ulong buttonPressStart = millis();

  while (digitalRead(SW_PIN) == LOW) {
    if (millis() - buttonPressStart >= 1000) {
      if (!EDIT_mode) {
        editSettings();
      } else {
        saveSettings();
      }
      break;
    }
    yield();
  }
}

/**
 * Rotary encoder state handler
 *
 * Adjusts the desired temperature value (temperatureValueEdited) between
 * temperature_min and temperature_max according to the rotary direction
 * of the encoder.
 * This function is called when the encoder's CLK pin is LOW so only DT
 * state needs to be checked to determine the rotary direction.
 * When rotated, the EDIT timeout value is (re)set to the current millisecond.
 *
 * @param encoderDTState State of the encoder's DT connection.
 */
void IRAM_ATTR onEncoderRotated(int encoderDTState) {
  if (encoderDTState == HIGH) {
    // if the DT state is (still) HIGH
    // the encoder is rotating in clockwise direction => increase the desired temperature
    if (temperatureValueEdited < temperature_max) {
      temperatureValueEdited++;
    }
  } else {
    // if the DT state is (already) LOW
    // the encoder is rotating in counter-clockwise direction => decrease the desired temperature
    if (temperatureValueEdited > temperature_min) {
      temperatureValueEdited--;
    }
  }
  EDIT_timeout = millis();
}

/**
 * Thermostat function.
 *
 * This function controls the ON / OFF state of the refrigerator compressor by checking:
 * a) the desired (stored) temperature against the actual (measured) temperature, and
 * b) the amount of time that the compressor has already been ON or OFF
 */
void thermostat() {
  if (compressorStatus == 0 && (temperatureValueActual - (0.5 * temperatureHysteresis) <= temperatureValueSaved ||
                                millis() - compressorTimeON >= (compressorONMaximum * 1000))) {
    compressorStatus = 1;
    compressorTimeOFF = millis();
    ESP_LOGI(__FILE__, "Compressor OFF\n");
  } else if (compressorStatus == 1 &&
             temperatureValueActual + (0.5 * temperatureHysteresis) >= temperatureValueSaved &&
             (millis() - compressorTimeOFF >= (compressorOFFMinimum * 1000) || compressorTimeOFF == 0)) {
    compressorStatus = 0;
    compressorTimeON = millis();
    ESP_LOGI(__FILE__, "Compressor ON\n");
  }
  digitalWrite(OPTO_PIN, compressorStatus);
  digitalWrite(LED_PIN,  compressorStatus);
}

/**
 * Setup function
 */
void setup() {
  SetupLogging();

  // Create the global SensESPApp() object.
  SensESPAppBuilder builder;
  sensesp_app = builder.get_app();

  // configure encoder pins as inputs
  pinMode(CLK_PIN,  INPUT);
  pinMode(DT_PIN,   INPUT);
  pinMode(SW_PIN,   INPUT);
  pinMode(OPTO_PIN, OUTPUT);
  pinMode(LED_PIN,  OUTPUT);

  digitalWrite(OPTO_PIN, HIGH);
  digitalWrite(LED_PIN,  HIGH);

  // OLED display
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB18_tf);

  // Retrieve the last known engine_running_time value
  if (preferences.begin("environment", false)) {
    // When opening the preferences returns a false, the value of temperatureValueSaved will remain 0
    // which will "block" the storage of the parameter further on. Also it will be kept at 0 and shown
    // as such (and the text "parameter error") in the displays so the user will know there is something
    // wrong with the device.
    temperatureValueSaved = preferences.getInt("ins_refr_temp", 10); // Set to default at 10 to get it off the "error" value of 0
  }

  /*
     Tell SensESP where the sensor is connected to the board
     ESP32 pins are specified as just the X in GPIOX
  */
  DallasTemperatureSensors* dts = new DallasTemperatureSensors(DS18B20_PIN);

  // Define how often SensESP should read the sensor(s) in milliseconds
  const unsigned int sensor_read_interval = 5000; // Define how often (in milliseconds) new samples are acquired

   // Measure temperature of refrigerator
  auto* fridge_temp = new OneWireTemperature(dts, sensor_read_interval);

  /**
   * Add observer that stores the current value of the obtained data in degrees Celcius every time it changes.
   */
  fridge_temp->attach([fridge_temp]() {
    temperatureValueActual = (fridge_temp->get() - 273);
  });

  // /vessels/<RegExp>/environment/inside/[A-Za-z0-9]+/temperature
  // Units: K (Kelvin)
  // Description: This regex pattern is used for validation of the identifier for the environmental zone, eg. engineRoom, mainCabin, refrigerator
  // Description: Temperature
  auto* fridge_temp_sk_output = new SKOutputFloat("environment.inside.refrigerator.temperature", "", new SKMetadata("K"));
}

/**
 * Main program loop
 */
void loop() {
  if (!EDIT_mode) {
    event_loop()->tick();
  }

  buttonState = digitalRead(SW_PIN);
  if (buttonState != buttonStatePrevious) {
    if (buttonState == LOW) {
      onButtonPressed();
    }
    buttonStatePrevious = buttonState;
  }

  /**
   * Simple handling of the rotary encoder state. This could be done via interrupt
   * but the ESP32 that was used wasn't fast enough to determine the rotary
   * direction. Perhaps with a future version this can be handled more efficient.
   */
  if (EDIT_mode) {
    noInterrupts();
    encoderCLKState = digitalRead(CLK_PIN);
    encoderDTState  = digitalRead(DT_PIN);
    interrupts();
    if (encoderCLKState != encoderCLKStatePrevious) {
      if (encoderCLKState == LOW) {
        onEncoderRotated(encoderDTState);
      }
    }
  }
  encoderCLKStatePrevious = encoderCLKState;

  // Timeout of EDIT mode after 10 seconds
  if (EDIT_mode && millis() - EDIT_timeout >= 10000) {
    EDIT_mode = false;
    digitalWrite(LED_PIN, HIGH);
    ESP_LOGI(__FILE__, "EDIT mode timeout\n");
  }

  // The actual thermostat function
  thermostat();

  // Display information
  u8g2.clearBuffer();
  char temperatureBuffer[6];
  if (EDIT_mode) {
    // Draw a frame to indicate EDIT mode is enabled
    u8g2.drawFrame(oledXOffset, oledYOffset, oledWidth, oledHeight);
    sprintf(temperatureBuffer, "%d%cC", temperatureValueEdited, 0xB0);
    u8g2.setCursor(oledXOffset + (temperatureValueEdited>=10?7:15), oledYOffset + 29);
  } else {
    sprintf(temperatureBuffer, "%d%cC", (int)temperatureValueActual, 0xB0);
    u8g2.setCursor(oledXOffset + (temperatureValueActual>=10?7:15), oledYOffset + 29);
  }
  u8g2.print(temperatureBuffer);
  u8g2.sendBuffer();
}
