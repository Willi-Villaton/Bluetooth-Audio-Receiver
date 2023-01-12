#include "BluetoothA2DPSink32.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//Definig all pin numbers used
#define STATUS_LED 19
#define CONTEXT_LED 18
#define BUTTON 4

//Defining the bluetooth module globally
BluetoothA2DPSink a2dp_sink;

//Defining all the things needed for the shutdown timers
esp_a2d_connection_state_t last_state;
uint16_t minutes = 5;
unsigned long shutdown_ms = millis() + 1000 * 60 * minutes;


// ------------------------------------------- INIT FUNCTIONS -------------------------------------------

/*
 * This function initialises all the pins needed in this program.
 */
void pin_init() {
  i2s_pin_config_t i2s_config = {
    .bck_io_num = 22,
    .ws_io_num = 23,
    .data_out_num = 21,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  a2dp_sink.set_pin_config(i2s_config);

  //Custom IO-Pins
  pinMode(STATUS_LED, OUTPUT);
  pinMode(CONTEXT_LED, OUTPUT);
  pinMode(BUTTON, INPUT);
}


/*
 * This function initialises the ESP32s I2S Interface used to communicate with the DAC Board.
 */ 
void i2s_init() {
  const static i2s_config_t i2s_config = {
      .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = 44100, // updated automatically by A2DP
      .bits_per_sample = (i2s_bits_per_sample_t) 32,
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .communication_format = (i2s_comm_format_t) (I2S_COMM_FORMAT_STAND_I2S),
      .intr_alloc_flags = 0, // default interrupt priority
      .dma_buf_count = 8,
      .dma_buf_len = 64,
      .use_apll = true,
      .tx_desc_auto_clear = true // avoiding noise in case of data unavailability
  };

  a2dp_sink.set_i2s_config(i2s_config);
}


// ------------------------------------------- UTIL FUNCTIONS -------------------------------------------

/*
 * This function resets the timer of the inactivity kill switch.
 */
void reset_shutdown() {
  shutdown_ms = millis() + 1000 * 60 * minutes; 
}


/*
 * This function toggles on and off the specified leds
 */
void blink_led(uint8_t led) {

  if (led < 12 && led > 33) {
    return;
  }

  //Stupid integration of a blink
  digitalWrite(led, LOW);
  delay(250);
  digitalWrite(led, HIGH);
  delay(250);
  digitalWrite(led, LOW);
  delay(250);
  digitalWrite(led, HIGH);
  delay(250);
  digitalWrite(led, LOW);
  delay(250);
  digitalWrite(led, HIGH);
  delay(250);
}

/*
 * This function includes an inactivity check / idle timeout.
 */
void checkout_inactive() {

  // Check for timeout
  if (millis() > shutdown_ms){
    
    // stop the esp32
    Serial.println("Shutting down");
    blink_led(STATUS_LED);
    digitalWrite(STATUS_LED, LOW);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, HIGH);
    delay(1000);
    esp_deep_sleep_start();

  }

  // Check state
  esp_a2d_connection_state_t state = a2dp_sink.get_connection_state();
  if (last_state != state){
    bool is_connected = state == ESP_A2D_CONNECTION_STATE_CONNECTED;
    Serial.println(is_connected ? "Connected" : "Not connected");    
    if (!is_connected && digitalRead(CONTEXT_LED)) {

      //Change status to disconnected
      digitalWrite(CONTEXT_LED, LOW);

    } else {

      //Change status to connected
      blink_led(CONTEXT_LED);
      reset_shutdown();

    }
    last_state = state;
  }
}


/*
 * This function confirms the pincode for the device side.
 */
void confirm() {
  a2dp_sink.confirm_pin_code();
}


/*
 * This function resets the timer of the inactivity kill switch.
 */
void checkout_pincode() {
  if (a2dp_sink.pin_code() != 0 && digitalRead(BUTTON)) {
    blink_led(STATUS_LED);
    a2dp_sink.debounce(confirm, 5000);
  }
}


// ------------------------------------------- ESP MAIN FUNCTIONS -------------------------------------------

/*
 * This function initialises everything.
 */
void setup() {

  //Serial interface for debug
  Serial.begin(115200);
  delay(1000); //Take some time to open up the Serial Monitor

  Serial.println("Starting device...");
  
  //Initialising the hardware
  pin_init();
  i2s_init();

  //Setting up all a2dp_sink  parameters
  a2dp_sink.set_bits_per_sample(32);
  a2dp_sink.set_on_data_received(reset_shutdown);
  a2dp_sink.activate_pin_code(true);
  a2dp_sink.set_auto_reconnect(true);

  //Starting Bluetooth Module
  a2dp_sink.start("Peacemaker", false);
  Serial.println("The device started, now you can pair it with bluetooth!");

  //Setting STATUS to bright as long as running
  digitalWrite(STATUS_LED, HIGH);
}


/*
 * This function is run constantly.
 */
void loop() {

  checkout_pincode();
  checkout_inactive();
  
  // Only do 1 second cycles, cause it is not needed to be any faster
  delay(1000);
}