
/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <Wire.h>

#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640

#define TA_SHIFT 8 //Default shift for MLX90640 in open air

static float mlx90640To[768];
paramsMLX90640 mlx90640;

// BLE Service
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery

#define WAKE_LOW_PIN  PIN_BUTTON1
//#define WAKE_HIGH_PIN PIN_A1

#define SLEEPING_DELAY 5000                                // sleep after 30 seconds of blinking

void startAdv(void);
void gotoSleep(unsigned long time);
void gotoSystemOnSleep(unsigned long time);
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
boolean isConnected();

boolean is_sleeping = false;
boolean is_connected_ble = false;


extern "C" void TIMER2_IRQHandler(void) {
  if ((NRF_TIMER2->EVENTS_COMPARE[0] != 0) &&
      ((NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE0_Msk) != 0)) {
    NRF_TIMER2->EVENTS_COMPARE[0] = 0;  // Clear compare register 0 event
  }
  is_sleeping = false;
  Serial.println("good morning!");
}

void startTimer(unsigned long us) {
  NRF_TIMER2->TASKS_STOP = 1;
  NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;  // Set the timer in Counter Mode
  NRF_TIMER2->TASKS_CLEAR = 1;  // clear the task first to be usable for later
  NRF_TIMER2->PRESCALER = 4;    // Set prescaler. Higher number gives slower
                                // timer.
  NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_32Bit
                        << TIMER_BITMODE_BITMODE_Pos;
  NRF_TIMER2->CC[0] = us;  // Set value for TIMER2 compare register 0

  // Enable interrupt on Timer 2, both for CC[0] and CC[1] compare match events
  NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Enabled
                         << TIMER_INTENSET_COMPARE0_Pos;
  // Clear the timer when COMPARE0 event is triggered
  NRF_TIMER2->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled
                       << TIMER_SHORTS_COMPARE0_CLEAR_Pos;

  NRF_TIMER2->TASKS_START = 1;  // Start TIMER
  NVIC_EnableIRQ(TIMER2_IRQn);
}

void setup()
{

  // sleep before BLE Advertisement
  delay(1000*30);
  //is_sleeping = true;
  //startTimer(30 * 1000 * 1000);
  //while(is_sleeping) {
  //  __SEV();
  //  __WFE();
  //  __WFE();
  //}

  Serial.begin(115200);
  Serial.println("wakeUp\n");
  //Wire.begin();
  //Wire.setClock(400000); //Increase I2C clock speed to 400kHz

#if CFG_DEBUG
  // Blocking wait for connection when debug mode is enabled via IDE
  while ( !Serial ) yield();
#endif
  
  Serial.println("Bluefruit52 BLEUART Example");
  Serial.println("---------------------------\n");

  //if (isConnected() == false)
  //{
  //  Serial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
  //  while (1);
  //}
  //Serial.println("MLX90640 online!");

  //Get device parameters - We only have to do this once
  //int status;
  //uint16_t eeMLX90640[832];
  //status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  //if (status != 0)
  //  Serial.println("Failed to load system parameters");

  //status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  //if (status != 0)
  //  Serial.println("Parameter extraction failed");

  //Once params are extracted, we can release eeMLX90640 array

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behavior, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  // Set up and start advertising
  startAdv();

  Serial.println("Please use Adafruit's Bluefruit LE app to connect in UART mode");
  Serial.println("Once connected, enter character(s) that you wish to send");
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

void loop()
{
  digitalWrite(LED_BUILTIN, LOW);

  //for (byte x = 0 ; x < 2 ; x++) //Read both subpages
  //{
  //  uint16_t mlx90640Frame[834];
  //  int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
  //  if (status < 0)
  //  {
  //    Serial.print("GetFrame Error: ");
  //    Serial.println(status);
  //  }

  //  float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
  //  float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

  //  float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
  //  float emissivity = 0.95;

  //  MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
  //}

  //for (int x = 0 ; x < sizeof(mlx90640To) / sizeof(mlx90640To[0]); x++)
  //{
  //  char buf[17];

  //  //Serial.print("Pixel ");
  //  //Serial.print(x);
  //  //Serial.print(": ");
  //  //Serial.print(mlx90640To[x], 2);
  //  //Serial.print("C");
  //  //Serial.println();

  //  snprintf(buf, sizeof(buf), "pix %03d: %5.2fC\n", x, mlx90640To[x]);
  //  Serial.print(buf);

  //  if(is_connected_ble) {

  //    bleuart.write(buf, sizeof(buf));
  //    //delay(100);

  //    if(x == (sizeof(mlx90640To) / sizeof(mlx90640To[0])) -1) {
  //      delay(1000);
  //      is_sleeping = true;

  //      startTimer(10 * 1000 * 1000);
  //    }
  //  }

  //}
    if(is_connected_ble) {

      bleuart.write("test", sizeof("test"));
        delay(1000);
        is_sleeping = true;

        startTimer(30 * 1000 * 1000);
    }


  // Forward data from HW Serial to BLEUART
  //while (Serial.available())
  //{
  //  // Delay to wait for enough input, since we have a limited transmission buffer
  //  delay(2);

  //  uint8_t buf[64];
  //  int count = Serial.readBytes(buf, sizeof(buf));
  //  bleuart.write( buf, count );
  //}

  // Forward from BLEUART to HW Serial
  while ( bleuart.available() )
  {
    uint8_t ch;
    ch = (uint8_t) bleuart.read();
    Serial.write(ch);
  }

  delay(1000);

  digitalWrite(LED_BUILTIN, HIGH);
  //gotoSleep();
  if(is_sleeping) {
    Serial.println("good night!!!!!");
    delay(100);
    NVIC_SystemReset();
    //systemOff(PIN_BUTTON1, WAKE_LOW_PIN);
  }
  while(is_sleeping) {
    __SEV();
    __WFE();
    __WFE();
  }

}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);

  is_connected_ble = true;
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);

  is_connected_ble = false;
}

//Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected()
{
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}