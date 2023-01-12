
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

const byte MLX90640_address = 0x33; // Default 7-bit unshifted address of the MLX90640

#define TA_SHIFT 8 // Default shift for MLX90640 in open air

/**
 * ここから設定系
 */
#define SLEEP_SEC 15 // System ON sleepの秒数

#define DEBUG true // falseでI2C処理削除

#define PRINT_DEBUG false // falseでUSBシリアル削除

#define ENABLE_BLE true // BLE出力ON

#define ENABLE_SYS_OFF true          // SYSTEM OFF Sleepモード
#define SYS_OFF_WAKE_UP_FALLING true // SYSTEM OFF wakeup highかlowか

#define ENABLE_LOWPWR false // power mode を LOWPWRに設定(system off優先)

#define SOFT_DEVICE_FUNC_NO_USE true // softdevice関数を使わない（System offなど動作しないためtrue推奨）
/**
 * ここまで設定系
 */

static float mlx90640To[768];
paramsMLX90640 mlx90640;

// BLE Service
BLEDfu bledfu;   // OTA DFU service
BLEDis bledis;   // device information
BLEUart bleuart; // uart over ble
BLEBas blebas;   // battery

void startAdv(void);
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
boolean isConnected();

boolean is_sleeping = false;
boolean is_connected_ble = false;

extern "C" void TIMER2_IRQHandler(void)
{
  if ((NRF_TIMER2->EVENTS_COMPARE[0] != 0) &&
      ((NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE0_Msk) != 0))
  {
    NRF_TIMER2->EVENTS_COMPARE[0] = 0; // Clear compare register 0 event
  }
  is_sleeping = false;
#if PRINT_DEBUG
  Serial.println("good morning!");
#endif
}

void startTimer(unsigned long us)
{
  NRF_TIMER2->TASKS_STOP = 1;
  NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer; // Set the timer in Counter Mode
  NRF_TIMER2->TASKS_CLEAR = 1;              // clear the task first to be usable for later
  NRF_TIMER2->PRESCALER = 4;                // Set prescaler. Higher number gives slower
                                            // timer.
  NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_32Bit
                        << TIMER_BITMODE_BITMODE_Pos;
  NRF_TIMER2->CC[0] = us; // Set value for TIMER2 compare register 0

  // Enable interrupt on Timer 2, both for CC[0] and CC[1] compare match events
  NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Enabled
                         << TIMER_INTENSET_COMPARE0_Pos;
  // Clear the timer when COMPARE0 event is triggered
  NRF_TIMER2->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled
                       << TIMER_SHORTS_COMPARE0_CLEAR_Pos;

  NRF_TIMER2->TASKS_START = 1; // Start TIMER
  NVIC_EnableIRQ(TIMER2_IRQn);
}

void initThermal()
{
  digitalWrite(PIN_D8, HIGH); // I2C module Power ON
  delay(100);                 // Init I2C module
#if DEBUG

  Wire.begin();          // Init I2C lib
  Wire.setClock(400000); // Increase I2C clock speed to 400kHz

  if (isConnected() == false)
  {
#if PRINT_DEBUG
    Serial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
#endif
    while (1)
      ;
  }
#if PRINT_DEBUG
  Serial.println("MLX90640 online!");
#endif

  // Get device parameters - We only have to do this once
  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0)
#if PRINT_DEBUG
    Serial.println("Failed to load system parameters");
#endif

  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
#if PRINT_DEBUG
  if (status != 0)
    Serial.println("Parameter extraction failed");
#endif
#endif
}

void endThermal()
{

#if DEBUG
  Wire.end();
#endif
  digitalWrite(PIN_D8, LOW); // I2C module Power OFF

  pinMode(PIN_WIRE_SDA, INPUT); // remove internal pullup
  pinMode(PIN_WIRE_SCL, INPUT); // remove internal pullup
}

void setup()
{

#if PRINT_DEBUG
#else
  // UART off
  NRF_UART0->ENABLE = 0;
#endif

#if ENABLE_SYS_OFF
  pinMode(PIN_D3, INPUT);

  // bitmask
  NRF_GPIO->PIN_CNF[PIN_D3] &= ~((uint32_t)GPIO_PIN_CNF_SENSE_Msk);

#if SYS_OFF_WAKE_UP_FALLING
  // FALLINGで起床
  NRF_GPIO->PIN_CNF[PIN_D3] |= ((uint32_t)GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
#else
  // RISINGで起床
  NRF_GPIO->PIN_CNF[PIN_D3] |= ((uint32_t)GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);
#endif

#endif

#if PRINT_DEBUG
  Serial.begin(9600);
  Serial.println("---------------------------\n");
#endif

  pinMode(PIN_D8, OUTPUT);

#if PRINT_DEBUG
#if CFG_DEBUG
  // Blocking wait for connection when debug mode is enabled via IDE
  while (!Serial)
    yield();
#endif

  Serial.println("Bluefruit52 BLEUART Example");
  Serial.println("---------------------------\n");
#endif

  // Once params are extracted, we can release eeMLX90640 array

#if ENABLE_BLE
  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behavior, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(false);

  // Config the peripheral connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4); // Check bluefruit.h for supported values
  // Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
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
#endif

#if PRINT_DEBUG
  Serial.println("Please use Adafruit's Bluefruit LE app to connect in UART mode");
  Serial.println("Once connected, enter character(s) that you wish to send");
#endif
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
  Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
  Bluefruit.Advertising.start(0);             // 0 = Don't stop advertising after n seconds
}

void loop()
{
#if PRINT_DEBUG
  Serial.println("---------------------------\n");
#endif

  initThermal();
#if DEBUG
  for (byte x = 0; x < 2; x++) // Read both subpages
  {
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    if (status < 0)
    {
#if PRINT_DEBUG
      Serial.print("GetFrame Error: ");
      Serial.println(status);
#endif
    }

    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT; // Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
  }
#endif

  endThermal();

#if DEBUG
  for (int x = 0; x < sizeof(mlx90640To) / sizeof(mlx90640To[0]); x++)
  {
    char buf[17];

#if PRINT_DEBUG
    Serial.print("Pixel ");
    Serial.print(x);
    // Serial.print(": ");
    // Serial.print(mlx90640To[x], 2);
    // Serial.print("C");
    Serial.println();
#endif

    snprintf(buf, sizeof(buf), "pix %03d: %5.2fC\n", x, mlx90640To[x]);

#if PRINT_DEBUG
    Serial.print(buf);
#endif

#if ENABLE_BLE
    if (is_connected_ble)
    {

      bleuart.write(buf, sizeof(buf));
      // delay(100);

      if (x == (sizeof(mlx90640To) / sizeof(mlx90640To[0])) - 1)
      {
        delay(1000);
        is_sleeping = true;

        startTimer(SLEEP_SEC * 1000 * 1000);
      }
    }
    else
    {
      if (x == (sizeof(mlx90640To) / sizeof(mlx90640To[0])) - 1)
      {
        is_sleeping = true;

        startTimer(SLEEP_SEC * 1000 * 1000);
      }
    }
#else
    if (x == (sizeof(mlx90640To) / sizeof(mlx90640To[0])) - 1)
    {
      is_sleeping = true;
#if ENABLE_SYS_OFF
#else
      startTimer(SLEEP_SEC * 1000 * 1000);
#endif
    }
#endif
  }
#else
  is_sleeping = true;

  startTimer(SLEEP_SEC * 1000 * 1000);
#endif

#if PRINT_DEBUG
  // Forward data from HW Serial to BLEUART
  // while (Serial.available())
  //{
  //  // Delay to wait for enough input, since we have a limited transmission buffer
  //  delay(2);

  //  uint8_t buf[64];
  //  int count = Serial.readBytes(buf, sizeof(buf));
  //  bleuart.write( buf, count );
  //}

  // Forward from BLEUART to HW Serial
  // while ( bleuart.available() )
  //{
  //  uint8_t ch;
  //  ch = (uint8_t) bleuart.read();
  //  Serial.write(ch);
  //}
#endif

  // delay(1000);
  //  gotoSleep();
  if (is_sleeping)
  {
#if PRINT_DEBUG
    Serial.println("good night!!!!!");
#endif
    delay(100);
  }
#if ENABLE_SYS_OFF
#if SOFT_DEVICE_FUNC_NO_USE
  NRF_POWER->SYSTEMOFF = 1;
#else
  sd_power_system_off();
#endif
#elif ENABLE_LOWPWR
#if SOFT_DEVICE_FUNC_NO_USE
  NRF_POWER->TASKS_LOWPWR = 1;
#else
  sd_power_mode_set(NRF_POWER_MODE_LOWPWR); // CPUスリープ中のパワーモード
#endif
#else
  while (is_sleeping)
  {
    // sd_app_evt_wait();
    __WFE();
    __SEV();
    __WFE();
  }
#endif
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection *connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = {0};
  connection->getPeerName(central_name, sizeof(central_name));

#if PRINT_DEBUG
  Serial.print("Connected to ");
  Serial.println(central_name);
#endif

  is_connected_ble = true;
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void)conn_handle;
  (void)reason;

#if PRINT_DEBUG
  Serial.println();
  Serial.print("Disconnected, reason = 0x");
  Serial.println(reason, HEX);
#endif

  is_connected_ble = false;
}

// Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected()
{
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0)
    return (false); // Sensor did not ACK
  return (true);
}