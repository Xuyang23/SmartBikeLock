#include <Arduino.h>
#include <EEPROM.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <MFRC522.h>
// SARA5_controller.cpp
#include "SARA5_controller.h" //INSTAL SparkFun_u-blox_SARA-R5_Arduino_Library.h
							  //INSTALL <SoftwareSerial.h>

// UUIDs for BLE services and characteristics for UART communication
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define RST_PIN     22 // Configurable, choose a pin not used by other SPI devices
#define SS_PIN     5  // Configurable, VSPI SS pin by default on ESP32
#define BUZZER_PIN   12  // Choose an appropriate pin for the buzzer
#define WAKEUP_PIN_Tilt  4  // Tilt sensor pin to detect movement
#define WAKEUP_PIN_NFC  25 // NFC activation pin
#define sleep_time_second 10 // Sleep for 10s 
#define LED_PIN 2  // LED pin
#define analog_PIN 13 // Analog pin for light sensor

#define _powerPin 14 //PWR pin SARA R5
#define Extern_LED 33
#define Lock_PIN 32 // Pin to detect lock state

// Function declarations for various functionalities
// put function declarations here:
void BLE_config(void);
void Alarm_on(void);
void Alarm_off(void);
void LED_on(void);
void LED_off(void);
void NFC_config(void);
void wake_config_Unlock(void);
void wake_config_Stolen(void);
void wake_config_timer(void);
bool checkAccess(void);
void Thingspeak_config(void);
void ADC_config(void);
void auto_led(void);
void thingspeak_write(void);

// Global variables for BLE communication
BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

//Define object for sara
SARA5_controller SaraR5;

// Data structure to store data in EEPROM
struct data {
  const uint8_t addr_num_wakeup_tilt = 0;
  uint8_t num_wakeup_tilt;
};
// State enumeration for different modes of operation
typedef enum {
  Stolen,
  fall_asleep,
  unlock,
  debug
} States_t;

// Volatile variables for state management and flag settings
volatile States_t State = debug;
volatile data DATA;
volatile bool auto_led_flag = true;
volatile bool NFC_MATCHED = false;
volatile bool BLE_MATCHED = false;
bool initial_flag_stolen = false;
bool initial_flag_unlock = false;
bool initial_flag_stolen_TS = false;
bool BLE_config_flag = false;
unsigned long previousMillis = 0;  
const unsigned long interval = 60000;
uint8_t alarm_times = 0;

// NFC reader instance
MFRC522 mfrc522(SS_PIN, RST_PIN); // Create MFRC522 instance
byte accessUID[4] = {0xA1, 0xF0, 0xEA, 0x1D}; // Predefined UID of the authorized card
RTC_DATA_ATTR uint8_t wake_timer_Count = 0;

// BLE server callbacks for connection management
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("BLE connected!");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("BLE disconnected!");
      pServer->startAdvertising(); // restart advertising

    }
};

// BLE characteristic callbacks for handling received data
class MyCallbacks: public BLECharacteristicCallbacks {
  // This method is called when the BLE central device writes data to this characteristic
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();  // Retrieve the value written to the characteristic

      if (rxValue.length() > 0) { // Check if the received string is not empty
        Serial.println("*********");
        Serial.print("Received Value: ");  // Print the label
        // Loop through each character in the received string and print it
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);
        Serial.println();
        // Convert the first character of the received value to an integer to determine the command
        int command = rxValue[0]-'0';  // Convert ASCII to integer

        switch (command){
          case 0:{  // Command '0' to turn on the buzzer
          Alarm_on(); // Turn on the Buzzer
          String message = "Turn on the Buzzer\n";
          pTxCharacteristic->setValue(message.c_str());
          pTxCharacteristic->notify();  // Notify connected BLE client
          break;}
          case 1:{// Turn on the LED
            String message = "Turn on the LED\n";
          pTxCharacteristic->setValue(message.c_str());
          pTxCharacteristic->notify();
          LED_on(); // Activate the LED
          auto_led_flag = false;  // Disable automatic LED control
          break;}
          case 2:{ // Turn off the LED
            String message = "Turn off the LED\n";
          pTxCharacteristic->setValue(message.c_str());
          pTxCharacteristic->notify();  
          LED_off(); // Deactivate the LED
          auto_led_flag = false;  // Disable automatic LED control
          break;}
          case 3:{ // Unlock the bike
            String message = "Unlock the bike\n";
          pTxCharacteristic->setValue(message.c_str());
          pTxCharacteristic->notify();
          Alarm_off(); 
          State = unlock; // Change state to unlock
          BLE_MATCHED = true;  // Set BLE matched flag
          break;}
          case 4:{ // lock the bike
            String message = "Lock the bike\n";
          pTxCharacteristic->setValue(message.c_str());
          pTxCharacteristic->notify();  
          
          State = fall_asleep;   // Change state to fall asleep
          break;}
          case 5:{
            String message = "Auto LED on\n";
          pTxCharacteristic->setValue(message.c_str());
          pTxCharacteristic->notify();
          auto_led_flag = true;  // Enable automatic LED control
          break;}
          case 6:{
          Alarm_off();  // Deactivate the buzzer
          String message = "Turn off the buzzer\n";
          pTxCharacteristic->setValue(message.c_str());
          pTxCharacteristic->notify();
          break;}
          default: break;
        }
      }
    }
};

// Initial setup function, runs once
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);  // Set the buzzer pin as an output

  switch (esp_sleep_get_wakeup_cause()) 
  {
  case ESP_SLEEP_WAKEUP_EXT1: {
  uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
  delay(10);  // Short delay for stabilization
  if (wakeup_pin_mask != 0) 
  {  // If there is at least one pin that triggered the wakeup
    int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
    switch(pin){
      case WAKEUP_PIN_Tilt:  // If the tilt sensor triggered the wakeup
      State = Stolen;
      break;
      case WAKEUP_PIN_NFC:  // If the NFC sensor triggered the wakeup
      State =  unlock;  // Set state to unlock indicating an NFC action was detected
      break;
    }
  }
  else {
    Serial.println("Wake up from No reason\n");
    State = fall_asleep;  // Default to fall_asleep state if no specific pin was triggered
  }
  break;
  }
/*  case ESP_SLEEP_WAKEUP_TIMER:
  {
    Serial.printf("Wake up from timer. Time spent in deep sleep %d\n",wake_timer_Count);
    wake_timer_Count += 1;
    State = fall_asleep;  // Set state to fall asleep, typically to conserve power
    wake_config_timer();  // Configure the timer for the next sleep cycle
    thingspeak_write();
    break;
  }
  */
  default: break;
  }
}

// Main loop, runs repeatedly
void loop() {
  // State machine handling different states
  // put your main code here, to run repeatedly:
  switch(State){
  case Stolen: 
  // Handle stolen state
  if (!initial_flag_stolen)
  {
    wake_config_Stolen();
    initial_flag_stolen = true;
  }
  else{
  if (DATA.num_wakeup_tilt<15)
  {State = fall_asleep;}
  else
  {
    if(!BLE_config_flag)
    {
      BLE_config();
      BLE_config_flag = true;

    }
  // enable ThingSpeak
  if (!initial_flag_stolen_TS){
  // turn on the alarm
  Alarm_on();
  delay(500);
  // enable NFC detection
  NFC_config();
  Thingspeak_config();
  initial_flag_stolen_TS = true;
  
  }
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
       previousMillis = currentMillis;
       Serial.println("Sending data to thingspeak..");
    thingspeak_write();}
    
    NFC_MATCHED = checkAccess();
    if (NFC_MATCHED)
    {State = unlock;
    EEPROM.begin(4096);
    DATA.num_wakeup_tilt = 0;
    EEPROM.write(DATA.addr_num_wakeup_tilt, DATA.num_wakeup_tilt);
    EEPROM.commit();
    }
    }
    // Thingspeak part: send the message

    // *******************************************
  }
  break;
  case fall_asleep: 
  // Handle fall asleep state
  esp_sleep_enable_ext1_wakeup(BIT(WAKEUP_PIN_Tilt) | BIT(WAKEUP_PIN_NFC), ESP_EXT1_WAKEUP_ANY_HIGH );
  // esp_sleep_enable_timer_wakeup(sleep_time_second * 1000000);
  // only need to send the location message.
  
  // **************************************
  esp_deep_sleep_start();  
  break;

  case unlock: 
  // Handle unlock state
  if (!initial_flag_unlock)
  {
    wake_config_Unlock();
    initial_flag_unlock = true;
  }


  if(!BLE_config_flag)
  {
    BLE_config();
    BLE_config_flag = true;
  }

  else
  {
    if (!NFC_MATCHED)
    {
    for (int i=0 ;i<10 ;i++)
    {
      NFC_MATCHED = checkAccess();
      if (NFC_MATCHED || BLE_MATCHED){
      EEPROM.begin(4096);
      DATA.num_wakeup_tilt = 0;
      EEPROM.write(DATA.addr_num_wakeup_tilt, DATA.num_wakeup_tilt);
      EEPROM.commit();
      NFC_MATCHED = true;
      break;
      }
      LED_on();
      delay(500);
      LED_off();
      delay(500);
      Serial.print(i);
      if (i == 9)
      {State = Stolen;
      EEPROM.begin(4096);
      DATA.num_wakeup_tilt = 15;
      EEPROM.write(DATA.addr_num_wakeup_tilt, DATA.num_wakeup_tilt);
      EEPROM.commit();}
    }
    }
    else{ // the bike now is unlocked!

    pinMode(Extern_LED,OUTPUT);
    digitalWrite(Extern_LED,HIGH);
    if (auto_led_flag)
    {Serial.print("auto led mode\n");
    auto_led();
    }
    }
  }
  break;


  case debug: 
  // Handle debug state
  Serial.println("Debug mode\n");
  EEPROM.begin(4096);
  DATA.num_wakeup_tilt = 0;
  EEPROM.write(DATA.addr_num_wakeup_tilt, DATA.num_wakeup_tilt);
  EEPROM.commit();
  esp_sleep_enable_ext1_wakeup(BIT(WAKEUP_PIN_Tilt) | BIT(WAKEUP_PIN_NFC), ESP_EXT1_WAKEUP_ANY_HIGH);
  // esp_sleep_enable_timer_wakeup(sleep_time_second * 1000000); /* Conversion factor for micro seconds to seconds */
  esp_deep_sleep_start();
  break;
  }
  delay(1000);
}

// put function definitions here:
void BLE_config(void) {
  // Create the BLE Device
  BLEDevice::init("Bike Service");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
										CHARACTERISTIC_UUID_TX,
										BLECharacteristic::PROPERTY_NOTIFY
									);
                      
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
											 CHARACTERISTIC_UUID_RX,
											BLECharacteristic::PROPERTY_WRITE
										);

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  pServer->startAdvertising();
  Serial.println("Waiting a client connection to notify...");
  return;
}

void Alarm_on(void)
{
  digitalWrite(BUZZER_PIN,HIGH);
  return;
}

void Alarm_off(void)
{
  digitalWrite(BUZZER_PIN,LOW);
  return;
}

void LED_on(void)
{
  digitalWrite(LED_PIN,HIGH);
}

void LED_off(void)
{
  digitalWrite(LED_PIN,LOW);
}
void NFC_config(void)
{
  SPI.begin();       // Init SPI bus
  mfrc522.PCD_Init();    // Init MFRC522
  delay(40);         // Optional delay
  mfrc522.PCD_DumpVersionToSerial(); // Show details of PCD - MFRC522 Card Reader details
  Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));
  return;
}

// Configuration for stolen state
void wake_config_Stolen(void)
{
  pinMode(BUZZER_PIN,OUTPUT);
  EEPROM.begin(4096);
  DATA.num_wakeup_tilt = EEPROM.read(DATA.addr_num_wakeup_tilt);
  DATA.num_wakeup_tilt += 1;
  EEPROM.write(DATA.addr_num_wakeup_tilt, DATA.num_wakeup_tilt);
  EEPROM.commit();
  Serial.printf("Wake up from GPIO: %d times\n", DATA.num_wakeup_tilt);

  
  
  return;
}

void IRAM_ATTR buttonISR() {
  State = fall_asleep; // when the button is pressed, lock the bike
}

// Configuration for unlock state
void wake_config_Unlock(void)
{
  pinMode(Lock_PIN,INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(Lock_PIN), buttonISR, RISING); 
  pinMode(Extern_LED,OUTPUT);
  
  pinMode(LED_PIN,OUTPUT);
  LED_off();
  pinMode(BUZZER_PIN,OUTPUT);
  Alarm_off();
  Serial.printf("Wake up from GPIO: unlock the bike\n");

  NFC_config();
  
  // turn on ADC for auto LED
  ADC_config();
  return;
}

 // Configuration for timer wakeup

void thingspeak_write(void)
{
  // wake device Indicate waking up
  SaraR5.Wake_device();
  //start GNNS Indicate GNNS  
  SaraR5.read_GNNS(true);
  //power off GNNS Indicate GNNS off
  //SaraR5.GPSpowerOFF();
  //Post GNNS indicate GNNS POST
  SaraR5.GNSS_publish(true);
  delay(10000);
}
void Thingspeak_config(void) //this function can take anywhere from 5sec to 5min
{
  // you can write the config code for Thingspeak here: 
  pinMode(_powerPin, OUTPUT);
  // ******************************
  digitalWrite(_powerPin, HIGH); //POWER DOWN!!!
	
  Serial.println("Waiting for device to startup...");
  digitalWrite(_powerPin, LOW); //POWER UP!!!
  delay(2000); //2secs low for power UP 
  digitalWrite(_powerPin, HIGH);
  delay(100);
  digitalWrite(_powerPin, LOW);
  delay(3000);
  Serial.println("Device ready...");
  
  // put your setup code here, to run once: FALSE/TRUE FOR DEBUG 
  if(SaraR5.SARA5_setup(false) == SARA5_ERROR_SUCCESS){
    Serial.println(F("Setup Sucess!"));
  }else{
    Serial.println(F("Setup Failed damn..."));
  }
}

// Check NFC card access
bool checkAccess(void) {
 if (!mfrc522.PICC_IsNewCardPresent()) {
  return false; // No card detected
 }
 if (!mfrc522.PICC_ReadCardSerial()) {
  return false; // Error in reading card
 }

 // Compare the UID of the card with the predefined UID
 if (memcmp(mfrc522.uid.uidByte, accessUID, 4) == 0) {
  Serial.println("Access approved");
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100); // Buzz for 0.1 second
  digitalWrite(BUZZER_PIN, LOW);
  return true; // Access granted
 } else {
  Serial.println("Access denied");
  for (int i = 0; i<3; i++)
  {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(500); // Buzz for 0.5 second
  digitalWrite(BUZZER_PIN, LOW);
  delay(500);
  }
  return false; // Access denied
 }
}

// Configure ADC
void ADC_config(void)
{
  //set the resolution to 12 bits (0-4096)
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
}

// Automatic LED control based on ambient light
void auto_led(void)
{
  // read the analog / millivolts value for pin 2:
  int analogValue = analogRead(analog_PIN);
    // print out the values you read:
  Serial.printf("ADC analog value = %d\n",analogValue);

  if(analogValue<1500){
    LED_off();
  }
  else
    LED_on();
  delay(1000);
}