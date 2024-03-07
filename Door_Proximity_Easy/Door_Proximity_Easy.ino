/*
 * iOS 기기 도어락 근접 도어 오픈(iOS device door lock proximity door opening)
 * 
  현관 도어락 근접 도어 자동 제어 모듈 펌웨어(Door lock proximity door automatic control module firmware)
  [KOR]
  - 인증된 아이폰/아이패드/애플워치 연결을 위한(iOS) IRK 지원
  - 페어링 전용 학습 버튼 입력(GPIO 2) 및 암호없음
  - 도어제어 접점신호 출력(GPIO 42) 
  - 재실 신호 출력(GPIO 41)
  - 도어 근접하면 자동으로 열림(GPIO 6 핀으로 펄스 출력)
  - 도어 제어장치 한개에 최대 20개의 iOS 기기 학습 지원
  - Paring 스위치를 눌러 학습(20개 초과 학습되면 가장 처음 장치가 삭제됨)
  - 학습된 위치의 RSSI를 임계값으로 저장함(학습위치보다 더 가까워지면 열림)
  - Arduino\Library\AduinoBLE 폴더 압축 후 삭제 후 사용
  - 지원하는 아두이노 디바이스 ESP32 NORMAL/C3/S2/S3
  [ENG]
  - IRK support for authenticated iPhone/iPad/Apple Watch connection (iOS)
  - Pairing-only learning button input (GPIO 2) and no password
  - Door control contact signal output (GPIO 42)
  - Presence signal output (GPIO 41)
  - Automatically opens when the door approaches (pulse output through GPIO 6 pin)
  - Supports learning of up to 20 iOS devices for one door control device
  - Learning by pressing the Paring switch (if more than 20 devices are learned, the first device is deleted)
  - Saves the RSSI of the learned location as a threshold (opens when closer than the learned location)
  - Use after compressing and deleting the Arduino\Library\AduinoBLE folder.
  - Supported Arduino devices ESP32 NORMAL/C3/S2/S3

  2022 우혁준, uhj0305@gmail.com
*/
#include <iostream>
#include <string>
#include <Arduino.h>
#include <Ticker.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <esp_task_wdt.h>
#include <EEPROM.h>           //https://github.com/espressif/arduino-esp32/tree/master/libraries/EEPROM

#include "CRC32.h"            //https://github.com/RobTillaart/CRC
#include "printHelpers.h"     //https://github.com/RobTillaart/printHelpers
#include "irk.h"

#define ESP32
#define MORE_DEBUG

//#define ESP32_LEGACY
//#define ESP32C3
#define ESP32_S3

#ifdef ESP32_S3
#include "esp_random.h"
#endif

uint64_t chipid;
//Libraries

//Constants
#define EEPROM_SIZE 1024

#define WDT_TIMEOUT 30     // define a 3 seconds WDT (Watch Dog Timer)

char device_id_buf[50];
char device_id_crc32[30];

CRC32 crc;

#define SCAN_TIME 1

uint8_t connected_irk[ESP_BT_OCTET16_LEN];
uint8_t device_addr[ESP_BT_OCTET16_LEN];

#define MAC_LEN 6
#define RECV_PAYLOAD_SIZE 28
unsigned char AdMac[MAC_LEN];
uint8_t irkMac[MAC_LEN];
  
// Define a struct to hold the data on a single BLE device
struct BleDevice {
  char address[18];
  char name[80];
  char manufacturer[80];
  int appearance;
  int rssi;
  int txPower;
};

// create a static array to hold 80 BLE Devices
static struct BleDevice BleDevices[80];  
static int BleDeviceCounter = 0;

// BLE variables
BLEScan* pBLEScan;
BLEScanResults foundDevices;
static char dev_uuid[80];
static int ind = 0;           // used in loop as an index
static int lastFreeHeap = 0;
static int deviceCounter = 0;

BLEServer *pServer = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

bool door_unlock_control = false;
bool detect_near_device = false;

int remote_c_rssi;
unsigned char remote_Macaddr[MAC_LEN];
#define MAX_AUTH_DEV_NUM 50 //Max bonded device number
unsigned char secu_Macaddr[MAX_AUTH_DEV_NUM][sizeof(esp_bd_addr_t)];
unsigned char secu_Macirk[MAX_AUTH_DEV_NUM][sizeof(esp_bt_octet16_t)];
int secu_bond_num = 0;

void show_bonded_devices(void);
void door_control();
void rled_on();
void Led_Toggle_Timer();
void Device_Disconnect_Timer();
void update_rssi_info(uint8_t nv_pos, int nv_rssi);
bool check_overlap_device_info();
void update_rssi_info_ondemand(uint8_t nv_pos, int nv_rssi);
void myshutdown();

Ticker r_timer;
Ticker led_timer;
Ticker disconn_timer;

const float repeatPeriod = 1000; //ms
const float LedPeriod = 1000; //ms

unsigned int sec_counter = 0;

#define RGB_INDI

/* Built in RGB LED MCU Board */
#ifdef RGB_BUILTIN
  #undef RGB_BUILTIN
  #define RGB_BUILTIN 48 // RGB LED PORT 48
#endif

#ifdef RGB_BUILTIN
  //HAVE RGBLED Port 48
  uint8_t R_BRIGHTNESS = 0;
  uint8_t G_BRIGHTNESS = 0;
  uint8_t B_BRIGHTNESS = 0;
  #define RGB_BRIGHTNESS (0xF)
#endif

/* Independent RGB LED MCU Board */
#ifdef RGB_INDI
  #define GLED_PIN 7//4:Green/5:Blue/3:Red, 15:Green/16:Blue/17:Red
  #define BLED_PIN 6
  #define RLED_PIN 5
#endif

/* Switch Define */
#define PARING_PIN 2 /* Paring(Learn RSSI, short click) / Clear data switch(Long click) */
#define DOOR_SW_PIN 42 //6 /* Door control(Normal low, 500ms high pulse out when open control) */
#define PRESENCE_DET_PIN 41 /* Presence status */

#define MAX_DEVICE_NUM  20
#define MAGIC_CODE_VAL 0x5256913

struct SecurityDevice {
  boolean valid;
  uint8_t pid_key[ESP_BT_OCTET16_LEN];
  int rssi;
};

struct SecurityDevice SecDevice[MAX_DEVICE_NUM]; 
boolean near_device_flag[MAX_DEVICE_NUM];
unsigned int near_device_loss_tmr_count[MAX_DEVICE_NUM];
unsigned int weak_signal_cnt[MAX_DEVICE_NUM];
unsigned int device_missing_timeout_count[MAX_DEVICE_NUM]; //If it is not visible from the SCAN for more than 5 seconds, it is considered to have moved away
boolean scan_device_flag[MAX_DEVICE_NUM];
boolean ondemand_door_control = false;
unsigned int valid_id_count = 0;
boolean door_control_before = false;
unsigned int door_control_before_cnt = 0;
uint8_t match_id_pos = 0;
boolean device_match = false;

#define NEAR_TO_FAR_hysteresis 10 //Threshold RSSI value moving away
#define MISS_TIMEOUT 30 //Unrecognized timeout value
#define LOSS_TIMEOUT 300 //Disappear timeout value
#define AVERAGE_COUNT 5 //average number of times
#define WEAK_SIGNAL_CNT_MAX 5 //Weak signal count
#define LONGKEY_TIMER 10 //Default setting(clear all devices) long key timer
#define DOOR_RETRY_TIMEOUT 5 //Door control retry allowed timeout
int cur_rssi;
int adv_rssi;

int rssi_sum[MAX_DEVICE_NUM][AVERAGE_COUNT];
int door_control_cnt[MAX_DEVICE_NUM];

static const char * LOG_TAG = "PROXIMITY";

struct config_position{
  uint8_t rpos;
  unsigned int magic_code;
};

struct config_position cfg_pos;

typedef enum {
  enum_rpos = 0,
  enum_device_pos = sizeof(cfg_pos),
}id_eeprom_address;

id_eeprom_address enum_eeprom_address;

bool paring_status = false;
uint32_t paringtmrCount = 0;

#define byte uint8_t

#define heartRateService BLEUUID((uint16_t)0x180D)

BLECharacteristic heartRateMeasurementCharacteristics(BLEUUID((uint16_t)0x2A37), BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic sensorPositionCharacteristic(BLEUUID((uint16_t)0x2A38), BLECharacteristic::PROPERTY_READ);
BLEDescriptor heartRateDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor sensorPositionDescriptor(BLEUUID((uint16_t)0x2901));

byte flags = 0b00111110;
byte bpm;
byte heart[8] = { 0b00001110, 60, 0, 0, 0 , 0, 0, 0};
byte hrmPos[1] = {2};

template <typename T, size_t N>
void show_address(const T (&address)[N])
{
  char address_device[18];
  
  //strcpy(address_device, address.toString().c_str());
  for(byte i=0; i<MAC_LEN; i++) {
    remote_Macaddr[i] = address[i];
  }
  //Serial.printf("%02X%02X%02X%02X%02X%02X\n\r", address[0], address[1], address[2], address[3], address[4], address[5]);
  Serial.print(address[0], HEX);
  for (uint8_t i = 1; i < N; i++)
    Serial.printf(":%02x", address[i]);
}

void get_bonded_device_number()
{
  int dev_num = esp_ble_get_bond_device_num();
  secu_bond_num = dev_num;
}

// BLE Callback
class MyServerCallbacks: public BLEServerCallbacks 
{
  public:
    static int16_t connection_id;

  /* dBm to distance parameters; How to update distance_factor 1.place the
   * phone at a known distance (2m, 3m, 5m, 10m) 2.average about 10 RSSI
   * values for each of these distances, Set distance_factor so that the
   * calculated distance approaches the actual distances, e.g. at 5m. */
  static constexpr float reference_power  = -50; //rssi reffrence 
  static constexpr float distance_factor = 3.5; 
      
    esp_err_t get_rssi() 
    { 
      //esp_ble_gap_read_rssi(remote_addr);
      return esp_ble_gap_read_rssi(remote_addr); 
    }
    static float get_distance(const int8_t rssi)
    { return pow(10, (reference_power - rssi)/(10*distance_factor)); }
      
  private:
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {
      deviceConnected = true;

      if(paring_status == false) {
        // Update connection variables
        connection_id = param->connect.conn_id;
        memcpy(&remote_addr, param->connect.remote_bda, sizeof(remote_addr));
        // Install the RSSI callback
        BLEDevice::setCustomGapHandler(&MyServerCallbacks::rssi_event);
        // Show new connection info
        Serial.printf("Connection #: %i, remote: ", connection_id);
        show_address(param->connect.remote_bda);
        Serial.printf(" [Callback installed]\n");
      }
      //deviceConnected = true;
      Serial.println("Device Connected.");
      
      if(paring_status == false) {
        ondemand_door_control = true;
        //After at least 60 seconds, will be dissconnected by timer.
        disconn_timer.once(1, Device_Disconnect_Timer);
        get_bonded_device_number();
      }
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      ondemand_door_control = false;
      Serial.println("Device Disconnected.");
      BLEDevice::setCustomGapHandler(nullptr);
      connection_id = -1;
    }
    static void rssi_event(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
 
    static esp_bd_addr_t remote_addr;
};

int16_t MyServerCallbacks::connection_id = -1;
//bool MyServerCallbacks::motor_state = 0;
esp_bd_addr_t MyServerCallbacks::remote_addr = {};
 
void MyServerCallbacks::rssi_event(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    static int16_t rssi_average = 0;
 
#ifdef VERBOSE
  show_address(remote_addr);
#endif
    if (event == ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT)
    {
        //
        // Adjust damping_factor to lower values to have a more reactive response
        const float damping_factor = 0.8;
        rssi_average = rssi_average * damping_factor + 
          param->read_rssi_cmpl.rssi * (1 - damping_factor);
        //Get Current RSSI
        remote_c_rssi = param->read_rssi_cmpl.rssi;
        

    //Serial.printf("%hi, %g\n",
    //  param->read_rssi_cmpl.rssi, get_distance(rssi_average)
    //);
    }

}
 
MyServerCallbacks monitor;

class MyCallbacks: public BLECharacteristicCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Device Connected.");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Device Disconnected.");
    }  
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);

        Serial.println();
        Serial.println("*********");
      }
    }
};

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice adDevice) {
      
        if(deviceConnected == true)  {
          adDevice.getScan()->stop();
        }
        
        if (adDevice.haveServiceUUID()) {  
            strcpy(dev_uuid, adDevice.getServiceUUID().toString().c_str());   
        } else {
            strcpy(dev_uuid, "UNKNOWN");   
        }

        if (adDevice.haveName()) {         
            strcpy(BleDevices[BleDeviceCounter].name, adDevice.getName().c_str());  
        } else {
            strcpy(BleDevices[BleDeviceCounter].name, "");
        }
        
        if (adDevice.haveManufacturerData()) {
          char *hex = BLEUtils::buildHexData(NULL, (uint8_t*)adDevice.getManufacturerData().data(), adDevice.getManufacturerData().length());
          strcpy(BleDevices[BleDeviceCounter].manufacturer, hex);
          free(hex);
          //strcpy(BleDevices[BleDeviceCounter].manufacturer, BLEUtils::buildHexData(NULL, (uint8_t*)adDevice.getManufacturerData().data(), adDevice.getManufacturerData().length()));
        } else {
            strcpy(BleDevices[BleDeviceCounter].manufacturer, "");
        }
        
        if (adDevice.haveTXPower()) {
            BleDevices[BleDeviceCounter].txPower = adDevice.getTXPower();
        } else {
            BleDevices[BleDeviceCounter].txPower = -999;      // -999 means NULL
        }
        
        if (adDevice.haveAppearance()) {                   
            BleDevices[BleDeviceCounter].appearance = adDevice.getAppearance();
        } else {   // clear the appearance
            BleDevices[BleDeviceCounter].appearance = -999 ;  // -999 means NULL
        }
        
        strcpy(BleDevices[BleDeviceCounter].address, adDevice.getAddress().toString().c_str());
        BleDevices[BleDeviceCounter].rssi = adDevice.getRSSI();
        cur_rssi = adDevice.getRSSI();
        
        if(adDevice.haveManufacturerData()) {//BleDevices[BleDeviceCounter].manufacturer[0] == 0x4c) { //Apple ID
          char address_device[18];
          char temp[3];
          unsigned char temp_amac;
          
          //Serial.println(adDevice.getAddress());
          BLEAddress pAddress = adDevice.getAddress();
          strcpy(address_device, pAddress.toString().c_str());
          //Serial.println(address_device);

          //irkMac = adDevice.getAddress()
          //const uint8_t* point = getAddress();//adDevice.getAddress();
          for (byte i = 0; i < MAC_LEN; i++)
          {
            temp[0] = address_device[0+i*3];
            temp[1] = address_device[1+i*3];
            temp[2] = 0;
            //Serial.printf("%s ",temp);
            temp_amac = strtol(temp, NULL, 16);//atoi(temp);
            //Serial.printf("0x%02X ",temp_amac);
            //Serial.print(temp_amac,HEX);
            //AdMac[MAC_LEN-1-i] = temp_amac;//pAddress[i];
            AdMac[i] = temp_amac;
          }
           
          //Serial.println();
          //cur_rssi = adDevice.getRSSI();
          int x = 0;
          
          for (byte i = 0; i < MAX_DEVICE_NUM; i++) {
            if(SecDevice[i].valid == true) {
              //printf("SecDevice[%d].valid == true\n",i);
              if(btm_ble_addr_resolvable(AdMac,SecDevice[i].pid_key)) {
                #ifdef MORE_DEBUG
                  Serial.printf("Resolved MacAdd=%02X%02X%02X%02X%02X%02X Near Detected %dth device(Saved rssi=%idBm Current rssi=%ddBm).\r\n"
                    ,AdMac[0],AdMac[1],AdMac[2],AdMac[3],AdMac[4],AdMac[5]
                  ,i+1,SecDevice[i].rssi,cur_rssi);
                #else
                  //Serial.printf(".");
                  //x++;
                  //if(x%20 == 0) {
                  //  Serial.printf("\n\r");
                  //}
                #endif

                if(SecDevice[i].rssi == 0) {
                  Serial.printf("Update RSSI Information.\n\r");
                  update_rssi_info(i, cur_rssi);
                }
                scan_device_flag[i] = true;
                device_missing_timeout_count[i] = 0;
                //Serial.println("device found : device_missing_timeout_count reset.");
                
                if ((near_device_flag[i] == false) && (SecDevice[i].rssi < (cur_rssi))) { //+5 rssi margin
                  Serial.printf("MacAdd=%02X%02X%02X%02X%02X%02X Near Detected %dth device(Saved RSSI=%idBm Current RSSI=%ddBm).\r\n"
                    ,AdMac[0],AdMac[1],AdMac[2],AdMac[3],AdMac[4],AdMac[5]
                  ,i+1,SecDevice[i].rssi,cur_rssi);                  
                  near_device_flag[i] = true;
                  led_timer.attach_ms(LedPeriod/2, Led_Toggle_Timer);
                  //Near Detect true
                  //detect_near_device = true;
                  printf("%dth Device Near Detect.(S:%ddBm C:%ddBm)\n\r", i+1, SecDevice[i].rssi,cur_rssi);
                  door_control();
                  near_device_loss_tmr_count[i] = 0;
                  weak_signal_cnt[i] = 0;
                }
                //if the hysteresis exceeds the limit or times out, reset.
                if(near_device_flag[i] == true) {
                  if((SecDevice[i].rssi - NEAR_TO_FAR_hysteresis) > cur_rssi) {
                    weak_signal_cnt[i]++;
                    //printf("weak_signal_cnt[%d]=%d\n", i, weak_signal_cnt[i]);
                    if(weak_signal_cnt[i] > WEAK_SIGNAL_CNT_MAX) {
                      near_device_flag[i] = false;
                      near_device_loss_tmr_count[i] = 0;
                      printf("%dth Device Far Detect.(S:%ddBm C:%ddBm)\n\r", i+1, SecDevice[i].rssi,cur_rssi); 
                      led_timer.attach_ms(LedPeriod, Led_Toggle_Timer);
                      //Near Detect false
                      //detect_near_device = false;
                    }
                  }
                  else {
                    if(weak_signal_cnt[i] > 0) {
                      //printf("weak_signal_cnt[%d]=%d reset\n", i, weak_signal_cnt[i]);
                      weak_signal_cnt[i] = 0;
                      rled_on();
                    }
                  }
                }//if(near_device_flag[i] == true)
              }//if(btm_ble_addr_resolvable(AdMac,SecDevice[i].pid_key)) {
              else {
                scan_device_flag[i] = false;
              }
            } //if(SecDevice[i].valid == true) {
          }
        }
        //BleDeviceCounter++;    // increment the static device counter last
        //if(deviceConnected == true)  {
        //  adDevice.getScan()->stop();
        //}
    }

};

uint16_t ble_conn_id;
void Device_Disconnect_Timer()
{
  ble_conn_id = pServer->getConnId();
  //Serial.print("ble_conn_id=");
  //Serial.println(ble_conn_id);
  pServer->disconnect(ble_conn_id);
}

int reset_cnt=0;
void Led_Toggle_Timer() {
  detect_near_device = false;
  for(int i=0; i<MAX_DEVICE_NUM; i++) {
    if(near_device_flag[i] == true) {
      detect_near_device = true;
      break;
    }
  }
  if((detect_near_device == false) || (paring_status == true)) {
    digitalWrite(PRESENCE_DET_PIN, LOW);
    digitalWrite(GLED_PIN, !digitalRead(GLED_PIN));
#ifdef RGB_BUILTIN
    if(digitalRead(GLED_PIN) == HIGH) {
      G_BRIGHTNESS = RGB_BRIGHTNESS;
    }
    else {
      G_BRIGHTNESS = 0x00;
    }
    neopixelWrite(RGB_BUILTIN,R_BRIGHTNESS,G_BRIGHTNESS,B_BRIGHTNESS); // Green
#endif        
  }
  else if(detect_near_device == true){
    digitalWrite(PRESENCE_DET_PIN, HIGH);
    digitalWrite(GLED_PIN, HIGH);
#ifdef RGB_BUILTIN
        G_BRIGHTNESS = RGB_BRIGHTNESS;
    neopixelWrite(RGB_BUILTIN,R_BRIGHTNESS,G_BRIGHTNESS,B_BRIGHTNESS); // Green
#endif    
  }

  reset_cnt++;
  if(paring_status == false) {
    if(reset_cnt >= 86400/2) { //Self reset 2 times a day.
      myshutdown();
    }
  }
}

void rled_on()
{
  digitalWrite(RLED_PIN, HIGH);
#ifdef RGB_BUILTIN
  R_BRIGHTNESS = RGB_BRIGHTNESS;
  neopixelWrite(RGB_BUILTIN,R_BRIGHTNESS,G_BRIGHTNESS,B_BRIGHTNESS); // Red
#endif      
  delay(100);
  digitalWrite(RLED_PIN, LOW);
#ifdef RGB_BUILTIN
  R_BRIGHTNESS = 0x00;
  neopixelWrite(RGB_BUILTIN,R_BRIGHTNESS,G_BRIGHTNESS,B_BRIGHTNESS); // Red
#endif  
}

void bled_on()
{
  digitalWrite(BLED_PIN, HIGH);
#ifdef RGB_BUILTIN
  B_BRIGHTNESS = RGB_BRIGHTNESS;
  neopixelWrite(RGB_BUILTIN,R_BRIGHTNESS,G_BRIGHTNESS,B_BRIGHTNESS); // Blue
#endif      
  delay(100);
  digitalWrite(BLED_PIN, LOW); 
#ifdef RGB_BUILTIN
  B_BRIGHTNESS = 0x00;
  neopixelWrite(RGB_BUILTIN,R_BRIGHTNESS,G_BRIGHTNESS,B_BRIGHTNESS); // Blue
#endif    
}

char buttonValue;
unsigned int long_key_wait = 0;
void Repeat_timer() 
{
  if(door_control_before == true) {
    door_control_before_cnt++;
    if(door_control_before_cnt > DOOR_RETRY_TIMEOUT) {
      door_control_before_cnt = 0;
      door_control_before = false;
      Serial.println("Let's Door Control Enable.");
    }
  }
  buttonValue = digitalRead(PARING_PIN);
  
  //LONGKEY_TIMER If you press the pairing switch for more than 10 seconds, everything will be reset and rebooted.
  if(buttonValue == 0) {
    long_key_wait++;
    if(long_key_wait > LONGKEY_TIMER) {
      Serial.printf("clear nv memory and reset(Long key).\n\r");
      clear_nv();
      //Delay 1s, clear all the bonded devices
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      remove_all_bonded_devices();  //Delay 1s, clear all the bonded devices      
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      myshutdown();
    }
  }
  else {
    long_key_wait = 0;
  }

  if((paring_status == false) && (buttonValue == 0)) {
    //Paring mode start!!!.
    paring_status = true;
    paringtmrCount = 0;
    led_timer.attach_ms(LedPeriod/10, Led_Toggle_Timer);
    Serial.printf("paring_status == true...\n\r");
  }
  if(paring_status == true) {
    paringtmrCount++;
  }

  sec_counter++;
  //printf("Repeat_timer=%d\n", sec_counter); 
  //Serial.print(">");
  
  if((paring_status == true) && (paringtmrCount == 60)) {
    myshutdown();
  }

  for (byte i = 0; i < MAX_DEVICE_NUM; i++) {
    //if If the hysteresis exceeds the limit or times out, reset.
    if(near_device_flag[i] == true) { //Shows connected.
      if(scan_device_flag[i] == false) { //If not visible, increase timeout timer.
        device_missing_timeout_count[i]++;
        //Serial.println("device_missing_timeout_count++");
        if(device_missing_timeout_count[i] > MISS_TIMEOUT) {
          near_device_flag[i] = false;
          device_missing_timeout_count[i] = 0;
          Serial.print(i+1);
          Serial.println("th Device Suddenly Missed.");
        }
      }
      else { //If it is visible, reset the timer.
        device_missing_timeout_count[i] = 0;
        //Serial.println("device_missing_timeout_count reset");
      }
    }
    else {
       
    }
    
    if(near_device_flag[i] == true) { //Shows connected.
      near_device_loss_tmr_count[i]++;
      if((near_device_loss_tmr_count[i] > LOSS_TIMEOUT)) { //If it times out and the device is not detected.
        //near_device_flag[i] = false;
        near_device_loss_tmr_count[i] = 0;
        printf("Device Detect Long Time(Over 5minutes) = %dth Device\n", i+1);
        //led_timer.attach_ms(LedPeriod, Led_Toggle_Timer);
      }
    }
    else {
      near_device_loss_tmr_count[i] = 0;
    }
    
  }

}

void init_gpio()
{
  pinMode(RLED_PIN, OUTPUT);
  pinMode(GLED_PIN, OUTPUT);
  pinMode(BLED_PIN, OUTPUT);
  pinMode(DOOR_SW_PIN, OUTPUT);
  pinMode(PRESENCE_DET_PIN, OUTPUT);
  pinMode(PARING_PIN, INPUT_PULLUP);
  digitalWrite(DOOR_SW_PIN, LOW);
  digitalWrite(RLED_PIN, LOW);
  digitalWrite(GLED_PIN, LOW);
  digitalWrite(BLED_PIN, LOW);
  digitalWrite(PRESENCE_DET_PIN, LOW);
  #ifdef RGB_BUILTIN
  R_BRIGHTNESS = 0;
  G_BRIGHTNESS = 0;
  B_BRIGHTNESS = 0;
  neopixelWrite(RGB_BUILTIN,R_BRIGHTNESS,G_BRIGHTNESS,B_BRIGHTNESS); // RGB_BUILTIN Init
  #endif
}

void init_nv()
{
  valid_id_count = 0;
  //Init EEPROM
  EEPROM.begin(EEPROM_SIZE);
  //for(int i=0; i<sizeof(myblelearn); i++)
  EEPROM.get(enum_rpos, cfg_pos);
  for (byte i = 0; i < MAX_DEVICE_NUM; i++) {
    EEPROM.get(enum_device_pos+i*sizeof(SecDevice[0]), SecDevice[i]);
    //count valid id!
    if(SecDevice[i].valid == true) {
      valid_id_count++;
    }
  }
  //printf("rpos=%d, Magic Code=0x%08x\n", cfg_pos.rpos, cfg_pos.magic_code);
  if((cfg_pos.rpos > MAX_DEVICE_NUM) || (cfg_pos.magic_code != MAGIC_CODE_VAL)) { //Position Error == Clear EEPROM Aerea
    clear_nv();
  }  
  printf("NV Memory Read OK...(rpos=%d, Magic Code=0x%08X)\n",cfg_pos.rpos,cfg_pos.magic_code);
  display_id_info();
}

void update_rssi_info(uint8_t nv_pos, int nv_rssi)
{
  //RSSI not updated after learning > Updated immediately after reboot.
  if((SecDevice[nv_pos].valid == true) && (SecDevice[nv_pos].rssi == 0)) {
    //curr_rssi update to SecDevice[nv_pos].rssi
    SecDevice[nv_pos].rssi = nv_rssi;
  }
  //EEPROM WRITE
  EEPROM.put(enum_device_pos+nv_pos*sizeof(SecDevice[0]), SecDevice[nv_pos]);
  EEPROM.commit();
  printf("NV RSSI Update OK...\n");  
}

void update_rssi_info_ondemand(uint8_t nv_pos, int nv_rssi)
{
  //RSSI not updated after learning > Updated immediately after reboot.
  if((SecDevice[nv_pos].valid == true)) {
    //curr_rssi update to SecDevice[nv_pos].rssi
    SecDevice[nv_pos].rssi = nv_rssi;
  }
  //EEPROM WRITE
  EEPROM.put(enum_device_pos+nv_pos*sizeof(SecDevice[0]), SecDevice[nv_pos]);
  EEPROM.commit();
  printf("NV RSSI On Demand Update OK[%d][%ddBm]...\n",nv_pos, nv_rssi);  
}

void write_device_info() {
  if(check_overlap_device_info() == true) {
    printf("NV Item Overlaped...\n");
    return;
  }
  else {
    
  }
  SecDevice[cfg_pos.rpos].valid = true;
  for(byte i=0; i<ESP_BT_OCTET16_LEN; i++) {
    SecDevice[cfg_pos.rpos].pid_key[i] = connected_irk[i];
  }
  cfg_pos.rpos = cfg_pos.rpos%MAX_DEVICE_NUM;
  SecDevice[cfg_pos.rpos].rssi = 0; //RSSI cannot be specified at the time of learning. After learning and rebooting, RSSI is updated by matching the same ID.
  EEPROM.put(enum_device_pos+cfg_pos.rpos*sizeof(SecDevice[0]), SecDevice[cfg_pos.rpos]);
  cfg_pos.rpos++;
  EEPROM.put(enum_rpos, cfg_pos);
  EEPROM.commit();
  printf("NV Item Write OK...\n");
}

void update_cfg_pos_rpos()
{
  cfg_pos.rpos = 5;
  EEPROM.put(enum_rpos, cfg_pos);
  EEPROM.commit();
  printf("NV Item Write OK...\n");
}

int get_device_info()
{
  
}

bool check_overlap_device_info() {
  byte match_cnt = 0;
  
  for(byte i=0; i<MAX_DEVICE_NUM; i++) {
    match_cnt = 0;
    for(byte j=0; j<ESP_BT_OCTET16_LEN; j++) {
      if((connected_irk[j] == SecDevice[i].pid_key[j]) && (SecDevice[i].valid == true)) {
        match_cnt++;
      }
    }
    if(match_cnt == ESP_BT_OCTET16_LEN) {
      cfg_pos.rpos = i;
      match_id_pos = i;
      return true; //Overlaped ID
    }
  }
  return false; //New ID
}

void clear_nv()
{
  for(int i=0; i<EEPROM_SIZE; i++) {
    EEPROM.write(i, 0);
  }
  cfg_pos.magic_code = MAGIC_CODE_VAL;
  cfg_pos.rpos = 0;
  EEPROM.put(enum_rpos, cfg_pos);
  EEPROM.commit();
  EEPROM.get(enum_rpos, cfg_pos);
  for (byte i = 0; i < MAX_DEVICE_NUM; i++) {
    EEPROM.get(enum_device_pos+i*sizeof(SecDevice[0]), SecDevice[i]);
  }  
  printf("Clear NV Memory...(Magic code=0x%08x)\n",cfg_pos.magic_code);
}

void init_var()
{
  for(byte i=0; i<MAX_DEVICE_NUM; i++) {
    near_device_flag[i] = false;
    weak_signal_cnt[i] = 0;
    door_control_cnt[i] = 0;
  }
}

void door_control()
{
  if(door_control_before == true) {
    printf("door_control_before  == true.\n");
    return;
  }
  door_control_cnt[match_id_pos]++;
  printf("Door Open Output 500ms(High).\n");
  door_control_before = true;
  digitalWrite(DOOR_SW_PIN, HIGH);
  bled_on();
  delay(400);
  digitalWrite(DOOR_SW_PIN, LOW);
}

void door_unlock()
{
  printf("Door Unlock Output 500ms(High).\n");

  digitalWrite(DOOR_SW_PIN, HIGH);
  bled_on();
  delay(400);
  digitalWrite(DOOR_SW_PIN, LOW);
}

void door_lock()
{
  printf("Door Lock Output 500ms(High).\n");

  digitalWrite(DOOR_SW_PIN, HIGH);
  bled_on();
  delay(400);
  digitalWrite(DOOR_SW_PIN, LOW);
}

void get_crc32()
{
  sprintf(device_id_buf,"%04X%08X",(uint16_t)(chipid >> 32),(uint32_t)chipid);
  crc.setPolynome(0x04C11DB7);
  crc.add((uint8_t*)device_id_buf, 50);
  for(int i=0;i<30;i++) {
    device_id_crc32[i] = 0;
  }
  //sprintf(device_id_crc32,"FABLE-%08X DOOR PROXIMITY",crc.getCRC());
  sprintf(device_id_crc32,"FABLE-%08X-PROXIMITY",crc.getCRC());
  //Serial.print(device_id_crc32);
  //Serial.println(crc.getCRC(), HEX);
  //printf("BLE Device Name:%s\n\r", device_id_crc32);

  crc.reset();
}

void setup()
{
    Serial.begin(115200);
    init_gpio();
    r_timer.attach_ms(repeatPeriod, Repeat_timer); //changer.once(30, change);
    led_timer.attach_ms(LedPeriod, Led_Toggle_Timer);
    chipid = ESP.getEfuseMac(); //The chip ID is essentially its MAC address(length: 6 bytes).
    get_crc32();
    view_boot_message();
    display_sysinfo();
    init_nv();
    //update_cfg_pos_rpos();
    init_var();
    esp_task_wdt_init(WDT_TIMEOUT, true);  // enable panic so ESP32 restarts
    esp_task_wdt_add(NULL);                           // add current thread to WDT watch
    Serial.println("Enable for watchdog...");
        
    // BLE setup
    Serial.println("Scanning for BT Devices...");
    BleDeviceCounter = 0;
    BLEDevice::init(device_id_crc32);
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(false); 
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);  // less or equal setInterval value
    setup_security();  
}

int avg_rssi = 0;
int sum_rssi;
int avg_cnt;
void loop()
{
  byte match_cnt = 0;
  byte matched_addr_pos = 0;
  
    esp_task_wdt_reset();   // Added to repeatedly reset the Watch Dog Timer
    BleDeviceCounter = 0;   // Set counter to zero
    if(deviceConnected == false) {
      foundDevices = pBLEScan->start(SCAN_TIME, false);   // Start scan
      if(avg_cnt > 0) {
        
        //Update Current RSSI to NV Memory
        if(device_match == true) {
          //Serial.printf("Matched Device rpos = %d, average rssi=%d\n\r", match_id_pos, avg_rssi);
          update_rssi_info_ondemand(match_id_pos,avg_rssi);
          ondemand_door_control = false;
          door_control();
          //Reboot Here For Heap Clear!!!
          myshutdown();
        }
      }
      avg_cnt = 0;
      avg_rssi = 0;
      sum_rssi = 0;
    }
    else if(paring_status == false){
      //If the IRK is the same when connected without pairing, 
      //the RSSI value at the location of the corresponding terminal is updated in the NV area.
      //At this time, the RSSI of the connected device is updated by averaging the measurements for about xx seconds
      //(values ​​below -99dBm are discarded).
      if((deviceConnected ==true)) {
        bool match_addr = false;
        byte mathch_cnt_mac = 0;

        bleMessageTask();
        #ifdef MORE_DEBUG
        printf("Connected Address:");
        #endif
        for(unsigned char i=0; i<secu_bond_num; i++) {
          mathch_cnt_mac = 0;
          for(unsigned char j = 0; j<sizeof(esp_bd_addr_t); j++)
          {
            #ifdef MORE_DEBUG
            printf("%02X",secu_Macaddr[i][j]);
            #endif
            if(remote_Macaddr[j] == secu_Macaddr[i][j]) {
              mathch_cnt_mac++;
            }
            if(mathch_cnt_mac == sizeof(esp_bd_addr_t)) {
              #ifdef MORE_DEBUG
              printf("[Address Matched=%d]", i);
              #endif
              matched_addr_pos = i;

              for(byte k=0; k<MAX_DEVICE_NUM; k++) {
                match_cnt = 0;
                for(byte l=0; l<ESP_BT_OCTET16_LEN; l++) {
                  if((secu_Macirk[matched_addr_pos][l] == SecDevice[k].pid_key[l]) && (SecDevice[k].valid == true)) {
                    match_cnt++;
                    printf("[%02X%02X]",secu_Macirk[matched_addr_pos][l], SecDevice[k].pid_key[l]);
                  }
                  //printf("\n\r");
                }
                //printf("\n\r");
                if(match_cnt == ESP_BT_OCTET16_LEN) {
                  match_id_pos = k;
                  printf("\n\rrssi update match_id_pos=%d\n\r",match_id_pos);
                  device_match = true;
                  match_cnt = 0;
                }
              }
            }
          }
          #ifdef MORE_DEBUG
          printf(", ");
          #endif
        }    
        #ifdef MORE_DEBUG
        printf("\r\n");
        #endif
      }
      // Request RSSI from the remote address
      if (monitor.get_rssi() != ESP_OK) {
        Serial.println("RSSI request failed.");    
      }
      else {
        #ifdef MORE_DEBUG
        printf("Current RSSI = %ddBm\n\r", remote_c_rssi);
        #endif
        if((remote_c_rssi > (-99)) && (remote_c_rssi != 0)) {
          avg_cnt++;
          sum_rssi += remote_c_rssi;
          avg_rssi = sum_rssi/avg_cnt;
          #ifdef MORE_DEBUG
          Serial.printf("Average RSSI = %ddBm.\n\r", avg_rssi);
          #endif
        }
        else {
          #ifdef MORE_DEBUG
          Serial.printf("RSSI too low.\n\r");
          #endif
        }
      }
    }
    
    deviceCounter = BleDeviceCounter;
    pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }    
    
    //Serial.printf("Free Heap = %i  <%i> \n", ESP.getFreeHeap(), (ESP.getFreeHeap()-lastFreeHeap));
    //lastFreeHeap = ESP.getFreeHeap();
    delay(100);                // delay 1/2 second
}

class MySecurity : public BLESecurityCallbacks {  //BLE class pin code
  uint32_t onPassKeyRequest(){
    ESP_LOGI(LOG_TAG, "PassKeyRequest");
    return 123456;
  }
  void onPassKeyNotify(uint32_t pass_key){
    ESP_LOGI(LOG_TAG, "The passkey Notify number:%d", pass_key);
  }
  bool onConfirmPIN(uint32_t pass_key){
    ESP_LOGI(LOG_TAG, "The passkey YES/NO number:%d", pass_key);
    vTaskDelay(5000);
    //if(paring_status == false) {
    //  return false;
    //}
    return true;
  }
  bool onSecurityRequest(){
    ESP_LOGI(LOG_TAG, "SecurityRequest");
    //if(paring_status == false) {
    //  return false;
    //}    
    return true;
  }
  void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl){
    ESP_LOGI(LOG_TAG, "Starting BLE work!");
    //if(paring_status == true) {
      show_bonded_devices();
  }  
};

void setup_security() {
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  BLEDevice::setSecurityCallbacks(new MySecurity());
  
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *pService = pServer->createService(heartRateService);

  // Create HRM BLE Characteristics
  pService->addCharacteristic(&heartRateMeasurementCharacteristics);

  heartRateDescriptor.setValue("Rate from 0 to 200");
  heartRateMeasurementCharacteristics.addDescriptor(&heartRateDescriptor);
  
  pService->addCharacteristic(&sensorPositionCharacteristic);
  sensorPositionDescriptor.setValue("Position 0 - 6");
  sensorPositionCharacteristic.addDescriptor(&sensorPositionDescriptor);

  // Start the service
  pService->start();

  // Start advertising 240228 For Scan Menu
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(heartRateService);//SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to hrm notify...");

 	/* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
	esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;     //bonding with peer device after authentication
	esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
	uint8_t key_size = 16;      //the key size should be 7~16 bytes
	uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
	uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
	//set static passkey
	uint32_t passkey = 123456;
	uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
	//uint8_t oob_support = ESP_BLE_OOB_DISABLE;
	esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
	//esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));
	/* If your BLE device acts as a Slave, the init_key means you hope which types of key of the master should distribute to you,
	and the response key means which key you can distribute to the master;
	If your BLE device acts as a master, the response key means you hope which types of key of the slave should distribute to you,
	and the init key means which key you can distribute to the slave. */
	esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

  Serial.println("Characteristic defined! Now you can read it in your phone!");
	//Delay 1s, clear all the bonded devices
	//vTaskDelay(1000 / portTICK_PERIOD_MS);
	//remove_all_bonded_devices();  
}

#define GATTS_TABLE_TAG "SEC_GATTS_DEMO"
void show_bonded_devices(void)
{
  int dev_num = esp_ble_get_bond_device_num();

  esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
  esp_ble_get_bond_device_list(&dev_num, dev_list);
  ESP_LOGI(GATTS_TABLE_TAG, "Bonded devices list : %d", dev_num);
  printf("Bonded devices list : %d\n\r", dev_num);
  secu_bond_num = dev_num;
  
  for (int i = 0; i < dev_num; i++) 
  {
    esp_log_buffer_hex(GATTS_TABLE_TAG, (void *)dev_list[i].bd_addr, sizeof(esp_bd_addr_t));

    printf("Add:");
    for(unsigned char j = 0; j<sizeof(esp_bd_addr_t); j++)
    {
        printf("%02X",*(((byte*)(dev_list[i].bd_addr))+j));

      device_addr[j] = *(((byte*)(dev_list[i].bd_addr))+j);
      secu_Macaddr[i][j] = device_addr[j];
    }
    printf("\r\n");

    printf("IRK:");
    for(unsigned char j = 0; j<sizeof(esp_bt_octet16_t); j++)
    {
      //For Security issue
      if( ((j>=0)&&(j<=1)) || ((j>=(sizeof(esp_bt_octet16_t)-2))&&(j<=sizeof(esp_bt_octet16_t))) ) {      
      printf("%02X",*(((byte*)(dev_list[i].bond_key.pid_key.irk))+j));
      }
      else {
        printf("X");
      }
      connected_irk[j] = *(((byte*)(dev_list[i].bond_key.pid_key.irk))+j);
      secu_Macirk[i][j] = connected_irk[j];
    }
    printf("\r\n");

    //If IRK is received in pairing state, authentication has passed and device information is updated.
    //In this case, the RSSI will not be updated, but will be updated if the RSSI is at 0 after rebooting.
    if(paring_status == true) {
      write_device_info(); //Will Be Check Paring Button Time out Here...
      if(i == (dev_num-1)) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        myshutdown();
      }
    }
    //If connected and registered IRK, Door Control Here!!! (manual forced control)
    else if(check_overlap_device_info() == true) {
      ondemand_door_control = false;
    }    
  }

  free(dev_list);
  //Delay 1s, clear all the bonded devices
  //vTaskDelay(1000 / portTICK_PERIOD_MS);
  //remove_all_bonded_devices();  //Delay 1s, clear all the bonded devices
}

void remove_all_bonded_devices(void)
{
  int dev_num = esp_ble_get_bond_device_num();

  esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
  esp_ble_get_bond_device_list(&dev_num, dev_list);
  for (int i = 0; i < dev_num; i++) {
    esp_ble_remove_bond_device(dev_list[i].bd_addr);
  }

  free(dev_list);
}

void view_boot_message()
{
  char tempbuf[50];

  Serial.print("\n\r****************************************");
  Serial.print("\n\r*  Fable v2.0 : BLE Proximity Access   *");
  Serial.print("\n\r*  (Auto Door Open & Close : Gate)     *");
  Serial.print("\n\r****************************************");
  Serial.print("\n\r*                                      *");
  sprintf(tempbuf,"\n\r*  Compiled : %s %s     *",__DATE__,__TIME__);
  Serial.print(tempbuf);
  Serial.print("\n\r*  Copyright(c) 2023 Woo Hyuk Joon     *");
  Serial.print("\n\r*                                      *");
  Serial.print("\n\r*  Phone : +82-10-9925-6913            *");
  Serial.print("\n\r*  Mail : uhj0305@gmail.com            *");
  Serial.print("\n\r*                                      *");
  sprintf(tempbuf,"\n\r*  BLE : %s *", device_id_crc32);
  Serial.print(tempbuf);  
  Serial.print("\n\r*                                      *");
  Serial.print("\n\r****************************************");
  //Serial.print("\n\r*  Unique ID : 0x");
  //Serial.printf("%04X", (uint16_t)(chipid >> 32)); //print High 2 bytes
  //Serial.printf("%08X", (uint32_t)chipid); //print Low 4bytes.  
  //Serial.print("          *");
  //Serial.print("\n\r****************************************\n\r"); 
}

void myshutdown()
{
  EEPROM.end();
  Serial.print("\n\rReboot Device... Copyright(c) 2023 Woo Hyuk Joon*******\n\r\n\r\n\r");
  ESP.restart(); //ESP32
}

void display_id_info()
{
  for(byte i=0; i<MAX_DEVICE_NUM; i++) {
    printf("ID%02d, ",i);
    printf("VALID:%d",SecDevice[i].valid);
    printf(", RSSI:%03ddBm",SecDevice[i].rssi);
    printf(", IRK:");
    for(byte j=0; j<sizeof(esp_bt_octet16_t); j++) {
      //For Security issue
      if( ((j>=0)&&(j<=1)) || ((j>=(sizeof(esp_bt_octet16_t)-2))&&(j<=sizeof(esp_bt_octet16_t))) ) {
        printf("%02X",SecDevice[i].pid_key[j]);
      }
      else {
        printf("X");
      }
    }
    printf("\r\n");
  }
}

void display_sysinfo()
{
    uint64_t chipid;
    chipid=ESP.getEfuseMac(); // The chip ID is essentially its MAC address(length: 6 bytes)
    Serial.printf("\n\rESP32 Chip ID = %04X", (uint16_t)(chipid>>32)); // print High 2 bytes
    Serial.printf("%08X\n\r", (uint32_t)chipid); // print Low 4bytes
    Serial.println("---------------------------------");
     
    Serial.printf("Chip Revision %d\n\r", ESP.getChipRevision());
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    Serial.printf("Number of Core: %d\n\r", chip_info.cores);
    Serial.printf("CPU Frequency: %d MHz\n\r", ESP.getCpuFreqMHz());  
    Serial.println();
     
    Serial.printf("Flash Chip Size = %d byte\n\r", ESP.getFlashChipSize());
    Serial.printf("Flash Frequency = %d Hz\n\r", ESP.getFlashChipSpeed());
    Serial.println();
     
    Serial.printf("ESP-IDF version = %s\n\r", esp_get_idf_version());
    Serial.println();
     
    Serial.printf("Total Heap Size = %d\n\r", ESP.getHeapSize());
    Serial.printf("Free Heap Size = %d\n\r", ESP.getFreeHeap());
    Serial.printf("Lowest Free Heap Size = %d\n\r", ESP.getMinFreeHeap());
    Serial.printf("Largest Heap Block = %d\n\r", ESP.getMaxAllocHeap());
    Serial.println();
     
    uint8_t mac0[6];
    esp_efuse_mac_get_default(mac0);
    Serial.printf("Default Mac Address = %02X:%02X:%02X:%02X:%02X:%02X\n\r", mac0[0], mac0[1], mac0[2], mac0[3], mac0[4], mac0[5]);
     
    uint8_t mac3[6];
    esp_read_mac(mac3, ESP_MAC_WIFI_STA);
    Serial.printf("[Wi-Fi Station] Mac Address = %02X:%02X:%02X:%02X:%02X:%02X\n\r", mac3[0], mac3[1], mac3[2], mac3[3], mac3[4], mac3[5]);
     
    uint8_t mac4[7];
    esp_read_mac(mac4, ESP_MAC_WIFI_SOFTAP);
    Serial.printf("[Wi-Fi SoftAP] Mac Address = %02X:%02X:%02X:%02X:%02X:%02X\n\r", mac4[0], mac4[1], mac4[2], mac4[3], mac4[4], mac4[5]);
     
    uint8_t mac5[6];
    esp_read_mac(mac5, ESP_MAC_BT);
    Serial.printf("[Bluetooth] Mac Address = %02X:%02X:%02X:%02X:%02X:%02X\n\r", mac5[0], mac5[1], mac5[2], mac5[3], mac5[4], mac5[5]);
     
    uint8_t mac6[6];
    esp_read_mac(mac6, ESP_MAC_ETH);
    Serial.printf("[Ethernet] Mac Address = %02X:%02X:%02X:%02X:%02X:%02X\n\r", mac6[0], mac6[1], mac6[2], mac6[3], mac6[4], mac6[5]);
}

void bleMessageTask()
{
  heart[1] = (byte)bpm;
  int energyUsed = 3000;
  heart[3] = energyUsed / 256;
  heart[2] = energyUsed - (heart[2] * 256);
  //printf("BPM: %d\n",bpm);

  heartRateMeasurementCharacteristics.setValue(heart, 8);
  heartRateMeasurementCharacteristics.notify();

  sensorPositionCharacteristic.setValue(hrmPos, 1);
  bpm = (byte)randomGen(80,180);  
}

// Just a random generator
int randomGen(int min, int max)
{
	int range; 
	range = max-min + 1;
#ifdef ESP32_S3
  return esp_random()%range + min;
#else
	return rand()%range + min;
#endif  
}