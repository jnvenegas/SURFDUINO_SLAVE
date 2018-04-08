//----------------------------
//
// ARDUINO
//
//----------------------------
#include <Arduino.h>
#include <Wire.h>

//----------------------------
//
// I2C
//
//----------------------------

#include <SlaveData.h>

//slave module I2C address
#define I2C_SLV_NODE_ADDRESS 79
//slave Data struct for transfer over I2C
SlaveDataStruct slvData;
//buffer to send data to master I2C node
uint8_t buffer[sizeof(SlaveDataStruct)];

//send data to master upon its I2C request
void onRequest()
{
  Wire.write(buffer,sizeof(SlaveDataStruct));
  slvData.bleCmd ='\0';
}

//----------------------------
//
// GPS
//
//----------------------------
#include "SoftwareSerial256.h"
#include <TinyGPS++.h>

//TinyGPS++ NMEA parsing object
TinyGPSPlus gps;
//software serial buffer we attach to GPS device to get 
//output and send input (when configuring)
SoftwareSerial gpsOutputBuffer(A1, A0);
//baud rate for software serial
static const uint32_t gpsBaud = 9600;

//----------------------------
//
// BLE
//
//----------------------------

//ble.h forward delcares a lot of the functions
//to prevent comiplation errors in the BLE code
//at the bottom of this file
#include "Ble.h"

//ST BLE module code
#include <STBLE.h>

//divisable by 15 as per iOS guidelines
#define  ADV_INTERVAL_MIN_MS  30 
//divisable by 15 and bigger than min as per iOS guidelines
#define  ADV_INTERVAL_MAX_MS  120 

//** adds extra flash and memory requirements if set to true**
#ifndef BLE_DEBUG
#define BLE_DEBUG false
#endif
#if defined (ARDUINO_ARCH_AVR)
#define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#endif
#if BLE_DEBUG
#include <stdio.h>
char sprintbuff[100];
#define PRINTF(...) {sprintf(sprintbuff,__VA_ARGS__);SerialMonitorInterface.print(sprintbuff);}
#else
#define PRINTF(...)
#endif

//variables to keep track of connection status of the BLE
volatile uint8_t set_connectable = 1;
uint16_t connection_handle = 0;
int connected = FALSE;

//buffer variables to send and receive data
uint8_t ble_rx_buffer[21];
uint8_t ble_rx_buffer_len = 0;
uint8_t ble_connection_state = false;
#define PIPE_UART_OVER_BTLE_UART_TX_TX 0

//----------------------------
//
// MAIN LOOP AND FUNCTIONS
//
//----------------------------

//non blocking GPS consumption function
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (gpsOutputBuffer.available())
      gps.encode(gpsOutputBuffer.read());
    
    //poll BLE module
    aci_loop();

  } while (millis() - start < ms);
}

//setup device and application objects
void setup() {
  SerialMonitorInterface.begin(9600);
  while (!SerialMonitorInterface); //This line will block until a serial monitor is opened with TinyScreen+!
  
  //**BLE setup

  BLEsetup();

  //**GPS setup

  //pins used by the GPS module for power functions
  const uint8_t onOffPin = A3;
  const uint8_t sysOnPin = A2;

    //start the software serial that capures the output of the GPS
  gpsOutputBuffer.begin(gpsBaud);

  //GPS device to wake mode
  pinMode(sysOnPin, INPUT);
  digitalWrite(onOffPin, LOW);
  pinMode(onOffPin, OUTPUT);
  delay(100);

  //wait for GPS to wake up
  while (digitalRead(sysOnPin) == LOW)
  {
      digitalWrite(onOffPin, HIGH);
      delay(5);
      digitalWrite(onOffPin, LOW);
      delay(100);
  }
  delay(100);

  //setup I2C for this node
  Wire.begin(I2C_SLV_NODE_ADDRESS);
  //master will send us data and we will use this function to store it
  Wire.onRequest(onRequest);
}

void loop() {

  //process GPS stream and poll BLE
  smartDelay(1000);
    
  //if we have received any data from the phone
  if (ble_rx_buffer_len)
  {
    //see what command the phone has sent
    char cmd = ble_rx_buffer[0];
      
      //temporary response to commands which just echos command to phone
      /*char buf[15];
      sprintf(buf, "you said %c lud\0", cmd);
      lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, (uint8_t*)buf, strlen(buf));
      */

    //add our command to our I2C data packet for the master
    slvData.bleCmd = cmd;
    //reset buffer length to 0 so we are ready for the next command
    ble_rx_buffer_len = 0;
  }
 
  //get new valid data in to gps struct container which 
  //is send over I2C to master node
  if(gps.date.isValid() && gps.date.isUpdated())
  {
    char sz[32];
    sprintf(sz, "%02d%02d%02d ", gps.date.year(),gps.date.month(), gps.date.day());
    strncpy(slvData.dateString,sz,6);
  } 
  if(gps.time.isValid() && gps.time.isUpdated())
  {
    char sz[32];
    sprintf(sz, "%02d%02d%02d ", gps.time.hour(),gps.time.minute(), gps.time.second());
    strncpy(slvData.timeString,sz,6);
  } 
  if(gps.speed.isValid() && gps.speed.isUpdated())
  {
    slvData.mph = gps.speed.mph();
    slvData.isGpsLocked = true;
  } 
  else
  {
    //speed is our most important data point as it is used
    //to trigger the master node logging, so we base our
    //validity variable used by the master node on if the current
    //speed value from the GPS is good to use or not.
    slvData.isGpsLocked = false;
  }
  if(gps.location.isValid() && gps.location.isUpdated())
  {
    slvData.latitude = gps.location.lat();
    slvData.longitude = gps.location.lng();
  } 

  //copy gps data struct to the buffer used to hold data we send
  //to the master over I2C
  memcpy(buffer,&slvData,sizeof(slvData));

  Serial.println(slvData.dateString);
  Serial.println(slvData.timeString);
  Serial.println(slvData.isGpsLocked);
  Serial.println(slvData.latitude);
  Serial.println(slvData.longitude);
  Serial.println(slvData.mph);
  Serial.println(slvData.bleCmd);
   Serial.println(sizeof(SlaveDataStruct));
  Serial.println("----**-----"); 

}

//----------------------------           
/*
    Any code beyond this point is a mystery to me and a bit messy
    so for the moment i will leave if dumped at the end of this
    file to do its job without alteration.
*/
//----------------------------

int BLEsetup() {
  int ret;

  HCI_Init();
  /* Init SPI interface */
  BNRG_SPI_Init();
  /* Reset BlueNRG/BlueNRG-MS SPI interface */
  BlueNRG_RST();

  uint8_t bdaddr[] = {0x12, 0x34, 0x00, 0xE1, 0x80, 0x02};

  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);

  if (ret) {
    PRINTF("Setting BD_ADDR failed.\n");
  }

  ret = aci_gatt_init();

  if (ret) {
    PRINTF("GATT_Init failed.\n");
  }

  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);

  if (ret) {
    PRINTF("GAP_Init failed.\n");
  }

  const char *name = "BlueNRG";

  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0, strlen(name), (uint8_t *)name);

  if (ret) {
    PRINTF("aci_gatt_update_char_value failed.\n");
  } else {
    PRINTF("BLE Stack Initialized.\n");
  }

  ret = Add_UART_Service();

  if (ret == BLE_STATUS_SUCCESS) {
    PRINTF("UART service added successfully.\n");
  } else {
    PRINTF("Error while adding UART service.\n");
  }

  /* +4 dBm output power */
  ret = aci_hal_set_tx_power_level(1, 3);
}

void aci_loop() {
  HCI_Process();
  ble_connection_state = connected;
  if (set_connectable) {
    setConnectable();
    set_connectable = 0;
  }
  if (HCI_Queue_Empty()) {
    //Enter_LP_Sleep_Mode();
  }
}

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
  do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
    uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
    uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
    uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
  }while(0)

#define COPY_UART_SERVICE_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x6E, 0x40, 0x00, 0x01, 0xB5, 0xA3, 0xF3, 0x93, 0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E)
#define COPY_UART_TX_CHAR_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x6E, 0x40, 0x00, 0x02, 0xB5, 0xA3, 0xF3, 0x93, 0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E)
#define COPY_UART_RX_CHAR_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x6E, 0x40, 0x00, 0x03, 0xB5, 0xA3, 0xF3, 0x93, 0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E)

uint16_t UARTServHandle, UARTTXCharHandle, UARTRXCharHandle;


uint8_t Add_UART_Service(void)
{
  tBleStatus ret;
  uint8_t uuid[16];

  COPY_UART_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 7, &UARTServHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  COPY_UART_TX_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(UARTServHandle, UUID_TYPE_128, uuid, 20, CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE,
                           16, 1, &UARTTXCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  COPY_UART_RX_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(UARTServHandle, UUID_TYPE_128, uuid, 20, CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, 0,
                           16, 1, &UARTRXCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  return BLE_STATUS_SUCCESS;

fail:
  PRINTF("Error while adding UART service.\n");
  return BLE_STATUS_ERROR ;

}

uint8_t lib_aci_send_data(uint8_t ignore, uint8_t* sendBuffer, uint8_t sendLength) {
  return !Write_UART_TX((char*)sendBuffer, sendLength);
}

uint8_t Write_UART_TX(char* TXdata, uint8_t datasize)
{
  tBleStatus ret;

  ret = aci_gatt_update_char_value(UARTServHandle, UARTRXCharHandle, 0, datasize, (uint8_t *)TXdata);

  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error while updating UART characteristic.\n") ;
    return BLE_STATUS_ERROR ;
  }
  return BLE_STATUS_SUCCESS;

}

void Read_Request_CB(uint16_t handle)
{
  if (connection_handle != 0)
    aci_gatt_allow_read(connection_handle);
}

void setConnectable(void)
{
  tBleStatus ret;

  const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME, 'B', 'l', 'u', 'e', 'N', 'R', 'G'};

  hci_le_set_scan_resp_data(0, NULL);
  PRINTF("General Discoverable Mode.\n");

  ret = aci_gap_set_discoverable(ADV_IND,
                                 (ADV_INTERVAL_MIN_MS * 1000) / 625, (ADV_INTERVAL_MAX_MS * 1000) / 625,
                                 STATIC_RANDOM_ADDR, NO_WHITE_LIST_USE,
                                 sizeof(local_name), local_name, 0, NULL, 0, 0);

  if (ret != BLE_STATUS_SUCCESS)
    PRINTF("%d\n", (uint8_t)ret);

}

void Attribute_Modified_CB(uint16_t handle, uint8_t data_length, uint8_t *att_data)
{
  if (handle == UARTTXCharHandle + 1) {
    int i;
    for (i = 0; i < data_length; i++) {
      ble_rx_buffer[i] = att_data[i];
    }
    ble_rx_buffer[i] = '\0';
    ble_rx_buffer_len = data_length;
  }
}

void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle) {

  connected = TRUE;
  connection_handle = handle;

  PRINTF("Connected to device:");
  for (int i = 5; i > 0; i--) {
    PRINTF("%02X-", addr[i]);
  }
  PRINTF("%02X\r\n", addr[0]);
}

void GAP_DisconnectionComplete_CB(void) {
  connected = FALSE;
  PRINTF("Disconnected\n");
  /* Make the device connectable again. */
  set_connectable = TRUE;
}

void HCI_Event_CB(void *pckt)
{
  hci_uart_pckt *hci_pckt = (hci_uart_pckt *)pckt;
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;

  if (hci_pckt->type != HCI_EVENT_PKT)
    return;

  switch (event_pckt->evt) {

    case EVT_DISCONN_COMPLETE:
      {
        //evt_disconn_complete *evt = (void *)event_pckt->data;
        GAP_DisconnectionComplete_CB();
      }
      break;

    case EVT_LE_META_EVENT:
      {
        evt_le_meta_event *evt = (evt_le_meta_event *)event_pckt->data;

        switch (evt->subevent) {
          case EVT_LE_CONN_COMPLETE:
            {
              evt_le_connection_complete *cc = (evt_le_connection_complete *)evt->data;
              GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
            }
            break;
        }
      }
      break;

    case EVT_VENDOR:
      {
        evt_blue_aci *blue_evt = (evt_blue_aci *)event_pckt->data;
        switch (blue_evt->ecode) {

          case EVT_BLUE_GATT_READ_PERMIT_REQ:
            {
              evt_gatt_read_permit_req *pr = (evt_gatt_read_permit_req *)blue_evt->data;
              Read_Request_CB(pr->attr_handle);
            }
            break;

          case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
            {
              evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
              Attribute_Modified_CB(evt->attr_handle, evt->data_length, evt->att_data);
            }
            break;
        }
      }
      break;
  }
}