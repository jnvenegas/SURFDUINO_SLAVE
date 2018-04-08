#ifndef BLE
#define BLE

#include <Arduino.h>

int BLEsetup();
void aci_loop();
uint8_t Add_UART_Service(void);
uint8_t lib_aci_send_data(uint8_t ignore, uint8_t* sendBuffer, uint8_t sendLength);
uint8_t Write_UART_TX(char* TXdata, uint8_t datasize);
void Read_Request_CB(uint16_t handle);
void setConnectable(void);
void Attribute_Modified_CB(uint16_t handle, uint8_t data_length, uint8_t *att_data);
void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);
void srfaDisconnect();

#endif