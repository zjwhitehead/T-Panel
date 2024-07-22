/*
 * @Description: On-board RS485 communication routine on T-Panel
 * @Author: LILYGO_L
 * @Date: 2023-06-21 18:42:16
 * @LastEditTime: 2024-07-20 16:35:29
 * @License: GPL 3.0
 */
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include "pin_config.h"
#include "XL95x5_Driver.h"

static uint32_t temp = 0;

// XL95x5 initializes the configuration
XL95x5 Class_XL95x5(XL95x5_IIC_ADDRESS, XL95x5_SDA, XL95x5_SCL);

HardwareSerial SerialPort(2); // use UART2
char DATA1 = 0;

void setup()
{
    Serial.begin(9600);
    Serial.println("Ciallo1");

    // 初始化串口，并重新定义引脚
    // 参数包括串行通信的波特率、串行模式、使用的 RX 引脚和 TX 引脚。
    SerialPort.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);

    Class_XL95x5.begin();
    Class_XL95x5.read_all_reg(); // Read all registers

    Class_XL95x5.portMode(XL95x5_PORT_0, OUTPUT); // Configure the XL95x5 full port mode
    Class_XL95x5.portMode(XL95x5_PORT_1, OUTPUT);

    Class_XL95x5.digitalWrite(XL95X5_RS485_CON, HIGH);
}

void loop()
{
    Class_XL95x5.digitalWrite(XL95X5_RS485_CON, HIGH);
    temp++;
    SerialPort.printf("%d", temp);

    delay(1000);

    while (SerialPort.available() > 0)
    {
        Class_XL95x5.digitalWrite(XL95X5_RS485_CON, LOW);
        DATA1 = SerialPort.read();
        Serial.print(DATA1);
    }
    delay(1000);
}
