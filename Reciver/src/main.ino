//#include <Arduino.h>
// Arduino9x_RX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (receiver)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Arduino9x_TX
//https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/rfm9x-test
//http://forum.arduino.cc/index.php?topic=313587.0
//https://www.instructables.com/id/ESP8266-GMail-Sender/

#include <SPI.h>
#include <RH_RF95.h>
#include <ESP8266WiFi.h>
#include "Gsender.h"

#pragma region Globals
const char *ssid = "jotjena";         // WIFI network name
const char *password = "hejhejhallo"; // WIFI network password
uint8_t connection_state = 0;         // Connected to WIFI or not
uint16_t reconnect_interval = 10000;  // If not connected wait time to try again
#pragma endregion Globals
/*
  WeMos D1          RFM9x Module
  GPIO12 (D6) <----> MISO
  GPIO13 (D7) <----> MOSI
  GPIO14 (D5) <----> CLK
  GPIO15 (D8) <----> DIO0/D2 OR DIO1/D3 OR DIO2/D4
  GPIO16 (D0) <----> SEL Chip Select (depending on bottom solder PAD position) 
  */

int RFM95_CS = D8;
int RFM95_RST = D2;
int RFM95_INT = D1;

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 868.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
//#define LED 13
int LED = LED_BUILTIN;
struct dataStruct
{
    float BatteryVoltage;
    bool lidOpened;
    int ID;
    long int rest;

} myData;

//Protos:
//void Awaits();
//uint8_t WiFiConnect(const char *nSSID, const char *nPassword);

uint8_t WiFiConnect(const char *nSSID = nullptr, const char *nPassword = nullptr)
{
    static uint16_t attempt = 0;
    Serial.print("Connecting to ");
    if (nSSID)
    {
        WiFi.begin(nSSID, nPassword);
        Serial.println(nSSID);
    }
    else
    {
        WiFi.begin(ssid, password);
        Serial.println(ssid);
    }

    uint8_t i = 0;
    while (WiFi.status() != WL_CONNECTED && i++ < 50)
    {
        delay(200);
        Serial.print(".");
    }
    ++attempt;
    Serial.println("");
    if (i == 51)
    {
        Serial.print("Connection: TIMEOUT on attempt: ");
        Serial.println(attempt);
        if (attempt % 2 == 0)
            Serial.println("Check if access point available or SSID and Password\r\n");
        return false;
    }
    Serial.println("Connection: ESTABLISHED");
    Serial.print("Got IP address: ");
    Serial.println(WiFi.localIP());
    return true;
}
void setup()
{
    pinMode(LED, OUTPUT);
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);

    Serial.begin(9600);
    delay(100);
    connection_state = WiFiConnect();
    if (!connection_state) // if not connected to WIFI
        Awaits();          // constantly trying to connect

    Serial.println("Lora Reciver and Gmail sender :D");

    // manual reset
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    while (!rf95.init())
    {
        Serial.println("LoRa radio init failed");
    }
    Serial.println("LoRa radio init OK!");

    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    if (!rf95.setFrequency(RF95_FREQ))
    {
        Serial.println("setFrequency failed");
    }
    Serial.print("Set Freq to: ");
    Serial.println(RF95_FREQ);

    // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

    // The default transmitter power is 13dBm, using PA_BOOST.
    // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
    // you can set transmitter powers from 5 to 23 dBm:
    rf95.setTxPower(23, false);
    Serial.println("Setup Done!");
}

void loop()
{
    if (rf95.available())
    {
        // Should be a message for us now
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len))
        {
            digitalWrite(LED, HIGH);
            RH_RF95::printBuffer("Received: ", buf, len);
            Serial.print("Got: ");

            memcpy(&myData, buf, sizeof(myData));
            Serial.println("");

            Serial.print("BatteryVoltage: ");
            Serial.print(myData.BatteryVoltage);

            Serial.print("  lidOened: ");
            Serial.print(myData.lidOpened);

            Serial.print("  ID: ");
            Serial.println(myData.ID);

            Serial.print("RSSI: ");
            Serial.println(rf95.lastRssi(), DEC);

            // Send a reply
            uint8_t data[] = "And hello back to you";
            rf95.send(data, sizeof(data));
            rf95.waitPacketSent();
            Serial.println("Sent a reply");
            digitalWrite(LED, LOW);
        }
        else
        {
            Serial.println("Receive failed");
        }
    }

    if (myData.lidOpened)
    {
        sendMail();
        myData.lidOpened = 0;
    }
}

void sendMail()
{
    String messageString = "Battery Voltage: ";
    messageString += String(myData.BatteryVoltage);
    messageString += " Volt";
    Gsender *gsender = Gsender::Instance(); // Getting pointer to class instance
    String subject = "Mailman is here :D";
    if (gsender->Subject(subject)->Send("pommengt@gmail.com", messageString))
    {
        Serial.println("Message send.");
    }
    else
    {
        Serial.print("Error sending message: ");
        Serial.println(gsender->getError());
    }
}
void Awaits()
{
    uint32_t ts = millis();
    while (!connection_state)
    {
        delay(50);
        if (millis() > (ts + reconnect_interval) && !connection_state)
        {
            connection_state = WiFiConnect();
            ts = millis();
        }
    }
}
