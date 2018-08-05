// LoRa 9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example LoRa9x_RX
//https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/rfm9x-test
//http://forum.arduino.cc/index.php?topic=313587.0

#include <Arduino.h>
#include <avr/power.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <LowPower.h>
#define RFM95_CS 9
#define RFM95_RST A0
#define RFM95_INT 2
#define boxLidPin 3
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 868.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

//protos
void wakeup();
void sleep();
int battVolt();
void whattoSend();

//Vars:
bool LidHasBeenOpened = 0;    //flag for keeping track if the mailbox lid has been opened.
int coolDownTimer = 0;        //debounce timer for the lid, so we dont spam.
bool messageSentFlag = false; //flag for keeping track if we have sent something

//This is the struct we send
struct dataStruct
{
  float BatteryVoltage;
  bool lidOpened;
  int ID;

} myData;

byte tx_buf[sizeof(myData)] = {0};

void setup()
{
  pinMode(RFM95_RST, OUTPUT);
  pinMode(boxLidPin, INPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(9600);
  delay(100);

  Serial.println("Mailbox lid detector...With LoRa! :D ");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init())
  {
    Serial.println("LoRa radio init failed");
    while (1)
      ;
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ))
  {
    Serial.println("setFrequency failed");
    while (1)
      ;
  }
  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  power_twi_disable(); // TWI (I2C) disabled
  power_adc_disable(); // ADC converter disabled

  myData.ID = 21; //21  är brevlådan
  //attachInterrupt(digitalPinToInterrupt(boxLidPin), wakeup, FALLING);
}

void loop()
{

  if (LidHasBeenOpened && !messageSentFlag)
  {
    whattoSend();
  }

  //Serial.println("Waiting for reply...");

  //going to sleep

  if (millis() - coolDownTimer > 30000)
  {
    sleep();
  }
  /* else
  {
    Serial.println(millis() - coolDownTimer);
  } */
}

void whattoSend()
{
  messageSentFlag = true;
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  myData.BatteryVoltage = battVolt() / 1000.00;
  myData.lidOpened = LidHasBeenOpened;
  //myData.counter = counter++;

  Serial.println("Sending to rf95_server");
  byte zize = sizeof(myData);
  Serial.println("Sending...");
  delay(10);

  memcpy(tx_buf, &myData, sizeof(myData));
  rf95.send((uint8_t *)tx_buf, zize);

  Serial.println("Waiting for packet to complete...");
  rf95.sleep();
  rf95.waitPacketSent();
  rf95.sleep();
  if (rf95.waitAvailableTimeout(1000))
  {
    // Should be a reply message for us now
    if (rf95.recv(buf, &len))
    {
      Serial.print("Got reply: ");
      Serial.println((char *)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("No reply, is there a listener around?");
  }
  // delay(1000);
}

void sleep()
{
  Serial.println("Going to sleep");
  delay(200);

  messageSentFlag = false;
  LidHasBeenOpened = false;
  rf95.sleep();

  attachInterrupt(digitalPinToInterrupt(boxLidPin), wakeup, FALLING);
  delay(200);
  // LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  detachInterrupt(digitalPinToInterrupt(boxLidPin));
  LidHasBeenOpened = true;
  LidHasBeenOpened = true;
  coolDownTimer = millis();
  Serial.println("Woken up");
}

void wakeup()
{
  //delay(200);
}

int battVolt()
{
  power_adc_enable(); // ADC converter
  long result = 0;

  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2);            // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC))
    ; // measuring

  uint8_t low = ADCL;  // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  power_adc_disable();        // ADC converter

  return result;
}

/*
   Name: powerDown
 * Description: Putting microcontroller into power down state. This is
 *			   the lowest current consumption state. Use this together with
 *			   external pin interrupt to wake up through external event
 *			   triggering (example: RTC clockout pin, SD card detect pin).
 *
 * Argument   Description
 * =========    ===========
 * 1. period     Duration of low power mode. Use SLEEP_FOREVER to use other wake
 *				up resource:
 *				(a) SLEEP_15MS - 15 ms sleep
 *				(b) SLEEP_30MS - 30 ms sleep
 *				(c) SLEEP_60MS - 60 ms sleep
 *				(d) SLEEP_120MS - 120 ms sleep
 *				(e) SLEEP_250MS - 250 ms sleep
 *				(f) SLEEP_500MS - 500 ms sleep
 *				(g) SLEEP_1S - 1 s sleep
 *				(h) SLEEP_2S - 2 s sleep
 *				(i) SLEEP_4S - 4 s sleep
 *				(j) SLEEP_8S - 8 s sleep
 *				(k) SLEEP_FOREVER - Sleep without waking up through WDT
 *
 * 2. adc		ADC module disable control. Turning off the ADC module is
 *				basically removing the purpose of this low power mode.
 *				(a) ADC_OFF - Turn off ADC module
 *				(b) ADC_ON - Leave ADC module in its default state
 *
 * 3. bod		Brown Out Detector (BOD) module disable control:
 *				(a) BOD_OFF - Turn off BOD module
 *				(b) BOD_ON - Leave BOD module in its default state
 *

 Enabling:
   power_adc_enable(); // ADC converter
   power_spi_enable(); // SPI
   power_usart0_enable(); // Serial (USART)
   power_timer0_enable(); // Timer 0
   power_timer1_enable(); // Timer 1
   power_timer2_enable(); // Timer 2
   power_twi_enable(); // TWI (I2C)

   Disabling:
   power_adc_disable(); // ADC converter
   power_spi_disable(); // SPI
   power_usart0_disable();// Serial (USART)
   power_timer0_disable();// Timer 0
   power_timer1_disable();// Timer 1
   power_timer2_disable();// Timer 2
   power_twi_disable(); // TWI (I2C)
*/