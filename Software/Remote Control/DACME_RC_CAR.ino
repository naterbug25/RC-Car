/*
   Nathan Huber
   DACME Final Project
   RC Car Controller
   03/26/2019
*/

/*=============================================
            Buring Bootloader
  ============================================= */

/*
   Needed: Arduino Uno, ATmega328P, 16MHz Crystal, (2) 22pF capacitors, jumper wires
   Step0: Connect Wires: Arduino Uno Pins <--> ATmega328P
                                       10 <--> 1
                                       11 <--> 17
                                       12 <--> 18
                                       13 <--> 19
                                       5V <--> 7
                                       GND <--> 8
          Place crystal across Pin 9 and Pin 10 on the ATmega328P with a 22pF cap on each pin to ground.
   Step1: Open Examples->ArduinoISP.ino sketch and download it to the Uno.
   Step2: Tools -> Board "Arduino Uno" and Tools -> Programer "Arduino as ISP"
   Step3: Tools -> Burn Bootloader
   Step4: Wait a few minutes. "Success!"
*/

/*=============================================
            Controller Model A
  ============================================= */

/*=============================================
            NRF24L01 Radio Library
  ============================================= */

#include  <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "RF24_config.h"
#include "printf.h"

// For Model "A": Comment out "byte pipes[][31] = {"B0000","B0001"..." and leave the other uncommented
// For Model "B": Comment out "byte pipes[][31] = {"A0000","A0001"..." and leave the other uncommented
byte pipes[][31] = {"A0000", "A0001", "A0002", "A0003", "A0004", "A0005", "A0006", "A0007", "A0008", "A0009", "A0010", "A0011", "A0012", "A0013", "A0014", "A0015", "A0016", "A0017", "A0018", "A0019", "A0020", "A0021", "A0022", "A0023", "A0024", "A0025", "A0026", "A0027", "A0028", "A0029", "A0030", "A0031"};
//byte pipes[][31] = {"B0000","B0001","B0002","B0003","B0004","B0005","B0006","B0007","B0008","B0009","B0010","B0011","B0012","B0013","B0014","B0015","B0016","B0017","B0018","B0019","B0020","B0021","B0022","B0023","B0024","B0025","B0026","B0027","B0028","B0029","B0030","B0031"};

RF24 radio(8, 7);

/*=============================================
            Digital IO
  ============================================= */
const int Dip_Pin_0_IN = 0; // Input pin on the dip switch package
const int Dip_Pin_1_IN = 1; // Input pin on the dip switch package
const int Dip_Pin_2_IN = 2; // Input pin on the dip switch package
const int Dip_Pin_3_IN = 3; // Input pin on the dip switch package
const int Dip_Pin_4_IN = 4; // Input pin on the dip switch package
const int Joy_PB_IN = 14; // Input from pushbutton on joystick. !!! Using Analog Input for better PCB tracing.
const int LED_OUT = 9; // Output to LED Pulse: No comm. Solid: Comm

/*=============================================
            Analog IO
  ============================================= */
const int Joy_X_AIN = 1; // Analog Input pin for joystick
const int Joy_Y_AIN = 2; // Analog Input pin for joystick

/**************************************************************************************
                                      Timers
 **************************************************************************************/
// *** Timer 1: Transmit Interval ***
unsigned long Timer_1_Current;                            // Current time = mills
unsigned long Timer_1_Prev = 0;                           // Previous millis refresh time.
const long Timer_1_Interval = 100;                        // Wait time(milli sec) before timer fires.
bool Timer_1_Fired = false;                               // Timer Enabled

// *** Timer 2: LED Pulse ***
unsigned long Timer_2_Current;                            // Current time = mills
unsigned long Timer_2_Prev = 0;                           // Previous millis refresh time.
const long Timer_2_Interval = 750;                        // Wait time(milli sec) before timer fires.
bool Timer_2_Fired = false;                               // Timer Enabled

/*=============================================
                Variables
  ============================================= */
int Channel_Select = 0; // Summation of the
int Joy_X_Value = 0;  // Value read from joystick
int Joy_Y_Value = 0;  // Value read from joystick
int Joy_PB_Value = 0; // Value from pushbutton on joystick
int Joy_X_Offset; // Offset subtracted from value sent
int Joy_Y_Offset; // Offset subtracted from value sent
int RF_TX_MSG; // Message sent over the RF Module

int Joy_X_RF_IND = 1000; // Offset of value for car to id what value was sent
int Joy_Y_RF_IND = 2000; // Offset of value for car to id what value was sent
int Joy_PB_RF_IND = 3000; // Offset of value for car to id what value was sent

byte Transmit_X_Y = 0;  // Choose whether the X, Y, or PB value is sent. This is toggled each Timer_1 pulse
byte TX_Error_Cntr = 0; // Counter of how many times it has failed
byte TX_Error_Cntr_Lim = 150; // Number of times it can fail before faulting
bool Comm_Error; // True: Comm Error
bool LED_Status;

void setup() {
  printf_begin();

  // *** Digital IO Pinmode ***
  pinMode(Dip_Pin_0_IN, INPUT_PULLUP ); // Setup inputs
  pinMode(Dip_Pin_1_IN, INPUT_PULLUP ); // Setup inputs
  pinMode(Dip_Pin_2_IN, INPUT_PULLUP ); // Setup inputs
  pinMode(Dip_Pin_3_IN, INPUT_PULLUP ); // Setup inputs
  pinMode(Dip_Pin_4_IN, INPUT_PULLUP ); // Setup inputs
  pinMode(Joy_PB_IN, INPUT_PULLUP); // Setup inputs
  pinMode(LED_OUT, OUTPUT); // Setup outputs

  // *** Read in what channel to communicate with (0 to 31)***
  Channel_Select = (digitalRead(Dip_Pin_4_IN) * (16) + digitalRead(Dip_Pin_3_IN) * (8) + digitalRead(Dip_Pin_2_IN) * (4) + digitalRead(Dip_Pin_1_IN) * (2) + digitalRead(Dip_Pin_0_IN) * (1));

  // *** Setup radio settings ***
  SPI.begin();  // Start SPI Communication
  radio.begin();                           // Setup and configure rf radio
  radio.setPALevel(RF24_PA_MAX);           // Set amplification level
  radio.setDataRate(RF24_1MBPS);           // Set data rate
  radio.setRetries(2, 15);                 // Delay between retry
  radio.setCRCLength(RF24_CRC_8);          // CRC Data detection
  radio.openWritingPipe(pipes[Channel_Select]);         // Open writing pipes
  radio.printDetails();                   // Print Configuration

  Calibrate_Joytick(); // Calibrate joystick
}

void loop() {
  Timer_Refresh();                         // Refresh Timer values
  Joy_Stick();                             // Read in Joy stick values
  RF_Transmit();                          // Transmit
  LED(); // Update LED
}

/**************************************************************************************
                                Timer Refresh
 **************************************************************************************/
void Timer_Refresh()
{
  // *** Reload timer values ***
  Timer_1_Current = millis();                            // Refresh timer's value.
  Timer_2_Current = millis();                            // Refresh timer's value.

  // *** Disable all timers ***
  Timer_1_Fired = false;                                 // Reset Timer 1
  Timer_2_Fired = false;                                 // Reset Timer 2

  // *** Check Timer 1 ***
  if ((Timer_1_Current - Timer_1_Prev) >= Timer_1_Interval)
  {
    Timer_1_Prev = Timer_1_Current;                      // Previous timer variable updated
    Timer_1_Fired = true;                                // Set timer fired true for one scan
  }
  // *** Check Timer 2 ***
  if ((Timer_2_Current - Timer_2_Prev) >= Timer_2_Interval)
  {
    Timer_2_Prev = Timer_2_Current;                      // Previous timer variable updated
    Timer_2_Fired = true;                                // Set timer fired true for one scan
  }
}

/******************************************************************************
                              Joy Stick
 ******************************************************************************/
void Joy_Stick()
{
  // *** Read Analog Inputs ***
  Joy_X_Value = analogRead(Joy_X_AIN); // Read value in on analog input
  Joy_Y_Value = analogRead(Joy_Y_AIN); // Read value in on analog input

  // *** Scale the value down to be 8 bits ***
  Joy_X_Value = map(Joy_X_Value, 0, 1023, 0, 255);    // Scale the value to be within a byte range (0 to 255)
  Joy_Y_Value = map(Joy_Y_Value, 0, 1023, 0, 255);    // Scale the value to be within a byte range (0 to 255)
  // *** Read PB on joystick with ANALOG input *** (boolean)
  Joy_PB_Value = digitalRead(Joy_PB_IN); // Read in pushbutton on joy stick
  // Reverse the logic so it becomes postive logic (Internal pullup)
  if (Joy_PB_Value == 0) // Button pressed
  {
    Joy_PB_Value = 1;     // Button pressed
  }
  else // NO Button pressed
  {
    Joy_PB_Value = 0; // NO Button pressed
  }
}

/******************************************************************************
                              Radio Transmission
 ******************************************************************************/
void RF_Transmit()
{
  // *** If Timer 1 Fired, transmit data ***
  if (Timer_1_Fired)
  {
    switch (Transmit_X_Y)
    {

      
      // *** Transmit X Value ***
      case 0:
    //    Serial.println("Transmit X");
        RF_TX_MSG = Joy_X_Value - Joy_X_Offset + Joy_X_RF_IND; // Offset the value by 1000 to indicate X direction
        // If the value goes under minimum limit, fix it
        if (RF_TX_MSG < Joy_X_RF_IND)
        {
          RF_TX_MSG = Joy_X_RF_IND;
        }
        if (!radio.write(&RF_TX_MSG, sizeof(RF_TX_MSG))) {

          TX_Error_Cntr++;
        }
        else {
          TX_Error_Cntr = 0; // Reset counter
        }
        Transmit_X_Y = 1;
        break;
      // *** Transmit Y Value ***
      case 1:
    //    Serial.println("Transmit Y");
        RF_TX_MSG = Joy_Y_Value - Joy_Y_Offset + Joy_Y_RF_IND; // Offset the value by 2000 to indicate Y direction
        // If the value goes under minimum limit, fix it
        if (RF_TX_MSG < Joy_Y_RF_IND)
        {
          RF_TX_MSG = Joy_Y_RF_IND;
        }
        if (!radio.write( &RF_TX_MSG, sizeof(RF_TX_MSG ))) {

          TX_Error_Cntr++;
        }
        else {
          TX_Error_Cntr = 0; // Reset counter
        }
        Transmit_X_Y = 2;
        break;
      // *** Transmit PB Value ***
      case 2:
    //    Serial.println("Transmit PB");
        RF_TX_MSG = Joy_PB_Value + Joy_PB_RF_IND; // Offset the value by 2000 to indicate Y direction
        // If the value goes under minimum limit, fix it
        if (RF_TX_MSG < Joy_PB_RF_IND)
        {
          RF_TX_MSG = Joy_PB_RF_IND;
        }
        if (!radio.write( &RF_TX_MSG, sizeof(RF_TX_MSG ))) {

          TX_Error_Cntr++;
        }
        else {
          TX_Error_Cntr = 0; // Reset counter
        }
        Transmit_X_Y = 0;
        break;
    }

    if (TX_Error_Cntr >= TX_Error_Cntr_Lim)
    {
      Comm_Error = true; // Comm errors. Start pulsing LED
      TX_Error_Cntr = TX_Error_Cntr_Lim + 1; // Avoid counter overflow
      
    }
    else {
      Comm_Error = false; // No errors
    }
  }
}

/******************************************************************************
                              LED Status
 ******************************************************************************/
void LED()
{
  if (Comm_Error)
  {
    if (Timer_2_Fired)
    {
      if (LED_Status)
      {
        digitalWrite(LED_OUT, HIGH);
        LED_Status = false;
      }
      else {
        digitalWrite(LED_OUT, LOW);
        LED_Status = true;
      }
    }
  }
  else
  {
    digitalWrite(LED_OUT, HIGH);
    LED_Status = true;
  }
}

/**************************************************************************************
                                Calibrate Joy Stick
 **************************************************************************************/
void Calibrate_Joytick()
{
  int Joy_X_Sum = 0; // Sum of values to average
  int Joy_Y_Sum = 0; // Sum of values to average
  int Joy_Stick_Center = 127; // Center of joystick (0 to 255)
  int Joy_X_Avg = 0; // Average value read
  int Joy_Y_Avg = 0; // Average value read
  int x;
  for ( x = 0; x <= 5; x++)
  {
    // *** Read Analog Inputs ***
    Joy_X_Value = analogRead(Joy_X_AIN); // Read value in on analog input
    Joy_Y_Value = analogRead(Joy_Y_AIN); // Read value in on analog input

    // *** Scale the value down to be 8 bits ***
    Joy_X_Value = map(Joy_X_Value, 0, 1023, 0, 255);    // Scale the value to be within a byte range (0 to 255)
    Joy_Y_Value = map(Joy_Y_Value, 0, 1023, 0, 255);    // Scale the value to be within a byte range (0 to 255)

    Joy_X_Sum = Joy_X_Sum + Joy_X_Value; // Sum all the values together
    Joy_Y_Sum = Joy_Y_Sum + Joy_Y_Value; // Sum all the values together
    delay(50); // Delay
  }

  Joy_X_Avg = Joy_X_Sum / x; // Calculate average
  Joy_Y_Avg = Joy_Y_Sum / x; // Calculate average

  Joy_X_Offset = Joy_X_Avg - Joy_Stick_Center; // Offset to be applied
  Joy_Y_Offset = Joy_Y_Avg - Joy_Stick_Center; // Offset to be applied
}














