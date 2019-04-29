/*
   Nathan Huber
   DACME Final Project
   RC Car Controller
   03/26/2019
*/

/*=============================================
               Car Model A
  ============================================= */

/*=============================================
            NRF24L01 Radio Library
  ============================================= */

#include  <Arduino.h>
#include  <SPI.h>
#include  <Servo.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "RF24_config.h"
#include "printf.h"


//byte pipes[][31] = {"A0000", "A0001", "A0002", "A0003", "A0004", "A0005", "A0006", "A0007", "A0008", "A0009", "A0010", "A0011", "A0012", "A0013", "A0014", "A0015", "A0016", "A0017", "A0018", "A0019", "A0020", "A0021", "A0022", "A0023", "A0024", "A0025", "A0026", "A0027", "A0028", "A0029", "A0030", "A0031"};
byte pipes[][31] = {"B0000","B0001","B0002","B0003","B0004","B0005","B0006","B0007","B0008","B0009","B0010","B0011","B0012","B0013","B0014","B0015","B0016","B0017","B0018","B0019","B0020","B0021","B0022","B0023","B0024","B0025","B0026","B0027","B0028","B0029","B0030","B0031"};
RF24 radio(8, 7);


/*=============================================
            Digital IO
  ============================================= */
const int Dip_Pin_0_IN=0;  // Input pin on the dip switch package
const int Dip_Pin_1_IN=1;  // Input pin on the dip switch package
const int Dip_Pin_2_IN=2;  // Input pin on the dip switch package
const int Dip_Pin_3_IN=3;  // Input pin on the dip switch package
const int Dip_Pin_4_IN=4;  // Input pin on the dip switch package

/*=============================================
            Analog IO
  ============================================= */                                          
Servo ServL;  // Create servo object to control left servo
Servo ServR;  // Create servo object to control right servo

/**************************************************************************************
                                      Timers
 **************************************************************************************/
// *** Timer 1: Update ESC Motor Speed ***
unsigned long Timer_1_Current;                            // Current time = mills
unsigned long Timer_1_Prev = 0;                           // Previous millis refresh time. Timer 1: RF Transmit
const long Timer_1_Interval = 100;                        // Wait time(milli sec) before timer fires. Timer 1: RF Transmit
bool Timer_1_Fired = false;                               // Timer Enabled


/*=============================================
                Variables
  ============================================= */
int Channel_Select = 0; // Summation of the
unsigned long RX_RX_MSG;
bool RF_Rx;  // No RX Message
int RF_Timeout; // Timeout of waiting to RX message
int RF_Timeout_Period = 500; // Time in milliseconds till we stop waiting for response
int RF_RX_MSG; // Message received over the RF Module

int Joy_X;
int Joy_Y;
int SetLeft;
int SetRight;
const int SpeedLimit = 120; //110 - 180
bool skip;
/*=============================================
        Forward Function Declaration
  ============================================= */
  void Timer_Refresh();                         
  void RF_Receive(); 
  bool Button_Control();
  void Servo_Control();
  int SpeedLimiter(int ServoVal);

/*=============================================
                Setup Loop
============================================= */

void setup() 
{
  printf_begin();
  Serial.begin(115200); // Serial begin
  // *** Digital IO Pinmode ***
  pinMode(Dip_Pin_0_IN, INPUT_PULLUP ); // Setup inputs
  pinMode(Dip_Pin_1_IN, INPUT_PULLUP ); // Setup inputs
  pinMode(Dip_Pin_2_IN, INPUT_PULLUP ); // Setup inputs
  pinMode(Dip_Pin_3_IN, INPUT_PULLUP ); // Setup inputs
  pinMode(Dip_Pin_4_IN, INPUT_PULLUP ); // Setup inputs

  // *** Read in what channel to communicate with (0 to 31)***
  Channel_Select = (digitalRead(Dip_Pin_4_IN) * (16) + digitalRead(Dip_Pin_3_IN) * (8) + digitalRead(Dip_Pin_2_IN) * (4) + digitalRead(Dip_Pin_1_IN) * (2) + digitalRead(Dip_Pin_0_IN) * (1));
  Serial.print("Channel_Select: ");
  Serial.println(Channel_Select);
  // *** Setup radio settings ***
  SPI.begin();  // Start SPI Communication
  radio.begin();                           // Setup and configure rf radio
  radio.setPALevel(RF24_PA_MAX);           // Set amplification level
  radio.setDataRate(RF24_1MBPS);           // Set data rate
  radio.setRetries(2, 15);                 // Delay between retry
  radio.setCRCLength(RF24_CRC_8);          // CRC Data detection
  radio.openReadingPipe(1, pipes[Channel_Select]);        // Open reading pipes
  radio.printDetails();                    // Print Configuration
  // *** Setup servo settings ***
  ServR.attach(5);  // attaches the servo on pin 9 to the servo object
  ServL.attach(6); // attaches the servo on pin 9 to the servo object
  ServR.write(90);  
  ServL.write(90);
}

void loop() 
{
  Timer_Refresh();                         // Refresh Timer values
  RF_Receive();                            // Receive
  
  if(Button_Control() == false)
  {
    Servo_Control();
  }


  
  //Serial.println("");
  //Serial.print("RF_Rx: ");
  //Serial.println(RF_Rx);
  //Serial.print("RX_RX_MSG: ");
  //Serial.println(RX_RX_MSG);
}

/**************************************************************************************
                                Timer Refresh
 **************************************************************************************/
void Timer_Refresh()
{
  // *** Reload timer values ***
  Timer_1_Current = millis();                            // Refresh timer's value.

  // *** Disable all timers ***
  Timer_1_Fired = false;                                 // Reset Timer 1

  // *** Check Timer 1 ***
  if ((Timer_1_Current - Timer_1_Prev) >= Timer_1_Interval)
  {
    Timer_1_Prev = Timer_1_Current;                      // Previous timer variable updated
    Timer_1_Fired = true;                                // Set timer fired true for one scan
  }
}


/******************************************************************************
                              Radio Receive
 ******************************************************************************/
void RF_Receive()
{
  radio.startListening();

  unsigned long RF_Timeout = millis();               // Timeout to listen for receive
  RF_Rx = true;                                   // RX Message not received reset

  while ( !radio.available() ) 
  {                     // Wait to receive message
    if (micros() - RF_Timeout > RF_Timeout_Period ) 
    {           // Break loop if no message received
      RF_Rx = false;
      break;
    }
  }

  if (RF_Rx)   // Message received
  {
    radio.read( &RX_RX_MSG, sizeof(unsigned long) );
  }
  else
  {
    RX_RX_MSG=0;  // Clear the message
  }
}
/******************************************************************************
                              Spin Mode
 ******************************************************************************/
bool Button_Control()
{
  int Joy_PB;

  if(RX_RX_MSG >= 3000)                                   // Looks for button value
  {
    Joy_PB  = RX_RX_MSG - 3000;                          // Sets "button" to message

      if(Joy_PB == 1)
      {
          SetLeft = 50;
          SetRight = 50;
          ServL.write(SetLeft);
          ServR.write(SetRight);
          return true;
          Serial.println("true");
      }
      else
      {
        return false;

      }
    
    
  }

}

/******************************************************************************
                              Servo Control
 ******************************************************************************/
void Servo_Control()
{

  bool gotX = false;
  bool gotY = false; 
  bool zero = true;

  if(RX_RX_MSG >= 1000 && RX_RX_MSG <= 1255 && RF_Rx == true)               // Looks for Y value
  {
    gotY = true;
    Joy_Y = RX_RX_MSG - 1000;                                               //Removes offset
    if(Joy_Y >= 105 && Joy_Y <= 160 && (Joy_X >= 105 && Joy_X <= 160))
    {
      zero = true;
      SetLeft = 90;
      SetRight = 90;
      ServL.write(SetLeft);
      ServR.write(SetRight);
      // Serial.println("     ");
      // Serial.println("stop");
    }
    else
    {
      zero = false;
    }
    
    // Serial.print("Joy_Y:  ");
    // Serial.print(Joy_Y);
    // Serial.print("\r");
       
  }

  else if(RX_RX_MSG >= 2000 && RX_RX_MSG <= 2255 && RF_Rx == true)          // Looks for X value
  {
    gotX = true;
    Joy_X = RX_RX_MSG - 2000;                                                   //Removes offset
    // Serial.print("Joy_X:  ");
    // Serial.print(Joy_X);
    // Serial.print("     ");   
  }




  if(zero == false && (gotX == true || gotY == true))                // Joystick not at origin and X or Y recieved
  {
    if (Joy_Y > 150 )                                                // Move Forward if Y > 150 out of 0-255
    {

       SetLeft = SpeedLimiter(map(Joy_Y, 0, 255, 0, 180));           // scale 0-255 to 0-180, Left servo forward from 90 - 180 -> && SpeedLimiter
       SetRight = SpeedLimiter(map(Joy_Y, 0, 255, 180, 0));          // scale 0-255 to 180-0, Right servo forward from 90 - 0 && -> SpeedLimiter

        // Serial.println("     ");
        // Serial.println("Forward");
      if(Joy_X > 160)                                                // Forward Right
      {
        SetLeft = 105;                                               // Set left slower than right
        SetRight = SpeedLimiter(map(Joy_Y, 0, 255, 180, 0));         // Scale 0-255 to 180-0, Right servo forward from 90 -> 0 && -> SpeedLimiter
        // Serial.println("     ");
        // Serial.println("Slight Right");

      }
      if(Joy_X < 105)                                                // Forward Left
      {

        SetRight = 75;                                               // Set right slower than left
        SetLeft = SpeedLimiter(map(Joy_Y, 0, 255, 0, 180));          // Scale 0-255 to 0-180, Left servo forward from 90 -> 180 && -> SpeedLimiter

        // Serial.println("     ");
        // Serial.println("Slight Left");
      }

      ServL.write(SetLeft);
      ServR.write(SetRight);
    }

    if (Joy_Y < 105 )                                                // Reverse
    
    {
      SetRight = SpeedLimiter(map(Joy_Y, 0, 255, 180, 0));           // scale 0-255 to 0-180, Right servo reverse from 90 -> 180 && -> SpeedLimiter
      SetLeft = SpeedLimiter(map(Joy_Y, 0, 255, 0, 180));            // scale 0-255 to 180-0, Left servo reverse from 90 -> 0 && -> SpeedLimiter

        // Serial.println("Reverse");
      if(Joy_X > 160)                                                // Reverse Right
      {

        SetLeft = 75;
        SetRight = SpeedLimiter(map(Joy_Y, 0, 255, 180, 0));         
        // Serial.println("     ");
        // Serial.println("Reverse Right");
      }

      if(Joy_X < 105)                                                // Reverse Left
      {
        SetRight = 105;                                              // Set left slower than right
        SetLeft = SpeedLimiter(map(Joy_Y, 0, 255, 0, 180)); 

        // Serial.println("     ");
        // Serial.println("Reverse Left");
      }
      ServL.write(SetLeft);                                          // Send values to servo
      ServR.write(SetRight);
    }

    if(Joy_X > 160 && (Joy_Y >= 105 && Joy_Y <= 150))                // Full Right Turn
    {
      SetLeft = 90;                                                  // Left servo off
      SetRight = SpeedLimiter(map(Joy_X, 0, 255, 180, 0));           // Right on
      // Serial.println("     ");
      // Serial.println("Right Full");
      ServL.write(SetLeft);
      ServR.write(SetRight);
    }

    if(Joy_X < 105 && (Joy_Y >= 105 && Joy_Y <= 150))                // Full Left Turn
    {
      SetRight = 90;                                                 // Right off
      SetLeft = SpeedLimiter(map(Joy_X, 0, 255, 180, 0));            // Left on

      // Serial.println("     ");
      // Serial.println("Left Full");
      ServL.write(SetLeft);                                          // Send to servos
      ServR.write(SetRight);
    }
    


  }

}

int SpeedLimiter(int ServoVal)                  // Function to limit speed
{
    if(ServoVal >= SpeedLimit)                  // 90-180 Speed Limit
    {
      return SpeedLimit;                        // Return Set Speedlimit
    }
    else if(ServoVal <= (abs(SpeedLimit-180)))  // 90-0 Speed Limit (Reversed Servo)
    {
      return abs(SpeedLimit-180);               // Basically return 50
    }
    else
    {
      return ServoVal;                          // No limiting needed, Return servo values
    }
    
}