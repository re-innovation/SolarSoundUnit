/*
  The code for a Solar Powered Birdsong Unit
  This is recharged by solar PV power
  It has a PIR which sets off birdsong stored onto
  a SoundOut MP3 Audio Module (EA MOD 1021) from Embedded Adventures
  http://www.embeddedadventures.com/soundout_mp3_module_mod-1021.html

  The power to the sound unit and amplifier are controlled
  using a digital output pin.

  This must have a decent power supply (eg. battery) or it will brown-out reset.

  This runs on 
  A Nano 5V
  A SoundOut MP3 Audio Module (EA MOD 1021) from Embedded Adventures
  http://www.embeddedadventures.com/soundout_mp3_module_mod-1021.html
  A li-po rider + PV module is used to recharge a 2000mAh li-ion battery
  A simple PIR sensor is used to trigger the unit.

  To reduce power consumption the unit goes to sleep a LOT.
  An interuupt from the PIR pin causes it to wake up
  
  If its triggered the a random sound file is played.
  If not then it just goes back to sleep again.
  Current when asleep = 1-1.5mA
  Current when awake = 20mA
  Current when playing sounds = 100-200mA.

  DFPlayer Mini
  Datasheet:
  https://www.openhacks.com/uploadsproductos/dfplayer_mini_-_robot_wiki.pdf
  Code:
  http://www.dfrobot.com/wiki/index.php/DFPlayer_Mini_SKU:DFR0299
  
  Use LDR to sense if dark or not.
  If dark then do not play sounds.

  Volume dial is on pin A4 - this is mapped to 0-31 and set as the volume.
  
  More infomation here:
  matt@re-innovation.co.uk
  www.re-innovation.co.uk

*/

/*
  Updates:
  5/5/15  First version of the code - Matt Little
  1/6/15  Adding sleep code to reduce power consumption - Matt Little
  18/5/16 Changing player to DFPlayer Mini. Same unit. - Matt Little
  18/5/16 Sorting MP3 file playing issue. Should always play the same file.
  26/5/16 Adjusting volume
*/

#include <stdlib.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <SoftwareSerial.h>
#include <DFPlayer_Mini_Mp3.h>

// ***** Digital I/O *********
#define powerControl  6
#define pirInterrupt 2
#define led 13
#define ldrEnable 5   // This is set low to enable the ldr reading
#define ldrAdjust A2
#define volumePin A4  

// Sound Module Connections
#define TX 11    // " 12
#define RX 10   // " 13

// Define the MP3 Module's Commands that we'll be using
#define cmdNextTrack  0x01                  // Next track.      
#define cmdPrevTrack  0x02                  // Previous track.
#define cmdRepeatPlay 0x11                  // Repeat play.

#define cmdSetVolume  0x06                  // Set Volume to a specified value.
#define cmdSetEq      0x07                  // Set Equalizer to a specified value.
#define cmdReset      0x0c                  // Reset MP3 Module.
#define cmdsdTracks   0x47                  // Reset MP3 Module.
#define cmdLowPower   0x0A                  // Put module into Low Power mode

// ******* Analogue I/O **********
int tuneBusy = A0;  // Measure when the audio module has finnished.
int ldr = A1; // a light sensor for measuring if dark 

int tune;  // This is the tune file to play

boolean tooDark = LOW;  // Flag in case its too dark...

// *********** USER VARIABLES ************************//

int tuneMax = 20;  // This is the maximum tune ID (usually 20)

int darkSetpoint = 600; // Adjust for the switch off - Adjusted by POT now.

int delaySec = 2; // delay between files playing

unsigned int volume = 10;                   // Starting MP3 module volume (0 - 31).

// ********** END OF USER VARIABLES *****************//
int sensorValue = 0;  // Holds the busy pin sensor value.

boolean busyPin = 0;  // This is a flag for the busyPin

SoftwareSerial mp3(RX, TX); // RX, TX


unsigned int eq = 0;                        // Normal/Pop/Rock/Jazz/Classic/Base equalizer.

unsigned int playTune = 0;  // This holds a random number from



void interruptPIR() {
  // Want to wake up from sleep here
  //sleep_disable(); 
  // An interrupt on Pin D2 should wake this up.
  detachInterrupt(0);
  sleep_disable();

}



/***************************************************
 *  Name:        enterSleep
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Enters the arduino into sleep mode.
 *
 ***************************************************/
void enterSleep(void)
{
  
  sleep_enable();
  // Attach interrupt again
  attachInterrupt(0, interruptPIR, LOW);
  delay(100);
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  
  
  cli();
  sleep_bod_disable();
  sei();
  sleep_cpu();
  
  /* The program will continue from here. */

  
  /* First thing to do is disable sleep. */
  sleep_disable(); 
}

void setup()
{
  
  // Set all other pins to input with pull_up to reduce power consumption

  DDRD &= B01000011;       // set Arduino  as inputs, leaves 0,1 (RX & TX) 2 & 5 as is
  DDRB = B00000000;        // set pins 8 to 13 as inputs, leaves 10,11 as is
  PORTD |= B10111100;      // enable pullups on pins 2 to 7, leaves 0,1 (RX & TX) 2 & 5 as is
  PORTB |= B11111111;      // enable pullups on pins 8 to 13, leaves 10,11 as is
  
  // initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards:
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);             // MP3 module busy signal is active low.
  
  pinMode(ldrEnable, INPUT_PULLUP);
  
  pinMode(RX,INPUT);  
  pinMode(TX,OUTPUT);
  pinMode(powerControl, OUTPUT);  
  digitalWrite(powerControl, LOW);
  
  Serial.begin(9600);
  mp3.begin(9600);   // Initialise software serial.
                         
  mp3_set_serial (mp3); //set Serial for DFPlayer-mini mp3 module  
  randomSeed(analogRead(A3));  // Read a disconnected pin to seed the random algorithm
}


void loop()
{ 

    
    if(digitalRead(pirInterrupt)==HIGH)
    {
      Serial.println("Going to sleep...");
      /* Re-enter sleep mode. */
      enterSleep();
    }

    darkSetpoint = analogRead(ldrAdjust);
    Serial.print("darkSetpoint = ");
    Serial.println(darkSetpoint);
    
  
    // First check the LDR
    // If dark then do not play sounds

    if(digitalRead(ldrEnable)==LOW)
    {
      Serial.println(analogRead(ldr));
      if(analogRead(ldr)>=darkSetpoint)
      {
        Serial.println("Too Dark");
        digitalWrite(led, HIGH);
        tooDark = HIGH;
      }
      else
      {
        tooDark=LOW;
      }
    }
    
    if(digitalRead(ldrEnable)==HIGH||tooDark==LOW)
    {
    
    // Only if the PIR is activated do we play the tune:
    // This code will just wake up if the PIR is activated
    // If PIR is triggered then it will play a random bird sample.
    // Otherwise it just goes to sleep again
      Serial.println("PIR TRIGGERED");
      digitalWrite(led, HIGH);
      digitalWrite(powerControl, HIGH);
      
      for (int i = 0; i <= 100; i++)
      {
        delay(10);
      }
      mp3_reset;
      // Read volume from Pin A4 (potentiometer on this)
      volume = map(analogRead(volumePin), 0, 1023, 0, 31);
      Serial.print("Volume:");
      Serial.println(volume);
      
      mp3_set_volume (volume);    
      delay(100);
  
      // Randomly decide on the next tune:
      Serial.print("Next Track:");
      tune = random(1,(tuneMax+1));
      
      Serial.println(tune);

      mp3_play (tune);
           
     for(int i=0;i<200;i++)
     {
        sensorValue = analogRead(tuneBusy);
        delay(10);
     }  
      
      // Wait until track finished
      while(sensorValue<=400) // Busy signal is ACTIVE LOW
      {
        for(int i=0;i<=10;i++)
        {
          delay(10);
        }
        sensorValue = analogRead(tuneBusy);
      }
      
      digitalWrite(powerControl, LOW);     
      // Switch OFF power
      for (int i = 0; i <= 100; i++)
      {
        delay(10);
      }
    }
    delay(delaySec*1000);
    digitalWrite(led, LOW);             // MP3 module busy signal is active low
}

