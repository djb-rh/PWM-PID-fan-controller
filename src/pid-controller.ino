/*

  Written by djb@donniebarnes.com. Placed in the public domain. Do with it what you will.

*/

#include <pid.h>
#include <LiquidCrystal_I2C_Spark.h>
#include <clickButton.h>

#include <Particle.h>
// #include <spark_wiring_i2c.h>

#include "elapsedMillis.h"

// SHT31 I2C address is 0x44(68)
#define Addr 0x44

int temp = 0;
double cTemp = 0.0, fTemp = 0.0, humidity = 0.0;

int setFanSpeed(String speed);
char publishString[40];

//wiring
int relayPin = D2;  // not currently actually used
int fanpin = D3;    // pwm signal out to fan(s)
int pinYellow = D6; // up
int pinBlue = D7;   // down

// This all uses the elapsedMillis library to setup the ability to turn on the backlight for 10s at a time
// #define LCD_BACKLIGHT_INTERVAL 60000
#define LCD_BACKLIGHT_INTERVAL 6000000
#define SP_INCREMENTS 0.2
elapsedMillis lcdBacklightInterval;
bool backlight = false;

ClickButton buttonYellow(pinYellow, LOW, CLICKBTN_PULLUP);
ClickButton buttonBlue(pinBlue, LOW, CLICKBTN_PULLUP);

int buttonYellowClicks = 0;
int buttonBlueClicks = 0;

double fanspeed = 150; // default fan speed

void maingateHandler(const char *eventname, const char *data);
void clubgateHandler(const char *eventname, const char *data);
int mainstatus(String command);
int clubstatus(String command);

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 20 chars and 4 line display

// double kp=2;   //proportional parameter
// double ki=5;   //integral parameter
// double kd=1;   //derivative parameter
// these next three work fine for the V1 air handler
//double kp=75;   //proportional parameter
//double ki=1;   //integral parameter
//double kd=3;   //derivative parameter
double kp=200;   //proportional parameter
double ki=.5;   //integral parameter
double kd=2.5;   //derivative parameter
// Minimum and Maximum PWM command, according fan specs and noise level required
// My fans draw 1.05A at full bore but my power supply is a 2A (24V), so I may need to put a meter
// on it and see what Max would keep it at 1A. The fans also won't start up to something below 
// around 50 maybe, so I need to be above that. But 90 is fine as they are always going to need
// more cooling than that.
double commandMin = 20;
double commandMax = 255;
// default starter temp. 
double setTemp = 64.5;
// stupid variable to see if we've done this thing yet
int boottask = 0;

// declare my PID. Give it the temp from the sensor, the fanspeed variable to set, the target temp, 
// the proportioning parameters, and make it REVERSE, which means target cooling.
PID myPID(&fTemp, &fanspeed, &setTemp, kp, ki, kd, PID::REVERSE);


/* This function is called once at start up ----------------------------------*/
void setup()
{
   
    Particle.variable("fTemp", fTemp);
    Particle.variable("humidity", humidity);
    Particle.variable("fanspeed", fanspeed);
    Particle.variable("kp", kp);
    Particle.variable("ki", ki);
    Particle.variable("kd", kd);
    Particle.variable("sp", setTemp);

    Particle.function("backlight", backlightSet);
    Particle.function("kp", kpSet);
    Particle.function("ki", kiSet);
    Particle.function("kd", kdSet);
    Particle.function("sp", spSet);
    
    // listen to turn on or off from main gate ID and call a handler if it changes, same for clubhouse gate
    Particle.subscribe("main_gate_1", maingateHandler);
    Particle.subscribe("clubhouse_gate_1", clubgateHandler);

    // setup some buttons
    pinMode(pinYellow, INPUT_PULLUP);
    pinMode(pinBlue, INPUT_PULLUP);
    
    pinMode(fanpin, OUTPUT);  // set the fanpin to an output
    pinMode(relayPin, OUTPUT); // set the relay pin to an output
    
    digitalWrite(relayPin, HIGH); // turn on relay at startup
    
    // variables that need to be setup for the button debounce library
    // buttonYellow.debounceTime   = 20;   // Debounce timer in ms
    // buttonYellow.multiclickTime = 250;  // Time limit for multi clicks
    // buttonYellow.longClickTime  = 1000; // time until "held-down clicks" register
    
    // some things we can set from the Particle app
    Particle.function("relayState", djbRelay);

	Serial.begin(115200);
    lcd.init();  //initialize the lcd

    // turn on the backlight at boot time (it'll turn off after 10s)
    lcd.backlight();  // turn backlight on
    backlight = true;
  
    lcd.setCursor (0, 0 );            // go to the top left corner
    lcd.print("Main Gate: DUNNO"); // write this string on the top row
    lcd.setCursor (0, 1 );            // go to the third row
    lcd.print("CH Gate: DUNNO"); // pad with spaces for centering
    lcd.setCursor(17,3);
    sprintf(publishString, "%3.0f", fanspeed);
    lcd.print(publishString);
    lcd.setCursor(16,2);
    sprintf(publishString, "%3.1f", setTemp);
    lcd.print(publishString);

    //turn the PID on
    myPID.SetMode(PID::AUTOMATIC);
    myPID.SetOutputLimits(commandMin, commandMax);

    // do a request for the state and the above subscribes will get called when they see the request
    // delay(10000);
    // Particle.publish("getstate", "1");


}

/* This function loops forever --------------------------------------------*/
void loop()
{

    // if we're connected to the cloud and we haven't done this yet, publish the getstate
    // need this because cloud connection might not happen for a few seconds, but we can be doing other
    // things until then
    if(!boottask){
        if(Particle.connected()){
            Particle.publish("getstate", "1");
            boottask=1;
        }
    }
    // these functions get the button data
    buttonYellow.Update();
    buttonBlue.Update();
    
    if(buttonYellow.clicks != 0) buttonYellowClicks = buttonYellow.clicks;
    if(buttonBlue.clicks != 0) buttonBlueClicks = buttonBlue.clicks;
    
    if (buttonYellowClicks == 1) 
    {
        if (!backlight) { // if backlight is OFF
            lcd.backlight(); // turn backlight on
            backlight = true; // backlight is now on
            lcdBacklightInterval = 0;
        } else {
	        setTemp = setTemp+SP_INCREMENTS;
	        lcd.setCursor(16,2);
	        sprintf(publishString, "%3.1f", setTemp);
	        lcd.print(publishString);
                lcdBacklightInterval = 0; // reset the interval timer
        }

    }
    
    if (buttonBlueClicks == 1) 
    {
        if (!backlight) { // if backlight is OFF
            lcd.backlight(); // turn backlight on
            backlight = true; // backlight is now on
            lcdBacklightInterval = 0;
        } else {
	        setTemp = setTemp-SP_INCREMENTS;
	        lcd.setCursor(16,2);
	        sprintf(publishString, "%3.1f", setTemp);
	        lcd.print(publishString);
                lcdBacklightInterval = 0; // reset the interval timer
        }

    }
    
    buttonYellowClicks = 0;
    buttonBlueClicks = 0;
    delay(5);
 
    // get temp and do things with it
    unsigned int data[6];
    // Start I2C Transmission
    Wire.beginTransmission(Addr);
    // Send 16-bit command byte
    Wire.write(0x2C);
    Wire.write(0x06);
    // Stop I2C transmission
    Wire.endTransmission();
    // delay(300);

    // Start I2C Transmission
    Wire.beginTransmission(Addr);
    // Select data register
    Wire.write(0x00);
    // Stop I2C Transmission
    Wire.endTransmission();

    // Request 6 bytes of data from the device
    Wire.requestFrom(Addr, 6);

    // Read 6 bytes of data
    // temp msb, temp lsb, crc, hum msb, hum lsb, crc
    if (Wire.available() == 6)
    {
        data[0] = Wire.read();
        data[1] = Wire.read();
        data[2] = Wire.read();
        data[3] = Wire.read();
        data[4] = Wire.read();
        data[5] = Wire.read();
    }

    // Convert the data
    temp = (data[0] * 256) + data[1];
    cTemp = -45.0 + (175.0 * temp / 65535.0);
    fTemp = (cTemp * 1.8) + 32;
    humidity = 100 * ((data[3] * 256) + data[4]) / 65535.0;

    lcd.setCursor(0, 2);
    sprintf(publishString, "Temp %2.2f", fTemp);
    lcd.print(publishString);
    lcd.setCursor(0,3);
    sprintf(publishString, "Humidity: %2.2f", humidity);
    lcd.print(publishString);

    //process PID
    myPID.Compute();

    // write new fanspeed (25khz carrier signal is what these fans want)
    analogWrite(fanpin, fanspeed, 25600);
    lcd.setCursor (17,3);
    sprintf(publishString, "%3.0f", fanspeed);
    lcd.print(publishString);

    if (backlight) { // if backlight is on
        if (lcdBacklightInterval > LCD_BACKLIGHT_INTERVAL) { // check to see if it's time to turn it off
            backlight = false; // set the state to off
            lcd.noBacklight(); // turn it off
        }
    }

}

void maingateHandler(const char *eventname, const char *data)
{
    // pass the command that was sent to the main gate down to us
    mainstatus(data);
}

int mainstatus(String command)
{
    if(command == "ON")
    {
         lcd.setCursor (0, 0);            // go to the 2nd row
         lcd.print("Main Gate: Open     "); // pad string with spaces for centering
	return 1;
    }
    if(command == "OFF")
    {
         lcd.setCursor (0, 0);            // go to the 2nd row
         lcd.print("Main Gate: Closed    "); // pad string with spaces for centering
	return 1;

    }
    return 0;
}

void clubgateHandler(const char *eventname, const char *data)
{
    // pass the command that was sent to the clubhouse gate down to us
    clubstatus(data);
}

int clubstatus(String command)
{
    if(command == "ON")
    {
         lcd.setCursor (0, 1);            // go to the 2nd row
         lcd.print("CH Gate: Open     "); // pad string with spaces for centering
	return 1;
    }
    if(command == "OFF")
    {
         lcd.setCursor (0, 1);            // go to the 2nd row
         lcd.print("CH Gate: Closed    "); // pad string with spaces for centering

	return 1;
    }
    return 0;
}


int djbRelay(String command){
	if(command.equalsIgnoreCase("on")){
		digitalWrite(relayPin, HIGH);
		return 1;
	}
	if(command.equalsIgnoreCase("off")){
		digitalWrite(relayPin, LOW);
		return 1;
	}
	return 0;
}

int backlightSet(String command){
	if(command.equalsIgnoreCase("on")){
		lcd.backlight();
                backlight = true;
                lcdBacklightInterval = 0; // reset the counter to 0 because it just turned on
		return 1;
	}
	else {
            lcd.noBacklight();
            backlight = false;
        }
	return 1;
}

int kpSet(String command){

    kp = command.toFloat();
    myPID.SetTunings(kp, ki, kd);
    return 1;

}

int kiSet(String command){

    ki = command.toFloat();
    myPID.SetTunings(kp, ki, kd);
    return 1;

}

int kdSet(String command){

    kd = command.toFloat();
    myPID.SetTunings(kp, ki, kd);
    return 1;

}

int spSet(String command){

    setTemp = command.toFloat();
    lcd.setCursor(16,2);
    sprintf(publishString, "%3.1f", setTemp);
    lcd.print(publishString);
    return 1;

}


