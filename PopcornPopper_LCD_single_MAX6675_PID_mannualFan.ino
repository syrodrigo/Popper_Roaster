/**************************************
 * Version: 0820
 * Add Serial port command (from Roastlogger) control
 **************************************/
 
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>  // Arduino PID Library by Brett Beauregard

#define CELSIUS //Set temperature data in CELSIUS.
#define DP 1  // No. decimal places for serial output
#define maxLength 30                  // maximum length for strings used

// thermocouple reading Max 6675 pins
const int SO  = 2;    // SO pin on MAX6675
const int SCKa = 3;    // SCKa pin on MAX6675
const int CS1 = 6;    // CS (chip 1 select) pin on MAX6675

// set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27,20,4);  

// PID controller
double Input, Setpoint;                          // parameters for PID
double pidOutput = 0;                            //use as %, 100 is always on, 0 is always off
// pid control based on RoR
double P = 30;                                         //P term
double I = 0.4;                                      //I term
double D = 5;                                         //D term
// pid control based on BT
double P2 = 1.0;                                      //P term, lower heating rate
double I2 = 0.01;                                      //I term, lower heating rate
double D2 = 0.0;                                       //D term, lower heating rate

PID myPID(&Input, &pidOutput, &Setpoint, P, I, D, DIRECT);  // higher temperature rising rate
PID myPID2(&Input, &pidOutput, &Setpoint, P2, I2, D2, DIRECT);  // lower temperature rising rate

// set output pin for heating
const int heaterPin =  9;         // digital pin for pulse width modulation of heater


/****************************************************************************
 *  After setting the above pin assignments you can use the remainder of this
 *   sketch as is or change it, if you wish, to add additional functionality
 * 
 ****************************************************************************/

// time constants
const int timePeriod = 2000;          // total time period of PWM milliseconds see note on setupHeat before changing
const int tcTimePeriod = 250;         // 250 ms loop to read thermocouples
 
// thermocouple settings
float calibrate1 = 0.0;  // Temperature compensation for T1

// set global variables

//temporary values for temperature to be read
float temp1 = 0.0;                   // temporary temperature variable
float t1 = 0.0;                      // Last average temperature on thermocouple 1 - average of four readings
float tCumulative1 = 0.0;            // cumulative total of temperatures read before averaged
int noGoodReadings1 = 0;             // counter of number. of good readings for average calculation 
float BT = 0.0;                      // Bean temperature
int tempUpdateCounter = 0;               // counting TC reading update
int lastTempUpdateCounter = 0;               // counting TC reading update

int inByte = 0;                       // incoming serial byte
String inString = String(maxLength);  // input String
const int arduino = 0;                // either overridden if power pot is set below 97% to power pot control
const int computer = 1;              

int pidOn = 0;                       // state of pid control

// loop control variables
unsigned long lastTCTimerLoop = 0;        // for timing the thermocouple loop
int tcLoopCount = 0;                  // counter to run serial output once every 4 loops of 250 ms t/c loop
int startTime = 0;            //roasting start

int controlBy = arduino;              // default is arduino control. PC sends "pccontrol" to gain control or
                                      // swapped back to Arduino control if PC sends "arduinocontrol"

// PID variables - initial values just guesses, actual values set by computer. Not used
//double pidSetpoint, pidInput, pidOutput;
double pidBias = 0;


/***************PID Variables******************/

// coffee roasting profile
const float dryTemp = 150.0; // previously setting: 170.0
float lastTemp = 0.0;
float tempSlope = 0.0;
float DryTempSlope = 0.6; // set bean dry temperature rising rate to 17.5 per minute (total drying time is about 8 minutes)
const float dropTemp = 220; // should not over 230, avoid too much oil
int RoastPhase = 0;         // Roast phase 1: drying; 2:Maillard + Develpment

// Heating variables
int  timeOn;                          // millis PWM is on out of total of timePeriod (timeOn = timePeriod for 100% power)
unsigned long lastTimePeriod = 0;         // millis since last turned on pwm
//int power = 0;                      //use as %, 100 is always on, 0 is always off, default 100, not used. Use pidOutput instead

/*****************************************************************
 * Read the Max6675 device 1 or 2.  Returns temp as float or  -1.0
 * if an error reading device.
 * Note at least 240 ms should elapse between readings of a device
 * to allow it to settle to new reading.  If not the last reading 
 * will be returned again.
 *****************************************************************/
float readThermocouple(int CS, float calibrate) //device selected by passing in the relavant CS (chip select)
{
  int value = 0;
  int error_tc = 0;
  float temp = 0.0;

  digitalWrite(CS,LOW); // Enable device

  // wait for it to settle
  delayMicroseconds(1);

  /* Cycle the clock for dummy bit 15 */
  digitalWrite(SCKa,HIGH);
  digitalWrite(SCKa,LOW);

  //wait for it to settle
  delayMicroseconds(1);

  /* Read bits 14-3 from MAX6675 for the Temp 
   Loop for each bit reading the value and 
   storing the final value in 'temp' 
   */
  for (int i=11; i>=0; i--){
    digitalWrite(SCKa,HIGH);  // Set Clock to HIGH
    value += digitalRead(SO) << i;  // Read data and add it to our variable
    digitalWrite(SCKa,LOW);  // Set Clock to LOW
  }

  // Read the TC input to check for error
  digitalWrite(SCKa,HIGH); // Set Clock to HIGH
  error_tc = digitalRead(SO); // Read data
  digitalWrite(SCKa,LOW);  // Set Clock to LOW

  digitalWrite(CS,HIGH); // Disable device 1

  value = value + calibrate;  // Add the calibration value

  temp = (value*0.25);  // Multiply the value by 0.25 to get temp in ˚C

  // return -1 if an error occurred, otherwise return temp
  if(error_tc == 0) {
    return temp; 
  } 
  else { 
    return -1.0; 
  }
}

/****************************************************************************
 * Read temperatures from Max6675 chips Sets t1 and t2, -1 if an error
 * occurred.  Max6675 needs 240 ms between readings or will return last
 * value again. I am reading it once per second.
 ****************************************************************************/
void getTemperatures()
{
 
  temp1 = readThermocouple(CS1, calibrate1);
  
  if (temp1 > 0.0) 
  {
    tCumulative1 += temp1;
    noGoodReadings1 ++;
  }
}


/****************************************************************************
 * Set up power pwm control for heater.  Hottop uses a triac that switches
 * only on zero crossing so normal pwm will not work.
 * Minimum time slice is a half cycle or 10 millisecs in UK.  
 * Loop in this prog may need up to 10 ms to complete.
 * I will use 20 millisecs as time slice with 100 power levels that
 * gives 2000 milliseconds total time period.
 ****************************************************************************/
void setupHeater() {
  // set the digital pin as output:
  pinMode(heaterPin, OUTPUT);      
  digitalWrite(heaterPin, LOW);//set Heater pin off on start
}

/****************************************************************************
 * Toggles the heater on/off based on the current power level.  Power level
 * may be determined by arduino or computer.
 ****************************************************************************/
void doPWM()
{
  timeOn = timePeriod * pidOutput / 100; //recalc the millisecs on to get this power level, user may have changed
 
 if (millis() - lastTimePeriod > timePeriod){
    lastTimePeriod = millis();
  }
  
 if (millis() - lastTimePeriod < timeOn){
      digitalWrite(heaterPin, HIGH); // turn heater on
  } else {
      digitalWrite(heaterPin, LOW); // turn heater off
 }

}


void calcTempSlope(){                   // units in *C/second
  if (tempUpdateCounter - lastTempUpdateCounter > 0 && (tempUpdateCounter - lastTempUpdateCounter) % 2 == 0)
    {
      lastTempUpdateCounter = tempUpdateCounter;
      tempSlope = (t1 - lastTemp) * 1000 / 2000;  // 2 seconds per cycle
      lastTemp = t1;
    }
}


/****************************************************************************
 * check if serial input is waiting if so add it to inString.  
 * Instructions are terminated with \n \r or 'z' 
 * If this is the end of input line then call doInputCommand to act on it.
 ****************************************************************************/
void getSerialInput()
{
  //check if data is coming in if so deal with it
  if (Serial.available() > 0) {

    // read the incoming data as a char:
    char inChar = Serial.read();
    // if it's a newline or return or z, print the string:
    if ((inChar == '\n') || (inChar == '\r') || (inChar == 'z')) {

      //do whatever is commanded by the input string
      if (inString.length() > 0) doInputCommand();
      inString = "";        //reset for next line of input
    } 
    else {
      // if we are not at the end of the string, append the incoming character
      if (inString.length() < maxLength) {
                inString += inChar; 

      }
      else {
        // empty the string and set it equal to the incoming char:
      //  inString = inChar;
           inString = "";
           inString += inChar;
      }
    }
  }
}


/****************************************************************************
 * Called when an input string is received from computer
 * designed for key=value pairs or simple text commands. 
 * Performs commands and splits key and value 
 * and if key is defined sets value otherwise ignores
 ****************************************************************************/
void doInputCommand()
{
  float v = -1;
  inString.toLowerCase();
  int indx = inString.indexOf('=');

  if (indx < 0){  //this is a message not a key value pair

    /*****************************************************
    if (inString.equals("pccontrol")) {
      controlBy = computer;      
    } 
    if (inString.equals("arduinocontrol")) {
      controlBy = arduino;        
    }
     *****************************************************/
    if (inString.equals("load")) {
      pidOn = 1;
      RoastPhase = 1;
    }
    if (inString.equals("eject")) {
      pidOn = 0;
      RoastPhase = 0;
    }

  } 
  else {  //this is a key value pair for decoding
    String key = inString.substring(0, indx);
    String value = inString.substring(indx+1, inString.length());

    //parse string value and return float v
    char buf[value.length()+1];
    value.toCharArray(buf,value.length()+1);
    v = atof (buf); 

    
    //only set value if we have a valid positive number - atof will return 0.0 if invalid
    if (v >= 0)
    {
      if (key.equals("power") && v < 101){  
        
        pidOutput = v;//convert v to integer for power 
        controlBy = computer;  // change mode to "manual control" when set power value from roasterLogger
      } 
     /*************************************************************
      else
      if (key.equals("sett2")){ // set PID setpoint to T2 value
        pidSetpoint = v;
        
      } else
      if (key.equals("pidbias")) { // set pid bias default is 0%
        pidBias = v;
        myPID.Reset();// this resets the PID deleting any accumulated error
      } else
      if (key.equals("pidp")) { 
        P = v;
      } else
      if (key.equals("pidi")) { 
        I = v;
      } else
      if (key.equals("pidd")) { 
        D = v;
      }
      ***************************************************************/
    }
     
  }
}


/****************************************************************************
 * Send data to computer once every second.  Data such as temperatures, etc.
 * This allows current settings to be checked by the controlling program
 * and changed if, and only if, necessary.
 * This is quicker that resending data from the controller each second
 * and the Arduino having to read and interpret the results.
 ****************************************************************************/
void doSerialOutput() {

  //send data to pc
  BT = t1; // get from do250msLoop()
  Serial.print("t1=");
  Serial.println(BT, DP);
  Serial.print("power%=");
  Serial.println(pidOutput, 0);
}

/****************************************************************************
 * Update the values on the LCD
 ****************************************************************************/
void outputLCD() {
  BT = t1; // get from do250msLoop()
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("BT: ");
  lcd.print(BT, DP);
  lcd.print("*C");
  lcd.setCursor(12, 0);
  lcd.print("RP:"); // RoastPhase
  lcd.print(RoastPhase);
  lcd.setCursor(0, 1);
  lcd.print("Heating: ");
  lcd.print(pidOutput, DP);
  lcd.print(" %");
}

/****************************************************************************
 * Called by main loop once every 250 ms
 * Used to read each thermocouple once every 250 ms
 *
 * Once per second averages temperature results, updates potentiometer and outputs data
 * to serial port.
 ****************************************************************************/
void do250msLoop()
{
  getTemperatures();

  if (tcLoopCount > 3)  // once every four loops (1 second) calculate average temp, update Pot and do serial output
  {
    tcLoopCount = 0;

    if (noGoodReadings1 > 0)
    {
      t1 = tCumulative1 / noGoodReadings1;
    }
    else
    {
      t1 = -1.0;
    }
    noGoodReadings1 = 0;
    tCumulative1 = 0.0;
    doSerialOutput(); // once per second
    outputLCD(); // once per second
    tempUpdateCounter ++;
  }
  tcLoopCount ++;
}

/*****************************************************************
 * Initializing Arduino
 *****************************************************************/

void setup()
{
  // start serial port at 9600 baud:
  Serial.begin(115200);    

  //set up pin modes for Max6675
  pinMode(CS1, OUTPUT);
  pinMode(SO, INPUT);
  pinMode(SCKa, OUTPUT);
  // deselect both Max6675's
  digitalWrite(CS1, HIGH);

  // Light up LCD
  lcd.init();    // initialize the lcd 
  lcd.backlight();
  lcd.setCursor(1, 0);
  lcd.print("Popper Roaster");
  lcd.setCursor(4, 1);
  lcd.print("ver.0820");
  delay(3000);
  
  //turn PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 100);
  myPID.SetSampleTime(2000);  // pid re-evaluate every 2000ms

  myPID2.SetMode(AUTOMATIC);
  myPID2.SetOutputLimits(0, 100);
  myPID2.SetSampleTime(2000);  // pid re-evaluate every 2000ms
  
  // setup HeaterPin
  setupHeater();
  
  startTime = millis();    // start counting roasting time in miliseconds
  lastTimePeriod = startTime;  // counting PWM period in minutes
}

/****************************************************************************
 * Main loop must not use delay!  PWM heater control relies on loop running
 * at least every 40 ms.  If it takes longer then heater will be on slightly
 * longer than planned. Not a big problem if 1% becomes 1.2%! But keep loop fast.
 * Currently loop takes about 4-5 ms to run so no problem.
 ****************************************************************************/
void loop(){
  
    getSerialInput();// check if any serial data waiting
  
    // loop to run four lastTCTimerLoops every second to read thermocouple (TC) temperature and update PWM etc.
    if (millis() - lastTCTimerLoop >= 250)
    {
      lastTCTimerLoop = millis();
      do250msLoop();
    }

    if (t1 > dryTemp && t1 <= dropTemp) RoastPhase = 2;
    calcTempSlope();
    
    if (pidOn == 1) {
  
      if (RoastPhase == 1)
      {
        Input = tempSlope;
        Setpoint = DryTempSlope;
        myPID.Compute();
        doPWM();
        controlBy = arduino;
      }
  
      if (RoastPhase == 2){
       if (controlBy == arduino) {
        Input = t1;
        Setpoint = dropTemp;
        myPID2.Compute();
        doPWM();
        }
        if (controlBy == computer){ // manual control heat power
          doPWM();
          }
      }
      
      /***************************************
        if (t1 > dropTemp){
          //pidOn = 0;
          digitalWrite(heaterPin, LOW);    // turn off heater
          pidOutput = 0;
      }
       ***************************************/
    }
    
    if (pidOn == 0) {
      digitalWrite(heaterPin, LOW);
      pidOutput = 0;
  }
}
