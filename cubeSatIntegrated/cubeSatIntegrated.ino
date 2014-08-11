/***************************************************************************************************************
*  FILE: flatSat.ino
*  AUTHOR: QUAIL (aka Rohan)
*  VERSION: 1.0.3
*
*  KNOWN BUGS:
*   IMU values are not completely calibrated and can achieve wrong values over time
*   IMU values do not compensate for large metalic objects in its field
*    
***************************************************************************************************************/


/*****************************************************************/
/*********** USER SETUP AREA! Set your options here! *************/
/*****************************************************************/

// HARDWARE OPTIONS
/*****************************************************************/
// Select your hardware here by uncommenting one line!
#define HW__VERSION_CODE 10736 // SparkFun "9DOF Razor IMU" version "SEN-10736" (HMC5883L magnetometer)

      //Some of this code was hacked up from another SparkFun IMU board, 
      //which we did not have, but motified the code so it would work with 3 different sensors

// OUTPUT OPTIONS
/*****************************************************************/
#define OUTPUT__BAUD_RATE 9600

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT__DATA_INTERVAL 15  // in milliseconds

// Output mode definitions (do not change)
#define OUTPUT__MODE_CALIBRATE_SENSORS 0 // Outputs sensor min/max values as text for manual calibration
#define OUTPUT__MODE_ANGLES 1 // Outputs yaw/pitch/roll in degrees
#define OUTPUT__MODE_SENSORS_CALIB 2 // Outputs calibrated sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_RAW 3 // Outputs raw (uncalibrated) sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_BOTH 4 // Outputs calibrated AND raw sensor values for all 9 axes
// Output format definitions (do not change)
#define OUTPUT__FORMAT_TEXT 0 // Outputs data as text
#define OUTPUT__FORMAT_BINARY 1 // Outputs data as binary float

// Select your startup output mode and format here!
int output_mode = OUTPUT__MODE_ANGLES;
int output_format = OUTPUT__FORMAT_TEXT;

// Select if serial continuous streaming output is enabled per default on startup.
#define OUTPUT__STARTUP_STREAM_ON true  // true or false

boolean output_errors = true;  // true or false
#define OUTPUT__HAS_RN_BLUETOOTH false  // true or false

// SENSOR CALIBRATION
/*****************************************************************/
#define ACCEL_X_MIN ((float) 0) 
#define ACCEL_X_MAX ((float) 2)
#define ACCEL_Y_MIN ((float) 0)
#define ACCEL_Y_MAX ((float) 2)
#define ACCEL_Z_MIN ((float) 0)
#define ACCEL_Z_MAX ((float) 5)

// Gyroscope
// "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
#define GYRO_AVERAGE_OFFSET_X ((float) -16.22)
#define GYRO_AVERAGE_OFFSET_Y ((float) 40.76)
#define GYRO_AVERAGE_OFFSET_Z ((float) 6.16)

#define CALIBRATION__MAGN_USE_EXTENDED true
const float magn_ellipsoid_center[3] = {170.696, 49.0454, -69.9695};
const float magn_ellipsoid_transform[3][3] = {{0.885395, 0.00993606, 0.0300371}, {0.00993606, 0.883177, 0.0144505}, {0.0300371, 0.0144505, 0.989619}};

// DEBUG OPTIONS
/*****************************************************************/
// When set to true, gyro drift correction will not be applied
#define DEBUG__NO_DRIFT_CORRECTION true
// Print elapsed time after each I/O loop
#define DEBUG__PRINT_LOOP_TIME false

/*****************************************************************/
/****************** END OF USER SETUP AREA FOR IMU!  *********************/
/*****************************************************************/

//The following code is for the sensor payload on the AGS cube sat

#ifndef HW__VERSION_CODE
  // Generate compile error
  #error YOU HAVE TO SELECT THE HARDWARE YOU ARE USING! See "HARDWARE OPTIONS" in "USER SETUP AREA" at top of Razor_AHRS.ino!
#endif

#include <Wire.h>
#include <SFE_TSL2561.h> //Includes the Luminsity Sensor Library
#include <LiquidCrystal.h>   //Includes the LCD screen Library
#include <dht.h>

SFE_TSL2561 light;       //Object for the Luminosity Sesnor

boolean lumRawData = false;  //Boolean to get data0 and data1 for Lum Sensor 
boolean celcius = false;    //Boolean to get Celcius Data
boolean isLcd = true;       //Boolean to get LCD screen
boolean rawAccel = false;    //Boolean to get rawData for accel, used for calibration
                             //or to configure the accel

boolean satRawPrint = true;

int mq3_analogPin = 8;   //hydrogen sensor port
int ch4_analogPin = 9;   //methane sensor port

boolean gain;        //??
unsigned int ms;     //varible for the milliseconds for function calls
float temp;  //stores the temperature value

int SHT_clockPin = 3;  // pin used for clock //for el temp and humid
int SHT_dataPin  = 2;  // pin used for data

double lux;    // Resulting lux value
boolean good;  // True if neither sensor is saturated

dht DHT;

#define DHT21_PIN 5
  
int mq3_value;
int ch4_value;

// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))


// Gain for gyroscope (ITG-3200)
#define GYRO_GAIN 0.06957 // Same gain on all axes
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) // Calculate the scaled gyro readings in radians per second

// DCM parameters
#define Kp_ROLLPITCH 0.02f
#define Ki_ROLLPITCH 0.00002f
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f

// Stuff
#define STATUS_LED_PIN 13  // Pin number of status LED
#define GRAVITY 256.0f // "1G reference" used for DCM filter and accelerometer calibration
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

// Sensor variables
float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float accel_min[3];
float accel_max[3];

float magnetom[3];
float magnetom_min[3];
float magnetom_max[3];
float magnetom_tmp[3];

float gyro[3];
float gyro_average[3];
int gyro_num_samples = 0;

// DCM variables
float MAG_Heading;
float Accel_Vector[3]= {0, 0, 0}; // Store the acceleration in a vector
float Gyro_Vector[3]= {0, 0, 0}; // Store the gyros turn rate in a vector
float Omega_Vector[3]= {0, 0, 0}; // Corrected Gyro_Vector data
float Omega_P[3]= {0, 0, 0}; // Omega Proportional correction
float Omega_I[3]= {0, 0, 0}; // Omega Integrator
float Omega[3]= {0, 0, 0};
float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};
float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Euler angles
float yaw;
float pitch;
float roll;

boolean first1 = true;

// DCM timing in the main loop
unsigned long timestamp;
unsigned long timestamp_old;
float G_Dt; // Integration time for DCM algorithm

// More output-state variables
boolean output_stream_on;
boolean output_single_on;
int curr_calibration_sensor = 0;
boolean reset_calibration_session_flag = true;
int num_accel_errors = 0;
int num_magn_errors = 0;
int num_gyro_errors = 0;

void read_sensors() {
  Read_Gyro(); // Read gyroscope
  Read_Accel(); // Read accelerometer
  Read_Magn(); // Read magnetometer
}

// Read every sensor and record a time stamp
// Init DCM with unfiltered orientation
// TODO re-init global vars?
void reset_sensor_fusion() {
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};

  read_sensors();
  timestamp = millis();
  
  // GET PITCH
  // Using y-z-plane-component/x-component of gravity vector
  pitch = -atan2(accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2]));
	
  // GET ROLL
  // Compensate pitch of gravity vector 
  Vector_Cross_Product(temp1, accel, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);
  // Normally using x-z-plane-component/y-component of compensated gravity vector
  // roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
  // Since we compensated for pitch, x-z-plane-component equals z-component:
  roll = atan2(temp2[1], temp2[2]);
  
  // GET YAW
  Compass_Heading();
  yaw = MAG_Heading;
  
  // Init rotation matrix
  init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
}

// Apply calibration to raw sensor readings
void compensate_sensor_errors() {
    // Compensate accelerometer error
    accel[0] = (accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
    accel[1] = (accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
    accel[2] = (accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;

    // Compensate magnetometer error
#if CALIBRATION__MAGN_USE_EXTENDED == true
    for (int i = 0; i < 3; i++)
      magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
    Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);
#else
    magnetom[0] = (magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
    magnetom[1] = (magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
    magnetom[2] = (magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;
#endif

    // Compensate gyroscope error
    gyro[0] -= GYRO_AVERAGE_OFFSET_X;
    gyro[1] -= GYRO_AVERAGE_OFFSET_Y;
    gyro[2] -= GYRO_AVERAGE_OFFSET_Z;
}

// Reset calibration session if reset_calibration_session_flag is set
void check_reset_calibration_session()
{
  // Raw sensor values have to be read already, but no error compensation applied

  // Reset this calibration session?
  if (!reset_calibration_session_flag) return;
  
  // Reset acc and mag calibration variables
  for (int i = 0; i < 3; i++) {
    accel_min[i] = accel_max[i] = accel[i];
    magnetom_min[i] = magnetom_max[i] = magnetom[i];
  }

  // Reset gyro calibration variables
  gyro_num_samples = 0;  // Reset gyro calibration averaging
  gyro_average[0] = gyro_average[1] = gyro_average[2] = 0.0f;
  
  reset_calibration_session_flag = false;
}

void turn_output_stream_on()
{
  output_stream_on = true;
  digitalWrite(STATUS_LED_PIN, HIGH);
}

void turn_output_stream_off()
{
  output_stream_on = false;
  digitalWrite(STATUS_LED_PIN, LOW);
}

// Blocks until another byte is available on serial port
char readChar()
{
  while (Serial.available() < 1) { } // Block
  return Serial.read();
}

void setup()
{
  
  // Init serial output
  Serial.begin(OUTPUT__BAUD_RATE);
  Serial1.begin(OUTPUT__BAUD_RATE);
  Serial2.begin(OUTPUT__BAUD_RATE);
  
  // Init status LED
  pinMode (STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  // Init sensors
  delay(50);  // Give sensors enough time to start
  I2C_Init();
  Accel_Init();
  Magn_Init();
  Gyro_Init();
  
  // Read sensors, init DCM algorithm
  delay(20);  // Give sensors enough time to collect data
  reset_sensor_fusion();

  // Init output
#if (OUTPUT__HAS_RN_BLUETOOTH == true) || (OUTPUT__STARTUP_STREAM_ON == false)
  turn_output_stream_off();
#else
  turn_output_stream_on();
#endif

  light.begin();      //begin the light sensor
   
  unsigned char ID;   //used for a print... kinda useless
  
  if (light.getID(ID))  //kidna useless, use this for debugging
  {
    //SUCCESS
  }
  else
  {
    byte error = light.getError();
    printError(error);
  }
  
  gain = 0;
  unsigned char time = 2;
  light.setTiming(gain,time,ms);
  light.setPowerUp();
}

unsigned long count = 0;
unsigned long sinceLast = 0;

// Main loop
void loop()
{
count = millis() - sinceLast;  
if(count >= 800)
{
  sinceLast = millis();
    //following is lum code
  unsigned int data0, data1;        
  
  if (light.getData(data0,data1))
  { 
    if(lumRawData) //if true, then we print out raw values of lum sensor
    {   
      //PRINT LUM
      if(satRawPrint)
      {
      Serial1.print("data0: "); 
      Serial1.print(data0);
      Serial1.print(" data1: ");
      Serial1.print(data1);
      
      Serial2.print("data0: "); 
      Serial2.print(data0);
      Serial2.print(" data1: ");
      Serial2.print(data1);
      }
    }
    boolean good;  // True if neither sensor is saturated
  
    good = light.getLux(gain,ms,data0,data1,lux); 

    Serial1.print(lux);
    Serial1.print("|");
    
    Serial2.print(lux);
    Serial2.print("|");
  }
  else
  {
    //runs this when there is an I2C connection error
    byte error = light.getError();
    printError(error);
  }
  //END OF LUM CODE
  
  //following is the gas sensor code
  
  //hydrogen gas sensor
  mq3_value = analogRead(mq3_analogPin);  //reads data from anaglog pin

  Serial1.print(mq3_value);
  Serial1.print("|");
  
  Serial2.print(mq3_value);
  Serial2.print("|");
  
  //methane gas sensor
  ch4_value = analogRead(ch4_analogPin);  //reads data from analog pin
  
  Serial1.print(ch4_value);
  Serial1.print("|");
  
  Serial2.print(ch4_value);
  Serial2.print("|");

  int chk;
  chk = DHT.read21(DHT21_PIN);
  switch (chk)
  {
    case DHTLIB_OK:  
		//Serial.print("OK,\t"); 
		break;
    case DHTLIB_ERROR_CHECKSUM: 
		Serial.print("Checksum error,\t"); 
		break;
    case DHTLIB_ERROR_TIMEOUT: 
		Serial.print("Time out error,\t"); 
		break;
    default: 
		Serial.print("Unknown error,\t"); 
		break;
  }
    // DISPLAY DATA
    Serial1.print(DHT.humidity, 1);
    Serial1.print("|");
    Serial1.println(DHT.temperature, 1);
    
    Serial2.print(DHT.humidity, 1);
    Serial2.print("|");
    Serial2.println(DHT.temperature - 2, 1);
}

    //END OF TEMP AND HUMIDITY

  // Read incoming control messages
  if (Serial.available() >= 2)
  {
    if (Serial.read() == '#') // Start of new control message
    {
      int command = Serial.read(); // Commands
      if (command == 'f') // request one output _f_rame
        output_single_on = true;
      else if (command == 's') // _s_ynch request
      {
        // Read ID
        byte id[2];
        id[0] = readChar();
        id[1] = readChar();
        
        // Reply with synch message
        Serial.print("#SYNCH");
        Serial.write(id, 2);
        Serial.println();
      }
      else if (command == 'o') // Set _o_utput mode
      {
        char output_param = readChar();
        if (output_param == 'n')  // Calibrate _n_ext sensor
        {
          curr_calibration_sensor = (curr_calibration_sensor + 1) % 3;
          reset_calibration_session_flag = true;
        }
        else if (output_param == 't') // Output angles as _t_ext
        {
          output_mode = OUTPUT__MODE_ANGLES;
          output_format = OUTPUT__FORMAT_TEXT;
        }
        else if (output_param == 'b') // Output angles in _b_inary format
        {
          output_mode = OUTPUT__MODE_ANGLES;
          output_format = OUTPUT__FORMAT_BINARY;
        }
        else if (output_param == 'c') // Go to _c_alibration mode
        {
          output_mode = OUTPUT__MODE_CALIBRATE_SENSORS;
          reset_calibration_session_flag = true;
        }
        else if (output_param == 's') // Output _s_ensor values
        {
          char values_param = readChar();
          char format_param = readChar();
          if (values_param == 'r')  // Output _r_aw sensor values
            output_mode = OUTPUT__MODE_SENSORS_RAW;
          else if (values_param == 'c')  // Output _c_alibrated sensor values
            output_mode = OUTPUT__MODE_SENSORS_CALIB;
          else if (values_param == 'b')  // Output _b_oth sensor values (raw and calibrated)
            output_mode = OUTPUT__MODE_SENSORS_BOTH;

          if (format_param == 't') // Output values as _t_text
            output_format = OUTPUT__FORMAT_TEXT;
          else if (format_param == 'b') // Output values in _b_inary format
            output_format = OUTPUT__FORMAT_BINARY;
        }
        else if (output_param == '0') // Disable continuous streaming output
        {
          turn_output_stream_off();
          reset_calibration_session_flag = true;
        }
        else if (output_param == '1') // Enable continuous streaming output
        {
          reset_calibration_session_flag = true;
          turn_output_stream_on();
        }
        else if (output_param == 'e') // _e_rror output settings
        {
          char error_param = readChar();
          if (error_param == '0') output_errors = false;
          else if (error_param == '1') output_errors = true;
          else if (error_param == 'c') // get error count
          {
            Serial.print("#AMG-ERR:");
            Serial.print(num_accel_errors); Serial.print(",");
            Serial.print(num_magn_errors); Serial.print(",");
            Serial.println(num_gyro_errors);
          }
        }
      }
#if OUTPUT__HAS_RN_BLUETOOTH == true
      // Read messages from bluetooth module
      // For this to work, the connect/disconnect message prefix of the module has to be set to "#".
      else if (command == 'C') // Bluetooth "#CONNECT" message (does the same as "#o1")
        turn_output_stream_on();
      else if (command == 'D') // Bluetooth "#DISCONNECT" message (does the same as "#o0")
        turn_output_stream_off();
#endif // OUTPUT__HAS_RN_BLUETOOTH == true
    }
    else
    { } // Skip character
  }

  // Time to read the sensors again?
  if((millis() - timestamp) >= OUTPUT__DATA_INTERVAL)
  {
    timestamp_old = timestamp;
    timestamp = millis();
    if (timestamp > timestamp_old)
      G_Dt = (float) (timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else G_Dt = 0;

    // Update sensor readings
    read_sensors();

    if (output_mode == OUTPUT__MODE_CALIBRATE_SENSORS)  // We're in calibration mode
    {
      check_reset_calibration_session();  // Check if this session needs a reset
      if (output_stream_on || output_single_on) output_calibration(curr_calibration_sensor);
    }
    else if (output_mode == OUTPUT__MODE_ANGLES)  // Output angles
    {
      // Apply sensor calibration
      compensate_sensor_errors();
    
      // Run DCM algorithm
      Compass_Heading(); // Calculate magnetic heading
      Matrix_update();
      Normalize();
      Drift_correction();
      Euler_angles();
      
      if (output_stream_on || output_single_on) output_angles();
    }
    else  // Output sensor values
    {      
      if (output_stream_on || output_single_on) output_sensors();
    }
    
    output_single_on = false;
    
#if DEBUG__PRINT_LOOP_TIME == true
    Serial.print("loop time (ms) = ");
    Serial.println(millis() - timestamp);
#endif
  }
#if DEBUG__PRINT_LOOP_TIME == true
  else
  {
    Serial.println("waiting...");
  }
#endif
}


void printError(byte error) //LUM FUNC
{
  if(satRawPrint)
  {
  Serial.print("I2C error: ");
  Serial.print(error,DEC);
  Serial.print(", ");
  }
  
  switch(error)
  {
    case 0:
    if(satRawPrint)
    {
      Serial.println("success");
    }
      break;
    case 1:
    if(satRawPrint)
    {  
      Serial.println("data too long for transmit buffer");
    }
      break;
    case 2:
    if(satRawPrint)
    {  
      Serial.println("received NACK on address (disconnected?)");
    }
      break;
    case 3:
    if(satRawPrint)
    {  
      Serial.println("received NACK on data");
    }
      break;
    case 4:
    if(satRawPrint)
    {
      Serial.println("other error");
    }  
      break;
    default:
    if(satRawPrint)
    {  
      Serial.println("unknown error");
    }
  }
}

void itgWrite(char address, char registerAddress, char data) //GYRO FUNC
{
  Wire.beginTransmission(address);  //Initiate a communication sequence with the desired i2c device
  Wire.write(registerAddress);  //Tell the I2C address which register we are writing to
  Wire.write(data);  //Send the value to write to the specified register
  Wire.endTransmission();  //End the communication sequence
}

unsigned char itgRead(char address, char registerAddress) //GYRO FUNC
{
  //variable for the contents read from the i2c device.
  unsigned char data=0;
  //Send the register address to be read.
  Wire.beginTransmission(address);
  //Send the Register Address
  Wire.write(registerAddress);
  //End the communication sequence
  Wire.endTransmission();

  //Ask the I2C device for data
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 1);
  
  //Waits for a response from the I2C device
  if(Wire.available()){
    //Save the data sent from the I2C device
    data = Wire.read();
  }
  
  //End the communication sequence.
  Wire.endTransmission();
  
  //Return the data read during the operation
  return data;
}

float getTemperature(){
  //Return Temperature in Celsius
  SHT_sendCommand(B00000011, SHT_dataPin, SHT_clockPin);
  SHT_waitForResult(SHT_dataPin);

  int val = SHT_getData(SHT_dataPin, SHT_clockPin);
  SHT_skipCrc(SHT_dataPin, SHT_clockPin);
  return (float)val * 0.01 - 40; //convert to celsius
}

float getHumidity(){
  //Return  Relative Humidity
  SHT_sendCommand(B00000101, SHT_dataPin, SHT_clockPin);
  SHT_waitForResult(SHT_dataPin);
  int val = SHT_getData(SHT_dataPin, SHT_clockPin);
  SHT_skipCrc(SHT_dataPin, SHT_clockPin);
  return -4.0 + 0.0405 * val + -0.0000028 * val * val; 
}


void SHT_sendCommand(int command, int dataPin, int clockPin){
  // send a command to the SHTx sensor
  // transmission start
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, HIGH);
  digitalWrite(dataPin, LOW);
  digitalWrite(clockPin, LOW);
  digitalWrite(clockPin, HIGH);
  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, LOW);

  // shift out the command (the 3 MSB are address and must be 000, the last 5 bits are the command)
  shiftOut(dataPin, clockPin, MSBFIRST, command);

  // verify we get the right ACK
  digitalWrite(clockPin, HIGH);
  pinMode(dataPin, INPUT);

  if (digitalRead(dataPin)) Serial.println("ACK error 0");
  digitalWrite(clockPin, LOW);
  if (!digitalRead(dataPin)) Serial.println("ACK error 1");
}


void SHT_waitForResult(int dataPin){
  // wait for the SHTx answer
  pinMode(dataPin, INPUT);

  int ack; //acknowledgement

  //need to wait up to 2 seconds for the value
  for (int i = 0; i < 1000; ++i){
    delay(2);
    ack = digitalRead(dataPin);
    if (ack == LOW) break;
  }

  if (ack == HIGH) Serial.println("ACK error 2");
}

int SHT_getData(int dataPin, int clockPin){
  // get data from the SHTx sensor

  // get the MSB (most significant bits)
  pinMode(dataPin, INPUT);
  pinMode(clockPin, OUTPUT);
  byte MSB = shiftIn(dataPin, clockPin, MSBFIRST);

  // send the required ACK
  pinMode(dataPin, OUTPUT);
  digitalWrite(dataPin, HIGH);
  digitalWrite(dataPin, LOW);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);

  // get the LSB (less significant bits)
  pinMode(dataPin, INPUT);
  byte LSB = shiftIn(dataPin, clockPin, MSBFIRST);
  return ((MSB << 8) | LSB); //combine bits
}

void SHT_skipCrc(int dataPin, int clockPin){
  // skip CRC data from the SHTx sensor
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);
}
//END OF FILE

