#include <SoftwareSerial.h>
#include <Wire.h>
#include <TinyGPS.h>
#include <HMC5883L.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <rosserial_arduino/Test.h>

//pin settings
#define ledPin 13

#define EN1             4                     // ENABLE pin motor1
#define PWM1            5                     // PWM pin motor1
#define EN2             6                     // ENABLE pin motor2
#define PWM2            7                     // PWM pin motor2 

#define encodePinA1      19                       // encoder1 A pin
#define encodePinB1      18                       // encoder1 B pin
#define encodePinA2      3                       // encoder2 A pin
#define encodePinB2      2                       // encoder2 B pin

#define GPSTX 10
#define GPSRX 11

#define GYROSDA 20
#define GYROSCL 21

#define Gearratio1      12                      //motor1 gear ratio
#define Encoderpulse1   48*4                    //motor1 encoder pulse
#define Gearratio2      12                      //motor2 gear ratio
#define Encoderpulse2   48*4                    //motor2 encoder pulse

#define LOOPTIME        50                      // PID loop time
#define NUMREADINGS     10                      // samples for Amp average
#define LOOPTIMEVEL     10
#define LOOPTIMEGPS     2000                    //GPS refresh time
#define pi 3.141592

int motor1_rpm_cmd = 0;
int motor2_rpm_cmd = 0;

unsigned long lastMilli = 0;                    // loop timing
unsigned long lastMillimonitor = 0;             // loop timing
unsigned long lastMilliPrint = 0;               // loop timing
unsigned long dtMilli = 0;
unsigned long dtMillimonitor = 100;             // Communication period
unsigned long dtMillispeed = 0;
unsigned long lastMillispeed = 0;

float speed_term = 5.0; //RPM
float accel = 2.5;
float speed_req1 = 0;                            // motor1 speed (Set Point)
float speed_profile1 = 0;
float speed_act1 = 0;                            // motor1 speed (actual value)
float speed_act1_rad = 0;
float speed_act1_rpm = 0;
float speed_act1_rps = 0;
float speed_act1_filtered = 0;

float speed_req2 = 0;                            // motor1 speed (Set Point)
float speed_profile2 = 0;
float speed_act2 = 0;                            // motor1 speed (actual value)
float speed_act2_rad = 0;
float speed_act2_rpm = 0;
float speed_act2_rps = 0;
float speed_act2_filtered = 0;

int PWM_val1 = 0;                                // motor1 PWM (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int PWM_val1_prev = 0;
int PWM_val1_desired = 0;
int PWM_val2 = 0;                                // motor2 PWM (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int PWM_val2_prev = 0;
int PWM_val2_desired = 0;

int voltage = 0;                                 // in mV
int current1 = 0;                                // motor 1 current in mA
int current2 = 0;                                // motor 2 current in mA

volatile long count1 = 0;                        // motor 1 rev counter
volatile long count2 = 0;                        // motor 2 rev counter
long inc_enc_pos1 = 0;
long inc_enc_pos2 = 0;
float abs_enc_pos1 = 0;
float abs_enc_pos2 = 0;

int Precount1[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // motor 1 last count
int Precount2[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // motor 2 last count

long newpre1 = 0;
long newpre2 = 0;

float Kp1 = 0.57;                                // motor1 PID proportional control Gain
float Ki1 = 2.77;                                // motor1 PID Integral control Gain
float Kd1 = 0.88;                                // motor1 PID Derivitave control Gain
float Ka1 = 3.25;

float Kp2 = 0.57;                                // motor2 PID proportional control Gain
float Ki2 = 2.77;                                // motor2 PID Derivitave control Gain
float Kd2 = 0.88;                                // motor2 PID Integral control Gain
float Ka2 = 3.25;

float controll_inc = 1;
float controll_inc_i = 0.01;

float pid_i1 = .0;
float pid_d1 = .0;
float pid_a1 = 0;
float pidTerm_err1 = 0;

float pid_i2 = .0;
float pid_d2 = .0;
float pid_a2 = 0;
float pidTerm_err2 = 0;

int lastspeed1 = 0;
int lastspeed2 = 0;

int manual = 0;
float beta = 1.0;
float error1 = 0.0;
float error2 = 0.0;

float last_error1 = 0;
float last_error2 = 0;

float pwm_prev1 = 0;
float pwm_prev2 = 0;
float enc_prev1 = 0;
float enc_prev2 = 0;

String inputString = "";
boolean stringComplete = false;

//for GPS data
SoftwareSerial mySerial(GPSTX, GPSRX); // TX, RX        //GPS functions
TinyGPS gps;
int gpsdata = 0;
float flat, flon;
unsigned long lastMilliGPS = 0;                   // loop timing
unsigned long dtMilliGPS = 0;
uint8_t gps_config_change[63] = {
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xFA, 0x00, 0x01, 0x00, 0x01, 0x00,
  0x10, 0x96,
  //Baud change
  0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08,
  0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x02, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xBF, 0x78,

  //Save config
  0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1D, 0xAB
};
void gpsdump(TinyGPS &gps);


//for GYRO data
HMC5883L compass;
int compassdata = 0;
float headingDegrees = 0.0;

// mode 1=manual 0,=auto
int mode = 0;

//ROS
ros::NodeHandle  nh;
using rosserial_arduino::Test;
std_msgs::String status_msg;
ros::Publisher chatter("motor_status", &status_msg);
void callback(const Test::Request & req, Test::Response & res)
{
  inputString = req.input;
  res.output = req.input;
  stringComplete = true;
}
ros::ServiceServer<Test::Request, Test::Response> server("motor_cmd", &callback);
char message[256] = "";
char tempBuffer[256];

void setup()
{
  analogReference(DEFAULT);                            //Reference 5V (Mega condition)

  Serial.begin(57600);                               //Monitoring & Command serial
  // set the data rate for the GPS port
  mySerial.begin(9600);
  mySerial.begin(9600);                              // Set GPS baudrate 115200 from 9600
  mySerial.write(gps_config_change, sizeof(gps_config_change));
  mySerial.end();
  mySerial.begin(115200);

  pinMode(EN1, OUTPUT);                                //OUTPUT Pinmodes
  pinMode(EN2, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);

  pinMode(encodePinA1, INPUT);                          //Encoder Pinmodes
  pinMode(encodePinB1, INPUT);
  pinMode(encodePinA2, INPUT);
  pinMode(encodePinB2, INPUT);

  digitalWrite(encodePinA1, HIGH);                      // turn on Endocer pin pullup resistor
  digitalWrite(encodePinB1, HIGH);
  digitalWrite(encodePinA2, HIGH);
  digitalWrite(encodePinB2, HIGH);

  attachInterrupt(5, rencoder1, CHANGE);               //Encoder pin interrupt setting
  attachInterrupt(4, rencoder1, CHANGE);
  attachInterrupt(3, rencoder2, CHANGE);
  attachInterrupt(2, rencoder2, CHANGE);

  analogWrite(PWM1, PWM_val1);
  analogWrite(PWM2, PWM_val2);
  digitalWrite(EN1, LOW);
  digitalWrite(EN2, LOW);

  if (compass.begin())
  {
    compassdata = 1;
    compass.setRange(HMC5883L_RANGE_1_3GA);   // Set measurement range
    compass.setMeasurementMode(HMC5883L_CONTINOUS);   // Set measurement mode
    compass.setDataRate(HMC5883L_DATARATE_30HZ);    // Set data rate
    compass.setSamples(HMC5883L_SAMPLES_8);    // Set number of samples averaged
    compass.setOffset(0, 0);    // Set calibration offset. See HMC5883L_calibration.ino
  }
  else
    compassdata = 0;

  nh.initNode();
  nh.advertise(chatter);
  nh.advertiseService(server);
}

// LPF filter setup
float LPFVAR = 0.08;
float lpFilter(float value, float prev_value, float beta)
{
  float lpf = 0;
  lpf = (beta * value) + (1 - beta) * prev_value;
  return lpf;
}

// Kalman filter setup
float Pk = 1.0;
float varP = pow(0.01, 2);  // pow(0.01, 2)
float R = pow(0.5, 2);
float Kk = 1.0;
float Xk = 20.0;
float kalmanFilter(float value)
{
  Pk = Pk + varP;
  Kk = Pk / (Pk + R);
  Xk = (Kk * value) + (1 - Kk) * Xk;
  Pk = (1 - Kk) * Pk;
  return Xk;
}
int term = 0;

void loop()
{
  digitalWrite(ledPin, LOW);
  Vector norm = compass.readNormalize();
  bool newdata = false;
  checkEvent();

  dtMilli = millis() - lastMilli;                                             // calculate dt
  dtMillispeed = millis() - lastMillispeed;
  dtMilliGPS = millis() - lastMilliGPS;

  // check command
  if (stringComplete)
  {
    Serial.println(inputString);
    calculateRPM(inputString);
    inputString = "";
    stringComplete = false;
    if ( motor1_rpm_cmd > 320)
      speed_req1 = 320;
    else if ( motor1_rpm_cmd < -320)
      speed_req1 = -320;
    else
      speed_req1 = motor1_rpm_cmd;

    if ( motor2_rpm_cmd > 320)
      speed_req2 = 320;
    else if ( motor2_rpm_cmd < -320)
      speed_req2 = -320;
    else
      speed_req2 = motor2_rpm_cmd;
  }

  //////////////////////////////////////////////////////// MOTOR CONTROLL

  if (dtMillispeed >= LOOPTIMEVEL)
  {

    speedcalculation(dtMillispeed);                                                           // calculate speed, volts and Amps

    speed_act1_filtered = lpFilter(speed_act1_rpm, enc_prev1, LPFVAR);
    enc_prev1 = speed_act1_filtered;
    newpre1 = count1;

    speed_act2_filtered = lpFilter(speed_act2_rpm, enc_prev2, LPFVAR);
    enc_prev2 = speed_act2_filtered;
    newpre2 = count2;

    lastMillispeed = millis();
  }

  if (dtMilli >= LOOPTIME)                                                    // time loop check, soft real time
  {

    buildVelProfile(speed_req1, speed_act1_filtered, &speed_profile1);
    buildVelProfile(speed_req2, speed_act2_filtered, &speed_profile2);

    PWM_val1 = updatePid1(speed_profile1, speed_act1_filtered, dtMilli, Kp1, Ki1, Kd1);
    PWM_val2 = updatePid2(speed_profile2, speed_act2_filtered, dtMilli, Kp2, Ki2, Kd2);               // Motor 2 PWM calculation


    if (PWM_val1 > 0)
    {
      digitalWrite(EN1, HIGH);
    }
    else
    {
      digitalWrite(EN1, LOW);
    }

    if (PWM_val2 < 0)
    {
      digitalWrite(EN2, HIGH);
    }
    else
    {
      digitalWrite(EN2, LOW);
    }


    analogWrite(PWM1, abs(PWM_val1));                                               // send PWM to motor
    analogWrite(PWM2, abs(PWM_val2));                                               // send PWM to motor


    lastMilli = millis();
  }


  // Every 2 seconds update GPS
  if (dtMilliGPS >= LOOPTIMEGPS)
  {
    if (mySerial.available())
    {
      char c = mySerial.read();
      //Serial.print(c);  // uncomment to see raw GPS data
      if (gps.encode(c))
      {
        newdata = true;
        gpsdata = 1;
        // break;  // uncomment to print new data immediately!
      }
    }
    lastMilliGPS = millis();
  }
  if (newdata)
  {
    gpsdump(gps);
  }
  else
    gpsdata = 0;

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);
  float declinationAngle = (8.0 + (21.0 / 60.0)) / (180 / M_PI);  
                          // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  heading += declinationAngle;
  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }
  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }
  headingDegrees = heading * 180 / M_PI; // Convert to degrees


  // Publish motor status
  publishStatus();
  nh.spinOnce();
}

void publishStatus()
{
  if ((millis() - lastMilliPrint) >= dtMillimonitor)
  {
    lastMilliPrint = millis();

    dtostrf(speed_act1_filtered , 4, 0, tempBuffer);
    strcat(message, tempBuffer); strcat(message, ",");
    dtostrf(speed_act2_filtered , 4, 0, tempBuffer);
    strcat(message, tempBuffer); strcat(message, ",");
    dtostrf(count1 , 10, 0, tempBuffer);
    strcat(message, tempBuffer); strcat(message, ",");
    dtostrf(count2 , 10, 0, tempBuffer);
    strcat(message, tempBuffer); strcat(message, ",");

    dtostrf(gpsdata , 1, 0, tempBuffer);
    strcat(message, tempBuffer); strcat(message, ",");
    dtostrf(flat , 5, 5, tempBuffer);
    strcat(message, tempBuffer); strcat(message, ",");
    dtostrf(flon , 5, 5, tempBuffer);
    strcat(message, tempBuffer); strcat(message, ",");
    dtostrf(headingDegrees , 5, 2, tempBuffer);
    strcat(message, tempBuffer); strcat(message, ",");

    status_msg.data = message;
    chatter.publish( &status_msg );
    message[0] = '\0';
  }
}

void speedcalculation(unsigned long dt)  // calculate speed, volts and Amps
{
  //speed_act1 = (((float)count1 - (float)newpre1) * 1000) / dt / (Encoderpulse1)  / Gearratio1 *3.141592*2;          // ((count difference) / (dt) / 1000) / (pulses * gear ratio) * 2* 3.141592 = rad/sec
  //speed_act2 = (((float)count2 - (float)newpre2) * 1000) / dt / (Encoderpulse2)  / Gearratio2 *3.141592*2;          // ((count difference) / (dt) / 1000) / (pulses * gear ratio) * 2* 3.141592 = rad/sec

  speed_act1_rps = (((float)count1 - (float)newpre1) * 1000) / dt / (Encoderpulse1)  / Gearratio1 ;
  speed_act1_rpm = (((float)count1 - (float)newpre1) * 1000) / dt / (Encoderpulse1)  / Gearratio1 * 60;
  speed_act1_rad = (((float)count1 - (float)newpre1) * 1000) / dt / (Encoderpulse1)  / Gearratio1 * 3.141592 * 2;
  inc_enc_pos1 = count1 / ((float)Encoderpulse1 * (float)Gearratio1) * 360.0;
  speed_act1 = speed_act1_rpm;

  speed_act2_rps = (((float)count2 - (float)newpre2) * 1000) / dt / (Encoderpulse2)  / Gearratio2 ;
  speed_act2_rpm = (((float)count2 - (float)newpre2) * 1000) / dt / (Encoderpulse2)  / Gearratio2 * 60;
  speed_act2_rad = (((float)count2 - (float)newpre2) * 1000) / dt / (Encoderpulse2)  / Gearratio2 * 3.141592 * 2;
  speed_act2 = speed_act2_rpm;
  inc_enc_pos2 = count2 / ((float)Encoderpulse2 * (float)Gearratio2) * 360.0;
}

void buildVelProfile(float desired_vel, float current_vel, float *speed_profile)
{
  float accel = 5.0;
  float vel_diff = desired_vel - current_vel;
  float time_interval = 100; //ms

  if (abs(desired_vel - *speed_profile) < 10)
    accel = abs(desired_vel - *speed_profile);

  if (desired_vel > *speed_profile)
    *speed_profile = *speed_profile + accel;
  else if (desired_vel < *speed_profile)
    *speed_profile = *speed_profile - accel;
}

float updatePid1( float targetValue, float currentValue, unsigned long dt, float Kp, float Ki, float Kd)    // compute PWM value
{

  float pidTerm = 0;                                                            // PID correction
  float pidTerm_sat = 0;
  //float error=0;                                                                  // error initiallization
  //static float last_error=0;                                                      // For D control, static last_error
  error1 = targetValue - currentValue;                                          // error calculation

  pid_a1 = pidTerm_err1 * Ka1;
  pid_i1 = pid_i1 + (error1 + pid_a1) * dt / 1000;                             // Error Integral
  pid_d1 = (error1 - last_error1) ;

  pidTerm = (Kp * error1) + Ki * pid_i1 + (Kd * (pid_d1));

  last_error1 = error1;                                                   //for D control

  if (pidTerm > 250)
    pidTerm_sat = 250;
  else if (pidTerm < -250)
    pidTerm_sat = -250;
  else
    pidTerm_sat = pidTerm;

  pidTerm_err1 = pidTerm_sat - pidTerm;

  return pidTerm_sat;
}

float updatePid2( float targetValue, float currentValue, unsigned long dt, float Kp, float Ki, float Kd)    // compute PWM value
{

  float pidTerm = 0;                                                            // PID correction
  float pidTerm_sat = 0;
  //float error=0;                                                                  // error initiallization
  //static float last_error=0;                                                      // For D control, static last_error
  error2 = targetValue - currentValue;                                          // error calculation

  pid_a1 = pidTerm_err2 * Ka2;
  pid_i2 = pid_i2 + (error2 + pid_a1) * dt / 1000;                           // Error Integral
  pid_d2 = (error2 - last_error2);

  pidTerm = (Kp * error2) + Ki * pid_i2 + (Kd * (pid_d2)) ;

  last_error2 = error2;                                                          //for D control
  if (pidTerm > 250)
    pidTerm_sat = 250;
  else if (pidTerm < -250)
    pidTerm_sat = -250;
  else
    pidTerm_sat = pidTerm;

  pidTerm_err2 = pidTerm_sat - pidTerm;

  return pidTerm_sat;
}


void calculateRPM(String command)
{
  char charInput[10];
  String motor1_cmd, motor2_cmd;
  String motor1_distance;

  int pos_R, pos_L, pos_E, pos_D;
  sprintf(charInput, "%s", command.c_str());

 if (strchr(charInput, 'p') - charInput == 0)
  {
    if (strchr(charInput, '+') - charInput == 1)
    {
      Kp1 = Kp1 + controll_inc_i;
    }
    else if (strchr(charInput, '-') - charInput == 1)
    {
      Kp1 = Kp1 - controll_inc_i;
    }
  }
  else if (strchr(charInput, 'o') - charInput == 0)
  {
    if (strchr(charInput, '+') - charInput == 1)
    {
      Ki1 += controll_inc_i;
    }
    else if (strchr(charInput, '-') - charInput == 1)
    {
      Ki1 -= controll_inc_i;
    }
  }
  else if (strchr(charInput, 'i') - charInput == 0)
  {
    if (strchr(charInput, '+') - charInput == 1)
    {
      Kd1 += controll_inc_i;
    }
    else if (strchr(charInput, '-') - charInput == 1)
    {
      Kd1 -= controll_inc_i;
    }
  }
  else if (strchr(charInput, 'l') - charInput == 0)
  {
    if (strchr(charInput, '+') - charInput == 1)
    {
      Kp2 += controll_inc_i;
    }
    else if (strchr(charInput, '-') - charInput == 1)
    {
      Kp2 -= controll_inc_i;
    }
  }
  else if (strchr(charInput, 'k') - charInput == 0)
  {
    if (strchr(charInput, '+') - charInput == 1)
    {
      Ki2 += controll_inc_i;
    }
    else if (strchr(charInput, '-') - charInput == 1)
    {
      Ki2 -= controll_inc_i;
    }
  }
  else if (strchr(charInput, 'j') - charInput == 0)
  {
    if (strchr(charInput, '+') - charInput == 1)
    {
      Kd2 += controll_inc_i;
    }
    else if (strchr(charInput, '-') - charInput == 1)
    {
      Kd2 -= controll_inc_i;
    }
  }
  else if (strchr(charInput, 'u') - charInput == 0)
  {
    if (strchr(charInput, '+') - charInput == 1)
    {
      Ka1 += controll_inc_i;
    }
    else if (strchr(charInput, '-') - charInput == 1)
    {
      Ka1 -= controll_inc_i;
    }
  }
  else if (strchr(charInput, 'h') - charInput == 0)
  {
    if (strchr(charInput, '+') - charInput == 1)
    {
      Ka2 += controll_inc_i;
    }
    else if (strchr(charInput, '-') - charInput == 1)
    {
      Ka2 -= controll_inc_i;
    }
  }
  
  else {
    pos_R = (strchr(charInput, 'R') - charInput);
    pos_L = (strchr(charInput, 'L') - charInput);
    pos_E = (strchr(charInput, 'E') - charInput);
    for (int i = pos_R + 1; i < pos_L; i++)
    {
      motor1_cmd += charInput[i];
    }
    for (int i = pos_L + 1; i < pos_E; i++)
    {
      motor2_cmd += charInput[i];
    }
    motor1_rpm_cmd = atoi(motor1_cmd.c_str());
    motor2_rpm_cmd = atoi(motor2_cmd.c_str());
  }
}

void checkEvent() {
  digitalWrite(ledPin, HIGH);
  char character;
  String string;
  while (Serial.available()) {
    character = Serial.read();
    delay(10);
    string.concat(character);
  }
  if (string != "") {
    inputString = string;
    stringComplete = true;
    string = "";
  }
}

void gpsdump(TinyGPS &gps)
{
  long lat, lon;
  // float flat, flon;
  unsigned long age, date, time, chars;
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;

  
  gps.f_get_position(&flat, &flon, &age);
 }


//count from quad encoder

void rencoder1()   // pulse and direction, direct port reading to save cycles
{
  static long oldencodePinB1 = 0;

  if (digitalRead(encodePinA1) ^ oldencodePinB1)
  {
    count1++;
  }
  else
  {
    count1--;
  }
  oldencodePinB1 = digitalRead(encodePinB1);
}

void rencoder2()  // pulse and direction, direct port reading to save cycles
{
  static long oldencodePinB2 = 0;

  if (digitalRead(encodePinA2) ^ oldencodePinB2)
  {
    count2++;
  }
  else
  {
    count2--;
  }
  oldencodePinB2 = digitalRead(encodePinB2);
}

