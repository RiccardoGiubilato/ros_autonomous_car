#define USB_USBCON
#include <ros.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle_<ArduinoHardware, 1, 1, 125, 125> nh;
geometry_msgs::Vector3 pub_msg;
ros::Publisher pub("/car/power", &pub_msg);


static int potenza_linear = 0;
static int potenza_angular = 0;

// True if the car is rotating, false if translating
bool rotating = false;

float mapf(const float& x, const float& in_min, const float& in_max, const float& out_min, const float& out_max)
{ return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* =========== */
/* MPU6050 IMU */
/* =========== */

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
//VectorInt16 aa;         // [x, y, z]            accel sensor measurements
//VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
//float euler[3];         // [psi, theta, phi]    Euler angle container
static float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



/* ============== */
/* PIN DEFINITION */
/* ============== */
int encoderPin1 = 2;
int encoderPin2 = 3;

const int triggerPort = 12;
const int echoPort = 8;
const int led = 13;

static int mA_left = 4; //pin digitale per determinare gli stati logici da inviare al modulo
static int mB_left = 5; //pin digitale per determinare gli stati logici da inviare al modulo
static int mA_right = 6; //pin digitale per determinare gli stati logici da inviare al modulo
static int mB_right = 7; //pin digitale per determinare gli stati logici da inviare al modulo
static int power_left  = 9;
static int power_right = 10;

/* ========== */
/* PARAMETERS */
/* ========== */

bool verb = false;
unsigned long cycle_time = 0;

/* ENCODER */
volatile int lastEncoded = 0;
volatile long encoderValue = 0;
 
long lastencoderValue = 0;
 
int lastMSB = 0;
int lastLSB = 0;

/* SONAR */
static long durata, distanza;

/* Timeout for the sonar range measurements in microseconds. 
 *  1 meter max range = 2 * 3.3 ms = 6600 microseconds
 *  speed of sound: 300 m/s
 *  path length for 1m = 2m (return signal)
 */
const int sonar_timeout = 5000;

void vel_callback(const geometry_msgs::Vector3& msg) {

  // Limited power for rotations!!
  potenza_linear = mapf(abs(msg.x), 0, 1, 80, 255);
  potenza_angular = mapf(abs(msg.z), 0, 1, 80, 150);

  if(msg.z < 0) {
    
    rotating = true;
    digitalWrite(mA_left, LOW);
    digitalWrite(mB_left, HIGH);
    digitalWrite(mA_right, LOW);
    digitalWrite(mB_right, HIGH);

    analogWrite(power_left,  potenza_angular);
    analogWrite(power_right, potenza_angular);
  }

  if(msg.z > 0) {
    
    rotating = true;
    digitalWrite(mA_left, HIGH);
    digitalWrite(mB_left, LOW);
    digitalWrite(mA_right, HIGH);
    digitalWrite(mB_right, LOW);

    analogWrite(power_left,  potenza_angular);
    analogWrite(power_right, potenza_angular);
  }

  if(msg.x < 0) {

    rotating = false;
    digitalWrite(mA_left, HIGH);
    digitalWrite(mB_left, LOW);
    digitalWrite(mA_right, LOW);
    digitalWrite(mB_right, HIGH);

    analogWrite(power_left,  potenza_linear);
    analogWrite(power_right, potenza_linear);
  }

  if(msg.x > 0) {

    rotating = false;
    digitalWrite(mA_left, LOW);
    digitalWrite(mB_left, HIGH);
    digitalWrite(mA_right, HIGH);
    digitalWrite(mB_right, LOW);

    analogWrite(power_left,  potenza_linear);
    analogWrite(power_right, potenza_linear);
  }

  if((msg.x == 0) && (msg.z == 0)) {
    
    rotating = false;
    digitalWrite(mA_left, LOW);
    digitalWrite(mB_left, LOW);
    digitalWrite(mA_right, LOW);
    digitalWrite(mB_right, LOW); 

    analogWrite(power_left,  potenza_linear);
    analogWrite(power_right, potenza_linear);
  }
}
ros::Subscriber<geometry_msgs::Vector3> s("/car/cmd_vel",vel_callback);
 
void setup() {

  nh.initNode();
  nh.subscribe(s);
  nh.advertise(pub);
  
  Serial.begin (57600);

 // MPU6050
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    //Serial.begin(115200);
    // while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    // Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    /* verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
    */

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

  // ENCODER
  pinMode(encoderPin1, INPUT_PULLUP);
  pinMode(encoderPin2, INPUT_PULLUP);
  attachInterrupt(0, updateEncoder, CHANGE);
  attachInterrupt(1, updateEncoder, CHANGE);

  // SONAR
  pinMode(triggerPort, OUTPUT);
  pinMode(echoPort, INPUT);
  pinMode(led, OUTPUT);

  // MOTORS
  pinMode(mA_left, OUTPUT);
  pinMode(mB_left, OUTPUT);
  pinMode(mA_right, OUTPUT);
  pinMode(mB_right, OUTPUT);
  pinMode(power_left, OUTPUT);
  pinMode(power_right, OUTPUT);

  
 
}
 
void loop(){
/*
    digitalWrite(mA_left, LOW);
    digitalWrite(mB_left, HIGH);
    digitalWrite(mA_right, HIGH);
    digitalWrite(mB_right, LOW);

    analogWrite(power_left,  150);
    analogWrite(power_right, 150);*/

  // Starting time
  // cycle_time = millis();

  // ENCODER
  // Serial.print("Rotazione: ");
  // Serial.println(encoderValue); 

  // SONAR
  //porta bassa l'uscita del trigger
  digitalWrite( triggerPort, LOW );
  //invia un impulso di 10microsec su trigger
  digitalWrite( triggerPort, HIGH );
  delayMicroseconds( 10 );
  digitalWrite( triggerPort, LOW );
  
  durata = pulseIn( echoPort, HIGH, sonar_timeout);
  distanza = 0.034 * durata / 2;
  
  //Serial.print("distanza: ");

  if((distanza < 10) && (distanza != 0)){
   digitalWrite(led, HIGH);
  }
  else{
   digitalWrite(led, LOW);
  }

  //MPU
  // if programming failed, don't try to do anything
    if (!dmpReady) return;

    /*
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }
    */

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //Serial.print("ypr\t");
            //Serial.print(ypr[0] * 180/M_PI);
            //Serial.print("\t");
            //Serial.print(ypr[1] * 180/M_PI);
            //Serial.print("\t");
            //Serial.println(ypr[2] * 180/M_PI);
        #endif


     }
  /*   
  if(verb) {   
    Serial.print("\n");
    Serial.print("Encoder: ");
    Serial.print(encoderValue);
    Serial.print("\t");
    Serial.print("Range: ");
    Serial.print(distanza);
    Serial.print("\t");
    Serial.print("Yaw: ");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t"); 
    Serial.print("Cycle time: ");
    Serial.print(millis() - cycle_time);
  }
  */
  pub_msg.x = encoderValue;
  pub_msg.y = ypr[0] * 180/M_PI;
  pub_msg.z = distanza;
  pub.publish(&pub_msg);

  nh.spinOnce();
}

void updateEncoder(){
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit
 
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value
 
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;
 
  lastEncoded = encoded; //store this value for next time
}

