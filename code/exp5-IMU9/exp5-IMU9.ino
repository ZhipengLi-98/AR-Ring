#include <CapacitiveSensor.h>
int ledPin = 13;
CapacitiveSensor cs_4_2 = CapacitiveSensor(4,2);

#include <Wire.h>                               // I2C library, gyroscope
//-------- REGISTER MAP for ADXL345 chip   -----// Gyroscope MPU3050 / ITG3200 / ITG3205 
#define GYRO 0x68                               // when AD0 is connected to GND ,gyro address is 0x68; GY-85
//#define GYRO 0x69                             // when AD0 is connected to VCC ,gyro address is 0x69  
#define PWR_MGM     (0x3E)                      // Register 3E – Power Management Control
                                                //   used to manage the power control, select the clock source, 
                                                //   and to issue a master reset to the device - pg 27
#define SMPLRT_DIV  (0x15)                      // Register 21 – Sample Rate Divider
                                                //   determines the sample rate of the ITG-3205 gyros
#define DLPF_FS     (0x16)                      // Register 22 – DLPF, Full Scale
                                                //   configures several parameters related to the sensor acquisition.
                                                //   The FS_SEL parameter allows setting the full-scale range of the gyro sensors
                                                //   The power-on-reset value of FS_SEL is 00h. Set to 03h for proper operation.
#define INT_CFG     (0x17)                      // Register 23 – Interrupt Configuration
                                                //   configures the interrupt operation of the device.  
#define G_TO_READ   (8)                         // 2 bytes for each axis x, y, z
#define TEMP_OUT_H  (0x1B)                      // 16-bit temperature data (2’s complement format)
                                                // https://github.com/xioTechnologies/Serial-Oscilloscope 
int g_offx = -15;                               // Off-Set are Board specific - Use Serial Oscilloscope program above
int g_offy = -35;                               // do it by trial and error; modify the signal 
int g_offz = -35;                               // and write down until you can make zero all the axes
float hx, hy, hz, turetemp;

//-------- REGISTER MAP for ADXL345 chip   -----// 
#define ACC         (0x53)                      // ADXL345 ACC address
#define POWER_CTL   (0x2D)                      // Power-saving features control 
#define DATA_FORMAT (0x31)                      // Data format control 
#define BW_RATE     (0x2C)                      // Data rate and power mode control 
#define DATAX0      (0x32)                      // First address for axis x

#define Magnetometer_mX0 0x03
#define Magnetometer_mX1 0x04
#define Magnetometer_mZ0 0x05
#define Magnetometer_mZ1 0x06
#define Magnetometer_mY0 0x07
#define Magnetometer_mY1 0x08
int mX0, mX1;
int mY0, mY1;
int mZ0, mZ1;
float mX_out, mY_out, mZ_out;
float heading, headingDegrees, headingFiltered, declination;
float Xm, Ym, Zm;
#define Magnetometer 0x1E //I2C 7bit address of HMC5883

#define A_TO_READ   (6)                         // num of bytes we are going to read each time 
                                                // two bytes for each axis
//---- initializes the gyroscope -  ITG3205 ----//
void initGyro()
{
   writeTo(GYRO, PWR_MGM,    0B00000000);       //  int osc, no rst, no sleep, normal mode
   writeTo(GYRO, SMPLRT_DIV, 0B00000111);        // divider is 7d
   writeTo(GYRO, DLPF_FS,    0B00011110);       // +/-2000 DPS, DLPF=5Hz, Int.sampleRate=1KHz,
   writeTo(GYRO, INT_CFG,    0B00000000);       // logic high, push-pull, no latch, no int
}

//-- Initialization - Turning on the ADXL345 ---//
/*by default the device is in +-2g range reading*/
void initAcc() {
  writeTo(ACC, POWER_CTL,   0B00001000);        // 1 places the part into measurement mode     
  writeTo(ACC, DATA_FORMAT, 0B00001011);        // +-16g range, right-justified, full resolution
  writeTo(ACC, BW_RATE,     0B00101100);        // 1100 -> ODR = 400Hz, 200 bandwidth = ODR/2 
                                                // (ODR = Output Data Rate); Table 7 & 8 DS
                                                // same outputting data at each 10 ms (T = 1/ F)                                              
}

void initMag() {
  Wire.beginTransmission(Magnetometer);
  Wire.write(0x02); // Select mode register
  Wire.write(0x00); // Continuous measurement mode
  Wire.endTransmission();
}

//-------- Returns all data Function   ---------//
void getGyroscopeData(int * result)
{
   int regAddress = TEMP_OUT_H;
   int temp, x, y, z;
   byte buff[G_TO_READ];
   readFrom(GYRO, regAddress, G_TO_READ,
   buff); //read the gyro data from the ITG3200
   result[0] = ((buff[2] << 8) | buff[3]) + g_offx;
   result[1] = ((buff[4] << 8) | buff[5]) + g_offy;
   result[2] = ((buff[6] << 8) | buff[7]) + g_offz;
   result[3] = (buff[0] << 8) | buff[1];        // temperature
 }

 //-------- Returns all data Function   ---------//
void getAccelerometerData(int * result) {

  int regAddress = DATAX0;                      // first axis-acceleration-data register on the ADXL345
  byte buff[A_TO_READ];                         // simultaneous reading as recommended by the data sheet
  readFrom(ACC, regAddress, A_TO_READ, buff);   // read the acceleration data from the ADXL345
                                                // each axis reading comes in 10 bit resolution
                                                // ie 2 bytes. Least Significat Byte first!!
                                                //thus we are converting both bytes in to one int
  result[0] = (((int)buff[1]) << 8) | buff[0];   
  result[1] = (((int)buff[3])<< 8) | buff[2];
  result[2] = (((int)buff[5]) << 8) | buff[4];

}

void getMagData(int * result) {
  
}

//----------------  Set Up   -------------------//
void setup()
{
   Serial.begin(500000);
   Wire.begin();
   initGyro();
   initAcc();
   initMag();
  pinMode(ledPin, OUTPUT);
  cs_4_2.set_CS_AutocaL_Millis(0xFFFFFFFF);
}

void loopGyr() {
   byte addr;
   int gyro[4];
   getGyroscopeData(gyro);
   hx = gyro[0] / 14.375;
   hy = gyro[1] / 14.375;
   hz = gyro[2] / 14.375;
   turetemp = 35+ ((double) (gyro[3] + 13200)) / 280; // temperature
   //Serial.print(" Gx=");
   Serial.print(hx);
   Serial.print(" ");
   //Serial.print(" Gy=");
   Serial.print(hy);
   Serial.print(" ");
   //Serial.print(" Gz=");
   Serial.print(hz);
   Serial.print(" ");
}

void loopAcc() {
  int acc[3];
  getAccelerometerData(acc);
  // hx = acc[0] / 256.0;
  // hy = acc[1] / 256.0;
  // hz = acc[2] / 256.0;
  hx = acc[0];
  hy = acc[1];
  hz = acc[2];
  //Serial.print(" Ax=");
  Serial.print(hx);
  Serial.print(" ");
  //Serial.print(" Ay=");
  Serial.print(hy);
  Serial.print(" ");
  //Serial.print(" Az=");
  Serial.print(hz);
  Serial.print(" ");
}

void loopMag() {
  // — — X-Axis
  Wire.beginTransmission(Magnetometer); // transmit to device
  Wire.write(Magnetometer_mX1);
  Wire.endTransmission();
  Wire.requestFrom(Magnetometer, 1);
  //if (Wire.available() <= 1)
  //{
    mX0 = Wire.read();
  //}
  Wire.beginTransmission(Magnetometer); // transmit to device
  Wire.write(Magnetometer_mX0);
  Wire.endTransmission();
  Wire.requestFrom(Magnetometer, 1);
  //if (Wire.available() <= 1)
  //{
    mX1 = Wire.read();
  //}
  // — — Y-Axis
  Wire.beginTransmission(Magnetometer); // transmit to device
  Wire.write(Magnetometer_mY1);
  Wire.endTransmission();
  Wire.requestFrom(Magnetometer, 1);
  //if (Wire.available() <= 1)
  //{
    mY0 = Wire.read();
  //}
  Wire.beginTransmission(Magnetometer); // transmit to device
  Wire.write(Magnetometer_mY0);
  Wire.endTransmission();
  Wire.requestFrom(Magnetometer, 1);
  //if (Wire.available() <= 1)
  //{
    mY1 = Wire.read();
  //}

  // — — Z-Axis
  Wire.beginTransmission(Magnetometer); // transmit to device
  Wire.write(Magnetometer_mZ1);
  Wire.endTransmission();
  Wire.requestFrom(Magnetometer, 1);
  //if (Wire.available() <= 1)
  //{
    mZ0 = Wire.read();
  //}
  Wire.beginTransmission(Magnetometer); // transmit to device
  Wire.write(Magnetometer_mZ0);
  Wire.endTransmission();
  Wire.requestFrom(Magnetometer, 1);
  //if (Wire.available() <= 1)
  //{
    mZ1 = Wire.read();
  //}

  // — — X-Axis
  mX1 = mX1 << 8;
  mX_out = mX0 + mX1; // Raw data
  // From the datasheet: 0.92 mG/digit
  Xm = mX_out * 0.00092; // Gauss unit
  //* Earth magnetic field ranges from 0.25 to 0.65 Gauss, so these are
  //the values that we need to get approximately.
  // — — Y-Axis
  mY1 = mY1 << 8;
  mY_out = mY0 + mY1;
  Ym = mY_out * 0.00092;
  // — — Z-Axis
  mZ1 = mZ1 << 8;
  mZ_out = mZ0 + mZ1;
  Zm = mZ_out * 0.00092;

  //Serial.print(" Mx=");
  Serial.print(mX_out);
  Serial.print(" ");
  //Serial.print(" My=");
  Serial.print(mY_out);
  Serial.print(" ");
  //Serial.print(" Mz=");
  Serial.print(mZ_out);
  Serial.print(" ");
  
  /*
  // ==============================
  //Calculating Heading
  heading = atan2(Ym, Xm);
  // Correcting the heading with the declination angle depending on your
  // location
  // You can find your declination angle at:
  // http://www.ngdc.noaa.gov/geomag-web/
  // At my location it’s 4.2 degrees => 0.073 rad
  // My city = -12.44 degrees = > -0.19540706305329 rad
  //  http://www.magnetic-declination.com/
  // declination = 0.073;
  declination = -0.195;
  heading += declination;
  // Correcting when signs are reveresed
  if (heading < 0) heading += 2 * PI;
  // Correcting due to the addition of the declination angle
  if (heading > 2 * PI)heading -= 2 * PI;
  headingDegrees = heading * 180 / PI; // The heading in Degrees unit
  // Smoothing the output angle / Low pass filter
  headingFiltered = headingFiltered * 0.85 + headingDegrees * 0.15;
  //Sending the heading value through the Serial Port to Processing IDE
  Serial.print(headingFiltered);
  Serial.print(" ");
  */
}

void loopTouch() {
  long cap =  cs_4_2.capacitiveSensor(5);
  // Serial.print(cap);
  //  Serial.print(" ");
    if (cap > 30) {
    digitalWrite(ledPin, HIGH);
    //Serial.print(" Touch=");
    
    Serial.print(1);
  } else {
    digitalWrite(ledPin, LOW);
    //Serial.print(" Touch=");
    Serial.print(0);
  }
}

//----------------    Loop   -------------------//
void loop()
{
  long t = millis();
  Serial.print(t);
  Serial.print(" ");
  loopGyr();
  loopAcc();
  // loopMag();
  loopTouch();
  Serial.println();
}
//---------------- Functions------------------  //
/*Writes val to address register on ACC*/
void writeTo(int DEVICE, byte address, byte val) {
   Wire.beginTransmission(DEVICE);              //start transmission to ACC 
   Wire.write(address);                         // send register address
   Wire.write(val);                             // send value to write
   Wire.endTransmission();                      //end transmission    
}
/*reads num bytes starting from address register on ACC in to buff array*/
 void readFrom(int DEVICE, byte address, int num, byte buff[]) {
   Wire.beginTransmission(DEVICE);              //start transmission to ACC 
   Wire.write(address);                         //sends address to read from
   Wire.endTransmission();                      //end transmission
 
   // Wire.beginTransmission(DEVICE);           //start transmission to ACC
   // Removed in Sept 14, 2017: Thanks to https://github.com/Koepel
   // Wire.requestFrom() does a complete I2C transaction on its own
   // Wire.beginTransmission() and Wire.endTransmission() are only used when writing data
   Wire.requestFrom(DEVICE, num);               // request 6 bytes from ACC
 
   int i = 0;
   while(Wire.available())                      //ACC may send less than requested (abnormal)
 { 
   buff[i] = Wire.read();                       // receive a byte
   i++;
 }
   // Wire.endTransmission();                   //end transmission. Perfect Koepel! Thank you very much for your comments!
}
