#include <Wire.h>

#define MPU9250_ADDR 0x68
#define AK8963_ADDR 0x0C

#define AK8963_CNTRL 0x0A
#define AK8963_FUSE_ROM 0x0F
#define AK8963_ASA 0x10
#define AK8963_HXL 0x03
#define EXT_SENS_DATA_00 0x49

#define I2C_SLV0_ADDR 0x25
#define I2C_SLV0_REG 0x26
#define I2C_SLV0_CTRL 0x27
#define I2C_SLV0_DO 0x63

float mag_sensitivity[3];
int16_t mag_x_raw, mag_y_raw, mag_z_raw;

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  setup_sensor();

  Serial.println("SETUP DONE!");
  delay(3000);
}

void loop() {
  uint8_t data[7];
  Wire.beginTransmission(MPU9250_ADDR);                                   //Start communication with the gyro.
  Wire.write(0x49);                                                       //Start reading @ register 43h and auto increment with every read.
  Wire.endTransmission();                                                 //End the transmission.
  Wire.requestFrom(MPU9250_ADDR, 7);                                      //Request 6 bytes from the gyro.

  while (Wire.available() < 7);
  for (int i = 0; i < 7; i++) data[i] = Wire.read();
  mag_x_raw = (int16_t) data[1] << 8 | data[0];
  mag_y_raw = (int16_t) data[3] << 8 | data[2];
  mag_z_raw = (int16_t) data[5] << 8 | data[4];

  mag_x_raw *= mag_sensitivity[0];
  mag_y_raw *= mag_sensitivity[1];
  mag_z_raw *= mag_sensitivity[2];

  Serial.print(mag_x_raw);
  Serial.print(",");
  Serial.print(mag_y_raw);
  Serial.print(",");
  Serial.println(mag_z_raw);

  delay(10);
}

void setup_sensor() {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x6A);                                                          //User Control register
  Wire.write(0x20);                                                          //I2C_MST_EN bit enabled
  Wire.endTransmission();

  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x24);                                                          //I2C_MST_CTRL register
  Wire.write(0x0D);                                                          //00001101 --> 400kHz I2C
  Wire.endTransmission();

  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x6B);                                                          //PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                          //00000000 --> activate the gyro
  Wire.endTransmission();

  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x1B);                                                          //GYRO_CONFIG register (1B hex)
  Wire.write(0x08);                                                          //00001000 --> 500dps full scale
  Wire.endTransmission();

  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x1C);                                                          //ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                                                          //00010000 (+/- 8g full scale range)
  Wire.endTransmission();

  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x1A);                                                          //CONFIG register (1A hex)
  Wire.write(0x03);                                                          // 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();

  //Initialise AK896 Magnetometer
  writeMagRegister(AK8963_CNTRL, 0x00);                                      //Power down AK8963
  delay(100);
  writeMagRegister(AK8963_CNTRL, AK8963_FUSE_ROM);                           //set AK8963 to FUSE ROM access
  delay(100);

  //READ bytes from AK8963 Magnetometer
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(I2C_SLV0_ADDR);                                              //I2C_SLV0_ADDR
  Wire.write(0x8C);                                                       //Set the I2C slave address of AK8963 and set for read.
  Wire.endTransmission();

  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(I2C_SLV0_REG);                                               //I2C_SLV0_REG
  Wire.write(AK8963_ASA);                                                 //I2C slave 0 register address from where to begin data transfer
  Wire.endTransmission();

  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(I2C_SLV0_CTRL);                                              //I2C_SLV0_CTRL
  Wire.write(B10000011);                                                  //Enable I2C and read 3 bytes
  Wire.endTransmission();
  delay(1);

  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(EXT_SENS_DATA_00);                                           //Read the x, y,and z-axis calibration values
  Wire.endTransmission();
  Wire.requestFrom(MPU9250_ADDR, 3);

  uint8_t info[3];
  while (Wire.available() < 3);
  for (int i = 0; i < 3; i++) info[i] = Wire.read();
  mag_sensitivity[0] =  (float) (info[0] - 128) / 256.0f + 1.0f;             // Return x-axis sensitivity adjustment values, etc.
  mag_sensitivity[1] =  (float) (info[1] - 128) / 256.0f + 1.0f;
  mag_sensitivity[2] =  (float) (info[2] - 128) / 256.0f + 1.0f;

  writeMagRegister(AK8963_CNTRL, 0x00);                                      //Power down AK8963
  delay(100);
  writeMagRegister(AK8963_CNTRL, B00010110);                                 //16 bit resolution, Continuous mode 2
  delay(100);

  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(I2C_SLV0_ADDR);                                              //I2C_SLV0_ADDR
  Wire.write(0x8C);                                                       //Set the I2C slave address of AK8963 and set for read.
  Wire.endTransmission();

  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(I2C_SLV0_REG);                                               //I2C_SLV0_REG
  Wire.write(AK8963_HXL);                                                 //I2C slave 0 register address from where to begin data transfer
  Wire.endTransmission();

  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(I2C_SLV0_CTRL);                                              //I2C_SLV0_CTRL
  Wire.write(B10000111);                                                 //Enable I2C and read 7 bytes
  Wire.endTransmission();
  delay(1);
}

void writeMagRegister(uint8_t subAddress, uint8_t data) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(I2C_SLV0_ADDR);
  Wire.write(AK8963_ADDR);
  Wire.endTransmission();

  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(I2C_SLV0_REG);
  Wire.write(subAddress);
  Wire.endTransmission();

  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(I2C_SLV0_DO);
  Wire.write(data);
  Wire.endTransmission();

  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(I2C_SLV0_CTRL);
  Wire.write(B10000001);
  Wire.endTransmission();
}
