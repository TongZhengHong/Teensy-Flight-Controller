#define AK8963_CNTRL 0x0A
#define AK8963_FUSE_ROM 0x0F
#define AK8963_ASA 0x10
#define AK8963_HXL 0x03
#define EXT_SENS_DATA_00 0x49

#define I2C_SLV0_ADDR 0x25
#define I2C_SLV0_REG 0x26
#define I2C_SLV0_CTRL 0x27
#define I2C_SLV0_DO 0x63

void calibrate_sensors() {
  Serial.print("Calibrating sensor");
  for (int i = 0; i < 2000; i++) {
    if (i % 200 == 0) Serial.print(".");

    Wire.beginTransmission(MPU9250_ADDR);                                   //Start communication with the gyro.
    Wire.write(0x43);                                                       //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                                 //End the transmission.
    Wire.requestFrom(MPU9250_ADDR, 6);                                      //Request 6 bytes from the gyro.

    while (Wire.available() < 6);
    gyro_x_raw = (int16_t) Wire.read() << 8 | (int16_t) Wire.read();
    gyro_y_raw = (int16_t) Wire.read() << 8 | (int16_t) Wire.read();
    gyro_z_raw = (int16_t) Wire.read() << 8 | (int16_t) Wire.read();

    gyro_cal[1] += gyro_x_raw;
    gyro_cal[2] += gyro_y_raw;
    gyro_cal[3] += gyro_z_raw;
  }

  gyro_cal[1] /= 2000;
  gyro_cal[2] /= 2000;
  gyro_cal[3] /= 2000;

  Serial.println();
  Serial.println("Gyroscope calibration done!");
  for (int i = 1; i < 4; i++) Serial.println(gyro_cal[i]);

  Serial.print("Calibrating accelerometer offsets");
  for (int i = 0; i < 1000; i++) {
    if (i % 100 == 0) Serial.print(".");
    calibrate_accel();
    acc_cal_pitch += angle_pitch;
    acc_cal_roll += angle_roll;

    pulse_esc();
    maintain_loop_time();
  }
  acc_cal_pitch /= 1000;
  acc_cal_roll /= 1000;

  Serial.println();
  Serial.println("Accelerometer offset calibration done!");
  Serial.println("Roll: " + (String) acc_cal_roll);
  Serial.println("Pitch: " + (String) acc_cal_pitch);
}

void calibrate_accel() {
  Wire.beginTransmission(MPU9250_ADDR);                                   //Start communication with the gyro.
  Wire.write(0x3B);                                                       //Start reading @ register 43h and auto increment with every read.
  Wire.endTransmission();                                                 //End the transmission.
  Wire.requestFrom(MPU9250_ADDR, 14);                                      //Request 6 bytes from the gyro.

  while (Wire.available() < 14);
  acc_x_raw = (int16_t) Wire.read() << 8 | (int16_t) Wire.read();
  acc_y_raw = (int16_t) Wire.read() << 8 | (int16_t) Wire.read();
  acc_z_raw = (int16_t) Wire.read() << 8 | (int16_t) Wire.read();
  temperature = (int16_t) Wire.read() << 8 | (int16_t) Wire.read();
  gyro_x_raw = (int16_t) Wire.read() << 8 | (int16_t) Wire.read();
  gyro_y_raw = (int16_t) Wire.read() << 8 | (int16_t) Wire.read();
  gyro_z_raw = (int16_t) Wire.read() << 8 | (int16_t) Wire.read();

  gyro_x_raw -= gyro_cal[1];
  gyro_y_raw -= gyro_cal[2];
  gyro_z_raw -= gyro_cal[3];

#ifdef X_FLIP
  gyro_x_raw *= -1;
  acc_x_raw *= -1;
#endif

#ifdef Y_FLIP
  gyro_y_raw *= -1;
  acc_y_raw *= -1;
#endif

#ifdef Z_FLIP
  gyro_z_raw *= -1;
  acc_z_raw *= -1;
#endif

  acc_x_raw = accel_cal[0] * acc_x_raw + accel_cal[1];
  acc_y_raw = accel_cal[2] * acc_y_raw + accel_cal[3];
  acc_z_raw = accel_cal[4] * acc_z_raw + accel_cal[5];

  //Gyro calculations 0.0000610687 = (0.004 / 65.5)
  angle_roll += gyro_x_raw * 0.0000610687;
  angle_pitch += gyro_y_raw * 0.0000610687;

  angle_roll_acc = (float) (atan2(acc_y_raw, acc_z_raw)) * RAD_TO_DEG;
  angle_pitch_acc = (float) (atan2(acc_z_raw, acc_x_raw)) * RAD_TO_DEG;

  angle_pitch_acc += (float) 90.0;
  if (angle_roll_acc > 90) angle_roll_acc -= (float) 180;
  else angle_roll_acc += (float) 180;

  angle_roll = angle_roll * 0.96 + angle_roll_acc * 0.04;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.
  angle_pitch = angle_pitch * 0.96 + angle_pitch_acc * 0.04;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
}

void calculate_pitch_roll() {
  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_x / 65.5) * 0.3);     //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_y / 65.5) * 0.3);     //Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_z / 65.5) * 0.3);         //Gyro pid input is deg/sec.

  uint8_t data[7];
  Wire.beginTransmission(MPU9250_ADDR);                                   //Start communication with the gyro.
  Wire.write(0x3B);                                                       //Start reading @ register 43h and auto increment with every read.
  Wire.endTransmission();                                                 //End the transmission.
  Wire.requestFrom(MPU9250_ADDR, 21);                                      //Request 6 bytes from the gyro.

  while (Wire.available() < 21);
  acc_x_raw = (int16_t) Wire.read() << 8 | Wire.read();
  acc_y_raw = (int16_t) Wire.read() << 8 | Wire.read();
  acc_z_raw = (int16_t) Wire.read() << 8 | Wire.read();
  temperature = (int16_t) Wire.read() << 8 | Wire.read();
  gyro_x_raw = (int16_t) Wire.read() << 8 | Wire.read();
  gyro_y_raw = (int16_t) Wire.read() << 8 | Wire.read();
  gyro_z_raw = (int16_t) Wire.read() << 8 | Wire.read();
  for (int i = 0; i < 7; i++) data[i] = Wire.read();
  mag_x_raw = (int16_t) data[1] << 8 | data[0];
  mag_y_raw = (int16_t) data[3] << 8 | data[2];
  mag_z_raw = (int16_t) data[5] << 8 | data[4];

  gyro_x_raw -= gyro_cal[1];
  gyro_y_raw -= gyro_cal[2];
  gyro_z_raw -= gyro_cal[3];

#ifndef DEBUG_GYRO
  Serial.print(gyro_x_raw);
  Serial.print(",");
  Serial.print(gyro_y_raw);
  Serial.print(",");
  Serial.println(gyro_z_raw);
#endif

#ifndef DEBUG_ACCEL
  Serial.print(acc_x_raw);
  Serial.print(",");
  Serial.print(acc_y_raw);
  Serial.print(",");
  Serial.println(acc_z_raw);
#endif

#ifdef X_FLIP
  gyro_x_raw *= -1;
  acc_x_raw *= -1;
#endif

#ifdef Y_FLIP
  gyro_y_raw *= -1;
  acc_y_raw *= -1;
#endif

#ifdef Z_FLIP
  gyro_z_raw *= -1;
  acc_z_raw *= -1;
#endif

  calculate_moving_average();

  acc_x = accel_cal[0] * acc_x + accel_cal[1];
  acc_y = accel_cal[2] * acc_y + accel_cal[3];
  acc_z = accel_cal[4] * acc_z + accel_cal[5];

  //Gyro calculations 0.0000610687 = (0.004 / 65.5)
  angle_roll += gyro_x * 0.0000610687;
  angle_pitch += gyro_y * 0.0000610687;
  angle_yaw += gyro_z * 0.0000610687;

  //Accelerometer angle calculations
  angle_roll_acc = (float) (atan2(acc_y, acc_z)) * RAD_TO_DEG;
  angle_pitch_acc = (float) (atan2(acc_z, acc_x)) * RAD_TO_DEG;

  angle_pitch_acc += (float) 90.0;
  if (angle_roll_acc > 90) angle_roll_acc -= (float) 180;
  else angle_roll_acc += (float) 180;

  angle_roll_acc -= acc_cal_roll;                                       //Accelerometer calibration value for roll.
  angle_pitch_acc -= acc_cal_pitch;                                     //Accelerometer calibration value for pitch.

  angle_roll = angle_roll * 0.96 + angle_roll_acc * 0.04;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.
  angle_pitch = angle_pitch * 0.96 + angle_pitch_acc * 0.04;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.

  roll = angle_roll;
  pitch = angle_pitch;

  roll_level_adjust = roll * 15;                                      //Calculate the roll angle correction
  pitch_level_adjust = pitch * 15;                                    //Calculate the pitch angle correction

#ifndef DEBUG_PITCH_ROLL
  Serial.print(roll);
  Serial.print(",");
  Serial.println(pitch);
#endif
}

void calculate_moving_average() {
  gyro_x_sum += gyro_x_raw;
  gyro_x_sum -= gyro_x_mem[gyro_loop_counter];
  gyro_x_mem[gyro_loop_counter] = gyro_x_raw;
  gyro_x = gyro_x_sum / 8.0;

  gyro_y_sum += gyro_y_raw;
  gyro_y_sum -= gyro_y_mem[gyro_loop_counter];
  gyro_y_mem[gyro_loop_counter] = gyro_y_raw;
  gyro_y = gyro_y_sum / 8.0;

  gyro_z_sum += gyro_z_raw;
  gyro_z_sum -= gyro_z_mem[gyro_loop_counter];
  gyro_z_mem[gyro_loop_counter] = gyro_z_raw;
  gyro_z = gyro_z_sum / 8.0;

  acc_x_sum += acc_x_raw;
  acc_x_sum -= acc_x_mem[acc_loop_counter];
  acc_x_mem[acc_loop_counter] = acc_x_raw;
  acc_x = acc_x_sum / 16.0;

  acc_y_sum += acc_y_raw;
  acc_y_sum -= acc_y_mem[acc_loop_counter];
  acc_y_mem[acc_loop_counter] = acc_y_raw;
  acc_y = acc_y_sum / 16.0;

  acc_z_sum += acc_z_raw;
  acc_z_sum -= acc_z_mem[acc_loop_counter];
  acc_z_mem[acc_loop_counter] = acc_z_raw;
  acc_z = acc_z_sum / 16.0;

  if (gyro_loop_counter == 7) gyro_loop_counter = 0;
  else gyro_loop_counter++;

  if (acc_loop_counter == 15) acc_loop_counter = 0;
  else acc_loop_counter++;
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

  //  Wire.beginTransmission(BARO_ADDR);
  //  Wire.write(0xF5);
  //  Wire.write();
  //  Wire.endTransmission();
  //
  //  Wire.beginTransmission(BARO_ADDR);
  //  Wire.write(0xF4);
  //  Wire.write();
  //  Wire.endTransmission();
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
