using namespace std;

#include <Wire.h>
#include <EEPROM.h>

#include <vector>
#include <algorithm>

#define MPU9250_ADDR 0x68
#define AK8963_ADDR 0x0C
#define BARO_ADDR 0x76

#define AK8963_CNTRL 0x0A
#define AK8963_FUSE_ROM 0x0F
#define AK8963_ASA 0x10
#define AK8963_HXL 0x03
#define EXT_SENS_DATA_00 0x49

#define I2C_SLV0_ADDR 0x25
#define I2C_SLV0_REG 0x26
#define I2C_SLV0_CTRL 0x27
#define I2C_SLV0_DO 0x63

#define MAG_THRESHOLD 40

int error = 0;

//Transmitter variables
unsigned long timer, timer_1, timer_2, timer_3, timer_4, timer_5, timer_6, current_time;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4, last_channel_5, last_channel_6;
volatile int receiver_input[7];

//IMU
int16_t acc_x, acc_y, acc_z;
int16_t mag_x_raw, mag_y_raw, mag_z_raw;
int16_t prev_mag_x, prev_mag_y, prev_mag_z;
int16_t mag_sensitivity[3];
vector<int16_t> mag_data_x, mag_data_y, mag_data_z;

//EEPROM variables
uint16_t center_channel_1, center_channel_2, center_channel_3, center_channel_4;
uint16_t high_channel_1, high_channel_2, high_channel_3, high_channel_4;
uint16_t low_channel_1, low_channel_2, low_channel_3, low_channel_4;
float accel_results[6], compass_cal_values[6];

typedef union {
  float decimal;
  uint8_t bytes[4];
} converter;

converter number;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(14), receiver_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(15), receiver_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(16), receiver_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(17), receiver_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(22), receiver_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(23), receiver_change, CHANGE);

  for (int i = 1; i < 6; i++) receiver_input[i] = 0;

  setup_sensor();

  Serial.println("Setup DONE");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// MAIN LOOP ///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  //  if (error == 0) wait_for_receiver();      //Wait for transmitter to be turned on within 10 seconds
  //  if (error == 0) record_center_position(); //Find out the center positions of transmitter (including throttle)
  //  if (error == 0) register_min_max();       //Register the min and max values of the receiver channels
  //  if (error == 0) calibrate_accel();        //Measure the endpoints of each accelerometer axis
  if (error == 0) calibrate_mag();          //Find the scale and offset values for the magnetometer
  //  if (error == 0) save_eeprom_data();       //If all is good, store the information in the EEPROM

  if (error == 0) Serial.println("Setup SUCCESS! \n You may close the program now :)");
  else Serial.println("There is an error during setup. Please try again.");

  while (1);

  delay(100);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// MAIN LOOP ///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

void wait_for_receiver() {
  Serial.println("Turn on your transmitter in the next 10 seconds.");
  byte zero = 0;
  timer = millis() + 10000;
  while (timer > millis() && zero < 15)  {
    if (receiver_input[1] < 2100 && receiver_input[1] > 900)zero |= 0b00000001;
    if (receiver_input[2] < 2100 && receiver_input[2] > 900)zero |= 0b00000010;
    if (receiver_input[3] < 2100 && receiver_input[3] > 900)zero |= 0b00000100;
    if (receiver_input[4] < 2100 && receiver_input[4] > 900)zero |= 0b00001000;
    delay(500);
    Serial.print(F("."));
  }
  if (zero == 0)  {
    error = 1;
    Serial.println(F("."));
    Serial.println(F("No valid receiver signals found!"));
  }
  else Serial.println(F(" OK"));
}

void record_center_position() {
  delay(2000);
  Serial.println(F("Place all sticks in the center position within 10 seconds."));
  for (int i = 9; i > 0; i--)  {
    delay(1000);
    Serial.print(i);
    Serial.print(" ");
  }
  Serial.println(" ");

  //Store the central stick positions
  center_channel_1 = receiver_input[1];
  center_channel_2 = receiver_input[2];
  center_channel_3 = receiver_input[3];
  center_channel_4 = receiver_input[4];

  Serial.println(F(""));
  Serial.println(F("Center positions stored."));
  Serial.print(F("Input D8 = "));
  Serial.println(receiver_input[1]);
  Serial.print(F("Input D9 = "));
  Serial.println(receiver_input[2]);
  Serial.print(F("Input D10 = "));
  Serial.println(receiver_input[3]);
  Serial.print(F("Input D11 = "));
  Serial.println(receiver_input[4]);
  Serial.println(F(""));
  Serial.println(F(""));

  delay(2000);
}

void register_min_max() {
  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("Gently move all the sticks simultaneously to their extends"));
  Serial.println(F("When ready put the sticks back in their center positions"));

  byte zero = 0;
  low_channel_1 = receiver_input[1];
  low_channel_2 = receiver_input[2];
  low_channel_3 = receiver_input[3];
  low_channel_4 = receiver_input[4];

  while (receiver_input[1] < center_channel_1 + 20 && receiver_input[1] > center_channel_1 - 20) {
    //Serial.println(receiver_input[1]);
    delay(250); //check if channel 1 centered
  }
  Serial.println(F("Measuring endpoints...."));
  while (zero < 15) {
    if (receiver_input[1] < center_channel_1 + 20 && receiver_input[1] > center_channel_1 - 20)  zero |= 0b00000001;
    if (receiver_input[2] < center_channel_2 + 20 && receiver_input[2] > center_channel_2 - 20)  zero |= 0b00000010;
    if (receiver_input[3] < center_channel_3 + 20 && receiver_input[3] > center_channel_3 - 20)  zero |= 0b00000100;
    if (receiver_input[4] < center_channel_4 + 20 && receiver_input[4] > center_channel_4 - 20)  zero |= 0b00001000;

    if (receiver_input[1] < low_channel_1) low_channel_1 = receiver_input[1];
    if (receiver_input[2] < low_channel_2) low_channel_2 = receiver_input[2];
    if (receiver_input[3] < low_channel_3) low_channel_3 = receiver_input[3];
    if (receiver_input[4] < low_channel_4) low_channel_4 = receiver_input[4];

    if (receiver_input[1] > high_channel_1)  high_channel_1 = receiver_input[1];
    if (receiver_input[2] > high_channel_2)  high_channel_2 = receiver_input[2];
    if (receiver_input[3] > high_channel_3)  high_channel_3 = receiver_input[3];
    if (receiver_input[4] > high_channel_4)  high_channel_4 = receiver_input[4];

    delay(100);
  }

  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("High, low and center values found during setup"));
  Serial.print(F("Digital input 08 values: "));
  Serial.print(low_channel_1);
  Serial.print(F(" - "));
  Serial.print(center_channel_1);
  Serial.print(F(" - "));
  Serial.println(high_channel_1);
  Serial.print(F("Digital input 09 values: "));
  Serial.print(low_channel_2);
  Serial.print(F(" - "));
  Serial.print(center_channel_2);
  Serial.print(F(" - "));
  Serial.println(high_channel_2);
  Serial.print(F("Digital input 10 values: "));
  Serial.print(low_channel_3);
  Serial.print(F(" - "));
  Serial.print(center_channel_3);
  Serial.print(F(" - "));
  Serial.println(high_channel_3);
  Serial.print(F("Digital input 11 values: "));
  Serial.print(low_channel_4);
  Serial.print(F(" - "));
  Serial.print(center_channel_4);
  Serial.print(F(" - "));
  Serial.println(high_channel_4);
}

void calibrate_accel() {
  String text_prompt[6] = {"Place the drone upright on a flat surface (Z axis LOW)",
                           "Place the drone on its RIGHT side(X axis LOW)",
                           "Place the drone on its LEFT side (X axis HIGH)",
                           "Point the nose of the drone DOWNWARDS (Y axis LOW)",
                           "Point the nose of the drone UPWARDS (Y axis HIGH)",
                           "Flip the drone upside down (Z axis HIGH)"
                          };
  int32_t axis[6] = {3, 2, 2, 1, 1, 3};

  Serial.println(F(""));
  Serial.println(F("==================================================="));
  Serial.println(F("Measuring accelerometer endpoints"));
  Serial.println(F("==================================================="));
  Serial.print("Starting in: ");
  for (int i = 10; i > 0; i--) {
    Serial.print(i);
    Serial.print(" ");
    delay(1000);
  }
  Serial.println();

  for (int i = 0; i < 6; i++) {
    Serial.println(text_prompt[i]);
    for (int j = 5; j > 0; j--) {
      Serial.print(j);
      Serial.print(" ");
      delay(1000);
    }
    Serial.print(F("Hold it there"));

    int32_t acc_x_sum = 0, acc_y_sum = 0, acc_z_sum = 0;
    for (int k = 0; k < 2000; k++) {
      if (k % 200 == 0) Serial.print(".");
      Wire.beginTransmission(MPU9250_ADDR);                                   //Start communication with the gyro.
      Wire.write(0x3B);                                                       //Start reading @ register 43h and auto increment with every read.
      Wire.endTransmission();                                                 //End the transmission.
      Wire.requestFrom(MPU9250_ADDR, 6);                                      //Request 6 bytes from the gyro.

      while (Wire.available() < 6);
      acc_x = (int16_t) Wire.read() << 8 | Wire.read();
      acc_y = (int16_t) Wire.read() << 8 | Wire.read();
      acc_z = (int16_t) Wire.read() << 8 | Wire.read();

      acc_x *= -1;
      acc_y *= -1;

      acc_x_sum += acc_x;
      acc_y_sum += acc_y;
      acc_z_sum += acc_z;
    }
    Serial.println();
    if (axis[i] == 1) {
      acc_x_sum /= 2000;
      Serial.println(acc_x_sum);
      axis[i] = acc_x_sum;
    } else if (axis[i] == 2) {
      acc_y_sum /= 2000;
      Serial.println(acc_y_sum);
      axis[i] = acc_y_sum;
    } else if (axis[i] == 3) {
      acc_z_sum /= 2000;
      Serial.println(acc_z_sum);
      axis[i] = acc_z_sum;
    }
    Serial.println();
  }
  int32_t data[6];
  Serial.print("X axis LOW: ");
  Serial.println(axis[3]);
  data[0] = axis[3];
  Serial.print("X axis HIGH: ");
  Serial.println(axis[4]);
  data[1] = axis[4];

  Serial.print("Y axis LOW: ");
  Serial.println(axis[1]);
  data[2] = axis[1];
  Serial.print("Y axis HIGH: ");
  Serial.println(axis[2]);
  data[3] = axis[2];

  Serial.print("Z axis LOW: ");
  Serial.println(axis[0]);
  data[4] = axis[0];
  Serial.print("Z axis HIGH: ");
  Serial.println(axis[5]);
  data[5] = axis[5];

  for (int i = 0; i < 3; i++) {
    float gradient = 8192.0 / (data[i * 2 + 1] - data[i * 2]);
    float intercept = -4096.0 - gradient * data[i * 2];

    accel_results[i * 2] = gradient;
    accel_results[i * 2 + 1] = intercept;
  }

  Serial.println();
  Serial.print("X axis gradient: ");
  Serial.println(accel_results[0], 5);
  Serial.print("X axis intercept: ");
  Serial.println(accel_results[1], 5);
  Serial.print("Y axis gradient: ");
  Serial.println(accel_results[2], 5);
  Serial.print("Y axis intercept: ");
  Serial.println(accel_results[3], 5);
  Serial.print("Z axis gradient: ");
  Serial.println(accel_results[4], 5);
  Serial.print("Z axis intercept: ");
  Serial.println(accel_results[5], 5);
}

void calibrate_mag() {
  Serial.println(F(""));
  Serial.println(F("==================================================="));
  Serial.println(F("Measuring Magnetometer endpoints"));
  Serial.println(F("==================================================="));
  Serial.print("Starting in: ");
  for (int i = 10; i > 0; i--) {
    Serial.print(i);
    Serial.print(" ");
    delay(1000);
  }
  Serial.println();

  uint32_t start = millis();
  uint32_t difference = 0;

  while (difference < 50000) {
    difference = millis() - start;

    uint8_t data[7];
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(0x49);
    Wire.endTransmission();
    Wire.requestFrom(MPU9250_ADDR, 7);

    while (Wire.available() < 7);
    for (int i = 0; i < 7; i++) data[i] = Wire.read();
    mag_x_raw = (int16_t) data[1] << 8 | data[0];
    mag_y_raw = (int16_t) data[3] << 8 | data[2];
    mag_z_raw = (int16_t) data[5] << 8 | data[4];

    mag_x_raw *= mag_sensitivity[0];
    mag_y_raw *= mag_sensitivity[1];
    mag_z_raw *= mag_sensitivity[2];

    if ((mag_x_raw != prev_mag_x) && (mag_y_raw != prev_mag_y) && (mag_z_raw != prev_mag_z)) {
      //      Serial.print(difference / 1000);
      //      Serial.print(": ");
      Serial.print(mag_x_raw);
      Serial.print(",");
      Serial.print(mag_y_raw);
      Serial.print(",");
      Serial.println(mag_z_raw);

      mag_data_x.push_back(mag_x_raw);
      mag_data_y.push_back(mag_y_raw);
      mag_data_z.push_back(mag_z_raw);
    }
    prev_mag_x = mag_x_raw;
    prev_mag_y = mag_y_raw;
    prev_mag_z = mag_z_raw;
  }
  Serial.println("\n Calculating offsets and scales for magnetometer... \n");

  uint8_t prev_percent = 0;
  vector<uint16_t> outlier_index;
  for (uint16_t i = 0; i < mag_data_x.size(); i++) {
    float smallest_dist[6] = {9999, 9999, 9999, 9999, 9999, 9999};

    uint8_t percentage = (float) i / (float) mag_data_x.size() * 10.0;
    if (percentage != prev_percent) Serial.println((String) percentage + "%");
    prev_percent = percentage;

    for (uint16_t j = 0; j < mag_data_x.size(); j++) {
      int16_t x = mag_data_x[j] - mag_data_x[i];
      int16_t y = mag_data_y[j] - mag_data_y[i];
      int16_t z = mag_data_z[j] - mag_data_z[i];

      float distance = sqrt(sq(x) + sq(y) + sq(z));
      if (distance != 0) {
        smallest_dist[5] = distance;
        sort(smallest_dist, smallest_dist + sizeof(smallest_dist) / sizeof(smallest_dist[0]));
      }
    }

    float ave_dist = 0.0;
    for (int k = 0; k < 5; k++) ave_dist += smallest_dist[k];
    ave_dist /= 5;

    if (ave_dist > MAG_THRESHOLD) {
      Serial.println(ave_dist);
      outlier_index.push_back(i);     //Mark the coordinate down for removal
    }
  }

  sort(outlier_index.begin(), outlier_index.end());
  reverse(outlier_index.begin(), outlier_index.end());

  for (uint8_t i = 0; i < outlier_index.size(); i++) {
    Serial.print((String) outlier_index[i] + " ");
    mag_data_x.erase(mag_data_x.begin() + outlier_index[i]);
    mag_data_y.erase(mag_data_y.begin() + outlier_index[i]);
    mag_data_z.erase(mag_data_z.begin() + outlier_index[i]);
  }
  Serial.println();

  for (uint16_t i = 0; i < mag_data_x.size(); i++) {
    if (mag_data_x[i] < compass_cal_values[0]) compass_cal_values[0] = mag_data_x[i];
    if (mag_data_x[i] > compass_cal_values[1]) compass_cal_values[1] = mag_data_x[i];
    if (mag_data_y[i] < compass_cal_values[2]) compass_cal_values[2] = mag_data_y[i];
    if (mag_data_y[i] > compass_cal_values[3]) compass_cal_values[3] = mag_data_y[i];
    if (mag_data_z[i] < compass_cal_values[4]) compass_cal_values[4] = mag_data_z[i];
    if (mag_data_z[i] > compass_cal_values[5]) compass_cal_values[5] = mag_data_z[i];
  }

  float mag_x_offset = (compass_cal_values[1] + compass_cal_values[0]) / 2.0;
  float mag_y_offset = (compass_cal_values[3] + compass_cal_values[2]) / 2.0;
  float mag_z_offset = (compass_cal_values[5] + compass_cal_values[4]) / 2.0;

  float avg_delta_x = (compass_cal_values[1] - compass_cal_values[0]) / 2.0;
  float avg_delta_y = (compass_cal_values[3] - compass_cal_values[2]) / 2.0;
  float avg_delta_z = (compass_cal_values[5] - compass_cal_values[4]) / 2.0;

  float avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3.0;

  float mag_x_scale = avg_delta / avg_delta_x;
  float mag_y_scale = avg_delta / avg_delta_y;
  float mag_z_scale = avg_delta / avg_delta_z;

  Serial.print("X Offset: ");
  Serial.println(mag_x_offset);
  Serial.print("Y Offset: ");
  Serial.println(mag_y_offset);
  Serial.print("Z Offset: ");
  Serial.println(mag_z_offset);
  Serial.println();

  Serial.print("X Scale: ");
  Serial.println(mag_x_scale);
  Serial.print("Y Scale: ");
  Serial.println(mag_y_scale);
  Serial.print("Z Scale: ");
  Serial.println(mag_z_scale);

  compass_cal_values[0] = mag_x_scale;
  compass_cal_values[1] = mag_x_offset;
  compass_cal_values[2] = mag_y_scale;
  compass_cal_values[3] = mag_y_offset;
  compass_cal_values[4] = mag_z_scale;
  compass_cal_values[5] = mag_z_offset;

  /*
    //Sort arrays for computation of offsets
    sort(mag_data_x.begin(), mag_data_x.end());
    sort(mag_data_y.begin(), mag_data_y.end());
    sort(mag_data_z.begin(), mag_data_z.end());

    //Compute offsets first
    float mag_x_offset = (mag_data_x.back() + mag_data_x.front()) / 2.0;
    float mag_y_offset = (mag_data_y.back() + mag_data_y.front()) / 2.0;
    float mag_z_offset = (mag_data_z.back() + mag_data_z.front()) / 2.0;

    Serial.print("X Offset: ");
    Serial.println(mag_x_offset);
    Serial.print("Y Offset: ");
    Serial.println(mag_y_offset);
    Serial.print("Z Offset: ");
    Serial.println(mag_z_offset);

    compass_cal_values[0] = mag_x_offset;
    compass_cal_values[1] = mag_y_offset;
    compass_cal_values[2] = mag_z_offset;

    uint32_t ave_vector = 0;
    for (int i = 0; i < mag_data_x.size(); i++) {
      //Center the sphere by removing the offset
      mag_data_x[i] -= mag_x_offset;
      mag_data_y[i] -= mag_y_offset;
      mag_data_z[i] -= mag_z_offset;

      //Compute the average total magnetic vector
      ave_vector += sqrt(sq(mag_data_x[i]) + sq(mag_data_y[i]) + sq(mag_data_z[i]));
    }
    ave_vector /= mag_data_x.size();

    for (int i = mag_data_x.size() - 1; i >= 0; i--) {
      float vector = sqrt(sq(mag_data_x[i]) + sq(mag_data_y[i]) + sq(mag_data_z[i]));
      if (vector > 1.3 * ave_vector) {
        mag_data_x.erase(mag_data_x.begin() + i);
        mag_data_y.erase(mag_data_y.begin() + i);
        mag_data_z.erase(mag_data_z.begin() + i);
      }
    }

    //Sort arrays for computation of offsets and scales
    sort(mag_data_x.begin(), mag_data_x.end());
    sort(mag_data_y.begin(), mag_data_y.end());
    sort(mag_data_z.begin(), mag_data_z.end());

    //Compute offsets again
    mag_x_offset = (mag_data_x.back() + mag_data_x.front()) / 2.0;
    mag_y_offset = (mag_data_y.back() + mag_data_y.front()) / 2.0;
    mag_z_offset = (mag_data_z.back() + mag_data_z.front()) / 2.0;

    //Compute scales
    float avg_delta_x = (mag_data_x.back() - mag_data_x.front()) / 2.0;
    float avg_delta_y = (mag_data_y.back() - mag_data_y.front()) / 2.0;
    float avg_delta_z = (mag_data_z.back() - mag_data_z.front()) / 2.0;

    float avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3.0;

    float mag_x_scale = avg_delta / avg_delta_x;
    float mag_y_scale = avg_delta / avg_delta_y;
    float mag_z_scale = avg_delta / avg_delta_z;

    Serial.print("X LOW: ");
    Serial.println(mag_data_x.front());
    Serial.print("X HIGH: ");
    Serial.println(mag_data_x.back());
    Serial.print("Y LOW: ");
    Serial.println(mag_data_y.front());
    Serial.print("Y HIGH: ");
    Serial.println(mag_data_y.back());
    Serial.print("Z LOW: ");
    Serial.println(mag_data_z.front());
    Serial.print("Z HIGH: ");
    Serial.println(mag_data_z.back());
    Serial.println();

    Serial.print("X Scale: ");
    Serial.println(mag_x_scale);
    Serial.print("X Offset: ");
    Serial.println(mag_x_offset);
    Serial.print("Y Scale: ");
    Serial.println(mag_y_scale);
    Serial.print("Y Offset: ");
    Serial.println(mag_y_offset);
    Serial.print("Z Scale: ");
    Serial.println(mag_z_scale);
    Serial.print("Z Offset: ");
    Serial.println(mag_z_offset);

    compass_cal_values[0] = mag_x_scale;
    compass_cal_values[1] += mag_x_offset;
    compass_cal_values[2] = mag_y_scale;
    compass_cal_values[3] += mag_y_offset;
    compass_cal_values[4] = mag_z_scale;
    compass_cal_values[5] += mag_z_offset;*/

  //Save compass calibration values into EEPROM
  for (int i = 0; i < 6; i++) {
    number.decimal = compass_cal_values[i];
    EEPROM.write(i * 4 + 48, number.bytes[0]);
    EEPROM.write(i * 4 + 49, number.bytes[1]);
    EEPROM.write(i * 4 + 50, number.bytes[2]);
    EEPROM.write(i * 4 + 51, number.bytes[3]);
  } //last register number: 71
}

void save_eeprom_data() {
  Serial.println(F(""));
  Serial.println(F("==================================================="));
  Serial.println(F("Storing EEPROM information"));
  Serial.println(F("==================================================="));
  Serial.println(F("Writing EEPROM"));
  delay(1000);

  EEPROM.write(0, center_channel_1 & 0b11111111);
  EEPROM.write(1, center_channel_1 >> 8);
  EEPROM.write(2, center_channel_2 & 0b11111111);
  EEPROM.write(3, center_channel_2 >> 8);
  EEPROM.write(4, center_channel_3 & 0b11111111);
  EEPROM.write(5, center_channel_3 >> 8);
  EEPROM.write(6, center_channel_4 & 0b11111111);
  EEPROM.write(7, center_channel_4 >> 8);
  EEPROM.write(8, high_channel_1 & 0b11111111);
  EEPROM.write(9, high_channel_1 >> 8);
  EEPROM.write(10, high_channel_2 & 0b11111111);
  EEPROM.write(11, high_channel_2 >> 8);
  EEPROM.write(12, high_channel_3 & 0b11111111);
  EEPROM.write(13, high_channel_3 >> 8);
  EEPROM.write(14, high_channel_4 & 0b11111111);
  EEPROM.write(15, high_channel_4 >> 8);
  EEPROM.write(16, low_channel_1 & 0b11111111);
  EEPROM.write(17, low_channel_1 >> 8);
  EEPROM.write(18, low_channel_2 & 0b11111111);
  EEPROM.write(19, low_channel_2 >> 8);
  EEPROM.write(20, low_channel_3 & 0b11111111);
  EEPROM.write(21, low_channel_3 >> 8);
  EEPROM.write(22, low_channel_4 & 0b11111111);
  EEPROM.write(23, low_channel_4 >> 8);

  for (int i = 0; i < 6; i++) {
    number.decimal = accel_results[i];
    EEPROM.write(i * 4 + 24, number.bytes[0]);
    EEPROM.write(i * 4 + 25, number.bytes[1]);
    EEPROM.write(i * 4 + 26, number.bytes[2]);
    EEPROM.write(i * 4 + 27, number.bytes[3]);
  } //last register number: 47

  for (int i = 0; i < 6; i++) {
    number.decimal = compass_cal_values[i];
    EEPROM.write(i * 4 + 48, number.bytes[0]);
    EEPROM.write(i * 4 + 49, number.bytes[1]);
    EEPROM.write(i * 4 + 50, number.bytes[2]);
    EEPROM.write(i * 4 + 51, number.bytes[3]);
  } //last register number: 71

  Serial.println(F("Done!"));

  //Write the EEPROM signature
  EEPROM.write(72, 'J');
  EEPROM.write(73, 'M');
  EEPROM.write(74, 'B');

  //To make sure evrything is ok, verify the EEPROM data.
  Serial.println(F("Verify EEPROM data"));
  delay(1000);
  if (center_channel_1 != ((EEPROM.read(1) << 8) | EEPROM.read(0)))error = 1;
  if (center_channel_2 != ((EEPROM.read(3) << 8) | EEPROM.read(2)))error = 1;
  if (center_channel_3 != ((EEPROM.read(5) << 8) | EEPROM.read(4)))error = 1;
  if (center_channel_4 != ((EEPROM.read(7) << 8) | EEPROM.read(6)))error = 1;

  if (high_channel_1 != ((EEPROM.read(9) << 8) | EEPROM.read(8)))error = 1;
  if (high_channel_2 != ((EEPROM.read(11) << 8) | EEPROM.read(10)))error = 1;
  if (high_channel_3 != ((EEPROM.read(13) << 8) | EEPROM.read(12)))error = 1;
  if (high_channel_4 != ((EEPROM.read(15) << 8) | EEPROM.read(14)))error = 1;

  if (low_channel_1 != ((EEPROM.read(17) << 8) | EEPROM.read(16)))error = 1;
  if (low_channel_2 != ((EEPROM.read(19) << 8) | EEPROM.read(18)))error = 1;
  if (low_channel_3 != ((EEPROM.read(21) << 8) | EEPROM.read(20)))error = 1;
  if (low_channel_4 != ((EEPROM.read(23) << 8) | EEPROM.read(22)))error = 1;

  for (int i = 0; i < 6; i++) {
    number.bytes[0] = EEPROM.read(i * 4 + 24);
    number.bytes[1] = EEPROM.read(i * 4 + 25);
    number.bytes[2] = EEPROM.read(i * 4 + 26);
    number.bytes[3] = EEPROM.read(i * 4 + 27);
    if (accel_results[i] != number.decimal) error = 1;
  }

  for (int i = 0; i < 6; i++) {
    number.bytes[0] = EEPROM.read(i * 4 + 48);
    number.bytes[1] = EEPROM.read(i * 4 + 49);
    number.bytes[2] = EEPROM.read(i * 4 + 50);
    number.bytes[3] = EEPROM.read(i * 4 + 51);
    if (compass_cal_values[i] != number.decimal) error = 1;
  }

  if ('J' != EEPROM.read(72))error = 1;
  if ('M' != EEPROM.read(73))error = 1;
  if ('B' != EEPROM.read(74))error = 1;

  if (error == 1)Serial.println(F("EEPROM verification failed!!!"));
  else Serial.println(F("Verification done"));
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

void receiver_change() {
  current_time = micros();
  //* ========================================= Channel 1 =========================================
  if (GPIOD_PDIR & 2) { //0000 0000 0000 0000 0000 0000 0000 0010 --> PTD1
    if (last_channel_1 == 0) {                                              //Input 14 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  } else if (last_channel_1 == 1) {                                         //Input 14 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    receiver_input[1] = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }
  //* ========================================= Channel 2 =========================================
  if (GPIOC_PDIR & 1) { //0000 0000 0000 0000 0000 0000 0000 0001 --> PTC0
    if (last_channel_2 == 0) {                                              //Input 15 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_2 to current_time.
    }
  } else if (last_channel_2 == 1) {                                          //Input 15 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
    receiver_input[2] = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
  }
  //* ========================================= Channel 3 =========================================
  if (GPIOB_PDIR & 1) { //0000 0000 0000 0000 0000 0000 0000 0001 --> PTB0
    if (last_channel_3 == 0) {                                              //Input 16 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  } else if (last_channel_3 == 1) {                                          //Input 16 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    receiver_input[3] = current_time - timer_3;                             //Channel 3 is current_time - timer_3.
  }
  //* ========================================= Channel 4 =========================================
  if (GPIOB_PDIR & 2) { //0000 0000 0000 0000 0000 0000 0000 0010 --> PTB1
    if (last_channel_4 == 0) {                                              //Input 17 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  } else if (last_channel_4 == 1) {                                          //Input 17 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    receiver_input[4] = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
  }
  //* ========================================= Channel 5 =========================================
  if (GPIOC_PDIR & 2) { //0000 0000 0000 0000 0000 0000 0000 0010 --> PTC1
    if (last_channel_5 == 0) {                                              //Input 22 changed from 0 to 1.
      last_channel_5 = 1;                                                   //Remember current input state.
      timer_5 = current_time;                                               //Set timer_5 to current_time.
    }
  } else if (last_channel_5 == 1) {                                          //Input 22 is not high and changed from 1 to 0.
    last_channel_5 = 0;                                                     //Remember current input state.
    receiver_input[5] = current_time - timer_5;                             //Channel 5 is current_time - timer_5.
  }
  //* ========================================= Channel 6 =========================================
  if (GPIOC_PDIR & 4) { //0000 0000 0000 0000 0000 0000 0000 0100 --> PTC2
    if (last_channel_6 == 0) {                                              //Input 23 changed from 0 to 1.
      last_channel_6 = 1;                                                   //Remember current input state.
      timer_6 = current_time;                                               //Set timer_6 to current_time.
    }
  } else if (last_channel_6 == 1) {                                          //Input 23 is not high and changed from 1 to 0.
    last_channel_6 = 0;                                                     //Remember current input state.
    receiver_input[6] = current_time - timer_6;                             //Channel 6 is current_time - timer_6.
  }
}

void find_min_max() {

}
