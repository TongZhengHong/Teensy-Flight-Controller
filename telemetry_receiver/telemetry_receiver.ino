#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define LAST_BYTE 34

//#define DEBUG_PRINT
#define TUNING_MODE

#define CE_PIN 7
#define CSN_PIN 8
#define RADIO_CHANNEL 100

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

bool start_count = false;
uint8_t check_byte;
uint8_t prev_receiver_byte, receiver_byte;
uint8_t data_count, receiver_data[50];
uint8_t pid_mode = 1;

float prev_p_roll, prev_i_roll, prev_d_roll;
float prev_p_pitch, prev_i_pitch, prev_d_pitch;
float prev_p_yaw, prev_i_yaw, prev_d_yaw;

//Data
int8_t roll, pitch;
uint16_t heading;
int32_t latitude, longitude;
uint16_t channel_1, channel_2, channel_3, channel_4;
uint16_t loop_time;
float pid_p_roll, pid_i_roll, pid_d_roll;
float pid_p_pitch, pid_i_pitch, pid_d_pitch;
float pid_p_yaw, pid_i_yaw, pid_d_yaw;

void setup() {
  Serial.begin(115200);

  radio.begin();
  radio.setChannel(RADIO_CHANNEL);
  radio.setPayloadSize(sizeof(receiver_byte));
  radio.setRetries(0, 0);
  radio.setAutoAck(false);
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.startListening();

  Serial.println("Setup DONE");
}

void loop() {
  //  test_receive_string();
  //  return;

  if (radio.available()) {
    radio.read(&receiver_byte, sizeof(receiver_byte));
#ifdef DEBUG_PRINT
    Serial.print(data_count);
    Serial.print(": ");
    Serial.println(receiver_byte);
#endif

    if (prev_receiver_byte == 'J' && receiver_byte == 'B') {    //Track start byte
      start_count = true;
      data_count = 3;
      check_byte = 'B';
      //      Serial.println(check_byte);
    } else if (start_count) {                                   //Track subsequent message
      receiver_data[data_count] = receiver_byte;

      if (data_count == LAST_BYTE) {
#ifdef DEBUG_PRINT
        Serial.println("Check: " + (String) receiver_data[data_count] + ", " + (String) check_byte);
#endif
        if (check_byte == receiver_data[data_count]) {
          //Serial.println("UPDate!!" + (String) check_byte);
          allocate_data();
          display_data();
        } else {                                                //Check byte not correct --> Reset data
          data_count = 3;
          check_byte = 'B';
        }
        start_count = false;
      } else check_byte ^= receiver_byte;
      data_count++;
    }
  }
  prev_receiver_byte = receiver_byte;

#ifndef DEBUG_PRINT
//  display_data();
#endif
}

void display_data() {
  Serial.print(roll);
  Serial.print(" | ");
  Serial.print(pitch);
  Serial.print(" | ");
  Serial.print(heading);
  Serial.print(" | ");
  Serial.print(latitude);
  Serial.print(" | ");
  Serial.print(longitude);
  Serial.print(" | ");
  Serial.print(channel_1);
  Serial.print(" | ");
  Serial.print(channel_2);
  Serial.print(" | ");
  Serial.print(channel_3);
  Serial.print(" | ");
  Serial.print(channel_4);
  Serial.print(" | ");
  Serial.print(loop_time);
  Serial.println(" | ");
  
  if (pid_mode == 1) Serial.print(">");
  Serial.print(pid_p_roll);
  Serial.print(" | ");
  if (pid_mode == 2) Serial.print(">");
  Serial.print(pid_i_roll, 3);
  Serial.print(" | ");
  if (pid_mode == 3) Serial.print(">");
  Serial.print(pid_d_roll);
  Serial.print(" | ");

  if (pid_mode == 4) Serial.print(">");
  Serial.print(pid_p_pitch);
  Serial.print(" | ");
  if (pid_mode == 5) Serial.print(">");
  Serial.print(pid_i_pitch, 3);
  Serial.print(" | ");
  if (pid_mode == 6) Serial.print(">");
  Serial.print(pid_d_pitch);
  Serial.print(" | ");

  if (pid_mode == 7) Serial.print(">");
  Serial.print(pid_p_yaw);
  Serial.print(" | ");
  if (pid_mode == 8) Serial.print(">");
  Serial.print(pid_i_yaw, 3);
  Serial.print(" | ");
  if (pid_mode == 9) Serial.print(">");
  Serial.print(pid_d_yaw);
  Serial.println(" | ");

  if (prev_p_roll != pid_p_roll) pid_mode = 1;
  if (prev_i_roll != pid_i_roll) pid_mode = 2;
  if (prev_d_roll != pid_d_roll) pid_mode = 3;

  if (prev_p_pitch != pid_p_pitch) pid_mode = 4;
  if (prev_i_pitch != pid_i_pitch) pid_mode = 5;
  if (prev_d_pitch != pid_d_pitch) pid_mode = 6;

  if (prev_p_yaw != pid_p_yaw) pid_mode = 7;
  if (prev_i_yaw != pid_i_yaw) pid_mode = 8;
  if (prev_d_yaw != pid_d_yaw) pid_mode = 9;

  prev_p_roll = pid_p_roll; prev_i_roll = pid_i_roll; prev_d_roll = pid_d_roll; 
  prev_p_pitch = pid_p_pitch; prev_i_pitch = pid_i_pitch; prev_d_pitch = pid_d_pitch;
  prev_p_yaw = pid_p_yaw; prev_i_yaw = pid_i_yaw; prev_d_yaw = pid_d_yaw;
}

void allocate_data() {
  roll = receiver_data[3] - 100;
  pitch = receiver_data[4] - 100;
  heading = (uint16_t) receiver_data[5] | (uint16_t) receiver_data[6] << 8;
  latitude = (int32_t) receiver_data[7] | (int32_t) receiver_data[8] << 8 | (int32_t) receiver_data[9] << 16 | (int32_t) receiver_data[10] << 24;
  longitude = (int32_t) receiver_data[11] | (int32_t) receiver_data[12] << 8 | (int32_t) receiver_data[13] << 16 | (int32_t) receiver_data[14] << 24;
  channel_1 = (uint16_t) receiver_data[15] | (uint16_t) receiver_data[16] << 8;
  channel_2 = (uint16_t) receiver_data[17] | (uint16_t) receiver_data[18] << 8;
  channel_3 = (uint16_t) receiver_data[19] | (uint16_t) receiver_data[20] << 8;
  channel_4 = (uint16_t) receiver_data[21] | (uint16_t) receiver_data[22] << 8;
  loop_time = (uint16_t) receiver_data[23] | (uint16_t) receiver_data[24] << 8;

  pid_p_roll = receiver_data[25];
  pid_i_roll = receiver_data[26];
  pid_d_roll = receiver_data[27];

  pid_p_pitch = receiver_data[28];
  pid_i_pitch = receiver_data[29];
  pid_d_pitch = receiver_data[30];

  pid_p_yaw = receiver_data[31];
  pid_i_yaw = receiver_data[32];
  pid_d_yaw = receiver_data[33];

  pid_p_roll /= 10;
  pid_i_roll /= 1000;
  pid_d_roll /= 10;

  pid_p_pitch /= 10;
  pid_i_pitch /= 1000;
  pid_d_pitch /= 10;

  pid_p_yaw /= 10;
  pid_i_yaw /= 1000;
  pid_d_yaw /= 10;
}

void test_receive_string() {
  if (radio.available()) {
    char data[32] = "";
    radio.read(&data, sizeof(data));
    Serial.println(data);
  }
}
