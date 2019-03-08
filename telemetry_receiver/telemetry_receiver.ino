#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define LAST_BYTE 25

#define CE_PIN 7
#define CSN_PIN 8

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

bool start_count = false;
uint8_t check_byte;
uint8_t prev_receiver_byte, receiver_byte;
uint8_t data_count, receiver_data[50];

//Data
int8_t roll, pitch;
uint16_t heading;
int32_t latitude, longitude;
uint16_t channel_1, channel_2, channel_3, channel_4;
uint16_t loop_time;

void setup() {
  Serial.begin(115200);

  radio.begin();
  radio.setPayloadSize(sizeof(receiver_byte));
  radio.setRetries(0,0);
  //radio.setAutoAck(false);
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
    Serial.println(receiver_byte);

    if (prev_receiver_byte == 'J' && receiver_byte == 'B') {    //Track start byte
      start_count = true;
      data_count = 3;
      check_byte = 'B';
      //      Serial.println(check_byte);
    } else if (start_count) {                                   //Track subsequent message
      receiver_data[data_count] = receiver_byte;

      if (data_count == LAST_BYTE) {
//        Serial.println("Check: " + (String) receiver_data[data_count] + ", " + (String) check_byte);
        if (check_byte == receiver_data[data_count]) {
          allocate_data();
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

  //  display_data();
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
}

void test_receive_string() {
  if (radio.available()) {
    char data[32] = "";
    radio.read(&data, sizeof(data));
    Serial.println(data);
  }
}
