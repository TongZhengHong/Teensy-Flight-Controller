uint8_t check_byte;

void initialise_telemetry() {
  radio.begin();
  radio.setPayloadSize(sizeof(telemetry_data));
  radio.setRetries(0, 0);
  //radio.setAutoAck(false); --> Test!!!
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.stopListening();
}

void test_telemetry() {
  byte testing = 100;
  radio.write(&testing, sizeof(testing));
}

void send_telemetry_data() {
  telemetry_loop_counter++;
  if (telemetry_loop_counter == 1) telemetry_data = 'J';
  else if (telemetry_loop_counter == 2) {
    check_byte = 0;
    telemetry_data = 'B';
  }

  // Send ROLL and PITCH angles ======================================================================================
  else if (telemetry_loop_counter == 3) telemetry_data = roll + 100;      //Adding 100 prevents negative numbers.
  else if (telemetry_loop_counter == 4) telemetry_data = pitch + 100;     //Adding 100 prevents negative numbers.

  // Send HEADING  ===================================================================================================
  else if (telemetry_loop_counter == 5) {
    telemetry_temp = heading;
    telemetry_data = telemetry_temp;
  }
  else if (telemetry_loop_counter == 6) telemetry_data = telemetry_temp >> 8;

  // Send Latitude ===================================================================================================
  if (telemetry_loop_counter == 7) {
    telemetry_temp = gps_lat;
    telemetry_data = telemetry_temp;
  }
  if (telemetry_loop_counter == 8) telemetry_data = telemetry_temp >> 8;
  if (telemetry_loop_counter == 9) telemetry_data = telemetry_temp >> 16;
  if (telemetry_loop_counter == 10) telemetry_data = telemetry_temp >> 24;

  // Send Longitude ===================================================================================================
  if (telemetry_loop_counter == 11) {
    telemetry_temp = gps_lon;
    telemetry_data = telemetry_temp;
  }
  if (telemetry_loop_counter == 12) telemetry_data = telemetry_temp >> 8;
  if (telemetry_loop_counter == 13) telemetry_data = telemetry_temp >> 16;
  if (telemetry_loop_counter == 14) telemetry_data = telemetry_temp >> 24;

  // Send transmitter values ==========================================================================================
  else if (telemetry_loop_counter == 15) {
    telemetry_temp = receiver_input_channel_1;
    telemetry_data = telemetry_temp;
  }
  else if (telemetry_loop_counter == 16) telemetry_data = telemetry_temp >> 8;

  else if (telemetry_loop_counter == 17) {
    telemetry_temp = receiver_input_channel_2;
    telemetry_data = telemetry_temp;
  }
  else if (telemetry_loop_counter == 18) telemetry_data = telemetry_temp >> 8;

  else if (telemetry_loop_counter == 19) {
    telemetry_temp = receiver_input_channel_3;
    telemetry_data = telemetry_temp;
  }
  else if (telemetry_loop_counter == 20) telemetry_data = telemetry_temp >> 8;

  else if (telemetry_loop_counter == 21) {
    telemetry_temp = receiver_input_channel_4;
    telemetry_data = telemetry_temp;
  }
  else if (telemetry_loop_counter == 22) telemetry_data = telemetry_temp >> 8;

  //Send loop time ====================================================================================================
  else if (telemetry_loop_counter == 23) {
    telemetry_temp = difference;
    telemetry_data = telemetry_temp;
  }
  else if (telemetry_loop_counter == 24) telemetry_data = telemetry_temp >> 8;

  //After 125 loops the telemetry_loop_counter variable is reset. This way the telemetry data is send every half second.
  if (telemetry_loop_counter == 100) telemetry_loop_counter = 0;

  if (telemetry_loop_counter < 25) {
#ifndef DEBUG_TELEMETRY
    Serial.println((String) telemetry_loop_counter + ": " + (String) telemetry_data);
#endif
    check_byte ^= telemetry_data;

    radio.write(&telemetry_data, sizeof(telemetry_data));
  } else if (telemetry_loop_counter == 25) {
#ifndef DEBUG_TELEMETRY
    Serial.println("Check: " + (String) check_byte);
#endif
    radio.write(&check_byte, sizeof(check_byte));
  }
}
