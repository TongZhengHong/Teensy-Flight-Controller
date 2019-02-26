
void initialise_telemetry() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void send_telemetry_data() {
  telemetry_loop_counter++;
  if (telemetry_loop_counter == 1) telemetry_data = 'J';
  else if (telemetry_loop_counter == 2) telemetry_data = 'B';
  else if (telemetry_loop_counter == 3) telemetry_data = roll + 100;      //Adding 100 prevents negative numbers.
  else if (telemetry_loop_counter == 4) telemetry_data = pitch + 100;     //Adding 100 prevents negative numbers.

  //After 125 loops the telemetry_loop_counter variable is reset. This way the telemetry data is send every half second.
  if (telemetry_loop_counter == 125) telemetry_loop_counter = 0;

  if (telemetry_loop_counter <= 4) {
    radio.write(&telemetry_data, sizeof(telemetry_data));
  }
}
