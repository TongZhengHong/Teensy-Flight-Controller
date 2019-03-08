#include <EEPROM.h>

byte eeprom_data[75];
uint16_t throttle;
volatile int receiver_input;
byte last_channel_3;
uint32_t timer_3, current_time;

void setup() {
  Serial.begin(115200);

  for (int start = 0; start <= 74; start++)
    eeprom_data[start] = EEPROM.read(start);
  while (eeprom_data[72] != 'J' || eeprom_data[73] != 'M' || eeprom_data[74] != 'B')
    delay(10);

  attachInterrupt(digitalPinToInterrupt(14), receiver_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(15), receiver_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(16), receiver_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(17), receiver_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(22), receiver_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(23), receiver_change, CHANGE);

  PORTD_PCR2 = (1 << 8); //configuring pin 7 as GPIO
  PORTD_PCR3 = (1 << 8); //configuring pin 8 as GPIO
  PORTD_PCR4 = (1 << 8); //configuring pin 6 as GPIO
  PORTD_PCR7 = (1 << 8); //configuring pin 5 as GPIO
  PORTD_PCR5 = (1 << 8); //configuring pin 20 as GPIO
  PORTD_PCR6 = (1 << 8); //configuring pin 21 as GPIO
  GPIOD_PDDR |= 252; 
}

void loop() {
  throttle = convert_receiver_channel(3); //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
  Serial.println(throttle);

  GPIOD_PSOR |= 252;    //0000 0000 0000 0000 0000 0000 1111 1100 --> Setting pins 5,6,7,8,20,21 as HIGH
  delayMicroseconds(throttle);
  GPIOD_PCOR |= 252;    //0000 0000 0000 0000 0000 0000 1111 1100 --> Setting pins 5,6,7,8,20,21 as LOW
  delay(3);
}

int convert_receiver_channel(byte function) {
  int low, center, high, actual;
  int difference;

  actual = receiver_input;                                             //Read the actual receiver value for the corresponding function
  low = (eeprom_data[function * 2 + 15] << 8) | eeprom_data[function * 2 + 14];  //Store the low value for the specific receiver input channel
  center = (eeprom_data[function * 2 - 1] << 8) | eeprom_data[function * 2 - 2]; //Store the center value for the specific receiver input channel
  high = (eeprom_data[function * 2 + 7] << 8) | eeprom_data[function * 2 + 6];   //Store the high value for the specific receiver input channel

  if (actual < center)  { //The actual receiver value is lower than the center value
    if (actual < low)
      actual = low;                                                      //Limit the lowest value to the value that was detected during setup
    difference = ((long)(center - actual) * (long)500) / (center - low); //Calculate and scale the actual value to a 1000 - 2000us value
    return 1500 - difference;
  }
  else if (actual > center)  { //The actual receiver value is higher than the center value
    if (actual > high)
      actual = high;                                                      //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center) * (long)500) / (high - center); //Calculate and scale the actual value to a 1000 - 2000us value
    return 1500 + difference;
  }
  else
    return 1500;
}

void receiver_change() {
  current_time = micros();
  //* ========================================= Channel 3 =========================================
  if (GPIOB_PDIR & 1) { //0000 0000 0000 0000 0000 0000 0000 0001 --> PTB0
    if (last_channel_3 == 0) {                                              //Input 16 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  } else if (last_channel_3 == 1) {                                         //Input 16 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    receiver_input = current_time - timer_3;                                //Channel 3 is current_time - timer_3.
  }
}
