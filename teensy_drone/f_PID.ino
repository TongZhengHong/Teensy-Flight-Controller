void set_pid_offsets() {
  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_3 > 1050)  {
    if (receiver_input_channel_1 > 1508)
      pid_roll_setpoint = receiver_input_channel_1 - 1508;
    else if (receiver_input_channel_1 < 1492)
      pid_roll_setpoint = receiver_input_channel_1 - 1492;

    pid_roll_setpoint -= roll_level_adjust; //Subtract the angle correction from the standardized receiver roll input value.
    pid_roll_setpoint /= 3.0;               //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.
  }

  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_3 > 1050)  {
    if (receiver_input_channel_2 > 1508)
      pid_pitch_setpoint = 1508 - receiver_input_channel_2;
    else if (receiver_input_channel_2 < 1492)
      pid_pitch_setpoint = 1492 - receiver_input_channel_2;

    pid_pitch_setpoint -= pitch_level_adjust; //Subtract the angle correction from the standardized receiver pitch input value.
    pid_pitch_setpoint /= 3.0;                //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.
  }

  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_3 > 1050)  {                               
  //Do not yaw when turning off the motors.
    if (receiver_input_channel_4 > 1508) {
      pid_yaw_setpoint = receiver_input_channel_4 - 1508;
      pid_yaw_setpoint /= 3.0;                                          //Max = 500/3 degrees per second

#ifdef HEADING_HOLD_MODE
      prev_heading = heading;                                           //Record the last heading
#endif

    } else if (receiver_input_channel_4 < 1492) {
      pid_yaw_setpoint = receiver_input_channel_4 - 1492;
      pid_yaw_setpoint /= 3.0;                                          //Max = 500/3 degrees per second

#ifdef HEADING_HOLD_MODE
      prev_heading = heading;                                           //Record the last heading
#endif
    } else {                                                            //Yaw stick at center --> No transmitter input
#ifdef HEADING_HOLD_MODE
      pid_yaw_setpoint = prev_heading;
#endif
    }
  }

#ifndef DEBUG_PID_SETPOINT
  Serial.print(pid_roll_setpoint);
  Serial.print(" ");
  Serial.print(pid_pitch_setpoint);
  Serial.print(" ");
  Serial.print(pid_yaw_setpoint);
  Serial.println();
#endif
}

void calculate_pid() {
  //* ========================================= Roll calculations =========================================
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > pid_max_roll) pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < pid_max_roll * -1) pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll) pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1) pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //* ========================================= Pitch calculations =========================================
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch) pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1) pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1) pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //* ========================================= Yaw calculations =========================================
#ifdef HEADING_HOLD_MODE
  float heading_error = heading - pid_yaw_setpoint;
  if (abs(heading_error) > 180) {
    if (heading_error > 0) {
      heading_error -= 360;
    } else {
      heading_error += 360;
    }
  }
  pid_error_temp = heading_error;
#endif

#ifndef HEADING_HOLD_MODE
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
#endif

  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw) pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1) pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1) pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;

#ifndef DEBUG_PID_OUTPUT
  Serial.print(" ");
  Serial.print(pid_output_roll);
  Serial.print(", ");
  Serial.print(pid_output_pitch);
  Serial.print(", ");
  Serial.print(pid_output_yaw);
  Serial.println();
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void set_pid_offsets_angles() {
  pid_roll_setpoint = 0;
  if (receiver_input_channel_3 > 1050)  {
    if (receiver_input_channel_1 > 1508)
      pid_roll_setpoint = receiver_input_channel_1 - 1508;
    else if (receiver_input_channel_1 < 1492)
      pid_roll_setpoint = receiver_input_channel_1 - 1492;

    pid_roll_setpoint /= 10.0;                                //Max angle: 500/10 = 50 degrees
  }

  pid_pitch_setpoint = 0;
  if (receiver_input_channel_3 > 1050)  {
    if (receiver_input_channel_2 > 1508)
      pid_pitch_setpoint = 1508 - receiver_input_channel_2;
    else if (receiver_input_channel_2 < 1492)
      pid_pitch_setpoint = 1492 - receiver_input_channel_2;

    pid_pitch_setpoint /= 10.0;                               //Max angle: 500/10 = 50 degrees
  }

  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_3 > 1050)  { //Do not yaw when turning off the motors.
    if (receiver_input_channel_4 > 1508)
      pid_yaw_setpoint = receiver_input_channel_4 - 1508;
    else if (receiver_input_channel_4 < 1492)
      pid_yaw_setpoint = receiver_input_channel_4 - 1492;

    pid_yaw_setpoint /= 3.0;                                  //Max yaw rate: 500/3 degrees per second
  }

#ifndef DEBUG_PID_SETPOINT
  Serial.print(pid_roll_setpoint);
  Serial.print(" ");
  Serial.print(pid_pitch_setpoint);
  Serial.print(" ");
  Serial.print(pid_yaw_setpoint);
  Serial.println();
#endif
}

void calculate_pid_angles() {
  //* ========================================= Roll calculations =========================================
  pid_error_temp = angle_roll - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > pid_max_roll) pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < pid_max_roll * -1) pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll) pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1) pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //* ========================================= Pitch calculations =========================================
  pid_error_temp = angle_pitch - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch) pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1) pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch) pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1) pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //* ========================================= Yaw calculations =========================================
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw) pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1) pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw) pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1) pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;

#ifndef DEBUG_PID_OUTPUT
  Serial.print(" ");
  Serial.print(pid_output_roll);
  Serial.print(", ");
  Serial.print(pid_output_pitch);
  Serial.print(", ");
  Serial.print(pid_output_yaw);
  Serial.println();
#endif
}

void testing(){
  pid_roll_setpoint = 0;
  if (receiver_input_channel_3 > 1050)  {
    if (receiver_input_channel_1 > 1508)
      pid_roll_setpoint = receiver_input_channel_1 - 1508;
    else if (receiver_input_channel_1 < 1492)
      pid_roll_setpoint = receiver_input_channel_1 - 1492;

    pid_roll_setpoint /= 10.0;                                //Max roll angle: 500/10 = 50 degrees
  }

  float angle_error = angle_roll - pid_roll_setpoint;
  //pid_roll_setpoint = p_gain * angle_error + 

  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_output_roll = pid_p_gain_roll * pid_error_temp;

  pid_pitch_setpoint = 0;
  if (receiver_input_channel_3 > 1050)  {
    if (receiver_input_channel_2 > 1508)
      pid_pitch_setpoint = 1508 - receiver_input_channel_2;
    else if (receiver_input_channel_2 < 1492)
      pid_pitch_setpoint = 1492 - receiver_input_channel_2;

    pid_pitch_setpoint /= 10.0;                               //Max pitchangle: 500/10 = 50 degrees
  }
  
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp;

  //* ========================================= Yaw calculations =========================================
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_3 > 1050)  { //Do not yaw when turning off the motors.
    if (receiver_input_channel_4 > 1508)
      pid_yaw_setpoint = receiver_input_channel_4 - 1508;
    else if (receiver_input_channel_4 < 1492)
      pid_yaw_setpoint = receiver_input_channel_4 - 1492;

    pid_yaw_setpoint /= 3.0;                                  //Max yaw rate: 500/3 degrees per second
  }

  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw) pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1) pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw) pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1) pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}
