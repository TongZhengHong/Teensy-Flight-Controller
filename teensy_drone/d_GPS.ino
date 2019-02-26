const unsigned char UBX_HEADER[] = {0xB5, 0x62};

const char UBLOX_INIT[] PROGMEM = {
  // Disable NMEA
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24, // GxGGA off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B, // GxGLL off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32, // GxGSA off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39, // GxGSV off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40, // GxRMC off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47, // GxVTG off

  // Disable UBX
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xDC, //NAV-PVT off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xB9, //NAV-POSLLH off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0xC0, //NAV-STATUS off

  // Enable UBX
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xE1, //NAV-PVT on
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, //NAV-POSLLH on
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x14,0xC5, //NAV-STATUS on

  // Rate
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12, //(10Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //(1Hz)
};

struct NAV_PVT {
  uint8_t cls;
  uint8_t id;
  uint16_t len;
  uint32_t iTOW;          // GPS time of week of the navigation epoch (ms)

  uint16_t year;         // Year (UTC)
  uint8_t month;         // Month, range 1..12 (UTC)
  uint8_t day;           // Day of month, range 1..31 (UTC)
  uint8_t hour;          // Hour of day, range 0..23 (UTC)
  uint8_t minute;        // Minute of hour, range 0..59 (UTC)
  uint8_t second;        // Seconds of minute, range 0..60 (UTC)

  int8_t valid;          // Validity Flags (see graphic below)
  uint32_t tAcc;         // Time accuracy estimate (UTC) (ns)
  int32_t nano;          // Fraction of second, range -1e9 .. 1e9 (UTC) (ns)
  uint8_t fixType;       // GNSSfix Type, range 0..5
  int8_t flags;          // Fix Status Flags
  int8_t flags2;
  uint8_t numSV;         // Number of satellites used in Nav Solution

  int32_t lon;           // Longitude (deg)
  int32_t lat;           // Latitude (deg)
  int32_t height;        // Height above Ellipsoid (mm)
  int32_t hMSL;          // Height above mean sea level (mm)
  uint32_t hAcc;         // Horizontal Accuracy Estimate (mm)
  uint32_t vAcc;         // Vertical Accuracy Estimate (mm)

  int32_t velN;          // NED north velocity (mm/s)
  int32_t velE;          // NED east velocity (mm/s)
  int32_t velD;          // NED down velocity (mm/s)
  int32_t gSpeed;        // Ground Speed (2-D) (mm/s)
  int32_t headingMotion; // Heading of motion 2-D (deg)
  uint32_t sAcc;         // Speed Accuracy Estimate
  uint32_t headingAcc;   // Heading Accuracy Estimate
  uint16_t pDOP;         // Position dilution of precision

  uint8_t reserved1;
  uint8_t reserved2;
  uint8_t reserved3;
  uint8_t reserved4;
  uint8_t reserved5;
  uint8_t reserved6;

  int32_t headingVehicle;
  int16_t magDec;
  uint16_t magAcc;
};

NAV_PVT pvt;
uint8_t count = 0;

void readGPS() {
  static unsigned char checksum[2];
  const int payloadSize = sizeof(NAV_PVT);

  while (Serial1.available() > 0) {
    byte data = Serial1.read();
    if (count < 2) {                                      //Track the header
      if (data == UBX_HEADER[count]) count++;
      else count = 0;
    } else {                                              //Reading the subsequent information
      if ((count - 2) < payloadSize)
        ((unsigned char*)(&pvt))[count - 2] = data;

      count++;
      if (count == (payloadSize + 2)) {
        calculate_checksum(checksum);
      } else if (count == (payloadSize + 3)) {
        if (data != checksum[0]) count = 0;
      } else if (count == (payloadSize + 4)) {
        count = 0;
        if (data == checksum[1]) {                        //Data package is correct!
#ifndef DEBUG_GPS
          Serial.print("#SV: ");      Serial.print(pvt.numSV);
          Serial.print(" fixType: "); Serial.print(pvt.fixType);
          Serial.print(" Latitude: "); Serial.print(pvt.lat / 10000000.0f, 6);
          Serial.print(" Longitutde: "); Serial.print(pvt.lon / 10000000.0f, 6);
          Serial.println();
#endif
        }
      } else if (count > (payloadSize + 4)) {
        count = 0;
      }
    }
  }
}

void calculate_checksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_PVT); i++) {
    CK[0] += ((unsigned char*)(&pvt))[i];
    CK[1] += CK[0];
  }
}

void setup_gps() {
  // Send configuration data to GPS with UBX protocol
  for (uint16_t i = 0; i < sizeof(UBLOX_INIT); i++) {
    Serial1.write(pgm_read_byte(UBLOX_INIT + i));
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
}
