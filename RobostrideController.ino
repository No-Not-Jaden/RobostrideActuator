/*
 * Adafruit MCP2515 FeatherWing CAN Receiver Example
 */

#include <Adafruit_MCP2515.h>

#ifdef ESP8266
   #define CS_PIN    2
#elif defined(ESP32) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3)
   #define CS_PIN    14
#elif defined(TEENSYDUINO)
   #define CS_PIN    8
#elif defined(ARDUINO_STM32_FEATHER)
   #define CS_PIN    PC5
#elif defined(ARDUINO_NRF52832_FEATHER)  /* BSP 0.6.5 and higher! */
   #define CS_PIN    27
#elif defined(ARDUINO_MAX32620FTHR) || defined(ARDUINO_MAX32630FTHR)
   #define CS_PIN    P3_2
#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040)
   #define CS_PIN    7
#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_CAN)
   #define CS_PIN    PIN_CAN_CS
#elif defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_RASPBERRY_PI_PICO_W) // PiCowbell CAN Bus
   #define CS_PIN    20
#else
    // Anything else, defaults!
   #define CS_PIN    5
#endif

// Set CAN bus baud rate
#define CAN_BAUDRATE (1000000)

Adafruit_MCP2515 mcp(CS_PIN);

const unsigned long hostId = 240;
bool motorEnabled = false;
unsigned long time_ms;
int stage = 0;

void setup() {
  Serial.begin(1000000);
  while(!Serial) delay(10);

  Serial.println("MCP2515 Receiver test!");

  if (!mcp.begin(CAN_BAUDRATE)) {
    Serial.println("Error initializing MCP2515.");
    while(1) delay(10);
  }
  Serial.println("MCP2515 chip found");
  time_ms = millis();
}

// Function to extract a range of bits from a long
unsigned long extractBits(long value, int start, int end) {
    if (start > end || start < 0 || end >= (sizeof(long) * 8)) {
        Serial.println("Invalid argument");
        return 0;
    }
    
    // Create a mask for the desired range
    unsigned long mask = ((1UL << (end - start + 1)) - 1) << start;

    // Apply the mask and shift the bits to the right
    return (value & mask) >> start;
}

void printBinary(unsigned long number, int bits) {
    for (int i = bits - 1; i >= 0; i--) {
        Serial.print((number >> i) & 1); // Extract and print each bit
    }
}

void enableMotor(long motorId) {
  uint32_t packet = 0b10010000000000000000000000000; // 18 -> set to operation mode
  packet += hostId << 8;
  packet += motorId;
  mcp.beginExtendedPacket(packet);
  mcp.write(0b00000101);
  mcp.write(0b01110000);
  mcp.write(0b00000000);
  mcp.write(0b00000000);
  mcp.write(0b00000000);
  mcp.write(0b00000000);
  mcp.write(0b00000000);
  mcp.write(0b00000000);
  mcp.endPacket();
  
  packet = 0b00011000000000000000000000000; // 3 -> enable motor
  packet += hostId << 8;
  packet += motorId;
  mcp.beginExtendedPacket(packet);
  mcp.write(0b00000000);
  mcp.write(0b00000000);
  mcp.write(0b00000000);
  mcp.write(0b00000000);
  mcp.write(0b00000000);
  mcp.write(0b00000000);
  mcp.write(0b00000000);
  mcp.write(0b00000000);
  mcp.endPacket();

  motorEnabled = true;
}

void disableMotor(long motorId) {
  uint32_t packet = 0b00100000000000000000000000000; // 4 -> disable motor
  packet += hostId << 8;
  packet += motorId;
  mcp.beginExtendedPacket(packet);
  mcp.write(0b00000000);
  mcp.write(0b00000000);
  mcp.write(0b00000000);
  mcp.write(0b00000000);
  mcp.write(0b00000000);
  mcp.write(0b00000000);
  mcp.write(0b00000000);
  mcp.write(0b00000000);
  mcp.endPacket();

  motorEnabled = false;
}

void sendCommand(long motorId, float torque, float angle, float velocity, float kp, float kd) {
  uint32_t packet = 0b00001000000000000000000000000; // 1 -> motor control
  uint16_t torqueData = map(torque * 10, -1200, 1200, 0, 65535);
  uint16_t velocityData = map(velocity * 100, -1500, 1500, 0, 65535);
  uint16_t angleData = map(angle * 1000, -4000 * PI, 4000 * PI, 0, 65535);
  uint16_t kpData = map(kp * 10, 0, 50000, 0, 65535);
  uint16_t kdData = map(kd * 100, 0, 10000, 0, 63353);
  packet += torqueData << 8;
  packet += motorId;
  Serial.print("ID: ");
  printBinary(packet, 29);
  Serial.println();
  printBinary(angleData >> 8, 8);
  printBinary(angleData, 8);
  printBinary(velocityData >> 8, 8);
  printBinary(velocityData, 8);
  printBinary(kpData >> 8, 8);
  printBinary(kpData, 8);
  printBinary(kdData >> 8, 8);
  printBinary(kdData, 8);
  Serial.println();
  mcp.beginExtendedPacket(packet);
  mcp.write(angleData >> 8);
  mcp.write(angleData);
  mcp.write(velocityData >> 8);
  mcp.write(velocityData);
  mcp.write(kpData >> 8);
  mcp.write(kpData);
  mcp.write(kdData >> 8);
  mcp.write(kdData);
  mcp.endPacket();
}

uint8_t bytes[8];

// get the current angle of the motor. Returns -4pi to 4pi
float getCurrentAngle() {
  long angleData = (bytes[0] << 8) + bytes[1];
  return map(angleData, 0, 65535, -4000 * PI, 4000 * PI) / 1000.0;
}

float getAngularVelocity() {
  long angleData = (bytes[2] << 8) + bytes[3];
  return map(angleData, 0, 65535, -1500, 1500) / 100.0;
}

float getTorque() {
  long torqueData = (bytes[4] << 8) + bytes[5];
  return map(torqueData, 0, 65535, -1200, 1200) / 10.0;
}

float getTemperature() {
  long tempData = (bytes[6] << 8) + bytes[7];
  return tempData / 10.0;
}

void loop() {

  if (true && millis() - time_ms > 5000) {
    Serial.print("Stage: "); Serial.println(stage);
    if (stage == 0) {
      enableMotor(127);
    } else if (stage == 1) {
      sendCommand(127, 0, 0, 0, 5, 1);
    } else if (stage == 2) {
      sendCommand(127, 0, 3.14, 0, 5, 1);
    } else if (stage == 3) {
      sendCommand(127, 0, 0, 0, 5, 1);
    } else if (stage == 4) {
      disableMotor(127);
    }

    stage++;
    if (stage == 5) {
      stage = 0;
    }
    time_ms += 5000;
  }

  // try to parse packet
  int packetSize = mcp.parsePacket();
  
  if (packetSize) {
    // received a packet
    Serial.print("Received Packet. ");
    printBinary(mcp.packetId(), 29); Serial.println();
    if (mcp.packetExtended()) {
      long mode = extractBits(mcp.packetId(), 24, 28);
      long id = extractBits(mcp.packetId(), 0, 7);
      long data = extractBits(mcp.packetId(), 8, 23);
      Serial.print("Communication type: "); Serial.println(mode);
      if (mode == 2) { //id == hostId
        uint8_t motorID = extractBits(data, 0, 7);
        Serial.print("Receieved Motor Feedback from ID: "); Serial.println(motorID);
        Serial.print("Faults: "); printBinary(extractBits(mcp.packetId(), 16, 21), 6); Serial.println();
        Serial.print("Mode status: "); Serial.println(extractBits(mcp.packetId(), 22, 23));
        int i = 0;
        while (mcp.available()) {
          uint8_t d = mcp.read();
          //printBinary(d, 8);
          if (i < 8) {
            bytes[i] = d;
            i++;
          }
        }
        Serial.print("Angle: "); Serial.print(getCurrentAngle()); Serial.print(" rad   ");
        Serial.print("Velocity: "); Serial.print(getAngularVelocity()); Serial.print(" rad/s   ");
        Serial.print("Torque: "); Serial.print(getTorque()); Serial.print(" Nm   ");
        Serial.print("Temp: "); Serial.print(getTemperature()); Serial.println(" C");
      } else if (mode == 21) {
        // fault feedback
        Serial.print("Motor Fault from ID: "); Serial.println(id);
        while (mcp.available()) {
          uint8_t d = mcp.read();
          printBinary(d, 8);
          
        }
        disableMotor(id);
      } else {
        while (mcp.available()) {
          uint8_t d = mcp.read();
          printBinary(d, 8);
          
        }
      }
      Serial.println();
      Serial.println();
    }

  }
}