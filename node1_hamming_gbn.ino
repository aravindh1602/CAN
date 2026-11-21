#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <mcp2515.h>
#include <math.h>
#include <string.h>

// ==========================================================
// HARDWARE
// ==========================================================
#define MPU6050_SDA 21
#define MPU6050_SCL 22
#define MCP2515_CS 5

Adafruit_MPU6050 mpu;
MCP2515 mcp2515(MCP2515_CS);

struct can_frame canMsg;

// ==========================================================
// EXISTING CAN IDs
// ==========================================================
#define ACCEL_CAN_ID 0x036
#define GYRO_CAN_ID  0x035
#define CAN_ACK_ID   0x037

// ==========================================================
// GBN CONTROL IDs
// ==========================================================
#define MODE_CMD_ID     0x050
#define GBN_TRIGGER_ID  0x051
#define HEARTBEAT_ID    0x052
#define GBN_CUM_ACK_ID  0x053

// ==========================================================
// GBN TYPES
// ==========================================================
#define TYPE_ACC   0
#define TYPE_GYRO  1

// ==========================================================
// STOP-WAIT SETTINGS
// ==========================================================
#define MAX_RETRIES 3
#define ACK_TIMEOUT 500

// ==========================================================
// HEARTBEAT
// ==========================================================
#define HEARTBEAT_INTERVAL 500
unsigned long lastHeartbeat = 0;

// ==========================================================
// MODE
// ==========================================================
bool gbnMode = false;

// ==========================================================
// ERROR INJECTION
// ==========================================================
bool errorInjection = false;
int errorByte = -1;
bool checksumError = false;
bool packetLoss = false;

// ==========================================================
// XOR
// ==========================================================
byte calculateXOR(byte *data, byte length)
{
  byte checksum = 0;

  for (byte i = 0; i < length; i++)
  {
    checksum ^= data[i];
  }

  return checksum;
}

// ==========================================================
// GBN ONLY: SHORTENED HAMMING(46,40) + 5-BYTE PAYLOAD
// 40 data bits + 6 Hamming parity bits = 46-bit codeword.
// Parity byte uses bits 0..5; bits 6..7 are zero.
// ==========================================================
const byte HAMMING_PARITY_POS[6] = {1, 2, 4, 8, 16, 32};

bool isParityPosition(byte position)
{
  return position == 1 || position == 2 || position == 4 ||
         position == 8 || position == 16 || position == 32;
}

bool getPayloadBit(const byte payload[5], byte bitIndex)
{
  return (payload[bitIndex / 8] >> (7 - (bitIndex % 8))) & 0x01;
}

void setPayloadBit(byte payload[5], byte bitIndex, bool value)
{
  byte mask = (byte)(1U << (7 - (bitIndex % 8)));
  if (value) payload[bitIndex / 8] |= mask;
  else       payload[bitIndex / 8] &= (byte)~mask;
}

byte hammingEncode46_40(const byte payload[5])
{
  byte codeword[47] = {0}; // positions 1..46
  byte dataBit = 0;

  for (byte pos = 1; pos <= 46; pos++)
  {
    if (!isParityPosition(pos))
      codeword[pos] = getPayloadBit(payload, dataBit++);
  }

  byte parityByte = 0;
  for (byte p = 0; p < 6; p++)
  {
    byte parity = 0;
    byte parityPosition = HAMMING_PARITY_POS[p];

    for (byte pos = 1; pos <= 46; pos++)
    {
      if ((pos & parityPosition) && pos != parityPosition)
        parity ^= codeword[pos];
    }

    codeword[parityPosition] = parity;
    if (parity) parityByte |= (byte)(1U << p);
  }

  return parityByte;
}

// Corrects one flipped bit in the 46-bit Hamming codeword.
// Returns false only if the syndrome points outside the shortened codeword.
bool hammingDecode46_40(byte payload[5], byte &parityByte,
                        byte &syndrome, bool &corrected)
{
  byte codeword[47] = {0};
  byte dataBit = 0;

  for (byte pos = 1; pos <= 46; pos++)
  {
    if (isParityPosition(pos))
    {
      byte p = 0;
      while (p < 6 && HAMMING_PARITY_POS[p] != pos) p++;
      codeword[pos] = (parityByte >> p) & 0x01;
    }
    else
    {
      codeword[pos] = getPayloadBit(payload, dataBit++);
    }
  }

  syndrome = 0;
  for (byte pos = 1; pos <= 46; pos++)
  {
    if (codeword[pos]) syndrome ^= pos;
  }

  corrected = false;
  if (syndrome == 0) return true;
  if (syndrome > 46) return false;

  // Flip the indicated codeword bit locally.
  codeword[syndrome] ^= 1;
  corrected = true;

  // Rebuild the five-byte data payload from corrected codeword bits.
  memset(payload, 0, 5);
  dataBit = 0;
  for (byte pos = 1; pos <= 46; pos++)
  {
    if (!isParityPosition(pos))
      setPayloadBit(payload, dataBit++, codeword[pos]);
  }

  // Keep the parity byte consistent too (useful for diagnostics).
  parityByte = 0;
  for (byte p = 0; p < 6; p++)
    if (codeword[HAMMING_PARITY_POS[p]])
      parityByte |= (byte)(1U << p);

  return true;
}

void printHexByte(byte value)
{
  if (value < 0x10) Serial.print('0');
  Serial.print(value, HEX);
}

void printHexArray(const byte *data, byte length)
{
  for (byte i = 0; i < length; i++)
  {
    if (i) Serial.print(' ');
    printHexByte(data[i]);
  }
  Serial.println();
}

// ==========================================================
// SERIAL COMMANDS
// ==========================================================
void processSerialCommand()
{
  if (!Serial.available())
    return;

  String command = Serial.readStringUntil('\n');
  command.trim();

  if (command.startsWith("ERR BYTE "))
  {
    int n = command.substring(9).toInt();
    if (n >= 0 && n <= (gbnMode ? 4 : 5))
    {
      errorInjection = true;
      errorByte = n;
      checksumError = false;
      packetLoss = false;

      if (gbnMode)
      {
        Serial.print("ERROR INJECTION: DATA BYTE ");
        Serial.println(n);
        Serial.println("Will affect NEXT ORIGINAL packet only");
      }
      else
      {
        Serial.print("[INJECT] Data byte ");
        Serial.print(n);
        Serial.println(" will be corrupted on the first attempt.");
      }
    }
    else
    {
      Serial.println(gbnMode ? "Use ERR BYTE 0 to ERR BYTE 4 in GBN" : "Use ERR BYTE 0 to ERR BYTE 5");
    }
  }
  else if (command.equalsIgnoreCase("ERR CHECKSUM"))
  {
    errorInjection = true;
    errorByte = -1;
    checksumError = true;
    packetLoss = false;

    if (gbnMode)
    {
      Serial.println("ERROR INJECTION: CHECKSUM");
      Serial.println("Will affect NEXT ORIGINAL packet only");
    }
    else
    {
      Serial.println("[INJECT] Checksum error armed for next packet.");
    }
  }
  else if (command.equalsIgnoreCase("DROP"))
  {
    // KEEP PACKET DROP BEHAVIOR/PRINTS UNCHANGED
    errorInjection = true;
    errorByte = -1;
    checksumError = false;
    packetLoss = true;

    Serial.println("ERROR INJECTION: PACKET LOSS");
    Serial.println("NEXT ORIGINAL packet will be dropped");
  }
  else if (command.equalsIgnoreCase("ERR OFF"))
  {
    errorInjection = false;
    errorByte = -1;
    checksumError = false;
    packetLoss = false;
    Serial.println("ERROR INJECTION: OFF");
  }
  else if (command.equalsIgnoreCase("STATUS"))
  {
    Serial.println();
    Serial.println("----- ERROR STATUS -----");
    if (!errorInjection)
      Serial.println("OFF");
    else if (packetLoss)
      Serial.println("DROP - NEXT ORIGINAL PACKET");
    else if (checksumError)
      Serial.println("CHECKSUM ERROR - NEXT ORIGINAL PACKET");
    else {
      Serial.print("DATA BYTE ERROR - BYTE ");
      Serial.println(errorByte);
    }
  }
}

// ==========================================================
// HEARTBEAT// ==========================================================
// HEARTBEAT
// ==========================================================
void sendHeartbeat()
{
  if (
    millis() - lastHeartbeat <
    HEARTBEAT_INTERVAL
  )
    return;

  lastHeartbeat = millis();

  struct can_frame hb;

  hb.can_id = HEARTBEAT_ID;
  hb.can_dlc = 1;
  hb.data[0] = 1;

  mcp2515.sendMessage(&hb);
}

// ==========================================================
// APPLY ONE-SHOT ERROR TO GBN FRAME
// ==========================================================
bool applyGBNFault(struct can_frame &f)
{
  if (!errorInjection) return true;

  if (packetLoss)
  {
    Serial.println("FAULT: PACKET LOSS - CAN FRAME NOT SENT");
    errorInjection = false;
    packetLoss = false;
    return false;
  }

  if (errorByte >= 0 && errorByte <= 4)
  {
    byte index = 1 + errorByte;
    Serial.print("FAULT: DATA BYTE "); Serial.println(errorByte);
    Serial.print("BEFORE : 0x"); printHexByte(f.data[index]);
    f.data[index] ^= 0x01;
    Serial.print(" | AFTER : 0x"); printHexByte(f.data[index]); Serial.println();
    errorInjection = false;
    errorByte = -1;
    return true;
  }

  if (checksumError)
  {
    Serial.println("FAULT: XOR CHECKSUM BYTE");
    Serial.print("BEFORE : 0x"); printHexByte(f.data[7]);
    f.data[7] ^= 0x01;
    Serial.print(" | AFTER : 0x"); printHexByte(f.data[7]); Serial.println();
    errorInjection = false;
    checksumError = false;
    return true;
  }

  return true;
}

// ==========================================================
// BUILD + SEND GBN PACKET
// ==========================================================
// Cache the last three original packets so GBN retransmits identical data.
struct can_frame node1GBNCache[3];
bool node1GBNCacheValid[3] = {false, false, false};
byte node1GBNCacheSeq[3] = {0, 0, 0};

void packThreeSigned13(int16_t x, int16_t y, int16_t z, byte payload[5])
{
  memset(payload, 0, 5);
  int16_t values[3] = {x, y, z};
  byte bitIndex = 0;

  for (byte axis = 0; axis < 3; axis++)
  {
    uint16_t encoded = ((uint16_t)values[axis]) & 0x1FFF;
    for (int8_t bit = 12; bit >= 0; bit--)
    {
      if ((encoded >> bit) & 1) setPayloadBit(payload, bitIndex, true);
      bitIndex++;
    }
  }
  // 39 bits used; final payload bit remains zero.
}

void printNode1GBNReadings(byte type, float x, float y, float z)
{
  if (type == TYPE_ACC)
  {
    Serial.println("[IMU ACCELEROMETER]");
    Serial.print("AX : "); Serial.print(x, 2); Serial.println(" m/s^2");
    Serial.print("AY : "); Serial.print(y, 2); Serial.println(" m/s^2");
    Serial.print("AZ : "); Serial.print(z, 2); Serial.println(" m/s^2");
  }
  else
  {
    Serial.println("[IMU GYROSCOPE]");
    Serial.print("GX : "); Serial.print(x, 1); Serial.println(" deg/s");
    Serial.print("GY : "); Serial.print(y, 1); Serial.println(" deg/s");
    Serial.print("GZ : "); Serial.print(z, 1); Serial.println(" deg/s");
  }
}

void printNode1GBNFrame(const struct can_frame &f)
{
  Serial.print("CAN ID          : 0x");
  if (f.can_id < 0x100) Serial.print('0');
  if (f.can_id < 0x10) Serial.print('0');
  Serial.println(f.can_id, HEX);
  Serial.print("DLC             : "); Serial.print(f.can_dlc); Serial.println(" BYTES");
  Serial.print("CAN DATA FRAME  : "); printHexArray(f.data, 8);
  Serial.print("SEQUENCE        : "); Serial.println(f.data[0]);
  Serial.print("PAYLOAD         : "); printHexArray(&f.data[1], 5);
  Serial.print("HAMMING BYTE    : 0x"); printHexByte(f.data[6]); Serial.println();
  Serial.print("CHECKSUM BYTE   : 0x"); printHexByte(f.data[7]); Serial.println();
}

void sendGBNPacket(byte seq, byte type, bool retransmission)
{
  struct can_frame f;
  byte slot = seq % 3;

  if (retransmission && node1GBNCacheValid[slot] && node1GBNCacheSeq[slot] == seq)
  {
    f = node1GBNCache[slot];
    Serial.println();
    Serial.println("========== NODE 1 | GBN RETRANSMISSION ==========");
    Serial.print("REUSING ORIGINAL PACKET | SEQ: "); Serial.println(seq);
  }
  else
  {
    sensors_event_t accelerometer, gyroscope, temperature;
    mpu.getEvent(&accelerometer, &gyroscope, &temperature);

    float sx, sy, sz;
    int16_t qx, qy, qz;

    if (type == TYPE_ACC)
    {
      sx = accelerometer.acceleration.x;
      sy = accelerometer.acceleration.y;
      sz = accelerometer.acceleration.z;
      // Signed 13-bit, 0.01 m/s^2 resolution, range +/-40.95 m/s^2.
      qx = (int16_t)constrain((long)lroundf(sx * 100.0f), -4096L, 4095L);
      qy = (int16_t)constrain((long)lroundf(sy * 100.0f), -4096L, 4095L);
      qz = (int16_t)constrain((long)lroundf(sz * 100.0f), -4096L, 4095L);
    }
    else
    {
      // Adafruit MPU6050 gyro values are rad/s; convert to deg/s.
      sx = gyroscope.gyro.x * 57.2957795f;
      sy = gyroscope.gyro.y * 57.2957795f;
      sz = gyroscope.gyro.z * 57.2957795f;
      // Signed 13-bit, 0.1 deg/s resolution, range +/-409.5 deg/s.
      qx = (int16_t)constrain((long)lroundf(sx * 10.0f), -4096L, 4095L);
      qy = (int16_t)constrain((long)lroundf(sy * 10.0f), -4096L, 4095L);
      qz = (int16_t)constrain((long)lroundf(sz * 10.0f), -4096L, 4095L);
    }

    byte payload[5];
    packThreeSigned13(qx, qy, qz, payload);

    f.can_id = (type == TYPE_ACC) ? ACCEL_CAN_ID : GYRO_CAN_ID;
    f.can_dlc = 8;
    f.data[0] = seq;
    memcpy(&f.data[1], payload, 5);
    f.data[6] = hammingEncode46_40(payload);
    f.data[7] = calculateXOR(payload, 5);

    node1GBNCache[slot] = f; // Cache clean packet before one-shot fault.
    node1GBNCacheValid[slot] = true;
    node1GBNCacheSeq[slot] = seq;

    Serial.println();
    Serial.println(type == TYPE_ACC ? "========== NODE 1 | IMU ACC ==========" : "========== NODE 1 | IMU GYRO =========");
    if (type == TYPE_ACC) printNode1GBNReadings(type, sx, sy, sz);
    else printNode1GBNReadings(type, sx, sy, sz);
    Serial.println("[GBN DATA PACKING]");
    Serial.println("DATA FORMAT      : 3 x SIGNED 13-BIT VALUES (39 BITS)");
    Serial.println("PADDING          : 1 BIT");
    Serial.println("QUANTIZATION     : ACC 0.01 m/s^2 | GYRO 0.1 deg/s");
    Serial.println("HAMMING ENCODING : SHORTENED HAMMING (46,40)");
    Serial.print("HAMMING PARITY   : 0x"); printHexByte(f.data[6]); Serial.println();
    Serial.print("XOR CHECKSUM     : 0x"); printHexByte(f.data[7]); Serial.println();
  }

  // Apply a one-shot fault only to an original transmission, never to RETX.
  if (!retransmission && !applyGBNFault(f))
  {
    Serial.println("[PREPARED CAN FRAME - NOT TRANSMITTED]");
    printNode1GBNFrame(f);
    Serial.println("TRANSMISSION     : INTENTIONALLY DROPPED");
    Serial.print("DROPPED SEQ      : "); Serial.println(seq);
    return;
  }

  Serial.println("[CAN TRANSMISSION]");
  printNode1GBNFrame(f);
  Serial.print("TRANSMISSION     : "); Serial.println(retransmission ? "RETRANSMITTED" : "SENT");

  if (mcp2515.sendMessage(&f) == MCP2515::ERROR_OK)
    Serial.println("CAN TX STATUS    : SUCCESS");
  else
    Serial.println("CAN TX STATUS    : FAILED");
  Serial.println("================================================");
}

// ==========================================================
// HANDLE GBN TRIGGER
// ==========================================================
void handleGBNTrigger(
  struct can_frame &f
)
{
  if (f.can_dlc < 4)
    return;

  if (f.data[0] != 1)
    return;

  byte seq = f.data[1];
  byte type = f.data[2];

  bool retransmission =
    f.data[3] != 0;

  if (!gbnMode)
    return;

  if (
    type != TYPE_ACC &&
    type != TYPE_GYRO
  )
    return;

  sendGBNPacket(
    seq,
    type,
    retransmission
  );
}

// ==========================================================
// RECEIVE CUMULATIVE ACK
// ==========================================================
void handleCumulativeACK(
  struct can_frame &f
)
{
  if (f.can_dlc < 2)
    return;

  byte ackSeq = f.data[1];

  Serial.print(
    "CUMULATIVE ACK <= SEQ "
  );

  Serial.println(ackSeq);
}

// ==========================================================
// RECEIVE CONTROL
// ==========================================================
void serviceControl()
{
  struct can_frame f;

  while (
    mcp2515.readMessage(&f)
    == MCP2515::ERROR_OK
  )
  {
    if (
      f.can_id ==
      MODE_CMD_ID
    )
    {
      if (f.can_dlc >= 1)
      {
        bool newMode =
          (f.data[0] == 1);

        if (newMode != gbnMode)
        {
          gbnMode = newMode;

          if (gbnMode)
            Serial.println(
              "MODE -> GBN"
            );
          else
            Serial.println(
              "MODE -> STOP-WAIT"
            );
        }
      }
    }

    else if (
      f.can_id ==
      GBN_TRIGGER_ID
    )
    {
      handleGBNTrigger(f);
    }

    else if (
      f.can_id ==
      GBN_CUM_ACK_ID
    )
    {
      handleCumulativeACK(f);
    }

    else if (
      f.can_id ==
      CAN_ACK_ID
    )
    {
      // Ignore old ACK in GBN
    }
  }
}


// ==========================================================
// STOP-WAIT SERIAL PRINT HELPERS
// ==========================================================
void printCANData(const struct can_frame &message, byte length)
{
  for (byte i = 0; i < length; i++)
  {
    if (i) Serial.print(" ");
    printHexByte(message.data[i]);
  }
  Serial.println();
}

void printIMUValues(const struct can_frame &message)
{
  int16_t x = ((int16_t)message.data[0] << 8) | message.data[1];
  int16_t y = ((int16_t)message.data[2] << 8) | message.data[3];
  int16_t z = ((int16_t)message.data[4] << 8) | message.data[5];

  if (message.can_id == ACCEL_CAN_ID)
  {
    Serial.print("AX              : "); Serial.print(x / 100.0, 2); Serial.println(" m/s^2");
    Serial.print("AY              : "); Serial.print(y / 100.0, 2); Serial.println(" m/s^2");
    Serial.print("AZ              : "); Serial.print(z / 100.0, 2); Serial.println(" m/s^2");
  }
  else
  {
    Serial.print("GX              : "); Serial.print(x / 100.0, 2); Serial.println(" rad/s");
    Serial.print("GY              : "); Serial.print(y / 100.0, 2); Serial.println(" rad/s");
    Serial.print("GZ              : "); Serial.print(z / 100.0, 2); Serial.println(" rad/s");
  }
}

void printStopWaitTransmission(
  const struct can_frame &originalMessage,
  const struct can_frame &transmittedMessage,
  byte checksumToPrint
)
{
  Serial.println();
  if (originalMessage.can_id == ACCEL_CAN_ID)
    Serial.println("========== NODE 1 | IMU ACC ==========");
  else
    Serial.println("========== NODE 1 | IMU GYRO =========");

  // Sensor values and Data are always the ORIGINAL packet.
  // Only Payload represents the frame actually transmitted.
  printIMUValues(originalMessage);

  Serial.println();
  Serial.println("========== CAN TRANSMISSION ==========");
  Serial.print("CAN ID          : 0x");
  if (originalMessage.can_id < 0x100) Serial.print("0");
  if (originalMessage.can_id < 0x10) Serial.print("0");
  Serial.println(originalMessage.can_id, HEX);
  Serial.print("Data            : "); printCANData(originalMessage, 6);
  Serial.print("Checksum        : 0x"); printHexByte(checksumToPrint); Serial.println();
  Serial.print("Payload         : "); printCANData(transmittedMessage, 7);
  Serial.println("Transmission    : SENT");
  Serial.println();
}

void printAckResult(bool received, uint16_t receivedID)
{
  Serial.print("Received ACK    : ");
  if (received) {
    Serial.print("0x");
    if (receivedID < 0x10) Serial.print("0");
    Serial.println(receivedID, HEX);
  } else {
    Serial.println("NONE");
  }
  Serial.print("ACK Status      : ");
  Serial.println(received ? "RECEIVED" : "TIMEOUT");
  Serial.println("======================================");
}

// ==========================================================
// STOP-WAIT + XOR
// ==========================================================
bool sendCANWithACK(struct can_frame *message)
{
  bool messageSent = false;
  int retries = 0;
  struct can_frame originalMessage = *message;

  while (!messageSent && retries < MAX_RETRIES)
  {
    *message = originalMessage;
    byte displayChecksum = originalMessage.data[6];

    if (retries > 0)
    {
      Serial.print("[RETRY] Transmission attempt ");
      Serial.print(retries + 1);
      Serial.print("/");
      Serial.println(MAX_RETRIES);
    }

    // ------------------------------------------------------
    // APPLY ONE-SHOT ERROR ONLY TO FIRST ATTEMPT
    // ------------------------------------------------------
    if (retries == 0 && errorInjection)
    {
      if (packetLoss)
      {
        // KEEP PACKET DROP BEHAVIOR UNCHANGED
        Serial.println("FAULT: PACKET LOSS");
        errorInjection = false;
        packetLoss = false;
        delay(ACK_TIMEOUT);
        retries++;
        continue;
      }

      if (errorByte >= 0 && errorByte <= 5)
      {
        byte originalByte = message->data[errorByte];
        message->data[errorByte] ^= 0x01;
        byte changedByte = message->data[errorByte];

        Serial.print("[INJECT] Byte ");
        Serial.print(errorByte);
        Serial.print(" changed from 0x");
        printHexByte(originalByte);
        Serial.print(" to 0x");
        printHexByte(changedByte);
        Serial.println();

        errorInjection = false;
        errorByte = -1;
      }
      else if (checksumError)
      {
        message->data[6] ^= 0x01;
        Serial.println("[INJECT] Checksum byte corrupted.");
        errorInjection = false;
        checksumError = false;
      }
    }

    printStopWaitTransmission(originalMessage, *message, displayChecksum);

    bool sentOK =
      (mcp2515.sendMessage(message) == MCP2515::ERROR_OK);

    if (!sentOK)
    {
      Serial.println("Transmission    : FAILED");
      Serial.println("ACK Status      : TIMEOUT");
      Serial.println("======================================");
      retries++;
      continue;
    }

    unsigned long start = millis();
    bool ackReceived = false;
    uint16_t receivedAckID = 0;

    while (millis() - start < ACK_TIMEOUT)
    {
      struct can_frame f;
      if (mcp2515.readMessage(&f) != MCP2515::ERROR_OK)
        continue;

      if (f.can_id == CAN_ACK_ID)
      {
        ackReceived = true;
        receivedAckID = f.can_id;
        break;
      }


      if (f.can_id == 0x038)
      {
        Serial.println("Received CAN ID: 0x038 | Node 2 Data | Ignored");
      }
      else if (f.can_id == 0x039)
      {
        Serial.println("Received CAN ID: 0x039 | Node 2 ACK | Ignored");
      }

      if (f.can_id == MODE_CMD_ID)
      {
        if (f.can_dlc >= 1)
          gbnMode = (f.data[0] == 1);
        if (gbnMode)
          break;
      }
    }

    Serial.println();
    Serial.println("========== ACK VERIFICATION ==========");
    Serial.println("Expected ACK    : 0x037");
    if (ackReceived)
    {
      Serial.print("Received ACK    : 0x");
      if (receivedAckID < 0x100) Serial.print("0");
      if (receivedAckID < 0x10) Serial.print("0");
      Serial.println(receivedAckID, HEX);
      Serial.println("ACK Status      : RECEIVED");
      Serial.println("======================================");
      messageSent = true;

      if (retries > 0)
        Serial.println("[SUCCESS] Original packet retransmitted successfully.");
    }
    else
    {
      Serial.println("Received ACK    : NONE");
      Serial.println("ACK Status      : TIMEOUT");
      Serial.println("======================================");
      retries++;

      if (retries < MAX_RETRIES)
      {
        Serial.print("[WARN] ACK not received. Retrying (");
        Serial.print(retries + 1);
        Serial.print("/");
        Serial.print(MAX_RETRIES);
        Serial.println(")...");
      }
    }
  }

  if (!messageSent)
    Serial.println("Failed to send after 3 attempts");

  return messageSent;
}

// ==========================================================
// SETUP// ==========================================================
// SETUP
// ==========================================================
void setup()
{
  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println(
    "================================"
  );

  Serial.println(
    "NODE 1 - ESP32 + MPU6050"
  );

  Serial.println(
    "================================"
  );

  Wire.begin(
    MPU6050_SDA,
    MPU6050_SCL
  );

  if (!mpu.begin(0x68))
  {
    Serial.println(
      "MPU6050 NOT FOUND!"
    );

    while (1)
      delay(1000);
  }

  mpu.setAccelerometerRange(
    MPU6050_RANGE_8_G
  );

  mpu.setGyroRange(
    MPU6050_RANGE_500_DEG
  );

  mpu.setFilterBandwidth(
    MPU6050_BAND_21_HZ
  );

  SPI.begin();

  mcp2515.reset();

  mcp2515.setBitrate(
    CAN_500KBPS,
    MCP_8MHZ
  );

  mcp2515.setNormalMode();

  Serial.println(
    "NODE 1 READY"
  );
}

// ==========================================================
// LOOP
// ==========================================================
void loop()
{
  processSerialCommand();

  sendHeartbeat();

  serviceControl();

  // ========================================================
  // GBN MODE
  // ========================================================
  if (gbnMode)
  {
    delay(2);
    return;
  }

  // ========================================================
  // EXISTING STOP-WAIT MODE
  // ========================================================
  sensors_event_t accelerometer;
  sensors_event_t gyroscope;
  sensors_event_t temperature;

  mpu.getEvent(
    &accelerometer,
    &gyroscope,
    &temperature
  );

  int16_t ax =
    (int16_t)(
      accelerometer.acceleration.x * 100
    );

  int16_t ay =
    (int16_t)(
      accelerometer.acceleration.y * 100
    );

  int16_t az =
    (int16_t)(
      accelerometer.acceleration.z * 100
    );

  int16_t gx =
    (int16_t)(
      gyroscope.gyro.x * 100
    );

  int16_t gy =
    (int16_t)(
      gyroscope.gyro.y * 100
    );

  int16_t gz =
    (int16_t)(
      gyroscope.gyro.z * 100
    );

  // --------------------------------------------------------
  // ACC
  // --------------------------------------------------------
  canMsg.can_id =
    ACCEL_CAN_ID;

  canMsg.can_dlc = 7;

  canMsg.data[0] =
    (ax >> 8) & 0xFF;

  canMsg.data[1] =
    ax & 0xFF;

  canMsg.data[2] =
    (ay >> 8) & 0xFF;

  canMsg.data[3] =
    ay & 0xFF;

  canMsg.data[4] =
    (az >> 8) & 0xFF;

  canMsg.data[5] =
    az & 0xFF;

  canMsg.data[6] =
    calculateXOR(
      canMsg.data,
      6
    );

  Serial.println(
    "Sending ACCELEROMETER..."
  );

  sendCANWithACK(&canMsg);

  // --------------------------------------------------------
  // GYRO
  // --------------------------------------------------------
  canMsg.can_id =
    GYRO_CAN_ID;

  canMsg.can_dlc = 7;

  canMsg.data[0] =
    (gx >> 8) & 0xFF;

  canMsg.data[1] =
    gx & 0xFF;

  canMsg.data[2] =
    (gy >> 8) & 0xFF;

  canMsg.data[3] =
    gy & 0xFF;

  canMsg.data[4] =
    (gz >> 8) & 0xFF;

  canMsg.data[5] =
    gz & 0xFF;

  canMsg.data[6] =
    calculateXOR(
      canMsg.data,
      6
    );

  Serial.println(
    "Sending GYROSCOPE..."
  );

  sendCANWithACK(&canMsg);

  delay(1000);
}
