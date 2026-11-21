#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <mcp2515.h>
#include <math.h>
#include <string.h>

// ==========================================================
// HARDWARE
// ==========================================================
#define BMP280_SDA D2
#define BMP280_SCL D1
#define MCP2515_CS D8

Adafruit_BMP280 bmp;
MCP2515 mcp2515(MCP2515_CS);

struct can_frame canMsg;

// ==========================================================
// EXISTING CAN
// ==========================================================
#define SENSOR_CAN_ID 0x038
#define CAN_ACK_ID    0x039

// ==========================================================
// GBN CONTROL
// ==========================================================
#define MODE_CMD_ID    0x050
#define GBN_TRIGGER_ID 0x051
#define HEARTBEAT_ID   0x052
#define GBN_CUM_ACK_ID 0x053

#define TYPE_BMP 2

// ==========================================================
// STOP WAIT
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
// ERROR
// ==========================================================
bool errorInjection = false;
int errorByte = -1;
bool checksumError = false;
bool packetLoss = false;

// ==========================================================
// XOR
// ==========================================================
byte calculateXOR(
  byte *data,
  byte length
)
{
  byte checksum = 0;

  for (
    byte i = 0;
    i < length;
    i++
  )
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
// SERIAL COMMAND
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
    if (!errorInjection) Serial.println("OFF");
    else if (packetLoss) Serial.println("DROP - NEXT ORIGINAL PACKET");
    else if (checksumError) Serial.println("CHECKSUM ERROR - NEXT ORIGINAL PACKET");
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

  hb.data[0] = 2;

  mcp2515.sendMessage(&hb);
}

// ==========================================================
// APPLY GBN FAULT
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

// Cache three original packets so retransmission repeats identical bytes.
struct can_frame node2GBNCache[3];
bool node2GBNCacheValid[3] = {false, false, false};
byte node2GBNCacheSeq[3] = {0, 0, 0};

void printNode2GBNFrame(const struct can_frame &f)
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

// ==========================================================
// GBN SEND
//
// byte 0 = sequence
// byte 1-6 = temperature + pressure
// byte 7 = XOR
// ==========================================================
void sendGBNPacket(byte seq, bool retransmission)
{
  struct can_frame f;
  byte slot = seq % 3;

  if (retransmission && node2GBNCacheValid[slot] && node2GBNCacheSeq[slot] == seq)
  {
    f = node2GBNCache[slot];
    Serial.println();
    Serial.println("========== NODE 2 | BMP280 RETRANSMISSION ==========");
    Serial.print("REUSING ORIGINAL PACKET | SEQ: "); Serial.println(seq);
  }
  else
  {
    float temperature = bmp.readTemperature();
    float pressure = bmp.readPressure() / 100.0f;

    if (isnan(temperature) || isnan(pressure))
    {
      Serial.println("BMP280 reading failed");
      return;
    }

    // Five-byte payload: signed temperature x100 (16 bits), pressure x100 (24 bits).
    int32_t tempScaled = (int32_t)lroundf(temperature * 100.0f);
    uint32_t pressureScaled = (uint32_t)lroundf(pressure * 100.0f);
    tempScaled = constrain(tempScaled, -32768L, 32767L);
    if (pressureScaled > 0xFFFFFFUL) pressureScaled = 0xFFFFFFUL;

    byte payload[5];
    payload[0] = (byte)((uint16_t)tempScaled >> 8);
    payload[1] = (byte)tempScaled;
    payload[2] = (byte)(pressureScaled >> 16);
    payload[3] = (byte)(pressureScaled >> 8);
    payload[4] = (byte)pressureScaled;

    f.can_id = SENSOR_CAN_ID;
    f.can_dlc = 8;
    f.data[0] = seq;
    memcpy(&f.data[1], payload, 5);
    f.data[6] = hammingEncode46_40(payload);
    f.data[7] = calculateXOR(payload, 5);

    node2GBNCache[slot] = f;
    node2GBNCacheValid[slot] = true;
    node2GBNCacheSeq[slot] = seq;

    Serial.println();
    Serial.println("========== NODE 2 | BMP280 ==========");
    Serial.println("[BMP280 SENSOR READINGS]");
    Serial.print("TEMPERATURE     : "); Serial.print(temperature, 2); Serial.println(" C");
    Serial.print("PRESSURE        : "); Serial.print(pressure, 2); Serial.println(" hPa");
    Serial.println("[GBN DATA PACKING]");
    Serial.println("TEMP            : SIGNED 16-BIT, 0.01 C RESOLUTION");
    Serial.println("PRESSURE        : UNSIGNED 24-BIT, 0.01 hPa RESOLUTION");
    Serial.println("HAMMING ENCODING: SHORTENED HAMMING (46,40)");
    Serial.print("HAMMING PARITY  : 0x"); printHexByte(f.data[6]); Serial.println();
    Serial.print("XOR CHECKSUM    : 0x"); printHexByte(f.data[7]); Serial.println();
  }

  if (!retransmission && !applyGBNFault(f))
  {
    Serial.println("[PREPARED CAN FRAME - NOT TRANSMITTED]");
    printNode2GBNFrame(f);
    Serial.println("TRANSMISSION    : INTENTIONALLY DROPPED");
    Serial.print("DROPPED SEQ     : "); Serial.println(seq);
    return;
  }

  Serial.println("[CAN TRANSMISSION]");
  printNode2GBNFrame(f);
  Serial.print("TRANSMISSION    : "); Serial.println(retransmission ? "RETRANSMITTED" : "SENT");

  if (mcp2515.sendMessage(&f) == MCP2515::ERROR_OK)
    Serial.println("CAN TX STATUS   : SUCCESS");
  else
    Serial.println("CAN TX STATUS   : FAILED");
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

  // Target Node 2
  if (f.data[0] != 2)
    return;

  if (!gbnMode)
    return;

  byte seq =
    f.data[1];

  byte type =
    f.data[2];

  bool retransmission =
    f.data[3] != 0;

  if (type != TYPE_BMP)
    return;

  sendGBNPacket(
    seq,
    retransmission
  );
}

// ==========================================================
// CUMULATIVE ACK
// ==========================================================
void handleCumulativeACK(
  struct can_frame &f
)
{
  if (f.can_dlc < 2)
    return;

  Serial.print(
    "CUMULATIVE ACK <= SEQ "
  );

  Serial.println(
    f.data[1]
  );
}

// ==========================================================
// SERVICE CONTROL
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

void printBMPValues(const struct can_frame &message)
{
  int16_t tempValue = ((int16_t)message.data[0] << 8) | message.data[1];
  uint32_t pressureValue =
    ((uint32_t)message.data[2] << 24) |
    ((uint32_t)message.data[3] << 16) |
    ((uint32_t)message.data[4] << 8) |
    message.data[5];

  Serial.print("Temperature     : "); Serial.print(tempValue / 100.0, 2); Serial.println(" C");
  Serial.print("Pressure        : "); Serial.print(pressureValue / 100.0, 2); Serial.println(" hPa");
}

void printStopWaitTransmission(
  const struct can_frame &originalMessage,
  const struct can_frame &transmittedMessage,
  byte checksumToPrint
)
{
  Serial.println();
  Serial.println("========== NODE 2 | BMP280 ==========");

  // Sensor values and Data are always the ORIGINAL packet.
  // Only Payload represents the frame actually transmitted.
  printBMPValues(originalMessage);

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

// ==========================================================
// STOP-WAIT + XOR
// ==========================================================
bool sendCANWithACK(struct can_frame *message)
{
  bool messageSent = false;
  int retries = 0;
  struct can_frame original = *message;
  byte displayChecksum = original.data[6];

  while (!messageSent && retries < MAX_RETRIES)
  {
    *message = original;

    if (retries > 0)
    {
      Serial.print("[RETRY] Transmission attempt ");
      Serial.print(retries + 1);
      Serial.print("/");
      Serial.println(MAX_RETRIES);
    }

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

    printStopWaitTransmission(original, *message, displayChecksum);

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

      if (f.can_id == 0x036)
        Serial.println("Received CAN ID: 0x036 | Node 1 ACC | Ignored");
      else if (f.can_id == 0x035)
        Serial.println("Received CAN ID: 0x035 | Node 1 GYRO | Ignored");
      else if (f.can_id == 0x037)
        Serial.println("Received CAN ID: 0x037 | Node 1 ACK | Ignored");

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
    Serial.println("Expected ACK    : 0x039");
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
    "NODE 2 - ESP8266 + GY-BMP280"
  );

  Serial.println(
    "================================"
  );

  Wire.begin(
    BMP280_SDA,
    BMP280_SCL
  );

  if (!bmp.begin(0x76))
  {
    Serial.println(
      "GY-BM280 NOT FOUND!"
    );

    while (1)
      delay(1000);
  }

  SPI.begin();

  mcp2515.reset();

  mcp2515.setBitrate(
    CAN_500KBPS,
    MCP_8MHZ
  );

  mcp2515.setNormalMode();

  Serial.println(
    "NODE 2 READY"
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
  // GBN
  // ========================================================
  if (gbnMode)
  {
    delay(2);
    return;
  }

  // ========================================================
  // EXISTING STOP-WAIT
  // ========================================================
  float temperature =
    bmp.readTemperature();

  float pressure =
    bmp.readPressure() / 100.0;

  if (
    isnan(temperature) ||
    isnan(pressure)
  )
  {
    Serial.println(
      "GY-BM280 reading failed!"
    );

    delay(1000);

    return;
  }

  int16_t tempInt =
    (int16_t)(
      temperature * 100
    );

  uint32_t pressureInt =
    (uint32_t)(
      pressure * 100
    );

  canMsg.can_id =
    SENSOR_CAN_ID;

  canMsg.can_dlc = 7;

  // TEMPERATURE
  canMsg.data[0] =
    (tempInt >> 8) & 0xFF;

  canMsg.data[1] =
    tempInt & 0xFF;

  // PRESSURE
  canMsg.data[2] =
    (pressureInt >> 24) & 0xFF;

  canMsg.data[3] =
    (pressureInt >> 16) & 0xFF;

  canMsg.data[4] =
    (pressureInt >> 8) & 0xFF;

  canMsg.data[5] =
    pressureInt & 0xFF;

  // XOR
  canMsg.data[6] =
    calculateXOR(
      canMsg.data,
      6
    );

  Serial.println(
    "Sending NODE 2 BMP280..."
  );

  sendCANWithACK(
    &canMsg
  );

  delay(1000);
}
