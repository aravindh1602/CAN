#include <SPI.h>
#include <mcp2515.h>
#include <LiquidCrystal.h>
#include <math.h>
#include <string.h>

void sendACK(uint16_t ackID);

// ==========================================================
// LCD
// ==========================================================

#define LCD_RS 13
#define LCD_EN 14
#define LCD_D4 25
#define LCD_D5 26
#define LCD_D6 27
#define LCD_D7 33

LiquidCrystal lcd(
  LCD_RS,
  LCD_EN,
  LCD_D4,
  LCD_D5,
  LCD_D6,
  LCD_D7
);

// ==========================================================
// MCP2515
// ==========================================================

#define MCP2515_CS 5

MCP2515 mcp2515(MCP2515_CS);

struct can_frame canMsg;

// ==========================================================
// CAN IDs
// ==========================================================

#define NODE1_ACCEL_CAN_ID  0x036
#define NODE1_GYRO_CAN_ID   0x035
#define NODE1_ACK_ID        0x037

#define NODE2_SENSOR_CAN_ID 0x038
#define NODE2_ACK_ID        0x039

// ==========================================================
// GBN CONTROL IDs
// ==========================================================

#define MODE_CMD_ID       0x050
#define GBN_TRIGGER_ID    0x051
#define HEARTBEAT_ID      0x052
#define GBN_CUM_ACK_ID    0x053

// ==========================================================
// DATA TYPES
// ==========================================================

#define TYPE_ACC   0
#define TYPE_GYRO  1
#define TYPE_BMP   2

// ==========================================================
// GBN SETTINGS
// ==========================================================

#define GBN_WINDOW_SIZE       3
#define GBN_PACKET_INTERVAL   1000UL
#define GBN_TIMEOUT            1500UL

// ==========================================================
// NODE DETECTION
// ==========================================================

#define NODE_TIMEOUT 5000UL

bool node1Received = false;
bool node2Received = false;

unsigned long lastNode1Heartbeat = 0;
unsigned long lastNode2Heartbeat = 0;

// ==========================================================
// MODE
// ==========================================================

bool gbnMode = false;

// ==========================================================
// GBN STATE
// ==========================================================

byte expectedSeq = 0;
byte windowBase = 0;

byte packetsTriggered = 0;

bool windowWaiting = false;

unsigned long nextPacketTime = 0;
unsigned long windowTimeoutStart = 0;

// ==========================================================
// GBN ERROR STATE
// ==========================================================


// ==========================================================
// SENSOR VALUES
// ==========================================================

float latestAccelX = 0;
float latestAccelY = 0;
float latestAccelZ = 0;

float latestGyroX = 0;
float latestGyroY = 0;
float latestGyroZ = 0;

float latestTemperatureC = 0;
float latestPressure_hPa = 0;

// ==========================================================
// SINGLE NODE LCD
// ==========================================================

const unsigned long LCD_UPDATE_INTERVAL = 2000;

unsigned long lastLCDUpdate = 0;

int lcdScreen = 0;

// ==========================================================
// GBN LCD RECOVERY
// ==========================================================

byte lcdRecoveryPhase = 0;
// 0 = normal
// 1 = recovery
// 2 = replay recovered packets

byte lcdRecoveryStartSeq = 0;
byte lcdRecoveryEndSeq = 0;

// ==========================================================
// LCD EVENT QUEUE
// ==========================================================

#define LCD_QUEUE_SIZE 20

struct LCD_Event
{
  byte type;
  byte seq;
  byte nextSeq;
  byte discardSeq;
};

LCD_Event lcdQueue[LCD_QUEUE_SIZE];

byte lcdHead = 0;
byte lcdTail = 0;

bool lcdEventActive = false;

LCD_Event currentLCDEvent;

unsigned long lcdEventStart = 0;

#define GBN_LCD_DELAY 1800UL

// ==========================================================
// LCD EVENT TYPES
// ==========================================================

#define LCD_NORMAL    0
#define LCD_ERROR     1
#define LCD_TIMEOUT   2
#define LCD_RETX      3
#define LCD_COMPLETE  4
#define LCD_CORRECTED  5
#define LCD_CHECKSUM   6
#define LCD_ACCEPT     7
#define LCD_NO_RETX    8

// ==========================================================
// XOR CHECKSUM
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
// LCD PRINT
// ==========================================================

void showLCD(const char *line1, const char *line2)
{
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print(line1);

  lcd.setCursor(0, 1);
  lcd.print(line2);
}

// ==========================================================
// QUEUE LCD EVENT
// ==========================================================

void queueLCD(
  byte type,
  byte seq = 0,
  byte nextSeq = 0,
  byte discardSeq = 0
)
{
  if (
    type == LCD_NORMAL &&
    lcdRecoveryPhase == 1
  )
  {
    return;
  }

  byte nextTail =
    (lcdTail + 1) % LCD_QUEUE_SIZE;

  if (nextTail == lcdHead)
    return;

  lcdQueue[lcdTail].type = type;
  lcdQueue[lcdTail].seq = seq;
  lcdQueue[lcdTail].nextSeq = nextSeq;
  lcdQueue[lcdTail].discardSeq = discardSeq;

  lcdTail = nextTail;
}

// Immediately interrupt normal LCD screens for an important event.
// A checksum-error screen already being displayed is allowed to finish
// before the timeout/retransmission screens are queued.
void queueLCDPriority(byte type, byte seq = 0, byte nextSeq = 0, byte discardSeq = 0)
{
  bool keepChecksumScreen =
    lcdEventActive && currentLCDEvent.type == LCD_CHECKSUM;

  lcdHead = 0;
  lcdTail = 0;

  if (!keepChecksumScreen)
    lcdEventActive = false;

  queueLCD(type, seq, nextSeq, discardSeq);
}

// ==========================================================
// GBN LCD
// ==========================================================

void updateGBNLCD()
{
  if (!gbnMode)
    return;

  if (!lcdEventActive)
  {
    if (lcdHead == lcdTail)
      return;

    currentLCDEvent =
      lcdQueue[lcdHead];

    lcdHead =
      (lcdHead + 1) % LCD_QUEUE_SIZE;

    lcdEventActive = true;

    lcdEventStart = millis();

    // ------------------------------------------------------
    // NORMAL
    // ------------------------------------------------------

    if (
      currentLCDEvent.type ==
      LCD_NORMAL
    )
    {
      char line1[17];
      char line2[17];

      byte type =
        currentLCDEvent.seq % 3;

      if (type == TYPE_ACC)
      {
        snprintf(
          line1,
          sizeof(line1),
          "NODE1 ACC"
        );
      }
      else if (type == TYPE_GYRO)
      {
        snprintf(
          line1,
          sizeof(line1),
          "NODE1 GYRO"
        );
      }
      else
      {
        snprintf(
          line1,
          sizeof(line1),
          "NODE2 TEMP/PRES"
        );
      }

      snprintf(
        line2,
        sizeof(line2),
        "SEQ:%u NEXT:%u",
        currentLCDEvent.seq,
        currentLCDEvent.nextSeq
      );

      showLCD(line1, line2);
    }

    // ------------------------------------------------------
    // ERROR
    // ------------------------------------------------------

    else if (
      currentLCDEvent.type ==
      LCD_ERROR
    )
    {
      char line1[17];
      char line2[17];

      snprintf(
        line1,
        sizeof(line1),
        "CORRUPT SEQ:%u",
        currentLCDEvent.seq
      );

      snprintf(
        line2,
        sizeof(line2),
        "DISCARD SEQ:%u",
        currentLCDEvent.discardSeq
      );

      showLCD(line1, line2);
    }

    // ------------------------------------------------------
    // TIMEOUT
    // ------------------------------------------------------

    else if (
      currentLCDEvent.type ==
      LCD_TIMEOUT
    )
    {
      char line2[17];

      snprintf(
        line2,
        sizeof(line2),
        "SEQ:%u",
        currentLCDEvent.seq
      );

      showLCD(
        "SEQ TIMEOUT",
        line2
      );
    }

    // ------------------------------------------------------
    // RETRANSMISSION
    // ------------------------------------------------------

    else if (
      currentLCDEvent.type ==
      LCD_RETX
    )
    {
      char line2[17];

      snprintf(
        line2,
        sizeof(line2),
        "SEQ:%u,%u,%u",
        currentLCDEvent.seq,
        currentLCDEvent.nextSeq,
        currentLCDEvent.discardSeq
      );

      showLCD(
        "GBN RETX",
        line2
      );
    }

    // ------------------------------------------------------
    // COMPLETE
    // ------------------------------------------------------

    else if (
      currentLCDEvent.type ==
      LCD_COMPLETE
    )
    {
      // Recovery is complete. Do not invent/replay LCD packets:
      // normal screens resume when the next real in-order packets arrive.
      lcdRecoveryPhase = 0;

      showLCD(
        "STATUS: OK",
        "GBN COMPLETE"
      );
    }

    else if (currentLCDEvent.type == LCD_CORRECTED)
    {
      char line2[17];
      snprintf(line2, sizeof(line2), "SEQ:%02u BIT:%02u", currentLCDEvent.seq, currentLCDEvent.nextSeq);
      showLCD("ERROR CORRECTED", line2);
    }
    else if (currentLCDEvent.type == LCD_CHECKSUM)
    {
      char line2[17];
      snprintf(line2, sizeof(line2), "SEQ:%02u REJECT", currentLCDEvent.seq);
      showLCD("CHECKSUM ERROR", line2);
    }
    else if (currentLCDEvent.type == LCD_ACCEPT)
    {
      char line2[17];
      snprintf(line2, sizeof(line2), "SEQ:%02u ACCEPT", currentLCDEvent.seq);
      showLCD("XOR: VALID", line2);
    }
    else if (currentLCDEvent.type == LCD_NO_RETX)
    {
      showLCD("STATUS: OK", "GBN: NO RETX");
    }

    return;
  }

  if (
    millis() -
    lcdEventStart >=
    GBN_LCD_DELAY
  )
  {
    if (
      currentLCDEvent.type == LCD_NORMAL &&
      lcdRecoveryPhase == 2 &&
      currentLCDEvent.seq == lcdRecoveryEndSeq
    )
    {
      lcdRecoveryPhase = 0;
    }

    lcdEventActive = false;
  }
}

// ==========================================================
// SINGLE NODE LCD
// ==========================================================

void updateSingleNodeLCD()
{
  if (gbnMode)
    return;

  if (
    millis() -
    lastLCDUpdate <
    LCD_UPDATE_INTERVAL
  )
  {
    return;
  }

  lastLCDUpdate = millis();

  // --------------------------------------------------------
  // NO NODE
  // --------------------------------------------------------

  if (
    !node1Received &&
    !node2Received
  )
  {
    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("CAN RECEIVER");

    lcd.setCursor(0, 1);
    lcd.print("Waiting...");

    return;
  }

  // --------------------------------------------------------
  // NODE 1 ONLY
  // --------------------------------------------------------

  if (
    node1Received &&
    !node2Received
  )
  {
    lcd.clear();

    if (lcdScreen == 0)
    {
      lcd.setCursor(0, 0);
      lcd.print("N1 ACCEL X:");
      lcd.print(latestAccelX, 1);

      lcd.setCursor(0, 1);
      lcd.print("Y:");
      lcd.print(latestAccelY, 1);

      lcd.print(" Z:");
      lcd.print(latestAccelZ, 1);

      lcdScreen = 1;
    }
    else
    {
      lcd.setCursor(0, 0);
      lcd.print("N1 GYRO X:");
      lcd.print(latestGyroX, 1);

      lcd.setCursor(0, 1);
      lcd.print("Y:");
      lcd.print(latestGyroY, 1);

      lcd.print(" Z:");
      lcd.print(latestGyroZ, 1);

      lcdScreen = 0;
    }

    return;
  }

  // --------------------------------------------------------
  // NODE 2 ONLY
  // --------------------------------------------------------

  if (
    !node1Received &&
    node2Received
  )
  {
    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("N2 BMP280");

    lcd.setCursor(0, 1);
    lcd.print("T:");
    lcd.print(latestTemperatureC, 1);

    lcd.print(" P:");
    lcd.print(latestPressure_hPa, 0);

    return;
  }
}

// ==========================================================
// SEND MODE COMMAND
// ==========================================================

void sendModeCommand(bool enable)
{
  struct can_frame f;

  f.can_id = MODE_CMD_ID;
  f.can_dlc = 1;

  f.data[0] =
    enable ? 1 : 0;

  mcp2515.sendMessage(&f);
}

// ==========================================================
// SEND CUMULATIVE ACK
// ==========================================================

void sendCumulativeACK()
{
  struct can_frame f;

  f.can_id = GBN_CUM_ACK_ID;
  f.can_dlc = 3;

  f.data[0] = 0;
  f.data[1] = expectedSeq - 1;
  f.data[2] = expectedSeq;

  mcp2515.sendMessage(&f);

  Serial.print("CUMULATIVE ACK = ");
  Serial.print(expectedSeq - 1);

  Serial.print(" | EXPECTED = ");
  Serial.println(expectedSeq);
}

// ==========================================================
// SEND GBN TRIGGER
// ==========================================================

void sendGBNTrigger(
  byte seq,
  bool retransmission
)
{
  byte type =
    seq % 3;

  byte targetNode;

  if (type == TYPE_BMP)
    targetNode = 2;
  else
    targetNode = 1;

  struct can_frame f;

  f.can_id = GBN_TRIGGER_ID;
  f.can_dlc = 4;

  f.data[0] = targetNode;
  f.data[1] = seq;
  f.data[2] = type;
  f.data[3] =
    retransmission ? 1 : 0;

  mcp2515.sendMessage(&f);

  if (retransmission)
    Serial.print("GBN RETX TRIGGER -> ");
  else
    Serial.print("GBN TRIGGER -> ");

  Serial.print("SEQ ");
  Serial.print(seq);

  Serial.print(" TYPE ");
  Serial.println(type);
}

// ==========================================================
// START GBN WINDOW
// ==========================================================

void startGBNWindow()
{
  windowBase = expectedSeq;

  packetsTriggered = 0;

  windowWaiting = false;

  nextPacketTime =
    millis() +
    GBN_PACKET_INTERVAL;

  Serial.println();
  Serial.print("NEW GBN WINDOW: ");
  Serial.print(windowBase);
  Serial.print(" - ");
  Serial.println(
    (byte)(windowBase + 2)
  );
}

// ==========================================================
// START GBN
// ==========================================================

void startGBN()
{
  if (gbnMode)
    return;

  gbnMode = true;

  expectedSeq = 0;

  lcdHead = 0;
  lcdTail = 0;
  lcdEventActive = false;

  sendModeCommand(true);

  startGBNWindow();

  Serial.println();
  Serial.println("========== GBN START ==========");
  Serial.println("WINDOW SIZE = 3");
}

// ==========================================================
// STOP GBN
// ==========================================================

void stopGBN()
{
  if (!gbnMode)
    return;

  gbnMode = false;

  windowWaiting = false;
  packetsTriggered = 0;

  sendModeCommand(false);

  Serial.println();
  Serial.println("========== GBN STOP ==========");
}

// ==========================================================
// CHECK MODE
// ==========================================================

void checkMode()
{
  bool both =
    node1Received &&
    node2Received;

  if (
    both &&
    !gbnMode
  )
  {
    startGBN();
  }

  if (
    !both &&
    gbnMode
  )
  {
    stopGBN();
  }
}

// ==========================================================
// DECODE ACC
// GBN format:
// data[0] = SEQ
// data[1..6] = ACC
// data[7] = XOR
// ==========================================================

void decodeACC(byte *d)
{
  int16_t x =
    ((int16_t)d[1] << 8) |
    d[2];

  int16_t y =
    ((int16_t)d[3] << 8) |
    d[4];

  int16_t z =
    ((int16_t)d[5] << 8) |
    d[6];

  latestAccelX =
    x / 100.0;

  latestAccelY =
    y / 100.0;

  latestAccelZ =
    z / 100.0;
}

// ==========================================================
// DECODE GYRO
// GBN format:
// data[0] = SEQ
// data[1..6] = GYRO
// data[7] = XOR
// ==========================================================

void decodeGYRO(byte *d)
{
  int16_t x =
    ((int16_t)d[1] << 8) |
    d[2];

  int16_t y =
    ((int16_t)d[3] << 8) |
    d[4];

  int16_t z =
    ((int16_t)d[5] << 8) |
    d[6];

  latestGyroX =
    x / 100.0;

  latestGyroY =
    y / 100.0;

  latestGyroZ =
    z / 100.0;
}

// ==========================================================
// DECODE BMP — GBN ONLY
//
// GBN format:
// data[0] = SEQ
// data[1..2] = TEMP
// data[3..6] = PRESSURE
// data[7] = XOR
// ==========================================================

void decodeBMP(byte *d)
{
  int16_t temperature =
    ((int16_t)d[1] << 8) |
    d[2];

  uint32_t pressure =
    ((uint32_t)d[3] << 24) |
    ((uint32_t)d[4] << 16) |
    ((uint32_t)d[5] << 8) |
    d[6];

  latestTemperatureC =
    temperature / 100.0;

  latestPressure_hPa =
    pressure / 100.0;
}

// ==========================================================
// DECODE BMP — STOP AND WAIT ONLY
//
// IMPORTANT FIX:
//
// Stop-Wait Node2 packet:
// data[0] = TEMP MSB
// data[1] = TEMP LSB
// data[2..5] = PRESSURE
// data[6] = XOR
//
// There is NO sequence byte in Stop-Wait.
// ==========================================================

void decodeStopWaitBMP(byte *d)
{
  int16_t temperature =
    ((int16_t)d[0] << 8) |
    d[1];

  uint32_t pressure =
    ((uint32_t)d[2] << 24) |
    ((uint32_t)d[3] << 16) |
    ((uint32_t)d[4] << 8) |
    d[5];

  latestTemperatureC =
    temperature / 100.0;

  latestPressure_hPa =
    pressure / 100.0;
}

// ==========================================================
// GBN-ONLY UNPACKING: 5-BYTE SENSOR PAYLOAD
// ==========================================================
int16_t unpackSigned13(const byte payload[5], byte bitOffset)
{
  uint16_t value = 0;
  for (byte i = 0; i < 13; i++)
    value = (uint16_t)((value << 1) | getPayloadBit(payload, bitOffset + i));
  if (value & 0x1000) value |= 0xE000; // sign extend 13-bit two's complement
  return (int16_t)value;
}

void decodeGBNACC(const byte payload[5])
{
  latestAccelX = unpackSigned13(payload, 0) / 100.0f;
  latestAccelY = unpackSigned13(payload, 13) / 100.0f;
  latestAccelZ = unpackSigned13(payload, 26) / 100.0f;
}

void decodeGBNGYRO(const byte payload[5])
{
  latestGyroX = unpackSigned13(payload, 0) / 10.0f;
  latestGyroY = unpackSigned13(payload, 13) / 10.0f;
  latestGyroZ = unpackSigned13(payload, 26) / 10.0f;
}

void decodeGBNBMP(const byte payload[5])
{
  int16_t temperature = (int16_t)(((uint16_t)payload[0] << 8) | payload[1]);
  uint32_t pressure = ((uint32_t)payload[2] << 16) |
                      ((uint32_t)payload[3] << 8) | payload[4];
  latestTemperatureC = temperature / 100.0f;
  latestPressure_hPa = pressure / 100.0f;
}

void printReceiverGBNFrame(const struct can_frame &f)
{
  Serial.print("CAN ID          : 0x");
  if (f.can_id < 0x100) Serial.print('0');
  if (f.can_id < 0x10) Serial.print('0');
  Serial.println(f.can_id, HEX);
  Serial.print("DLC             : "); Serial.print(f.can_dlc); Serial.println(" BYTES");
  Serial.print("CAN DATA FRAME  : "); printHexArray(f.data, f.can_dlc);
  Serial.print("SEQUENCE        : "); Serial.println(f.data[0]);
  Serial.print("PAYLOAD         : "); printHexArray(&f.data[1], 5);
  Serial.print("HAMMING BYTE    : 0x"); printHexByte(f.data[6]); Serial.println();
  Serial.print("CHECKSUM BYTE   : 0x"); printHexByte(f.data[7]); Serial.println();
}

// ==========================================================
// RECEIVE GBN
// ==========================================================

void receiveGBN()
{
  if (canMsg.can_dlc != 8)
  {
    Serial.println("GBN INVALID DLC - EXPECTED 8");
    return;
  }

  byte seq = canMsg.data[0];
  byte type;
  if (canMsg.can_id == NODE1_ACCEL_CAN_ID) type = TYPE_ACC;
  else if (canMsg.can_id == NODE1_GYRO_CAN_ID) type = TYPE_GYRO;
  else if (canMsg.can_id == NODE2_SENSOR_CAN_ID) type = TYPE_BMP;
  else return;

  Serial.println();
  Serial.println("========== CAN RECEIVER | GBN ==========");
  Serial.print("NODE ID          : "); Serial.println(type == TYPE_BMP ? "NODE 2" : "NODE 1");
  Serial.print("PACKET TYPE      : ");
  if (type == TYPE_ACC) Serial.println("IMU ACC");
  else if (type == TYPE_GYRO) Serial.println("IMU GYRO");
  else Serial.println("BMP280");
  printReceiverGBNFrame(canMsg);

  byte correctedPayload[5];
  memcpy(correctedPayload, &canMsg.data[1], 5);
  byte correctedParity = canMsg.data[6];
  byte syndrome = 0;
  bool corrected = false;

  Serial.println("----------------------------------------");
  Serial.println("[HAMMING VERIFICATION]");
  bool hammingOK = hammingDecode46_40(correctedPayload, correctedParity, syndrome, corrected);

  if (!hammingOK)
  {
    Serial.print("SYNDROME         : "); Serial.println(syndrome);
    Serial.println("HAMMING STATUS   : INVALID CODEWORD");
    Serial.println("PACKET STATUS    : REJECTED");
    sendCumulativeACK();
    return;
  }

  if (corrected)
  {
    Serial.println("HAMMING STATUS   : SINGLE-BIT ERROR DETECTED");
    Serial.print("SYNDROME         : "); Serial.println(syndrome);
    Serial.print("ERROR BIT        : CODEWORD POSITION "); Serial.println(syndrome);
    Serial.println("CORRECTION       : SUCCESS");
    Serial.print("RECOVERED DATA   : "); printHexArray(correctedPayload, 5);
  }
  else
  {
    Serial.println("HAMMING STATUS   : VALID");
    Serial.println("ERROR DETECTED   : NO");
    Serial.println("CORRECTION       : NOT REQUIRED");
  }

  Serial.println("----------------------------------------");
  Serial.println("[XOR VERIFICATION]");
  byte calculatedXOR = calculateXOR(correctedPayload, 5);
  byte receivedXOR = canMsg.data[7];
  Serial.print("CALCULATED XOR   : 0x"); printHexByte(calculatedXOR); Serial.println();
  Serial.print("RECEIVED XOR     : 0x"); printHexByte(receivedXOR); Serial.println();

  if (calculatedXOR != receivedXOR)
  {
    Serial.println("XOR STATUS       : INVALID");
    Serial.println("ERROR TYPE       : CHECKSUM CORRUPTION");
    Serial.println("PACKET STATUS    : REJECTED");
    Serial.println("CUMULATIVE ACK   : REPEAT LAST IN-ORDER ACK");
    queueLCDPriority(LCD_CHECKSUM, seq);
    sendCumulativeACK();
    Serial.println("========================================");
    return;
  }

  Serial.println("XOR STATUS       : VALID");
  Serial.println("PACKET INTEGRITY : VERIFIED");

  if (seq == expectedSeq)
  {
    if (type == TYPE_ACC)
    {
      decodeGBNACC(correctedPayload);
      node1Received = true;
      lastNode1Heartbeat = millis();
    }
    else if (type == TYPE_GYRO)
    {
      decodeGBNGYRO(correctedPayload);
      node1Received = true;
      lastNode1Heartbeat = millis();
    }
    else
    {
      decodeGBNBMP(correctedPayload);
      node2Received = true;
      lastNode2Heartbeat = millis();
    }

    if (corrected)
    {
      // Hamming correction + XOR validation both succeeded and this is
      // the expected in-order packet: show its recovery sequence first.
      queueLCDPriority(LCD_CORRECTED, seq, syndrome);
      queueLCD(LCD_ACCEPT, seq);
      queueLCD(LCD_NO_RETX, seq);
    }
    else
    {
      queueLCD(LCD_NORMAL, seq, (byte)(seq + 1));
    }

    expectedSeq++;
    Serial.print("PACKET STATUS    : ACCEPTED | SEQ "); Serial.println(seq);
    Serial.print("NEXT EXPECTED    : "); Serial.println(expectedSeq);
    sendCumulativeACK();

    if (packetsTriggered >= GBN_WINDOW_SIZE &&
        expectedSeq == (byte)(windowBase + GBN_WINDOW_SIZE))
    {
      windowWaiting = false;
      queueLCD(LCD_COMPLETE);
      startGBNWindow();
    }
    Serial.println("========================================");
    return;
  }

  if ((byte)(seq - expectedSeq) < 128)
  {
    Serial.print("OUT-OF-ORDER      : DISCARD SEQ "); Serial.println(seq);
    sendCumulativeACK();
    Serial.println("========================================");
    return;
  }

  Serial.print("DUPLICATE / OLD SEQ: "); Serial.println(seq);
  sendCumulativeACK();
  Serial.println("========================================");
}

// ==========================================================
// STOP-WAIT SERIAL PRINT HELPERS
// ==========================================================
void printPayload(byte *data, byte length)
{
  for (byte i = 0; i < length; i++)
  {
    if (i) Serial.print(" ");
    printHexByte(data[i]);
  }
  Serial.println();
}

void printStopWaitIMUData(byte canID, byte *d)
{
  int16_t x = ((int16_t)d[0] << 8) | d[1];
  int16_t y = ((int16_t)d[2] << 8) | d[3];
  int16_t z = ((int16_t)d[4] << 8) | d[5];

  if (canID == NODE1_ACCEL_CAN_ID)
  {
    Serial.println();
    Serial.println("========== NODE 1: IMU ACC ==========");
    Serial.print("AX              : "); Serial.print(x / 100.0, 2); Serial.println(" m/s^2");
    Serial.print("AY              : "); Serial.print(y / 100.0, 2); Serial.println(" m/s^2");
    Serial.print("AZ              : "); Serial.print(z / 100.0, 2); Serial.println(" m/s^2");
  }
  else
  {
    Serial.println();
    Serial.println("========== NODE 1: IMU GYRO =========");
    Serial.print("GX              : "); Serial.print(x / 100.0, 2); Serial.println(" rad/s");
    Serial.print("GY              : "); Serial.print(y / 100.0, 2); Serial.println(" rad/s");
    Serial.print("GZ              : "); Serial.print(z / 100.0, 2); Serial.println(" rad/s");
  }
}

void printStopWaitBMPData(byte *d)
{
  int16_t temperature = ((int16_t)d[0] << 8) | d[1];
  uint32_t pressure =
    ((uint32_t)d[2] << 24) |
    ((uint32_t)d[3] << 16) |
    ((uint32_t)d[4] << 8) |
    d[5];

  Serial.println();
  Serial.println("========== NODE 2: BMP280 ==========");
  Serial.print("Temperature     : "); Serial.print(temperature / 100.0, 2); Serial.println(" C");
  Serial.print("Pressure        : "); Serial.print(pressure / 100.0, 2); Serial.println(" hPa");
}

void printStopWaitVerification(
  const char *nodeName,
  uint16_t canID,
  byte *data,
  byte received,
  byte calculated,
  bool valid,
  uint16_t ackID
)
{
  Serial.println();
  Serial.println("========== CHECKSUM VERIFICATION ==========");
  Serial.print("Node: "); Serial.println(nodeName);
  Serial.print("CAN ID: 0x"); Serial.println(canID, HEX);
  Serial.print("Payload: "); printPayload(data, 7);
  Serial.print("Checksum received: 0x"); printHexByte(received); Serial.println();
  Serial.print("Checksum calculated: 0x"); printHexByte(calculated); Serial.println();

  if (valid)
  {
    Serial.println("Checksum status: VALID");
  }
  else
  {
    Serial.println("CHECKSUM ERROR: MISMATCH");
    Serial.println("Packet rejected; ACK not sent.");
  }

  Serial.println("============================================");
}


void receiveStopWaitACC()
{
  if (canMsg.can_dlc != 7)
    return;

  byte received = canMsg.data[6];
  byte calculated = calculateXOR(canMsg.data, 6);

  if (received != calculated)
  {
    printStopWaitVerification(
      "NODE 1 - IMU",
      NODE1_ACCEL_CAN_ID,
      canMsg.data,
      received,
      calculated,
      false,
      NODE1_ACK_ID
    );
    return;
  }

  printStopWaitVerification(
    "NODE 1 - IMU",
    NODE1_ACCEL_CAN_ID,
    canMsg.data,
    received,
    calculated,
    true,
    NODE1_ACK_ID
  );

  decodeACC(canMsg.data);
  node1Received = true;
  lastNode1Heartbeat = millis();
  sendACK(NODE1_ACK_ID);
  printStopWaitIMUData(NODE1_ACCEL_CAN_ID, canMsg.data);
}

void receiveStopWaitGYRO()
{
  if (canMsg.can_dlc != 7)
    return;

  byte received = canMsg.data[6];
  byte calculated = calculateXOR(canMsg.data, 6);

  if (received != calculated)
  {
    printStopWaitVerification(
      "NODE 1 - IMU",
      NODE1_GYRO_CAN_ID,
      canMsg.data,
      received,
      calculated,
      false,
      NODE1_ACK_ID
    );
    return;
  }

  printStopWaitVerification(
    "NODE 1 - IMU",
    NODE1_GYRO_CAN_ID,
    canMsg.data,
    received,
    calculated,
    true,
    NODE1_ACK_ID
  );

  decodeGYRO(canMsg.data);
  node1Received = true;
  lastNode1Heartbeat = millis();
  sendACK(NODE1_ACK_ID);
  printStopWaitIMUData(NODE1_GYRO_CAN_ID, canMsg.data);
}

void receiveStopWaitBMP()
{
  if (canMsg.can_dlc != 7)
    return;

  byte received = canMsg.data[6];
  byte calculated = calculateXOR(canMsg.data, 6);

  if (received != calculated)
  {
    printStopWaitVerification(
      "NODE 2 - BMP280",
      NODE2_SENSOR_CAN_ID,
      canMsg.data,
      received,
      calculated,
      false,
      NODE2_ACK_ID
    );
    return;
  }

  printStopWaitVerification(
    "NODE 2 - BMP280",
    NODE2_SENSOR_CAN_ID,
    canMsg.data,
    received,
    calculated,
    true,
    NODE2_ACK_ID
  );

  decodeStopWaitBMP(canMsg.data);
  node2Received = true;
  lastNode2Heartbeat = millis();
  sendACK(NODE2_ACK_ID);
  printStopWaitBMPData(canMsg.data);
}

// ==========================================================
// SEND NORMAL ACK
// ==========================================================
void sendACK(uint16_t ackID)
{
  struct can_frame f;
  f.can_id = ackID;
  f.can_dlc = 0;

  for (byte i = 0; i < 8; i++)
    f.data[i] = 0;

  MCP2515::ERROR err = mcp2515.sendMessage(&f);

  if (err == MCP2515::ERROR_OK)
  {
    Serial.print("ACK sent: 0x");
    if (ackID < 0x10) Serial.print("0");
    Serial.println(ackID, HEX);
  }
  else
  {
    Serial.print("ACK send failed: 0x");
    if (ackID < 0x10) Serial.print("0");
    Serial.println(ackID, HEX);
  }
}

// ==========================================================
// PROCESS CAN
// ==========================================================

void processCAN()
{
  while (
    mcp2515.readMessage(&canMsg)
    == MCP2515::ERROR_OK
  )
  {
    // ------------------------------------------------------
    // HEARTBEAT
    // ------------------------------------------------------

    if (
      canMsg.can_id ==
      HEARTBEAT_ID
    )
    {
      if (
        canMsg.can_dlc >= 1
      )
      {
        if (
          canMsg.data[0] == 1
        )
        {
          node1Received = true;

          lastNode1Heartbeat =
            millis();
        }

        else if (
          canMsg.data[0] == 2
        )
        {
          node2Received = true;

          lastNode2Heartbeat =
            millis();
        }
      }

      continue;
    }

    // ------------------------------------------------------
    // GBN DATA
    // ------------------------------------------------------

    if (
      gbnMode &&
      (
        canMsg.can_id ==
        NODE1_ACCEL_CAN_ID ||

        canMsg.can_id ==
        NODE1_GYRO_CAN_ID ||

        canMsg.can_id ==
        NODE2_SENSOR_CAN_ID
      )
    )
    {
      receiveGBN();
      continue;
    }

    // ------------------------------------------------------
    // STOP-WAIT ACC
    // ------------------------------------------------------

    if (
      !gbnMode &&
      canMsg.can_id ==
      NODE1_ACCEL_CAN_ID
    )
    {
      receiveStopWaitACC();
      continue;
    }

    // ------------------------------------------------------
    // STOP-WAIT GYRO
    // ------------------------------------------------------

    if (
      !gbnMode &&
      canMsg.can_id ==
      NODE1_GYRO_CAN_ID
    )
    {
      receiveStopWaitGYRO();
      continue;
    }

    // ------------------------------------------------------
    // STOP-WAIT BMP
    // ------------------------------------------------------

    if (
      !gbnMode &&
      canMsg.can_id ==
      NODE2_SENSOR_CAN_ID
    )
    {
      receiveStopWaitBMP();
      continue;
    }
  }
}

// ==========================================================
// NODE TIMEOUT
// ==========================================================

void checkNodeTimeout()
{
  unsigned long now =
    millis();

  if (
    node1Received &&
    now -
    lastNode1Heartbeat >=
    NODE_TIMEOUT
  )
  {
    node1Received = false;

    Serial.println(
      "NODE 1 DISCONNECTED"
    );
  }

  if (
    node2Received &&
    now -
    lastNode2Heartbeat >=
    NODE_TIMEOUT
  )
  {
    node2Received = false;

    Serial.println(
      "NODE 2 DISCONNECTED"
    );
  }

  checkMode();
}

// ==========================================================
// GBN TRANSMISSION SCHEDULER
// ==========================================================
//
// SEQ 0 -> ACC
// SEQ 1 -> GYRO
// SEQ 2 -> BMP
//
// Then:
// SEQ 3 -> ACC
// SEQ 4 -> GYRO
// SEQ 5 -> BMP
//
// Window size = 3
// ==========================================================

void processGBNScheduler()
{
  if (!gbnMode)
    return;

  if (
    packetsTriggered >=
    GBN_WINDOW_SIZE
  )
  {
    return;
  }

  if (
    millis() <
    nextPacketTime
  )
  {
    return;
  }

  byte seq =
    windowBase +
    packetsTriggered;

  sendGBNTrigger(
    seq,
    false
  );

  packetsTriggered++;

  Serial.print(
    "ORIGINAL GBN PACKET "
  );

  Serial.print(seq);

  Serial.print(
    " TRIGGERED. COUNT="
  );

  Serial.println(
    packetsTriggered
  );

  nextPacketTime =
    millis() +
    GBN_PACKET_INTERVAL;

  // After third packet
  if (
    packetsTriggered ==
    GBN_WINDOW_SIZE
  )
  {
    windowWaiting = true;

    windowTimeoutStart =
      millis();
  }
}

// ==========================================================
// GBN TIMEOUT
// ==========================================================

void processGBNTimeout()
{
  if (!gbnMode)
    return;

  if (!windowWaiting)
    return;

  if (
    millis() -
    windowTimeoutStart <
    GBN_TIMEOUT
  )
  {
    return;
  }

  // --------------------------------------------------------
  // ALL RECEIVED
  // --------------------------------------------------------

  if (
    expectedSeq ==
    (byte)(
      windowBase +
      GBN_WINDOW_SIZE
    )
  )
  {
    windowWaiting = false;

    queueLCD(
      LCD_COMPLETE
    );

    startGBNWindow();

    return;
  }

  // --------------------------------------------------------
  // FIRST MISSING
  // --------------------------------------------------------

  byte firstMissing =
    expectedSeq;

  byte lastSeq =
    windowBase + 2;

  Serial.println();

  Serial.print(
    "SEQ TIMEOUT -> "
  );

  Serial.println(
    firstMissing
  );

  // --------------------------------------------------------
  // LCD RECOVERY
  // --------------------------------------------------------

  lcdRecoveryStartSeq =
    firstMissing;

  lcdRecoveryEndSeq =
    lastSeq;

  lcdRecoveryPhase = 1;
  queueLCDPriority(LCD_TIMEOUT, firstMissing);

  // --------------------------------------------------------
  // RETX LCD
  // --------------------------------------------------------

  byte second =
    firstMissing + 1;

  byte third =
    firstMissing + 2;

  queueLCD(
    LCD_RETX,
    firstMissing,
    second,
    third
  );

  Serial.print(
    "GBN RETRANSMIT FROM SEQ "
  );

  Serial.print(
    firstMissing
  );

  Serial.print(
    " THROUGH "
  );

  Serial.println(
    lastSeq
  );

  // --------------------------------------------------------
  // RETRANSMIT
  // --------------------------------------------------------

  byte seq =
    firstMissing;

  while (true)
  {
    sendGBNTrigger(
      seq,
      true
    );

    if (
      seq == lastSeq
    )
    {
      break;
    }

    seq++;

    delay(10);
  }

  windowTimeoutStart =
    millis();

  windowWaiting = true;
}

// ==========================================================
// SETUP
// ==========================================================

void setup()
{
  Serial.begin(115200);

  lcd.begin(16, 2);

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("CAN RECEIVER");

  lcd.setCursor(0, 1);
  lcd.print("Waiting...");

  SPI.begin();

  mcp2515.reset();

  mcp2515.setBitrate(
    CAN_500KBPS,
    MCP_8MHZ
  );

  mcp2515.setNormalMode();

  Serial.println();
  Serial.println(
    "================================"
  );

  Serial.println(
    "CAN RECEIVER"
  );

  Serial.println(
    "GBN WINDOW = 3"
  );

  Serial.println(
    "CUMULATIVE ACK ENABLED"
  );

  Serial.println(
    "STOP-WAIT BMP DECODER FIXED"
  );

  Serial.println(
    "NODE1 ACK ID = 0x037"
  );

  Serial.println(
    "NODE2 ACK ID = 0x039"
  );

  Serial.println(
    "================================"
  );
}

// ==========================================================
// LOOP
// ==========================================================

void loop()
{
  // CAN FIRST
  processCAN();

  // NODE STATUS
  checkNodeTimeout();

  // GBN
  if (gbnMode)
  {
    processGBNScheduler();

    processGBNTimeout();

    updateGBNLCD();
  }
  else
  {
    updateSingleNodeLCD();
  }
}
