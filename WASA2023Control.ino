// #define LOBOT_DEBUG
// #define MINI_STICK

#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <SPI.h>
#include <EEPROM.h>

const int BOOT_PIN = 0;

const int WIFI_STATUS_SW_PIN = 19;
const int WIFI_STATUS_LED_PIN = 4;

const int LADDER_STICK_PIN = 33;  // 32 or 33
const int LADDER_CHECK_PIN = 13;
const int LADDER_MINI_STICK_PIN = 39;
const int ELEVATOR_STICK_PIN = 34;  // 34 or 35
const int ELEVATOR_CHECK_PIN = 12;
const int ELEVATOR_MINI_STICK_PIN = 36;

const int SCK_PIN = 18;   // CLK
const int LATCH_PIN = 5;  // CE
const int SDI_PIN = 23;   // MOSI
const byte DIGITS[] = {
  0b11111100,  // 0
  0b01100000,  // 1
  0b11011010,  // 2
  0b11110010,  // 3
  0b01100110,  // 4
  0b10110110,  // 5
  0b10111110,  // 6
  0b11100000,  // 7
  0b11111110,  // 8
  0b11110110,  // 9
};

const int BTN_SIZE = 7;
const int BTN_PIN[BTN_SIZE] = { 25, 26, 27, 14, 15, 21, 22 };  // ボタン入力ピン
enum {
  BTN_TRIM_UP = 0,
  BTN_TRIM_DOWN = 1,
  BTN_ROT_MODE = 2,
  BTN_DEF_LEFT = 3,
  BTN_DEF_RIGHT = 4,
  BTN_DEF_UP = 5,
  BTN_DEF_DOWN = 6
};
enum {
  ROT_MODE_LINEAR = 0,
  ROT_MODE_SINH1 = 1,
  ROT_MODE_SINH2 = 2,
  ROT_MODE_SINH3 = 3,
  ROT_MODE_MAX = 4
};
static int ROTATION_MODE = 0;
static unsigned char lastBtnSt[BTN_SIZE] = { 0 };   // 前回ボタン状態
static unsigned char fixedBtnSt[BTN_SIZE] = { 0 };  // 確定ボタン状態
static int btnPushCnt[BTN_SIZE] = { 0 };            // カウント数
static unsigned long smpltmr[BTN_SIZE] = { 0 };     // サンプル時間

const int SERVO_ID_LADDER = 2;
const int SERVO_ID_ELEVATOR = 3;
const int SERVO_BROADCAST = 254;

const int JOYSTICK_NEUTRAL = 2048;
const int SERVO_FREQUENCY = 100;

static int ladder_rotation = 0;
static int elevator_rotation = 0;
static int trim = 0;
static int step = 4;

WebServer server(80);
static int wifi_status;
const char *const STA_SSID = "WASA2023Measurement";
const char *const STA_PASS = "wasa2023";

#pragma region SERVO_LIBRARY
#define GET_LOW_BYTE(A) (uint8_t)((A))
//Aの下位バイト
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
//Aの上位バイト
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
//Aの上位バイトとBの下位バイトを合わせる

#define LOBOT_SERVO_FRAME_HEADER 0x55
#define LOBOT_SERVO_MOVE_TIME_WRITE 1
#define LOBOT_SERVO_MOVE_TIME_READ 2
#define LOBOT_SERVO_MOVE_TIME_WAIT_WRITE 7
#define LOBOT_SERVO_MOVE_TIME_WAIT_READ 8
#define LOBOT_SERVO_MOVE_START 11
#define LOBOT_SERVO_MOVE_STOP 12
#define LOBOT_SERVO_ID_WRITE 13
#define LOBOT_SERVO_ID_READ 14
#define LOBOT_SERVO_ANGLE_OFFSET_ADJUST 17
#define LOBOT_SERVO_ANGLE_OFFSET_WRITE 18
#define LOBOT_SERVO_ANGLE_OFFSET_READ 19
#define LOBOT_SERVO_ANGLE_LIMIT_WRITE 20
#define LOBOT_SERVO_ANGLE_LIMIT_READ 21
#define LOBOT_SERVO_VIN_LIMIT_WRITE 22
#define LOBOT_SERVO_VIN_LIMIT_READ 23
#define LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define LOBOT_SERVO_TEMP_MAX_LIMIT_READ 25
#define LOBOT_SERVO_TEMP_READ 26
#define LOBOT_SERVO_VIN_READ 27
#define LOBOT_SERVO_POS_READ 28
#define LOBOT_SERVO_OR_MOTOR_MODE_WRITE 29
#define LOBOT_SERVO_OR_MOTOR_MODE_READ 30
#define LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define LOBOT_SERVO_LOAD_OR_UNLOAD_READ 32
#define LOBOT_SERVO_LED_CTRL_WRITE 33
#define LOBOT_SERVO_LED_CTRL_READ 34
#define LOBOT_SERVO_LED_ERROR_WRITE 35
#define LOBOT_SERVO_LED_ERROR_READ 36


byte LobotCheckSum(byte buf[]) {
  byte i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++) {
    temp += buf[i];
  }
  temp = ~temp;
  i = (byte)temp;
  return i;
}

void LobotSerialServoMove(HardwareSerial &SerialX, uint8_t id, int16_t position, uint16_t time) {
  byte buf[10];
  if (position < 0)
    position = 0;
  if (position > 1000)
    position = 1000;
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(position);
  buf[6] = GET_HIGH_BYTE(position);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = LobotCheckSum(buf);
  SerialX.write(buf, 10);
}

void LobotSerialServoStopMove(HardwareSerial &SerialX, uint8_t id) {
  byte buf[6];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_MOVE_STOP;
  buf[5] = LobotCheckSum(buf);
  SerialX.write(buf, 6);
}

void LobotSerialServoSetID(HardwareSerial &SerialX, uint8_t oldID, uint8_t newID) {
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = oldID;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_ID_WRITE;
  buf[5] = newID;
  buf[6] = LobotCheckSum(buf);
  SerialX.write(buf, 7);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO ID WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++) {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
}

void LobotSerialServoSetMode(HardwareSerial &SerialX, uint8_t id, uint8_t Mode, int16_t Speed) {
  byte buf[10];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_OR_MOTOR_MODE_WRITE;
  buf[5] = Mode;
  buf[6] = 0;
  buf[7] = GET_LOW_BYTE((uint16_t)Speed);
  buf[8] = GET_HIGH_BYTE((uint16_t)Speed);
  buf[9] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO Set Mode");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++) {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  SerialX.write(buf, 10);
}
void LobotSerialServoLoad(HardwareSerial &SerialX, uint8_t id) {
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 1;
  buf[6] = LobotCheckSum(buf);

  SerialX.write(buf, 7);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO LOAD WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++) {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
}

void LobotSerialServoUnload(HardwareSerial &SerialX, uint8_t id) {
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 0;
  buf[6] = LobotCheckSum(buf);

  SerialX.write(buf, 7);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO LOAD WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++) {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
}


int LobotSerialServoReceiveHandle(HardwareSerial &SerialX, byte *ret) {
  bool frameStarted = false;
  bool receiveFinished = false;
  byte frameCount = 0;
  byte dataCount = 0;
  byte dataLength = 2;
  byte rxBuf;
  byte recvBuf[32];
  byte i;

  while (SerialX.available()) {
    rxBuf = SerialX.read();
    delayMicroseconds(100);
    if (!frameStarted) {
      if (rxBuf == LOBOT_SERVO_FRAME_HEADER) {
        frameCount++;
        if (frameCount == 2) {
          frameCount = 0;
          frameStarted = true;
          dataCount = 1;
        }
      } else {
        frameStarted = false;
        dataCount = 0;
        frameCount = 0;
      }
    }
    if (frameStarted) {
      recvBuf[dataCount] = (uint8_t)rxBuf;
      if (dataCount == 3) {
        dataLength = recvBuf[dataCount];
        if (dataLength < 3 || dataCount > 7) {
          dataLength = 2;
          frameStarted = false;
        }
      }
      dataCount++;
      if (dataCount == dataLength + 3) {

#ifdef LOBOT_DEBUG
        Serial.print("RECEIVE DATA:");
        for (i = 0; i < dataCount; i++) {
          Serial.print(recvBuf[i], HEX);
          Serial.print(":");
        }
        Serial.println(" ");
#endif

        if (LobotCheckSum(recvBuf) == recvBuf[dataCount - 1]) {

#ifdef LOBOT_DEBUG
          Serial.println("Check SUM OK!!");
          Serial.println("");
#endif

          frameStarted = false;
          memcpy(ret, recvBuf + 4, dataLength);
          return 1;
        }
        return -1;
      }
    }
  }
  return -1;
}


int LobotSerialServoReadPosition(HardwareSerial &SerialX, uint8_t id) {
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_POS_READ;
  buf[5] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO Pos READ");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++) {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  while (SerialX.available())
    SerialX.read();

  SerialX.write(buf, 6);

  while (!SerialX.available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2048;

#ifdef LOBOT_DEBUG
  Serial.println(ret);
#endif
  return ret;
}
int LobotSerialServoReadVin(HardwareSerial &SerialX, uint8_t id) {
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_VIN_READ;
  buf[5] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO VIN READ");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++) {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  while (SerialX.available())
    SerialX.read();

  SerialX.write(buf, 6);

  while (!SerialX.available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2049;

#ifdef LOBOT_DEBUG
  Serial.println(ret);
#endif
  return ret;
}
#pragma endregion

double curvature = 0;
int ToRotation(int x) {
  if (ROTATION_MODE == ROT_MODE_LINEAR) {
    return (int)((x - 2048.0) / 2048.0 * 250 + 500);
  } else {
    return (int)(sinh((x - 2048.0) / (2048.0 / curvature)) / sinh(curvature) * 250 + 500);
  }
}

void ServoTask(void *pvParameters) {
  while (true) {
    trim = (btnPushCnt[BTN_TRIM_UP] - btnPushCnt[BTN_TRIM_DOWN]);
    if (trim > 9) {
      btnPushCnt[BTN_TRIM_UP] = 9 + btnPushCnt[BTN_TRIM_DOWN];
      trim = 9;
    }
    if (trim < 0) {
      btnPushCnt[BTN_TRIM_DOWN] = btnPushCnt[BTN_TRIM_UP];
      trim = 0;
    }
    int t = 1000 / SERVO_FREQUENCY;
#ifdef MINI_STICK
    ladder_rotation = analogRead(LADDER_MINI_STICK_PIN);
    elevator_rotation = analogRead(ELEVATOR_MINI_STICK_PIN);
#else
    ladder_rotation = digitalRead(LADDER_CHECK_PIN) == LOW ? analogRead(LADDER_STICK_PIN) : JOYSTICK_NEUTRAL;
    elevator_rotation = digitalRead(ELEVATOR_CHECK_PIN) == LOW ? analogRead(ELEVATOR_STICK_PIN) : JOYSTICK_NEUTRAL;
#endif
    ladder_rotation = ToRotation(ladder_rotation) + GetDefaultAngle(SERVO_ID_LADDER) + btnPushCnt[BTN_DEF_LEFT] - btnPushCnt[BTN_DEF_RIGHT];
    elevator_rotation = ToRotation(elevator_rotation) + GetDefaultAngle(SERVO_ID_ELEVATOR) + trim * step + btnPushCnt[BTN_DEF_UP] - btnPushCnt[BTN_DEF_DOWN];
    LobotSerialServoMove(Serial2, SERVO_ID_LADDER, ladder_rotation, t);
    LobotSerialServoMove(Serial2, SERVO_ID_ELEVATOR, elevator_rotation, t);
    // Serial.println(ladder_rotation);
    // Serial.println(elevator_rotation);
    // Serial.println();
    delay(t);
  }
}

void SaveTask(void *pvParameters) {
  while (true) {
    if ((btnPushCnt[BTN_DEF_LEFT] | btnPushCnt[BTN_DEF_RIGHT] | btnPushCnt[BTN_DEF_UP] | btnPushCnt[BTN_DEF_DOWN]) != 0) {
      SetDefaultAngle(SERVO_ID_LADDER, GetDefaultAngle(SERVO_ID_LADDER) + btnPushCnt[BTN_DEF_LEFT] - btnPushCnt[BTN_DEF_RIGHT]);
      SetDefaultAngle(SERVO_ID_ELEVATOR, GetDefaultAngle(SERVO_ID_ELEVATOR) + btnPushCnt[BTN_DEF_UP] - btnPushCnt[BTN_DEF_DOWN]);
      Serial.printf("Left:%d, Right:%d, Up:%d, Down:%d\nPosition saved\n", btnPushCnt[BTN_DEF_LEFT], btnPushCnt[BTN_DEF_RIGHT], btnPushCnt[BTN_DEF_UP], btnPushCnt[BTN_DEF_DOWN]);
      btnPushCnt[BTN_DEF_LEFT] = btnPushCnt[BTN_DEF_RIGHT] = btnPushCnt[BTN_DEF_UP] = btnPushCnt[BTN_DEF_DOWN] = 0;
    }
    delay(1000);
  }
}

void SetDefaultAngle(unsigned char ID, int DefaultAngle) {
  if (ID == SERVO_BROADCAST) return;
  if (DefaultAngle > 700 || DefaultAngle < 300) return;
  // IDに応じて保存場所を変更
  EEPROM.put<int>(ID * sizeof(int), DefaultAngle);
  EEPROM.commit();
}

int GetDefaultAngle(unsigned char ID) {
  if (ID == SERVO_BROADCAST) return 0;
  int DefaultAngle = 0;
  EEPROM.get<int>(ID * sizeof(int), DefaultAngle);
  return DefaultAngle;
}

#pragma region SERVER_HANDLE
void handleRoot() {
  server.send(200, "text/plain", "WASA2023 Control");
}

void handleNotFound() {
  server.send(404, "text/plain", "Not Found");
}

void handleSetDefaultAngle() {
  if (server.hasArg("ID") && server.hasArg("Angle")) {
    unsigned char id = server.arg("ID").toInt();
    short angle = server.arg("Angle").toInt();
    SetDefaultAngle(id, angle);
    server.send(200, "text/plain", "Succeed");
  }
  server.send(200, "text/plain", "Failed");
}

void handleGetDefaultAngle() {
  if (server.hasArg("ID")) {
    unsigned char id = server.arg("ID").toInt();
    server.send(200, "text/plain", String(GetDefaultAngle(id)));
  }
  server.send(200, "text/plain", "Failed");
}

void handleSetTorque() {
  if (server.hasArg("ID") && server.hasArg("Torque")) {
    unsigned char id = server.arg("ID").toInt();
    server.send(200, "text/plain", "Succeed");
    int torque = server.arg("Torque").toInt();
    if (torque == 1) LobotSerialServoLoad(Serial2, id);
    if (torque == 0) LobotSerialServoUnload(Serial2, id);
    server.send(200, "text/plain", "Succeed");
  }
  server.send(200, "text/plain", "NULL");
}
#pragma endregion

void SendTask(void *pvParameters) {
  while (true) {
    if ((WiFi.status() == WL_CONNECTED)) {
      HTTPClient http;
      String url = "http://192.168.4.1/SetServoRotation?";
      url += String("Ladder=") + String(ladder_rotation);
      url += "&";
      url += String("Elevator=") + String(elevator_rotation);
      http.begin(url);
      // start connection and send HTTP header
      int httpCode = http.GET();
      // httpCode will be negative on error
      if (httpCode > 0) {
        // file found at server
        if (httpCode == HTTP_CODE_OK) {
          String payload = http.getString();
        }
      } else {
        Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
      }
      http.end();
    }
    delay(50);
  }
}

void btnEvent(int k) {
  if (millis() - smpltmr[k] < 10) return;
  smpltmr[k] = millis();

  int btnSt = digitalRead(BTN_PIN[k]);
  int cmp = (btnSt == lastBtnSt[k]);
  lastBtnSt[k] = btnSt;

  if (!cmp) return;

  if (!btnSt && (btnSt != fixedBtnSt[k])) {
    btnPushCnt[k]++;
    fixedBtnSt[k] = btnSt;
  }

  if (btnSt) {
    fixedBtnSt[k] = btnSt;
  }
}

void DisplayCurrent() {
  digitalWrite(LATCH_PIN, LOW);
  SPI.transfer(DIGITS[ROTATION_MODE]); // 回転モード（線形・非線形）の表示
  SPI.transfer(DIGITS[trim]);  // トリム値
  digitalWrite(LATCH_PIN, HIGH);
}

void setup() {
  // put your setup code here, to run once;
  Serial.begin(115200);
  Serial2.begin(115200);
  EEPROM.begin(1024);

  pinMode(BOOT_PIN, INPUT_PULLUP);
  pinMode(BTN_PIN[BTN_TRIM_UP], INPUT_PULLUP);
  pinMode(BTN_PIN[BTN_TRIM_DOWN], INPUT_PULLUP);
  pinMode(BTN_PIN[BTN_ROT_MODE], INPUT_PULLUP);
  pinMode(BTN_PIN[BTN_DEF_LEFT], INPUT_PULLUP);
  pinMode(BTN_PIN[BTN_DEF_RIGHT], INPUT_PULLUP);
  pinMode(BTN_PIN[BTN_DEF_UP], INPUT_PULLUP);
  pinMode(BTN_PIN[BTN_DEF_DOWN], INPUT_PULLUP);
  pinMode(WIFI_STATUS_SW_PIN, INPUT_PULLUP);
  pinMode(WIFI_STATUS_LED_PIN, OUTPUT);
  pinMode(LADDER_CHECK_PIN, INPUT_PULLUP);
  pinMode(ELEVATOR_CHECK_PIN, INPUT_PULLUP);
  pinMode(LADDER_STICK_PIN, ANALOG);
  pinMode(ELEVATOR_STICK_PIN, ANALOG);
  pinMode(LADDER_MINI_STICK_PIN, ANALOG);
  pinMode(ELEVATOR_MINI_STICK_PIN, ANALOG);
  pinMode(SCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(SDI_PIN, OUTPUT);

  digitalWrite(LATCH_PIN, HIGH);
  SPI.begin();
  SPI.setBitOrder(LSBFIRST);
  SPI.setDataMode(0);
  delay(100);

  SetDefaultAngle(2, 500);
  SetDefaultAngle(3, 500);

  if (digitalRead(WIFI_STATUS_SW_PIN) == HIGH) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("WASA2023Control", "wasa2023");
    server.on("/", handleRoot);
    server.on("/PostDefault", handleSetDefaultAngle);
    server.on("/GetDefault", handleGetDefaultAngle);
    server.on("/PostTorque", handleSetTorque);
    server.onNotFound(handleNotFound);
    server.begin();
    Serial.println("WIFI_AP_STA");
    wifi_status = true;
    digitalWrite(WIFI_STATUS_LED_PIN, HIGH);
  } else {
    WiFi.mode(WIFI_STA);
    Serial.println("WIFI_STA");
    wifi_status = false;
    digitalWrite(WIFI_STATUS_LED_PIN, LOW);
  }
  WiFi.begin(STA_SSID, STA_PASS);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) Serial.println("Connection Failed!");
  else Serial.println("Connection Succeed!");
  delay(100);

  xTaskCreatePinnedToCore(ServoTask, "ServoTask", 4096, NULL, 1, NULL, 0);
  delay(100);
  xTaskCreatePinnedToCore(SaveTask, "SaveTask", 4096, NULL, 1, NULL, 0);
  delay(100);
  xTaskCreatePinnedToCore(SendTask, "SendTask", 4096, NULL, 1, NULL, 0);
  delay(100);
}

void loop() {
  if (wifi_status) {
    server.handleClient();
  }
  for (int i = 0; i < BTN_SIZE; i++)
    btnEvent(i);

  ROTATION_MODE = btnPushCnt[BTN_ROT_MODE] % ROT_MODE_MAX;

  DisplayCurrent();

  if (digitalRead(BOOT_PIN) == LOW) {
    if (digitalRead(WIFI_STATUS_SW_PIN) == HIGH) {
      LobotSerialServoUnload(Serial2, SERVO_ID_LADDER);
      LobotSerialServoUnload(Serial2, SERVO_ID_ELEVATOR);
    } else {
      LobotSerialServoLoad(Serial2, SERVO_ID_LADDER);
      LobotSerialServoLoad(Serial2, SERVO_ID_ELEVATOR);
    }
  }
}