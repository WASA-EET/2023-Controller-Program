#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <Adafruit_DPS310.h>
#include <Adafruit_BNO055.h>

static int cadence = 0;
static int power = 0;

#define ENABLE_ALERT
// #define ENABLE_POWER

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <body>
  </body>
</html>
)rawliteral";


static int buzzer_code = 0;
enum {
  BUZZER_NONE,
  BUZZER_LOG_ON,
  BUZZER_LOG_OFF,
  BUZZER_MAX
};

//#define PRINT_DEBUG_DPS
//#define PRINT_DEBUG_BNO
//#define PRINT_DEBUG_GPS
#define PRINT_DEBUG_ALTITUDE
//#define PRINT_DEBUG_TACHO
//#define PRINT_DEBUG_RPM
//#define PRINT_DEBUG_CONTROL
//#define PRINR_DEBUG_LOOP

const int RPM_PIN = 4;
const int SDCARD_DRIVE = 5;
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const int DPS_PIN[4] = { 15, 14, 12, 13 };
const int ALTITUDE_PIN = 34;
const int TACHO_PIN[2] = { 32, 33 };
const int BUZZER_PIN = 25;
const int LOG_LED_PIN = 26;
const int LOG_SW_PIN = 27;
const int SLIDE_VL_PIN = 35;

float ladder_rotation = 0;
float elevator_rotation = 0;

#pragma region DPS310
// 気圧計による気圧、温度の測定及び高度の計算
Adafruit_DPS310 dps;
Adafruit_Sensor *dps_temp = dps.getTemperatureSensor();
Adafruit_Sensor *dps_pressure = dps.getPressureSensor();
float ground_pressure = 1013.25f;
float temperature = 0;
float pressure = 0;
float dps_altitude = 0;

void InitDPS() {
  if (!dps.begin_SPI(DPS_PIN[0], DPS_PIN[1], DPS_PIN[2], DPS_PIN[3])) {
    Serial.println("Failed to find DPS");
    return;
  }
  // Setup highest precision
  dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);

  dps_temp->printSensorDetails();
  dps_pressure->printSensorDetails();
  Serial.println("DPS OK!");
}
void GetDPS() {
  sensors_event_t temp_event, pressure_event;
  if (dps.temperatureAvailable()) {
    dps_temp->getEvent(&temp_event);
    temperature = temp_event.temperature;
#ifdef PRINT_DEBUG_DPS
    Serial.print("Temperature = ");
    Serial.print(temperature);
    Serial.println(" *C");
#endif
  }
  if (dps.pressureAvailable()) {
    dps_pressure->getEvent(&pressure_event);
    pressure = pressure_event.pressure;
#ifdef PRINT_DEBUG_DPS
    Serial.print("Pressure = ");
    Serial.print(pressure);
    Serial.println(" hPa");
#endif
  }
  dps_altitude = dps.readAltitude(ground_pressure);
#ifdef PRINT_DEBUG_DPS
  Serial.print("Altitude(DPS310) = ");
  Serial.print(dps_altitude);
  Serial.print(" m ");
  Serial.print("(Ground Pressure: ");
  Serial.print(ground_pressure);
  Serial.println(")");
  Serial.println();
#endif
}
#pragma endregion

#pragma region BNO055
// 9軸センサによる姿勢角の計算
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
float roll = 0, pitch = 0, yaw = 0;
float standard_roll = 0;
float standard_pitch = 0;
float standard_yaw = 0;

void InitBNO() {
  if (!bno.begin()) {
    Serial.println("Failed to find BNO");
    return;
  }
  Serial.println("BNO OK!");
  bno.setExtCrystalUse(false);
}
void GetBNO() {
  // センサフュージョンによる方向推定値の取得と表示
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  yaw = euler.x();
  pitch = euler.y() * -1;
  roll = euler.z();
#ifdef PRINT_DEBUG_BNO
  Serial.print("DIR_xyz:");
  Serial.print(yaw - standard_yaw);
  Serial.print(", ");
  Serial.print(pitch);
  Serial.print(", ");
  Serial.print(roll);
  Serial.println();
#endif
}
#pragma endregion

#pragma region GPS
// GPSの測定
#define GPSSerial Serial2
TinyGPSPlus gps;
double gps_latitude = 0;   // 緯度（小数第9位まで）
double gps_longitude = 0;  // 経度（小数第9位まで）
uint16_t gps_year = 0;       // 西暦
uint8_t gps_month = 0;       // 月
uint8_t gps_day = 0;         // 日
uint8_t gps_hour = 0;       // 時
uint8_t gps_minute = 0;     // 分
uint8_t gps_second = 0;     // 秒
double gps_altitude = 0;   // 高度 メートル単位
double gps_course = 0;     // 進行方向(deg)
double gps_speed = 0;      // 対地速度(m/s) 精度は高くないので参考程度に

void InitGPS() {
  GPSSerial.begin(9600);
}
void GetGPS() {
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());
    if (gps.location.isUpdated()) {
      gps_latitude = gps.location.lat();
      gps_longitude = gps.location.lng();
      gps_year = gps.date.year();
      gps_month = gps.date.month();
      gps_day = gps.date.day();
      gps_hour = (gps.time.hour() + 9) % 24;  // 時差を考慮すること
      gps_minute = gps.time.minute();
      gps_second = gps.time.second();
      gps_altitude = gps.altitude.meters();  // 高度 メートル単位
      gps_course = gps.course.deg();      // 進行方向(deg)
      gps_speed = gps.speed.mps();
#ifdef PRINT_DEBUG_GPS
      Serial.print("Latitude:  ");
      Serial.println(gps_latitude);
      Serial.print("Longitude: ");
      Serial.println(gps_longitude);
      Serial.print("Year: ");
      Serial.println(gps_year);
      Serial.print("Month: ");
      Serial.println(gps_month);
      Serial.print("Day: ");
      Serial.println(gps_day);
      Serial.print("Hour: ");
      Serial.println(gps_hour);
      Serial.print("Minute: ");
      Serial.println(gps_minute);
      Serial.print("Second: ");
      Serial.println(gps_second);
      Serial.print("Altitude: ");
      Serial.println(gps_altitude);
      Serial.print("Course: ");
      Serial.println(gps_course);
      Serial.print("Speed: ");
      Serial.println(gps_speed);
      Serial.println();
#endif
    }
  }
}
#pragma endregion

#pragma region ALTITUDE
// 超音波センサによる高度計のコード
uint16_t altitude = 1000;    // 高度(cm)
uint16_t altitude_read = 0;  // 高度(実際に読み取った値)
uint16_t diff_alti = 0;

double snapCurve(uint16_t x) {
  double y = 1 - 1 / ((double)x + 1);
  return y;
}

void GetAltitude() {
  altitude_read = analogRead(ALTITUDE_PIN) / 2 * 3.3 / 4.7; // アナログ入力で読み取る場合
  // altitude_read = pulseIn(ALTITUDE_PIN, HIGH, 124000) / 58.0;  // パルス入力で読み取る場合
  diff_alti = abs(altitude_read - altitude);
  altitude += (altitude_read - altitude) * snapCurve(diff_alti * 0.1);

#ifdef PRINT_DEBUG_ALTITUDE
  Serial.printf("Altitude: %d\n", altitude);
#endif
}
#pragma endregion

#pragma region TACHO
// 対気速度計のコード
hw_timer_t *timer = NULL;

volatile uint16_t tach_interrupts = 0;

boolean slit_rori_last = true;

uint32_t tach_rotation = 0;
uint32_t tach_last_time = 0;
uint32_t tach_delta_time = 0;

float air_speed = 30.0;

const uint32_t min_tach_delta = 150000;

void GetTacho() {
  tach_delta_time = micros() - tach_last_time;

  if (tach_delta_time > min_tach_delta) {
    noInterrupts();
    if (tach_interrupts > 5) {
      tach_rotation = (uint32_t)((double)1000000000.0 * ((double)tach_interrupts / tach_delta_time));
      air_speed = (tach_rotation * tach_rotation * -7.0 * pow(10, -16.0) + tach_rotation * 3.0 * pow(10, -7.0)) * 1.0;
    } else {
      tach_rotation = 0;
      air_speed = 0;
    }
    interrupts();
    tach_interrupts = 0;
    tach_last_time = micros();
#ifdef PRINT_DEBUG_TACHO
    Serial.printf("Air Speed: %f\n", air_speed);
#endif
  }
}

void IRAM_ATTR tach_interrupt_count() {
  if (digitalRead(TACHO_PIN[0]) != slit_rori_last) {
    tach_interrupts++;
    slit_rori_last = !slit_rori_last;
  }
}

void InitTacho() {
  interrupts();
  timer = timerBegin(0, getApbFrequency() / 1000000, true);
  timerAttachInterrupt(timer, &tach_interrupt_count, true);
  timerAlarmWrite(timer, 20, true);
  timerAlarmEnable(timer);
  tach_last_time = micros();
}
#pragma endregion

#pragma region ROTATION_SPEED
// 回転数系のコード
volatile uint16_t propeller_interrupts = 0;
uint32_t propeller_rotation = 106666;
uint32_t propeller_last_time = 0;
#define min_interrupts 5

uint32_t propeller_delta_time = 0;

const uint32_t min_propeller_delta = 500000;

void propeller_interrupt_count() {
  propeller_interrupts++;
}

void attachPropeller() {
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), propeller_interrupt_count, CHANGE);
}

void detachPropeller() {
  detachInterrupt(digitalPinToInterrupt(RPM_PIN));
}

void GetRPM() {
  propeller_delta_time = micros() - propeller_last_time;
  if (propeller_delta_time > min_propeller_delta) {
    detachPropeller();
    if (propeller_interrupts > min_interrupts) {
      propeller_rotation = (uint32_t)((double)1000000000.0 * ((double)propeller_interrupts / (double)propeller_delta_time) * 0.0009375);
    } else {
      propeller_rotation = 0;
    }
    attachPropeller();
    propeller_interrupts = 0;
    propeller_last_time = micros();
  }
#ifdef PRINT_DEBUG_RPM
  Serial.println(propeller_rotation);
  // r = 0.0009375 = 0.001 (i/ks -> i/s) * 60 (i/s -> i/m) / 64(i/m -> r/m)
#endif
}

void InitRPM() {
  attachPropeller();
  interrupts();
  propeller_last_time = micros();
}
#pragma endregion

#pragma region LOG
// ログをmicroSDに保存するためのシステム
bool log_state;
uint32_t log_start_time;
uint32_t log_sw_start = 0;  // スイッチを押し始めた時間
File fp;
#define PRINT_COMMA fp.print(", ")

void SDWriteTask(void *pvParameters) {
  while (fp && log_state) {
    fp.printf("%d/%d/%d", gps_year, gps_month, gps_day);
    PRINT_COMMA;
    fp.printf("%d:%02d:%02d", gps_hour, gps_minute, gps_second);
    PRINT_COMMA;
    fp.printf("%.9f", gps_latitude);
    PRINT_COMMA;
    fp.printf("%.9f", gps_longitude);
    PRINT_COMMA;
    fp.print(gps_altitude);
    PRINT_COMMA;
    fp.print(gps_speed);
    PRINT_COMMA;
    fp.print(gps_speed);
    PRINT_COMMA;
    fp.print(roll - standard_roll);
    PRINT_COMMA;
    fp.print(pitch - standard_pitch);
    PRINT_COMMA;
    fp.print(yaw - standard_yaw);
    PRINT_COMMA;
    fp.print(temperature);
    PRINT_COMMA;
    fp.print(pressure);
    PRINT_COMMA;
    fp.print(ground_pressure);
    PRINT_COMMA;
    fp.print(dps_altitude);
    PRINT_COMMA;
    fp.print(altitude);
    PRINT_COMMA;
    fp.print(air_speed);
    PRINT_COMMA;
    fp.print(propeller_rotation);
    PRINT_COMMA;
    fp.print(cadence);
    PRINT_COMMA;
    fp.print(power);
    PRINT_COMMA;
    fp.printf("%.3f", ladder_rotation);
    PRINT_COMMA;
    fp.printf("%.3f", elevator_rotation);
    PRINT_COMMA;
    fp.print((millis() - log_start_time) / 1000.0);
    PRINT_COMMA;
    fp.println();
    delay(50);
  }
  fp.close();
  Serial.println("File closed");
  buzzer_code = BUZZER_LOG_OFF;
  vTaskDelete(NULL);
}
void StartSDWrite() {
  if (log_state) return;
  String path;
  int count = 0;
  do {
    path = String("/") + String(count) + String(".csv");
    count++;
  } while (SD.exists(path));
  fp = SD.open(path, FILE_WRITE);
  if (fp) {
    Serial.println(path + " Opened");
    log_state = true;
    buzzer_code = BUZZER_LOG_ON;
  } else {
    Serial.println(path + " Open Failed");
    log_state = false;
    buzzer_code = BUZZER_LOG_OFF;
    return;
  }
  log_start_time = millis();
  // 基準値の設定
  ground_pressure = pressure;
  standard_roll = roll;
  standard_pitch = pitch;
  standard_yaw = yaw;
  fp.println("Date, Time, Latitude, Longitude, GPSAltitude, GPSCourse, GPSSpeed, Roll, Pitch, Yaw, Temperature, Pressure, GroundPressure, DPSAltitude, Altitude, AirSpeed, PropellerRotationSpeed, Cadence, Power, Ladder, Elevator, RunningTime");

  xTaskCreatePinnedToCore(SDWriteTask, "SDWriteTask", 4096, NULL, 1, NULL, 0);
}
void StopSDWrite() {
  log_state = false;
}
void UpdateLogState() {
  // 1秒以上押し続けたらログ収集を開始（または停止）する
  if (digitalRead(LOG_SW_PIN) == LOW) {
    if (log_sw_start == 0)
      log_sw_start = millis();
    long log_sw_time = millis() - log_sw_start;  // スイッチを押している時間

    if (log_sw_time > 1000) {
      log_sw_start = millis() + 1000000;  // 連続で反応しないようにする
      if (!log_state) StartSDWrite();
      else StopSDWrite();
    }
  } else {
    log_sw_start = 0;
  }

  digitalWrite(LOG_LED_PIN, log_state ? HIGH : LOW);
}
void InitSD() {
  log_state = false;
  if (!SD.begin(SDCARD_DRIVE)) {
    Serial.println("SD Drive does not work!");
    return;
  }
  Serial.println("SD OK!");
}
#pragma endregion

#pragma region SERVER
// 計測側APのパラメータ
const IPAddress localIP(192, 168, 4, 1);  // 自身のIPアドレス（操舵用マイコンのIPアドレスとは重複させないこと！）
const IPAddress gateway(192, 168, 4, 0);  // ゲートウェイ（ゲートウェイとサブネットマスクによるの支配区間が操舵と被らないようにすること）
const IPAddress subnet(255, 255, 255, 0); // サブネットマスク
// HTTPサーバーでの処理
WebServer server(80);

void handleRoot() {
  server.send(HTTP_CODE_OK, "text/html", index_html);
}
void handleLogStart() {
  StartSDWrite();
  server.send(HTTP_CODE_OK, "text/plain", String(log_state));
}
void handleLogStop() {
  StopSDWrite();
  server.send(HTTP_CODE_OK, "text/plain", String(log_state));
}
void handleNotFound() {
  server.send(HTTP_CODE_NOT_FOUND, "text/plain", "Not Found");
}
void handleSetGroundPressure() {
  if (server.hasArg("Pressure")) {
    ground_pressure = server.arg("Pressure").toFloat();
  }
  server.send(HTTP_CODE_OK, "text/plain", String(ground_pressure));
}
void handleGetGroundPressure() {
  server.send(HTTP_CODE_OK, "text/plain", String(ground_pressure));
}
void handleSetServoRotation() {  // 操舵から値を受信する。サーボが動くわけではないので注意
  if (server.hasArg("Ladder")) {
    ladder_rotation = server.arg("Ladder").toFloat();
  }
  if (server.hasArg("Elevator")) {
    elevator_rotation = server.arg("Elevator").toFloat();
  }
#ifdef PRINT_DEBUG_CONTROL
  Serial.printf("Ladder Rotation: %d\n", ladder_rotation);
  Serial.printf("Elevator Rotation: %d\n", elevator_rotation);
#endif
  String str;
  str += ladder_rotation;
  str += ", ";
  str += elevator_rotation;
  server.send(HTTP_CODE_OK, "text/plain", str);
}
void handleGetMeasurementData() {
  // JSONを作成する
  StaticJsonDocument<JSON_OBJECT_SIZE(50)> json_array;
  char json_string[4096];
  // JSONに変換したいデータを連想配列で指定する
  json_array["Year"] = gps_year;
  json_array["Month"] = gps_month;
  json_array["Day"] = gps_day;
  json_array["Hour"] = gps_hour;
  json_array["Minute"] = gps_minute;
  json_array["Second"] = gps_second;
  json_array["Latitude"] = gps_latitude;
  json_array["Longitude"] = gps_longitude;
  json_array["GPSAltitude"] = gps_altitude;
  json_array["GPSCourse"] = gps_course;
  json_array["GPSSpeed"] = gps_speed;
  json_array["Roll"] = roll - standard_roll;
  json_array["Pitch"] = pitch - standard_pitch;
  json_array["Yaw"] = yaw - standard_yaw;
  json_array["Temperature"] = temperature;
  json_array["Pressure"] = pressure;
  json_array["GroundPressure"] = ground_pressure;
  json_array["DPSAltitude"] = dps_altitude;
  json_array["Altitude"] = altitude;
  json_array["AirSpeed"] = air_speed;
  json_array["PropellerRotationSpeed"] = propeller_rotation;
  json_array["Cadence"] = cadence;
  json_array["Power"] = power;
  json_array["Ladder"] = ladder_rotation;
  json_array["Elevator"] = elevator_rotation;
  json_array["RunningTime"] = millis() / 1000.0;

  // JSONフォーマットの文字列に変換する
  serializeJson(json_array, json_string, sizeof(json_string));

  server.send(HTTP_CODE_OK, "text/plain", json_string);
}

void InitServer() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP("WASA2023Measurement", "wasa2023");
  WiFi.softAPConfig(localIP, gateway, subnet);
  server.on("/", handleRoot);
  server.on("/SetGroundPressure", handleSetGroundPressure);
  server.on("/GetGroundPressure", handleGetGroundPressure);
  server.on("/SetServoRotation", handleSetServoRotation);
  server.on("/LogStart", handleLogStart);
  server.on("/LogStop", handleLogStop);
  server.on("/GetMeasurementData", handleGetMeasurementData);
  server.onNotFound(handleNotFound);
  server.begin();
}
#pragma endregion

void BuzzerTask(void *pvParameters) {
  int alert = 0;
  while (true) {
    // ログの記録開始、終了時に音を鳴らした方が分かりやすいかなぁと
    if (buzzer_code == BUZZER_LOG_ON) {
      Serial.println("Buzzer log on");
      for (int i = 0; i < 2; i++) {
        dacWrite(BUZZER_PIN, 255);
        vTaskDelay(400);
        dacWrite(BUZZER_PIN, 0);
        vTaskDelay(100);
      }
    }
    if (buzzer_code == BUZZER_LOG_OFF) {
      for (int i = 0; i < 4; i++) {
        dacWrite(BUZZER_PIN, 255);
        vTaskDelay(200);
        dacWrite(BUZZER_PIN, 0);
        vTaskDelay(100);
      }
    }
    if (buzzer_code != BUZZER_NONE)
      buzzer_code = BUZZER_NONE;

    if (abs(roll) > 10.0) alert++;
    else alert = 0;
    dacWrite(BUZZER_PIN, alert >= 1 ? analogRead(SLIDE_VL_PIN) / 16 : 0);
    delay(100);
  }
}

void PowerTask(void *pvParameters) {
  while (true) {
    if (Serial1.read() == 255) {
      uint8_t a = Serial1.read(), b = Serial1.read();
      power = (a << 8) | b;
      cadence = Serial1.read();
    }
    delay(50);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(RPM_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  pinMode(LOG_LED_PIN, OUTPUT);
  pinMode(LOG_SW_PIN, INPUT_PULLUP);
  pinMode(TACHO_PIN[0], INPUT_PULLUP);
  pinMode(TACHO_PIN[1], INPUT_PULLUP);
  pinMode(ALTITUDE_PIN, ANALOG);  // PULSE入力ならINPUT, アナログ入力ならANALOG
  pinMode(SLIDE_VL_PIN, ANALOG);

  InitDPS();
  delay(100);
  InitBNO();
  delay(100);
  InitGPS();
  delay(100);
  InitTacho();
  delay(100);
  InitRPM();
  delay(100);
  InitSD();
  delay(100);
  InitServer();
  delay(100);
#ifdef ENABLE_ALERT
  xTaskCreatePinnedToCore(BuzzerTask, "BuzzerTask", 4096, NULL, 1, NULL, 0);
  delay(100);
#endif
#ifdef ENABLE_POWER
  Serial1.begin(115200, SERIAL_8N1, 2, 0);  // RX = GPIO2, TX = GPIO0
  xTaskCreatePinnedToCore(PowerTask, "PowerTask", 4096, NULL, 1, NULL, 0);
#endif
  delay(100);
}

void loop() {
  server.handleClient();
  GetDPS();
  GetBNO();
  GetGPS();
  GetAltitude();
  GetTacho();
  GetRPM();
  UpdateLogState();
#ifdef PRINR_DEBUG_LOOP
  Serial.printf("Current Time: %d\n", millis());
#endif
}
