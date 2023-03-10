#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <Adafruit_DPS310.h>
#include <Adafruit_BNO055.h>

//#define PRINT_DEBUG_DPS
//#define PRINT_DEBUG_BNO
//#define PRINT_DEBUG_GPS
//#define PRINT_DEBUG_ALTITUDE
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
const int YAW_PIN = 36;
const int FREQ_PIN = 39;

#pragma region CONTROL_DATA
int ladder_rotation = 0;
int elevator_rotation = 0;
#pragma endregion

#pragma region DPS310
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
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
float roll, pitch, yaw;
float standard_yaw = 0;

void InitBNO() {
  if (!bno.begin()) {
    Serial.println("Failed to find BNO");
  }
  Serial.println("BNO OK!");
  bno.setExtCrystalUse(false);
}
void GetBNO() {
  standard_yaw = (analogRead(YAW_PIN) - 2048) / 2048.0 * 180.0;

  // ?????????????????????????????????????????????????????????????????????
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  yaw = euler.x();
  pitch = euler.y();
  roll = euler.z();
#ifdef PRINT_DEBUG_BNO
  Serial.print(" ???DIR_xyz:");
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
#define GPSSerial Serial2
TinyGPSPlus gps;
uint32_t gps_latitude = 0;     // ?????? 10000000??? 359752780
uint32_t gps_longitude = 0;    // ?????? 10000000??? 1395238890
uint16_t gps_year = 0;     // ??????
uint8_t gps_month = 0;     // ???
uint8_t gps_day = 0;       // ???
uint16_t gps_hour = 0;     // ???
uint16_t gps_minute = 0;   // ???
uint16_t gps_second = 0;   // ???
uint32_t gps_altitude = 0;  // ?????? ???????????????
uint16_t gps_course = 0;    // ????????????(deg) 100???
uint16_t gps_speed = 0;  // ????????????(m/s) 1000??? ??????????????????????????????????????????

void InitGPS() {
  GPSSerial.begin(9600);
}
void GetGPS() {
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());
    if (gps.location.isUpdated()) {
      gps_latitude = (uint32_t)(gps.location.lat() * 10000000); // 10000000???
      gps_longitude = (uint32_t)(gps.location.lng() * 10000000); // 10000000???
      gps_year = (uint16_t)gps.date.year();
      gps_month = (uint8_t)gps.date.month();
      gps_day = (uint8_t)gps.date.day();
      gps_hour = gps.time.hour() + 9;  // ???????????????????????????
      gps_minute = gps.time.minute();
      gps_second = gps.time.second();
      gps_altitude = (uint32_t)gps.altitude.value(); // ?????? ???????????????
      gps_course = (uint16_t)gps.course.value(); // ????????????(deg) 100???
      gps_speed = (uint16_t)(gps.speed.mps() * 1000);
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
uint16_t altitude = 1000;    // ??????(cm)
uint16_t altitude_read = 0;  // ??????(???????????????????????????)
uint16_t diff_alti = 0;

double snapCurve(uint16_t x) {
  double y = 1 - 1 / ((double)x + 1);
  return y;
}

void GetAltitude() {
  altitude_read = 2 * analogRead(ALTITUDE_PIN);
  diff_alti = abs(altitude_read - altitude);
  altitude += (altitude_read - altitude) * snapCurve(diff_alti * 0.1);

#ifdef PRINT_DEBUG_ALTITUDE
  Serial.printf("Altitude: %d\n", altitude);
#endif
}
#pragma endregion

#pragma region TACHO
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
      air_speed = (tach_rotation * tach_rotation * -7.0 * pow(10, -16.0) + tach_rotation * 3.0 * pow(10, -7.0)) * 10.0;
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
bool log_state;
uint32_t log_start_time;
static unsigned char lastBtnSt = 0;   // ?????????????????????
static unsigned char fixedBtnSt = 0;  // ?????????????????????
static unsigned long smpltmr = 0;     // ??????????????????
File file;
#define PRINT_COMMA file.print(", ")

void SDWriteTask(void *pvParameters) {
  while (file && log_state) {
    file.print(millis() - log_start_time);
    PRINT_COMMA;
    file.print(gps_year);
    PRINT_COMMA;
    file.print(gps_month);
    PRINT_COMMA;
    file.print(gps_day);
    PRINT_COMMA;
    file.print(gps_hour);
    PRINT_COMMA;
    file.print(gps_minute);
    PRINT_COMMA;
    file.print(gps_second);
    PRINT_COMMA;
    file.print(gps_latitude);
    PRINT_COMMA;
    file.print(gps_longitude);
    PRINT_COMMA;
    file.print(gps_altitude);
    PRINT_COMMA;
    file.print(gps_speed);
    PRINT_COMMA;
    file.print(gps_speed);
    PRINT_COMMA;
    file.print(roll);
    PRINT_COMMA;
    file.print(pitch);
    PRINT_COMMA;
    file.print(yaw - standard_yaw);
    PRINT_COMMA;
    file.print(temperature);
    PRINT_COMMA;
    file.print(pressure);
    PRINT_COMMA;
    file.print(ground_pressure);
    PRINT_COMMA;
    file.print(dps_altitude);
    PRINT_COMMA;
    file.print(altitude);
    PRINT_COMMA;
    file.print(air_speed);
    PRINT_COMMA;
    file.print(propeller_rotation);
    PRINT_COMMA;
    file.print(ladder_rotation);
    PRINT_COMMA;
    file.print(elevator_rotation);
    PRINT_COMMA;
    file.println();
    delay(50);
  }
  file.close();
  Serial.println("File closed");
  vTaskDelete(NULL);
}
void StartSDWrite() {
  if (file || log_state) return;
  String path;
  int count = 0;
  do {
    path = String(count) + String(".csv");
    count++;
  } while (SD.exists(path));
  file = SD.open(path, FILE_WRITE);
  if (file) {
    Serial.println(path + " Opened");
    log_state = true;
  } else {
    Serial.println(path + " Open Failed");
    log_state = false;
  }
  log_start_time = millis();
  file.println("RunningTime, Year, Month, Day, Hour, Minute, Second, Latitude, Longitude, GPSAltitude, GPSCourse, GPSSpeed, Roll, Pitch, Yaw, Temperature, Pressure, GroundPressure, DPSAltitude, Altitude, AirSpeed, PropellerRotationSpeed, Ladder, Elevator");
  
  xTaskCreatePinnedToCore(SDWriteTask, "SDWriteTask", 4096, NULL, 1, NULL, 0);
}
void StopSDWrite() {
  log_state = false;
}
void UpdateLogState() {
  if (millis() - smpltmr < 10) return;
  smpltmr = millis();

  int btnSt = digitalRead(LOG_SW_PIN);
  int cmp = (btnSt == lastBtnSt);
  lastBtnSt = btnSt;

  if (!cmp) return;

  if (!btnSt && (btnSt != fixedBtnSt)) {
    fixedBtnSt = btnSt;

    if (!log_state) StartSDWrite();
    else StopSDWrite();
  }

  if (btnSt) {
    fixedBtnSt = btnSt;
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
WebServer server(80);

void handleRoot() {
  server.send(200, "text/plain", "WASA2023 Measurement");
}
void handleNotFound() {
  server.send(404, "text/plain", "Not Found");
}
void handleSetGroundPressure() {
  if (server.hasArg("Pressure")) {
    ground_pressure = server.arg("Pressure").toFloat();
  }
  server.send(200, "text/plain", String(ground_pressure));
}
void handleGetGroundPressure() {
  server.send(200, "text/plain", String(ground_pressure));
}
void handleSetServoRotation() {  // ???????????????????????????????????????????????????????????????????????????????????????
  if (server.hasArg("Ladder")) {
    ladder_rotation = server.arg("Ladder").toInt();
  }
  if (server.hasArg("Elevator")) {
    elevator_rotation = server.arg("Elevator").toInt();
  }
#ifdef PRINT_DEBUG_CONTROL
  Serial.printf("Ladder Rotation: %d\n", ladder_rotation);
  Serial.printf("Elevator Rotation: %d\n", elevator_rotation);
#endif
  String str;
  str += ladder_rotation;
  str += elevator_rotation;
  server.send(200, "text/plain", str);
}
void handleGetMeasurementData() {
  // JSON???????????????
  StaticJsonDocument<JSON_OBJECT_SIZE(50)> json_array;
  char json_string[4096];
  // JSON?????????????????????????????????????????????????????????
  json_array["RunningTime"] = millis();
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
  json_array["Roll"] = roll;
  json_array["Pitch"] = pitch;
  json_array["Yaw"] = yaw - standard_yaw;
  json_array["Temperature"] = temperature;
  json_array["Pressure"] = pressure;
  json_array["GroundPressure"] = ground_pressure;
  json_array["DPSAltitude"] = dps_altitude;
  json_array["Altitude"] = altitude;
  json_array["AirSpeed"] = air_speed;
  json_array["PropellerRotationSpeed"] = propeller_rotation;
  json_array["Ladder"] = ladder_rotation;
  json_array["Elevator"] = elevator_rotation;
  
  // JSON?????????????????????????????????????????????
  serializeJson(json_array, json_string, sizeof(json_string));

  server.send(200, "text/plain", json_string);
}

void InitServer() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP("WASA2023Measurement", "wasa2023");
  server.on("/", handleRoot);
  server.on("/SetGroundPressure", handleSetGroundPressure);
  server.on("/GetGroundPressure", handleGetGroundPressure);
  server.on("/SetServoRotation", handleSetServoRotation);
  server.on("/GetMeasurementData", handleGetMeasurementData);
  server.onNotFound(handleNotFound);
  server.begin();
}
#pragma endregion

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
  pinMode(ALTITUDE_PIN, ANALOG);
  pinMode(SLIDE_VL_PIN, ANALOG);
  pinMode(YAW_PIN, ANALOG);
  pinMode(FREQ_PIN, ANALOG);

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
  delay((int)(analogRead(FREQ_PIN) / 4.096));
#ifdef PRINR_DEBUG_LOOP
  Serial.printf("Current Time: %d\n", millis());
#endif
}