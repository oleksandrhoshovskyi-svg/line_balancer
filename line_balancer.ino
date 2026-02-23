#include <Servo.h>

#define MOTOR_PIN  9
#define SENSOR_PIN A0

Servo motor;

bool control_enabled = true;

int motor_0 = 90;
int servo_min = 65;
int servo_max = 115;
const int OUT_MAX = 30;

#define USE_SOFT_LIMITS 1
const float SOFT1 = 11.0f;
const float SOFT2 = 13.0f;
const int OUT_SOFT1 = 18;
const int OUT_SOFT2 = 12;

float Kp = 1.3f;
float Ki = 0.15f;
float Kd = 0.35f;

float integral = 0.0f;
float last_error = 0.0f;
const float INTEGRAL_MAX = 300.0f;

uint32_t lastPID = 0;
uint32_t lastDebug = 0;
const uint32_t PID_PERIOD_MS = 20;
const uint32_t DEBUG_PERIOD_MS = 200;

float filtered_raw = 0.0f;
const float FILTER_ALPHA = 0.25f;

int dir = -1;
float target_pos = 0.0f;

const int NPTS = 5;
const float posTable[NPTS] = { -14.0f, -7.0f, 0.0f, 7.0f, 14.0f };
const float rawTable[NPTS] = {  388.0f, 278.0f, 220.0f, 181.0f, 159.0f };

void process_serial();
void PID();

float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}
int clampi(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

float read_sensor_raw() {
  long sum = 0;
  const int N = 6;
  for (int i = 0; i < N; i++) {
    sum += analogRead(SENSOR_PIN);
    delayMicroseconds(300);
  }
  return (float)sum / (float)N;
}

float raw_to_rodpos(float raw) {
  if (raw >= rawTable[0]) {
    float r0 = rawTable[0], r1 = rawTable[1];
    float p0 = posTable[0], p1 = posTable[1];
    float t = (raw - r0) / (r1 - r0);
    return p0 + t * (p1 - p0);
  }

  if (raw <= rawTable[NPTS - 1]) {
    float r0 = rawTable[NPTS - 2], r1 = rawTable[NPTS - 1];
    float p0 = posTable[NPTS - 2], p1 = posTable[NPTS - 1];
    float slope = (p1 - p0) / (r1 - r0);
    return p1 + slope * (raw - r1);
  }

  for (int i = 0; i < NPTS - 1; i++) {
    float r0 = rawTable[i];
    float r1 = rawTable[i + 1];

    if (raw <= r0 && raw >= r1) {
      float t = (raw - r0) / (r1 - r0);
      return posTable[i] + t * (posTable[i + 1] - posTable[i]);
    }
  }

  return posTable[NPTS - 1];
}

int pid_step(float error, float dt) {
  float P = Kp * error;

  integral += error * dt;
  integral = clampf(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
  float I = Ki * integral;

  float D = Kd * ((error - last_error) / dt);
  last_error = error;

  float out = P + I + D;

  int out_i = (int)out;
  out_i = clampi(out_i, -OUT_MAX, OUT_MAX);
  return out_i;
}

//https://docs.masso.com.au/wiring-and-setup/setup-and-calibration/soft-and-hard-limits
#if USE_SOFT_LIMITS
int local_out_max(float pos) {
  float a = (pos < 0.0f) ? -pos : pos;
  if (a > SOFT2) return OUT_SOFT2;
  if (a > SOFT1) return OUT_SOFT1;
  return OUT_MAX;
}
#endif

void process_serial() {
  static String cmd = "";

  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      cmd.trim();
      if (cmd.length() == 0) { cmd = ""; continue; }

      if (cmd.equalsIgnoreCase("STOP")) {
        control_enabled = false;
        motor.write(motor_0);
        Serial.println("STOP: control disabled");
      }
      else if (cmd.equalsIgnoreCase("RUN")) {
        control_enabled = true;
        integral = 0.0f;
        last_error = 0.0f;
        Serial.println("RUN: control enabled");
      }
      else if (cmd.startsWith("KP")) {
        Kp = cmd.substring(2).toFloat();
        Serial.print("Kp="); Serial.println(Kp, 6);
      }
      else if (cmd.startsWith("KI")) {
        Ki = cmd.substring(2).toFloat();
        integral = 0.0f;
        Serial.print("Ki="); Serial.println(Ki, 6);
      }
      else if (cmd.startsWith("KD")) {
        Kd = cmd.substring(2).toFloat();
        Serial.print("Kd="); Serial.println(Kd, 6);
      }
      else if (cmd.startsWith("M0")) {
        motor_0 = cmd.substring(2).toInt();
        servo_min = motor_0 - 25;
        servo_max = motor_0 + 25;
        motor.write(motor_0);
        Serial.print("motor_0="); Serial.println(motor_0);
      }
      else if (cmd.startsWith("DIR")) {
        int v = cmd.substring(3).toInt();
        dir = (v >= 0) ? 1 : -1;
        integral = 0.0f;
        last_error = 0.0f;
        Serial.print("dir="); Serial.println(dir);
      }
      else if (cmd.startsWith("T")) {
        target_pos = cmd.substring(1).toFloat();
        target_pos = clampf(target_pos, -14.0f, 14.0f);
        integral = 0.0f;
        last_error = 0.0f;
        Serial.print("target_pos="); Serial.println(target_pos, 2);
      }
      else {
        float v = cmd.toFloat();
        bool is_numeric = (cmd == "0" || cmd == "-0" || v != 0.0f);

        if (is_numeric) {
          target_pos = clampf(v, -14.0f, 14.0f);
          integral = 0.0f;
          last_error = 0.0f;
          Serial.print("target_pos="); Serial.println(target_pos, 2);
        } else {
          Serial.println("Unknown cmd. Use STOP, RUN, 10, -7, 0, KP.., KI.., KD.., M0.., DIR-1");
        }
      }

      cmd = "";
    } else {
      cmd += c;
    }
  }
}

//From the lectures and previous projects and also:
//https://ctms.engin.umich.edu/CTMS/index.php?example=BallBeam&section=ControlPID
void PID() {
  uint32_t now = millis();
  if (now - lastPID < PID_PERIOD_MS) return;

  uint32_t elapsed = now - lastPID;
  lastPID = now;

  float dt = elapsed / 1000.0f;
  if (dt <= 0.0f) dt = PID_PERIOD_MS / 1000.0f;

  float raw = read_sensor_raw();

  // https://stackoverflow.com/questions/10990618/c-filter-for-noisy-analog-signal
  filtered_raw = (1.0f - FILTER_ALPHA) * filtered_raw + FILTER_ALPHA * raw;

  if (!control_enabled) {
    motor.write(motor_0);
    return;
  }

  float pos = raw_to_rodpos(filtered_raw);
  float error = dir * (target_pos - pos);

  int out = pid_step(error, dt);

#if USE_SOFT_LIMITS
  int lim = local_out_max(pos);
  out = clampi(out, -lim, lim);
#endif

  int servo_angle = motor_0 + out;
  servo_angle = clampi(servo_angle, servo_min, servo_max);
  motor.write(servo_angle);
}

void setup() {
  Serial.begin(9600);
  delay(300);

  motor.attach(MOTOR_PIN);

  float r0 = read_sensor_raw();
  filtered_raw = r0;

  servo_min = motor_0 - 25;
  servo_max = motor_0 + 25;

  motor.write(motor_0);

  Serial.println("=== READY ===");
  Serial.println("Send number (e.g. 10) to change target, or use T10.");
  Serial.println("Other cmds: STOP RUN KP.. KI.. KD.. M0.. DIR..");
  Serial.print("Initial RAW="); Serial.println(r0, 1);

  target_pos = clampf(raw_to_rodpos(filtered_raw), -14.0f, 14.0f);
  Serial.print("Auto target_pos="); Serial.println(target_pos, 2);

  lastPID = millis();
  lastDebug = millis();
}

void loop() {
  process_serial();
  PID();

  uint32_t now = millis();
  if (now - lastDebug >= DEBUG_PERIOD_MS) {
    lastDebug = now;
    float pos = raw_to_rodpos(filtered_raw);

    Serial.print(control_enabled ? "EN " : "DIS ");
    Serial.print("RAW:"); Serial.print(filtered_raw, 1);
    Serial.print(" POS:"); Serial.print(pos, 2);
    Serial.print(" T:"); Serial.print(target_pos, 2);
    Serial.print(" dir:"); Serial.print(dir);
#if USE_SOFT_LIMITS
    Serial.print(" lim:"); Serial.print(local_out_max(pos));
#endif
    Serial.print(" servo0:"); Serial.print(motor_0);
    Serial.print(" Kp:"); Serial.print(Kp, 2);
    Serial.print(" Ki:"); Serial.print(Ki, 3);
    Serial.print(" Kd:"); Serial.println(Kd, 2);
  }
}
