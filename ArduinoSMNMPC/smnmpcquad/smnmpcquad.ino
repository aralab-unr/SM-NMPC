#include <Wire.h>
#include <Arduino.h>
#include <sbus.h> 
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#define SBUS_BAUD_RATE 100000
bfs::SbusRx sbus_rx(&Serial5);

float vphi, vtheta, vpsi;
float vphicalibration, vthetacalibration, vpsicalibration;
int RateCalibrationNumber;
float AccX, AccY, AccZ;

float phi, theta, psi;
uint32_t LoopTimer;

float phikalman = 0, phiunkalman = 4;
float thetakalman = 0, thetaunkalman = 4;
float Kalman1DOutput[2];

double m = 1.85, g = 9.8;  
double degtorad = 0.01745;

float ch0 = 1001, ch1 = 1000, ch2 = 180, ch3 = 988, ch4 = 172;
double phir, thetar, psir;
double phird = 0, thetard = 0, psird = 0;

double cphi_ctrl = 2.5, ctheta_ctrl = 2.5, cpsi_ctrl = 3.5;
double Kx = 2.75, Ky = 2.75, Kz = 2.25;
double ixx = 0.0785, iyy = 0.0785, izz = 0.105;
double bphi = 1 / ixx;
double btheta = 1 / iyy;
double bpsi = 1 / izz;
double Kphi = 0.05, Ktheta = 0.05, Kpsi = 0.15;

double cx = 0.015, cy = 0.015, cz = 0.5;
double lam1 = 0.005, lam2 = 0.005, Ka = 0.15, eta = 0.005;
double Kdx = 0.0000267, Kdy = 0.0000267, Kdz = 0.0000625;

double pwm_value1, pwm_value2, pwm_value3, pwm_value4;

#define NX          ACADO_NX    // Number of differential states
#define NXA         ACADO_NXA   // Number of algebraic states
#define NU          ACADO_NU    // Number of control inputs
#define N           ACADO_N     // Number of control intervals
#define NOD         ACADO_NOD   // Number of online data
#define NY          ACADO_NY    // Number of references, nodes 0..N - 1
#define NYN         ACADO_NYN   // Number of references for node N
#define NUM_STEPS   1           // Number of simulation steps
#define VERBOSE     1           // Show iterations: 1, Silent: 0

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

double x = 0, y = 0, z = 0;
double xd = 0, yd = 0, zd = 0;
double xr = 0, yr = 0;
unsigned long lastOpticalTime = 0;
#define MICOLINK_MSG_HEAD 0xEF
#define MICOLINK_MAX_PAYLOAD_LEN 64
#define MICOLINK_MAX_LEN (MICOLINK_MAX_PAYLOAD_LEN + 7)
enum {
    MICOLINK_MSG_ID_RANGE_SENSOR = 0x51,  // Range Sensor
};
// MICOLINK Message Structure
typedef struct {
    uint8_t head;
    uint8_t dev_id;
    uint8_t sys_id;
    uint8_t msg_id;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[MICOLINK_MAX_PAYLOAD_LEN];
    uint8_t checksum;

    uint8_t status;
    uint8_t payload_cnt;
} MICOLINK_MSG_t;

// MICOLINK Payload for Range Sensor
#pragma pack(push, 1)
typedef struct {
    uint32_t time_ms;
    uint32_t distance;
    uint8_t strength;
    uint8_t precision;
    uint8_t dis_status;
    uint8_t reserved1;
    int16_t flow_vel_x;
    int16_t flow_vel_y;
    uint8_t flow_quality;
    uint8_t flow_status;
    uint16_t reserved2;
} MICOLINK_PAYLOAD_RANGE_SENSOR_t;
#pragma pack(pop)

void setup() {
  Serial.begin(115200);

  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    vphicalibration += vphi;
    vthetacalibration += vtheta;
    vpsicalibration += vpsi;
    delay(1);
  }
  
  vphicalibration /= 2000;
  vthetacalibration /= 2000;
  vpsicalibration /= 2000;

  Serial5.begin(SBUS_BAUD_RATE, SERIAL_8E2);
  sbus_rx.Begin();
  Serial3.begin(115200);

  acado_initializeSolver();

  analogWriteFrequency(3, 100);
  analogWriteFrequency(4, 100);
  analogWriteFrequency(5, 100);
  analogWriteFrequency(6, 100);
  analogWriteResolution(12);

  analogWrite(3, 409); // motor 1
  analogWrite(4, 409); //motor 2
  analogWrite(5, 409); // motor 3
  analogWrite(6, 409); //motor 4
  delay(3000);

  LoopTimer=micros();
  lastOpticalTime = millis();
}

void loop() {

  gyro_signals();
  read_receiver();
  getoptical();

  vphi -= vphicalibration;
  vtheta -= vthetacalibration;
  vpsi -= vpsicalibration;

  kalman_1d(phikalman, phiunkalman, vphi, phi);
  kalman_1d(thetakalman, thetaunkalman, vtheta, theta);

  double vphirad = vphi * degtorad;
  double vthetarad = vtheta * degtorad;
  double vpsirad = vpsi * degtorad;
  double phirad = phikalman * degtorad;
  double thetarad = thetakalman * degtorad;
  double psirad = psi * degtorad;

  double zr = map(ch2, 180, 1811, 0.0, 2.0);
  xr = xr + (1001 - ch1) / 800 * 0.01;
  yr = yr + (1000 - ch0) / 800 * 0.01;

  double kdx = 0.15, kdy = 0.15;
  double kdvx = 0.2, kdvy = 0.2;
  double thetar = kdx * (xr - x / 1000) - kdvx * xd / 1000;
  double phir = kdy * (y / 1000 - yr) + kdvy * yd / 1000;
  phir = constrain(phir, -0.2, 0.2);
  thetar = constrain(thetar, -0.2, 0.2);

  double psir = 0.005 * (ch3 - 988);

  phird = 2.275 * (phir - phirad);
  thetard = 2.275 * (thetar - thetarad);
  psird = 1.5 * (psir - psirad);

  double fphi = vphirad * vthetarad * (iyy - izz) / ixx;
  double ftheta = vpsirad * vphirad * (izz - ixx) / iyy;
  double fpsi = vphirad * vthetarad * (ixx - iyy) / izz;

  double scphi = cphi_ctrl * (phirad - phir) + (vphirad - phird);
  double sctheta = ctheta_ctrl * (thetarad - thetar) + (vthetarad - thetard);
  double scpsi = cpsi_ctrl * (psirad - psir) + (vpsirad - psird);

  double satphi = constrain(scphi, -0.1, 0.1);
  double sattheta = constrain(sctheta, -0.1, 0.1);
  double satpsi = constrain(scpsi, -0.1, 0.1);

  double U2s = (-Ky * cphi_ctrl * phirad - (cphi_ctrl + Ky) * vphirad + Ky * cphi_ctrl * phir + (cphi_ctrl + Ky) * phird - fphi - Kphi * satphi) / bphi;
  double U3s = (-Kx * ctheta_ctrl * thetarad - (ctheta_ctrl + Kx) * vthetarad + Kx * ctheta_ctrl * thetar + (ctheta_ctrl + Kx) * thetard - ftheta - Ktheta * sattheta) / btheta;
  double U4s = (-Kz * cpsi_ctrl * psirad - (cpsi_ctrl + Kz) * vpsirad + Kz * cpsi_ctrl * psir + (cpsi_ctrl + Kz) * psird - fpsi - Kpsi * satpsi) / bpsi;

  double U2smc = constrain(U2s, -1.5, 1.5);
  double U3smc = constrain(U3s, -1.5, 1.5);
  double U4smc = constrain(U4s, -1.5, 1.5);

  double cphi = cos(phirad), sphi = sin(phirad);
  double ctheta = cos(thetarad), stheta = sin(thetarad);
  double cpsi = cos(psirad), spsi = sin(psirad);

  double fx = -Kdx * xd / (1000 * m);
  double fy = -Kdy * yd / (1000 * m);
  double fz = ( -Kdz * zd / 1000 - m * g) / m;
  double bx = 1 / m * (spsi * sphi + cpsi * stheta * cphi);
  double by = 1 / m * (spsi * stheta * cphi - cpsi * sphi);
  double bz = 1 / m * (ctheta * cphi);

  double xrd = 0.05 * (xr - x / 1000);
  double yrd = 0.05 * (yr - y / 1000);
  double zrd = 2 * (zr - z / 1000);

  double sx = cx * (xr - x / 1000) + (xrd - xd / 1000);
  double sy = cy * (yr - y / 1000) + (yrd - yd / 1000);
  double sz = cz * (zr - z / 1000) + 2 * (zrd - zd / 1000);

  double ueqx = (cx * (xrd - xd / 1000) - fx) / bx;
  double ueqy = (cy * (yrd - yd / 1000) - fy) / by;
  double ueqz = (cz * (zrd - zd / 1000) - fz) / bz;

  double s3 = lam2 * lam1 * sx + lam2 * sy + sz;
  double sats3 = constrain(s3, -0.1, 0.1);

  double usw = -(lam2 * lam1 * bx * (ueqy + ueqz) + lam2 * by * (ueqx + ueqz) + bz * (ueqx + ueqy) - Ka * s3 - eta * sats3) / (lam2 * lam1 * bx + lam2 * by + bz);
  double Uz = ueqx + ueqy + ueqz + usw;
  double U1smc = constrain(Uz, 15.5, 21.5);
  unsigned int i;

  // Reset all solver memory
  memset(&acadoWorkspace, 0, sizeof(acadoWorkspace));
  memset(&acadoVariables, 0, sizeof(acadoVariables));

  // Prepare a consistent initial guess
  for (i = 0; i < N + 1; ++i) {
      acadoVariables.x[i * NX + 0] = x / 1000;
      acadoVariables.x[i * NX + 1] = y / 1000;
      acadoVariables.x[i * NX + 2] = z / 1000;
      acadoVariables.x[i * NX + 3] = phirad;
      acadoVariables.x[i * NX + 4] = thetarad;
      acadoVariables.x[i * NX + 5] = psirad;
      acadoVariables.x[i * NX + 6] = xd / 1000;
      acadoVariables.x[i * NX + 7] = yd / 1000;
      acadoVariables.x[i * NX + 8] = zd / 1000;
      acadoVariables.x[i * NX + 9] = vphirad;
      acadoVariables.x[i * NX + 10] = vthetarad;
      acadoVariables.x[i * NX + 11] = vpsirad;
  }

  // Prepare references
  for (i = 0; i < N; ++i) {
      acadoVariables.y[i * NY + 0] = zr;
      acadoVariables.y[i * NY + 1] = phir;
      acadoVariables.y[i * NY + 2] = thetar; 
      acadoVariables.y[i * NY + 3] = psir ;
      acadoVariables.y[i * NY + 4] = m * g;
      acadoVariables.y[i * NY + 5] = U2smc;
      acadoVariables.y[i * NY + 6] = U3smc;
      acadoVariables.y[i * NY + 7] = U4smc;
  }

  acadoVariables.yN[0] = zr;
  acadoVariables.yN[1] = phir; 
  acadoVariables.yN[2] = thetar; 
  acadoVariables.yN[3] = psir;

  for (i = 0; i < NX; ++i) {
      acadoVariables.x0[i] = acadoVariables.x[i];
  }

  // Perform ACADO steps
  acado_preparationStep();
  acado_feedbackStep();
  acado_shiftStates(2, 0, 0);
  acado_shiftControls(0);

  // Retrieve control variables
  real_t* u = acado_getVariablesU();
  
  //SMC 
  //double U1r = map(U1smc - m*g, -3.5, 3.5, -350, 350);
  //double U1manual = map(ch2, 180, 1811, 1000, 1800);
  //double U2r = 125 * U2smc;
  //double U3r = 125 * U3smc;
  //double U4r = 125 * U4smc;

  //NMPC
  double U1r = map(u[0] - m*g, -5, 5, -300, 300);
  double U1manual = map(ch2, 180, 1811, 1000, 1800);
  double U2r = 100 * u[1];
  double U3r = 100 * u[2];
  double U4r = 100 * u[3];

  if (ch4 < 700) {
    pwm_value1 = constrain(0.409 * (1480 + U1r - U2r - U3r - U4r), 409, 800);
    pwm_value2 = constrain(0.409 * (1480 + U1r + U2r - U3r + U4r), 409, 800);
    pwm_value3 = constrain(0.409 * (1480 + U1r - U2r + U3r + U4r), 409, 800);
    pwm_value4 = constrain(0.409 * (1480 + U1r + U2r + U3r - U4r), 409, 800);
  }
  else if (ch4 >= 700 && ch4 < 1000) {
    pwm_value1 = constrain(0.409 * (U1manual - U2r - U3r - U4r), 409, 800);
    pwm_value2 = constrain(0.409 * (U1manual + U2r - U3r + U4r), 409, 800);
    pwm_value3 = constrain(0.409 * (U1manual - U2r + U3r + U4r), 409, 800);
    pwm_value4 = constrain(0.409 * (U1manual + U2r + U3r - U4r), 409, 800);
  }
  else {
    pwm_value1 = 409;
    pwm_value2 = 409;
    pwm_value3 = 409;
    pwm_value4 = 409; 
  }
  analogWrite(3, pwm_value1); // motor 1
  analogWrite(5, pwm_value2); //motor 2
  analogWrite(4, pwm_value3); // motor 3
  analogWrite(6, pwm_value4); //motor 4

  Serial.print(x); Serial.print(","); 
  Serial.print(y); Serial.print(",");
  Serial.print(z); Serial.print(",");
  Serial.print(xr); Serial.print(","); 
  Serial.print(yr); Serial.print(",");
  Serial.print(zr); Serial.print(",");
  Serial.print(phirad); Serial.print(",");
  Serial.print(thetarad); Serial.print(",");
  Serial.print(psi); Serial.print(",");
  Serial.print(u[0]); Serial.print(",");
  Serial.print(u[1]); Serial.print(",");
  Serial.print(u[2]); Serial.print(",");
  Serial.print(u[3]); Serial.print(",");
  Serial.print(pwm_value1); Serial.print(",");
  Serial.print(pwm_value2); Serial.print(",");
  Serial.print(pwm_value3); Serial.print(",");
  Serial.print(pwm_value4); Serial.print(",");
  Serial.print(phir); Serial.print(",");
  Serial.print(thetar); Serial.print(",");
  Serial.print(psir); Serial.println(";");

  float dt = (micros() - LoopTimer) / 1000000.0;
  psi = psi + dt * vpsi;

  while (micros() - LoopTimer < 10000);
  LoopTimer = micros();
}

void read_receiver() {
    if (sbus_rx.Read()) {
        bfs::SbusData data = sbus_rx.data();
        ch0 = data.ch[0];
        ch1 = data.ch[1];
        ch2 = data.ch[2];
        ch3 = data.ch[3];
        ch4 = data.ch[4];
    }
}

void kalman_1d(float &KalmanState, float &KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + 0.01 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.01 * 0.01 * 4 * 4;
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 9); 
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
}

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  
  vphi = (float)GyroX / 65.5;
  vtheta = (float)GyroY / 65.5;
  vpsi = (float)GyroZ / 65.5;
  
  AccX = (float)AccXLSB / 4096 - 0.04;
  AccY = (float)AccYLSB / 4096 - 0.01;
  AccZ = (float)AccZLSB / 4096 - 0.21;
  
  phi = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * (180 / PI);
  theta = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * (180 / PI);
}

void getoptical() {
    while (Serial3.available()) {
        uint8_t sensor_data = Serial3.read();
        micolink_decode(sensor_data);
    }
}

void micolink_decode(uint8_t data) {
    static MICOLINK_MSG_t msg;

    if (!micolink_parse_char(&msg, data)) {
        return;
    }

    if (msg.msg_id == MICOLINK_MSG_ID_RANGE_SENSOR) {
        MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;
        memcpy(&payload, msg.payload, msg.len);
        unsigned long currentOpticalTime = millis();
        float dtOptical = (currentOpticalTime - lastOpticalTime) / 1000.0;

        if (payload.distance < 8000 && abs(payload.flow_vel_x) < 1000 && abs(payload.flow_vel_y) < 1000) {
            if (dtOptical > 0) {
                zd = (payload.distance - z) / dtOptical;
                z = payload.distance;
                //MTF-01
                //xd = - payload.flow_vel_y * z * 0.01;
                //yd = - payload.flow_vel_x * z * 0.01;

                //MTF-02
                xd = payload.flow_vel_x * z * 0.01;
                yd = -payload.flow_vel_y * z * 0.01;
                
                x += xd * dtOptical;
                y += yd * dtOptical;
                lastOpticalTime = currentOpticalTime;
            }
        }
    }
}

bool micolink_parse_char(MICOLINK_MSG_t* msg, uint8_t data) {
    switch (msg->status) {
        case 0: 
            if (data == MICOLINK_MSG_HEAD) {
                msg->head = data;
                msg->status = 1;
            }
            break;
        case 1: msg->dev_id = data; msg->status = 2; break;
        case 2: msg->sys_id = data; msg->status = 3; break;
        case 3: msg->msg_id = data; msg->status = 4; break;
        case 4: msg->seq = data; msg->status = 5; break;
        case 5:
            msg->len = data;
            if (msg->len == 0) {
                msg->status = 7;
            } else if (msg->len > MICOLINK_MAX_PAYLOAD_LEN) {
                msg->status = 0;
            } else {
                msg->status = 6;
            }
            break;
        case 6:
            msg->payload[msg->payload_cnt++] = data;
            if (msg->payload_cnt == msg->len) {
                msg->payload_cnt = 0;
                msg->status = 7;
            }
            break;
        case 7: 
            msg->checksum = data;
            msg->status = 0;
            return true;
        default: 
            msg->status = 0;
            msg->payload_cnt = 0;
            break;
    }
    return false;
}

