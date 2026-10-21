// =====================================================================
//  ROCKET FLIGHT COMPUTER  -  Teensy 4.1 + Ra-01H (SX1276)
//  Team 2026 IN-SPACe-010  -  PHOENIX dB.V1 avionics
//
//  Pins: from the schematic print (BAKE AKKA.pdf), 2026-10-19
//    Wire  (SDA 18, SCL 19)  PCA9685 0x40 (OE tied to GND)
//    Wire1 (SDA 17, SCL 16)  MS5611 (CSB high -> 0x76, 0x77 also probed), ADXL345 0x53 (SDO GND),
//                            INA219 0x40 (VIN+ = 5V+ from buck, VIN- = 5V_A)
//    Wire2 (SDA 25, SCL 24)  BNO08x 0x4B (PS0/PS1 GND = I2C), RST 26, INT 23
//    ADXL345 INT1 21, INT2 22 (not used: FIFO is polled)
//    DS18B20 DQ 20 (bare probe: fit 4.7 k DQ->3.3 V; internal pull-up used only as a fallback)
//    LoRa Ra-01H: NSS 10, RST 9, DIO0 2, SPI 11/12/13
//    GNSS SkyTraq PX1125G: Serial2 (RX2 7 <- GPS_TXD, TX2 8 -> GPS_RXD), 1PPS 27, GPIO 28, ERST 29
//    Buzzer: D4 -> 2N3904 -> active buzzer on 5V_A.  LED1 is a hard-wired power LED (not switchable).
//    No battery-voltage divider and no DS1302 on this board: "Voltage" = INA219 load voltage (5 V rail);
//    the coin cell keeps the Teensy's own RTC (SNVS), used only for reset-gap timing.
//
//  Radio (team 010 assignment): main 866.50 MHz, backup 865.25 MHz, SF9, BW 125 kHz, CR 4/5,
//    sync 0x4A, preamble 8, CRC on. +10 dBm on the pad and in RECOVERY, +17 dBm BOOST..LANDING.
//    Power-on always starts on main; after a software/watchdog reset the last channel is kept.
//    Rocket never transmits before the ground has spoken to it. TX stops for good 90 min after launch.
//
//  States (forward only, except the ground FPL command and an unconfirmed launch)
//    0 BOOT      initialise. BOOT complete when every peripheral is up, or at 15 s if barometer +
//                accelerometer + servos are up. PRE-LAUNCH when BITE is clean (rocket still), on FPL, or
//                forced 60 s after power-up regardless of faults. Once calibrated, launch detection is already
//                armed in BOOT (a reboot shortly before ignition must not lose the flight).
//    1 PRELAUNCH BITE + automatic calibration every 20 s; waits for launch, no timeout. A path (accelerometer /
//                barometer) is disarmed only when its sensor is not working (BITE A / B, P).
//    2 BOOST     |a| > 2 g for 100 ms OR baro velocity > 10 m/s for 100 ms
//                Launch confirmation: within 2.5 s the barometer must show >= 15 m of climb, or the accelerometer
//                0.6 s of CONTINUOUS > 2 g; otherwise it was a handling shock -> back to PRE-LAUNCH. Nothing
//                beyond BOOST happens before the launch is confirmed.
//    3 COAST     |a| < 2 g, alt rising over 0.5 s, v falling over 1 s and > 10 m/s; timeout 4 s after launch
//                Trends ("rising", "falling") = least-squares slope over the window. Threshold conditions use a
//                majority vote: 4 of the last 5 samples (landing: 8 of the last 10 = 0.5 s).
//                Door: v < 20 m/s while rising (above 50 m), OR altitude >= 1000 m (filtered baro; GNSS AGL only if
//                the barometer has failed); timeout 15 s after launch
//    4 APOGEE    alt rising/stationary over 0.5 s AND v < 10 m/s and falling over 1 s (above 50 m);
//                timeout 3 s after the door (only when the door opened by velocity or its timeout: after a 1000 m
//                door the 3 s timer would deploy early on high flights, see APOGEE_TIMER_AFTER_ALT_DOOR) or 17.5 s
//                after launch. CanSat now, separation +1 s (FSEQ ground command: +1.5 s).
//    5 DESCENT   4 s after APOGEE
//    6 LANDING   alt < 25 m in 8 of 10 samples (0.5 s); timeout 750 s in DESCENT
//    7 RECOVERY  10 s after LANDING: buzzer on, sensors off except GNSS, telemetry = GNSS fields only
//
//  Telemetry (1 Hz):
//   $T,2026 IN-SPACe-010,time_s,count,alt_0.1m,press_Pa,temp_0.1C,volt_0.01V,gnss_s_of_hour,
//      lat_1e-4,lon_1e-4,gnss_alt_0.1m,sats,acc_0.1ms2,spin_0.1dps,state,vehicle,vel_ms
//
//  Libraries: RadioLib 7.x, TinyGPSPlus, MS5611 (Rob Tillaart, reset(1)), 7Semi BNO08x 0.1.0/1.0.1,
//             Adafruit INA219, Adafruit BusIO, Adafruit PWM Servo Driver, OneWire, DallasTemperature,
//             RocketKF.h must be in this folder. Watchdog: Teensy WDOG1 driven directly (no library).
//             ADXL345: direct registers (the Adafruit ADXL345 library is hard-wired to Wire, not Wire1).
// =====================================================================

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <stdarg.h>
#include <RadioLib.h>
#include <TinyGPSPlus.h>
#include <MS5611.h>
#include <Adafruit_INA219.h>
#include <Adafruit_PWMServoDriver.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#define BNO_USE_I2C
#include <7Semi_BNO08x.h>
#include "RocketKF.h"

// ============================ PINS (schematic) =======================
#define PCA_WIRE         Wire     // SDA 18, SCL 19
#define SENS_WIRE        Wire1    // SDA 17, SCL 16: MS5611, ADXL345, INA219
#define BNO_WIRE         Wire2    // SDA 25, SCL 24: BNO08x
#define I2C_HZ           400000
#define PIN_LORA_CS      10
#define PIN_LORA_DIO0    2
#define PIN_LORA_RST     9
#define GNSS_SERIAL      Serial2  // RX2 = 7, TX2 = 8
#define PIN_GNSS_PPS     27
#define PIN_GNSS_GPIO    28
#define PIN_GNSS_ERST    29       // external reset, active low
#define PIN_DS18B20      20
#define PIN_BUZZER       4        // active buzzer via 2N3904, high = on
#define PIN_BNO_RST      26
#define PIN_BNO_INT      23
#define PIN_ADXL_INT1    21
#define PIN_ADXL_INT2    22

// I2C addresses
#define MS5611_ADDR_A    0x76     // CSB tied high on the schematic
#define MS5611_ADDR_B    0x77
#define BNO_ADDR         0x4B
#define INA219_ADDR      0x40
#define PCA_ADDR         0x40

#define TEAM_ID          "2026 IN-SPACe-010"
#define VEHICLE_ID       1
#define FILE_PREFIX      "IN-SPACe-010_ROCKET"
#define VRAIL_MIN_V      4.75f    // BITE: INA219 load voltage (5 V rail) below this = fault V
#define CURRENT_MAX_MA   2500.0f  // BITE info flag above this

// ============================ RADIO ==================================
#define CH_MAIN_MHZ      866.50
#define CH_BACKUP_MHZ    865.25
#define LORA_BW          125.0
#define LORA_SF          9
#define LORA_CR          5
#define LORA_SYNC        0x4A
#define LORA_PREAMBLE    8
#define LORA_POWER_PAD   10       // dBm: BOOT, PRELAUNCH, RECOVERY
#define LORA_POWER_FLIGHT 17      // dBm: BOOST .. LANDING

// ============================ TIMING =================================
#define TICK_MS              50       // 20 Hz sensors, states, SD; EKF every 2nd tick (10 Hz, as tuned)
#define TELEM_PERIOD_MS      1000     // competition: 1 Hz
#define SD_FLUSH_MS          1000
#define BOOT_MAX_MS          15000
#define BOOT_FORCE_MS        60000    // PRE-LAUNCH enforced 60 s after power-up
#define BOOT_BEEP_MS         1000
#define BITE_BOOT_MS         1000     // BITE cadence while waiting in BOOT
#define BITE_SEND_MS         10000    // binary BITE downlink cadence while in BOOT
#define BITE_PERIOD_MS       20000    // PRE-LAUNCH: BITE + automatic calibration
#define CAL_PULSE_MS         150
#define CAL_MIN_SAMPLES      20
#define CAL_MOVE_G           0.5f     // |a| spread over the window above this = moving, no calibration
#define CAL_VEL_MAX          0.5f     // m/s: IMU offsets only updated with vertical velocity ~0
#define GPS_NODATA_MS        2000
#define GPS_HUNG_MS          20000    // no NMEA at any baud for this long (ground only) -> ERST reset
#define GPS_RESET_MIN_MS     30000
#define GPS_RESET_MAX_MS     600000
#define LORA_REINIT_MS       60000
#define RADIO_RETRY_MS       5000
#define TX_TIMEOUT_MS        3000
#define TX_GUARD_MS          300      // listen window after each transmission (ground uplink slot)
#define MAX_TX_DEFER_MS      600
#define FREQ_TRIAL_MS        10000    // new channel must hear the ground within 10 s, else revert
#define TX_CUTOFF_MS         (90UL * 60UL * 1000UL)
#define WDT_TIMEOUT_S        3
#define RESUME_RAM_GAP_MS    60000
#define RESUME_SD_GAP_S      300
#define BOOT_OVERHEAD_MS     350
#define BARO_STALE_MS        500
#define ACC_STALE_MS         200
#define BNO_STALE_MS         1000

// ========================= FLIGHT LOGIC ==============================
#define G0                   9.80665f
#define STILL_LIN_G          1.0f     // gravity-free |a| below this = on the rail (PRE-LAUNCH)
#define LAUNCH_ACC_G         2.0f
#define LAUNCH_ACC_N         3        // 3 samples = 100 ms
#define LAUNCH_BARO_VEL      10.0f
#define LAUNCH_BARO_N        3
#define COAST_ACC_G          2.0f
#define COAST_MIN_VEL        10.0f
#define BOOST_TIMEOUT_MS     4000
#define DOOR_VEL             20.0f
#define DOOR_ALT_M           1000.0f
#define T_APO_MS             16000    // expected apogee for timeouts (sims: 13.3-15.7 s)
#define DOOR_TIMEOUT_MS      (T_APO_MS - 1000)     // 15 s after launch
#define APOGEE_VEL           10.0f
#define APOGEE_AFTER_DOOR_MS 3000
#define APOGEE_TIMEOUT_MS    (T_APO_MS + 1500)     // 17.5 s after launch
#define DEPLOY_STEP_MS       1000     // automatic: door -> +1 s CanSat -> +1 s separation
#define DEPLOY_SEP_MANUAL_MS 1500     // FSEQ (ground command): separation 1.5 s after CanSat
#define DESCENT_DELAY_MS     4000
#define LANDING_ALT_M        25.0f
#define LAND_MV_N            10       // landing: alt < 25 m in 8 of the last 10 samples (0.5 s)
#define LAND_MV_K            8
#define DESCENT_TIMEOUT_MS   750000UL
#define RECOVERY_DELAY_MS    10000
#define MV_N                 5        // majority vote window (samples at 20 Hz) for COAST, door, APOGEE
#define MV_K                 4        //   condition true in at least this many of them
#define SLOPE_H_N            11       // least-squares altitude slope over 0.5 s
#define SLOPE_V_N            21       // least-squares velocity slope over 1 s
#define APOGEE_STAT_MS       2.0f     // altitude slope above -2 m/s counts as rising/stationary
#define BURN_EST_MS          4100     // motor burn time (3.6 s) + margin: accelerometer-loss fallback
#define LAUNCH_CONFIRM_MS    2500     // launch confirmation deadline after launch detection
#define LAUNCH_CONFIRM_ALT   15.0f    //   barometer climb that confirms the launch
#define LAUNCH_CONFIRM_ACC_MS 600     //   or CONTINUOUS time above 2 g that confirms it (stuck barometer)
#define MIN_DEPLOY_ALT_M     50.0f    // door-by-velocity and verified apogee need at least this altitude
#define APOGEE_TIMER_AFTER_ALT_DOOR 0 // 1 = spec literal: "apogee 3 s after door" also when the door opened at
                                      //   1000 m (on flights above ~1100 m that deploys ~2 s before apogee)
#define PAD_SLOPE_N          11       // pad: barometric velocity = least-squares slope over 0.5 s
#define PAD_MEAN_N           20       // pad: altitude = mean of the last 1 s of barometer samples
#define RP_N                 32       // raw pressure history (1.6 s)
// Sensor mounting: which sensor axis points along the rocket, nose up. -1 = automatic: the axis closest to gravity
// at calibration (correct on an 80-90 deg rail when the sensor is mounted square to the airframe).
// Set these from the mechanical drawing to remove any doubt.
#define ADXL_ROCKET_AXIS     -1       // 0 = X, 1 = Y, 2 = Z
#define ADXL_ROCKET_SIGN     1        // +1 if that axis points to the nose, -1 if to the tail
#define BNO_ROCKET_AXIS      -1
#define BNO_ROCKET_SIGN      1
#define AXIS_MAX_TILT_DEG    20.0f    // calibration refused if gravity is further than this from the rocket axis
#define FALSE_LAUNCH_MS      5000
#define FPL_MAX_ALT_M        30.0f    // FPL refused above this altitude unless FPL,1
#define FORCE_TELEM_IN_FLIGHT 1       // BOOST switches telemetry on (still needs ground heard and TX not muted)

// BNO08x (7Semi library) report intervals and scale fix
#define BNO_GYRO_MS          20       // 50 Hz
#define BNO_LIN_MS           50       // 20 Hz
#define BNO_GRAV_MS          100      // 10 Hz
#define BNO_GRV_MS           100      // 10 Hz (orientation, logged)
#define RAD2DEG              57.2957795f

// ============================ SERVOS =================================
#define SERVO_FREQ_HZ    50
#define PCA_OSC_HZ       27000000
#define SERVO_COUNT      3
#define DOOR_CH          0
#define PAYLOAD_CH       1
#define SEP_CH           2
const char* SERVO_NAME[SERVO_COUNT] = {"door", "payload", "separation"};
const int LOCK_DEG[SERVO_COUNT]     = {50, 80, 80};
const int UNLOCK_DEG[SERVO_COUNT]   = {0, 0, 0};
const int SERVO_US_0DEG[SERVO_COUNT]  = {600, 600, 600};
const int SERVO_US_90DEG[SERVO_COUNT] = {1500, 1500, 1500};
#define SERVO_US_MIN     500
#define SERVO_US_MAX     2500
#define SERVO_DPS        300.0f
const uint8_t SEQ_ORDER[3] = {DOOR_CH, PAYLOAD_CH, SEP_CH};
enum SeqMode : uint8_t { SEQ_INDIVIDUAL = 0, SEQ_MANUAL = 1, SEQ_FULL = 2 };

// ============================ STATES =================================
enum FlightState : uint8_t { BOOT = 0, PRELAUNCH, BOOST, COAST, APOGEE, DESCENT, LANDING, RECOVERY };
const char* STATE_NAME[] = {"BOOT", "PRELAUNCH", "BOOST", "COAST", "APOGEE", "DESCENT", "LANDING", "RECOVERY"};

#define FLAG_DOOR_OPEN   0x01
#define FLAG_CANSAT_OUT  0x02
#define FLAG_SEP_MOVED   0x04
#define FLAG_SEP_FIRED   0x08
#define FLAG_BY_DETECT   0x10
#define FLAG_BY_TIMEOUT  0x20
#define FLAG_BY_MANUAL   0x40
#define FLAG_DEPLOY_RUN  0x80

// types used in function signatures: declared before the first function (Arduino IDE prototypes)
enum CalResult : uint8_t { CAL_OK = 0, CAL_STATE, CAL_LAUNCHED, CAL_FEW, CAL_MOVING, CAL_NOSENSOR, CAL_VELOCITY, CAL_AXIS };
const char* CAL_TEXT[] = {"OK", "STATE", "LAUNCHED", "FEWSAMPLES", "MOVING", "NOSENSOR", "VELOCITY", "AXIS"};
// majority vote over the last N samples (N <= 32)
struct Vote {
  uint32_t bits = 0;
  uint8_t n = 0;
  bool push(bool c, uint8_t N, uint8_t K) {
    bits = (bits << 1) | (c ? 1u : 0u);
    if (n < 32) n++;
    if (n < N) return false;
    uint32_t m = N >= 32 ? 0xFFFFFFFFu : ((1u << N) - 1);
    return (uint8_t)__builtin_popcount(bits & m) >= K;
  }
  void reset() { bits = 0; n = 0; }
};
enum RadioMode { RADIO_IDLE, RADIO_RX, RADIO_TX };
struct Code { char c; const char* name; };
struct Persist {
  uint32_t magic;
  uint16_t version;
  uint32_t missionMs;
  uint64_t rtcTicks;
  uint8_t  state, flags, bootDone, forced;
  uint16_t token;
  uint32_t stateEnterMs, bootDoneMs;
  int64_t  tLaunchMs, tDoorMs, tApogeeMs;
  uint32_t payloadDueMs, sepDueMs;
  uint8_t  deployActive, kfPhase, kfChute, altCalDone, imuCalDone, doorByAlt;
  float    vPeak, hPeak, hMax;
  float    kfH, kfV, kfB, kfTLaunch, kfTDet, kfTNow, accAxLast;
  uint32_t calCount, lastCalMs;
  float    p0Pa, t0K, sinElev, bias0, axialMean0, gnssGround;
  int8_t   accAxis, accSign, spinAxis, spinSign;
  float    gyroBias[3];
  uint8_t  telemetryOn, indicatorManual, groundHeard, txMuted, txCutoff, curCh;
  uint32_t packetCount;
  int16_t  servoDeg[SERVO_COUNT];
  uint16_t servoUs[SERVO_COUNT];
  int32_t  runIndex;
  uint32_t resetCount;
  uint32_t crc;
};

// BITE fault codes: bit n <-> letter FAULT_LETTER[n]
const char FAULT_LETTER[] = "BPARSGDIONFULKVCTMWXEH";
const char* FAULT_TEXT[] = {
  "B MS5611 barometer not responding",       "P barometer pressure out of range",
  "A ADXL345 not responding",                "R ADXL345 not reading ~1 g at rest",
  "S PCA9685 servo driver not responding",   "G servo output read-back mismatch",
  "D SD card missing / write failed",        "I BNO08x not responding / no data",
  "O GNSS 1PPS not seen (info)",             "N GNSS no data",
  "F GNSS no fix",                           "U GNSS no UTC time",
  "L LoRa radio not initialised",            "K no packet heard from ground yet",
  "V 5 V rail low (INA219 load voltage)",    "C INA219 not responding",
  "T DS18B20 not found / no valid reading",  "M memory integrity",
  "W rocket moving (gravity-free |a| >= 1 g)", "X not calibrated (altitude or IMU)",
  "E last reset was a crash / watchdog (info)", "H current draw high (info)"};
#define N_FAULTS         22
#define FAULT_CRITICAL   0x0000003Fu   // B P A R S G
#define FAULT_INFO       ((1u << 8) | (1u << 20) | (1u << 21))   // O E H never block
#define BITE_MARKER      0xB1

// =====================================================================
//  OBJECTS
// =====================================================================
Module loraMod(PIN_LORA_CS, PIN_LORA_DIO0, PIN_LORA_RST, RADIOLIB_NC);
SX1276 radio(&loraMod);
MS5611 baroA(MS5611_ADDR_A, &SENS_WIRE);
MS5611 baroB(MS5611_ADDR_B, &SENS_WIRE);
MS5611* baro = &baroA;
BnoI2CBus bnoBus(BNO_WIRE, -1, -1, BNO_ADDR, I2C_HZ, PIN_BNO_INT, PIN_BNO_RST);
BNO08x_7Semi bno(bnoBus);
Adafruit_INA219 ina(INA219_ADDR);
Adafruit_PWMServoDriver pwm(PCA_ADDR, PCA_WIRE);
OneWire oneWire(PIN_DS18B20);
DallasTemperature ds(&oneWire);
TinyGPSPlus gps;
File dataFile, eventFile;
char dataName[48] = "", eventName[48] = "";
int runIndex = -1;
RocketKF kf;
// ---- hardware watchdog (i.MX RT1062 WDOG1), same register sequence as the WDT_T4 library ----
// timeout in 0.5 s steps (0.5..128 s); a timeout resets the CPU and sets SRC_SRSR_WDOG_RST_B (crash flag E)
struct HwWatchdog {
  void begin(float timeoutS) {
    CCM_CCGR3 |= (3UL << 16);                                        // WDOG1 clock on
    uint8_t wt = (uint8_t)((constrain(timeoutS, 0.5f, 128.0f) - 0.5f) / 0.5f);
    WDOG1_WCR = WDOG_WCR_SRS | WDOG_WCR_WT(wt);                      // keep SRS set (no software reset)
    WDOG1_WICR = 0;                                                  // no pre-timeout interrupt
    WDOG1_WCR |= WDOG_WCR_WDE | WDOG_WCR_WDA | WDOG_WCR_WDT | WDOG_WCR_SRE;
    WDOG1_WMCR = 0;                                                  // power-down counter off
    feed();
  }
  void feed() { WDOG1_WSR = 0x5555; WDOG1_WSR = 0xAAAA; }
};
HwWatchdog wdt;

// =====================================================================
//  GLOBALS
// =====================================================================
bool baroOk = false, accOk = false, bnoOk = false, inaOk = false, dsOk = false, pcaOk = false;
bool sdOk = false, radioOk = false;
bool sensorsPowered = true;
uint32_t resetSrsr = 0;
bool warmBoot = false, crashBoot = false;
uint32_t resetCount = 0;

int64_t missionOffsetMs = 0;
uint32_t missionMs() { return (uint32_t)((int64_t)millis() + missionOffsetMs); }

// state machine
FlightState state = BOOT;
uint16_t stateToken = 0;
uint32_t stateEnterMs = 0;
uint8_t flags = 0;
bool bootDone = false, forcedPrelaunch = false;
uint32_t bootDoneMs = 0;                       // mission ms when BOOT completed (telemetry time origin)
uint8_t dbLaunchAcc = 0, dbLaunchBaro = 0;
Vote mvCoast, mvDoor, mvApogee, mvLand;
bool launchConfirmed = false, launchBaroRef = false;
uint32_t boostAccMs = 0, boostAccRunMs = 0;
uint8_t boostAccGap = 0;
bool doorByAlt = false;
float launchBaroMax = -1e9f;
float slopeH = 0, slopeV = 0, vMin1s = 0;
int64_t tLaunchMs = -1, tDoorMs = -1, tApogeeMs = -1;
uint32_t payloadDueMs = 0, sepDueMs = 0;
bool deployActive = false;
float vPeak = 0, hPeak = 0, hMax = 0;
bool accArmed = false, baroArmed = false;

// barometer
float pressPa = 0, pressBaro = 0, baroTempC = 0, baroAlt = 0;
bool baroValid = false;
uint32_t lastBaroOkMs = 0, baroErrors = 0;
uint8_t baroSame = 0;                          // identical consecutive readings (a live MS5611 never repeats 10x)
bool baroStuck = false;
uint8_t pressSrc = 0;                          // 0 none, 1 MS5611, 2 back-computed from GNSS
// accelerometer (ADXL345, FIFO-averaged)
uint8_t accAddr = 0x53;
float ax = 0, ay = 0, az = 0, accMag = 0, accAx = 0, accPeak = 0;
uint8_t accN = 0;
bool accValid = false;
uint32_t lastAccOkMs = 0;
// BNO08x
float gyro[3] = {0, 0, 0}, linAcc[3] = {0, 0, 0}, grav[3] = {0, 0, 0};
float roll = 0, pitch = 0, yaw = 0, spinDps = 0, linAccMag = 0, bnoLinMag = 0;
uint32_t lastBnoMs = 0, lastBnoLinMs = 0, bnoGyroCount = 0;
// temperature
float dsTempC = NAN;
bool dsValid = false;
uint8_t dsGood = 0, dsBad = 0;
// power
float inaBusV = 0, inaShuntMv = 0, loadV = 0, inaCurrent = 0;
// estimate
float altM = 0, velMs = 0;
uint8_t altSrc = 0;                            // 0 hold, 1 EKF, 2 baro alpha-beta, 3 GNSS alpha-beta
float padH = 0, padV = 0, padVLong = 0;        // pad: 1 s mean altitude, 0.5 s / 1.6 s least-squares baro velocity
bool padValid = false;
float rpHist[RP_N];                            // raw MS5611 pressure, 20 Hz
uint8_t rpIdx = 0, rpFill = 0;
float accTiltDeg = -1, bnoTiltDeg = -1;
float gH = 0, gV = 0;                          // GNSS-AGL alpha-beta (barometer fallback)
bool gInit = false;
uint32_t lastGAltMs = 0;
#define GNSS_ALPHA 0.4f
#define GNSS_BETA  0.1f
float accAxSum = 0, baroPSum = 0;
uint8_t ekfDiv = 0, ekfBaroN = 0;
float linMaxWindow = 0;
float linHist[20];
uint8_t linIdx = 0;

#define HIST_N 32
float hHist[HIST_N], vHist[HIST_N];
uint8_t histIdx = 0, histFill = 0;

// calibration
float p0Pa = 101325.0f, t0K = 298.15f, sinElev = 0.98769f, bias0 = 0, axialMean0 = G0;
float gnssGround = NAN;
int accAxis = 2, accSign = 1;
int spinAxis = 2, spinSign = 1;
float gyroBias[3] = {0, 0, 0};
bool altCalDone = false, imuCalDone = false;
uint32_t calCount = 0, lastCalMs = 0;
double cP = 0, cT = 0, cA[3] = {0, 0, 0}, cG = 0, cGy[3] = {0, 0, 0}, cGr[3] = {0, 0, 0};
uint32_t cNP = 0, cNA = 0, cNG = 0, cNB = 0, cNR = 0;
float cMin = 1e9f, cMax = -1e9f;

// BITE
uint32_t faultMask = 0xFFFFFFFFu, lastSentMask = 0xFFFFFFFFu;
uint32_t lastBiteMs = 0, lastBiteSentMs = 0;
static uint8_t memTestBuf[256];

// servos
int servoDeg[SERVO_COUNT] = {-1, -1, -1};
uint16_t servoUs[SERVO_COUNT] = {0, 0, 0};
float servoEst[SERVO_COUNT] = {-1, -1, -1};
bool servoMoving[SERVO_COUNT] = {false, false, false};
uint32_t lastServoUpdMs = 0;
uint8_t seqMode = SEQ_INDIVIDUAL, seqStep = 0;

// buzzer
uint32_t indicatorUntil = 0;
bool indicatorManual = false, indicatorRecovery = false;

// GNSS
uint32_t lastFixMs = 0, lastGpsCharMs = 0, lastGpsResetMs = 0, gpsReboots = 0;   // lastGpsCharMs: last valid sentence
uint32_t gpsResetWaitMs = GPS_RESET_MIN_MS;
uint32_t gpsBaud = 9600, lastBaudHopMs = 0, gpsBaudSwitchAt = 0, gpsRateCfgAt = 0;
uint8_t gpsAutoCfgTries = 0;
bool gpsNeedsCfg = false, gpsTalkerLogged = false;
volatile uint32_t ppsCount = 0, ppsLastMs = 0;
DMAMEM uint8_t gpsRxBuf[2048];

// radio
volatile bool radioFlag = false;
RadioMode radioMode = RADIO_IDLE;
bool groundHeard = false, telemetryOn = false, txMuted = false, txCutoff = false, txMutePending = false;
bool telemDue = false, lastTxWasReply = false;
uint8_t curCh = 0, prevCh = 0;
int8_t freqPendingCh = -1;
bool freqTrial = false;
uint32_t freqTrialMs = 0, freqAskedMs = 0;
int8_t radioPowerNow = -1;
uint32_t telemDueSince = 0, txStartMs = 0, lastTxDoneMs = 0;
uint32_t lastGroundRxMs = 0, lastRadioInitMs = 0;
uint32_t packetCount = 0, rxGood = 0, rxCrc = 0, loraReinits = 0;
#define REPLY_Q 10
uint8_t replyQ[REPLY_Q][96];
uint8_t replyLen[REPLY_Q];
uint8_t rqHead = 0, rqCount = 0;
uint32_t replyDueSince = 0;

bool refreshPending = false, loraReinitPending = false;
uint32_t refreshAskedMs = 0, loraReinitAskedMs = 0, txMuteAskedMs = 0;
uint32_t lastTickMs = 0, lastTelemMs = 0, lastFlushMs = 0, lastSlowMs = 0, lastDsMs = 0;

void savePersist();
uint64_t rtcTicks();
void logEvent(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void reply(bool fromRadio, const char* fmt, ...) __attribute__((format(printf, 2, 3)));
void enterState(FlightState next, const char* why);
void radioApplyPower();

// =====================================================================
//  GNSS TIME
// =====================================================================
bool gpsFix() { return gps.location.isValid() && gps.location.age() < 2000; }
int64_t daysFromCivil(int y, unsigned m, unsigned d) {
  y -= m <= 2;
  const int era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return (int64_t)era * 146097 + (int64_t)doe - 719468;
}
uint64_t gpsEpochMs() {
  if (!gps.date.isValid() || !gps.time.isValid() || gps.date.year() < 2024 || gps.time.age() > 5000) return 0;
  int64_t s = daysFromCivil(gps.date.year(), gps.date.month(), gps.date.day()) * 86400LL +
              gps.time.hour() * 3600L + gps.time.minute() * 60L + gps.time.second();
  return (uint64_t)(s * 1000 + gps.time.centisecond() * 10 + gps.time.age());
}
// telemetry "GNSS Time": seconds from the start of the current UTC hour (0 if no time)
long gnssSecondsOfHour() {
  if (!gps.time.isValid() || gps.time.age() > 5000) return 0;
  return gps.time.minute() * 60L + gps.time.second();
}
void gpsUtcStr(char* out, size_t n) {
  if (!gpsEpochMs()) { snprintf(out, n, "-"); return; }
  snprintf(out, n, "%04d-%02d-%02dT%02d:%02d:%02d.%02dZ", gps.date.year(), gps.date.month(), gps.date.day(),
           gps.time.hour(), gps.time.minute(), gps.time.second(), gps.time.centisecond());
}
float gnssAgl() {
  if (!gpsFix() || !gps.altitude.isValid() || isnan(gnssGround)) return NAN;
  return gps.altitude.meters() - gnssGround;
}

// =====================================================================
//  EVENT LOG   mission_ms,mission_time_s,gnss_utc,event
// =====================================================================
void logEvent(const char* fmt, ...) {
  char msg[220];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(msg, sizeof msg, fmt, ap);
  va_end(ap);
  char gt[32];
  gpsUtcStr(gt, sizeof gt);
  uint32_t t = missionMs();
  Serial.printf("[EVT %lu.%03lu %s] %s\n", (unsigned long)(t / 1000), (unsigned long)(t % 1000), gt, msg);
  if (sdOk && eventFile)
    eventFile.printf("%lu,%lu.%03lu,%s,%s\n", (unsigned long)t, (unsigned long)(t / 1000), (unsigned long)(t % 1000), gt, msg);
}

// =====================================================================
//  BUZZER (the board's LED is a hard-wired power LED)
// =====================================================================
void indicatorPulse(uint32_t ms) {
  uint32_t until = millis() + ms;
  if ((int32_t)(until - indicatorUntil) > 0) indicatorUntil = until;
}
void updateIndicator() {
  bool on = indicatorManual || indicatorRecovery || (int32_t)(indicatorUntil - millis()) > 0;
  digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
}
bool indicatorIsOn() { return digitalRead(PIN_BUZZER) == HIGH; }

// =====================================================================
//  RADIO REPLY QUEUE (text replies and the binary BITE frame)
// =====================================================================
bool txAllowed() { return radioOk && groundHeard && !txMuted && !txCutoff; }
void queueRaw(const uint8_t* d, size_t n) {
  if (!groundHeard || txMuted || txCutoff || n == 0) return;
  if (n > sizeof replyQ[0]) n = sizeof replyQ[0];
  if (rqCount >= REPLY_Q) { rqHead = (rqHead + 1) % REPLY_Q; rqCount--; }
  uint8_t idx = (rqHead + rqCount) % REPLY_Q;
  memcpy(replyQ[idx], d, n);
  replyLen[idx] = n;
  if (rqCount == 0) replyDueSince = millis();
  rqCount++;
}
void reply(bool fromRadio, const char* fmt, ...) {
  char b[96];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  if (fromRadio) queueRaw((const uint8_t*)b, strlen(b));
  else Serial.println(b);
}

// =====================================================================
//  SERVOS
// =====================================================================
uint16_t degToUs(uint8_t ch, int deg) {
  return SERVO_US_0DEG[ch] + (int32_t)(SERVO_US_90DEG[ch] - SERVO_US_0DEG[ch]) * deg / 90;
}
int servoRemaining(uint8_t ch) {
  if (servoDeg[ch] < 0 || servoEst[ch] < 0) return 0;
  return (int)lroundf(fabsf(servoDeg[ch] - servoEst[ch]));
}
void reportServo(uint8_t ch) {
  Serial.printf("$H,SV,%u,%d,%d,%d,%u,%u\n", ch, servoDeg[ch], servoEst[ch] < 0 ? -1 : (int)lroundf(servoEst[ch]),
                servoRemaining(ch), seqMode, seqStep);
}
bool servoSetUs(uint8_t ch, uint16_t us, int deg, const char* why) {
  if (!pcaOk) { logEvent("SERVO,ch=%u,FAIL_NO_PCA,%s", ch, why); return false; }
  pwm.writeMicroseconds(ch, us);
  servoDeg[ch] = deg;
  servoUs[ch] = us;
  if (deg < 0) servoEst[ch] = -1;
  else if (servoEst[ch] < 0) servoEst[ch] = deg;
  servoMoving[ch] = deg >= 0 && fabsf(servoEst[ch] - deg) > 0.5f;
  if (ch == SEP_CH) flags |= FLAG_SEP_MOVED;
  logEvent("SERVO,ch=%u,%s,deg=%d,us=%u,flags=0x%02X,%s", ch, SERVO_NAME[ch], deg, us, flags, why);
  reportServo(ch);
  savePersist();
  return true;
}
bool servoSetDeg(uint8_t ch, int deg, const char* why) { return servoSetUs(ch, degToUs(ch, deg), deg, why); }

void updateServos() {
  uint32_t now = millis();
  float dt = (now - lastServoUpdMs) / 1000.0f;
  lastServoUpdMs = now;
  for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) {
    if (!servoMoving[ch] || servoDeg[ch] < 0) continue;
    float d = servoDeg[ch] - servoEst[ch], step = SERVO_DPS * dt;
    if (fabsf(d) <= step) { servoEst[ch] = servoDeg[ch]; servoMoving[ch] = false; reportServo(ch); }
    else servoEst[ch] += d > 0 ? step : -step;
  }
}

void openDoor(const char* why, uint8_t src) {
  if (flags & FLAG_DOOR_OPEN) return;
  servoSetDeg(DOOR_CH, UNLOCK_DEG[DOOR_CH], why);
  flags |= FLAG_DOOR_OPEN | src;
  tDoorMs = missionMs();
  logEvent("DOOR,OPEN,%s,alt=%.2f,vel=%.2f,t_launch=%.2f", why, altM, velMs,
           tLaunchMs >= 0 ? (missionMs() - tLaunchMs) / 1000.0f : -1.0f);
  savePersist();
}
void releaseCanSat(const char* why, uint8_t src) {
  if (flags & FLAG_CANSAT_OUT) return;
  servoSetDeg(PAYLOAD_CH, UNLOCK_DEG[PAYLOAD_CH], why);
  flags |= FLAG_CANSAT_OUT | src;
  logEvent("CANSAT,RELEASED,%s,alt=%.2f,vel=%.2f", why, altM, velMs);
  savePersist();
}
void fireSeparation(const char* why, uint8_t src) {
  if (flags & FLAG_SEP_FIRED) return;
  servoSetDeg(SEP_CH, UNLOCK_DEG[SEP_CH], why);
  flags |= FLAG_SEP_FIRED | src;
  kf.chuteForced = true;
  logEvent("SEPARATION,%s,alt=%.2f,vel=%.2f,flags=0x%02X", why, altM, velMs, flags);
  savePersist();
}
void startDeployment(const char* why, uint8_t src, uint32_t sepStepMs) {
  if (flags & FLAG_DEPLOY_RUN) return;
  flags |= FLAG_DEPLOY_RUN | src;
  uint32_t now = missionMs();
  if (!(flags & FLAG_DOOR_OPEN)) { openDoor(why, src); payloadDueMs = now + DEPLOY_STEP_MS; }
  else payloadDueMs = now;
  sepDueMs = payloadDueMs + sepStepMs;
  deployActive = true;
  logEvent("DEPLOY,START,%s,cansat_in_ms=%lu,sep_in_ms=%lu", why, (unsigned long)(payloadDueMs - now),
           (unsigned long)(sepDueMs - now));
  savePersist();
}
void serviceDeployment() {
  if (!deployActive) return;
  uint32_t now = missionMs();
  uint8_t src = flags & (FLAG_BY_DETECT | FLAG_BY_TIMEOUT | FLAG_BY_MANUAL);
  if ((int32_t)(now - payloadDueMs) >= 0) releaseCanSat("deployment", src);
  if ((int32_t)(now - sepDueMs) >= 0) {
    fireSeparation("deployment", src);
    deployActive = false;
    logEvent("DEPLOY,DONE");
    savePersist();
  }
}

bool commandServo(uint8_t ch, int deg, const char* why) {
  if (!pcaOk) return false;
  if (ch == SEP_CH && deg == UNLOCK_DEG[SEP_CH] && !(flags & FLAG_SEP_FIRED)) {
    fireSeparation(why, FLAG_BY_MANUAL);
    return true;
  }
  if (ch == DOOR_CH && deg == UNLOCK_DEG[DOOR_CH]) { openDoor(why, FLAG_BY_MANUAL); return true; }
  if (ch == PAYLOAD_CH && deg == UNLOCK_DEG[PAYLOAD_CH]) { releaseCanSat(why, FLAG_BY_MANUAL); return true; }
  if (!servoSetDeg(ch, deg, why)) return false;
  if (state <= PRELAUNCH) {                       // pad test undone: re-arm that step
    uint8_t before = flags;
    if (ch == DOOR_CH) flags &= ~FLAG_DOOR_OPEN;
    if (ch == PAYLOAD_CH) flags &= ~FLAG_CANSAT_OUT;
    if (ch == SEP_CH) { flags &= ~FLAG_SEP_FIRED; kf.chuteForced = false; }
    if (!(flags & (FLAG_DOOR_OPEN | FLAG_CANSAT_OUT | FLAG_SEP_FIRED))) {
      flags &= ~(FLAG_BY_DETECT | FLAG_BY_TIMEOUT | FLAG_BY_MANUAL | FLAG_DEPLOY_RUN);
      deployActive = false;
      tDoorMs = -1;
    }
    if (flags != before) logEvent("DEPLOY,REARMED,ch=%u,flags=0x%02X", ch, flags);
  }
  return true;
}

// =====================================================================
//  SENSORS
// =====================================================================
bool i2cPresent(TwoWire& w, uint8_t addr) {
  w.beginTransmission(addr);
  return w.endTransmission() == 0;
}
bool initBaroAt(MS5611* b) {
  if (!b->begin()) return false;
  if (!b->reset(1)) return false;               // Rob Tillaart: math mode 1 (as required)
  b->setOversampling(OSR_ULTRA_HIGH);
  if (b->read() != MS5611_READ_OK) return false;
  float p = b->getPressurePascal();
  return isfinite(p) && p > 10000.0f && p < 120000.0f;
}
bool initBaro() {
  if (i2cPresent(SENS_WIRE, MS5611_ADDR_A) && initBaroAt(&baroA)) { baro = &baroA; }
  else if (i2cPresent(SENS_WIRE, MS5611_ADDR_B) && initBaroAt(&baroB)) { baro = &baroB; }
  else return false;
  pressBaro = baro->getPressurePascal();
  baroTempC = baro->getTemperature();
  lastBaroOkMs = millis();
  logEvent("MS5611,OK,addr=0x%02X,p=%.1f,t=%.2f", baro == &baroA ? MS5611_ADDR_A : MS5611_ADDR_B, pressBaro, baroTempC);
  return true;
}

// ---- ADXL345 (direct registers on Wire1, FIFO stream mode) ----
bool adxlFound(uint8_t addr) {
  SENS_WIRE.beginTransmission(addr); SENS_WIRE.write(0x00);
  if (SENS_WIRE.endTransmission(false) != 0) return false;
  SENS_WIRE.requestFrom(addr, (uint8_t)1);
  return SENS_WIRE.available() && SENS_WIRE.read() == 0xE5;
}
bool adxlWrite(uint8_t r, uint8_t v) {
  SENS_WIRE.beginTransmission(accAddr); SENS_WIRE.write(r); SENS_WIRE.write(v);
  return SENS_WIRE.endTransmission() == 0;
}
int adxlRead8(uint8_t r) {
  SENS_WIRE.beginTransmission(accAddr); SENS_WIRE.write(r);
  if (SENS_WIRE.endTransmission(false) != 0) return -1;
  if (SENS_WIRE.requestFrom(accAddr, (uint8_t)1) != 1) return -1;
  return SENS_WIRE.read();
}
bool initAdxl() {
  if (adxlFound(0x53)) accAddr = 0x53;
  else if (adxlFound(0x1D)) accAddr = 0x1D;
  else return false;
  bool ok = adxlWrite(0x2D, 0x00);          // standby while configuring
  ok &= adxlWrite(0x2C, 0x0B);              // 200 Hz ODR (100 Hz bandwidth)
  ok &= adxlWrite(0x31, 0x0B);              // full resolution, +-16 g
  ok &= adxlWrite(0x38, 0x00);              // FIFO bypass (clears it)
  ok &= adxlWrite(0x38, 0x80);              // FIFO stream mode, 32 entries
  ok &= adxlWrite(0x2D, 0x08);              // measure
  lastAccOkMs = millis();
  return ok;
}
// one FIFO entry (6 bytes) in one burst; returns false on a bus error
bool adxlReadXYZ(float& x, float& y, float& z) {
  SENS_WIRE.beginTransmission(accAddr); SENS_WIRE.write(0x32);
  if (SENS_WIRE.endTransmission(false) != 0) return false;
  if (SENS_WIRE.requestFrom(accAddr, (uint8_t)6) != 6) return false;
  uint8_t b[6];
  for (int i = 0; i < 6; i++) b[i] = SENS_WIRE.read();
  const float k = 0.0039f * G0;             // full resolution: 3.9 mg/LSB
  x = (int16_t)(b[0] | (b[1] << 8)) * k;
  y = (int16_t)(b[2] | (b[3] << 8)) * k;
  z = (int16_t)(b[4] | (b[5] << 8)) * k;
  return true;
}
// averages every sample produced since the last call (~10 at 200 Hz / 20 Hz): anti-aliased |a|
bool adxlReadFifo() {
  int st = adxlRead8(0x39);
  if (st < 0) return false;
  int n = st & 0x3F;
  if (n == 0) { accN = 0; return true; }     // nothing new yet (staleness is checked by the caller)
  if (n > 32) n = 32;
  double sx = 0, sy = 0, sz = 0, sm = 0;
  float pk = 0;
  int got = 0;
  for (int i = 0; i < n; i++) {
    float x, y, z;
    if (!adxlReadXYZ(x, y, z)) break;
    float m = sqrtf(x * x + y * y + z * z);
    sx += x; sy += y; sz += z; sm += m; pk = fmaxf(pk, m);
    got++;
  }
  if (!got) return false;
  ax = sx / got; ay = sy / got; az = sz / got;
  accMag = sm / got;
  accPeak = pk;
  accN = got;
  float a3[3] = {ax, ay, az};
  accAx = accSign * a3[accAxis];
  lastAccOkMs = millis();
  return true;
}

// ---- BNO08x (7Semi library on Wire2) ----
bool initBno() {
  BNO_WIRE.begin();
  BNO_WIRE.setClock(I2C_HZ);
  if (!i2cPresent(BNO_WIRE, BNO_ADDR)) {
    // it may still be held in reset: pulse RST once and look again
    pinMode(PIN_BNO_RST, OUTPUT);
    digitalWrite(PIN_BNO_RST, LOW); delay(10); digitalWrite(PIN_BNO_RST, HIGH); delay(300);
    if (!i2cPresent(BNO_WIRE, BNO_ADDR)) return false;
  }
  if (!bno.begin()) return false;            // resets the BNO via RST and waits 300 ms
  BNO_WIRE.setClock(I2C_HZ);                 // the library only sets the clock on AVR / ESP
  bool ok = bno.enableReport(GYROSCOPE_CALIBRATED, BNO_GYRO_MS);
  ok &= bno.enableReport(LINEAR_ACCELERATION, BNO_LIN_MS);
  ok &= bno.enableReport(GRAVITY, BNO_GRAV_MS);
  bool grv = bno.enableReport(GAME_ROTATION_VECTOR, BNO_GRV_MS);
  logEvent("BNO08x,%s,gyro/lin/grav=%d,grv=%d", ok ? "OK" : "ENABLE_FAIL", ok, grv);
  lastBnoMs = lastBnoLinMs = millis();
  return ok;
}
// One SHTP input packet can carry several reports (base timestamp + gyro + linear accel ...). The 7Semi library
// decodes only the first report of each packet and uses the wrong Q point for the gyro (Q8 instead of Q9), so the
// packets are read through the library and decoded here. SH-2 Q points: accel/linear/gravity Q8, gyro Q9, GRV Q14.
static inline float q16(const uint8_t* p, float scale) { return (int16_t)(p[0] | (p[1] << 8)) / scale; }
void bnoReport(uint8_t id, const uint8_t* r, uint32_t now) {      // r = report start (id byte)
  const uint8_t* d = r + 4;                                       // id, seq, status, delay, then data
  switch (id) {
    case GYROSCOPE_CALIBRATED:
      for (int i = 0; i < 3; i++) gyro[i] = q16(d + 2 * i, 512.0f);
      spinDps = (gyro[spinAxis] - gyroBias[spinAxis]) * spinSign * RAD2DEG;
      lastBnoMs = now;
      bnoGyroCount++;
      if (state <= PRELAUNCH && !kf.launched()) { for (int i = 0; i < 3; i++) cGy[i] += gyro[i]; cNB++; }
      break;
    case LINEAR_ACCELERATION:
      for (int i = 0; i < 3; i++) linAcc[i] = q16(d + 2 * i, 256.0f);
      bnoLinMag = sqrtf(linAcc[0] * linAcc[0] + linAcc[1] * linAcc[1] + linAcc[2] * linAcc[2]);
      lastBnoLinMs = now;
      break;
    case GRAVITY:
      for (int i = 0; i < 3; i++) grav[i] = q16(d + 2 * i, 256.0f);
      if (state <= PRELAUNCH && !kf.launched()) { cGr[0] += grav[0]; cGr[1] += grav[1]; cGr[2] += grav[2]; cNR++; }
      break;
    case GAME_ROTATION_VECTOR: {
      float x = q16(d, 16384.0f), y = q16(d + 2, 16384.0f), z = q16(d + 4, 16384.0f), w = q16(d + 6, 16384.0f);
      roll = atan2f(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * RAD2DEG;
      float sp = 2 * (w * y - z * x);
      pitch = asinf(sp > 1 ? 1 : (sp < -1 ? -1 : sp)) * RAD2DEG;
      yaw = atan2f(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * RAD2DEG;
      break;
    }
  }
}
void bnoParse(const uint8_t* p, int n, uint32_t now) {
  if (n < 9) return;
  uint16_t hdr = p[0] | (p[1] << 8);
  if (hdr & 0x8000) return;                                       // continuation of an oversized packet
  int len = min((int)(hdr & 0x7FFF), n);
  uint8_t ch = p[2] & 0x0F;
  if (ch != SHTP_CH_INPUT && ch != SHTP_CH_WAKE) return;
  int off = 4;
  while (off < len) {
    uint8_t id = p[off];
    int rl;
    switch (id) {
      case 0xFB: case 0xFA: rl = 5; break;                        // base timestamp / timestamp rebase
      case ACCELEROMETER: case GYROSCOPE_CALIBRATED: case MAGNETIC_FIELD_CALIBRATED:
      case LINEAR_ACCELERATION: case GRAVITY: rl = 10; break;
      case GAME_ROTATION_VECTOR: rl = 12; break;
      case ROTATION_VECTOR: case GEOMAGNETIC_ROTATION_VECTOR: rl = 14; break;
      case 0x07: rl = 16; break;                                  // uncalibrated gyro
      default: return;                                            // unknown report: length unknown, stop
    }
    if (off + rl > len) return;
    if (id < 0xF0) bnoReport(id, p + off, now);
    off += rl;
  }
}
void serviceBno() {
  if (!bnoOk || !sensorsPowered) return;
  uint32_t now = millis();
  uint8_t pkt[128];
  for (int i = 0; i < 8 && digitalRead(PIN_BNO_INT) == LOW; i++) {
    int n = bno.readPacket(pkt, sizeof pkt);
    if (n <= 0) break;
    bnoParse(pkt, n, now);
  }
  if (now - lastBnoMs > BNO_STALE_MS) {
    bnoOk = false;
    spinDps = 0;
    logEvent("BNO08x,STALE,no gyro report for %lu ms", (unsigned long)(now - lastBnoMs));
  }
}

bool initIna() { return ina.begin(&SENS_WIRE); }

// ---- DS18B20: bare probe on D20 ----
DeviceAddress dsAddr;
bool initDs() {
  pinMode(PIN_DS18B20, INPUT_PULLUP);       // weak internal pull-up: fallback only, fit 4.7 k externally
  ds.begin();
  if (ds.getDeviceCount() == 0 || !ds.getAddress(dsAddr, 0)) return false;
  if (OneWire::crc8(dsAddr, 7) != dsAddr[7] || dsAddr[0] != 0x28) return false;   // family 0x28 = DS18B20
  ds.setResolution(dsAddr, 11);              // 375 ms conversion
  ds.setWaitForConversion(false);
  ds.requestTemperaturesByAddress(dsAddr);
  lastDsMs = millis();
  dsGood = dsBad = 0;
  dsValid = false;
  return true;
}
void readDs() {
  if (!dsOk || millis() - lastDsMs < 500) return;
  float t = ds.getTempC(dsAddr);             // scratchpad CRC checked by the library (-127 on failure)
  // 85.0 is the power-on reset value: never a real first reading
  bool good = t > -55.0f && t < 125.0f && t != DEVICE_DISCONNECTED_C && !(t == 85.0f && !dsValid);
  if (good) { dsTempC = t; dsBad = 0; if (dsGood < 255) dsGood++; if (dsGood >= 3) dsValid = true; }
  else { dsGood = 0; if (dsBad < 255) dsBad++; if (dsBad >= 3) dsValid = false; }
  ds.requestTemperaturesByAddress(dsAddr);
  lastDsMs = millis();
}
float airTempC() { return dsValid ? dsTempC : baroTempC; }

bool initPca() {
  if (!i2cPresent(PCA_WIRE, PCA_ADDR)) return false;
  if (!pwm.begin()) return false;
  pwm.setOscillatorFrequency(PCA_OSC_HZ);
  pwm.setPWMFreq(SERVO_FREQ_HZ);
  return true;
}

void powerDownSensors() {
  if (accOk) adxlWrite(0x2D, 0x00);          // ADXL345 standby
  if (bnoOk) digitalWrite(PIN_BNO_RST, LOW); // BNO08x held in reset
  if (inaOk) ina.powerSave(true);
  sensorsPowered = false;                    // MS5611 / DS18B20 simply not read any more
  logEvent("RECOVERY,sensors off (MS5611, ADXL345, BNO08x, INA219, DS18B20), GNSS on");
}
void powerUpSensors() {
  sensorsPowered = true;
  if (inaOk) ina.powerSave(false);
  accOk = initAdxl();
  if (bnoOk) { digitalWrite(PIN_BNO_RST, HIGH); bnoOk = initBno(); }
  if (pcaOk) pwm.wakeup();
  indicatorRecovery = false;
  logEvent("SENSORS,ON,acc=%d,bno=%d", accOk, bnoOk);
}

void pushHistory(float h, float v) {
  hHist[histIdx] = h; vHist[histIdx] = v;
  histIdx = (histIdx + 1) % HIST_N;
  if (histFill < HIST_N) histFill++;
}
// last n samples of a ring buffer, oldest first; returns how many were available
int ringLast(const float* buf, int size, int idx, int fill, int n, float* out) {
  if (n > fill) n = fill;
  for (int i = 0; i < n; i++) out[i] = buf[(idx + size - n + i) % size];
  return n;
}
// least-squares slope (per second) of y[0..n-1] sampled every dt, oldest first
float lsqSlope(const float* y, int n, float dt) {
  if (n < 3) return 0;
  float xm = (n - 1) / 2.0f, ym = 0, sxy = 0, sxx = 0;
  for (int i = 0; i < n; i++) ym += y[i];
  ym /= n;
  for (int i = 0; i < n; i++) { float dx = i - xm; sxy += dx * (y[i] - ym); sxx += dx * dx; }
  return sxy / sxx / dt;
}

// pressure from a GNSS AGL height with the pad reference (barometer failed)
float pressureFromAlt(float h) {
  float base = 1.0f - 0.0065f * h / t0K;
  if (base <= 0.01f) return 0;
  return p0Pa * powf(base, G0 / (287.05f * 0.0065f));
}

void readSensors() {
  uint32_t now = millis();
  if (!sensorsPowered) { baroValid = accValid = false; return; }
  if (baroOk) {
    if (baro->read() == MS5611_READ_OK) {
      float p = baro->getPressurePascal();
      if (isfinite(p) && p > 10000.0f && p < 120000.0f) {
        baroSame = (p == pressBaro) ? (baroSame < 255 ? baroSame + 1 : 255) : 0;
        if (baroSame >= 10 && !baroStuck) { baroStuck = true; logEvent("MS5611,STUCK,p=%.1f: barometer ignored", p); }
        if (baroSame == 0 && baroStuck) { baroStuck = false; logEvent("MS5611,UNSTUCK"); }
        pressBaro = p; baroTempC = baro->getTemperature();
        if (!baroStuck) lastBaroOkMs = now;
      } else baroErrors++;
    } else baroErrors++;
  }
  baroValid = baroOk && now - lastBaroOkMs < BARO_STALE_MS;
  if (accOk) adxlReadFifo();                     // a bus error leaves lastAccOkMs stale
  // fresh millis(): the barometer read above blocks ~20 ms, so lastAccOkMs is later than `now`
  accValid = accOk && millis() - lastAccOkMs < ACC_STALE_MS;

  // gravity-free |a| for the stillness test: BNO08x linear acceleration, else ADXL |a| - g
  bool bnoLinFresh = bnoOk && now - lastBnoLinMs < 200;
  linAccMag = bnoLinFresh ? bnoLinMag : (accValid ? fabsf(accMag - G0) : 0.0f);
  linHist[linIdx] = linAccMag; linIdx = (linIdx + 1) % 20;
  linMaxWindow = 0;
  for (float v : linHist) linMaxWindow = fmaxf(linMaxWindow, v);

  // pressure output: MS5611, else back-computed from GNSS AGL
  if (baroValid) { pressPa = pressBaro; pressSrc = 1; }
  else if (gInit && altCalDone) { pressPa = pressureFromAlt(gH); pressSrc = 2; }
  else { pressPa = 0; pressSrc = 0; }
  baroAlt = (altCalDone && baroValid) ? kf.baroAltitude(pressBaro) : NAN;

  // ground samples for the next calibration
  if (state <= PRELAUNCH && !kf.launched()) {
    if (baroValid) { cP += pressBaro; cT += airTempC(); cNP++; }
    if (accValid && accN) {
      cA[0] += ax; cA[1] += ay; cA[2] += az; cNA++;
      cMin = fminf(cMin, accMag); cMax = fmaxf(cMax, accMag);
    }
    if (gpsFix() && gps.altitude.isValid()) { cG += gps.altitude.meters(); cNG++; }
  }
}

void readSlowSensors() {                         // 2 Hz
  if (!sensorsPowered) return;
  if (inaOk) {
    inaBusV = ina.getBusVoltage_V();
    inaShuntMv = ina.getShuntVoltage_mV();
    inaCurrent = ina.getCurrent_mA();
    loadV = inaBusV + inaShuntMv / 1000.0f;      // load voltage = bus + shunt drop (Adafruit method)
  }
  readDs();
}

// =====================================================================
//  GNSS  (SkyTraq PX1125G, binary protocol AN0037)
// =====================================================================
void onPps() { ppsCount++; ppsLastMs = millis(); }

void gpsSetBaud(uint32_t b) {
  GNSS_SERIAL.end();
  GNSS_SERIAL.begin(b);
  gpsBaud = b;
}
void stqSend(const uint8_t* p, uint16_t n) {
  uint8_t cs = 0;
  for (uint16_t i = 0; i < n; i++) cs ^= p[i];
  uint8_t h[4] = {0xA0, 0xA1, (uint8_t)(n >> 8), (uint8_t)n};
  GNSS_SERIAL.write(h, 4);
  GNSS_SERIAL.write(p, n);
  uint8_t t[3] = {cs, 0x0D, 0x0A};
  GNSS_SERIAL.write(t, 3);
  GNSS_SERIAL.flush();
}
// 1 = ACK, -1 = NACK, 0 = timeout. Blocking: setup only.
int stqWaitAck(uint8_t id, int sub, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  uint8_t st = 0, buf[16];
  uint16_t len = 0, k = 0;
  while (millis() - t0 < timeoutMs) {
    if (!GNSS_SERIAL.available()) continue;
    uint8_t c = GNSS_SERIAL.read();
    switch (st) {
      case 0: st = (c == 0xA0) ? 1 : 0; break;
      case 1: st = (c == 0xA1) ? 2 : (c == 0xA0 ? 1 : 0); break;
      case 2: len = (uint16_t)c << 8; st = 3; break;
      case 3: len |= c; k = 0; st = (len > 0 && len <= sizeof buf) ? 4 : 0; break;
      case 4: buf[k++] = c; if (k >= len) st = 5; break;
      case 5: {
        uint8_t cs = 0;
        for (uint16_t i = 0; i < len; i++) cs ^= buf[i];
        st = 0;
        if (cs != c || len < 2) break;
        if ((buf[0] == 0x83 || buf[0] == 0x84) && buf[1] == id && (sub < 0 || (len >= 3 && buf[2] == sub)))
          return buf[0] == 0x83 ? 1 : -1;
        break;
      }
    }
  }
  return 0;
}
int stqCmd(const char* name, const uint8_t* p, uint16_t n, int sub) {
  for (int tries = 0; tries < 2; tries++) {
    stqSend(p, n);
    int r = stqWaitAck(p[0], sub, 700);
    if (r) { logEvent("GPS,CFG,%s,%s", name, r > 0 ? "ACK" : "NACK"); return r; }
  }
  logEvent("GPS,CFG,%s,TIMEOUT", name);
  return 0;
}
// SkyTraq configuration payloads (all SRAM only: resent after every GNSS restart, no flash wear)
const uint8_t STQ_NMEA[9]   = {0x08, 1, 0, 0, 0, 1, 0, 0, 0};            // GGA + RMC only
const uint8_t STQ_NAV[4]    = {0x64, 0x17, 5, 0};                       // navigation mode 5 = airborne
const uint8_t STQ_SBAS[9]   = {0x62, 0x01, 1, 2, 8, 1, 3, 0x08, 0};     // SBAS on, auto ranging, GAGAN
const uint8_t STQ_BAUD[4]   = {0x05, 0, 5, 0};                          // COM1 115200
const uint8_t STQ_RATE10[3] = {0x0E, 10, 0};                            // 10 Hz (needs >= 38400 baud)
const uint8_t STQ_RATE1[3]  = {0x0E, 1, 0};

bool gpsDetect(uint32_t baud, uint32_t ms) {
  gpsSetBaud(baud);
  uint32_t p0 = gps.passedChecksum(), t0 = millis();
  while (millis() - t0 < ms) {
    while (GNSS_SERIAL.available()) gps.encode(GNSS_SERIAL.read());
    if (gps.passedChecksum() - p0 >= 2) { lastGpsCharMs = millis(); return true; }
  }
  return false;
}
// boot-time configuration with ACK checks (not used when resuming in flight)
void gnssConfigureBlocking() {
  bool fast = gpsDetect(115200, 700);
  if (!fast && !gpsDetect(9600, 2500)) {
    logEvent("GPS,CFG,no NMEA at 115200 or 9600: check wiring / power");
    gpsSetBaud(9600);
    gpsNeedsCfg = true;
    return;
  }
  logEvent("GPS,BAUD,%lu", (unsigned long)gpsBaud);
  stqCmd("NMEA_GGA_RMC", STQ_NMEA, sizeof STQ_NMEA, -1);
  stqCmd("NAV_MODE_AIRBORNE", STQ_NAV, sizeof STQ_NAV, 0x17);
  stqCmd("SBAS_GAGAN", STQ_SBAS, sizeof STQ_SBAS, 0x01);
  if (!fast) {
    if (stqCmd("BAUD_115200", STQ_BAUD, sizeof STQ_BAUD, -1) == 1) {
      delay(150);
      fast = gpsDetect(115200, 2000);
      if (!fast) { logEvent("GPS,BAUD,no data at 115200, back to 9600"); gpsDetect(9600, 2000); }
    }
  }
  if (fast) stqCmd("RATE_10HZ", STQ_RATE10, sizeof STQ_RATE10, -1);
  else stqCmd("RATE_1HZ", STQ_RATE1, sizeof STQ_RATE1, -1);
  gpsAutoCfgTries = fast ? 0 : 3;                 // at 9600 after a refused baud change: leave it there
  gpsNeedsCfg = false;
  logEvent("GPS,READY,baud=%lu,rate=%d Hz", (unsigned long)gpsBaud, fast ? 10 : 1);
}
// ERST is active low; hardware reset returns the module to 9600 baud and its saved settings
void gpsHardReset(const char* why) {
  digitalWrite(PIN_GNSS_ERST, LOW);
  delay(10);
  digitalWrite(PIN_GNSS_ERST, HIGH);
  gpsReboots++;
  gpsAutoCfgTries = 0;
  gpsNeedsCfg = true;
  lastGpsResetMs = millis();
  logEvent("GPS,RESET,%lu,%s", (unsigned long)gpsReboots, why);
}
void serviceGps() {
  uint32_t now = millis();
  uint32_t passedBefore = gps.passedChecksum();
  while (GNSS_SERIAL.available()) gps.encode(GNSS_SERIAL.read());
  // "data present" = a valid NMEA sentence. At the wrong baud the UART still delivers bytes (framing garbage),
  // which must not stop the baud search after a module reset.
  if (gps.passedChecksum() != passedBefore) lastGpsCharMs = now;
  if (!gpsTalkerLogged && gps.passedChecksum() > 5) {
    gpsTalkerLogged = true;
    logEvent("GPS,NMEA,passed=%lu,failed=%lu,fix=%d (TinyGPSPlus reads GP/GN RMC+GGA)",
             (unsigned long)gps.passedChecksum(), (unsigned long)gps.failedChecksum(), gpsFix());
  }
  // GNSS-AGL alpha-beta: barometer fallback for altitude, velocity and back-computed pressure
  if (gps.altitude.isUpdated() && gpsFix() && !isnan(gnssGround)) {
    float agl = gps.altitude.meters() - gnssGround;
    float dt = (now - lastGAltMs) / 1000.0f;
    if (!gInit || dt > 3.0f) { gH = agl; gV = 0; gInit = true; }
    else if (dt > 0.02f) {
      float pred = gH + gV * dt, r = agl - pred;
      gH = pred + GNSS_ALPHA * r;
      gV += GNSS_BETA * r / dt;
    }
    lastGAltMs = now;
  }
  if (gpsFix()) lastFixMs = now;
  if (now - lastGpsCharMs < 1000) gpsResetWaitMs = GPS_RESET_MIN_MS;

  // no data: hop between 115200 and 9600 (module restarted, or never configured)
  if (now - lastGpsCharMs > GPS_NODATA_MS && now - lastBaudHopMs > GPS_NODATA_MS) {
    gpsSetBaud(gpsBaud == 115200 ? 9600 : 115200);
    lastBaudHopMs = now;
    if (gpsBaud == 9600) gpsNeedsCfg = true;
    static uint32_t hops = 0;
    if (++hops <= 10 || hops % 30 == 0) logEvent("GPS,NO_DATA,trying %lu baud (hop %lu)", (unsigned long)gpsBaud, (unsigned long)hops);
    return;
  }
  // data at 9600 after a restart: resend the configuration without waiting (safe in flight)
  if (gpsBaud == 9600 && gpsNeedsCfg && now - lastGpsCharMs < 300 && gpsAutoCfgTries < 3) {
    gpsAutoCfgTries++;
    gpsNeedsCfg = false;
    stqSend(STQ_NMEA, sizeof STQ_NMEA);
    stqSend(STQ_NAV, sizeof STQ_NAV);
    stqSend(STQ_SBAS, sizeof STQ_SBAS);
    stqSend(STQ_BAUD, sizeof STQ_BAUD);
    gpsBaudSwitchAt = now + 200;
    logEvent("GPS,RECONFIG,try %u", gpsAutoCfgTries);
  }
  if (gpsBaudSwitchAt && (int32_t)(now - gpsBaudSwitchAt) >= 0) {
    gpsBaudSwitchAt = 0;
    gpsSetBaud(115200);
    lastBaudHopMs = now;
    gpsRateCfgAt = now + 500;
  }
  if (gpsRateCfgAt && (int32_t)(now - gpsRateCfgAt) >= 0) {
    gpsRateCfgAt = 0;
    stqSend(STQ_RATE10, sizeof STQ_RATE10);
  }
  // Module silent at every baud for 20 s (hung): hardware reset, on the ground only, never in flight.
  // A missing FIX is never reset: that would restart satellite acquisition from scratch (this board has no GNSS
  // backup battery, so every reset is a cold start of 30 s or more).
  bool ground = state <= PRELAUNCH || state == RECOVERY;
  if (ground && now - lastGpsCharMs > GPS_HUNG_MS && now - lastGpsResetMs > gpsResetWaitMs) {
    gpsHardReset("no data 20 s");
    gpsResetWaitMs = min(gpsResetWaitMs * 2, (uint32_t)GPS_RESET_MAX_MS);
  }
}

// =====================================================================
//  RADIO
// =====================================================================
float chFreq(uint8_t ch) { return ch ? CH_BACKUP_MHZ : CH_MAIN_MHZ; }
int8_t wantedPower() { return (state >= BOOST && state <= LANDING) ? LORA_POWER_FLIGHT : LORA_POWER_PAD; }
void onDio0() { radioFlag = true; }
void radioStartRx() {
  radioFlag = false;
  int st = radio.startReceive();
  radioMode = RADIO_RX;
  if (st != RADIOLIB_ERR_NONE) Serial.printf("[LORA] startReceive failed %d\n", st);
}
bool radioStep(const char* what, int st) {
  if (st == RADIOLIB_ERR_NONE) return true;
  Serial.printf("[LORA] %s failed %d\n", what, st);
  return false;
}
bool radioConfigure() {
  int8_t pw = wantedPower();
  if (!radioStep("begin", radio.begin())) return false;
  if (!radioStep("standby", radio.standby())) return false;
  if (!radioStep("frequency", radio.setFrequency(chFreq(curCh)))) return false;
  if (!radioStep("bandwidth", radio.setBandwidth(LORA_BW))) return false;
  if (!radioStep("sf", radio.setSpreadingFactor(LORA_SF))) return false;
  if (!radioStep("cr", radio.setCodingRate(LORA_CR))) return false;
  if (!radioStep("sync", radio.setSyncWord(LORA_SYNC))) return false;
  if (!radioStep("power", radio.setOutputPower(pw))) return false;
  if (!radioStep("preamble", radio.setPreambleLength(LORA_PREAMBLE))) return false;
  if (!radioStep("crc", radio.setCRC(true))) return false;
  radioPowerNow = pw;
  return true;
}
// expected: 866.50 MHz FRF=D8A000, 865.25 MHz FRF=D85000, MC1=72 (BW125, 4/5), MC2=94 (SF9, CRC), SYNC=4A
void radioDump() {
  Module* m = &loraMod;
  uint32_t frf = ((uint32_t)m->SPIreadRegister(0x06) << 16) | ((uint32_t)m->SPIreadRegister(0x07) << 8) |
                 m->SPIreadRegister(0x08);
  logEvent("LORA,CFG,ch=%u,FRF=%06lX (%.3f MHz),MC1=%02X,MC2=%02X,SYNC=%02X,PA=%02X,dBm=%d", curCh, (unsigned long)frf,
           frf * 32.0 / 524288.0, m->SPIreadRegister(0x1D), m->SPIreadRegister(0x1E) & 0xF4,
           m->SPIreadRegister(0x39), m->SPIreadRegister(0x09), radioPowerNow);
}
bool radioInit() {
  static bool isrAttached = false;
  if (isrAttached) { radio.clearDio0Action(); isrAttached = false; }
  bool ok = radioConfigure();
  if (!ok) {
    pinMode(PIN_LORA_RST, OUTPUT);
    digitalWrite(PIN_LORA_RST, LOW); delay(2); digitalWrite(PIN_LORA_RST, HIGH); delay(20);
    ok = radioConfigure();
  }
  if (!ok) { radioMode = RADIO_IDLE; return false; }
  radioDump();
  radio.setDio0Action(onDio0, RISING);
  isrAttached = true;
  radioStartRx();
  return true;
}
void radioReinit(const char* why) {
  loraReinits++;
  radioOk = radioInit();
  lastRadioInitMs = millis();
  logEvent("LORA,REINIT,%lu,%s,%s", (unsigned long)loraReinits, why, radioOk ? "OK" : "FAIL");
}
bool retune(uint8_t ch, const char* why) {
  curCh = ch;
  if (!radioOk) return false;
  radio.standby();
  int st = radio.setFrequency(chFreq(ch));
  radioStartRx();
  logEvent("LORA,CHANNEL,%u,%.2f MHz,%s,%s", ch, chFreq(ch), why, st == RADIOLIB_ERR_NONE ? "OK" : "FAIL");
  savePersist();
  if (st != RADIOLIB_ERR_NONE) { radioReinit("retune failed"); return false; }
  return true;
}
// +10 dBm on the pad and in RECOVERY, +17 dBm BOOST..LANDING (applied between transmissions)
void radioApplyPower() {
  int8_t pw = wantedPower();
  if (!radioOk || pw == radioPowerNow || radioMode == RADIO_TX) return;
  radio.standby();
  int st = radio.setOutputPower(pw);
  radioStartRx();
  if (st == RADIOLIB_ERR_NONE) { radioPowerNow = pw; logEvent("LORA,POWER,%d dBm", pw); }
}
bool radioSend(const uint8_t* msg, size_t n) {
  radioFlag = false;
  int st = radio.startTransmit(msg, n);
  if (st != RADIOLIB_ERR_NONE) { Serial.printf("[LORA] startTransmit failed %d\n", st); radioReinit("TX error"); return false; }
  radioMode = RADIO_TX;
  txStartMs = millis();
  return true;
}
bool channelBusy() { return (radio.getModemStatus() & 0x0A) != 0; }

// =====================================================================
//  BITE
// =====================================================================
bool memoryTest() {
  for (int pass = 0; pass < 2; pass++) {
    uint8_t pat = pass ? 0xAA : 0x55;
    for (size_t i = 0; i < sizeof memTestBuf; i++) memTestBuf[i] = pat ^ (uint8_t)i;
    for (size_t i = 0; i < sizeof memTestBuf; i++) if (memTestBuf[i] != (uint8_t)(pat ^ (uint8_t)i)) return false;
  }
  extern unsigned long _ebss;
  char* sp = (char*)__builtin_frame_address(0);
  return sp - (char*)&_ebss > 16384;
}
bool sdTest() {
  if (!sdOk || !dataFile || dataFile.getWriteError()) return false;
  File f = SD.open("/BITE_TEST.TMP", FILE_WRITE_BEGIN);
  if (!f) return false;
  const char* s = "BITE";
  f.write((const uint8_t*)s, 4);
  f.close();
  f = SD.open("/BITE_TEST.TMP", FILE_READ);
  if (!f) return false;
  char r[5] = {0};
  f.read((uint8_t*)r, 4);
  f.close();
  return !strcmp(r, s);
}
bool servoReadback() {
  if (!pcaOk) return false;
  for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) {
    if (!servoUs[ch]) continue;
    uint16_t off = pwm.getPWM(ch, true);
    uint16_t expect = (uint16_t)lroundf(servoUs[ch] * 4096.0f * SERVO_FREQ_HZ / 1e6f);
    if (abs((int)off - (int)expect) > 8) return false;
  }
  return true;
}
void restoreServos(bool resumed);
bool openNewLogs();
uint32_t lastRetryMs = 0;
// re-initialise whatever is down. slow = BNO08x too (blocks ~1 s): BOOT or on command only.
void retryPeripherals(bool slow) {
  if (!sensorsPowered || millis() - lastRetryMs < 5000) return;
  lastRetryMs = millis();
  if (!baroOk) baroOk = initBaro();
  if (!accOk) accOk = initAdxl();
  if (!inaOk) inaOk = initIna();
  if (!dsOk) dsOk = initDs();
  if (!pcaOk && (pcaOk = initPca())) restoreServos(false);
  wdt.feed();
  if (slow && !bnoOk) bnoOk = initBno();
  wdt.feed();
  if (!radioOk) radioReinit("retry");
  if (!sdOk && (sdOk = SD.begin(BUILTIN_SDCARD))) sdOk = openNewLogs() && sdOk;
}
uint32_t runBite(bool slow) {
  retryPeripherals(slow);
  uint32_t m = 0;
  auto set = [&](int bit, bool fault) { if (fault) m |= 1u << bit; };
  set(0, !baroOk);
  set(1, baroOk && (!baroValid || pressBaro < 30000 || pressBaro > 110000));
  set(2, !accOk || !accValid);
  set(3, accValid && fabsf(accMag - G0) > 1.5f);
  set(4, !pcaOk);
  set(5, pcaOk && !servoReadback());
  set(6, !sdTest());
  set(7, !bnoOk);
  uint32_t ppsLast = ppsLastMs;                  // read before millis(): written by the PPS interrupt
  set(8, millis() - ppsLast > 3000);
  set(9, millis() - lastGpsCharMs > 2000);
  set(10, !gpsFix());
  set(11, !gpsEpochMs());
  set(12, !radioOk);
  set(13, !groundHeard);
  set(14, inaOk && loadV < VRAIL_MIN_V);
  set(15, !inaOk);
  set(16, !dsOk || !dsValid);
  set(17, !memoryTest());
  set(18, linMaxWindow >= STILL_LIN_G * G0);
  set(19, !altCalDone || !imuCalDone);
  set(20, crashBoot);
  set(21, inaOk && fabsf(inaCurrent) > CURRENT_MAX_MA);
  faultMask = m;
  lastBiteMs = millis();
  return m;
}
void faultLetters(uint32_t m, char* out, size_t n) {
  size_t k = 0;
  for (int i = 0; i < N_FAULTS && k < n - 1; i++) if (m & (1u << i)) out[k++] = FAULT_LETTER[i];
  out[k] = '\0';
}
// binary BITE frame: B1, state, status, mask(4 LE), vrail cV(2 LE), sats, token(2 LE)
void queueBiteFrame() {
  uint8_t st = 0;
  if (!(faultMask & ~FAULT_INFO)) st |= 0x01;
  if (!(faultMask & FAULT_CRITICAL)) st |= 0x02;
  if (forcedPrelaunch) st |= 0x04;
  if (bootDone) st |= 0x08;
  uint16_t cv = (uint16_t)constrain(lroundf(loadV * 100.0f), 0, 65535);
  uint8_t sats = gps.satellites.isValid() ? (uint8_t)min(gps.satellites.value(), (uint32_t)255) : 0;
  uint8_t f[12] = {BITE_MARKER, (uint8_t)state, st,
                   (uint8_t)faultMask, (uint8_t)(faultMask >> 8), (uint8_t)(faultMask >> 16), (uint8_t)(faultMask >> 24),
                   (uint8_t)cv, (uint8_t)(cv >> 8), sats, (uint8_t)stateToken, (uint8_t)(stateToken >> 8)};
  queueRaw(f, sizeof f);
  lastSentMask = faultMask;
  lastBiteSentMs = millis();
}
void logBite(const char* why) {
  char l[N_FAULTS + 1];
  faultLetters(faultMask, l, sizeof l);
  logEvent("BITE,%s,mask=0x%06lX,faults=%s,load=%.2fV,%.0fmA,sats=%u,lin_max=%.2f,ds=%d", why,
           (unsigned long)faultMask, l[0] ? l : "none", loadV, inaCurrent,
           gps.satellites.isValid() ? (unsigned)gps.satellites.value() : 0u, linMaxWindow, dsValid);
}
// launch detection uses only the paths whose sensors passed BITE
void updateArming() {
  // Only a sensor that is not working disarms its path. "Not 1 g at rest" (R) and "moving" (W) are exactly what an
  // ignition looks like, so they must never disarm launch detection (a BITE at the moment of ignition did that).
  bool a = accValid && !(faultMask & (1u << 2));
  bool b = baroValid && altCalDone && !(faultMask & ((1u << 0) | (1u << 1)));
  if (a != accArmed || b != baroArmed) {
    accArmed = a; baroArmed = b;
    logEvent("LAUNCH_DETECT,acc=%s,baro=%s", a ? "ARMED" : "OFF", b ? "ARMED" : "OFF");
    if (!a && !b) logEvent("LAUNCH_DETECT,NO PATH ARMED: launch cannot be detected, timers will not start");
  }
}

// =====================================================================
//  CALIBRATION   altitude = zero the ground reference;  IMU = ADXL345 axis/offset + BNO08x gyro zero/spin axis
// =====================================================================
void resetCalAccumulators() {
  cP = cT = cG = 0;
  for (int i = 0; i < 3; i++) cA[i] = cGy[i] = cGr[i] = 0;
  cNP = cNA = cNG = cNB = cNR = 0; cMin = 1e9f; cMax = -1e9f;
}
void logCalEvent(const char* why) {
  // fields after CAL: n,why,p0_Pa,T0_K,sin_elev,acc_bias,acc_axis,acc_sign,axial_mean,gnss_ground_m,spin_axis,
  // spin_sign,gyro_bias_x,y,z (rad/s),alt_done,imu_done,acc_tilt_deg,bno_tilt_deg   (read back by the SD resume)
  logEvent("CAL,%lu,%s,%.2f,%.3f,%.5f,%.4f,%d,%d,%.4f,%.2f,%d,%d,%.5f,%.5f,%.5f,%d,%d,%.1f,%.1f", (unsigned long)calCount,
           why, p0Pa, t0K, sinElev, bias0, accAxis, accSign, axialMean0, isnan(gnssGround) ? -9999.0f : gnssGround,
           spinAxis, spinSign, gyroBias[0], gyroBias[1], gyroBias[2], altCalDone, imuCalDone, accTiltDeg, bnoTiltDeg);
}
bool padStill() { return !(cNA && cMax - cMin > CAL_MOVE_G * G0); }

CalResult calibrateAlt() {
  if (state > PRELAUNCH) return CAL_STATE;
  if (kf.launched()) return CAL_LAUNCHED;
  if (!padStill()) return CAL_MOVING;
  bool haveBaro = cNP >= CAL_MIN_SAMPLES, haveGnss = cNG > 10;
  if (!haveBaro && !haveGnss) return (baroOk || gpsFix()) ? CAL_FEW : CAL_NOSENSOR;
  if (haveBaro) { p0Pa = cP / cNP; t0K = cT / cNP + 273.15f; }
  if (haveGnss) {                            // GNSS fallback filter re-referenced, not restarted
    float ng = cG / cNG;
    if (gInit && !isnan(gnssGround)) gH -= ng - gnssGround;
    gnssGround = ng;
  }
  altCalDone = true;                         // pad altitude is recomputed from the raw pressure history
  return CAL_OK;
}
CalResult calibrateImu() {
  if (state > PRELAUNCH) return CAL_STATE;
  if (kf.launched()) return CAL_LAUNCHED;
  if (!accOk) return CAL_NOSENSOR;
  if (cNA < CAL_MIN_SAMPLES) return CAL_FEW;
  if (!padStill()) return CAL_MOVING;
  if (padValid && fabsf(padVLong) > CAL_VEL_MAX) return CAL_VELOCITY;  // offsets only with vertical velocity ~0
  float m[3] = {(float)(cA[0] / cNA), (float)(cA[1] / cNA), (float)(cA[2] / cNA)};
  float mag = sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
  if (mag < 5.0f) return CAL_NOSENSOR;
  // rocket axis: fixed by the mounting constant, or the axis closest to gravity (80-90 deg rail)
  int k = 0, sg;
  if (ADXL_ROCKET_AXIS >= 0) { k = ADXL_ROCKET_AXIS; sg = ADXL_ROCKET_SIGN; }
  else { for (int i = 1; i < 3; i++) if (fabsf(m[i]) > fabsf(m[k])) k = i; sg = m[k] >= 0 ? 1 : -1; }
  float along = sg * m[k];                   // specific force along the nose direction (+g sin(elevation))
  float tilt = along > 0 ? acosf(fminf(along / mag, 1.0f)) * RAD2DEG : 180.0f;
  if (tilt > AXIS_MAX_TILT_DEG) {
    logEvent("CAL_AXIS,adxl axis %d sign %d is %.1f deg from vertical (max %.0f): check mounting / rail", k, sg, tilt,
             AXIS_MAX_TILT_DEG);
    return CAL_AXIS;
  }
  accAxis = k; accSign = sg; accTiltDeg = tilt;
  axialMean0 = along;
  sinElev = along / mag;                     // sine of the rail elevation as the accelerometer sees it
  bias0 = mag - G0;                          // brings the measured |a| at rest to standard g
  if (cNB >= CAL_MIN_SAMPLES) {
    for (int i = 0; i < 3; i++) gyroBias[i] = cGy[i] / cNB;              // gyro zero
    if (cNR >= 5) {                                                       // spin axis from gravity / mounting
      float g[3] = {(float)(cGr[0] / cNR), (float)(cGr[1] / cNR), (float)(cGr[2] / cNR)};
      float gm = sqrtf(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
      int s = 0;
      if (BNO_ROCKET_AXIS >= 0) s = BNO_ROCKET_AXIS;
      else for (int i = 1; i < 3; i++) if (fabsf(g[i]) > fabsf(g[s])) s = i;
      float bt = gm > 1.0f ? acosf(fminf(fabsf(g[s]) / gm, 1.0f)) * RAD2DEG : -1.0f;
      spinAxis = s;
      spinSign = BNO_ROCKET_AXIS >= 0 ? BNO_ROCKET_SIGN : (g[s] >= 0 ? 1 : -1);
      bnoTiltDeg = bt;
      if (bt > AXIS_MAX_TILT_DEG) logEvent("CAL_AXIS,bno axis %d is %.1f deg from vertical: spin rate mixes in pitch/yaw", s, bt);
    }
  }
  imuCalDone = true;
  return CAL_OK;
}
// what: 1 altitude, 2 IMU, 3 both. Applies the new pad reference to the filter if anything changed.
void calibrate(uint8_t what, const char* why, CalResult& ra, CalResult& ri) {
  ra = ri = CAL_OK;
  if (what & 1) ra = calibrateAlt();
  if (what & 2) ri = calibrateImu();
  bool any = ((what & 1) && ra == CAL_OK) || ((what & 2) && ri == CAL_OK);
  if (any) {
    kf.setPad(t0K, p0Pa, sinElev, bias0, axialMean0);
    calCount++;
    indicatorPulse(CAL_PULSE_MS);
    logCalEvent(why);
    savePersist();
  }
  if ((what & 1) && ra != CAL_OK) logEvent("CAL_SKIP,altitude,%s,%s", CAL_TEXT[ra], why);
  if ((what & 2) && ri != CAL_OK) logEvent("CAL_SKIP,imu,%s,%s", CAL_TEXT[ri], why);
  resetCalAccumulators();
}

// =====================================================================
//  STATE MACHINE
// =====================================================================
bool debounce(bool cond, uint8_t& cnt, uint8_t n) {
  if (cond) { if (cnt < 255) cnt++; } else cnt = 0;
  return cnt >= n;
}
void markBootDone() {
  if (bootDone) return;
  bootDone = true;
  bootDoneMs = missionMs();
}

void enterState(FlightState next, const char* why) {
  if (next <= state) return;                       // forward only
  FlightState prev = state;
  state = next;
  stateToken++;
  stateEnterMs = missionMs();
  mvCoast.reset(); mvDoor.reset(); mvApogee.reset(); mvLand.reset();
  logEvent("TOKEN,%u,%s->%s,%s,alt=%.2f,vel=%.2f", stateToken, STATE_NAME[prev], STATE_NAME[next], why, altM, velMs);
  if (next == BOOST && FORCE_TELEM_IN_FLIGHT && !telemetryOn) {
    telemetryOn = true;
    logEvent("LORA,TELEMETRY_ON_IN_FLIGHT,ground_heard=%d,tx_muted=%d", groundHeard, txMuted);
  }
  if (next == APOGEE) tApogeeMs = stateEnterMs;
  if (next >= APOGEE && !(flags & FLAG_DEPLOY_RUN))
    startDeployment(next == APOGEE ? why : "late state entry",
                    strstr(why, "timeout") ? FLAG_BY_TIMEOUT : FLAG_BY_DETECT, DEPLOY_STEP_MS);
  if (next == RECOVERY) {
    powerDownSensors();
    if (pcaOk) pwm.sleep();
    indicatorRecovery = true;
    logEvent("FINAL,max_alt=%.1f,flight_time_s=%.1f,flags=0x%02X,lat=%.6f,lon=%.6f,gnss_alt=%.1f", hMax,
             tLaunchMs >= 0 ? (missionMs() - tLaunchMs) / 1000.0f : 0.0f, flags,
             gpsFix() ? gps.location.lat() : 0.0, gpsFix() ? gps.location.lng() : 0.0,
             gps.altitude.isValid() ? gps.altitude.meters() : 0.0);
    if (sdOk) { dataFile.flush(); eventFile.flush(); }
  }
  radioApplyPower();
  savePersist();
}

// back to PRE-LAUNCH from BOOST..RECOVERY: ground command FPL, or an unconfirmed launch (handling shock)
void forcePrelaunch(const char* why, bool byGround) {
  FlightState prev = state;
  if (state == RECOVERY) powerUpSensors();
  bool anyFired = flags & (FLAG_DOOR_OPEN | FLAG_CANSAT_OUT | FLAG_SEP_FIRED);
  deployActive = false;                            // pending deployment steps are cancelled
  if (!anyFired) flags &= ~(FLAG_BY_DETECT | FLAG_BY_TIMEOUT | FLAG_BY_MANUAL | FLAG_DEPLOY_RUN);
  tLaunchMs = tDoorMs = tApogeeMs = -1;
  doorByAlt = false;
  vPeak = hPeak = hMax = 0;
  dbLaunchAcc = dbLaunchBaro = 0;
  mvCoast.reset(); mvDoor.reset(); mvApogee.reset(); mvLand.reset();
  launchConfirmed = false;
  kf.setPad(t0K, p0Pa, sinElev, bias0, axialMean0);
  resetCalAccumulators();
  state = PRELAUNCH;
  stateToken++;
  stateEnterMs = missionMs();
  lastCalMs = stateEnterMs;
  if (byGround) forcedPrelaunch = true;
  markBootDone();
  logEvent("TOKEN,%u,%s->PRELAUNCH,%s,%s,alt=%.2f,flags=0x%02X,servos_fired=%d", stateToken, STATE_NAME[prev],
           byGround ? "FORCED" : "AUTO", why, altM, flags, anyFired);
  radioApplyPower();
  savePersist();
}

void updateEstimator() {
  const float dt = TICK_MS / 1000.0f;
  // pad barometer: altitude = mean of the last 1 s, velocity = least-squares slope over 0.5 s (and 1.6 s for the
  // calibration stillness check), all from the raw pressure history so a re-calibration shifts it consistently
  if (baroValid && altCalDone) {
    rpHist[rpIdx] = pressBaro;
    rpIdx = (rpIdx + 1) % RP_N;
    if (rpFill < RP_N) rpFill++;
  } else if (!baroValid) rpFill = 0;
  padValid = altCalDone && rpFill >= PAD_SLOPE_N;
  if (padValid) {
    float pr[RP_N], al[RP_N];
    int n = ringLast(rpHist, RP_N, rpIdx, rpFill, RP_N, pr);
    for (int i = 0; i < n; i++) al[i] = kf.baroAltitude(pr[i]);
    int m = min(n, PAD_MEAN_N);
    float sum = 0;
    for (int i = n - m; i < n; i++) sum += al[i];
    padH = sum / m;
    padV = lsqSlope(al + n - PAD_SLOPE_N, PAD_SLOPE_N, dt);
    padVLong = lsqSlope(al, n, dt);
  }
  // Accelerometer input for the filter. Without a valid accelerometer in flight: constant-velocity model during
  // the burn (thrust unknown), then free fall (gravity only) after burnout, which lags far less than constant
  // velocity near apogee. After separation the filter is in parachute mode anyway.
  float fIn = accValid ? accAx : axialMean0;
  if (state >= BOOST && !accValid && sensorsPowered && !(flags & FLAG_SEP_FIRED) && altCalDone) {
    bool burning = tLaunchMs >= 0 && (int64_t)missionMs() - tLaunchMs < BURN_EST_MS;
    if (burning) {
      if (!kf.chuteForced) { kf.chuteForced = true; logEvent("EKF,ACCEL_LOST,constant-velocity mode during burn"); }
    } else {
      if (kf.chuteForced || !kf.launched()) {
        float tl = tLaunchMs >= 0 ? kf.timeNow() - ((int64_t)missionMs() - tLaunchMs) / 1000.0f : kf.timeNow();
        kf.restore(kf.altitude(), kf.velocity(), kf.bias(), RocketKF::COAST, kf.timeNow(), tl, 0, false, kf.bias());
        logEvent("EKF,ACCEL_LOST,free-fall model after burnout");
      }
      fIn = kf.bias();                         // zero specific force: gravity only
    }
  }
  // EKF at 10 Hz on 2-sample averages; measurement = MS5611, else GNSS AGL, else none (prediction only)
  accAxSum += fIn;
  if (baroValid) { baroPSum += pressBaro; ekfBaroN++; }
  if (++ekfDiv >= 2) {
    ekfDiv = 0;
    if (altCalDone && sensorsPowered) {
      float h;
      if (ekfBaroN) h = kf.baroAltitude(baroPSum / ekfBaroN);
      else if (gInit && millis() - lastGAltMs < 500) h = gH;
      else h = kf.altitude();
      bool was = kf.launched();
      kf.step(accAxSum / 2, h);
      if (!was && kf.launched()) logEvent("EKF,LAUNCH,f=%.2f", accAxSum / 2);
    }
    accAxSum = baroPSum = 0;
    ekfBaroN = 0;
  }
  if (kf.launched() && !isnan(baroAlt)) hPeak = fmaxf(hPeak, baroAlt);
  if (kf.launched() && state == PRELAUNCH && (kf.timeNow() - kf.launchTime()) * 1000.0f > FALSE_LAUNCH_MS &&
      hPeak < 3.0f && tLaunchMs < 0) {
    logEvent("EKF,FALSE_LAUNCH");
    kf.setPad(t0K, p0Pa, sinElev, bias0, axialMean0);
    hPeak = 0;
  }
  // in flight the filter always supplies altitude and velocity (barometer, else GNSS, else accelerometer only)
  if (state >= BOOST && altCalDone) { altM = kf.altitude(); velMs = kf.velocity(); altSrc = 1; }
  else if (padValid) { altM = padH; velMs = padV; altSrc = 2; }
  else if (gInit) { altM = gH; velMs = gV; altSrc = 3; }
  else altSrc = 0;                                  // hold the last values
  if (state >= BOOST) { vPeak = fmaxf(vPeak, velMs); hMax = fmaxf(hMax, altM); }
  pushHistory(altM, velMs);
  // trends for the state machine: least-squares slopes over 0.5 s (altitude) and 1 s (velocity)
  float hb[SLOPE_V_N], vb[SLOPE_V_N];
  int nh = ringLast(hHist, HIST_N, histIdx, histFill, SLOPE_H_N, hb);
  int nv = ringLast(vHist, HIST_N, histIdx, histFill, SLOPE_V_N, vb);
  slopeH = lsqSlope(hb, nh, dt);
  slopeV = lsqSlope(vb, nv, dt);
  vMin1s = 1e9f;
  for (int i = 0; i < nv; i++) vMin1s = fminf(vMin1s, vb[i]);
}

// launch detection (PRE-LAUNCH, and BOOT once calibrated so a reboot shortly before ignition is still covered)
void checkLaunch(uint32_t now) {
  updateArming();
  bool acc = accArmed && debounce(accMag > LAUNCH_ACC_G * G0, dbLaunchAcc, LAUNCH_ACC_N);
  bool bar = baroArmed && debounce(padValid && padV > LAUNCH_BARO_VEL, dbLaunchBaro, LAUNCH_BARO_N);
  if (!acc && !bar) return;
  if (state == BOOT) {
    logBite("launch detected in BOOT");
    enterState(PRELAUNCH, "launch detected while in BOOT");
  }
  tLaunchMs = (int64_t)now - (int64_t)(LAUNCH_ACC_N - 1) * TICK_MS;
  launchConfirmed = false;
  launchBaroRef = baroArmed && baroValid;
  launchBaroMax = baroValid && !isnan(baroAlt) ? baroAlt : -1e9f;
  boostAccMs = boostAccRunMs = 0;
  boostAccGap = 0;
  enterState(BOOST, acc ? (bar ? "acc>2g 100ms & baro v>10" : "acc>2g 100ms") : "baro v>10 (no accel path)");
}

void updateState() {
  uint32_t now = missionMs();
  uint32_t inState = now - stateEnterMs;
  int64_t sinceLaunch = tLaunchMs >= 0 ? (int64_t)now - tLaunchMs : -1;
  bool hist = histFill >= SLOPE_V_N;

  // launch confirmation: a handling shock must never start the deployment timers
  if (state >= BOOST && state <= COAST && !launchConfirmed && tLaunchMs >= 0) {
    // a motor burn is continuous; separate handling shocks must not add up (one 50 ms dip tolerated)
    if (accValid && accMag > LAUNCH_ACC_G * G0) { boostAccRunMs += TICK_MS; boostAccGap = 0; }
    else if (++boostAccGap > 1) boostAccRunMs = 0;
    boostAccMs = max(boostAccMs, boostAccRunMs);
    if (baroValid && !isnan(baroAlt)) launchBaroMax = fmaxf(launchBaroMax, baroAlt);
    if (launchBaroMax >= LAUNCH_CONFIRM_ALT || boostAccMs >= LAUNCH_CONFIRM_ACC_MS) {
      launchConfirmed = true;
      logEvent("LAUNCH,CONFIRMED,baro_climb=%.1f,acc_above_2g_ms=%lu,t=%.2f", launchBaroMax, (unsigned long)boostAccMs,
               sinceLaunch / 1000.0f);
    } else if (sinceLaunch >= LAUNCH_CONFIRM_MS) {
      if (launchBaroRef && baroValid) {
        logEvent("LAUNCH,FALSE,baro_climb=%.1f,acc_above_2g_ms=%lu: handling shock, back to PRE-LAUNCH", launchBaroMax,
                 (unsigned long)boostAccMs);
        forcePrelaunch("unconfirmed launch", false);
        return;
      }
      launchConfirmed = true;
      logEvent("LAUNCH,UNCONFIRMED,barometer not usable: continuing on the timers");
    }
  }

  switch (state) {
    case BOOT: {
      if (!bootDone) {
        bool all = baroOk && accOk && bnoOk && inaOk && dsOk && pcaOk && sdOk && radioOk;
        bool critical = baroOk && accOk && pcaOk;
        if (all || (now >= BOOT_MAX_MS && critical)) {
          markBootDone();
          char l[N_FAULTS + 1];
          runBite(true);
          faultLetters(faultMask, l, sizeof l);
          logEvent("TOKEN,0,BOOT,%s,init_faults=%s", all ? "all peripherals up" : "15 s deadline, critical set up",
                   l[0] ? l : "none");
          CalResult a, b;
          if (cNP >= CAL_MIN_SAMPLES || cNA >= CAL_MIN_SAMPLES) { calibrate(3, "first", a, b); lastCalMs = now; }
        }
      }
      // PRE-LAUNCH enforced 60 s after power-up, whatever the faults
      if (now >= BOOT_FORCE_MS) {
        markBootDone();
        runBite(false);
        if (!altCalDone || !imuCalDone) { CalResult a, b; calibrate((altCalDone ? 0 : 1) | (imuCalDone ? 0 : 2), "pre-launch entry", a, b); }
        runBite(false);
        logBite("forced 60 s");
        updateArming();
        queueBiteFrame();
        enterState(PRELAUNCH, "forced 60 s after power-up");
        lastCalMs = now;
        break;
      }
      if (!bootDone) break;
      if (altCalDone || imuCalDone) {                 // launch detection already armed in BOOT
        checkLaunch(now);
        if (state != BOOT) break;
      }
      if ((!altCalDone || !imuCalDone) && now - lastCalMs >= 2000) {
        lastCalMs = now;
        CalResult a, b;
        calibrate((altCalDone ? 0 : 1) | (imuCalDone ? 0 : 2), "first", a, b);
      } else if (now - lastCalMs >= BITE_PERIOD_MS) {
        lastCalMs = now;
        CalResult a, b;
        calibrate(3, "periodic", a, b);
      }
      if (millis() - lastBiteMs >= BITE_BOOT_MS) {
        runBite(true);
        if (faultMask != lastSentMask || millis() - lastBiteSentMs >= BITE_SEND_MS) { logBite("boot"); queueBiteFrame(); }
        bool clean = !(faultMask & ~FAULT_INFO);
        if (clean || forcedPrelaunch) {
          logBite(clean ? "pass" : "forced");
          updateArming();
          queueBiteFrame();
          enterState(PRELAUNCH, clean ? "BITE pass, rocket still" : "forced by ground (FPL)");
        }
      }
      break;
    }
    case PRELAUNCH: {
      if (now - lastCalMs >= BITE_PERIOD_MS) {
        lastCalMs = now;
        CalResult a, b;
        calibrate(3, "periodic", a, b);
        runBite(false);
        logBite("periodic");
        if (faultMask != lastSentMask) queueBiteFrame();
      }
      checkLaunch(now);
      break;
    }
    case BOOST: {
      // nothing beyond BOOST before the launch is confirmed (a handling shock integrates into fake climb)
      bool c = launchConfirmed && hist && accMag < COAST_ACC_G * G0 && slopeH > 0 && slopeV < 0 && vMin1s > COAST_MIN_VEL;
      if (mvCoast.push(c, MV_N, MV_K)) enterState(COAST, "verified: |a|<2g, rising, v falling and >10 over 1 s");
      else if (sinceLaunch >= BOOST_TIMEOUT_MS && launchConfirmed) enterState(COAST, "timeout 4 s");
      break;
    }
    case COAST: {
      if (!(flags & FLAG_DOOR_OPEN)) {
        bool byVel = velMs < DOOR_VEL && slopeH > 0 && altM > MIN_DEPLOY_ALT_M;
        // 1000 m: filtered MS5611 first; GNSS AGL only when the barometer is not working
        float agl = gnssAgl();
        bool byAlt = baroValid ? (altM >= DOOR_ALT_M) : (!isnan(agl) && agl >= DOOR_ALT_M);
        if (mvDoor.push(byVel || byAlt, MV_N, MV_K)) {
          doorByAlt = !byVel;
          openDoor(byVel ? "v<20 rising" : (baroValid ? "baro alt>=1000" : "gnss agl>=1000 (baro failed)"), FLAG_BY_DETECT);
        }
        else if (sinceLaunch >= DOOR_TIMEOUT_MS) openDoor("timeout 15 s", FLAG_BY_TIMEOUT);
      }
      bool a = hist && slopeH > -APOGEE_STAT_MS && velMs < APOGEE_VEL && slopeV < 0 && hMax > MIN_DEPLOY_ALT_M;
      if (mvApogee.push(a, MV_N, MV_K)) enterState(APOGEE, "verified: v<10 falling, alt rising/stationary");
      else if (tDoorMs >= 0 && (!doorByAlt || APOGEE_TIMER_AFTER_ALT_DOOR) && (int64_t)now - tDoorMs >= APOGEE_AFTER_DOOR_MS)
        enterState(APOGEE, "timeout 3 s after door");
      else if (sinceLaunch >= APOGEE_TIMEOUT_MS) enterState(APOGEE, "timeout 17.5 s");
      break;
    }
    case APOGEE:
      if (inState >= DESCENT_DELAY_MS) enterState(DESCENT, "4 s after apogee");
      break;
    case DESCENT:
      if (mvLand.push(altM < LANDING_ALT_M, LAND_MV_N, LAND_MV_K)) enterState(LANDING, "alt<25 m over 0.5 s");
      else if (inState >= DESCENT_TIMEOUT_MS) enterState(LANDING, "timeout 750 s");
      break;
    case LANDING:
      if (inState >= RECOVERY_DELAY_MS) enterState(RECOVERY, "10 s after landing");
      break;
    case RECOVERY:
      break;
  }
}

// =====================================================================
//  SD DATA ROW (20 Hz)
// =====================================================================
const char* DATA_HEADER =
  "mission_ms,mission_time_s,telem_time_s,gnss_utc,gps_epoch_ms,state,state_name,token,flags,"
  "altitude_m,velocity_ms,ekf_bias_ms2,alt_source,baro_valid,baro_altitude_m,baro_velocity_ms,pressure_pa,"
  "pressure_source,ms5611_temp_c,ds18b20_temp_c,ds_valid,"
  "acc_x_ms2,acc_y_ms2,acc_z_ms2,acc_axial_ms2,acc_mag_ms2,acc_peak_ms2,acc_samples,"
  "gyro_x_dps,gyro_y_dps,gyro_z_dps,spin_dps,lin_acc_ms2,roll_deg,pitch_deg,yaw_deg,bno_ok,"
  "latitude_deg,longitude_deg,gnss_altitude_m,gnss_agl_m,gnss_vel_ms,satellites,hdop,gps_fix,gps_baud,pps_count,"
  "load_v,ina_bus_v,ina_shunt_mv,ina_current_ma,"
  "channel,freq_trial,ground_heard,tx_muted,tx_cutoff,telemetry_on,lora_dbm,packet_count,rx_good,rx_crc,"
  "lora_reinits,gps_resets,buzzer,"
  "door_cmd_deg,payload_cmd_deg,sep_cmd_deg,door_est_deg,payload_est_deg,sep_est_deg,door_us,payload_us,sep_us,"
  "fault_mask,t_launch_ms,t_door_ms,t_apogee_ms,t_state_ms,payload_due_ms,sep_due_ms,deploy_active,"
  "v_peak_ms,h_peak_m,h_max_m,cal_count,warm_resets,ekf_phase,boot_done,boot_done_ms,"
  "lora_tx,alt_slope_ms,vel_slope_ms2,launch_confirmed,ekf_baro_only,rtc_ticks,door_by_alt";

uint32_t telemTimeS() { return bootDone ? (missionMs() - bootDoneMs) / 1000 : 0; }

void logRow() {
  if (!sdOk || !dataFile || refreshPending) return;
  char gt[32];
  gpsUtcStr(gt, sizeof gt);
  bool fix = gpsFix();
  float agl = gnssAgl();
  uint32_t t = missionMs();
  dataFile.printf(
    "%lu,%lu.%03lu,%lu,%s,%llu,%u,%s,%u,0x%02X,"
    "%.2f,%.2f,%.3f,%u,%d,%.2f,%.2f,%.1f,"
    "%u,%.2f,%.2f,%d,"
    "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,"
    "%.2f,%.2f,%.2f,%.2f,%.3f,%.1f,%.1f,%.1f,%d,"
    "%.7f,%.7f,%.1f,%.1f,%.2f,%u,%.2f,%d,%lu,%lu,"
    "%.3f,%.3f,%.2f,%.1f,"
    "%u,%d,%d,%d,%d,%d,%d,%lu,%lu,%lu,"
    "%lu,%lu,%d,"
    "%d,%d,%d,%.1f,%.1f,%.1f,%u,%u,%u,"
    "0x%06lX,%lld,%lld,%lld,%lu,%lu,%lu,%d,"
    "%.2f,%.2f,%.2f,%lu,%lu,%d,%d,%lu,"
    "%d,%.2f,%.2f,%d,%d,%llu,%d\n",
    (unsigned long)t, (unsigned long)(t / 1000), (unsigned long)(t % 1000), (unsigned long)telemTimeS(), gt,
    (unsigned long long)gpsEpochMs(), (unsigned)state, STATE_NAME[state], stateToken, (unsigned)flags,
    altM, velMs, kf.bias(), (unsigned)altSrc, baroValid ? 1 : 0, isnan(baroAlt) ? -9999.0f : baroAlt, padV, pressPa,
    (unsigned)pressSrc, baroTempC, isnan(dsTempC) ? -99.0f : dsTempC, dsValid ? 1 : 0,
    ax, ay, az, accAx, accMag, accPeak, (unsigned)accN,
    gyro[0] * RAD2DEG, gyro[1] * RAD2DEG, gyro[2] * RAD2DEG, spinDps, linAccMag, roll, pitch, yaw, bnoOk ? 1 : 0,
    fix ? gps.location.lat() : 0.0, fix ? gps.location.lng() : 0.0,
    (fix && gps.altitude.isValid()) ? gps.altitude.meters() : 0.0, isnan(agl) ? -9999.0f : agl, gV,
    gps.satellites.isValid() ? (unsigned)gps.satellites.value() : 0u, gps.hdop.isValid() ? gps.hdop.hdop() : 99.99,
    fix ? 1 : 0, (unsigned long)gpsBaud, (unsigned long)ppsCount,
    loadV, inaBusV, inaShuntMv, inaCurrent,
    (unsigned)curCh, freqTrial ? 1 : 0, groundHeard ? 1 : 0, txMuted ? 1 : 0, txCutoff ? 1 : 0, telemetryOn ? 1 : 0,
    (int)radioPowerNow, (unsigned long)packetCount, (unsigned long)rxGood, (unsigned long)rxCrc,
    (unsigned long)loraReinits, (unsigned long)gpsReboots, indicatorIsOn() ? 1 : 0,
    servoDeg[0], servoDeg[1], servoDeg[2], servoEst[0], servoEst[1], servoEst[2], servoUs[0], servoUs[1], servoUs[2],
    (unsigned long)faultMask, (long long)tLaunchMs, (long long)tDoorMs, (long long)tApogeeMs, (unsigned long)stateEnterMs,
    (unsigned long)payloadDueMs, (unsigned long)sepDueMs, deployActive ? 1 : 0,
    vPeak, hPeak, hMax, (unsigned long)calCount, (unsigned long)resetCount, kf.phase(), bootDone ? 1 : 0,
    (unsigned long)bootDoneMs, radioMode == RADIO_TX ? 1 : 0, slopeH, slopeV, launchConfirmed ? 1 : 0,
    kf.chuteForced && !(flags & FLAG_SEP_FIRED) ? 1 : 0, (unsigned long long)rtcTicks(), doorByAlt ? 1 : 0);
}

// =====================================================================
//  PERSISTENCE (RAM2 / DMAMEM survives software and watchdog resets) + SD fallback
// =====================================================================
#define PERSIST_MAGIC   0x54524B54u
#define PERSIST_VERSION 3
DMAMEM Persist persist;

uint32_t persistCrc(const Persist& p) {
  const uint8_t* b = (const uint8_t*)&p;
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < offsetof(Persist, crc); i++) { h ^= b[i]; h *= 16777619u; }
  return h;
}
uint64_t rtcTicks() {
  uint32_t hi1, lo, hi2;
  do { hi1 = SNVS_HPRTCMR; lo = SNVS_HPRTCLR; hi2 = SNVS_HPRTCMR; } while (hi1 != hi2);
  return ((uint64_t)hi1 << 32) | lo;
}

void savePersist() {
  Persist& p = persist;
  p.magic = PERSIST_MAGIC; p.version = PERSIST_VERSION;
  p.missionMs = missionMs(); p.rtcTicks = rtcTicks();
  p.state = state; p.flags = flags; p.bootDone = bootDone; p.forced = forcedPrelaunch;
  p.token = stateToken; p.stateEnterMs = stateEnterMs; p.bootDoneMs = bootDoneMs;
  p.tLaunchMs = tLaunchMs; p.tDoorMs = tDoorMs; p.tApogeeMs = tApogeeMs;
  p.payloadDueMs = payloadDueMs; p.sepDueMs = sepDueMs; p.deployActive = deployActive;
  p.kfPhase = kf.phase(); p.kfChute = kf.chuteForced; p.altCalDone = altCalDone; p.imuCalDone = imuCalDone;
  p.doorByAlt = doorByAlt;
  p.vPeak = vPeak; p.hPeak = hPeak; p.hMax = hMax;
  p.kfH = kf.altitude(); p.kfV = kf.velocity(); p.kfB = kf.bias();
  p.kfTLaunch = kf.launchTime(); p.kfTDet = kf.detTime(); p.kfTNow = kf.timeNow(); p.accAxLast = accAx;
  p.calCount = calCount; p.lastCalMs = lastCalMs;
  p.p0Pa = p0Pa; p.t0K = t0K; p.sinElev = sinElev; p.bias0 = bias0; p.axialMean0 = axialMean0; p.gnssGround = gnssGround;
  p.accAxis = accAxis; p.accSign = accSign; p.spinAxis = spinAxis; p.spinSign = spinSign;
  for (int i = 0; i < 3; i++) p.gyroBias[i] = gyroBias[i];
  p.telemetryOn = telemetryOn; p.indicatorManual = indicatorManual; p.groundHeard = groundHeard;
  p.txMuted = txMuted; p.txCutoff = txCutoff; p.curCh = curCh;
  p.packetCount = packetCount;
  for (int i = 0; i < SERVO_COUNT; i++) { p.servoDeg[i] = servoDeg[i]; p.servoUs[i] = servoUs[i]; }
  p.runIndex = runIndex; p.resetCount = resetCount;
  p.crc = persistCrc(p);
  arm_dcache_flush(&persist, sizeof persist);
}

bool inFlight(uint8_t st) { return st >= BOOST && st <= LANDING; }

bool tryResumeRam(const char*& how, int64_t& gapMs) {
  arm_dcache_delete(&persist, sizeof persist);
  const Persist& p = persist;
  if (p.magic != PERSIST_MAGIC || p.version != PERSIST_VERSION || p.crc != persistCrc(p)) return false;
  if (!warmBoot && !inFlight(p.state)) return false;
  int64_t gapTicks = (int64_t)(rtcTicks() - p.rtcTicks);
  gapMs = gapTicks * 1000 / 32768;
  how = "ram+rtc";
  if (gapMs < 0 || gapMs > RESUME_RAM_GAP_MS) {
    if (!warmBoot) return false;
    gapMs = (int64_t)millis() + BOOT_OVERHEAD_MS; how = "ram+estimate";
  }
  missionOffsetMs = (int64_t)p.missionMs + gapMs - (int64_t)millis();
  state = (FlightState)constrain(p.state, 0, 7);
  flags = p.flags; bootDone = p.bootDone; forcedPrelaunch = p.forced;
  stateToken = p.token; stateEnterMs = p.stateEnterMs; bootDoneMs = p.bootDoneMs;
  tLaunchMs = p.tLaunchMs; tDoorMs = p.tDoorMs; tApogeeMs = p.tApogeeMs;
  payloadDueMs = p.payloadDueMs; sepDueMs = p.sepDueMs; deployActive = p.deployActive;
  vPeak = p.vPeak; hPeak = p.hPeak; hMax = p.hMax;
  calCount = p.calCount; lastCalMs = p.lastCalMs; altCalDone = p.altCalDone; imuCalDone = p.imuCalDone;
  doorByAlt = p.doorByAlt;
  p0Pa = p.p0Pa; t0K = p.t0K; sinElev = p.sinElev; bias0 = p.bias0; axialMean0 = p.axialMean0; gnssGround = p.gnssGround;
  accAxis = p.accAxis; accSign = p.accSign; spinAxis = p.spinAxis; spinSign = p.spinSign;
  for (int i = 0; i < 3; i++) gyroBias[i] = p.gyroBias[i];
  telemetryOn = p.telemetryOn; indicatorManual = p.indicatorManual; groundHeard = p.groundHeard;
  txMuted = p.txMuted; txCutoff = p.txCutoff; curCh = p.curCh ? 1 : 0;
  packetCount = p.packetCount;
  for (int i = 0; i < SERVO_COUNT; i++) { servoDeg[i] = p.servoDeg[i]; servoUs[i] = p.servoUs[i]; }
  runIndex = p.runIndex;
  resetCount = p.resetCount + 1;
  kf.setPad(t0K, p0Pa, sinElev, bias0, axialMean0);
  if (p.kfPhase != 0) {
    float g = gapMs / 1000.0f;
    float h = p.kfH + p.kfV * g, v = p.kfV;
    if (!p.kfChute) { h -= 0.5f * G0 * g * g; v -= G0 * g; }
    kf.restore(h, v, p.kfB, p.kfPhase, p.kfTNow + g, p.kfTLaunch, p.kfTDet, p.kfChute, p.accAxLast);
    kf.P[0][0] = 100; kf.P[1][1] = 100;
  }
  return true;
}

// ---------------------------------------------------------------- SD files
int newestLogIndex() {
  int best = -1;
  File root = SD.open("/");
  if (!root) return -1;
  const char* pre = FILE_PREFIX "_FLIGHT_";
  size_t pl = strlen(pre);
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    const char* n = f.name();
    if (!strncmp(n, pre, pl) && strlen(n) == pl + 7 && !strcmp(n + pl + 3, ".csv")) best = max(best, atoi(n + pl));
    f.close();
  }
  root.close();
  return best;
}
void setLogNames(int k) {
  runIndex = k;
  snprintf(dataName, sizeof dataName, "/" FILE_PREFIX "_FLIGHT_%03d.csv", k);
  snprintf(eventName, sizeof eventName, "/" FILE_PREFIX "_EVENTS_%03d.csv", k);
}
bool openNewLogs() {
  setLogNames(newestLogIndex() + 1);
  dataFile = SD.open(dataName, FILE_WRITE);
  eventFile = SD.open(eventName, FILE_WRITE);
  if (!dataFile || !eventFile) return false;
  dataFile.println(DATA_HEADER);
  eventFile.println("mission_ms,mission_time_s,gnss_utc,event");
  for (int i = 0; i < N_FAULTS; i++) eventFile.printf("0,0.000,-,LEGEND,BITE %s\n", FAULT_TEXT[i]);
  dataFile.flush(); eventFile.flush();
  return true;
}
bool reopenLogs(int k) {
  if (k < 0) return false;
  setLogNames(k);
  if (!SD.exists(dataName) || !SD.exists(eventName)) return false;
  dataFile = SD.open(dataName, FILE_WRITE);        // Teensy FILE_WRITE appends
  eventFile = SD.open(eventName, FILE_WRITE);
  if (!dataFile || !eventFile) return false;
  File* fs[2] = {&dataFile, &eventFile};
  const char* nm[2] = {dataName, eventName};
  for (int i = 0; i < 2; i++) {                     // a row cut by the reset must not merge with the next one
    File* f = fs[i];
    uint64_t n = f->size();
    if (!n) continue;
    File r = SD.open(nm[i], FILE_READ);
    if (!r) continue;
    r.seek(n - 1);
    int c = r.read();
    r.close();
    if (c != '\n') f->println();
  }
  return true;
}

template <typename F>
bool scanTail(const char* path, size_t chunk, F onLine) {
  File f = SD.open(path, FILE_READ);
  if (!f) return false;
  size_t size = f.size(), start = size > chunk ? size - chunk : 0;
  char* buf = (char*)malloc(chunk + 1);
  if (!buf) { f.close(); return false; }
  f.seek(start);
  size_t n = f.read((uint8_t*)buf, chunk);
  f.close();
  buf[n] = '\0';
  char* end = buf + n;
  bool first = true;
  while (end > buf) {
    char* e = end;
    while (e > buf && (e[-1] == '\n' || e[-1] == '\r')) e--;
    char* b = e;
    while (b > buf && b[-1] != '\n') b--;
    if (b == buf && start > 0) break;
    *e = '\0';
    if (e > b && onLine(b, first)) break;
    if (e > b) first = false;
    end = b;
  }
  free(buf);
  return true;
}
int splitCsv(char* s, char** f, int maxf) {
  int n = 0;
  char* save;
  for (char* p = strtok_r(s, ",", &save); p && n < maxf; p = strtok_r(nullptr, ",", &save)) f[n++] = p;
  return n;
}
bool readFirstLine(const char* path, char* out, size_t n) {
  File f = SD.open(path, FILE_READ);
  if (!f) return false;
  size_t k = 0;
  while (f.available() && k < n - 1) {
    char c = f.read();
    if (c == '\n' || c == '\r') break;
    out[k++] = c;
  }
  out[k] = '\0';
  f.close();
  return k > 0;
}
// column index of `name` in a CSV header line, -1 if missing
int colIndex(const char* header, const char* name) {
  size_t nl = strlen(name);
  int idx = 0;
  const char* p = header;
  while (*p) {
    const char* e = strchr(p, ',');
    size_t len = e ? (size_t)(e - p) : strlen(p);
    if (len == nl && !strncmp(p, name, nl)) return idx;
    if (!e) break;
    p = e + 1;
    idx++;
  }
  return -1;
}

// fallback when RAM did not survive: last SD row + last CAL event, checked against GNSS time
bool tryResumeSd(const char*& how, int64_t& gapOut) {
  if (!sdOk) return false;
  int k = newestLogIndex();
  if (k < 0) return false;
  setLogNames(k);
  static char header[1800], row[1400], cal[300];
  row[0] = cal[0] = '\0';
  if (!readFirstLine(dataName, header, sizeof header)) return false;
  bool lastIsRefresh = false;
  int hdrCommas = 0;
  for (const char* q = header; *q; q++) hdrCommas += *q == ',';
  scanTail(dataName, 16384, [&](char* line, bool) {
    if (!isdigit((unsigned char)line[0]) || strlen(line) >= sizeof row) return false;
    int c = 0;
    for (const char* q = line; *q; q++) c += *q == ',';
    if (c != hdrCommas) return false;              // row cut short by the reset: use the one before
    strcpy(row, line);
    return true;
  });
  scanTail(eventName, 65536, [&](char* line, bool first) {
    char* c1 = strchr(line, ',');
    char* c2 = c1 ? strchr(c1 + 1, ',') : nullptr;
    char* c3 = c2 ? strchr(c2 + 1, ',') : nullptr;
    if (!c3) return false;
    if (first && !strncmp(c3 + 1, "REFRESH", 7)) lastIsRefresh = true;
    if (!strncmp(c3 + 1, "CAL,", 4) && strlen(line) < sizeof cal) { strcpy(cal, line); return true; }
    return false;
  });
  if (!row[0] || !cal[0]) return false;
  static char* F[128];
  int nf = splitCsv(row, F, 128);
  auto col = [&](const char* name) -> const char* {
    int i = colIndex(header, name);
    return (i >= 0 && i < nf) ? F[i] : "0";
  };
  if (colIndex(header, "boot_done_ms") < 0 || colIndex(header, "boot_done_ms") >= nf) return false;   // other layout
  char* C[24];
  if (splitCsv(cal, C, 24) < 21) return false;   // ms,s,utc,CAL,n,why,p0,T0K,sinE,b0,axis,sign,axialMean,gnssG,spinAx,spinSg,gb0,gb1,gb2,alt,imu
  uint8_t st = constrain(atoi(col("state")), 0, 7);
  if (!warmBoot && !inFlight(st)) return false;

  int64_t gap = -1;
  // 1) battery-backed RTC (coin cell on VBAT keeps it counting through a power loss)
  uint64_t rtcLogged = strtoull(col("rtc_ticks"), nullptr, 10), rtcNow = rtcTicks();
  if (rtcLogged && rtcNow >= rtcLogged) {
    int64_t g = (int64_t)((rtcNow - rtcLogged) * 1000ULL / 32768ULL);
    if (g <= (int64_t)RESUME_SD_GAP_S * 1000) { gap = g; how = "sd+rtc"; }
  }
  // 2) GNSS time. The module is not reset with the Teensy and may still be at 115200 baud from the configuration.
  uint64_t logged = strtoull(col("gps_epoch_ms"), nullptr, 10), nowEp = 0;
  if (gap < 0) {
    if (!gpsDetect(115200, 500)) gpsDetect(9600, 1200);
    uint32_t t0 = millis();
    while (millis() - t0 < 1200 && !(nowEp = gpsEpochMs())) {
      while (GNSS_SERIAL.available()) gps.encode(GNSS_SERIAL.read());
      delay(5);
    }
  }
  if (gap >= 0) {
  } else if (logged && nowEp && nowEp >= logged && nowEp - logged <= (uint64_t)RESUME_SD_GAP_S * 1000ULL) {
    gap = (int64_t)(nowEp - logged); how = "sd+gnss";
  } else if ((!logged || !nowEp) && lastIsRefresh && warmBoot) {
    gap = (int64_t)millis() + BOOT_OVERHEAD_MS; how = "sd+refresh-marker";
  }
  if (gap < 0) return false;
  gapOut = gap;

  missionOffsetMs = (int64_t)strtoul(col("mission_ms"), nullptr, 10) + gap - (int64_t)millis();
  state = (FlightState)st;
  stateToken = atoi(col("token"));
  flags = (uint8_t)strtoul(col("flags"), nullptr, 0);
  float h = atof(col("altitude_m")), v = atof(col("velocity_ms")), b = atof(col("ekf_bias_ms2"));
  groundHeard = atoi(col("ground_heard")); txMuted = atoi(col("tx_muted")); telemetryOn = atoi(col("telemetry_on"));
  curCh = atoi(col("channel")) ? 1 : 0;
  packetCount = strtoul(col("packet_count"), nullptr, 10);
  const char* scmd[3] = {"door_cmd_deg", "payload_cmd_deg", "sep_cmd_deg"};
  const char* sus[3] = {"door_us", "payload_us", "sep_us"};
  for (int i = 0; i < SERVO_COUNT; i++) { servoDeg[i] = atoi(col(scmd[i])); servoUs[i] = atoi(col(sus[i])); }
  tLaunchMs = atoll(col("t_launch_ms")); tDoorMs = atoll(col("t_door_ms")); tApogeeMs = atoll(col("t_apogee_ms"));
  stateEnterMs = strtoul(col("t_state_ms"), nullptr, 10);
  payloadDueMs = strtoul(col("payload_due_ms"), nullptr, 10); sepDueMs = strtoul(col("sep_due_ms"), nullptr, 10);
  deployActive = atoi(col("deploy_active"));
  doorByAlt = atoi(col("door_by_alt"));
  vPeak = atof(col("v_peak_ms")); hPeak = atof(col("h_peak_m")); hMax = atof(col("h_max_m"));
  bootDone = atoi(col("boot_done")); bootDoneMs = strtoul(col("boot_done_ms"), nullptr, 10);
  int ph = atoi(col("ekf_phase"));
  calCount = strtoul(C[4], nullptr, 10);
  p0Pa = atof(C[6]); t0K = atof(C[7]); sinElev = atof(C[8]); bias0 = atof(C[9]);
  accAxis = constrain(atoi(C[10]), 0, 2); accSign = atoi(C[11]) < 0 ? -1 : 1; axialMean0 = atof(C[12]);
  float gg = atof(C[13]); gnssGround = gg < -9000 ? NAN : gg;
  spinAxis = constrain(atoi(C[14]), 0, 2); spinSign = atoi(C[15]) < 0 ? -1 : 1;
  gyroBias[0] = atof(C[16]); gyroBias[1] = atof(C[17]); gyroBias[2] = atof(C[18]);
  altCalDone = atoi(C[19]); imuCalDone = atoi(C[20]);
  kf.setPad(t0K, p0Pa, sinElev, bias0, axialMean0);
  if (ph != 0) {
    float tn = missionMs() / 1000.0f, g = gap / 1000.0f;
    bool chute = (flags & FLAG_SEP_FIRED) != 0;
    h += v * g;
    if (!chute) { h -= 0.5f * G0 * g * g; v -= G0 * g; }
    kf.restore(h, v, b, ph, tn, tLaunchMs >= 0 ? tLaunchMs / 1000.0f : tn, tn, chute, G0);
    kf.P[0][0] = 400; kf.P[1][1] = 400;            // 20 m / 20 m/s: the gap may have been under thrust
    hPeak = fmaxf(hPeak, 3.0f);
  }
  lastCalMs = missionMs();
  resetCount = 1;
  return true;
}

void startRefresh() {
  logRow();
  logEvent("REFRESH,epoch_ms=%llu", (unsigned long long)gpsEpochMs());
  if (sdOk) { dataFile.flush(); eventFile.flush(); }
  savePersist();
  refreshPending = true;
  refreshAskedMs = millis();
}
// actions that must wait until the reply has gone out on the air
void serviceDeferred() {
  bool sent = rqCount == 0 && radioMode != RADIO_TX;
  if (loraReinitPending && (sent || millis() - loraReinitAskedMs > 1500)) { loraReinitPending = false; radioReinit("commanded"); }
  if (txMutePending && (sent || millis() - txMuteAskedMs > 1500) && radioMode != RADIO_TX) {
    txMutePending = false;
    txMuted = true;
    telemDue = false;
    rqCount = 0;
    logEvent("LORA,TX_MUTED (receiver stays on)");
    savePersist();
  }
  if (freqPendingCh >= 0 && (sent || millis() - freqAskedMs > 1500) && radioMode != RADIO_TX) {
    prevCh = curCh;
    uint8_t ch = freqPendingCh;
    freqPendingCh = -1;
    retune(ch, "commanded FQ, trial");
    freqTrial = true;
    freqTrialMs = millis();
  }
  if (freqTrial && millis() - freqTrialMs > FREQ_TRIAL_MS && radioMode != RADIO_TX) {
    freqTrial = false;
    retune(prevCh, "nothing heard on new channel in 10 s: reverted");
  }
  if (refreshPending && (sent || millis() - refreshAskedMs > 2500)) {
    savePersist();
    if (sdOk) { dataFile.close(); eventFile.close(); }
    Serial.println("[REFRESH] restarting");
    Serial.flush();
    delay(20);
    SCB_AIRCR = 0x05FA0004;
    while (1) {}
  }
}

// =====================================================================
//  COMMANDS
// =====================================================================
bool toLong(const char* s, long& v) {
  if (!s || !*s) return false;
  char* end;
  v = strtol(s, &end, 10);
  return *end == '\0';
}
const Code CODES[] = {
  {'P', "PG"},   {'K', "LK"},     {'S', "SD"},     {'X', "XD"},      {'C', "CAL"},    {'B', "BITE"},
  {'T', "ST"},   {'V', "SV"},     {'W', "PW"},     {'L', "LOCK"},    {'U', "UNLOCK"}, {'A', "LOCKALL"},
  {'E', "TEST"}, {'M', "MSEQ"},   {'N', "NSTEP"},  {'F', "FSEQ"},    {'Q', "SEQRST"}, {'J', "SEP"},
  {'Z', "BZ"},   {'G', "GR"},     {'O', "LR"},     {'R', "RF"},      {'I', "FPL"},
};
bool expandCode(const char* s, char* out, size_t n) {
  char c = toupper((unsigned char)s[0]);
  if (!(s[1] == '\0' || isdigit((unsigned char)s[1]) || s[1] == '-')) return false;
  for (const Code& k : CODES) {
    if (k.c != c) continue;
    if (!strcmp(k.name, "TEST")) snprintf(out, n, "TEST%s", s + 1);
    else if (s[1]) snprintf(out, n, "%s,%s", k.name, s + 1);
    else snprintf(out, n, "%s", k.name);
    return true;
  }
  return false;
}
bool servoChArg(int n, char** tok, long& ch) { return n >= 2 && toLong(tok[1], ch) && ch >= 0 && ch < SERVO_COUNT; }

void handleCommand(char* raw, bool fromRadio, float rssi, float snr) {
  while (*raw == ' ') raw++;
  char* e = raw + strlen(raw);
  while (e > raw && (e[-1] == '\r' || e[-1] == '\n' || e[-1] == ' ')) *--e = '\0';
  if (!strncmp(raw, "$C,", 3)) raw += 3;
  else if (raw[0] == '$' || (uint8_t)raw[0] >= 0x80) return;    // another vehicle's packet / binary
  if (!*raw) return;
  char expanded[64];
  if (expandCode(raw, expanded, sizeof expanded)) raw = expanded;
  char copy[64];
  strncpy(copy, raw, sizeof copy - 1); copy[sizeof copy - 1] = '\0';
  char* tok[4] = {nullptr, nullptr, nullptr, nullptr};
  int n = 0;
  char* save;
  for (char* t = strtok_r(copy, ",", &save); t && n < 4; t = strtok_r(nullptr, ",", &save)) tok[n++] = t;
  if (!n) return;
  for (int i = 0; i < n; i++) for (char* p = tok[i]; *p; p++) *p = toupper((unsigned char)*p);
  const char* cmd = tok[0];

  if (fromRadio) {
    lastGroundRxMs = millis();
    if (!groundHeard) { groundHeard = true; logEvent("LORA,GROUND_HEARD,first command=%s,TX now allowed", cmd); }
    if (freqTrial) { freqTrial = false; logEvent("LORA,CHANNEL,%u,CONFIRMED", curCh); savePersist(); }
  }
  if (!strcmp(cmd, "LK")) { reply(fromRadio, "$H,ACK,LK,%u,%lu", (unsigned)state, (unsigned long)packetCount); return; }
  logEvent("CMD,%s,%s,rssi=%.0f,snr=%.1f", fromRadio ? "LORA" : "USB", raw, rssi, snr);

  static const char* servoCmds[] = {"SV", "PW", "LOCK", "UNLOCK", "LOCKALL", "TEST1", "TEST2", "TEST3", "MSEQ", "NSTEP", "FSEQ"};
  bool isServo = false;
  for (const char* s : servoCmds) if (!strcmp(cmd, s)) isServo = true;
  if (isServo && state == BOOST) { reply(fromRadio, "$H,NACK,%s,BOOST", cmd); return; }
  if (isServo && !pcaOk) { reply(fromRadio, "$H,NACK,%s,PCA", cmd); return; }
  const char* src = fromRadio ? "ground cmd" : "usb cmd";
  long a = 0, b = 0;

  if (!strcmp(cmd, "PG")) reply(fromRadio, "$H,ACK,PG,%d,%.1f,%u", (int)lroundf(rssi), snr, curCh);
  else if (!strcmp(cmd, "SD")) { telemetryOn = true; reply(fromRadio, "$H,ACK,SD"); }
  else if (!strcmp(cmd, "XD")) { telemetryOn = false; reply(fromRadio, "$H,ACK,XD"); }
  else if (!strcmp(cmd, "CAL")) {
    // CAL / CAL,0 = all, CAL,1 = altitude (CALA), CAL,2 = IMU (CALI)
    long w = 0;
    if (n >= 2 && (!toLong(tok[1], w) || w < 0 || w > 2)) { reply(fromRadio, "$H,NACK,CAL,ARGS"); return; }
    const char* nm = w == 1 ? "CALA" : (w == 2 ? "CALI" : "CAL");
    if (state > PRELAUNCH) { reply(fromRadio, "$H,NACK,%s,STATE", nm); return; }
    CalResult ra, ri;
    calibrate(w == 1 ? 1 : (w == 2 ? 2 : 3), "commanded", ra, ri);
    lastCalMs = missionMs();
    if (w == 1) {
      if (ra == CAL_OK) reply(fromRadio, "$H,ACK,CALA,%lu,%ld,%ld,%ld", (unsigned long)calCount, lroundf(p0Pa),
                              lroundf((t0K - 273.15f) * 10), isnan(gnssGround) ? -99999L : lroundf(gnssGround * 10));
      else reply(fromRadio, "$H,NACK,CALA,%s", CAL_TEXT[ra]);
    } else if (w == 2) {
      if (ri == CAL_OK) reply(fromRadio, "$H,ACK,CALI,%lu,%d,%d,%ld,%ld,%d,%d,%d,%ld", (unsigned long)calCount, accAxis,
                              accSign, lroundf(bias0 * 1000), lroundf(sinElev * 10000), spinAxis, spinSign, bnoOk ? 1 : 0,
                              lroundf(bnoTiltDeg * 10));
      else reply(fromRadio, "$H,NACK,CALI,%s", CAL_TEXT[ri]);
    } else {
      if (ra == CAL_OK || ri == CAL_OK) reply(fromRadio, "$H,ACK,CAL,%lu,%s,%s", (unsigned long)calCount, CAL_TEXT[ra], CAL_TEXT[ri]);
      else reply(fromRadio, "$H,NACK,CAL,%s,%s", CAL_TEXT[ra], CAL_TEXT[ri]);
    }
  } else if (!strcmp(cmd, "BITE")) {
    runBite(state <= PRELAUNCH);
    logBite("commanded");
    if (fromRadio) queueBiteFrame();
    else {
      char l[N_FAULTS + 1];
      faultLetters(faultMask, l, sizeof l);
      Serial.printf("$H,BITE,0x%06lX,%s\n", (unsigned long)faultMask, l);
      for (int i = 0; i < N_FAULTS; i++) if (faultMask & (1u << i)) Serial.printf("   %s\n", FAULT_TEXT[i]);
    }
  } else if (!strcmp(cmd, "FPL")) {
    bool any = n >= 2 && !strcmp(tok[1], "1");
    if (state == BOOT) {
      if ((faultMask & FAULT_CRITICAL) && !any) { reply(fromRadio, "$H,NACK,FPL,CRITICAL"); return; }
      forcedPrelaunch = true;
      markBootDone();
      logEvent("FPL,forced pre-launch from BOOT%s", any ? " (critical faults ignored)" : "");
      reply(fromRadio, "$H,ACK,FPL,%u", (unsigned)state);
    } else if (state == PRELAUNCH) {
      reply(fromRadio, "$H,ACK,FPL,%u", (unsigned)state);
    } else {
      if (altM > FPL_MAX_ALT_M && !any) { reply(fromRadio, "$H,NACK,FPL,ALTITUDE"); return; }
      forcePrelaunch(any ? "ground FPL,1" : "ground FPL", true);
      reply(fromRadio, "$H,ACK,FPL,%u", (unsigned)state);
    }
  } else if (!strcmp(cmd, "FQ")) {
    if (n < 2 || !toLong(tok[1], a) || (a != 0 && a != 1)) { reply(fromRadio, "$H,NACK,FQ,ARGS"); return; }
    reply(fromRadio, "$H,ACK,FQ,%ld", a);
    if (a != curCh || freqTrial) {
      if (!fromRadio) { prevCh = curCh; retune(a, "usb FQ"); }
      else { freqPendingCh = a; freqAskedMs = millis(); }
    }
  } else if (!strcmp(cmd, "TX")) {
    if (n < 2 || !toLong(tok[1], a) || (a != 0 && a != 1)) { reply(fromRadio, "$H,NACK,TX,ARGS"); return; }
    if (a == 1) {
      if (txCutoff) { reply(fromRadio, "$H,NACK,TX,CUTOFF"); return; }   // over the air this cannot be sent: TX is off for good
      txMuted = false;
      txMutePending = false;
      logEvent("LORA,TX_UNMUTED");
      reply(fromRadio, "$H,ACK,TX,1");
    } else {
      reply(fromRadio, "$H,ACK,TX,0");
      txMutePending = true;
      txMuteAskedMs = millis();
    }
  } else if (!strcmp(cmd, "ST")) {
    reply(fromRadio, "$H,ACK,ST,%u,%u,0x%02X,%d,%d,%d,%d,%d,%d,%u,%u", (unsigned)state, stateToken, flags,
          servoDeg[0], servoDeg[1], servoDeg[2],
          servoEst[0] < 0 ? -1 : (int)lroundf(servoEst[0]), servoEst[1] < 0 ? -1 : (int)lroundf(servoEst[1]),
          servoEst[2] < 0 ? -1 : (int)lroundf(servoEst[2]), seqMode, seqStep);
  } else if (!strcmp(cmd, "SV")) {
    if (n < 3 || !servoChArg(n, tok, a) || !toLong(tok[2], b) || b < 0 || b > 90 || b % 10) { reply(fromRadio, "$H,NACK,SV,ARGS"); return; }
    commandServo(a, b, src);
    reply(fromRadio, "$H,ACK,SV,%ld,%ld", a, b);
  } else if (!strcmp(cmd, "PW")) {
    if (n < 3 || !servoChArg(n, tok, a) || !toLong(tok[2], b) || b < SERVO_US_MIN || b > SERVO_US_MAX) { reply(fromRadio, "$H,NACK,PW,ARGS"); return; }
    servoSetUs(a, b, -1, src);
    reply(fromRadio, "$H,ACK,PW,%ld,%ld", a, b);
  } else if (!strcmp(cmd, "LOCK") || !strcmp(cmd, "UNLOCK")) {
    if (!servoChArg(n, tok, a)) { reply(fromRadio, "$H,NACK,%s,ARGS", cmd); return; }
    bool lock = cmd[0] == 'L';
    commandServo(a, lock ? LOCK_DEG[a] : UNLOCK_DEG[a], src);
    reply(fromRadio, "$H,ACK,%s,%ld,%d", cmd, a, lock ? LOCK_DEG[a] : UNLOCK_DEG[a]);
  } else if (!strcmp(cmd, "LOCKALL")) {
    deployActive = false;
    for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) commandServo(ch, LOCK_DEG[ch], "lock all");
    reply(fromRadio, "$H,ACK,LOCKALL");
  } else if (!strncmp(cmd, "TEST", 4) && strlen(cmd) == 5 && cmd[4] >= '1' && cmd[4] <= '3') {
    uint8_t i = cmd[4] - '1';
    seqMode = SEQ_INDIVIDUAL; seqStep = i;
    commandServo(SEQ_ORDER[i], UNLOCK_DEG[SEQ_ORDER[i]], "individual test");
    reply(fromRadio, "$H,ACK,%s,%u,%d", cmd, SEQ_ORDER[i], UNLOCK_DEG[SEQ_ORDER[i]]);
  } else if (!strcmp(cmd, "MSEQ")) {
    seqMode = SEQ_MANUAL; seqStep = 0;
    logEvent("SEQUENCE,MANUAL,ARMED");
    reply(fromRadio, "$H,ACK,MSEQ,0");
  } else if (!strcmp(cmd, "NSTEP")) {
    if (seqMode != SEQ_MANUAL) { reply(fromRadio, "$H,NACK,NSTEP,MODE"); return; }
    if (seqStep >= 3) { reply(fromRadio, "$H,NACK,NSTEP,DONE"); return; }
    uint8_t i = seqStep++;
    commandServo(SEQ_ORDER[i], UNLOCK_DEG[SEQ_ORDER[i]], "manual sequence");
    reply(fromRadio, "$H,ACK,NSTEP,%u,%u", seqStep, SEQ_ORDER[i]);
  } else if (!strcmp(cmd, "FSEQ")) {
    seqMode = SEQ_FULL;
    startDeployment(src, FLAG_BY_MANUAL, DEPLOY_SEP_MANUAL_MS);
    reply(fromRadio, "$H,ACK,FSEQ");
  } else if (!strcmp(cmd, "SEQRST")) {
    deployActive = false; seqMode = SEQ_INDIVIDUAL; seqStep = 0;
    logEvent("SEQUENCE,RESET");
    reply(fromRadio, "$H,ACK,SEQRST");
  } else if (!strcmp(cmd, "SEP")) {
    if (!pcaOk) { reply(fromRadio, "$H,NACK,SEP,PCA"); return; }
    fireSeparation(src, FLAG_BY_MANUAL);
    reply(fromRadio, "$H,ACK,SEP,0x%02X", flags);
  } else if (!strcmp(cmd, "BZ")) {
    if (n < 2 || !toLong(tok[1], a) || (a != 0 && a != 1)) { reply(fromRadio, "$H,NACK,BZ,ARGS"); return; }
    indicatorManual = (a == 1);
    reply(fromRadio, "$H,ACK,BZ,%ld", a);
  } else if (!strcmp(cmd, "GR")) {
    gpsHardReset("commanded");
    reply(fromRadio, "$H,ACK,GR,%lu", (unsigned long)gpsReboots);
  } else if (!strcmp(cmd, "LR")) {
    reply(fromRadio, "$H,ACK,LR");
    loraReinitPending = true; loraReinitAskedMs = millis();
  } else if (!strcmp(cmd, "RF")) {
    if (deployActive) { reply(fromRadio, "$H,NACK,RF,DEPLOYING"); return; }
    reply(fromRadio, "$H,ACK,RF");
    startRefresh();
  } else {
    reply(fromRadio, "$H,NACK,%s,UNKNOWN", cmd);
  }
  savePersist();
}

void handleUsb() {
  static char buf[96];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (len) { buf[len] = '\0'; handleCommand(buf, false, 0, 0); len = 0; } }
    else if (len < sizeof buf - 1) buf[len++] = c;
  }
}

// =====================================================================
//  TELEMETRY
//  $T,2026 IN-SPACe-010,time_s,count,alt_0.1m,press_Pa,temp_0.1C,volt_0.01V,gnss_s_of_hour,lat_1e-4,lon_1e-4,
//     gnss_alt_0.1m,sats,acc_0.1ms2,spin_0.1dps,state,vehicle,vel_ms
//  RECOVERY: same 18 fields; alt, press, temp, volt, sats, acc, spin and vel are sent as 0.
// =====================================================================
int buildTelemetry(char* out, size_t n) {
  bool fix = gpsFix();
  long lat = fix ? lround(gps.location.lat() * 10000.0) : 0;
  long lon = fix ? lround(gps.location.lng() * 10000.0) : 0;
  long galt = (fix && gps.altitude.isValid()) ? lround(gps.altitude.meters() * 10.0) : 0;
  long gsec = gnssSecondsOfHour();
  if (state == RECOVERY)
    return snprintf(out, n, "$T," TEAM_ID ",%lu,%lu,0,0,0,0,%ld,%ld,%ld,%ld,0,0,0,%u,%u,0",
                    (unsigned long)telemTimeS(), (unsigned long)packetCount, gsec, lat, lon, galt,
                    (unsigned)state, (unsigned)VEHICLE_ID);
  unsigned sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
  unsigned long press = pressPa > 0 ? (unsigned long)lroundf(pressPa) : 0;
  unsigned long volt = loadV > 0 ? (unsigned long)lroundf(loadV * 100.0f) : 0;
  return snprintf(out, n, "$T," TEAM_ID ",%lu,%lu,%ld,%lu,%ld,%lu,%ld,%ld,%ld,%ld,%u,%lu,%lu,%u,%u,%lu",
                  (unsigned long)telemTimeS(), (unsigned long)packetCount, lroundf(altM * 10.0f), press,
                  lroundf(airTempC() * 10.0f), volt, gsec, lat, lon, galt, sats,
                  (unsigned long)lroundf(accMag * 10.0f), (unsigned long)lroundf(fabsf(spinDps) * 10.0f),
                  (unsigned)state, (unsigned)VEHICLE_ID, (unsigned long)lroundf(fabsf(velMs)));
}

void serviceTx() {
  if (!txAllowed() || radioMode != RADIO_RX) return;
  uint32_t now = millis();
  if (now - lastTxDoneMs < TX_GUARD_MS) return;
  if (!rqCount && !telemDue) return;
  bool sendReply = rqCount && !(telemDue && lastTxWasReply);    // alternate so neither starves
  uint32_t since = sendReply ? replyDueSince : telemDueSince;
  if (channelBusy() && now - since < MAX_TX_DEFER_MS) return;
  if (sendReply) {
    uint8_t i = rqHead;
    rqHead = (rqHead + 1) % REPLY_Q;
    rqCount--;
    if (rqCount) replyDueSince = now;
    if (radioSend(replyQ[i], replyLen[i])) {
      if (replyQ[i][0] == BITE_MARKER) Serial.println("[TX] binary BITE");
      else Serial.printf("[TX] %.*s\n", replyLen[i], (const char*)replyQ[i]);
    }
    lastTxWasReply = true;
  } else {
    char buf[160];
    telemDue = false;
    packetCount++;
    int len = buildTelemetry(buf, sizeof buf);
    if (radioSend((const uint8_t*)buf, len)) Serial.println(buf);
    else packetCount--;
    lastTxWasReply = false;
  }
}

void serviceRadio() {
  uint32_t now = millis();
  if (!radioOk) { if (now - lastRadioInitMs >= RADIO_RETRY_MS) radioReinit("radio not running"); return; }
  if (radioFlag) {
    radioFlag = false;
    if (radioMode == RADIO_TX) {
      radio.finishTransmit();
      lastTxDoneMs = millis();
      radioStartRx();
    } else if (radioMode == RADIO_RX) {
      uint8_t buf[256];
      size_t len = radio.getPacketLength();
      if (len > 255) len = 255;
      int st = radio.readData(buf, len);
      float rssi = radio.getRSSI(), snr = radio.getSNR();
      radioStartRx();
      if (st == RADIOLIB_ERR_NONE) { buf[len] = '\0'; rxGood++; handleCommand((char*)buf, true, rssi, snr); }
      else if (st == RADIOLIB_ERR_CRC_MISMATCH || st == RADIOLIB_ERR_LORA_HEADER_DAMAGED) rxCrc++;
    }
  }
  if (radioMode == RADIO_TX && millis() - txStartMs > TX_TIMEOUT_MS) { radioReinit("TX timeout"); return; }
  now = millis();
  if (radioMode == RADIO_RX && now - lastGroundRxMs > LORA_REINIT_MS && now - lastRadioInitMs > LORA_REINIT_MS)
    radioReinit(groundHeard ? "no ground packet 60s" : "no link yet (60s)");
}

// =====================================================================
//  SETUP
// =====================================================================
void restoreServos(bool resumed) {
  if (!pcaOk) return;
  for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) {
    if (resumed && servoDeg[ch] >= 0) { servoEst[ch] = servoDeg[ch]; servoSetDeg(ch, servoDeg[ch], "resume"); }
    else if (resumed && servoUs[ch] >= SERVO_US_MIN) servoSetUs(ch, servoUs[ch], -1, "resume");
    else { servoEst[ch] = LOCK_DEG[ch]; servoSetDeg(ch, LOCK_DEG[ch], "boot: lock"); }
  }
  lastServoUpdMs = millis();
}

void setup() {
  resetSrsr = SRC_SRSR;
  SRC_SRSR = resetSrsr;
  bool por = resetSrsr & (SRC_SRSR_IPP_RESET_B | SRC_SRSR_IPP_USER_RESET_B);
  warmBoot = !por && (resetSrsr & (SRC_SRSR_LOCKUP_SYSRESETREQ | SRC_SRSR_WDOG_RST_B | SRC_SRSR_WDOG3_RST_B |
                                   SRC_SRSR_CSU_RESET_B | SRC_SRSR_JTAG_SW_RST | SRC_SRSR_TEMPSENSE_RST_B));
  crashBoot = !por && (resetSrsr & (SRC_SRSR_WDOG_RST_B | SRC_SRSR_WDOG3_RST_B));

  pinMode(PIN_BUZZER, OUTPUT); digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_LORA_CS, OUTPUT); digitalWrite(PIN_LORA_CS, HIGH);
  pinMode(PIN_GNSS_ERST, OUTPUT); digitalWrite(PIN_GNSS_ERST, HIGH);
  pinMode(PIN_GNSS_GPIO, INPUT);
  pinMode(PIN_GNSS_PPS, INPUT);
  pinMode(PIN_ADXL_INT1, INPUT);
  pinMode(PIN_ADXL_INT2, INPUT);
  pinMode(PIN_BNO_INT, INPUT_PULLUP);
  pinMode(PIN_BNO_RST, OUTPUT); digitalWrite(PIN_BNO_RST, HIGH);
  attachInterrupt(digitalPinToInterrupt(PIN_GNSS_PPS), onPps, RISING);
  Serial.begin(115200);
  Serial.println("\n=== ROCKET FC (Teensy 4.1) - 2026 IN-SPACe-010 ===");

  const char* how = "";
  int64_t gap = 0;
  bool resumed = tryResumeRam(how, gap);

  SENS_WIRE.begin(); SENS_WIRE.setClock(I2C_HZ);
  PCA_WIRE.begin();  PCA_WIRE.setClock(I2C_HZ);
  pcaOk = initPca();
  bool servosDone = false;
  if (resumed) {
    restoreServos(true);
    servosDone = true;
  }

  GNSS_SERIAL.begin(9600);
  GNSS_SERIAL.addMemoryForRead(gpsRxBuf, sizeof gpsRxBuf);   // MS5611 reads block ~20 ms per tick
  lastGpsCharMs = lastGpsResetMs = lastFixMs = lastBaudHopMs = millis();
  sdOk = SD.begin(BUILTIN_SDCARD);
  if (!resumed) resumed = tryResumeSd(how, gap);
  if (sdOk && !(resumed && reopenLogs(runIndex))) sdOk = openNewLogs();
  if (!servosDone) restoreServos(resumed);
  if (resumed && deployActive) serviceDeployment(); // a step that fell due during the reboot fires now (logged)

  baroOk = initBaro();
  accOk = initAdxl();
  bnoOk = initBno();
  inaOk = initIna();
  dsOk = initDs();
  if (!dsOk) logEvent("DS18B20,NOT FOUND: check 4.7 k pull-up DQ->3.3 V; MS5611 temperature used");
  SPI.begin();
  radioOk = radioInit();
  lastRadioInitMs = lastGroundRxMs = millis();

  // GNSS configuration with ACK checks, except when resuming in flight (no blocking in the air)
  if (resumed && inFlight(state)) { gpsSetBaud(115200); gpsNeedsCfg = false; logEvent("GPS,resume in flight: no blocking config"); }
  else gnssConfigureBlocking();
  lastGpsCharMs = lastBaudHopMs = millis();

  if (resumed && state >= BOOST) launchConfirmed = true;
  if (state <= PRELAUNCH) {                           // fault mask current at once: launch detection armed
    readSensors();
    runBite(false);
    updateArming();
  }
  if (resumed && state == RECOVERY) { powerDownSensors(); if (pcaOk) pwm.sleep(); indicatorRecovery = true; }
  if (!resumed) indicatorUntil = millis() + BOOT_BEEP_MS;
  updateIndicator();

  Serial.printf("MS5611 %d | ADXL345 %d | BNO08x %d | INA219 %d | DS18B20 %d | PCA9685 %d | SD %d | LoRa %d | srsr 0x%03lX | resumed %d\n",
                baroOk, accOk, bnoOk, inaOk, dsOk, pcaOk, sdOk, radioOk, (unsigned long)resetSrsr, resumed);
  if (!resumed) logEvent("POWER_ON,srsr=0x%03lX,crash=%d", (unsigned long)resetSrsr, crashBoot);
  else logEvent("RESUMED,%s,gap_ms=%lld,state=%s,token=%u,pkt=%lu,ch=%u,srsr=0x%03lX,warm_resets=%lu", how, (long long)gap,
                STATE_NAME[state], stateToken, (unsigned long)packetCount, curCh, (unsigned long)resetSrsr,
                (unsigned long)resetCount);
  savePersist();
  Serial.println("USB commands: PG SD XD CAL[,0|1|2] BITE FPL[,1] FQ,0|1 TX,0|1 ST SV,ch,deg PW,ch,us LOCK,ch "
                 "UNLOCK,ch LOCKALL TEST1-3 MSEQ NSTEP FSEQ SEQRST SEP BZ,0|1 GR LR RF");

  wdt.begin(WDT_TIMEOUT_S);
  lastTickMs = lastTelemMs = lastFlushMs = lastSlowMs = millis();
}

// =====================================================================
//  LOOP
// =====================================================================
void loop() {
  wdt.feed();
  serviceGps();
  serviceBno();
  handleUsb();
  serviceRadio();

  uint32_t now = millis();
  if (state == BOOT && !bootDone) retryPeripherals(true);
  if (now - lastTickMs >= TICK_MS) {
    lastTickMs += TICK_MS;
    if (now - lastTickMs > TICK_MS) lastTickMs = now;
    readSensors();
    updateEstimator();
    updateState();
    serviceDeployment();
    logRow();
    radioApplyPower();
    savePersist();
  }
  if (now - lastSlowMs >= 500) { lastSlowMs = now; readSlowSensors(); }
  updateServos();

  // 90 min after launch detection: transmitter off for good
  if (!txCutoff && tLaunchMs >= 0 && (int64_t)missionMs() - tLaunchMs >= (int64_t)TX_CUTOFF_MS) {
    txCutoff = true;
    telemDue = false;
    rqCount = 0;
    logEvent("LORA,TX_CUTOFF,90 min after launch: transmitter off for good");
    savePersist();
  }
  if (now - lastTelemMs >= TELEM_PERIOD_MS) {
    lastTelemMs = now;
    if (txAllowed() && telemetryOn) { if (!telemDue) { telemDue = true; telemDueSince = now; } }
    else { char b[160]; buildTelemetry(b, sizeof b); Serial.printf("[local] %s\n", b); }
  }
  serviceTx();
  serviceDeferred();
  if (sdOk && now - lastFlushMs >= SD_FLUSH_MS) { lastFlushMs = now; dataFile.flush(); eventFile.flush(); }
  updateIndicator();
}
