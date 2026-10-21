// =====================================================================
//  GROUND STATION  -  Teensy 4.1 + Ra-01H (SX1276)
//  Team 2026 IN-SPACe-010
//
//  Radio (team 010 assignment): main 866.50 MHz, backup 865.25 MHz, SF9, BW 125 kHz, CR 4/5, sync 0x4A,
//  preamble 8, CRC on, +10 dBm. Main channel at every power-on.
//  Wiring (Ra-01H -> Teensy 4.1): NSS 10, DIO0 2, RST 14, SCK 13, MISO 12, MOSI 11, 3.3 V, GND
//
//  ---- GUI -> Teensy (USB serial 115200, one line per button press) ----
//    <id>,<CMD>[,args]     id = any whole number chosen by the GUI, echoed back
//    <CMD>[,args]          also accepted (id 0)
//    CMD may be the long name OR the single-letter code.
//
//    Ground station only (not sent):
//      ENABLE (Y) / LINK   ground LoRa on (it is OFF after power-up)
//      DISABLE (D)         ground LoRa off: radio in reset, queue cleared
//      GS (H)              status line now
//      GF,0 | GF,1         retune the ground station only: 0 = main, 1 = backup
//    Rocket telemetry:  LIVE (S) = SD   HIBERNATE (X) = XD
//    Rocket commands:   PG SD XD BITE ST SEP RF LK GR LR LOCKALL MSEQ NSTEP FSEQ SEQRST TEST1|2|3
//      CAL   (C)       calibrate all          CALA (C1) altitude zero       CALI (C2) IMU (results: OK STATE LAUNCHED
//                      FEWSAMPLES MOVING NOSENSOR VELOCITY AXIS)
//      FSEQ  (F)       full sequence: door, +1 s CanSat, +1.5 s separation
//      FQ,0 | FQ,1     rocket + ground to main / backup channel (confirmed by a ping on the new channel)
//      TX,0 | TX,1     rocket transmitter off / on (its receiver stays on). Refused 90 min after launch.
//      FPL             force PRE-LAUNCH (BOOT: refused with critical faults; later states: refused above 30 m)
//      FPL,1           force PRE-LAUNCH ignoring those checks
//      SV,<ch>,<deg>  PW,<ch>,<us>  LOCK,<ch>  UNLOCK,<ch>  BZ,<0|1>
//
//  ---- Teensy -> GUI (every line starts with $) ----
//    $T,2026 IN-SPACe-010,time_s,count,alt_0.1m,press_Pa,temp_0.1C,volt_0.01V,gnss_s_of_hour,lat_1e-4,lon_1e-4,
//       gnss_alt_0.1m,sats,acc_0.1ms2,spin_0.1dps,state,vehicle,vel_ms         rocket telemetry, as received
//    $H,...                         rocket replies (ACK / NACK)
//      $H,ACK,CALA,<n>,<p0_Pa>,<T0_0.1C>,<gnss_ground_0.1m>
//      $H,ACK,CALI,<n>,<acc_axis>,<sign>,<bias_mm/s2>,<sin_elev_1e-4>,<spin_axis>,<spin_sign>,<bno_ok>,<bno_tilt_0.1deg>
//      $H,ACK,CAL,<n>,<alt result>,<imu result>      results: OK STATE LAUNCHED FEWSAMPLES MOVING NOSENSOR VELOCITY
//      $H,NACK,<cmd>,<reason>          e.g. $H,NACK,RF,DEPLOYING (refresh refused while deploying)
//    $B,<state>,<status>,<mask>,<letters>,<vrail_V>,<sats>,<token>      decoded binary BITE frame
//    $G,CMD,<id>,<status>[,<info>]  QUEUED | SENT,<try> | OK | REJECTED,<reason> | FAILED,no_reply | BADCMD |
//                                   BUSY,queue_full | REJECTED,lora_disabled | FAILED,lora_disabled
//    $G,FREQ,<ch>,<MHz>,<what>      what: SWITCHING | CONFIRMED | FAILED | MANUAL | SCAN | FOUND
//    $G,LINK,<rssi>,<snr>,<good>,<crc_fail>,<lost>       after every received packet
//    $G,CRC,<count>,<rssi>          $G,FOREIGN,<team>       $G,RADIO,...       $G,LORA,ENABLED|DISABLED
//    $G,STATUS,telem,status,crc,lost,rssi,snr,last_rx_ms,reinit,uplinks,link,radio,lora_enabled,channel
//
//  Uplink over the air = single-letter code + args: P=PG K=LK S=SD X=XD C=CAL B=BITE T=ST V=SV W=PW L=LOCK
//    U=UNLOCK A=LOCKALL E=TEST M=MSEQ N=NSTEP F=FSEQ Q=SEQRST J=SEP Z=BZ G=GR O=LR R=RF I=FPL;
//    CALA -> "C1", CALI -> "C2"; FQ and TX go as written ("FQ,1", "TX,0").
//  Channel rules: after FQ the rocket answers on the old channel, both retune, the ground pings (PG) on the new
//    one; no answer within 10 s -> both go back. Telemetry quiet for 5 s -> the ground hops main/backup every 3 s
//    until it hears the rocket again.
//  SD: /IN-SPACe-010_GS_TELEMETRY_NNN.csv (decoded $T) and /IN-SPACe-010_GS_EVENTS_NNN.csv (everything else)
//  Library: RadioLib (>= 7.0)
// =====================================================================

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <stdarg.h>
#include <RadioLib.h>

// ----------------------------- PINS ---------------------------------
#define LORA_CS    10
#define LORA_DIO0  2
#define LORA_RST   14

// ---------------------------- RADIO ---------------------------------
#define CH_MAIN_MHZ    866.50
#define CH_BACKUP_MHZ  865.25
#define LORA_BW        125.0
#define LORA_SF        9
#define LORA_CR        5
#define LORA_SYNC      0x4A
#define LORA_POWER     10
#define LORA_PREAMBLE  8
#define TEAM_ID        "2026 IN-SPACe-010"

// ---------------------------- TIMING --------------------------------
#define CMD_MAX_TRIES      8
#define CMD_RETRY_MS       2000
#define TELEM_FLOWING_MS   2500
#define CMD_AFTER_T_MS     20      // rocket keeps a 300 ms listen gap after each TX
#define LORA_REINIT_MS     60000
#define RADIO_RETRY_MS     5000
#define TX_TIMEOUT_MS      3000
#define STATUS_PERIOD_MS   5000
#define SD_FLUSH_MS        1000
#define FREQ_TRIAL_MS      10000   // new channel must answer the ping within 10 s
#define SCAN_QUIET_MS      5000    // telemetry quiet this long -> scan
#define SCAN_HOP_MS        3000    // hop main <-> backup every 3 s while scanning

#define FILE_PREFIX        "IN-SPACe-010_GS"
const char* STATE_NAME[] = {"BOOT", "PRELAUNCH", "BOOST", "COAST", "APOGEE", "DESCENT", "LANDING", "RECOVERY"};

// =====================================================================
//  GLOBALS
// =====================================================================
Module loraMod(LORA_CS, LORA_DIO0, LORA_RST, RADIOLIB_NC);
SX1276 radio(&loraMod);

volatile bool radioFlag = false;
enum RadioMode { RADIO_RX, RADIO_TX } radioMode = RADIO_RX;
bool radioOk = false;
bool loraEnabled = false;
bool isrAttached = false;
uint32_t txStartMs = 0, lastRocketRxMs = 0, lastTelemMs = 0, lastReinitMs = 0;
bool telemJustArrived = false, telemetryWanted = false, linkActive = false, telemWantedBeforeMute = false;

uint32_t rxTelem = 0, rxStatus = 0, rxCrc = 0, lost = 0, reinits = 0, uplinks = 0, foreign = 0;
long lastPkt = -1;
float lastRssi = 0, lastSnr = 0;

// channels
uint8_t curCh = 0, prevCh = 0;
bool freqTrial = false, scanning = false;
uint32_t freqTrialMs = 0, lastScanHopMs = 0;

struct Cmd { long id; char text[40]; char name[8]; };
#define CMD_Q 8
#define ID_FREQ_PING (-2)              // internal ping on a new channel (silent toward the GUI)
Cmd cmdQ[CMD_Q];
uint8_t cqHead = 0, cqCount = 0;
bool cmdActive = false;
uint8_t cmdTries = 0;
uint32_t lastCmdTxMs = 0;

bool sdOk = false;
File telemFile, eventFile;
uint32_t lastFlushMs = 0, lastStatusMs = 0;

#define LOCAL_ENABLE  'Y'
#define LOCAL_DISABLE 'D'
#define LOCAL_STATUS  'H'

// =====================================================================
//  OUTPUT + LOG
// =====================================================================
void logLine(const char* line, bool radioLine, float rssi, float snr) {
  if (!sdOk) return;
  uint32_t t = millis();
  if (radioLine)
    eventFile.printf("%lu,%lu.%03lu,%u,%.0f,%.2f,%s\n", (unsigned long)t, (unsigned long)(t / 1000),
                     (unsigned long)(t % 1000), curCh, rssi, snr, line);
  else
    eventFile.printf("%lu,%lu.%03lu,%u,,,%s\n", (unsigned long)t, (unsigned long)(t / 1000), (unsigned long)(t % 1000),
                     curCh, line);
}

void out(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void out(const char* fmt, ...) {
  char b[220];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  Serial.println(b);
  logLine(b, false, 0, 0);
}

void cmdStatus(const Cmd& c, const char* status, const char* info = nullptr) {
  if (c.id < 0) return;                                   // internal commands: silent
  if (info) out("$G,CMD,%ld,%s,%s", c.id, status, info);
  else out("$G,CMD,%ld,%s", c.id, status);
}

float chFreq(uint8_t ch) { return ch ? CH_BACKUP_MHZ : CH_MAIN_MHZ; }

void printStatus() {
  long since = lastRocketRxMs ? (long)(millis() - lastRocketRxMs) : -1;
  out("$G,STATUS,%lu,%lu,%lu,%lu,%.0f,%.1f,%ld,%lu,%lu,%d,%d,%d,%u", (unsigned long)rxTelem, (unsigned long)rxStatus,
      (unsigned long)rxCrc, (unsigned long)lost, lastRssi, lastSnr, since, (unsigned long)reinits,
      (unsigned long)uplinks, linkActive ? 1 : 0, radioOk ? 1 : 0, loraEnabled ? 1 : 0, curCh);
}

// =====================================================================
//  RADIO
// =====================================================================
void onDio0() { radioFlag = true; }

void radioStartRx() {
  radioFlag = false;
  int st = radio.startReceive();
  radioMode = RADIO_RX;
  if (st != RADIOLIB_ERR_NONE) out("$G,RADIO,ERR,startReceive %d", st);
}

bool radioStep(const char* what, int st) {
  if (st == RADIOLIB_ERR_NONE) return true;
  out("$G,RADIO,ERR,%s,%d", what, st);
  return false;
}

bool radioConfigure() {
  if (!radioStep("begin", radio.begin())) return false;
  if (!radioStep("standby", radio.standby())) return false;
  if (!radioStep("frequency", radio.setFrequency(chFreq(curCh)))) return false;
  if (!radioStep("bandwidth", radio.setBandwidth(LORA_BW))) return false;
  if (!radioStep("sf", radio.setSpreadingFactor(LORA_SF))) return false;
  if (!radioStep("cr", radio.setCodingRate(LORA_CR))) return false;
  if (!radioStep("sync", radio.setSyncWord(LORA_SYNC))) return false;
  if (!radioStep("power", radio.setOutputPower(LORA_POWER))) return false;
  if (!radioStep("preamble", radio.setPreambleLength(LORA_PREAMBLE))) return false;
  if (!radioStep("crc", radio.setCRC(true))) return false;
  return true;
}

// expected: 866.50 MHz FRF=D8A000, 865.25 MHz FRF=D85000, MC1=72 (BW125, 4/5), MC2=94 (SF9, CRC on), SYNC=4A
void radioDump() {
  Module* m = &loraMod;
  uint32_t frf = ((uint32_t)m->SPIreadRegister(0x06) << 16) | ((uint32_t)m->SPIreadRegister(0x07) << 8) |
                 m->SPIreadRegister(0x08);
  out("$G,RADIO,CFG,ch=%u,FRF=%06lX (%.3f MHz),MC1=%02X,MC2=%02X,SYNC=%02X,PA=%02X", curCh, (unsigned long)frf,
      frf * 32.0 / 524288.0, m->SPIreadRegister(0x1D), m->SPIreadRegister(0x1E) & 0xF4,
      m->SPIreadRegister(0x39), m->SPIreadRegister(0x09));
}

bool radioInit() {
  if (isrAttached) { radio.clearDio0Action(); isrAttached = false; }
  bool ok = radioConfigure();
  if (!ok) {
    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, LOW); delay(2); digitalWrite(LORA_RST, HIGH); delay(20);
    ok = radioConfigure();
  }
  if (!ok) return false;
  radioDump();
  radio.setDio0Action(onDio0, RISING);
  isrAttached = true;
  radioStartRx();
  return true;
}

void radioReinit(const char* why) {
  reinits++;
  lastReinitMs = millis();
  radioOk = radioInit();
  out("$G,RADIO,RESET,%s,%s", why, radioOk ? "OK" : "FAIL");
}

// retune between transmissions only (callers check radioMode)
bool retune(uint8_t ch) {
  curCh = ch;
  if (!radioOk) return false;
  radio.standby();
  int st = radio.setFrequency(chFreq(ch));
  radioStartRx();
  if (st != RADIOLIB_ERR_NONE) { radioReinit("retune failed"); return false; }
  return true;
}

// =====================================================================
//  COMMANDS FROM THE GUI
// =====================================================================
bool num(const char* s, long& v) {
  if (!s || !*s) return false;
  char* e;
  v = strtol(s, &e, 10);
  return *e == '\0';
}

// validates and rewrites the command in a canonical form ("sv,0,030" -> "SV,0,30"); name = reply name to match
bool canonical(char* text, char* name) {
  char copy[40];
  strncpy(copy, text, sizeof copy - 1); copy[sizeof copy - 1] = '\0';
  char* t[4] = {nullptr, nullptr, nullptr, nullptr};
  int n = 0;
  char* save;
  for (char* p = strtok_r(copy, ",", &save); p && n < 4; p = strtok_r(nullptr, ",", &save)) t[n++] = p;
  if (!n) return false;
  long a, b;
  if (!strcmp(t[0], "TEST")) {
    if (n != 2 || !num(t[1], a) || a < 1 || a > 3) return false;
    snprintf(text, 40, "TEST%ld", a);
    snprintf(name, 8, "TEST%ld", a);
    return true;
  }
  strncpy(name, t[0], 7); name[7] = '\0';
  if (!strcmp(t[0], "CALA") || !strcmp(t[0], "CALI")) {
    if (n != 1) return false;
    strcpy(text, t[0][3] == 'A' ? "CAL,1" : "CAL,2");
    return true;
  }
  if (!strcmp(t[0], "CAL") && n == 2) {
    if (!num(t[1], a) || a < 0 || a > 2) return false;
    if (a == 0) { strcpy(text, "CAL"); strcpy(name, "CAL"); }
    else { snprintf(text, 40, "CAL,%ld", a); strcpy(name, a == 1 ? "CALA" : "CALI"); }
    return true;
  }
  if (!strcmp(t[0], "FPL") && n == 2) {
    if (!num(t[1], a) || a != 1) return false;
    strcpy(text, "FPL,1"); return true;
  }
  static const char* simple[] = {"PG", "SD", "XD", "CAL", "BITE", "ST", "SEP", "RF", "LK", "GR", "LR",
                                 "LOCKALL", "TEST1", "TEST2", "TEST3", "MSEQ", "NSTEP", "FSEQ", "SEQRST", "FPL"};
  for (const char* s : simple) if (!strcmp(t[0], s)) { if (n != 1) return false; strcpy(text, s); return true; }
  if (!strcmp(t[0], "BZ") || !strcmp(t[0], "FQ") || !strcmp(t[0], "TX")) {
    if (n != 2 || !num(t[1], a) || (a != 0 && a != 1)) return false;
    snprintf(text, 40, "%s,%ld", t[0], a); return true;
  }
  if (!strcmp(t[0], "LOCK") || !strcmp(t[0], "UNLOCK")) {
    if (n != 2 || !num(t[1], a) || a < 0 || a > 2) return false;
    snprintf(text, 40, "%s,%ld", t[0], a); return true;
  }
  if (!strcmp(t[0], "SV")) {
    if (n != 3 || !num(t[1], a) || !num(t[2], b) || a < 0 || a > 2 || b < 0 || b > 90 || b % 10) return false;
    snprintf(text, 40, "SV,%ld,%ld", a, b); return true;
  }
  if (!strcmp(t[0], "PW")) {
    if (n != 3 || !num(t[1], a) || !num(t[2], b) || a < 0 || a > 2 || b < 500 || b > 2500) return false;
    snprintf(text, 40, "PW,%ld,%ld", a, b); return true;
  }
  return false;
}

struct Code { char c; const char* name; };
const Code CODES[] = {
  {'P', "PG"},     {'K', "LK"},     {'S', "SD"},     {'X', "XD"},     {'C', "CAL"},   {'B', "BITE"},
  {'T', "ST"},     {'V', "SV"},     {'W', "PW"},     {'L', "LOCK"},   {'U', "UNLOCK"}, {'A', "LOCKALL"},
  {'E', "TEST"},   {'M', "MSEQ"},   {'N', "NSTEP"},  {'F', "FSEQ"},   {'Q', "SEQRST"}, {'J', "SEP"},
  {'Z', "BZ"},     {'G', "GR"},     {'O', "LR"},     {'R', "RF"},     {'I', "FPL"},
};

// canonical long text -> code, in place: "SV,2,90" -> "V2,90", "TEST1" -> "E1", "CAL,1" -> "C1".
// Commands without a code (FQ, TX) stay as written.
void toCode(char* text) {
  char o[40];
  char* comma = strchr(text, ',');
  const char* args = comma ? comma + 1 : "";
  size_t nl = comma ? (size_t)(comma - text) : strlen(text);
  if (nl == 5 && !strncmp(text, "TEST", 4)) { snprintf(o, sizeof o, "E%c", text[4]); strcpy(text, o); return; }
  for (const Code& k : CODES) {
    if (strlen(k.name) == nl && !strncmp(text, k.name, nl)) {
      snprintf(o, sizeof o, "%c%s", k.c, args);
      strcpy(text, o);
      return;
    }
  }
}

// rocket code from the GUI -> long canonical text ("V2,90" -> "SV,2,90"); false = not a code
bool expandCode(const char* s, char* o, size_t n) {
  if (!(s[1] == '\0' || isdigit((unsigned char)s[1]) || s[1] == '-')) return false;
  for (const Code& k : CODES) {
    if (k.c != s[0]) continue;
    if (!strcmp(k.name, "TEST")) snprintf(o, n, "TEST%s", s + 1);
    else if (s[1]) snprintf(o, n, "%s,%s", k.name, s + 1);
    else snprintf(o, n, "%s", k.name);
    return true;
  }
  return false;
}

bool enqueue(long id, const char* text, const char* name) {
  if (cqCount >= CMD_Q) return false;
  Cmd& c = cmdQ[(cqHead + cqCount) % CMD_Q];
  c.id = id;
  strncpy(c.text, text, sizeof c.text - 1); c.text[sizeof c.text - 1] = '\0';
  strncpy(c.name, name, sizeof c.name - 1); c.name[sizeof c.name - 1] = '\0';
  cqCount++;
  return true;
}

void loraEnable() {
  if (loraEnabled && radioOk) { out("$G,LORA,ENABLED"); return; }
  loraEnabled = true;
  radioReinit("enabled");
  linkActive = true;
  lastRocketRxMs = millis();
  lastTelemMs = 0;
  out("$G,LORA,ENABLED");
  out("$G,FREQ,%u,%.2f,MANUAL", curCh, chFreq(curCh));
}

void loraDisable() {
  while (cqCount) {
    cmdStatus(cmdQ[cqHead], "FAILED", "lora_disabled");
    cqHead = (cqHead + 1) % CMD_Q;
    cqCount--;
  }
  cmdActive = false;
  cmdTries = 0;
  if (isrAttached) { radio.clearDio0Action(); isrAttached = false; }
  digitalWrite(LORA_RST, LOW);
  delay(1);
  digitalWrite(LORA_RST, HIGH);
  radioOk = false;
  radioFlag = false;
  radioMode = RADIO_RX;
  loraEnabled = false;
  linkActive = false;
  telemetryWanted = false;
  freqTrial = scanning = false;
  lastTelemMs = 0;
  out("$G,LORA,DISABLED");
}

void handleLine(char* s) {
  while (*s == ' ') s++;
  char* e = s + strlen(s);
  while (e > s && (e[-1] == ' ' || e[-1] == '\r' || e[-1] == '\n')) *--e = '\0';
  for (char* p = s; *p; p++) *p = toupper((unsigned char)*p);
  if (!strncmp(s, "$C,", 3)) s += 3;
  if (!*s) return;

  Cmd c; c.id = 0;
  if (isdigit((unsigned char)s[0])) {
    char* comma = strchr(s, ',');
    if (!comma) { out("$G,CMD,0,BADCMD"); return; }
    c.id = atol(s);
    s = comma + 1;
  }
  // ---- local commands ----
  bool one = s[1] == '\0';
  if ((one && s[0] == LOCAL_STATUS) || !strcmp(s, "GS")) { printStatus(); cmdStatus(c, "OK"); return; }
  if ((one && s[0] == LOCAL_ENABLE) || !strcmp(s, "ENABLE") || !strcmp(s, "LINK")) {
    loraEnable(); cmdStatus(c, radioOk ? "OK" : "FAILED", radioOk ? nullptr : "radio_init"); return;
  }
  if ((one && s[0] == LOCAL_DISABLE) || !strcmp(s, "DISABLE")) { loraDisable(); cmdStatus(c, "OK"); return; }
  if (!strncmp(s, "GF,", 3)) {                         // ground-only retune
    long ch;
    if (!num(s + 3, ch) || (ch != 0 && ch != 1)) { cmdStatus(c, "BADCMD"); return; }
    if (radioMode == RADIO_TX) { cmdStatus(c, "BUSY", "transmitting"); return; }
    freqTrial = scanning = false;
    retune(ch);
    out("$G,FREQ,%u,%.2f,MANUAL", curCh, chFreq(curCh));
    cmdStatus(c, "OK");
    return;
  }

  // ---- rocket commands ----
  if (!strcmp(s, "LIVE")) s = (char*)"SD";
  else if (!strcmp(s, "HIBERNATE")) s = (char*)"XD";
  char expanded[40];
  if (expandCode(s, expanded, sizeof expanded)) s = expanded;

  if (!loraEnabled) { cmdStatus(c, "REJECTED", "lora_disabled"); return; }
  strncpy(c.text, s, sizeof c.text - 1); c.text[sizeof c.text - 1] = '\0';
  if (!canonical(c.text, c.name)) { cmdStatus(c, "BADCMD"); return; }
  toCode(c.text);
  if (!enqueue(c.id, c.text, c.name)) { cmdStatus(c, "BUSY", "queue_full"); return; }
  linkActive = true;
  cmdStatus(c, "QUEUED");
}

void handleUsb() {
  static char buf[64];
  static uint8_t len = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { if (len) { buf[len] = '\0'; handleLine(buf); len = 0; } }
    else if (len < sizeof buf - 1) buf[len++] = ch;
  }
}

// =====================================================================
//  UPLINK (one command at a time, FIFO, retried)
// =====================================================================
void freqFailed(const char* why);
void finishCmd(const char* status, const char* info = nullptr) {
  Cmd& c = cmdQ[cqHead];
  if (c.id == ID_FREQ_PING && freqTrial) {
    if (!strcmp(status, "OK")) { freqTrial = false; out("$G,FREQ,%u,%.2f,CONFIRMED", curCh, chFreq(curCh)); }
    else freqFailed("no answer to ping");
  }
  cmdStatus(c, status, info);
  cqHead = (cqHead + 1) % CMD_Q;
  cqCount--;
  cmdActive = false;
  cmdTries = 0;
}

void serviceUplink() {
  if (!loraEnabled) return;
  uint32_t now = millis();
  if (!cqCount || !radioOk || radioMode != RADIO_RX) return;
  if (!cmdActive) { cmdActive = true; cmdTries = 0; lastCmdTxMs = 0; telemJustArrived = false; }

  bool flowing = lastTelemMs && now - lastTelemMs < TELEM_FLOWING_MS;
  if (flowing) {
    if (!telemJustArrived || now - lastTelemMs < CMD_AFTER_T_MS) return;   // send in the gap after a $T
    telemJustArrived = false;
  } else if (cmdTries > 0 && now - lastCmdTxMs < CMD_RETRY_MS) {
    return;
  }
  if (cmdTries >= CMD_MAX_TRIES) {
    finishCmd("FAILED", "no_reply");
    if (now - lastReinitMs > LORA_REINIT_MS) radioReinit("no reply");
    return;
  }
  char msg[48];
  snprintf(msg, sizeof msg, "%s", cmdQ[cqHead].text);
  radioFlag = false;
  int st = radio.startTransmit((const uint8_t*)msg, strlen(msg));
  if (st != RADIOLIB_ERR_NONE) { out("$G,RADIO,ERR,startTransmit %d", st); radioReinit("tx error"); return; }
  radioMode = RADIO_TX;
  txStartMs = lastCmdTxMs = now;
  cmdTries++;
  uplinks++;
  char t[8];
  snprintf(t, sizeof t, "%u", cmdTries);
  cmdStatus(cmdQ[cqHead], "SENT", t);
  if (sdOk) {
    char up[64];
    snprintf(up, sizeof up, "UP %s (%s)", msg, cmdQ[cqHead].name);
    logLine(up, false, 0, 0);
  }
}

// =====================================================================
//  CHANNEL SWITCH / SCAN
// =====================================================================
void freqStartTrial(uint8_t ch) {
  prevCh = curCh;
  scanning = false;
  retune(ch);
  freqTrial = true;
  freqTrialMs = millis();
  out("$G,FREQ,%u,%.2f,SWITCHING", curCh, chFreq(curCh));
  // ping on the new channel: put it at the front of the queue so it goes first
  if (cqCount < CMD_Q) {
    cqHead = (cqHead + CMD_Q - 1) % CMD_Q;
    Cmd& c = cmdQ[cqHead];
    c.id = ID_FREQ_PING;
    strcpy(c.text, "P");
    strcpy(c.name, "PG");
    cqCount++;
    cmdActive = false;
    cmdTries = 0;
  }
}
void freqFailed(const char* why) {
  uint8_t tried = curCh;
  freqTrial = false;
  if (radioMode != RADIO_TX) retune(prevCh); else curCh = prevCh;
  out("$G,FREQ,%u,%.2f,FAILED,%s,back on %u", tried, chFreq(tried), why, curCh);
}
void serviceChannels() {
  if (!loraEnabled || !radioOk || radioMode != RADIO_RX) return;
  uint32_t now = millis();
  if (freqTrial) {
    if (now - freqTrialMs > FREQ_TRIAL_MS) freqFailed("10 s");
    return;
  }
  // telemetry quiet for 5 s: hop main <-> backup every 3 s until the rocket is heard
  if (telemetryWanted && now - lastRocketRxMs > SCAN_QUIET_MS && now - lastScanHopMs >= SCAN_HOP_MS) {
    lastScanHopMs = now;
    scanning = true;
    retune(curCh ^ 1);
    out("$G,FREQ,%u,%.2f,SCAN", curCh, chFreq(curCh));
  }
}
void heardRocket() {
  lastRocketRxMs = millis();
  if (scanning) { scanning = false; out("$G,FREQ,%u,%.2f,FOUND", curCh, chFreq(curCh)); }
}

// =====================================================================
//  INCOMING PACKETS
// =====================================================================
bool field(const char* s, int n, char* o, size_t len) {
  for (int i = 0; i < n; i++) { s = strchr(s, ','); if (!s) return false; s++; }
  size_t k = 0;
  while (*s && *s != ',' && k < len - 1) o[k++] = *s++;
  o[k] = '\0';
  return true;
}

// $T,team,time_s,count,alt,press,temp,volt,gnss_s,lat,lon,gnss_alt,sats,acc,spin,state,vehicle,vel
#define T_FIELDS 18
void logTelemetry(const char* msg, float rssi, float snr, long lostHere) {
  if (!sdOk) return;
  char copy[220];
  strncpy(copy, msg, sizeof copy - 1); copy[sizeof copy - 1] = '\0';
  char* f[T_FIELDS + 2];
  int n = 0;
  char* save;
  for (char* p = strtok_r(copy, ",", &save); p && n < T_FIELDS + 1; p = strtok_r(nullptr, ",", &save)) f[n++] = p;
  uint32_t t = millis();
  if (n != T_FIELDS) {
    telemFile.printf("%lu,%lu.%03lu,%u,%.0f,%.2f,MALFORMED,%s\n", (unsigned long)t, (unsigned long)(t / 1000),
                     (unsigned long)(t % 1000), curCh, rssi, snr, msg);
    return;
  }
  long st = atol(f[15]);
  telemFile.printf(
    "%lu,%lu.%03lu,%u,%.0f,%.2f,%ld,%s,%ld,%ld,"
    "%.1f,%ld,%.2f,%.1f,%.2f,%ld,%.4f,%.4f,%.1f,%ld,%.1f,%.1f,"
    "%ld,%s,%ld,%ld\n",
    (unsigned long)t, (unsigned long)(t / 1000), (unsigned long)(t % 1000), curCh, rssi, snr, lostHere, f[1],
    atol(f[2]), atol(f[3]),
    atol(f[4]) / 10.0, atol(f[5]), atol(f[5]) / 100.0, atol(f[6]) / 10.0, atol(f[7]) / 100.0, atol(f[8]),
    atol(f[9]) / 10000.0, atol(f[10]) / 10000.0, atol(f[11]) / 10.0, atol(f[12]), atol(f[13]) / 10.0, atol(f[14]) / 10.0,
    st, (st >= 0 && st <= 7) ? STATE_NAME[st] : "?", atol(f[16]), atol(f[17]));
}

const char FAULT_LETTER[] = "BPARSGDIONFULKVCTMWXEH";
#define BITE_MARKER 0xB1
void linkLine(float rssi, float snr) {
  out("$G,LINK,%.0f,%.1f,%lu,%lu,%lu", rssi, snr, (unsigned long)(rxTelem + rxStatus), (unsigned long)rxCrc,
      (unsigned long)lost);
}
void handleBite(const uint8_t* b, size_t n, float rssi, float snr) {
  heardRocket();
  lastRssi = rssi; lastSnr = snr;
  rxStatus++;
  if (n < 12) { out("$G,BITE,SHORT,%u", (unsigned)n); return; }
  uint32_t mask = b[3] | ((uint32_t)b[4] << 8) | ((uint32_t)b[5] << 16) | ((uint32_t)b[6] << 24);
  uint16_t cv = b[7] | (b[8] << 8);
  uint16_t token = b[10] | (b[11] << 8);
  char letters[24];
  size_t k = 0;
  for (int i = 0; i < 22 && k < sizeof letters - 1; i++) if (mask & (1u << i)) letters[k++] = FAULT_LETTER[i];
  letters[k] = '\0';
  out("$B,%u,%u,0x%06lX,%s,%.2f,%u,%u", b[1], b[2], (unsigned long)mask, k ? letters : "-", cv / 100.0, b[9], token);
  if (cqCount && cmdActive && cmdTries > 0 && !strcmp(cmdQ[cqHead].name, "BITE")) finishCmd("OK");
  linkLine(rssi, snr);
}

void handlePacket(char* msg, float rssi, float snr) {
  // telemetry from another team on the same channel: shown, not counted as ours
  if (!strncmp(msg, "$T,", 3)) {
    char team[24];
    if (field(msg, 1, team, sizeof team) && strcmp(team, TEAM_ID)) {
      foreign++;
      out("$G,FOREIGN,%s,%.0f", team, rssi);
      return;
    }
  }
  heardRocket();
  lastRssi = rssi; lastSnr = snr;
  Serial.println(msg);
  logLine(msg, true, rssi, snr);

  if (!strncmp(msg, "$T,", 3)) {
    rxTelem++;
    char f[16];
    long lostHere = 0;
    if (field(msg, 3, f, sizeof f)) {                       // field 3 = packet count
      long pkt = atol(f);
      if (lastPkt >= 0 && pkt > lastPkt + 1) { lostHere = pkt - lastPkt - 1; lost += lostHere; }
      lastPkt = pkt;
    }
    logTelemetry(msg, rssi, snr, lostHere);
    lastTelemMs = millis();
    telemJustArrived = true;
    telemetryWanted = true;
    if (cqCount && cmdActive && cmdTries > 0 && !strcmp(cmdQ[cqHead].name, "LK")) finishCmd("OK");
  } else if (!strncmp(msg, "$H,", 3)) {
    rxStatus++;
    char kind[8], name[8], info[24];
    if (cqCount && cmdActive && field(msg, 1, kind, sizeof kind) && field(msg, 2, name, sizeof name) &&
        (!strcmp(kind, "ACK") || !strcmp(kind, "NACK")) && !strcmp(name, cmdQ[cqHead].name)) {
      bool ack = kind[0] == 'A';
      const char* nm = cmdQ[cqHead].name;
      char arg[8] = "";
      field(msg, 3, arg, sizeof arg);
      if (ack && !strcmp(nm, "SD")) { telemetryWanted = true; lastTelemMs = 0; }
      if (ack && !strcmp(nm, "XD")) telemetryWanted = false;
      if (ack && (!strcmp(nm, "RF") || !strcmp(nm, "LR"))) lastTelemMs = 0;
      if (ack && !strcmp(nm, "TX")) {
        if (arg[0] == '0') { telemWantedBeforeMute = telemetryWanted; telemetryWanted = false; lastTelemMs = 0; }
        else telemetryWanted = telemWantedBeforeMute;
      }
      bool startTrial = ack && !strcmp(nm, "FQ") && (arg[0] == '0' || arg[0] == '1') && (uint8_t)(arg[0] - '0') != curCh;
      uint8_t trialCh = arg[0] - '0';
      if (!ack && field(msg, 3, info, sizeof info)) finishCmd("REJECTED", info);
      else finishCmd(ack ? "OK" : "REJECTED");
      if (startTrial) freqStartTrial(trialCh);
    }
  }
  linkLine(rssi, snr);
}

void serviceRadio() {
  if (!loraEnabled) return;
  uint32_t now = millis();
  if (!radioOk) { if (now - lastReinitMs >= RADIO_RETRY_MS) radioReinit("radio not running"); return; }

  if (radioFlag) {
    radioFlag = false;
    if (radioMode == RADIO_TX) {
      radio.finishTransmit();
      radioStartRx();
    } else {
      uint8_t buf[256];
      size_t len = radio.getPacketLength();
      if (len > 255) len = 255;
      int st = radio.readData(buf, len);
      float rssi = radio.getRSSI(), snr = radio.getSNR();
      radioStartRx();
      if (st == RADIOLIB_ERR_NONE) {
        if (len && buf[0] == BITE_MARKER) handleBite(buf, len, rssi, snr);
        else {
          buf[len] = '\0';
          handlePacket((char*)buf, rssi, snr);
        }
      } else if (st == RADIOLIB_ERR_CRC_MISMATCH || st == RADIOLIB_ERR_LORA_HEADER_DAMAGED) {
        rxCrc++;
        out("$G,CRC,%lu,%.0f", (unsigned long)rxCrc, rssi);
      }
    }
  }
  if (radioMode == RADIO_TX && millis() - txStartMs > TX_TIMEOUT_MS) { radioReinit("tx timeout"); return; }

  now = millis();
  if (telemetryWanted && radioMode == RADIO_RX && now - lastRocketRxMs > LORA_REINIT_MS &&
      now - lastReinitMs > LORA_REINIT_MS)
    radioReinit("nothing heard 60s");
}

// =====================================================================
//  SD (built-in card)
// =====================================================================
int newestLogIndex() {
  int best = -1;
  File root = SD.open("/");
  if (!root) return -1;
  const char* pre = FILE_PREFIX "_TELEMETRY_";
  size_t pl = strlen(pre);
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    const char* n = f.name();
    if (!strncmp(n, pre, pl) && strlen(n) == pl + 7 && !strcmp(n + pl + 3, ".csv")) best = max(best, atoi(n + pl));
    f.close();
  }
  root.close();
  return best;
}

void openLog() {
  if (!SD.begin(BUILTIN_SDCARD)) return;
  int k = newestLogIndex() + 1;
  char tn[48], en[48];
  snprintf(tn, sizeof tn, "/" FILE_PREFIX "_TELEMETRY_%03d.csv", k);
  snprintf(en, sizeof en, "/" FILE_PREFIX "_EVENTS_%03d.csv", k);
  telemFile = SD.open(tn, FILE_WRITE);
  eventFile = SD.open(en, FILE_WRITE);
  if (!telemFile || !eventFile) return;
  telemFile.println("gs_ms,gs_time_s,channel,rssi_dbm,snr_db,packets_lost_before,team_id,"
                    "mission_time_s,packet_count,altitude_m,pressure_pa,pressure_hpa,temperature_c,voltage_v,"
                    "gnss_time_s,latitude_deg,longitude_deg,gnss_altitude_m,satellites,acceleration_ms2,spin_rate_dps,"
                    "state,state_name,vehicle_id,velocity_ms");
  eventFile.println("gs_ms,gs_time_s,channel,rssi_dbm,snr_db,line");
  telemFile.flush(); eventFile.flush();
  sdOk = true;
  out("$G,SD,%s,%s", tn, en);
}

// =====================================================================
//  SETUP / LOOP
// =====================================================================
void setup() {
  Serial.begin(115200);
  openLog();
  if (!sdOk) Serial.println("$G,SD,NONE");
  SPI.begin();
  pinMode(LORA_CS, OUTPUT); digitalWrite(LORA_CS, HIGH);
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, LOW);
  delay(1);
  digitalWrite(LORA_RST, HIGH);
  radioOk = false;
  curCh = 0;                           // main channel at every power-on
  lastReinitMs = millis();
#ifdef RADIOLIB_VERSION_MAJOR
  out("$G,RADIOLIB,%d.%d.%d", RADIOLIB_VERSION_MAJOR, RADIOLIB_VERSION_MINOR, RADIOLIB_VERSION_PATCH);
#endif
  out("$G,READY,LORA_DISABLED(send ENABLE or Y),main=%.2fMHz,backup=%.2fMHz,SF%d,BW%.0f,sync=0x%02X,cmds=ENABLE(Y);"
      "DISABLE(D);GS(H);GF,0|1;LIVE(S);HIBERNATE(X);PG;SD;XD;CAL(C);CALA(C1);CALI(C2);BITE;FPL[,1];FQ,0|1;TX,0|1;ST;"
      "SV,ch,deg;PW,ch,us;LOCK,ch;UNLOCK,ch;LOCKALL;TEST1-3;MSEQ;NSTEP;FSEQ;SEQRST;SEP;BZ,0|1;GR;LR;RF;LK",
      CH_MAIN_MHZ, CH_BACKUP_MHZ, LORA_SF, LORA_BW, LORA_SYNC);
  lastStatusMs = lastFlushMs = millis();
}

void loop() {
  handleUsb();
  serviceRadio();
  serviceChannels();
  serviceUplink();
  uint32_t now = millis();
  if (now - lastStatusMs >= STATUS_PERIOD_MS) { lastStatusMs = now; printStatus(); }
  if (sdOk && now - lastFlushMs >= SD_FLUSH_MS) { lastFlushMs = now; telemFile.flush(); eventFile.flush(); }
}
