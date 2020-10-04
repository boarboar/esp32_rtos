#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "../utils/log.h"

// AsyncUDP

// #include "stat.h"
// #include "cfg.h"
// #include "controller.h"

#include "cmd.h"

extern "C" int rom_phy_get_vdd33();

extern ComLogger xLogger;

CmdProc CmdProc::Cmd; // singleton

typedef int16_t (*VFP)(StaticJsonDocument<256>&,StaticJsonDocument<256>&);

int16_t _doCmd(StaticJsonDocument<256>&,StaticJsonDocument<256>&);
int16_t c_info(StaticJsonDocument<256>&,StaticJsonDocument<256>&);
// int16_t c_reset(JsonObject&,JsonObject&);
// int16_t c_setsyslog(JsonObject&,JsonObject&);
// int16_t c_getpos(JsonObject&,JsonObject&);
// int16_t c_resetMPU(JsonObject&,JsonObject&);
// int16_t c_drive(JsonObject&,JsonObject&);
// int16_t c_steer(JsonObject&,JsonObject&);
// int16_t c_move(JsonObject&,JsonObject&);
// int16_t c_setpidp(JsonObject&,JsonObject&);
// int16_t c_bear(JsonObject&,JsonObject&);

//VFP cmd_imp[10]={c_info, c_reset, c_setsyslog, c_getpos, c_resetMPU, c_drive, c_steer, c_move, c_setpidp, c_bear};

VFP cmd_imp[10]={c_info, c_info, c_info, c_info, c_info, c_info, c_info, c_info, c_info, c_info};

const char *CMDS="INFO\0RST\0SYSL\0POS\0RSTMPU\0D\0S\0M\0SPP\0B\0";
enum CMDS_ID {CMD_INFO=0, CMD_RESET=1, CMD_SETSYSLOG=2, CMD_POS=3, CMD_RESET_MPU=4, CMD_DRIVE=5, CMD_STEER=6, CMD_MOVE=7, CMD_SETPIDP=8, CMD_BEAR=9};
// {"I":1,"C":"INFO"}
// {"I":1,"C":"RST"}
// {"I":1,"C":"SYSL", "ON":1, "ADDR":"192.168.1.141", "PORT":4444}
// {"I":1,"C":"POS"}
// {"I":1,"C":"RSTMPU"}
// {"I":1,"C":"D","RPS":[0.1, -0.1]}
// {"I":1,"C":"S","S":-10}
// {"I":1,"C":"M","V":10}
// {"I":1,"C":"SPP","P":1,"PA":[30, 320, 10, 80, 100],"S":0}
// {"I":1,"C":"B","A":45}

int16_t CmdProc::init(uint16_t port) {
  if(udp_rcv.begin(port)) {
    isConnected=true;
    return 1;
  }
  return 0;
}   

boolean CmdProc::connected() { return isConnected; }

boolean CmdProc::read() {
  int packetSize = udp_rcv.parsePacket(); 
  
  if (packetSize) {

    xLogger.vAddLogMsg("UDP PSZ: ", packetSize);
    int len = udp_rcv.read(packetBuffer, BUF_SZ);
    if(len>BUF_SZ-1) len=BUF_SZ-1;
    packetBuffer[len] = 0;    
    xLogger.vAddLogMsg("UDP: ", packetBuffer);
      /*
      Serial.print("From: "); Serial.print(udp_rcv.remoteIP()); Serial.print(":"); Serial.print(udp_rcv.remotePort());
      Serial.print(" len: "); Serial.print(len);Serial.print(" Val: "); Serial.println(packetBuffer);
      */      
  }
 return packetSize>0; 
}

void CmdProc::respond() {
  udp_snd.beginPacket(udp_rcv.remoteIP(), udp_rcv.remotePort());
  udp_snd.write((uint8_t *)prtBuffer, strlen(prtBuffer));
  udp_snd.endPacket();
}

int16_t CmdProc::doCmd() {    
  StaticJsonDocument<256> docIn;
  StaticJsonDocument<256> docOut;
  DeserializationError error = deserializeJson(docIn, packetBuffer);
  if(error) {
    xLogger.vAddLogMsg("JS Error: ", error.c_str());
    docOut["I"] = -1;
    docOut["R"] = -1;
  } else {
    serializeJson(docIn, prtBuffer, BUF_SZ-1);
    xLogger.vAddLogMsg("JS In: ", prtBuffer);
    //serializeJson(docIn, Serial);
    _doCmd(docIn, docOut);
  }

  serializeJson(docOut, prtBuffer, BUF_SZ-1);
  xLogger.vAddLogMsg("JS Out: ", prtBuffer);
  
  return !error;
}

/*
int16_t CmdProc::getSysLogLevel() { return CfgDrv::Cfg.log_on;}

boolean CmdProc::sendEvent(uint16_t id, uint8_t module,  uint8_t level, uint8_t code, int8_t npa, int16_t *pa) {
  //if(CfgDrv::Cfg.log_on<SL_LEVEL_ALARM) return false; //!!! should be adjusted with level
  StaticJsonBuffer<400> jsonBufferOut;
  JsonObject& rootOut = jsonBufferOut.createObject();
  rootOut["C"] = level>CMD_LLEVEL_ALR ? "L" : "A";
  rootOut["T"] = millis();
  rootOut["I"] = id;
  rootOut["M"] = module;
  rootOut["F"] = code;
  if(pa && npa) {
    JsonArray& par = rootOut.createNestedArray("P");
    uint8_t mpa=npa-1;
    while(mpa && pa[mpa]) mpa--;
    if(mpa) {
      mpa++;    
      for(uint8_t i=0; i<mpa; i++) par.add(pa[i]);
    }    
  }
  _sendToSysLog(rootOut);
  return true;
}

boolean CmdProc::sendEvent(uint16_t id, uint8_t module,  uint8_t level, uint8_t code, const char *s) {
  //if(CfgDrv::Cfg.log_on<SL_LEVEL_ALARM) return false; //!!! should be adjusted with level
  StaticJsonBuffer<400> jsonBufferOut;
  JsonObject& rootOut = jsonBufferOut.createObject();
  rootOut["C"] = level>CMD_LLEVEL_ALR ? "L" : "A";
  rootOut["T"] = millis();
  rootOut["I"] = id;
  rootOut["M"] = module;
  rootOut["F"] = code;
  rootOut["S"] = s;
  _sendToSysLog(rootOut);
  return true;
}
  
void CmdProc::_sendToSysLog(JsonObject& rootOut) {
  rootOut.printTo(packetBuffer, BUF_SZ-1);
  udp_snd.beginPacket(CfgDrv::Cfg.log_addr, CfgDrv::Cfg.log_port);
  udp_snd.write(packetBuffer, strlen(packetBuffer));
  udp_snd.endPacket();  
}
*/

int16_t _doCmd(StaticJsonDocument<256> & root, StaticJsonDocument<256> & rootOut) {  
  rootOut["T"] = xTaskGetTickCount();

  long id = root["I"];
  const char* cmd = root["C"];
  
  //Serial.print("Id: "); Serial.print(id);
  rootOut["I"] = id;
  if(!cmd || !*cmd) {
    rootOut["R"] = -2;
    return 0;
  }
  if(cmd) {
    //Serial.print(" Cmd:"); Serial.println(cmd);
    rootOut["C"] = cmd;
    const char *p=CMDS;
    uint8_t i=0;
    while(*p && strcmp(p, cmd)) { p+=strlen(p)+1; i++; }
    //if(i>=CMD_NOCMD) { rootOut["R"] = -2; return 0;}
    if(!*p) { rootOut["R"] = -2; return 0;}
    rootOut["R"] = (*(cmd_imp[i]))(root, rootOut);
  }      
  return 0;
}

int16_t c_info(StaticJsonDocument<256>& /*root*/, StaticJsonDocument<256>& rootOut) {
  float voltage = ((float)rom_phy_get_vdd33()) / 1000;
  //Serial.println("INFO"); 
  //rootOut["MST"]=MpuDrv::Mpu.getStatus();
  //rootOut["MDR"]=MpuDrv::Mpu.isDataReady();
  // rootOut["MST"]=Controller::ControllerProc.getIMUStatus();
  // rootOut["CST"]=Controller::ControllerProc.getStatus() ? 0 : 6; // tmp
  rootOut["FHS"]=ESP.getFreeHeap();
  rootOut["FSS"]=ESP.getFreeSketchSpace();
  rootOut["VCC"]=(int16_t)(voltage*100);  
  rootOut["WP"]=(int16_t)(WiFi.RSSI());
  // rootOut["MDC"]=Stat::StatStore.cycle_mpu_dry_cnt;
  // rootOut["MOC"]=Stat::StatStore.mpu_owfl_cnt;
  // rootOut["MGC"]=Stat::StatStore.mpu_gup_cnt;
  // rootOut["MXC"]=Stat::StatStore.mpu_exc_cnt;
  // rootOut["MND"]=Stat::StatStore.mpu_ndt_cnt;
  // JsonArray& data = rootOut.createNestedArray("CYT");
  // for(int i=0; i<4; i++) data.add(Stat::StatStore.cycle_delay_cnt[i]);
  return 0;
}


// int16_t c_reset(JsonObject& /*root*/, JsonObject& /*rootOut*/) {
//   Serial.println(F("Resetting...")); 
//   delay(1000);
//   ESP.restart();
//   return 0;
// }


// int16_t c_setsyslog(JsonObject& root, JsonObject& /*rootOut*/) {
//   return CfgDrv::Cfg.setSysLog(root) ? 0 : -3;
// }

// int16_t c_setpidp(JsonObject& root, JsonObject& /*rootOut*/) {
//   if(CfgDrv::Cfg.setPidParams(root)) {
//     Controller::ControllerProc.setReset(Controller::CTL_RST_CTL);
//     return 0;
//   }
//   return -3;
// }

// int16_t c_resetMPU(JsonObject& root, JsonObject& rootOut) {
//   const char *action=root["A"];
//   if(!action || !*action) return -3;
//   if(!strcmp(action, "MPU")) {
//     Serial.println(F("Req to RST MPU/CTL...")); 
//     Controller::ControllerProc.setReset(Controller::CTL_RST_IMU);
//   } else if(!strcmp(action, "MPU_INT")) {
//     Serial.println(F("Resetting MPU/CTL integrator...")); 
//     //MpuDrv::Mpu.resetIntegrator();
//     //Controller::ControllerProc.resetIntegrator();
//     Controller::ControllerProc.setReset(Controller::CTL_RST_CTL);
//   } else return -3;

//   return 0;
// }


// int16_t c_getpos(JsonObject& /*root*/, JsonObject& rootOut) {
//   //{"C": "I", "T":12345, "R":0, "C": "POS", "YPR": [59, 12, 13], "A": [0.01, 0.02, -0.03], "P": [100.01, 200.44, 0.445]}
//   int16_t i;
//   rootOut["MST"]=Controller::ControllerProc.getIMUStatus();
//   JsonArray& ya = rootOut.createNestedArray("YPR");
//   ya.add(Controller::ControllerProc.getYaw_grad());
//   ya.add(0);
//   ya.add(0);
//   JsonArray& r = rootOut.createNestedArray("CRD");
//   r.add(Controller::ControllerProc.getX_cm());
//   r.add(Controller::ControllerProc.getY_cm());
//   r.add(0); // Z-crd
//   JsonArray& pw = rootOut.createNestedArray("W");
//   int16_t *pwrs=Controller::ControllerProc.getPower();
//   pw.add(pwrs[0]), pw.add(pwrs[1]);   
//   uint8_t ns=Controller::ControllerProc.getNumSensors();
//   JsonArray& s = rootOut.createNestedArray("S");
//   for(i=0; i<ns; i++) s.add((int)Controller::ControllerProc.getSensors()[i]);
//   rootOut["D"]=Controller::ControllerProc.getDist_cm();
//   rootOut["V"]=Controller::ControllerProc.getSpeed_cmps();
//   return 0;
// }

// int16_t c_drive(JsonObject& root, JsonObject& rootOut) {
//   if(!Controller::ControllerProc.getStatus()) return -5;
//   Serial.print(F("Drv req "));
//   if(root["RPS"].is<JsonArray&>()) {
//     JsonArray& rps=root["RPS"].asArray();
//     float r0=rps[0], r1=rps[1];
//     Serial.print("TR: "); Serial.print(r0); Serial.print(", "); Serial.print(r1);
//     //if(!Controller::ControllerProc.setTargRotRate(r0, r1)) return -5;    
//     if(!Controller::ControllerProc.setTargPower(r0, r1)) return -5;    
//   } else {
//     ; // get
//   }
//   Serial.println();
//   return 0;
// }

// int16_t c_steer(JsonObject& root, JsonObject& rootOut) {
//   if(!Controller::ControllerProc.getStatus()) return -5;
//   Serial.print(F("Steer req "));
//   int16_t steer_val=root["S"];
//   Serial.println(steer_val);
//   if(!Controller::ControllerProc.setTargSteering(steer_val)) return -5;
//   JsonArray& pw = rootOut.createNestedArray("W");
//   int16_t *pwrs=Controller::ControllerProc.getPower();
//   pw.add(pwrs[0]), pw.add(pwrs[1]);
//   rootOut["V"]=Controller::ControllerProc.getSpeed_cmps();
//   return 0;
// }

// int16_t c_move(JsonObject& root, JsonObject& rootOut) {
//   if(!Controller::ControllerProc.getStatus()) return -5;
//   Serial.print(F("Move req "));
//   int16_t speed_val=root["V"];
//   Serial.println(speed_val);
//   if(!Controller::ControllerProc.setTargSpeed(speed_val)) return -5;    
//   JsonArray& pw = rootOut.createNestedArray("W");
//   int16_t *pwrs=Controller::ControllerProc.getPower();
//   pw.add(pwrs[0]), pw.add(pwrs[1]);
//   rootOut["V"]=Controller::ControllerProc.getSpeed_cmps();
//   return 0;
// }

// int16_t c_bear(JsonObject& root, JsonObject& rootOut) {
//   if(!Controller::ControllerProc.getStatus()) return -5;
//   Serial.print(F("Bearing req "));
//   int16_t angle_val=root["A"];
//   Serial.println(angle_val);
//   if(!Controller::ControllerProc.setTargBearing(angle_val)) return -5;
//   JsonArray& pw = rootOut.createNestedArray("W");
//   int16_t *pwrs=Controller::ControllerProc.getPower();
//   pw.add(pwrs[0]), pw.add(pwrs[1]);
//   rootOut["V"]=Controller::ControllerProc.getSpeed_cmps();
//   return 0;
// }
