#ifndef _UMP_CMD_H_
#define _UMP_CMD_H_

const int BUF_SZ = 255;

#define SL_LEVEL_NONE   0
#define SL_LEVEL_ALARM  1
#define SL_LEVEL_MESSAGE  2

class CmdProc {
public:  
  //static const int8_t ALR_MPU_RESET=10;
  //enum Alarms {ALR_RESET=1, ALR_MPU_RESET=10, ALR_MPU_FAILURE=11, ALR_CTL_RESET=20, ALR_CTL_FAILURE=21, ALR_CTL_LOG=100}; 
  enum AlarmsLevels {CMD_LLEVEL_ALR=1,CMD_LLEVEL_LOG=2};  
public:  
  static CmdProc Cmd; // singleton  
  int16_t init(uint16_t port);
  int16_t doCmd();
  boolean connected();
  boolean read();
  void  respond();
  // int16_t getSysLogLevel();
  // boolean sendEvent(uint16_t id, uint8_t module,  uint8_t level, uint8_t code, int8_t npa=0, int16_t *pa=NULL);
  // boolean sendEvent(uint16_t id, uint8_t module,  uint8_t level, uint8_t code, const char *s);
protected:
  CmdProc() : isConnected(false) {;}
  void _sendToSysLog(JsonObject& rootOut);
  char packetBuffer[BUF_SZ];
  char prtBuffer[BUF_SZ];
  bool isConnected;
  WiFiUDP udp_rcv;
  WiFiUDP udp_snd;
};

#endif //_UMP_CMD_H_

