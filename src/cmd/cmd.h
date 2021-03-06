#ifndef _UMP_CMD_H_
#define _UMP_CMD_H_

const int BUF_SZ = 255;
const int EVENT_BUF_SZ = 80;
const int CMD_EVENT_Q_SZ=12;

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
  int16_t Init();
  int16_t Listen(uint16_t port);
  int16_t doCmd();
  void  doEvents();
  boolean connected();
  boolean read();
  void  respond();
  bool  setSysLog(StaticJsonDocument<256> &in);
  // int16_t getSysLogLevel();
  //boolean sendEvent(uint16_t id, uint8_t module,  uint8_t level, uint8_t code, int8_t npa=0, int16_t *pa=NULL);
  //boolean sendEvent(uint16_t id, uint8_t module,  uint8_t level, uint8_t code, const char *s);
  boolean putEvent(uint8_t module,  uint8_t level, uint8_t code, const char *s);
protected:
  struct QEvent
  {
    //uint8_t ucMessageID;
    //TickType_t xTick;
    uint8_t uModule;
    uint8_t uLevel; 
    uint8_t uCode;
    uint16_t uId;
    char ucData[EVENT_BUF_SZ ];
  };
  CmdProc() : isConnected(false), log_on(0), log_port(0), event_id(0) {;}
  void _sendToSysLog(StaticJsonDocument<256> &out);
  char packetBuffer[BUF_SZ];
  char prtBuffer[BUF_SZ];
  bool isConnected;
  WiFiUDP udp_rcv;
  WiFiUDP udp_snd;
  IPAddress log_addr;
  uint8_t log_on;
  uint16_t log_port;
  uint16_t event_id;
  struct QEvent rxEvent;
  QueueHandle_t xEventQueue;
  //xSemaphoreHandle xEventFree;
};

#endif //_UMP_CMD_H_

