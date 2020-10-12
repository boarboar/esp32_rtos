
#define CLOG_MSG_SZ 80
#define CLOG_Q_SZ 32
#define CLOG_PB_SZ 24

class ComLogger {  
  public:
    void Init();    
    void vAddLogMsg(const char *pucMsg=NULL);
    void vAddLogMsg(const char *pucMsg, const char *ps);
    void vAddLogMsg(const char *pucMsg, int16_t i);
    void vAddLogMsg(const char *pucMsg1, int16_t i1, const char *pucMsg2, int16_t i2);
    void vAddLogMsg(const char *pucMsg1, int32_t i1, int32_t i2, int32_t i3);
    void vAddLogMsg(const char *pucMsg1, int32_t i1, int32_t i2, int32_t i3, int32_t i4);
    void Process();
  protected:
  
  struct AMessage
  {
    unsigned char ucMessageID;
    TickType_t xTick;
    char ucData[ CLOG_MSG_SZ ];
  };
  
  //struct AMessage txMessage;
  struct AMessage rxMessage;
  QueueHandle_t xLogQueue;
  //xSemaphoreHandle xLogFree;
  unsigned char ucLastProcMsgID;
  unsigned char ucMessageID;

  char prtbuf[ CLOG_PB_SZ ];
};

void _itoa(int n, char s[], int zn=0);
void _ltoa(int32_t n, char s[], int zn=0);
void _ftoa2(float n, char s[]);

inline void itoa_cat(int n, char s[], int zn=0) { _itoa(n, s+strlen(s), zn); }
inline void ltoa_cat(int n, char s[], int zn=0) { _ltoa(n, s+strlen(s), zn); }
inline void ftoa2_cat(float f, char s[], int zn=0) { _ftoa2(f, s+strlen(s)); }
inline void sstrncat(char dst[], const char src[], int len) {strncat(dst, src, len-strlen(dst)-1);}
inline void s_itoa16_cat(int16_t n, char dst[], int len, int zn=0) {
  int sl = strlen(dst);
  if(len-sl-1 < max(6, zn)) return; else _itoa(n, dst+sl, zn);
}
inline void s_itoa32_cat(int32_t n, char dst[], int len, int zn=0) {
  int sl = strlen(dst);
  if(len-sl-1 < max(11, zn)) return; else _ltoa(n, dst+sl, zn);
}
inline void s_ftoa2_cat(float f, char dst[], int len) {
  int sl = strlen(dst);
  if(len-sl-1 < 14) return; else _ftoa2(f, dst+sl);
}
