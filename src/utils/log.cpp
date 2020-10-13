#include <Arduino.h>
#include "log.h"

void ComLogger::Init() {
    xLogQueue = xQueueCreate( CLOG_Q_SZ, sizeof( struct AMessage ) );

    if( xLogQueue == NULL )
    {
        /* Queue was not created and must not be used. */
        Serial.println("Couldn't create LQ");
        return;
    }

    ucMessageID = 0;
    ucLastProcMsgID = 0;
    
    Serial.println("Logger OK");
}

void ComLogger::vAddLogMsg(const char *pucMsg) {  
  struct AMessage txMessage;
  txMessage.ucMessageID = ++ucMessageID;
  if(pucMsg) 
    strncpy(txMessage.ucData, pucMsg, CLOG_MSG_SZ);          
  else *txMessage.ucData=0;  
  txMessage.xTick = xTaskGetTickCount();
  xQueueSendToBack( xLogQueue, ( void * ) &txMessage, ( TickType_t ) 0 );          
}

void ComLogger::vAddLogMsg(const char *pucMsg, const char *ps) {     
  struct AMessage txMessage;
  txMessage.ucMessageID = ++ucMessageID;
  if(pucMsg) 
    strncpy(txMessage.ucData, pucMsg, CLOG_MSG_SZ);          
  else *txMessage.ucData=0;  
  sstrncat(txMessage.ucData, ":", CLOG_MSG_SZ);          
  if(ps != NULL)
    sstrncat(txMessage.ucData, ps, CLOG_MSG_SZ);          
  txMessage.xTick = xTaskGetTickCount();
  xQueueSendToBack( xLogQueue, ( void * ) &txMessage, ( TickType_t ) 0 );          
}

void ComLogger::vAddLogMsg(const char *pucMsg, int16_t i) {     
  struct AMessage txMessage;
  txMessage.ucMessageID = ++ucMessageID;
  if(pucMsg) 
    strncpy(txMessage.ucData, pucMsg, CLOG_MSG_SZ);          
  else *txMessage.ucData=0;        
  sstrncat(txMessage.ucData, ":", CLOG_MSG_SZ);      
  s_itoa16_cat(i, txMessage.ucData, CLOG_MSG_SZ);     
  txMessage.xTick = xTaskGetTickCount();
  xQueueSendToBack( xLogQueue, ( void * ) &txMessage, ( TickType_t ) 0 );          
}

void ComLogger::vAddLogMsg(const char *pucMsg1, int16_t i1, const char *pucMsg2, int16_t i2) {     
  struct AMessage txMessage;
  txMessage.ucMessageID = ++ucMessageID;
  if(pucMsg1) 
    strncpy(txMessage.ucData, pucMsg1, CLOG_MSG_SZ);          
  else *txMessage.ucData=0;  
  sstrncat(txMessage.ucData, ":", CLOG_MSG_SZ);          
  s_itoa16_cat(i1, txMessage.ucData, CLOG_MSG_SZ);
  if(pucMsg2) {
    sstrncat(txMessage.ucData, ",", CLOG_MSG_SZ);                
    sstrncat(txMessage.ucData, pucMsg2, CLOG_MSG_SZ);          
  }
  sstrncat(txMessage.ucData, ":", CLOG_MSG_SZ);          
  s_itoa16_cat(i2, txMessage.ucData, CLOG_MSG_SZ);
  txMessage.xTick = xTaskGetTickCount();
  xQueueSendToBack( xLogQueue, ( void * ) &txMessage, ( TickType_t ) 0 );          
}

void ComLogger::vAddLogMsg(const char *pucMsg1, int32_t i1, int32_t i2, int32_t i3) {
  struct AMessage txMessage;
  txMessage.ucMessageID = ++ucMessageID;
  if(pucMsg1) 
    strncpy(txMessage.ucData, pucMsg1, CLOG_MSG_SZ);          
  else *txMessage.ucData=0;  
  sstrncat(txMessage.ucData, ":", CLOG_MSG_SZ);          
  s_itoa32_cat(i1, txMessage.ucData, CLOG_MSG_SZ);
  sstrncat(txMessage.ucData, ",", CLOG_MSG_SZ);          
  s_itoa32_cat(i2, txMessage.ucData, CLOG_MSG_SZ);
  sstrncat(txMessage.ucData, ",", CLOG_MSG_SZ);          
  s_itoa32_cat(i3, txMessage.ucData, CLOG_MSG_SZ);
  txMessage.xTick = xTaskGetTickCount();
  xQueueSendToBack( xLogQueue, ( void * ) &txMessage, ( TickType_t ) 0 );          
}

void ComLogger::vAddLogMsg(const char *pucMsg1, int32_t i1, int32_t i2, int32_t i3, int32_t i4) {
  struct AMessage txMessage;
  txMessage.ucMessageID = ++ucMessageID;  
  if(pucMsg1) 
    strncpy(txMessage.ucData, pucMsg1, CLOG_MSG_SZ);          
  else *txMessage.ucData=0;       
  sstrncat(txMessage.ucData, ":", CLOG_MSG_SZ);          
  s_itoa32_cat(i1, txMessage.ucData, CLOG_MSG_SZ);
  sstrncat(txMessage.ucData, ",", CLOG_MSG_SZ);          
  s_itoa32_cat(i2, txMessage.ucData, CLOG_MSG_SZ);
  sstrncat(txMessage.ucData, ",", CLOG_MSG_SZ);          
  s_itoa32_cat(i3, txMessage.ucData, CLOG_MSG_SZ);
    sstrncat(txMessage.ucData, ",", CLOG_MSG_SZ);          
  s_itoa32_cat(i4, txMessage.ucData, CLOG_MSG_SZ);
  txMessage.xTick = xTaskGetTickCount();    
  xQueueSendToBack( xLogQueue, ( void * ) &txMessage, ( TickType_t ) 0 );          
}

void ComLogger::Process() {  
  const long mss = 1000;
  const long msm = 60*mss;
  const long msh = 60*msm;
  const long msd = 24*msh;
  
  if( xQueueReceive( xLogQueue, &rxMessage, ( TickType_t ) 10 ) )
  {
    if((unsigned char)(rxMessage.ucMessageID - ucLastProcMsgID) != 1) {
      Serial.print("...skipped...");
      Serial.println((int)((unsigned char)(rxMessage.ucMessageID - ucLastProcMsgID)));
    }
    //Serial.print((int)rxMessage.ucMessageID);
    //Serial.print(" : ");
    // dd hh:mm:ss:ttt
    long t = (long)rxMessage.xTick*portTICK_PERIOD_MS;

    _ltoa((t%msd)/msh, prtbuf, 2); //hrs
    sstrncat(prtbuf, ":", CLOG_PB_SZ);          
    ltoa_cat((t%msh)/msm, prtbuf, 2); //min
    sstrncat(prtbuf, ":", CLOG_PB_SZ);          
    ltoa_cat((t%msm)/mss, prtbuf, 2); //sec
    sstrncat(prtbuf, ".", CLOG_PB_SZ);          
    ltoa_cat(t%mss, prtbuf, 4); //ms
    sstrncat(prtbuf, " ", CLOG_PB_SZ);          
    Serial.print(prtbuf);
    Serial.println(rxMessage.ucData);
    ucLastProcMsgID = rxMessage.ucMessageID;
    //vTaskDelay(100);
    vTaskDelay(20);         
   }   

}



/* reverse:  reverse string s in place */
 void reverse(char s[])
 {
     int i, j;
     char c;

     for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
         c = s[i];
         s[i] = s[j];
         s[j] = c;
     }
}  

/* itoa:  convert n to characters in s */
void _itoa(int n, char s[], int zn)
 {
     int i, sign;

     if ((sign = n) < 0)  /* record sign */
         n = -n;          /* make n positive */
     i = 0;
     do {       /* generate digits in reverse order */
         s[i++] = n % 10 + '0';   /* get next digit */
     } while ((n /= 10) > 0);     /* delete it */

     while(i<zn) s[i++] = '0';
     
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
}  


/* itoa:  convert n to characters in s */

void _ltoa(int32_t n, char s[], int zn)
 {
     int32_t i, sign;

     if ((sign = n) < 0)  /* record sign */
         n = -n;          /* make n positive */
     i = 0;
     do {       /* generate digits in reverse order */
         s[i++] = n % 10 + '0';   /* get next digit */
     } while ((n /= 10) > 0);     /* delete it */

     while(i<zn) s[i++] = '0';

     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
}  

void _ftoa(float f, char s[], int n) {
  int32_t m = 1;
  while(n--) m*=10;
  int32_t l = (int32_t)(f * m);
  _ltoa(l/m, s);
  strcat(s, ".");
  itoa_cat(abs(l)%m, s, n);
}
