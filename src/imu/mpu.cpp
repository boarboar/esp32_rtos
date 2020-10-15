#include <Arduino.h>
#include "../utils/log.h"
//#include "comm_mgr.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "mpu.h"

//#define SCALE_A (2.0f*8192.0f) // 1g = (9.80665 m/s^2)
#define SCALE_A (8192.0f) // 1g = (9.80665 m/s^2)

#define G_FORCE 9.80665f
#define G_SCALE (G_FORCE/SCALE_A)
#define A_K 0.3f

// MPF 2
// MPF 3

extern ComLogger xLogger;

MpuDrv MpuDrv::Mpu; // singleton

MpuDrv::MpuDrv() : dmpStatus(ST_0), integrate(false) {
  }

int8_t MpuDrv::getStatus() { return dmpStatus; }
uint8_t MpuDrv::isDataReady() { return dmpStatus==ST_READY && data_ready; }
uint8_t MpuDrv::isNeedReset() { return need_reset; }
void    MpuDrv::needReset() {  need_reset=true; }

bool MpuDrv::Acquire() {
  return xSemaphoreTake( xIMUFree, ( portTickType ) 10 ) == pdTRUE;
}


void MpuDrv::Release() {
  xSemaphoreGive( xIMUFree );
}


int16_t MpuDrv::cycle_dt() 
{
  int16_t res=cycle(xTaskGetTickCount()-xLastWakeTime);          
  xLastWakeTime=xTaskGetTickCount();
  return res;      
}

int16_t MpuDrv::init(bool integrate) {
  vSemaphoreCreateBinary(xIMUFree);      
  dmpStatus=ST_0;
  data_ready=0;
  fifoCount=0;
  count=0;
  conv_count=0;
  need_reset=0;
  this->integrate = integrate;

  for(int i=0; i<MPU_FAIL_CNT_SZ; i++) fail_cnt[i]=0;
  resetIntegrator();
  xLogger.vAddLogMsg("Init I2C dev...");
   
  mpu.initialize();
  // verify connection
  xLogger.vAddLogMsg("Test device conn...");
  if(!mpu.testConnection()) {
    xLogger.vAddLogMsg("MPU6050 conn fail");
    need_reset=1;
    return 0;
  }

  // load and configure the DMP  
  xLogger.vAddLogMsg("Init DMP...");

  uint8_t devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default (?)
  
    // turn on the DMP, now that it's ready
    xLogger.vAddLogMsg("Enab DMP...");
    mpu.setDMPEnabled(true);

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    // enter warmup/convergence stage 
    dmpStatus=ST_WUP;
    xStart=xLastWakeTime=xTaskGetTickCount();

    // TODO// xCommMgr.vAddAlarm(CommManager::CM_EVENT, CommManager::CM_MODULE_IMU, MPU_FAIL_INIT, 0); 
    xLogger.vAddLogMsg("DMP ok!"); //Serial.println(packetSize);    
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    //Serial.print("DMP Init fail, code "); Serial.println(devStatus);
    xLogger.vAddLogMsg("DMP Init fail");
    dmpStatus = ST_FAIL;
    // TODO// xCommMgr.vAddAlarm(CommManager::CM_ALARM, CommManager::CM_MODULE_IMU, MPU_FAIL_INIT, -1);
    need_reset=1;
    
  }
  return dmpStatus;
}

int16_t MpuDrv::cycle(uint16_t dt) {
  uint8_t i=0;
  bool settled=false;
  if (dmpStatus==ST_0 || dmpStatus==ST_FAIL) return -1;

  //if(data_ready && (micros()-start)/1000000L>1) { // micros()-start  = NEED TO BE FIXED!!!!
  if(data_ready && (xTaskGetTickCount()-xLastWakeTime)/1000L>1) { 
    // no data from MPU after 1sec
    xLogger.vAddLogMsg("MPU - no data in 1s!");
    need_reset=1;
    data_ready=0;
    fail_cnt[MPU_FAIL_NODATA_IDX]++;
    return -10;
  }

/*
    // wait for MPU interrupt or extra packet(s) available
    if (!mpuInterrupt && fifoCount < packetSize) return ;
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    */
  uint8_t mpuIntStatus = mpu.getIntStatus();


  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    fifoCount=0;
    fail_cnt[MPU_FAIL_FIFOOVFL_IDX]++;  // overfloods with the alarms
    return -2;
  } 
  // otherwise, check for DMP data ready interrupt (this should happen frequently)

  if (!(mpuIntStatus & 0x02) && (fifoCount < packetSize) ) return 0; // nothing to read
  
  fifoCount = mpu.getFIFOCount();
  if(fifoCount < packetSize) return 0; // ???
  /*
  //while (fifoCount < packetSize && i++<5) { fifoCount = mpu.getFIFOCount(); yield(); } 
  while (fifoCount < packetSize && i++<5) { 
    vTaskDelay(1);    
    fifoCount = mpu.getFIFOCount();     
    } 
  
  if(fifoCount < packetSize) {
    fail_cnt[MPU_FAIL_FIFOTMO_IDX]++;
    return 0; // giveup
  }
  */
  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  //mpu.resetFIFO(); fifoCount=0; // this is in case of overflows... 
  fifoCount -= packetSize;
  if(fifoCount >0) { 
    mpu.resetFIFO();
    fifoCount=0;
    fail_cnt[MPU_FAIL_FIFOEXCESS_IDX]++; // overfloods with the alarms
    return -3;
  }   
    
  mpu.dmpGetQuaternion(q16, fifoBuffer);
  mpu.dmpGetAccel(&aa16, fifoBuffer);
    
  if(dmpStatus==ST_WUP && count%200==1) { // warmup covergence state, check every 200th reading: 1,201,401,...
    int16_t ad16[3];
    int32_t qe=0, ae=0, e;        
    ad16[0]=aa16.x-aa16_0.x; ad16[1]=aa16.y-aa16_0.y; ad16[2]=aa16.z-aa16_0.z;                   
    for(i=0; i<4; i++) {e=q16[i]-q16_0[i]; if(e<0) e=-e; if(e>qe) qe=e;}
    for(i=0; i<3; i++) {e=ad16[i]; if(e<0) e=-e; if(e>ae) ae=e;}        
    xLogger.vAddLogMsg("QE", qe, "AE", ae);  
    // TODO// xCommMgr.vAddAlarm(CommManager::CM_INFO, CommManager::CM_MODULE_IMU, MPU_EVENT_CONV_PROG, qe, ae);       
    if(qe<QUAT_INIT_TOL && ae<ACC_INIT_TOL) {
      conv_count++;
      if((xTaskGetTickCount()-xStart)/1000L > INIT_PERIOD_MIN && conv_count>3) settled=true;             
    } else conv_count=0;  
    if((xTaskGetTickCount()-xStart)/1000L > INIT_PERIOD_MAX) {
      xLogger.vAddLogMsg("MPU Failed to converge");
      // TODO// xCommMgr.vAddAlarm(CommManager::CM_EVENT, CommManager::CM_MODULE_IMU, MPU_FAIL_CONVTMO, -1); 
      settled=true;      
    }

   for(i=0; i<4; i++) q16_0[i]=q16[i];
   aa16_0 = aa16;
        
   if(settled) {
      xLogger.vAddLogMsg("MPU converged");
      dmpStatus=ST_READY;        
      //xLastWakeTime=xTaskGetTickCount();
      // TODO// xCommMgr.vAddAlarm(CommManager::CM_EVENT, CommManager::CM_MODULE_IMU, MPU_FAIL_CONVTMO, 0);       
     }
  } // warmup

  count++; 
  
  if(dmpStatus==ST_READY) {
    if(integrate) {
      // =======actually, this is not needed if we do not use IMU accel-based integration integration
      // integrate motion
      Quaternion q, q0;
      VectorFloat ga;    
      VectorInt16 aaReal, aaWorld;
      
      float ts;
      // get world frame accel (with adjustment) - needed for V-integration
      mpu.dmpGetQuaternion(&q, q16);
      mpu.dmpGetQuaternion(&q0, q16_0);
      q0=q0.getConjugate();
      q=q0.getProduct(q); // real quaternion (relative to base)
      mpu.dmpGetGravity(&ga, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa16, &ga);
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);      
      //a.x=aaWorld.x*G_SCALE; a.y=aaWorld.y*G_SCALE; a.z=aaWorld.z*G_SCALE;      
      ga.x=aaWorld.x*G_SCALE; ga.y=aaWorld.y*G_SCALE; ga.z=aaWorld.z*G_SCALE;      
      if(settled) { // first time only - store base accel
        //a0=a;
        a0=ga;
        xLogger.vAddLogMsg("A Base:", a0.x, a0.y, a0.z); // (m/s^2)   
      }      
      // adjust to base accel
      //a.x-=a0.x; a.y-=a0.y; a.z-=a0.z;
      ga.x-=a0.x; ga.y-=a0.y; ga.z-=a0.z;
      // low-pass filter for acceleration
      a.x = a.x - A_K * (a.x - ga.x);
      a.y = a.y - A_K * (a.y - ga.y);
      a.z = a.z - A_K * (a.z - ga.z);
      ts=(float)dt/1000.0f;
      v.x+=a.x*ts; v.y+=a.y*ts; v.z+=a.z*ts;
  //      r.x+=v.x*ts; r.y+=v.y*ts; r.z+=v.z*ts;
    }
    data_ready=1; 
    return settled ? 2 : 1;
  }      
  return 10;
}

void MpuDrv::resetIntegrator() {
  a.x=a.y=a.z=0.0f;
  v.x=v.y=v.z=0.0f;  
  ypr[0]=ypr[1]=ypr[2]=1.0f;
}

void MpuDrv::process() {
  Quaternion q, q0;
  VectorFloat gravity;    
  //char buf[20];
  //xLogger.vAddLogMsg("QT ", q16[0], q16[1], q16[2], q16[3]);
  mpu.dmpGetQuaternion(&q, q16);
  mpu.dmpGetQuaternion(&q0, q16_0);
  q0=q0.getConjugate();
  q=q0.getProduct(q); // real quaternion (relative to base)
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); 
}

void MpuDrv::copyAlarms() {
  //copy alarms for flushing
  for(int i=0; i<MPU_FAIL_CNT_SZ; i++) {
    fail_cnt_buf[i]=fail_cnt[i];
    fail_cnt[i]=0;
  }
}

void MpuDrv::flushAlarms() {  
  for(int i=0; i<MPU_FAIL_CNT_SZ; i++) {
    if(fail_cnt_buf[i]) {      
      xLogger.vAddLogMsg("MPF", i, (const char *)NULL, fail_cnt_buf[i]);
      //xCommMgr.vAddAlarm(CommManager::CM_ALARM, CommManager::CM_MODULE_IMU, MPU_FAIL_CYCLE+i, fail_cnt_buf[i]); 
    }
  }
}
  
float MpuDrv::getYaw() { return ypr[0]; }

void MpuDrv::getAll(float* ypr, float* af, float* vf) {     
  if(ypr) {   
    ypr[0]=this->ypr[0]; ypr[1]=this->ypr[1]; ypr[2]=this->ypr[2];
  }
  if(af) {
    af[0]=a.x; af[1]=a.y; af[2]=a.z;
  }
  if(vf) {
    vf[0]=v.x; vf[1]=v.y; vf[2]=v.z;
  }
}  

