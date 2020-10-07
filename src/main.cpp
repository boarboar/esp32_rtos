#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <FS.h>
#include <SPIFFS.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>
#include "utils/log.h"
#include "imu/mpu.h"
#include "cmd/cmd.h"
#include "cred.inc"

#include <esp_task_wdt.h>

//  Hold-down the “BOOT” button in your ESP32 board
//  After you see the  “Connecting….” message in your Arduino IDE, release the finger from the “BOOT” button
//
//  SDA	SDA (default is GPIO 21)
//  SCL	SCL (default is GPIO 22) 
//  https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
//  data file upload:
//  pio run -t uploadfs

//3 seconds WDT
#define WDT_TIMEOUT 3

const int udp_port = 4444;
const int led1 = 2; // Pin of the LED

const int SCREEN_WIDTH = 128; // OLED display width, in pixels
const int SCREEN_HEIGHT = 64; // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
const int OLED_RESET     = -1; // Reset pin # (or -1 if sharing Arduino reset pin)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


//MpuDrv xIMU;
MpuDrv& xIMU = MpuDrv::Mpu;

ComLogger xLogger;
CmdProc& cmd = CmdProc::Cmd;

xTaskHandle MPUHandle = NULL;
SemaphoreHandle_t xDisplayMutex = NULL;
boolean fMPUReady=false;
boolean fDisplayUpdated=false;

//float yaw=0;
float ypr[3]={0, 0, 0};
char szIP[16]="";

void readFS() {
    if(!SPIFFS.begin(true)){
        Serial.println("SPIFFS Mount Failed");
        return;
    }
    Serial.println("FS mounted");

    File f = SPIFFS.open("/test.dat", "r");
    if (!f) {
      Serial.println(F("Failed to open config file"));
      return;
    }
    size_t size = f.size();

    // Serial.println("- read from file:");
    // while(f.available()){
    //     Serial.write(f.read());
    // }

    Serial.print(F("Cfg sz ")); Serial.println(size);

    StaticJsonDocument<100> doc;
    DeserializationError error = deserializeJson(doc, f);

    f.close();

    if (error) {
      Serial.println(F("Error: deserializeJson"));
      Serial.println(error.c_str());
    }
    Serial.print(F("serializeJson = "));
    serializeJson(doc, Serial);
    Serial.println("");
}

void displayUpdate() {
    char buf[64];
    display.clearDisplay(); 
    display.setCursor(0,0);
    strcpy(buf, "YPR: ");
    if(fMPUReady) {
      //itoa_cat((int)yaw, buf);
      for(int i=0; i<3; i++) {
        s_itoa16_cat((int)(ypr[i]*180.0 / PI), buf, 64);
        strcat(buf, " ");
      }
      //xLogger.vAddLogMsg("Yaw ", (int)yaw);
    }
    display.print(buf);
    display.setCursor(0,16);
    strcpy(buf, "IP: ");
    strncat(buf, szIP, 32);
    display.print(buf);
    fDisplayUpdated = true;
}

static void vSerialOutTask(void *pvParameters) {
    Serial.print("Serial Out Task started on core# ");
    Serial.println(xPortGetCoreID());
    for (;;) {
       xLogger.Process();
       vTaskDelay(20);
    }
}

static void vWiFiTask(void *pvParameters) {
  xLogger.vAddLogMsg("WiFi Task started on core# ", xPortGetCoreID());

  for (;;) {   
    szIP[0] = 0;
    WiFi.begin(CRED_WIFI_SSID, CRED_WIFI_PASS);
    xLogger.vAddLogMsg("Connecting to ", CRED_WIFI_SSID);

    int i = 0;
    while (WiFi.status() != WL_CONNECTED && i++ < 20) { 
      vTaskDelay(400); 
      xLogger.vAddLogMsg("Connecting...");
      }
    
    if(WiFi.status() != WL_CONNECTED) {
      xLogger.vAddLogMsg("Failed to connect with status ", WiFi.status());
      vTaskDelay(60000L); 
      continue;
    } else {
      xLogger.vAddLogMsg("Connected to ", CRED_WIFI_SSID);
      strcpy(szIP, WiFi.localIP().toString().c_str());
      xLogger.vAddLogMsg("With IP ", szIP);
    }

    if(cmd.init(udp_port)) {
        xLogger.vAddLogMsg("UDP listening on ", udp_port);
      } else {
        xLogger.vAddLogMsg("Failed to init UDP socket!");
        delay(1000);
        ESP.restart();
      } 

    for (;;) {
      vTaskDelay(20);
      if(WiFi.status() != WL_CONNECTED) {
        xLogger.vAddLogMsg("Connection lost with status ", WiFi.status());
        break;
      }
      if(cmd.connected()) 
      {
        if (cmd.read()) {
          cmd.doCmd();      
          yield(); 
          cmd.respond();      
        }
      }
    }
  }
}

static void vMotionTask(void *pvParameters) {
    //int16_t val[16];
    xLogger.vAddLogMsg("Motion task started on core# ", xPortGetCoreID());    
    for (;;) { 
      vTaskDelay(200); 
      //float yaw=0;

      if(xIMU.Acquire()) {
        xIMU.process();           
        //yaw=xIMU.getYaw()*180.0 / PI;
        xIMU.getAll(ypr, NULL, NULL);
        xIMU.Release();
        //xLogger.vAddLogMsg("Yaw ", yaw);
      }
    }
}

static void vI2C_Task(void *pvParameters) {
    int16_t mpu_res=0; 
    int16_t cnt = 0;   
    esp_err_t err = esp_task_wdt_add(xTaskGetCurrentTaskHandle());
    xLogger.vAddLogMsg("I2C task started on core# ", xPortGetCoreID(), " WDT ", err); 

    for (;;) { 
      
      cnt++;
      vTaskDelay(2); 
      if(xIMU.Acquire()) {
        mpu_res = xIMU.cycle_dt();       
        xIMU.Release();
        esp_task_wdt_reset(); // reset WDT - need to check IMU err here - TODO
      } else continue;
      if(mpu_res==2) {
        // IMU settled
        fMPUReady=true;
        xLogger.vAddLogMsg("IMU settled!");    
        xTaskCreate(vMotionTask, "TaskMotion", 1024, NULL, tskIDLE_PRIORITY + 2, NULL);
        //break;
      }
      if(cnt>100) {
        // every 200 ms refresh display
        if( xSemaphoreTake( xDisplayMutex, ( TickType_t ) 0 ) == pdTRUE ) { // do not wait   
          if(fDisplayUpdated) {   
            //unsigned long t0 = xTaskGetTickCount();
            display.display();
            fDisplayUpdated = false;
            //xLogger.vAddLogMsg("Disp updated in (ms) ",  xTaskGetTickCount() - t0);
          }
          cnt=0;
          xSemaphoreGive( xDisplayMutex );
        }
      }
    }
}

void LED_task(void *pvParameter)
{
  xLogger.vAddLogMsg("Task HELLO is running on ", xPortGetCoreID());
  for(;;){ // infinite loop

      digitalWrite(led1, HIGH);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      digitalWrite(led1, LOW);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      
    }
}

void disp_task(void *pvParameter)
{
  for(;;){ // infinite loop
      // Pause the task again for 2000 ms
      vTaskDelay(100 / portTICK_PERIOD_MS);
      if( xSemaphoreTake( xDisplayMutex, ( TickType_t ) 20 ) == pdTRUE ) {
        displayUpdate();
        xSemaphoreGive( xDisplayMutex );
      }
    }
}

void setup() {
  pinMode(led1, OUTPUT);
  Serial.begin(115200);
  Wire.begin();
  //Wire1.begin(18 , 19);

  Serial.print("Tick = ");
  Serial.println(portTICK_PERIOD_MS);

  if(btStop()) {
    Serial.println(F("BT stopped"));
  } else {
    Serial.println(F("BT stop failed"));
  }

  readFS(); // test

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    //for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(500); 

  // Clear the buffer
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.display();

  byte mac[6]; 
  uint8_t i = 0;
  WiFi.macAddress(mac);

  Serial.print(F("MAC: "));

  for(i=0; i<6; i++) {
      Serial.print(mac[i],HEX);
      if(i<5) Serial.print(F(":"));
    }
  Serial.println();

  xLogger.Init();
  xIMU.init();

  xDisplayMutex = xSemaphoreCreateMutex();

  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts

  xTaskCreatePinnedToCore(vSerialOutTask,
                "TaskSO",
                1024,
                NULL,
                tskIDLE_PRIORITY + 1, // low
                NULL, 0); 

  xTaskCreatePinnedToCore(vWiFiTask,
                "TaskWiFi",
                8192,
                NULL,
                tskIDLE_PRIORITY + 2, // med
                NULL, 0); 

  xTaskCreatePinnedToCore(vI2C_Task,
                "TaskIMU",
                4096,
                NULL,
                tskIDLE_PRIORITY + 3, // max
                &MPUHandle, 1);

  xTaskCreate(&LED_task, "LED_task", 1048, NULL, tskIDLE_PRIORITY, NULL);
  xTaskCreate(&disp_task, "disp_task", 4096, NULL, tskIDLE_PRIORITY, NULL);

}

void loop() {
  // put your main code here, to run repeatedly:if(cmd.connected()) 

}

