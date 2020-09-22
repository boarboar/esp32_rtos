#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "utils/log.h"
#include "imu/mpu.h"
#include "cred.inc"

//  Hold-down the “BOOT” button in your ESP32 board
//  After you see the  “Connecting….” message in your Arduino IDE, release the finger from the “BOOT” button
//
//  SDA	SDA (default is GPIO 21)
//  SCL	SCL (default is GPIO 22) 
//  https://randomnerdtutorials.com/esp32-pinout-reference-gpios/

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

ComLogger xLogger;
MpuDrv xIMU;
xTaskHandle MPUHandle = NULL;
SemaphoreHandle_t xDisplayMutex = NULL;
boolean fMPUReady=false;
//int yaw=0;

float yaw=0;

const int led1 = 2; // Pin of the LED

void displayUpdate() {
    char buf[32];
    display.clearDisplay(); 
    display.setCursor(0,0);
    strcpy(buf, "Yaw: ");
    if(fMPUReady) {
      itoa_cat((int)yaw, buf);
      xLogger.vAddLogMsg("Yaw ", (int)yaw);
    }
    display.print(buf);
    display.setCursor(0,16);
    display.print("IP:");
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
      xLogger.vAddLogMsg("With IP ", WiFi.localIP().toString().c_str());
    }

    for (;;) {
      vTaskDelay(200);
      if(WiFi.status() != WL_CONNECTED) {
        xLogger.vAddLogMsg("Connection lost with status ", WiFi.status());
        break;
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
        yaw=xIMU.getYaw()*180.0 / PI;
        xIMU.Release();
        //xLogger.vAddLogMsg("Yaw ", yaw);
      }

    }
}

static void vI2C_Task(void *pvParameters) {
    int16_t mpu_res=0; 
    int16_t cnt = 0;   
    xLogger.vAddLogMsg("I2C task started on core# ", xPortGetCoreID());    

    for (;;) { 
      cnt++;
      vTaskDelay(2); 
      if(xIMU.Acquire()) {
        mpu_res = xIMU.cycle_dt();       
        xIMU.Release();
      } else continue;
      if(mpu_res==2) {
        // IMU settled
        fMPUReady=true;
        xLogger.vAddLogMsg("IMU settled!");    
        xTaskCreate(vMotionTask, "TaskMotion", 1024, NULL, tskIDLE_PRIORITY + 2, NULL);
        //break;
      }
      if(cnt>400) {
        // evry 400 ms refresh display
        if( xSemaphoreTake( xDisplayMutex, ( TickType_t ) 0 ) == pdTRUE ) { // do not wait
       
          unsigned long t0 = xTaskGetTickCount();
          //displayUpdate();
          display.display();
          xLogger.vAddLogMsg("Disp updated in (ms) ",  xTaskGetTickCount() - t0);
          cnt=0;
          xSemaphoreGive( xDisplayMutex );
        }
      }
    }
}


void hello_task(void *pvParameter)
{
  for(;;){ // infinite loop

      digitalWrite(led1, HIGH);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      digitalWrite(led1, LOW);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      xLogger.vAddLogMsg("Task HELLO is running on ", xPortGetCoreID());
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

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  //display.setCursor(0,0);
  //display.print("Starting...");
  //display.drawPixel(10, 10, WHITE);

  display.display();


  Serial.print("Tick = ");
  Serial.println(portTICK_PERIOD_MS);

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
  //MpuDrv::Mpu.init();
  xIMU.init();

  xDisplayMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(vSerialOutTask,
                "TaskSO",
                2048,
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
                8192,
                NULL,
                tskIDLE_PRIORITY + 3, // max
                &MPUHandle, 1);



  //xTaskCreate(&hello_task, "hello_task", 2048, NULL, tskIDLE_PRIORITY, NULL);
  xTaskCreate(&disp_task, "disp_task", 4096, NULL, tskIDLE_PRIORITY, NULL);

}

void loop() {
  // put your main code here, to run repeatedly:
}

