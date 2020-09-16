#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "utils/log.h"
#include "imu/mpu.h"

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
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);

ComLogger xLogger;
xTaskHandle MPUHandle;
boolean fMPUReady=false;
//int yaw=0;

float yaw=0;

const int led1 = 2; // Pin of the LED

static void vSerialOutTask(void *pvParameters) {
    Serial.print("Serial Out Task started on core# ");
    Serial.println(xPortGetCoreID());
    for (;;) {
       xLogger.Process();
       vTaskDelay(20);
    }
}

static void vMotionTask(void *pvParameters) {
    //int16_t val[16];
    xLogger.vAddLogMsg("Motion Task started.");    
    for (;;) { 
      vTaskDelay(2000); 
      //float yaw=0;

      if(MpuDrv::Mpu.Acquire()) {
        MpuDrv::Mpu.process();           
        yaw=MpuDrv::Mpu.getYaw()*180.0 / PI;
        MpuDrv::Mpu.Release();
        xLogger.vAddLogMsg("Yaw ", yaw);
      }

    }
}

static void vIMU_Task(void *pvParameters) {
    int16_t mpu_res=0;    
    xLogger.vAddLogMsg("IMU Task started.");
    for (;;) { 
      vTaskDelay(2); 
      if(MpuDrv::Mpu.Acquire()) {
        mpu_res = MpuDrv::Mpu.cycle_dt();       
        MpuDrv::Mpu.Release();
      } else continue;
      if(mpu_res==2) {
        // IMU settled
        fMPUReady=true;
        xLogger.vAddLogMsg("IMU settled!");    
        xTaskCreate(vMotionTask,
                "TaskMotion",
                1024,
                NULL,
                tskIDLE_PRIORITY + 2, // med
                NULL);
    
        //break;
      }
    }
    // xLogger.vAddLogMsg("IMU Task completed.");
    
    // xTaskCreate(vMotionTask,
    //             "TaskMotion",
    //             1024,
    //             NULL,
    //             tskIDLE_PRIORITY + 2, // med
    //             NULL);

    // vTaskDelete(MPUHandle);

}



void hello_task(void *pvParameter);
void disp_task(void *pvParameter);

void setup() {
  pinMode(led1, OUTPUT);
  Serial.begin(115200);
  Wire.begin();
  Wire1.begin(18 , 19);

  delay(2000);

  Serial.print("Tick = ");
  Serial.println(portTICK_PERIOD_MS);
    
  xLogger.Init();
  MpuDrv::Mpu.init();

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
  display.setCursor(0,0);
  //display.print("Starting...");

  //display.drawPixel(10, 10, WHITE);

  display.display();

  delay(2000); // Pause for 2 seconds

  xTaskCreate(vSerialOutTask,
                "TaskSO",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1, // low
                NULL); 

  xTaskCreate(vIMU_Task,
                "TaskIMU",
                1024,
                NULL,
                tskIDLE_PRIORITY + 3, // max
                &MPUHandle);


  xTaskCreate(&hello_task, "hello_task", 2048, NULL, tskIDLE_PRIORITY, NULL);
  xTaskCreate(&disp_task, "disp_task", 2048, NULL, tskIDLE_PRIORITY, NULL);

}

void loop() {
  // put your main code here, to run repeatedly:
}

void hello_task(void *pvParameter)
{
  for(;;){ // infinite loop

      // Turn the LED on
      digitalWrite(led1, HIGH);

      // Pause the task for 500ms
      vTaskDelay(500 / portTICK_PERIOD_MS);

      // Turn the LED off
      digitalWrite(led1, LOW);

      // Pause the task again for 500ms
      vTaskDelay(500 / portTICK_PERIOD_MS);

      //Serial.print("Task HELLO is running on: ");
      //Serial.println(xPortGetCoreID());

      xLogger.vAddLogMsg("Task HELLO is running on ", xPortGetCoreID());
    }
}

void disp_task(void *pvParameter)
{
  char buf[32];
  for(;;){ // infinite loop
      // Pause the task again for 2000 ms
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      strcpy(buf, "Yaw: ");
      //itoa(yaw, buf);

      // display.clearDisplay(); 
      // display.setCursor(0,0);
      // display.print(buf);
      // display.display();

      //Serial.print("Task DISP is running on: ");
      //Serial.println(xPortGetCoreID());
      //xLogger.vAddLogMsg("Task DISP is running on ", xPortGetCoreID());

        //xLogger.vAddLogMsg("Yaw ", yaw);

    }
}