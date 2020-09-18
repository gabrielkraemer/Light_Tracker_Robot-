#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include "Servo.h"

// TASKS

/** This task reads the LDR sensors */
void producer();
/** This task calculates the each LDR's Lux */
void consumer();
/** This task determinates servo's angle */
void servo();


// Handles to suspend or resume tasks
TaskHandle_t producer_Handle;
TaskHandle_t consumer_Handle;
TaskHandle_t servo_Handle;

SemaphoreHandle_t LDR0_mutex; ///< LDR0's mutex
SemaphoreHandle_t LDR1_mutex; ///< LDR1's mutex

SemaphoreHandle_t LUX0_mutex; ///< LUX0's mutex
SemaphoreHandle_t LUX1_mutex; ///< LUX1's mutex

// Criate a Servo Object
Servo servo1;                 ///< Servo Objetc


float LUX0, LUX1;

float LDR0, LDR1;

// the setup function runs once when you press reset or power the board
void setup() {

  // BOARD configuration
  servo1.attach(6);     //make analog pin D6 as servo output
  servo1.write(90);
  
  pinMode(A5,INPUT);    //make analog pin A5 as input
  pinMode(A0,INPUT);    //make analog pin A0 as input 
  
  Serial.begin(9600);   //initialize serial monitor
  Serial.println("loading OS...");
  Serial.println(" ");

  // OS configuration
  LUX0_mutex = xSemaphoreCreateMutex();
  LUX1_mutex = xSemaphoreCreateMutex();
  
  LDR0_mutex = xSemaphoreCreateMutex();
  LDR1_mutex = xSemaphoreCreateMutex();
  
  // Now set up the tasks
  xTaskCreate( producer,  
               "producer",  
               128,                             // This stack size can be checked & adjusted by reading Highwater
               NULL,
               3,                               // priority
               &producer_Handle );

  xTaskCreate( consumer,  
               "consumer",  
               128,                             // This stack size can be checked & adjusted by reading Highwater
               NULL,
               2,                               // priority
               &consumer_Handle );

  xTaskCreate( servo,  
               "servo",  
               128,                             // This stack size can be checked & adjusted by reading Highwater
               NULL,
               1,                               // priority
               &servo_Handle );
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
  vTaskStartScheduler();
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/



void producer ()
{
  float x;
  
  while (true) {
                  
      Serial.println("TASK 1");
      
      x = analogRead(A0);         // Read LDR0 sensor
       
      if( xSemaphoreTake( LDR0_mutex , ( TickType_t ) 5 ) == pdTRUE  ){
          
        LDR0 = x;

        xSemaphoreGive( LDR0_mutex );
        
      }
    
      x = analogRead(A5);         // Read LDR1 sensor
      
      if( xSemaphoreTake( LDR1_mutex , ( TickType_t ) 5 ) == pdTRUE  ){
          
        LDR1 = x;

        xSemaphoreGive( LDR1_mutex );
        vTaskSuspend( producer_Handle );
        
     }   
  }
      
}

void consumer() 
{
    float ADC_value=0.00488, read_LDR;
      
    while (true) {
        
        Serial.println("TASK 2");      

        if( xSemaphoreTake( LDR0_mutex , ( TickType_t ) 5 ) == pdTRUE  ){
            
          read_LDR = LDR0;

          xSemaphoreGive( LDR0_mutex );

          if( xSemaphoreTake( LUX0_mutex , ( TickType_t ) 5 ) == pdTRUE  ){   
            
            LUX0=(250.000/(ADC_value*( read_LDR )))-50.000;                   // Calculates LDR0's LUX
            xSemaphoreGive( LUX0_mutex );
              
          }
        }

        if( xSemaphoreTake( LDR1_mutex , ( TickType_t ) 5 ) == pdTRUE  ){
            
          read_LDR = LDR1;

          xSemaphoreGive( LDR1_mutex );

          if( xSemaphoreTake( LUX1_mutex , ( TickType_t ) 5 ) == pdTRUE  ){   
            
            LUX1=(250.000/(ADC_value*( read_LDR )))-50.000;                  // Calculates LDR1's LUX
            xSemaphoreGive( LUX1_mutex );
              
          }
        }
        vTaskSuspend( consumer_Handle );
        vTaskResume( producer_Handle );
    }
}

void servo(){
  int pos;

  pos = 0;

  while (true) {
    
    Serial.println("TASK 3");
    
    if( xSemaphoreTake( LUX0_mutex , ( TickType_t ) 5 ) == pdTRUE  ){   
      
      if( xSemaphoreTake( LUX1_mutex , ( TickType_t ) 5 ) == pdTRUE  ){   

        if(LUX0 < LUX1 ){
          
          if(pos < 135){
            pos++;
          }
          
        }else{
          
          if(pos > 45){
            pos--;
          }
          
        }
      }            
      
    }
      
    servo1.write(pos);
    Serial.print("valor do LUX0: ");
    Serial.println(LUX0);
    Serial.print("valor do LUX1: ");
    Serial.println(LUX1);
    
    xSemaphoreGive( LUX0_mutex );
    xSemaphoreGive( LUX1_mutex );
     
    Serial.print("valor do angulo: ");
    Serial.println(pos);
    vTaskDelay(1);

    vTaskResume( consumer_Handle );
    
  }
  
}
