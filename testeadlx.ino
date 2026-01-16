#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

struct DataPacket {
  uint32_t timestamp;
  float x1, y1, z1; 
  float x2, y2, z2; 
};

#define SD_CS 5
QueueHandle_t dataQueue;
File dataFile;
char filename[32];

Adafruit_ADXL345_Unified accel1D = Adafruit_ADXL345_Unified(0x1D);
Adafruit_ADXL345_Unified accel53 = Adafruit_ADXL345_Unified(0x53);

// Matrizes de Calibração
const float Ainv1D[3][3] = {{0.952875, 0.006993, 0.003366}, {0.006993, 0.952172, -0.000777}, {0.003366, -0.000777, 0.987901}};
const float b1D[3]       = {0.094440, -0.361731, 0.911032};
const float Ainv53[3][3] = {{0.970592, -0.048166, 0.008696}, {-0.048166, 0.988076, -0.025318}, {0.008696, -0.025318, 0.984936}};
const float b53[3]       = {0.931123, -0.218200, 0.350909};

void TaskSensor(void *pvParameters);
void TaskSD(void *pvParameters);

void aplicarCalibracao(sensors_event_t &event, const float Ainv[3][3], const float b[3], float &ox, float &oy, float &oz) {
    float f0 = event.acceleration.x - b[0];
    float f1 = event.acceleration.y - b[1];
    float f2 = event.acceleration.z - b[2];
    ox = Ainv[0][0]*f0 + Ainv[0][1]*f1 + Ainv[0][2]*f2;
    oy = Ainv[1][0]*f0 + Ainv[1][1]*f1 + Ainv[1][2]*f2;
    oz = Ainv[2][0]*f0 + Ainv[2][1]*f1 + Ainv[2][2]*f2;
}

void setup() {
  Serial.begin(921600);
  Wire.begin();
  Wire.setClock(400000); 

  // Tenta iniciar o 0x1D
  if(!accel1D.begin()) { 
    Serial.println("ERRO: Sensor 0x1D nao encontrado! Verifique se o pino SDO esta no 3.3V."); 
    while(1); 
  }
  // Tenta iniciar o 0x53
  if(!accel53.begin()) { 
    Serial.println("ERRO: Sensor 0x53 nao encontrado! Verifique se o pino SDO esta no GND."); 
    while(1); 
  }
  
  accel1D.setDataRate(ADXL345_DATARATE_1600_HZ);
  accel1D.setRange(ADXL345_RANGE_16_G);
  accel53.setDataRate(ADXL345_DATARATE_1600_HZ);
  accel53.setRange(ADXL345_RANGE_16_G);

  if (!SD.begin(SD_CS)) { Serial.println("Erro SD!"); while(1); }

  sprintf(filename, "/log_%u.csv", micros());
  dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile) {
    dataFile.println("us;x1;y1;z1;x2;y2;z2");
  }

  dataQueue = xQueueCreate(200, sizeof(DataPacket));

  if (dataQueue != NULL) {
    xTaskCreatePinnedToCore(TaskSensor, "TaskSensor", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(TaskSD, "TaskSD", 8192, NULL, 2, NULL, 0);
  }
}

void TaskSensor(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1); 

  for (;;) {
    DataPacket packet;
    sensors_event_t ev1, ev2;
    packet.timestamp = micros();
    
    accel1D.getEvent(&ev1);
    accel53.getEvent(&ev2);
    
    aplicarCalibracao(ev1, Ainv1D, b1D, packet.x1, packet.y1, packet.z1);
    aplicarCalibracao(ev2, Ainv53, b53, packet.x2, packet.y2, packet.z2);

    xQueueSend(dataQueue, &packet, 0);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void TaskSD(void *pvParameters) {
  DataPacket rec;
  int flushCounter = 0;
  int printCounter = 0;

  for (;;) {
    if (xQueueReceive(dataQueue, &rec, portMAX_DELAY) == pdPASS) {
      
      // Print Serial: Imprime apenas 1 a cada 20 amostras para nao travar o monitor
      // (Isso mostra os dados a cada 20ms, o que é legível para humanos)
      if (++printCounter >= 20) {
        Serial.printf("S1: %.2f %.2f %.2f | S2: %.2f %.2f %.2f\n", 
                       rec.x1, rec.y1, rec.z1, rec.x2, rec.y2, rec.z2);
        printCounter = 0;
      }

      if (dataFile) {
        dataFile.printf("%u;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f\n", 
                        rec.timestamp, rec.x1, rec.y1, rec.z1, rec.x2, rec.y2, rec.z2);
        
        if (++flushCounter >= 500) {
          dataFile.flush();
          flushCounter = 0;
        }
      }
    }
  }
}

void loop() { vTaskDelete(NULL); }
