#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// Estrutura de dados compacta
struct DataPacket {
  uint32_t us;
  float x1, y1, z1, x2, y2, z2;
};

#define SD_CS 5
QueueHandle_t dataQueue;
File dataFile;

Adafruit_ADXL345_Unified accel1D = Adafruit_ADXL345_Unified(0x1D);
Adafruit_ADXL345_Unified accel53 = Adafruit_ADXL345_Unified(0x53);

// Matrizes de Calibração (Fieis ao seu modelo original)
const float Ainv1D[3][3] = {{0.952875, 0.006993, 0.003366}, {0.006993, 0.952172, -0.000777}, {0.003366, -0.000777, 0.987901}};
const float b1D[3]       = {0.094440, -0.361731, 0.911032};
const float Ainv53[3][3] = {{0.970592, -0.048166, 0.008696}, {-0.048166, 0.988076, -0.025318}, {0.008696, -0.025318, 0.984936}};
const float b53[3]       = {0.931123, -0.218200, 0.350909};

void setup() {
  Serial.begin(921600);
  Wire.begin();
  Wire.setClock(800000); // Clock I2C Turbo

  if(!accel1D.begin() || !accel53.begin()) {
    Serial.println("Erro ADXL!"); while(1);
  }
  
  accel1D.setDataRate(ADXL345_DATARATE_1600_HZ);
  accel53.setDataRate(ADXL345_DATARATE_1600_HZ);

  if (!SD.begin(SD_CS)) { Serial.println("Erro SD!"); while(1); }
  
  char filename[32];
  sprintf(filename, "/baja_%u.csv", millis());
  dataFile = SD.open(filename, FILE_WRITE);
  dataFile.println("us;x1;y1;z1;x2;y2;z2");

  // Fila maior (300) para aguentar picos de lentidão do SD
  dataQueue = xQueueCreate(300, sizeof(DataPacket));

  // CORE 1: Aquisição e Calibração (Alta Prioridade)
  xTaskCreatePinnedToCore(TaskAcquisition, "TaskAcq", 4096, NULL, 10, NULL, 1);
  
  // CORE 0: Escrita no SD e Serial Monitor (Baixa Prioridade)
  xTaskCreatePinnedToCore(TaskStorage, "TaskStore", 8192, NULL, 2, NULL, 0);
}

// --- CORE 1: AQUISIÇÃO E CÁLCULO ---
void TaskAcquisition(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1); // Tenta 1kHz

  for (;;) {
    DataPacket p;
    sensors_event_t ev1, ev2;
    p.us = micros();

    accel1D.getEvent(&ev1);
    accel53.getEvent(&ev2);

    // Calibração (Fiel ao seu modelo)
    float f1x = ev1.acceleration.x - b1D[0], f1y = ev1.acceleration.y - b1D[1], f1z = ev1.acceleration.z - b1D[2];
    p.x1 = Ainv1D[0][0]*f1x + Ainv1D[0][1]*f1y + Ainv1D[0][2]*f1x;
    p.y1 = Ainv1D[1][0]*f1x + Ainv1D[1][1]*f1y + Ainv1D[1][2]*f1x;
    p.z1 = Ainv1D[2][0]*f1x + Ainv1D[2][1]*f1y + Ainv1D[2][2]*f1x;

    float f2x = ev2.acceleration.x - b53[0], f2y = ev2.acceleration.y - b53[1], f2z = ev2.acceleration.z - b53[2];
    p.x2 = Ainv53[0][0]*f2x + Ainv53[0][1]*f2y + Ainv53[0][2]*f2x;
    p.y2 = Ainv53[1][0]*f2x + Ainv53[1][1]*f2y + Ainv53[1][2]*f2x;
    p.z2 = Ainv53[2][0]*f2x + Ainv53[2][1]*f2y + Ainv53[2][2]*f2x;

    xQueueSend(dataQueue, &p, 0); // Envia para o Core 0
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// --- CORE 0: SD E SERIAL ---
void TaskStorage(void *pvParameters) {
  DataPacket r;
  uint32_t count = 0;
  uint32_t timerRelatorio = millis();

  for (;;) {
    if (xQueueReceive(dataQueue, &r, portMAX_DELAY)) {
      if (dataFile) {
        dataFile.printf("%u;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f\n", r.us, r.x1, r.y1, r.z1, r.x2, r.y2, r.z2);
        count++;
      }

      // Relatório de frequência e Flush a cada 1 segundo
      if (millis() - timerRelatorio >= 1000) {
        Serial.printf("Frequência Real: %u Hz | Fila: %u\n", count, uxQueueMessagesWaiting(dataQueue));
        dataFile.flush();
        count = 0;
        timerRelatorio = millis();
      }
    }
  }
}

void loop() { vTaskDelete(NULL); }
