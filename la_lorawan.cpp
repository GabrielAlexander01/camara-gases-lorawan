/**
 * Archivo fuente para la comunicación con el servidor LoRaWAN.
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <RTClib.h>
#include "la_sensors.h"
#include "LoRaWan_APP.h"

// Cantidad de muestras a tomar antes del envío.
#define DATA_SAMPLES 5

/* Revisar el tiempo del sistema para la restauración del ambiente. */
bool checkRestoringEnvTime(void)
{
  Serial.println("Recuperando tiempo del rtc externo…");
  DateTime now = readRtcTime();
  uint8_t hour = now.hour();
  uint8_t minute = now.minute();
  Serial.print("Tiempo actual: ");
  Serial.print(hour);
  Serial.print("hh:");
  Serial.print(minute);
  Serial.println("mm");
  Serial.println("----------------------------------------");

  if (minute >= 45)
    return true;
  else
    return false;
}

/* Revisar el tiempo y esperar a la próxima ronda */
void checkSendingTime(void)
{
  Serial.println("Recuperando tiempo del rtc externo…");
  uint8_t minutesToSend[3] = { 15, 30, 45 };

  DateTime now = readRtcTime();
  uint8_t hour = now.hour();
  uint8_t minute = now.minute();
  Serial.print("Tiempo actual: ");
  Serial.print(hour);
  Serial.print("hh");
  Serial.print(":");
  Serial.println(minute);
  Serial.print("mm");
  
  uint8_t minuteToSend = 0;

  if (minute >= minutesToSend[2])
    minuteToSend = 59; // minuto de envío próxima ronda (0mm).
  else if (minute >= minutesToSend[1])
    minuteToSend = 44; // minuto de envío próxima ronda (45mm).
  else if (minute >= minutesToSend[0])
    minuteToSend = 29; // minuto de envío próxima ronda (30mm).
  else 
    minuteToSend = 14; // minuto de envío próxima ronda (15mm).

  Serial.print("Próximo tiempo de envío dispuesto: ");
  Serial.print(hour);
  Serial.print("hh");
  Serial.print(":");
  Serial.println(minuteToSend);
  Serial.print("mm");
  
  while(readRtcTime().minute() < minuteToSend)
  {
    Serial.println("Esperando minuto de envío…");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  Serial.println("Se llegó al tiempo de envío.");
}

/* Recopilación de datos de los sensores. */
void getSensorsValues(void)
{
  Serial.println("Recopilando datos de sensores…");
  for (int i=0; i<DATA_SAMPLES; i++)
  {
    readDht22();
    readMhz19b(); 
    readRawTgs2600();
    readDynamentPlatinum();
    readHw390();
    vTaskDelay(pdMS_TO_TICKS(5000));
  }     
}
