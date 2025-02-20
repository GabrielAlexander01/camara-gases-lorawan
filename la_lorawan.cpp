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

  if (minute >= 45 and minute < 58)
    return true;
  else
    return false;
}

// Tiempos de funcionamiento
#define MIN_HOUR 5
#define MIN_MINUTE 45
#define MAX_HOUR 17
#define MAX_MINUTE 45

/* Revisar el tiempo de funcionamiento del sistema */
void checkSystemTime(void)
{
  Serial.println("Verificando funcionamiento del sistema…");
  Serial.println("Recuperando tiempo del rtc externo…");
  
  DateTime now = readRtcTime();
  uint8_t hour = now.hour();
  uint8_t minute = now.minute();
  uint8_t second = now.second();
  Serial.print("Tiempo actual: ");
  Serial.print(hour);
  Serial.print("hh");
  Serial.print(":");
  Serial.print(minute);
  Serial.print("mm");
  Serial.print(second);
  Serial.println("ss");

  while((hour <= MIN_HOUR && minute < MIN_MINUTE) || (hour >= MAX_HOUR && minute > MAX_MINUTE))
  {
    Serial.println("En espera al tiempo de ejecución del sistema…");
    vTaskDelay(pdMS_TO_TICKS(5000));

    now = readRtcTime();
    hour = now.hour();
    minute = now.minute();
    second = now.second();
    Serial.print("Tiempo actualizado: ");
    Serial.print(hour);
    Serial.print("hh");
    Serial.print(":");
    Serial.print(minute);
    Serial.print("mm");
    Serial.print(second);
    Serial.println("ss");
  }

  Serial.println("Sistema en tiempo de funcionamiento…");
}

/* Revisar el tiempo y esperar a la próxima ronda */
void checkSendingTime(void)
{
  Serial.println("Recuperando tiempo del rtc externo…");
  uint8_t minutesToSend[3] = { 15, 30, 45 };

  DateTime now = readRtcTime();
  uint8_t hour = now.hour();
  uint8_t minute = now.minute();
  uint8_t second = now.second();
  Serial.print("Tiempo actual: ");
  Serial.print(hour);
  Serial.print("hh");
  Serial.print(":");
  Serial.print(minute);
  Serial.print("mm");
  Serial.print(second);
  Serial.println("ss");

  uint8_t minuteToSend = 0;
  uint8_t secondToSend = 30;

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
  Serial.print(minuteToSend);
  Serial.print("mm");
  Serial.print(secondToSend);
  Serial.print("ss");
  
  while(readRtcTime().minute() < minuteToSend)
  {
    Serial.println("Esperando minuto de envío…");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  while(readRtcTime().second() < secondToSend)
  {
    Serial.println("Esperando segundo de envío…");
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
