/**
* Punto de entrada de la app le-lora32-firmware.
* Se usan funciones de arduino que envuelven la API original de ESP-IDF.
**/

// Librerías.
#include <Arduino.h>
#include "LoRaWan_APP.h"
#include "la_lorawan.h"
#include "la_sensors.h"

/* Parámetros de la librería no usados. */
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda,0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };
uint32_t devAddr =  ( uint32_t )0x007e6ae1;

/* "Licencia" de la placa Heltec para usar las funciones de la librería del fabricante. */
uint32_t license[4] = {0x96EA4AC4, 0xF4BD4534, 0x9ED5D931, 0xBA2DA812};

/* Parámetros de la comunicación con el servidor LoRaWAN, usando OTAA y el método de join v.1.0.2. */
uint8_t devEui[] = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xDD, 0x27};
uint8_t appEui[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t appKey[] = {0x06, 0xDB, 0xC6, 0xB1, 0xC0, 0x7D, 0x51, 0xE4, 0x19, 0xC4, 0x68, 0xA2, 0x40, 0x37, 0x30, 0xF3};

/* Canales de frecuencia (Uso de los 7 primeros coincidentes con el estándar de
   parámetros regionales, empezando por 902.3 MHz con incrementos de 200 KHz entre canales. */  
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/* Selección de la región LoRaWAN para cumplir con los parámetros de la región. */
LoRaMacRegion_t loraWanRegion = LORAMAC_REGION_US915;

/* Clase del dispotivo LoRaWAN. */
DeviceClass_t  loraWanClass = CLASS_A;

/* Ciclo de trabajo de la transmisión de los datos del nodo. */
uint32_t appTxDutyCycle = 0;

/* Activación de OTAA. */
bool overTheAirActivation = true;

/* Habilitación de ADR (datarate adaptativo). */
bool loraWanAdr = true;

/* Uso de mensjaes confirmados (se requiere ACK). */
bool isTxConfirmed = true;

/* Puerto (Fport) de la aplicación LoRaWAN. */
uint8_t appPort = 14;

/* Número de intentos para transmitir la trama si la capa LoRaMAC del nodo
   no ha recibido un ACK del servidor.
  * Transmission nb | Data Rate
  * ----------------|-----------
  * 1 (first)       | DR
  * 2               | DR
  * 3               | max(DR-1,0)
  * 4               | max(DR-1,0)
  * 5               | max(DR-2,0)
  * 6               | max(DR-2,0)
  * 7               | max(DR-3,0)
  * 8               | max(DR-3,0)
*/
uint8_t confirmedNbTrials = 4;

/* Función setup para inicializar varios aspectos de la aplicación. */
void setup() {
  // 1. Puerto serial y configuración de la librería LoRa de Heltec.
  vTaskDelay(pdMS_TO_TICKS(2000));
  Serial.begin(115200);
  Mcu.setlicense(license, HELTEC_BOARD);
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  // 2. Iniciar actuadores.
  initFanRelay();
  initMotorDriver();
  // 3. Iniciar reloj rtc externo.
  initRtcClock();
  // 4. Iniciar sensores.
  initDht22();
  initMhz19b();
  initTgs2600();
  initDynamentPlatinum();
  initHw390();
  // 5. Inicialización del sistema - aprox. 4mm/4mm:30ss. */
  sensingInitialConfig();

  // Auxiliares de calibración.
  //calibrateHw390(); // Calibrar sensor capacitivo de humedad del suelo.
  //calibrateMhz19b(); // Calibrar sensor de co2.
  //setRtcTime(); // Establecer un tiempo en el reloj externo.
  //findMinTgs2600(); // Establecer un nuevo valor mínimo de mV del sensor CH4.

  // Pasamos a la función periódica Loop.
  Serial.println("----------------------------------------");
  Serial.println("Estableciendo bucle de medición y envío LoRaWAN…");
  vTaskDelay(pdMS_TO_TICKS(3000));
}

void loop()
{
  switch(deviceState)
  {
      /* Estado inicial, donde establecemos la lógica del sistema, como comprobar
      la hora y determinar si recolectar y enviar los datos, o reestablecer el sistema. */
      case DEVICE_STATE_INIT:
        Serial.println("El estado del dispositivo es DEVICE_STATE_INIT.");
        /* Revisamos si debemos restaurar el entorno */
        if (checkRestoringEnvTime())
          initRestoreEnvProccess();
        /* Comparamos el tiempo actual con los tiempo definidos para las rondas de envío,
          y esperamos al tiempo más próximo para enviar. */
        checkSendingTime();
        /* Recopilamos nuevos datos de los sensores */
        getSensorsValues();
        /* Iniciamos las configuraciones del envío LoRaWAN */
        LoRaWAN.init(loraWanClass,loraWanRegion);
        break;
      
      case DEVICE_STATE_JOIN:
        Serial.println("El estado del dispositivo es DEVICE_STATE_JOIN.");
        /* Intento de conexión/join con la red LoRaWAN den TTN */
        LoRaWAN.join();
        break;
    
      case DEVICE_STATE_SEND:
        Serial.println("El estado del dispositivo es DEVICE_STATE_SEND.");
        /* Preparación de la trama (payload) LoRaWAN */
        prepareTxFrame(appPort);
        LoRaWAN.send();
        deviceState = DEVICE_STATE_CYCLE;
        break;
      
      case DEVICE_STATE_CYCLE:
        Serial.println("El estado del dispositivo es DEVICE_STATE_CYCLE.");
        deviceState = DEVICE_STATE_SLEEP;
        break;

      case DEVICE_STATE_SLEEP:
        LoRaWAN.sleep(loraWanClass);
        break;
      
      default:  
        deviceState = DEVICE_STATE_INIT;
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
}


 
