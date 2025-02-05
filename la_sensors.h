#ifndef _LA_SENSORS_H
#define _LA_SENSORS_H

#include <RTClib.h>

/**
 * Lectura de sensores 
**/

/* Inicializa el driver del sensor DHT22. */
void initDht22(void);

/* Recopila la última lectura de temperatura y humedad del sensor DHT22. */
void readDht22(void);

/* Inicializa el driver del sensor MHZ19B. */
void initMhz19b(void);

/* Calibra manualmente el sensor MHZ19B. */
void calibrateMhz19b(void);

/* Recopila la última lectura de co2 del sensor MHZ19B. */
void readMhz19b(void);

/* Inicializa el adc para el sensor TGS2600. */
void initTgs2600(void);

/* Recopila la última lectura cruda de metano del sensor TGS2600. */
void readRawTgs2600(void);

/* Encontrar (calibrar) un nuevo valor mínimo del sensor TGS2600. */
void findMinTgs2600(void);

/* Inicia el puerto uart para el sensor dynamite. */
void initDynamentPlatinum(void);

/* Recopila la última lectura de n2o del sensor Dynament. */
void readDynamentPlatinum(void);

/* Inicia el adc para el sensor HW-390 */
void initHw390(void);

/* Calibrar sensor de humedad del suelo HW-390. */
void calibrateHw390(void);

/* Recopila la última lectura de humedad relativa del sensor HW-390. */
void readHw390(void);

/* Inicializa los pines de conexión con el driver A4988 */
void initMotorDriver(void);

/* Mueve el motor varias revoluciones (360°)(ajustar) en sentido horario */
void moveUpMotor(void);

/* Mueve el motor varias revoluciones (360°)(ajustar) en sentido antihorario */
void moveDownMotor(void);

/* Establece el motor en operación normal */
void wakeMotor(void);

/* Establece el modo de bajo consumo (sleep) para el motor */
void putMotorToSleep(void);

/* Inicializa un pin digital para el control de ventilador */
void initFanRelay(void);

/* Prende el ventilador previo al muestreo de datos */
void turnOnFan(void);

/* Apaga el ventilador tras terminar el muestreo de datos */
void turnOffFan(void);

/* Inicio de comunicación con el reloj RTC externo */
void initRtcClock(void);

/* Establecer un tiempo específico en el reloj externo */
void setRtcTime(void);

/* Lee el tiempo actual del reloj externo */
DateTime readRtcTime(void);

/* Estabilizar las lecturas de los sensores e iniciar otros procesos necesarios */
void sensingInitialConfig(void);

/* Inicia el proceso de restauración del entorno de medición */
void initRestoreEnvProccess(void);

/* Prepara el payload de la trama LoRaWAN */
void prepareTxFrame(uint8_t port);

#endif // _LA_SENSORS_H