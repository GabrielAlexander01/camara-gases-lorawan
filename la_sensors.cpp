/**
 * Archivo fuente para la lectura de sensores, control de actuadores y preparación de datos.
 */

#include <Arduino.h>
#include <cstring>
#include <HardwareSerial.h>
#include <esp32-hal-adc.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <MHZ19.h>
#include <RTClib.h>
#include <Wire.h>
#include "LoRaWan_APP.h"

/* Variables de medición formateadas para envío. */
uint8_t nodeId = 1;
int8_t innerLoraTemperature = 0;
uint8_t innerLoraHumidity = 0;
int8_t outerLoraTemperature = 0;
uint8_t outerLoraHumidity = 0;
uint8_t soilLoraMoisture = 0;
uint16_t innerLoraCo2 = 0;
float innerLoraCh4 = 0;
int16_t innerLoraN2o = 0;

/* Resumen de los pines gpio utilizados.
  GPIO19-VENTILADOR-DIGITAL-LISTO
  GPIO20-DHT22-INTERIOR-DIGITAL-LISTO
  GPIO26-DHT22-EXTERIOR-DIGITAL-LISTO
  GPIO46-ESPRX-MHZ19BTX-UART-LISTO
  GPIO45-ESPTX-MHZ19BRX-UART-LISTO
  GPIO41-SDARTC-I2C-LISTO
  GPIO42-SCLRTC-I2C-LISTO
  GPIO40-ESPRX-DynamiteTX-UART
  GPIO39-ESPTX-DynamiteRX-UART
  GPIO3-DIRECCIÓN-MOTOR-DIGITAL-LISTO
  GPIO4-PASO-MOTOR-DIGITAL-LISTO
  GPI7O5-EN-MOTOR-DIGITAL-LISTO
  GPIO6-TGS2600-ANALOG-ADC1-CHN5-LISTO
  GPIO7-HW-390-ANALOG-ADC1-CHN6-LISTO
*/

//////////////////////////////////////////////////////////////////////////////////////////////////

#define IDHT_PIN 20 // DHT22 interno
#define ODHT_PIN 26 // DHT22 externo
#define DHT_TYPE DHT22    

DHT_Unified innerDht22(IDHT_PIN, DHT_TYPE);
DHT_Unified outerDht22(ODHT_PIN, DHT_TYPE);

/* Inicializa el driver del sensor DHT22. */
void initDht22(void)
{
  innerDht22.begin();
  outerDht22.begin();

  sensor_t innerSensor;
  innerDht22.temperature().getSensor(&innerSensor);
  Serial.println("----------------------------------------");
  Serial.println("Iniciando sensores DHT22…");
  Serial.println("Rangos de temperatura.");
  Serial.print("Valor máximo:   "); Serial.print(innerSensor.max_value); Serial.println("°C");
  Serial.print("Valor mínimo:   "); Serial.print(innerSensor.min_value); Serial.println("°C");
  
  innerDht22.humidity().getSensor(&innerSensor);
  Serial.println("Rangos de humedad.");
  Serial.print("Valor máximo:   "); Serial.print(innerSensor.max_value); Serial.println("%");
  Serial.print("Valor mínimo:   "); Serial.print(innerSensor.min_value); Serial.println("%");
}

/* Recopila la última lectura de temperatura y humedad del sensor DHT22. */
void readDht22(void)
{
  sensors_event_t innerEvent;
  sensors_event_t outerEvent;

  innerDht22.temperature().getEvent(&innerEvent);
  if (isnan(innerEvent.temperature))
  {
    Serial.println("¡Error leyendo temperatura!");
    innerLoraTemperature = 0;
  }
  else
  {
    innerLoraTemperature = innerEvent.temperature;
    Serial.print("Temperatura interior: ");
    Serial.print(innerLoraTemperature);
    Serial.println("°C");
  }

  innerDht22.humidity().getEvent(&innerEvent);
  if (isnan(innerEvent.relative_humidity))
  {
    Serial.println("¡Error leyendo humedad!");
    innerLoraHumidity = 0;
  }
  else
  {
    innerLoraHumidity = innerEvent.relative_humidity;
    Serial.print("Humedad interior: ");
    Serial.print(innerLoraHumidity);
    Serial.println("%");
  }

  outerDht22.temperature().getEvent(&outerEvent);
  if (isnan(outerEvent.temperature))
  {
    Serial.println("¡Error leyendo temperatura!");
    outerLoraTemperature = 0;
  }
  else 
  {
    outerLoraTemperature = outerEvent.temperature;
    Serial.print("Temperatura exterior: ");
    Serial.print(outerLoraTemperature);
    Serial.println("°C");
  }

  outerDht22.humidity().getEvent(&outerEvent);
  if (isnan(outerEvent.relative_humidity))
  {
    Serial.println("¡Error leyendo humedad!");
    outerLoraHumidity = 0;
  }
  else
  {
    outerLoraHumidity = outerEvent.relative_humidity;
    Serial.print("Humedad exterior: ");
    Serial.print(outerLoraHumidity);
    Serial.println("%");
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////

#define MHZ_RX_PIN 46 // RxMcu-TxMhz19b
#define MHZ_TX_PIN 45 // TxMcu-RxMhz19b
#define MHZ_UART_PORT 1 // Puerto Uart para la comunicación
#define MHZ_BAUDRATE 9600 // Baudrate Uart para la comunicación entre Mcu-Mhz19
#define MHZ_PREHEAT_TIME 18e4 // 3 min en ms
#define MHZ_CALIBRATION_TIME 12e5 // 20 min en ms

MHZ19 innerMhz19; 
HardwareSerial mhzSerial(MHZ_UART_PORT);

/* Inicializa el driver del sensor MHZ19B. */
void initMhz19b(void)
{
  Serial.println("----------------------------------------");
  Serial.println("Iniciando sensor MHZ19B…");

  mhzSerial.begin(MHZ_BAUDRATE, SERIAL_8N1, MHZ_RX_PIN, MHZ_TX_PIN); 
  innerMhz19.begin(mhzSerial);

  char mhzVersion[4];      
  innerMhz19.autoCalibration(false);

  innerMhz19.getVersion(mhzVersion);
  Serial.print("Versión del firmware: ");
  for(byte i = 0; i < 4; i++)
  {
    Serial.print(mhzVersion[i]);
    if(i == 1)
      Serial.print(".");    
  }
   Serial.println("");    
   Serial.print("Rango de Co2: ");
   Serial.print(innerMhz19.getRange());  
   Serial.println(" ppm.");
}

/* Calibra manualmente el sensor MHZ19B. */
void calibrateMhz19b(void)
{
  Serial.println("Precalentando 3 minutos…");
  vTaskDelay(pdMS_TO_TICKS(MHZ_PREHEAT_TIME));
  Serial.println("Sensor Mhz19b precalentado.");
  Serial.println("Esperando 20 minutos para recopilar muestras…");
  vTaskDelay(pdMS_TO_TICKS(MHZ_CALIBRATION_TIME));
  Serial.println("Muestras recogidas.");

  Serial.println("Calibrando sensor manualmente…");
  innerMhz19.calibrate(); 
  Serial.println("Nuevo punto de referencia, sensor calibrado.");
}

/* Recopila la última lectura de co2 del sensor MHZ19B. */
void readMhz19b(void)
{
  if (!innerMhz19.getCO2())
  {
    Serial.println("¡Error leyendo co2!");
    innerLoraCo2 = 0;      
  }
  else
  {
    innerLoraCo2 = innerMhz19.getCO2();      
    Serial.print("Co2 interior: ");
    Serial.print(innerLoraCo2);
    Serial.println(" ppm.");  
  }               
}

//////////////////////////////////////////////////////////////////////////////////////////////////

/*
  MÓDULO SENSOR TGS2600 PARA MEDICIONES DE GAS METANO CH4 EN PPM.
  
  Se necesita conocer la caída de voltaje en una resistencia de carga puesta en serie al sensor.
  A tener en cuenta que al usar el MÓDULO (no chip) TGS2600, esto ya debería estar incluido, por
  lo que el proceso para obtener VRL sería leer el voltaje de AOUT con un ADC.
  VRL = Vc * (RL / (RL+RS)) Divisor convencional.

  Sin embargo, es importante considerar lo siguiente al medir el voltaje en el resistor de carga. El sensor
  utiliza lógica de 5v, mientras que el controlador LoRa ESP32 usa lógica de 3.3v. Esto es un problema,
  pues necesitamos reducir el voltaje de entrada al ADC del ESP32, lo que altera todas nuestras
  mediciones. ¿Qué debemos hacer?
  A la salida analógica de la resistencia de carga, la que se presupone es el voltaje medido en el pin
  analógico del MÓDULO (no chip) TGS2600, debemos crear un nuevo divisor de voltaje. Lo ideal es 
  crear uno donde ambas resistencias sean iguales para dividir el voltaje a la mitad.
  Por ejemplo, se planea disponer dos resistores de 10kohm a la salida analógica. Uno en serie con la 
  entrada al ADC del ucontrolador, y otro en la misma línea puesto a tierra. Con esto, deberíamos obtener la
  mitad a la entrada, es decir, si la lectura original de VRL fuése 5V, tendríamos 2.5V en el ESP32.
  Por ello, al considerar leer VRL, necesitamos un multiplicador (x2) en software para aproximar
  el valor original. Entonces, VRL = Vadc * 2.

  Para obtener RL (453 medidos) debemos revisar el módulo cuando se entregue, pues el datasheet
  no incluye esta inf. 
 */

#define TGS_ADC_PIN 6 // gpio6-adc1-chan5
#define TGS_ADC_RESOLUTION 12 // 12 bits 0-4095
#define LOAD_RESISTOR 453 // ohms
#define ZERO_RESISTOR 31904 // ohms.
const float X_MIN = 0.850; // valor mínimo de rs/ro, calculado en funcion de la gráfica característica 0-100 ppm.
const float X_MAX = 1.469; // valor máximo de rs/ro, calculado en funcion de la gráfica característica 0-100 ppm.
const float A = 3.922237; // Constantes de la función de aproximación exponencial-log.
const float B = -8.418636;

/* Inicializa el adc para el sensor TGS2600. */
void initTgs2600(void) {
  Serial.println("----------------------------------------");
  Serial.println("Iniciando sensor TGS2600…");
  pinMode(TGS_ADC_PIN, INPUT);
  analogReadResolution(TGS_ADC_RESOLUTION);
}

// Helper para mapeo de valores decimales en dos rangos distintos.
float mapFloat(float x, float inMin, float inMax, float outMin, float outMax) {
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

/* Recopila la última lectura cruda de metano del sensor TGS2600. */
void readRawTgs2600(void)
{
    int32_t digitalVrl = (analogReadMilliVolts(TGS_ADC_PIN) * 2);
    Serial.print("Voltaje medido actual: "); Serial.print(digitalVrl); Serial.println(" mV.");
    int32_t sensorResistance = ((5000.0 * LOAD_RESISTOR) / digitalVrl) - LOAD_RESISTOR;
    Serial.print("Resistencia Rs actual: "); Serial.println(sensorResistance);
    float sensorToReferenceRatio = (float)sensorResistance / (float)ZERO_RESISTOR;
    Serial.print("Ratio Rs/Ro actual es de: "); Serial.println(sensorToReferenceRatio);

    if (sensorToReferenceRatio >= 1)
      innerLoraCh4 = 1;
    else
      innerLoraCh4 = mapFloat(digitalVrl, 50, 5000, 1.0, 100.0); // máximo medido: 3886 mV.

    Serial.print("CH4 (metano): "); Serial.print(innerLoraCh4); Serial.println(" ppm.");
}
 
/* Encontrar (calibrar) un nuevo valor mínimo del sensor TGS2600. */
void findMinTgs2600(void) {
  while (true)
  {
    int32_t digitalVrl = (analogReadMilliVolts(TGS_ADC_PIN) * 2);
    Serial.print("Voltaje medido actual: "); Serial.print(digitalVrl); Serial.println(" mV.");
    int32_t sensorResistance = ((5000.0 * LOAD_RESISTOR) / digitalVrl) - LOAD_RESISTOR;
    Serial.print("Resistencia Rs actual: "); Serial.println(sensorResistance);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////

#define DPLI_RX_PIN 40 // RxMcu-TxDynamite
#define DPLI_TX_PIN 39 // TxMcu-RxDynamite
#define DPLI_UART_PORT 2 // Puerto Uart para la comunicación
#define DPLI_BAUDRATE 38400 // Baudrate Uart para la comunicación entre Mcu-Dynamite

HardwareSerial dynamentSerial(DPLI_UART_PORT);

/* Inicia el puerto uart para el sensor dynamite. */
void initDynamentPlatinum(void)
{
  Serial.println("----------------------------------------");
  Serial.println("Iniciando sensor Dynament Platinum…");
  dynamentSerial.begin(DPLI_BAUDRATE, SERIAL_8N1, DPLI_RX_PIN, DPLI_TX_PIN); 
}

/* Recopila la última lectura de n2o del sensor Dynament. */
void readDynamentPlatinum(void)
{
  uint8_t preDataBytes = 7; // 7 bytes antes del payload.
  uint8_t postDataBytes = 12; // byte número 12 después del payload.
  int8_t dataArray[4] = {0}; // 4 bytes de datos (float).
  uint8_t currentByte = 0; // Índice del byte en proceso.

  // Enviar código uart para recuperar/pedir última lectura de n2o.
  Serial.println("Enviando código dynamite para datos…");
  dynamentSerial.write(0x10);
  dynamentSerial.write(0x13);
  dynamentSerial.write(0x06);
  dynamentSerial.write(0x10);
  dynamentSerial.write(0x1F);
  dynamentSerial.write(0x00);
  dynamentSerial.write(0x58);

  Serial.println("Esperando respuesta de dynamite…");
  while (dynamentSerial.available())
  {
    currentByte++;
    if (currentByte > preDataBytes && currentByte < postDataBytes)
    {
      dataArray[currentByte-8] = dynamentSerial.read();
      Serial.print(dataArray[currentByte-8], HEX);
      Serial.print("/");
    }
    else
    {
      Serial.print(dynamentSerial.read(), HEX);
      Serial.print("|");
    }
  }
  Serial.println();
  Serial.println("Respuesta recuperada en hexadecimal, procesando datos…");

  float resultN2o = 0.;
  std::memcpy(&resultN2o, dataArray, sizeof(resultN2o));
  Serial.print("N2o interior (float): ");
  Serial.print(resultN2o);
  Serial.println(" ppm.");
  innerLoraN2o = (int16_t)resultN2o;
  Serial.print("N2o interior (int): ");
  Serial.print(innerLoraN2o);
  Serial.println(" ppm.");
}

/////////////////////////////////////////////////////////////////////////////////////////////////

#define HW_ADC_PIN 7 // gpio7-adc1-chan6
#define HW_ADC_RESOLUTION 12 // 12 bits 0-4095
#define HW_WATER_VALUE 2400 // Valor máximo-Sensor húmedo-100%
#define HW_DRY_VALUE 3147 // Valor mínimo-Sensor seco-0%

/* Inicia el adc para el sensor HW-390 */
void initHw390(void) {
  Serial.println("----------------------------------------");
  Serial.println("Iniciando sensor HW-390…");
  pinMode(HW_ADC_PIN, INPUT);
  analogReadResolution(HW_ADC_RESOLUTION);
}

/* Calibrar sensor de humedad del suelo HW-390. */
void calibrateHw390(void) {
  Serial.println("Calibrando sensor de humedad de suelo HW-390…");
  while(true){
    int rawSoilValue = analogReadMilliVolts(HW_ADC_PIN);
    Serial.println(rawSoilValue);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

/* Recopila la última lectura de humedad relativa del sensor HW-390. */
void readHw390(void) {
  int16_t rawSoilValue = analogReadMilliVolts(HW_ADC_PIN);
  soilLoraMoisture = map(rawSoilValue, HW_DRY_VALUE, HW_WATER_VALUE, 0, 100);
  Serial.print("Humedad del suelo: ");
  Serial.print(soilLoraMoisture);
  Serial.println(" %.");
}

/////////////////////////////////////////////////////////////////////////////////////////////////

#define MOTOR_EN_PIN 5 // pin apagar motor
#define MOTOR_STEP_PIN 4 // pin pasos del motor
#define MOTOR_DIRECTION_PIN 3 // pin dir del motor
#define STEPS_PER_REV 200 // pasos a dar por completar una vuelta/revolución (360°) 
#define REVS_TO_UP 10 // número de vueltas/revoluciones en subida (360°)
#define REVS_TO_DOWN 10 // número de vueltas/revoluciones en bajada (360°)

/* Inicializa los pines para el control del motor. */
void initMotorDriver(void) {
  Serial.println("----------------------------------------");
  Serial.println("Iniciando control de motor nema…");
  pinMode(MOTOR_STEP_PIN, OUTPUT);
  pinMode(MOTOR_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_EN_PIN, OUTPUT);
  digitalWrite(MOTOR_EN_PIN, HIGH);
}

/* Mueve el motor varias revoluciones (360°)(ajustar) en sentido horario. */
void moveUpMotor(void) {
  Serial.println("----------------------------------------");
  Serial.println("Moviendo motor en sentido horario, levantando…");
  digitalWrite(MOTOR_DIRECTION_PIN, HIGH);

  for (int i=0; i<REVS_TO_UP; i++)
  {
    for(int i = 0; i<STEPS_PER_REV; i++)
    {
      digitalWrite(MOTOR_STEP_PIN, HIGH);
      delayMicroseconds(1000);
      digitalWrite(MOTOR_STEP_PIN, LOW);
      delayMicroseconds(1000);
    }
    delay(100); // Este delay permite aumentar o disminuir la velocidad de las vueltas del motor.
  }
}

/* Mueve el motor varias revoluciones (360°)(ajustar) en sentido antihorario. */
void moveDownMotor(void) {
  Serial.println("----------------------------------------");
  Serial.println("Moviendo motor en sentido antihorario, cerrando…");
  digitalWrite(MOTOR_DIRECTION_PIN, LOW);

  for (int i=0; i<REVS_TO_DOWN; i++)
  {
    for(int i = 0; i<STEPS_PER_REV; i++)
    {
      digitalWrite(MOTOR_STEP_PIN, HIGH);
      delayMicroseconds(1000);
      digitalWrite(MOTOR_STEP_PIN, LOW);
      delayMicroseconds(1000);
    }
    delay(100); // Este delay permite aumentar o disminuir la velocidad de las vueltas del motor.
  }
}

/* Establece el motor en operación normal. */
void wakeMotor(void) {
  Serial.println("----------------------------------------");
  Serial.println("Estableciendo motor en operación regular…");
  digitalWrite(MOTOR_EN_PIN, LOW);
}

/* Establece el modo de bajo consumo para el motor. */
void putMotorToSleep(void) {
  Serial.println("----------------------------------------");
  Serial.println("Durmiendo motor, desactivando FETs…");
  digitalWrite(MOTOR_EN_PIN, HIGH);
}

//////////////////////////////////////////////////////////////////////////////////////////////////

#define FAN_RELAY_PIN 19

/* Iniciar pin digital para el control de ventilador. */
void initFanRelay(void)
{
  Serial.println("----------------------------------------");
  Serial.println("Iniciando control del ventilador…");
  pinMode(FAN_RELAY_PIN, OUTPUT);
  digitalWrite(FAN_RELAY_PIN, HIGH);
}

/* Enciende el ventilador. */
void turnOnFan(void)
{
  Serial.println("----------------------------------------");
  Serial.println("Encendiendo ventilador…");
  digitalWrite(FAN_RELAY_PIN, LOW);
}

/* Apaga el ventilador. */
void turnOffFan(void)
{
  Serial.println("----------------------------------------");
  Serial.println("Apagando ventilador…");
  digitalWrite(FAN_RELAY_PIN, HIGH);
}

/////////////////////////////////////////////////////////////////////////////////////////////////

#define RTC_SDA_PIN 41 // pines comunicación i2c con módulo rtc.
#define RTC_SCL_PIN 42 

RTC_DS3231 loraRtc;

/* Iniciar comunicación i2c con el reloj rtc externo. */
void initRtcClock(void)
{
  Serial.println("----------------------------------------");
  Serial.println("Iniciando reloj RTC DS3231…");
  Wire1.begin(RTC_SDA_PIN, RTC_SCL_PIN);

  while (!loraRtc.begin(&Wire1)) {
    Serial.println("Reintentando conexión al reloj RTC…");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/* Establecer un tiempo específico en el reloj externo. */
void setRtcTime(void)
{
  Serial.println("Estableciendo una hora y fecha en el reloj RTC…");
  vTaskDelay(pdMS_TO_TICKS(10000));  // Tiempo antes de establecer un nuevo tiempo para evitar reasignación.
  loraRtc.adjust(DateTime(2025, 1, 31, 20, 22, 0)); // Establecer el tiempo en el formato: Año/Mes/Día/Hora/Minuto/Segundo
  Serial.println("Fecha y hora ajustadas.");
}

/* Lee el tiempo del reloj externo. */
DateTime readRtcTime(void)
{
  return loraRtc.now();
}

/////////////////////////////////////////////////////////////////////////////////////////////////

#define NORMALIZE_TIME 6e4 // 1 min en ms
#define PREHEAT_TIME 18e4 // 3 min en ms
#define VENTILATE_TIME 6e4 // Tiempo de ventilación inicial (1min).
bool initialFan = false; // Activar la ventilación inicial.

/* Configurar por única vez el sistema. */
void sensingInitialConfig(void)
{
  uint32_t currentTime = millis();
  bool alredyVent = false;

  Serial.println("----------------------------------------");
  Serial.println("Configurando el sistema, inicio único…");
  Serial.println("Precalentando sensores, 3 min… (MHZ19B, TGS2600, Dynament)");

  if (initialFan)
  {
  Serial.println("Ventilando el entorno, 1 min…");
  turnOnFan();
  }


  while (millis() - currentTime < PREHEAT_TIME)
  {
    if (initialFan && millis() - currentTime > VENTILATE_TIME && !alredyVent)
    {
      turnOffFan();
      Serial.println("----------------------------------------");
      Serial.println("Cámara ventilada.");
      alredyVent = true;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  Serial.println("Sensores precalentados.");

  Serial.println("----------------------------------------");
  Serial.println("Normalizando muestras de sensores, 1 min…");

  currentTime = millis();
  while (millis() - currentTime < NORMALIZE_TIME)
  {
    readDht22();
    readMhz19b(); 
    readDynamentPlatinum();
    readRawTgs2600();
    readHw390();
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
  Serial.println("Valores normalizados.");
  Serial.println("Configuración inicial terminada.");
}

/////////////////////////////////////////////////////////////////////////////////////////////////

/* Inicia el proceso de restauración del entorno de medición. */
void initRestoreEnvProccess(void)
{
  Serial.println("----------------------------------------");
  Serial.println("Restaurando entorno de medición…");

  Serial.println("Recuperando tiempo del rtc externo…");
  uint8_t minuteToStop = 58;
  DateTime now = readRtcTime();
  uint8_t hour = now.hour();
  uint8_t minute = now.minute();
  Serial.print("Tiempo actual: ");
  Serial.print(hour);
  Serial.print("hh");
  Serial.print(":");
  Serial.print(minute);
  Serial.println("mm");
  
  // Proceso de: despertar motor, levantar tapa, prender ventilador, dormir motor.
  wakeMotor();
  moveUpMotor();
  turnOnFan();
  putMotorToSleep();

  // Tiempo en millis actual.
  uint32_t currentTime = millis();
  uint32_t fan_time = 3e5; // Tiempo a mantener prendido el ventilador (5min).
  bool alredyVent = false;

  // Mantener en espera hasta que se llegue al minuto 58.
  while(readRtcTime().minute() < minuteToStop)
  {
    if (millis() - currentTime > fan_time && !alredyVent)
    {
      alredyVent = true;
      turnOffFan();
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }

  // Proceso de: despertar motor, bajar tapa, dormir motor.
  wakeMotor();
  moveDownMotor();
  putMotorToSleep();

  Serial.println("----------------------------------------");
  Serial.println("Entorno interno restaurado.");
}

/////////////////////////////////////////////////////////////////////////////////////////////////

/* Prepara el payload de la trama LoRaWAN. */
void prepareTxFrame(uint8_t port)
{
  // Array para almacenar los cuatro bytes del valor de ch4.
  uint8_t innerLoraCh4Array[4];
  // Obtener la dirección del dato completo (float).
  uint8_t* innerLoraCh4Ptr = (uint8_t*)&innerLoraCh4;
  // Obtener los cuatro bytes que conforman el float.
  for (int i = 0; i < 4; i++) 
    innerLoraCh4Array[i] = innerLoraCh4Ptr[i];

  // Tamaño total del payload.
  appDataSize = 14;

  appData[0] = nodeId; // uint8_t nodeId
  appData[1] = innerLoraTemperature; // int8_t
  appData[2] = innerLoraHumidity; // uint8_t
  appData[3] = outerLoraTemperature; // int8_t
  appData[4] = outerLoraHumidity; // uint8_t
  appData[5] = soilLoraMoisture; // uint8_t
  appData[6] = (innerLoraCo2 >> 8) & 0xFF; // uint16_t [byte más significativo]
  appData[7] = innerLoraCo2 & 0xFF; // uint16_t [byte menos significativo]
  appData[8] = (innerLoraN2o >> 8) & 0xFF; // int16_t [byte más significativo]
  appData[9] = innerLoraN2o & 0xFF; // int16 [byte menos significativo]
  appData[10] = innerLoraCh4Array[3]; // float [byte menos significativo]
  appData[11] = innerLoraCh4Array[2]; // float
  appData[12] = innerLoraCh4Array[1]; // float
  appData[13] = innerLoraCh4Array[0]; // float [byte más significativo]

  Serial.print("El payload LoRaWAN contiene (hex): ");
  for (int i = 0; i < appDataSize; i++) {
    Serial.print(appData[i], HEX);
    Serial.print(" | ");
  }
  Serial.println();
}




