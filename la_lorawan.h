#ifndef LA_LORAWAN_H
#define LA_LORAWAN_H

/* Revisar el tiempo del sistema para la restauración del ambiente */
bool checkRestoringEnvTime(void);

/* Revisar el tiempo y esperar a la próxima ronda */
void checkSendingTime(void);

/* Recopilación de datos de los sensores */
void getSensorsValues(void);

#endif