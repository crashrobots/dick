/*
   ========================================================================================================================

   @ Software - DickV2 - robô seguidor de linha.
   @ Version - 2.0.0
   @ Devaloper by - RodriguesFAS Member Crash Robots
   @ Web Site - <https://crashrobots.github.io> | <http://rodriguesfas.com.br>
   @ License - MIT

   @ Observation:

   @ ATENSION:

   ========================================================================================================================

                                                      Historic Version

   @ 25 de julho de 2016  - Version 1.0.1  | Início.
   @ 01 de agosto de 2016 - Version 1.0.2  | Add Led, sensores QTRA

   ========================================================================================================================
*/

#include <DMPH.h>
#include <QTRSensors.h>

#define QUANT_SENSORS             5  /* Quantidade de sensores definidos. */
#define QUANT_SAMPLES_PER_SENSOR  5  /* Média de amostra por leituras analogicas de cada sensor */

QTRSensorsAnalog qtra((unsigned char[]) { 0, 1, 2, 3, 4 }, QUANT_SENSORS, QUANT_SAMPLES_PER_SENSOR, QTR_NO_EMITTER_PIN);
unsigned int sensorValues[QUANT_SENSORS];

DMPH motor_R(2, 4, 3);
DMPH motor_L(6, 7, 5);

/* Pin's LED RGB. */
#define VERMELHO 10
#define VERDE     9
#define AZUL     13

int last_proportional;
int integral;

/**
   ========================================================================================================================

   setup -

*/
void setup() {
  Serial.begin(9600);

  /* Define o pino digital como saída. */
  pinMode(VERDE, OUTPUT);
  pinMode(AZUL, OUTPUT);
  pinMode(VERMELHO, OUTPUT);

  calibration();
}

/**
   ========================================================================================================================

   calibration - Realiza as instruções necessárias para calibrar os sensores IR.

*/
void calibration() {
  ledRED(); /* Alerta luminoso de cor vermelha, informa que os sensores estão sendo calibrados. */

  /* Move robo em  giro em torno de si, para calibração de sensores. Obs.: Posicionar o robo em cima da linha.*/
  motor_L.move(-115);
  motor_R.move(115);

  /* Realiza calibração em alguns segundos. (em torno de 10s )*/
  for (int i = 0; i < 100; i++) { //400
    qtra.calibrate(); /* Ler todo os sensores, 10 vezes em 2,5ms. (25ms por cada chamada.) */
  }

  motor_L.move(0);
  motor_R.move(0);

  /* Imprime valores de configurações da calibração. */
  printCalibration();

  ledGREEN(); /* Alerta luminoso de cor verde, que informa a finaliozação da calibração de sensores.. */
}

/**
   ========================================================================================================================

   loop -

*/
void loop() {
  // Obtém a posição da linha
  // Aqui não estamos interessados nos valores individuais de cada sensor
  unsigned int position = qtra.readLine(sensorValues);

  // O termo proporcional deve ser 0 quando estamos na linha
  int proportional = ((int)position) - 2000;

  // Calcula o termo derivativo (mudança) e o termo integral (soma) da posição
  int derivative = proportional - last_proportional;
  integral += proportional;

  // Lembrando a ultima posição
  last_proportional = proportional;

  // Calcula a diferença entre o aranjo de potência dos dois motores
  // m1 - m2. Se for um número positivo, o robot irá virar para a 
  // direita. Se for um número negativo, o robot irá virar para a esquerda
  // e a magnetude dos números determinam a agudez com que fará as curvas/giros
  int power_difference = proportional/10 + integral/10000 + derivative*3/2;
    
  // Calcula a configuração atual dos motores.  Nunca vamos configurar
  // um motor com valor negativo
  const int max = 180;
    
  if(power_difference > max) power_difference = max;
  if(power_difference < -max) power_difference = -max;
  
  if(power_difference < 0){
    set_motors(max+power_difference, max);
  }
  else{
    set_motors(max, max-power_difference);
  }
}

// Acionamento dos motores
void set_motors(int left_speed, int right_speed){
  if(right_speed >= 0 && left_speed >= 0){
    motor_R.move(right_speed);
    motor_L.move(left_speed);
  }
  if(right_speed >= 0 && left_speed < 0){
    left_speed = -left_speed;
    motor_R.move(right_speed);
    motor_L.move(left_speed);
  }
  if(right_speed < 0 && left_speed >= 0){
    right_speed = -right_speed;
    motor_R.move(right_speed);
    motor_L.move(left_speed);
  } 
}

/**
   ========================================================================================================================
   ledRED() - Alerta luminoso, variação de VERDE para VERMELHO.
*/
void ledRED() {
  int ValVermelho = 255;
  int ValAzul = 0;
  int ValVerde = 0;

  for (int i = 0 ; i < 255 ; i += 1) {
    ValVerde += 1;
    ValVermelho -= 1;

    /**
       Em cada ciclo de diferença
       255 - ValVerde ledREDução
       255 - ValVermelho Aumenta
       Calsando uma transição gradual de VERDE para VERMELHO
    */

    analogWrite( VERDE, 255 - ValVerde );
    analogWrite( VERMELHO, 255 - ValVermelho );

    /* Aguardando para perceber cor */
    delay(10);
  }

}

/**
   ========================================================================================================================

   ledGREEN() - Alerta luminoso Verde, variação de AZUL para VERDE.

*/
void ledGREEN() {
  int ValVermelho = 0;
  int ValAzul = 0;
  int ValVerde = 255;

  for (int i = 0 ; i < 255 ; i += 1) {
    ValAzul += 1;
    ValVerde -= 1;

    /**
       Em cada ciclo de diferença
       255 - ValAzul ledREDução
       255 - ValVerde Aumenta
       Calsando uma transição gradual de AZUL para VERDE.
    */
    analogWrite( AZUL, 255 - ValAzul );
    analogWrite( VERDE, 255 - ValVerde );

    /* Aguardando para perceber cor */
    delay(10);
  }

}

/**
   ========================================================================================================================

   ledBLUE() - Alerta luminoso Azul, variação de VERMELHO para AZUL.

*/
void ledBLUE() {
  int ValVermelho = 0;
  int ValAzul = 255;
  int ValVerde = 0;

  for ( int i = 0 ; i < 255 ; i += 1 ) {
    ValVermelho += 1;
    ValAzul -= 1;

    /**
       Em cada ciclo de diferença
       255 - ValVermelho ledREDução
       255 - ValAzul Aumenta
       Calsando uma transição gradual de VERMELHO para AZUL.
    */
    analogWrite( VERMELHO, 255 - ValVermelho );
    analogWrite( AZUL, 255 - ValAzul );

    /* Aguardando para perceber cor */
    delay(10);
  }

}

/**
   ========================================================================================================================

   ledOFF() - Desliga Led.

*/
void ledOFF() {
  digitalWrite(VERDE, LOW);
  digitalWrite(AZUL, LOW);
  digitalWrite(VERMELHO, LOW);
}

/**
   ========================================================================================================================

   printCalibration - Mostra resultado da calibração, valores MAX e MIN.

*/
void printCalibration() {

  Serial.println(">>>>>| Calibração de Sensores IR. |<<<<<");

  /* Exibe os valore MIN obtidos. */
  for (int i = 0; i < QUANT_SENSORS; i++) {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  /* Exibe os valores MAX obitidos. */
  for (int i = 0; i < QUANT_SENSORS; i++) {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println("---------------------------------------");

}

