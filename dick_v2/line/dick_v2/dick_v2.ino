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

QTRSensorsAnalog qtra((unsigned char[]) {
  0, 1, 2, 3, 4
}, QUANT_SENSORS, QUANT_SAMPLES_PER_SENSOR, QTR_NO_EMITTER_PIN);
unsigned int sensorValues[QUANT_SENSORS];

DMPH motor_R(6, 7, 5);
DMPH motor_L(2, 4, 3);

/* Pin's LED RGB. */
#define VERMELHO 10
#define VERDE     9
#define AZUL     13

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
  motor_L.move(-90);
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

   readLine() - Realiza as intruções necessárias para ler os sensores IR.

*/
void readLine() {
  qtra.readLine(sensorValues);
}

/**
   ========================================================================================================================

   loop -

*/
void loop() {
  for (int i = 0; i < 5; i++) {
    //
  }
}

/**
  ========================================================================================================================

   actionMotor -

*/
void actionMotor() {

  /**
     ==== Status sensor line ====
     [0] BRANCO
     [1] PRETO

     ==== ID sensor line ====
     [R] sensor line da direita
     [C] sensor line do centro
     [L] sensor line da esquerda

     ===== TABLE ====
     R0 R0 C0 L0 L0 = move GO 0
     R0 R0 C1 L0 L0 = move GO 1
     R0 R1 C0 L0 L0 = move RIGHT 1
     R1 R1 C0 L0 L0 = move RIGHT 2
     R1 R0 C0 L0 L0 = move RIGHT 3
     R1 R1 C1 L0 L0 = Err
     R1 R1 C1 L1 L0 = Err
     R1 R1 C1 L1 L1 = STOP
     R0 R0 C0 L1 L0 = move LEFT 1
     R0 R0 C0 L1 L1 = move LEFT 2
     R0 R0 C0 L0 L1 = move LEFT 3
     R0 R0 C1 L1 L1 = Err
     R0 R1 C1 L1 L1 = Err

  */

  if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 0 && sensorValues[4] == 0) { /* move GO 0 */
    motor_R.move(100);
    motor_L.move(100);
  }
  else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 0 && sensorValues[4] == 0) { /* move GO 1 */
    motor_R.move(80);
    motor_L.move(80);
  }
  /******************************************************************************************************************/
  else if (sensorValues[0] == 0 && sensorValues[1] == 1 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0) { /* move RIGHT 1 */
    motor_R.move(100);
    motor_L.move(80);
  }
  else if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0) { /* move RIGHT 2 */
    motor_R.move(120);
    motor_L.move(80);
  }
  else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0) { /* move RIGHT 3 */
    motor_R.move(150);
    motor_L.move(80);
  }
  /******************************************************************************************************************/
  else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 1 && sensorValues[4] == 0) { /* move LEFT 1 */
    motor_L.move(100);
    motor_R.move(80);
  }
  else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 1 && sensorValues[4] == 1) { /* move LEFT 2 */
    motor_L.move(120);
    motor_R.move(80);
  }
  else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 1) { /* move LEFT 3 */
    motor_L.move(150);
    motor_R.move(80);
  }
  /******************************************************************************************************************/
  else if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1) { /* STOP */
    motor_R.move(0);
    motor_L.move(0);
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

