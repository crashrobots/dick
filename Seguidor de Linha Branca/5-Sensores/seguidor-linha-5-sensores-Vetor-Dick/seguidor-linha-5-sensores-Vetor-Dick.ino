/*
 * ========================================================================================================================
 * @ Software - Dick - robô seguidor de linha.
 * @ Version - 0.0.10
 * @ Devaloper by - Crash Robots
 * @ Web Site - https://crashrobots.github.io
 * @ License - MIT
 *
 * @ Observation:
 * @ ATENSION:
 *
 * ========================================================================================================================
 * Historic Version
 *
 * @ 27 de agosto   de 2015 - Version 0.0.1  | Início projeto robô seguidor de linha dick.
 * @ 30 de agosto   de 2015 - Version 0.0.2  | Calibração dos sensores IR, teste.
 * @ 31 de agosto   de 2015 - Version 0.0.3  | Recalibração, teste, implementação sonar.
 * @ 24 de setembro de 2015 - Version 0.0.4  | Add 6 sensores IR.
 * @ 24 de setembro de 2015 - Version 0.0.5  | Remove 1 sensor IR.
 * @ 01 de outubro  de 2015 - Version 0.0.5  | Calibração de sensores.
 * @ 28 de outubro  de 2015 - Version 0.0.7  | add metodo calibration, alteração de váriavei para o inglês, add map e constrain.
 * @ 01 de novembro de 2015 - Version 0.0.8  | Calibração de velocidade, implementação de Alertas Luminosos.
 * @ 03 de novembro de 2015 - Version 0.0.9  | Calibração de velocidade, curvas.
 * @ 04 de novembro de 2015 - Version 0.0.10 | Implementação de sonar, para detecção de obstáculo.
 * @ 09 de novembro de 2015 - Version 0.0.10 | Remorção do sonar, Add valores lidos por sensores em um vetor, usando lib. QTRSensors,
 *                                             com o intuito, economizar espaço na memoria, calibração mais precisa, e desempenho
 *                                             nas leituras.
 *
 * ========================================================================================================================
 */

/* Inclui as bibliotecas dependentes. */
#include <DMPH.h>
#include <QTRSensors.h>

/* Definição de variáveis para controle do LED RGB */
 int ValVermelho = 0;
 int ValAzul     = 0;
 int ValVerde    = 0;

/* Tempo de cor de transição. */
 const int delayTime = 10;

/* Pin de coneão para o LED RGB. */
 const int VERMELHO = 10;
 const int VERDE    = 9;
 const int AZUL     = 11;

/* Objetos motor_RIGHT, motor_LEFT que recebem por parametro(motorPin, motorPin, motorPinVel) */
 DMPH motor_RIGHT(6, 7, 5);
 DMPH motor_LEFT(2, 4, 3);

int speedMAX = 255;         /* Velocidade Máxima [255] */
int speedMIN = 0;           /* Velocidade Mínima [0]*/

/* Declara uma variável para cada sensor. (armazenará os valores de leitura de cada sensor.) */
 int sensorIR_Right_2 = 0;
 int sensorIR_Right_1 = 0;
 int sensorIR_Center  = 0;
 int sensorIR_Left_1  = 0;
 int sensorIR_Left_2  = 0;

/* Intencidade p/controle de luminosidade. */
 int lum = 300;

/* Configurações dos sensores. */
#define NUM_SENSORS             5  /* Quantidade de sensores em uso. */
#define NUM_SAMPLES_PER_SENSOR  4  /* Média de amostra por leituras analogicas de cada sensor */

/* Os sensores de 0 a 5 são ligado na entras analogicas de 05 respectivamente. */
 QTRSensorsAnalog qtra((unsigned char[]) {
  0, 1, 2, 3, 4
}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, QTR_NO_EMITTER_PIN);
 unsigned int sensorValues[NUM_SENSORS];


/**
 * ========================================================================================================================
 * setup() - Configuração inicial do sistema, definição de portas, clibração de sensoresn entre outras configurações.
 */
 void setup() {
  /* Inicia o Monitor Serial, pasando por paramentro o valor da sua taxa de transmição. */
  Serial.begin(9600);

  /* Define o pino digital como saída. */
  pinMode(VERDE, OUTPUT);
  pinMode(AZUL, OUTPUT);
  pinMode(VERMELHO, OUTPUT);

  /* Calibra sensores. */
  calibration();
}

/**
 * ========================================================================================================================
 * calibration() - Realiza as instruções necessárias para calibrar os sensores IR.
 */
 void calibration() {

  red(); /* Alerta luminoso de cor vermelha, informando o inicio de clibração dos sensores. */

  /* Move robo em  giro em torno de si, para calibração de sensores. Oba.: Posicionar o robo em cima da linha.*/
  motor_LEFT.move(-90);
  motor_RIGHT.move(115);

  /* Realiza calibração em alguns segundos. (em torno de 10s )*/
  for (int i = 0; i < 100; i++) { //400
    qtra.calibrate(); /* Ler todo os sensores, 10 vezes em 2,5ms. (25ms por cada chamada.) */
  }

  /* Imprime valores de configurações da calibração. */
  printCalibration();

  /* garante parada de giro do robo.. */
  motor_LEFT.move(0);
  motor_RIGHT.move(0);

  green(); /* Alerta luminoso de cor verde, que informa a finaliozação da calibração de sensores.. */

}

/**
* ========================================================================================================================
* loop() - Coração do sistema, execulta em um loop infinito até que o sitema seja desligado.
*
*/
void loop() {

  /* Ler sensores IR. */
  readLine();
}

/**
 * ========================================================================================================================
 * readLine() - Realiza as intruções necessárias para ler os sensores IR.
 */
 void readLine() {

  /* Ler valores dos sensores calibrados. */
  qtra.readLine(sensorValues);

  sensorIR_Right_2 = sensorValues[0];
  sensorIR_Right_1 = sensorValues[1];
  sensorIR_Center  = sensorValues[2];
  sensorIR_Left_1  = sensorValues[3];
  sensorIR_Left_2  = sensorValues[4];

  /* Chamada do método que controla os movimentos do robo, passando por parametro os valores dos IR. */
  controlMotor(sensorIR_Right_2, sensorIR_Right_1, sensorIR_Center, sensorIR_Left_1, sensorIR_Left_2);
}


/**
 * ========================================================================================================================
 * controlMotor() - Recebe os valores dos sensores IR e realiza as instruções que farão as ações de mover o robo, para
 *                  frente, esquerda, direita, parar, entre outras ações que faz com que siga linha.
 *
 * Branco = [0]
 * Preto  = [1]
 */
 void controlMotor(int sensorIR_Right_2, int sensorIR_Right_1, int sensorIR_Center, int sensorIR_Left_1, int sensorIR_Left_2) {

  /**
   * [Alinhamento de velocidade dos motores..]
   *
   * motor_RIGHT 120 == 120
   * motor_RIGHT 100 == 100
   * motor_RIGHT 70  == 70
   * 
   * motor_LEFT  120 == 150
   * motor_LEFT  100 == 70
   * motor_LEFT   40 == 70
   */

  /**
   * ========================================================================================================================
   * Frente - move o robo para frente.
   * ========================================================================================================================
   */
  /* [D1][D1] [C0] [E1][E1] */
   if (sensorIR_Right_2 > lum && sensorIR_Right_1 > lum && sensorIR_Center < lum && sensorIR_Left_1 > lum && sensorIR_Left_2 < lum) {
    motor_RIGHT.move(100);
    motor_LEFT.move(70);
    delay(10);
  }

  /**
   * ========================================================================================================================
   * Esquerda - move o robo para a esquerda, deacordo com cada intecidade...
   * ========================================================================================================================
   */

  /* [D1][D0] [C1] [E1][E1] */
   if (sensorIR_Right_2 > lum && sensorIR_Right_1 < lum && sensorIR_Center > lum && sensorIR_Left_1 > lum && sensorIR_Left_2 > lum) {
    motor_RIGHT.move(70);
    motor_LEFT.move(70);
  }
  /* [D0][D1] [C1] [E1][E1] */
  if (sensorIR_Right_2 < lum && sensorIR_Right_1 > lum && sensorIR_Center > lum && sensorIR_Left_1 > lum && sensorIR_Left_2 > lum) {
    motor_RIGHT.move(-100);
    motor_LEFT.move(120);
    delay(10);
  }
  /* [D0][D0] [C1] [E1][E1] */
  if (sensorIR_Right_2 < lum && sensorIR_Right_1 < lum && sensorIR_Center > lum && sensorIR_Left_1 > lum && sensorIR_Left_2 > lum) {
    motor_RIGHT.move(-100);
    motor_LEFT.move(120);
    delay(10);
  }
  /* [D0][D0] [C0] [E1][E1] */
  if (sensorIR_Right_2 < lum && sensorIR_Right_1 < lum && sensorIR_Center < lum && sensorIR_Left_1 > lum && sensorIR_Left_2 > lum) {
    motor_RIGHT.move(-100);
    motor_LEFT.move(120);
    delay(10);
  }
  /* [D0][D0] [C0] [E0][E1] */
  if (sensorIR_Right_2 < lum && sensorIR_Right_1 < lum && sensorIR_Center < lum && sensorIR_Left_1 < lum && sensorIR_Left_2 > lum) {
    motor_RIGHT.move(-100);
    motor_LEFT.move(120);
    delay(10);
  }


  /**
   * ========================================================================================================================
   * Direita - move o robo para a direita, deacor com cada intencidade...
   * ========================================================================================================================
   */

  /* [D1][D1] [C1] [E0][E1] */
   if (sensorIR_Right_2 > lum && sensorIR_Right_1 > lum && sensorIR_Center > lum && sensorIR_Left_1 < lum && sensorIR_Left_2 > lum) {
    motor_RIGHT.move(100);
    motor_LEFT.move(40);
  }
  /* [D1][D1] [C1] [E1][E0] */
  if (sensorIR_Right_2 > lum && sensorIR_Right_1 > lum && sensorIR_Center > lum && sensorIR_Left_1 > lum && sensorIR_Left_2 < lum) {
    motor_RIGHT.move(150);
    motor_LEFT.move(-70);
    delay(10);
  }
  /* [D1][D1] [C1] [E0][E0] */
  if (sensorIR_Right_2 > lum && sensorIR_Right_1 > lum && sensorIR_Center > lum && sensorIR_Left_1 < lum && sensorIR_Left_2 < lum) {
    motor_RIGHT.move(150);
    motor_LEFT.move(-70);
    delay(10);
  }
  /* [D1][D1] [C0] [E0][E0] */
  if (sensorIR_Right_2 > lum && sensorIR_Right_1 > lum && sensorIR_Center < lum && sensorIR_Left_1 < lum && sensorIR_Left_2 < lum) {
    motor_RIGHT.move(150);
    motor_LEFT.move(-70);
    delay(10);
  }
  /* [D1][D0] [C0] [E0][E0] */
  if (sensorIR_Right_2 > lum && sensorIR_Right_1 < lum && sensorIR_Center < lum && sensorIR_Left_1 < lum && sensorIR_Left_2 < lum) {
    motor_RIGHT.move(150);
    motor_LEFT.move(-70);
    delay(10);
  }

  /**
   * ========================================================================================================================
   * Para - para o robo.
   * ========================================================================================================================
   */

  /* [D0][D0][D0] [E0][E0][E0] */
   if (sensorIR_Right_2 < lum && sensorIR_Right_1 < lum && sensorIR_Center < lum && sensorIR_Left_1 < lum && sensorIR_Left_2 < lum) {
      //motor_RIGHT.move(0);
      //motor_LEFT.move(0);
    }

  //printSensores();

}

/**
 * ========================================================================================================================
 * red() - Alerta luminoso, variação de VERDE para VERMELHO.
 */
 void red() {

  int ValVermelho = 255;
  int ValAzul = 0;
  int ValVerde = 0;

  for (int i = 0 ; i < 255 ; i += 1) {
    ValVerde += 1;
    ValVermelho -= 1;

    /**
     * Em cada ciclo de diferença
     * 255 - ValVerde Redução
     * 255 - ValVermelho Aumenta
     * Calsando uma transição gradual de VERDE para VERMELHO
     */

     analogWrite( VERDE, 255 - ValVerde );
     analogWrite( VERMELHO, 255 - ValVermelho );

     /* Aguardando para perceber cor */
     delay(delayTime);
   }

 }

/**
 * ========================================================================================================================
 * green() - Alerta luminoso Verde, variação de AZUL para VERDE.
 */
 void green() {

  ValVermelho = 0;
  ValAzul = 0;
  ValVerde = 255;

  for (int i = 0 ; i < 255 ; i += 1) {
    ValAzul += 1;
    ValVerde -= 1;

    /**
     * Em cada ciclo de diferença
     * 255 - ValAzul Redução
     * 255 - ValVerde Aumenta
     * Calsando uma transição gradual de AZUL para VERDE.
     */

     analogWrite( AZUL, 255 - ValAzul );
     analogWrite( VERDE, 255 - ValVerde );

     /* Aguardando para perceber cor */
     delay( delayTime );
   }

 }

/**
 * ========================================================================================================================
 * blue() - Alerta luminoso Azul, variação de VERMELHO para AZUL.
 */
 void blue() {

  ValVermelho = 0;
  ValAzul = 255;
  ValVerde = 0;

  for ( int i = 0 ; i < 255 ; i += 1 ) {
    ValVermelho += 1;
    ValAzul -= 1;

    /**
     * Em cada ciclo de diferença
     * 255 - ValVermelho Redução
     * 255 - ValAzul Aumenta
     * Calsando uma transição gradual de VERMELHO para AZUL.
     */
     analogWrite( VERMELHO, 255 - ValVermelho );
     analogWrite( AZUL, 255 - ValAzul );

     /* Aguardando para perceber cor */
     delay( delayTime );
   }

 }




/* ===================================================== PRINT'S ======================================================= */



/**
 * ========================================================================================================================
 * printCalibration - Mostra resultado da calibração, valores MAX e MIN.
 *
 */
 void printCalibration() {

  Serial.println(">>>>>| Calibração de Sensores IR. |<<<<<");

  /* Exibe os valore MIN obtidos. */
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  /* Exibe os valores MAX obitidos. */
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println("---------------------------------------");

}

/**
 * ========================================================================================================================
 * printSensores() -
 */
 void printSensores() {
  // Sensores da direita.
  Serial.print("sensorDir_2: ");
  Serial.println(sensorIR_Right_2);
  Serial.print("sensorDir_1: ");
  Serial.println(sensorIR_Right_1);

  Serial.print("sensor_Cent: ");
  Serial.println(sensorIR_Center);

  // Sensores da esquerda.
  Serial.print("sensorEsq_1: ");
  Serial.println(sensorIR_Left_1);
  Serial.print("sensorEsq_2: ");
  Serial.println(sensorIR_Left_2);

  Serial.println("------------------------------------------");

}

/**
 * ========================================================================================================================
 * printVelMotro() -
 */
 void printVelMotor(int leftMotorSpeed, int rightMotorSpeed) {

  Serial.println(" ");
  Serial.print("leftMotorSpeed: ");
  Serial.println(leftMotorSpeed);
  Serial.print("rightMotorSpeed: ");
  Serial.println(rightMotorSpeed);
  Serial.println("-----------------------------------");

}























