/*
 * ========================================================================================================================
 * @ Software - Dick - robô seguidor de linha.
 * @ Version - 0.0.9
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
 * @ 27 de agosto   de 2015 - Version 0.0.1  | Início projeto robô seguidor de linha.
 * @ 30 de agosto   de 2015 - Version 0.0.2  | Calibração dos sensores IR, teste.
 * @ 31 de agosto   de 2015 - Version 0.0.3  | Recalibração, teste, implementação sonar.
 * @ 24 de setembro de 2015 - Version 0.0.4  | Add 6 sensores IR.
 * @ 24 de setembro de 2015 - Version 0.0.5  | Remove 1 sensor IR.
 * @ 01 de outubro  de 2015 - Version 0.0.5  | Calibração de sensores.
 * @ 28 de outubro  de 2015 - Version 0.0.7  | add metodo calibration, alteração de váriavei para o inglês, add map e constrain.
 * @ 01 de novembro de 2015 - Version 0.0.8  | Calibração de velocidade, implementação de Alertas Luminosos.
 * @ 03 de novembro de 2015 - Version 0.0.9  | Calibração de velocidade, curvas.
 *
 * ========================================================================================================================
 */

 /* Inclui as bibliotecas dependentes. */
 #include <DMPH.h>

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

 /* Declara uma variável para cada sensor. */
 int sensorIR_Right_2 = 0;
 int sensorIR_Right_1 = 0;
 int sensorIR_Center  = 0;
 int sensorIR_Left_1  = 0;
 int sensorIR_Left_2  = 0;

 /* Intencidade p/controle de luminosidade. */
 int lum = 300;

 /* Valor maximo do sesensor IR. */
 int sensorValueMAXRight_2 = 1023;
 int sensorValueMAXRight_1 = 1023;
 int sensorValueMAXCenter  = 1023;
 int sensorValueMAXLeft_1  = 1023;
 int sensorValueMAXLeft_2  = 1023;

 /* Valor minimo do sensor IR. */
 int sensorValueMINRight_2 = 0;
 int sensorValueMINRight_1 = 0;
 int sensorValueMINCenter  = 0;
 int sensorValueMINLeft_1  = 0;
 int sensorValueMINLeft_2  = 0;


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

  // calibra sensores.
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

  /* Realiza calibração em alguns segundos. */
  while (millis() < 5000) { //8000
    /* recebe os valores lidos. */
    sensorIR_Right_2 = analogRead(A0);
    sensorIR_Right_1 = analogRead(A1);
    sensorIR_Center  = analogRead(A2);
    sensorIR_Left_1  = analogRead(A3);
    sensorIR_Left_2  = analogRead(A4);

    /* grava os valor máximo lido pelos sensores. */
    if (sensorIR_Right_2 > sensorValueMAXRight_2) sensorValueMAXRight_2 = sensorIR_Right_2;
    if (sensorIR_Right_1 > sensorValueMAXRight_1) sensorValueMAXRight_1 = sensorIR_Right_1;
    if (sensorIR_Center  > sensorValueMAXCenter)  sensorValueMAXCenter  = sensorIR_Center;
    if (sensorIR_Left_1  > sensorValueMAXLeft_1)  sensorValueMAXLeft_1  = sensorIR_Left_1;
    if (sensorIR_Left_2  > sensorValueMAXLeft_1)  sensorValueMAXLeft_1  = sensorIR_Left_2;

    /* grava o valor mínimo lido pelo sensores. */
    if (sensorIR_Right_2 < sensorValueMINRight_2) sensorValueMINRight_2 = sensorIR_Right_2;
    if (sensorIR_Right_1 < sensorValueMINRight_1) sensorValueMINRight_1 = sensorIR_Right_1;
    if (sensorIR_Center  < sensorValueMINCenter)  sensorValueMINCenter  = sensorIR_Center;
    if (sensorIR_Left_1  < sensorValueMINLeft_1)  sensorValueMINLeft_1  = sensorIR_Left_1;
    if (sensorIR_Left_2  < sensorValueMINLeft_2)  sensorValueMINLeft_2  = sensorIR_Left_2;
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

  /* recebe os valores lidos, passando por parametro a porta analogica de conexão do sensor no Arduíno, realizando um mapeamento de valores máximo e minimos com a função map(). */
  sensorIR_Right_2 = map(analogRead(A0), sensorValueMINRight_2, sensorValueMAXRight_2, 0, 1023);
  sensorIR_Right_1 = map(analogRead(A1), sensorValueMINRight_1, sensorValueMAXRight_1, 0, 1023);
  sensorIR_Center  = map(analogRead(A2), sensorValueMINCenter,  sensorValueMAXCenter,  0, 1023);
  sensorIR_Left_1  = map(analogRead(A3), sensorValueMINLeft_1,  sensorValueMAXLeft_1,  0, 1023);
  sensorIR_Left_2  = map(analogRead(A4), sensorValueMINLeft_2,  sensorValueMAXLeft_2,  0, 1023);

  /* se os valores lidos e mapeados dos sensores estiverem fora do intervalo de calibração.. restrigem para dentro do intervalo. */
  sensorIR_Right_2 = constrain(sensorIR_Right_2, 0, 1023);
  sensorIR_Right_1 = constrain(sensorIR_Right_1, 0, 1023);
  sensorIR_Center  = constrain(sensorIR_Center,  0, 1023);
  sensorIR_Left_1  = constrain(sensorIR_Left_1,  0, 1023);
  sensorIR_Left_2  = constrain(sensorIR_Left_2,  0, 1023);

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
   * ========================================================================================================================
   * Frente - move o robo para frente.
   * ========================================================================================================================
   */

   /* [D1][D1] [C0] [E1][E1] */
   if (sensorIR_Right_2 > lum && sensorIR_Right_1 > lum && sensorIR_Center < lum && sensorIR_Left_1 > lum && sensorIR_Left_2 < lum) {
    motor_RIGHT.move(115); //165
    motor_LEFT.move(100);  //150
    delay(10);
  }

  /**
   * ========================================================================================================================
   * Esquerda - move o robo para a esquerda, deacordo com cada intecidade...
   * ========================================================================================================================
   */

   /* [D1][D0] [C1] [E1][E1] */
   if (sensorIR_Right_2 > lum && sensorIR_Right_1 < lum && sensorIR_Center > lum && sensorIR_Left_1 > lum && sensorIR_Left_2 > lum) {
    motor_RIGHT.move(80);
    motor_LEFT.move(100);
  }
  /* [D0][D1] [C1] [E1][E1] */
  if (sensorIR_Right_2 < lum && sensorIR_Right_1 > lum && sensorIR_Center > lum && sensorIR_Left_1 > lum && sensorIR_Left_2 > lum) {
    motor_RIGHT.move(-115);
    motor_LEFT.move(150);
    delay(10);
  }
  /* [D0][D0] [C1] [E1][E1] */
  if (sensorIR_Right_2 < lum && sensorIR_Right_1 < lum && sensorIR_Center > lum && sensorIR_Left_1 > lum && sensorIR_Left_2 > lum) {
    motor_RIGHT.move(-115);
    motor_LEFT.move(150);
    delay(10);
  }
  /* [D0][D0] [C0] [E1][E1] */
  if (sensorIR_Right_2 < lum && sensorIR_Right_1 < lum && sensorIR_Center < lum && sensorIR_Left_1 > lum && sensorIR_Left_2 > lum) {
    motor_RIGHT.move(-115);
    motor_LEFT.move(150);
    delay(10);
  }
  /* [D0][D0] [C0] [E0][E1] */
  if (sensorIR_Right_2 < lum && sensorIR_Right_1 < lum && sensorIR_Center < lum && sensorIR_Left_1 < lum && sensorIR_Left_2 > lum) {
    motor_RIGHT.move(-115);
    motor_LEFT.move(150);
    delay(10);
  }


  /**
   * ========================================================================================================================
   * Direita - move o robo para a direita, deacor com cada intencidade...
   * ========================================================================================================================
   */

   /* [D1][D1] [C1] [E0][E1] */
   if (sensorIR_Right_2 > lum && sensorIR_Right_1 > lum && sensorIR_Center > lum && sensorIR_Left_1 < lum && sensorIR_Left_2 > lum) {
    motor_RIGHT.move(115);
    motor_LEFT.move(80);
  }
  /* [D1][D1] [C1] [E1][E0] */
  if (sensorIR_Right_2 > lum && sensorIR_Right_1 > lum && sensorIR_Center > lum && sensorIR_Left_1 > lum && sensorIR_Left_2 < lum) {
    motor_RIGHT.move(150);
    motor_LEFT.move(-100);
    delay(10);
  }
  /* [D1][D1] [C1] [E0][E0] */
  if (sensorIR_Right_2 > lum && sensorIR_Right_1 > lum && sensorIR_Center > lum && sensorIR_Left_1 < lum && sensorIR_Left_2 < lum) {
    motor_RIGHT.move(150);
    motor_LEFT.move(-100);
    delay(10);
  }
  /* [D1][D1] [C0] [E0][E0] */
  if (sensorIR_Right_2 > lum && sensorIR_Right_1 > lum && sensorIR_Center < lum && sensorIR_Left_1 < lum && sensorIR_Left_2 < lum) {
    motor_RIGHT.move(150);
    motor_LEFT.move(-100);
    delay(10);
  }
  /* [D1][D0] [C0] [E0][E0] */
  if (sensorIR_Right_2 > lum && sensorIR_Right_1 < lum && sensorIR_Center < lum && sensorIR_Left_1 < lum && sensorIR_Left_2 < lum) {
    motor_RIGHT.move(150);
    motor_LEFT.move(-100);
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
  Serial.println("sensorValueMIN: ");
  Serial.print(sensorValueMINRight_2);
  Serial.print(sensorValueMINRight_1);
  Serial.print(sensorValueMINCenter);
  Serial.print(sensorValueMINLeft_1);
  Serial.print(sensorValueMINLeft_2);

  Serial.println("sensorValueMAX: ");
  Serial.print(sensorValueMAXRight_2);
  Serial.print(sensorValueMAXRight_1);
  Serial.print(sensorValueMAXCenter);
  Serial.print(sensorValueMAXLeft_1);
  Serial.print(sensorValueMAXLeft_2);
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























