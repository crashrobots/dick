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
 * @ 19 de novembro de 2015 - Versi0n 0.0.11 | Calibração de sensores, para detectar linha branca.
 * @ 24 de novembro de 2015 - Version 0.0.12 | Add método desligar led, melhoras no movimento do robor ao calibrar sensores. 
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
 #define delayTime  10

 /* Pin de coneão para o LED RGB. */
 const int VERMELHO = 10;
 const int VERDE    = 9;
 const int AZUL     = 11;

 /* Objetos motor_RIGHT, motor_LEFT que recebem por parametro(motorPin, motorPin, motorPinVel) */
 DMPH motor_RIGHT(6, 7, 5);
 DMPH motor_LEFT(2, 4, 3);

 #define speedDEFAULT 100
 #define speedMAX     220  /* Velocidade Máxima [255] */
 #define speedMIN       0  /* Velocidade Mínima [0]*/

 /* Intencidade p/controle de luminosidade. */
 #define lum 300

 /* Configurações dos sensores. */
 #define NUM_SENSORS                  5  	/* Quantidade de sensores em uso. */
 #define NUM_SAMPLES_PER_SENSOR       4  	/* Média de amostra por leituras analogicas de cada sensor */
 #define EMITTER_PIN QTR_NO_EMITTER_PIN 	/* emisor de controle (definido como não ultilizado). */

 /* Os sensores de 0 a 5 são ligado na entras analogicas de 05 respectivamente. */
 QTRSensorsAnalog qtra((unsigned char[]) { 0, 1, 2, 3, 4 }, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
 unsigned int sensorValues[NUM_SENSORS];
 unsigned int line_position = 0; /* Verifica a posição de cada sensor */


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

 	ledRED(); /* Alerta luminoso de cor vermelha, informando o inicio de clibração dos sensores. */

 	unsigned int counter; /* usado como um simples contador */
 	/* Realiza calibração em alguns segundos. (em torno de 10s )*/
  for (counter = 0; counter < 80; counter++) { //400

  	if(counter < 20 || counter >= 60){
  		/* Gira para a esquerda */
  		motor_LEFT.move(-90);
  		motor_RIGHT.move(115);
  	}
  	else{
  		/* Gira para a direita */
  		motor_LEFT.move(90);
  		motor_RIGHT.move(-115);
  	}

  	qtra.calibrate(); /* Ler todo os sensores, 10 vezes em 2,5ms. (25ms por cada chamada.) */
  	delay(10);
  }

  /* Imprime valores de configurações da calibração. */
  printCalibration();

  /* garante parada de giro do robo.. */
  motor_LEFT.move(0);
  motor_RIGHT.move(0);

  ledGREEN();   /* Alerta luminoso de cor verde, que informa a finaliozação da calibração de sensores.. */

  delay(2000);  /* Aguarda 5 segundos para posicionar o robô na linha. */

  ledOFF();     /* Alerta luminoso, desliga led, informa que vai seguir linha. */

}

/**
* ========================================================================================================================
* loop() - Coração do sistema, execulta em um loop infinito até que o sitema seja desligado.
*
*/
void loop() {
	percorrerLabirinto();
}

/**
 * ========================================================================================================================
 * percorrerLabirinto() - Realiza as intruções necessárias para ler os sensores IR.
 */
 void percorrerLabirinto() {
 	lerSensores();

 	/* usando 3 sensores no algoritimo de controle básico. */
 	
 	/* Siga em frente se o sensor central é sobre a linha preta. */
 	if((sensorValues[1] > lum) && (sensorValues[2] < lum) && (sensorValues[3] > lum))
 	set_motorsLF(speedDEFAULT, speedDEFAULT); /* siga em frente */

 	/* Vire à esquerda se o sensor na esquerda do sensor central estiver sobre a linha preta. */
 		else if((sensorValues[1] < lum) && (sensorValues[2] > lum) && (sensorValues[3] > lum))
 	set_motorsLF(speedDEFAULT, 0); /* vire à esquerda */

    // vira à direita se o sensor da direita do sensor central estiver sobre a linha preta
 			else if((sensorValues[1] > lum) && (sensorValues[2] > lum) && (sensorValues[3] < 1))
    set_motorsLF(0, speedDEFAULT);  /* vire à esquerda */
 		}

/**
 * ========================================================================================================================
 * lerSensores - 
 */
 void lerSensores() {
 	/* Ler valores dos sensores calibrados. */
 	qtra.readLine(sensorValues);
 	//printSensores();
 }

/**
 * ========================================================================================================================
 * set_motorsLF - 
 */
 void set_motorsLF(int motor1speed, int motor2speed){
  if (motor1speed > speedMAX ) motor1speed = speedMAX; // limite max da velocidade
  if (motor2speed > speedMAX ) motor2speed = speedMAX; // limite max da velocidade
  if (motor1speed < 0) motor1speed = 0; // mantem a velocidade do motro acima de 0
  if (motor2speed < 0) motor2speed = 0; // mantem a velocidade do motro acima de 0

  set_motors(motor1speed, motor2speed);
}

/**
 * ========================================================================================================================
 * set_motors - 
 */
 void set_motors(int rightMotorSpeed, int leftMotorSpeed){ 
 	if(rightMotorSpeed < 0){
 		motor_RIGHT.move(rightMotorSpeed);
 		}else{
 			motor_RIGHT.move(rightMotorSpeed);
 		}

 		if(leftMotorSpeed < 0){
 			motor_LEFT.move(leftMotorSpeed);
 			}else{
 				motor_LEFT.move(leftMotorSpeed); 
 			}
 		}

/**
 * ========================================================================================================================
 * ledRED() - Alerta luminoso, variação de VERDE para VERMELHO.
 */
 void ledRED() {

 	int ValVermelho = 255;
 	int ValAzul = 0;
 	int ValVerde = 0;

 	for (int i = 0 ; i < 255 ; i += 1) {
 		ValVerde += 1;
 		ValVermelho -= 1;

    /**
     * Em cada ciclo de diferença
     * 255 - ValVerde ledREDução
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
 * ledGREEN() - Alerta luminoso Verde, variação de AZUL para VERDE.
 */
 void ledGREEN() {

 	ValVermelho = 0;
 	ValAzul = 0;
 	ValVerde = 255;

 	for (int i = 0 ; i < 255 ; i += 1) {
 		ValAzul += 1;
 		ValVerde -= 1;

    /**
     * Em cada ciclo de diferença
     * 255 - ValAzul ledREDução
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
 * ledBLUE() - Alerta luminoso Azul, variação de VERMELHO para AZUL.
 */
 void ledBLUE() {

 	ValVermelho = 0;
 	ValAzul = 255;
 	ValVerde = 0;

 	for ( int i = 0 ; i < 255 ; i += 1 ) {
 		ValVermelho += 1;
 		ValAzul -= 1;

    /**
     * Em cada ciclo de diferença
     * 255 - ValVermelho ledREDução
     * 255 - ValAzul Aumenta
     * Calsando uma transição gradual de VERMELHO para AZUL.
     */
     analogWrite( VERMELHO, 255 - ValVermelho );
     analogWrite( AZUL, 255 - ValAzul );

     /* Aguardando para perceber cor */
     delay( delayTime );
 }

}


/**
 * ========================================================================================================================
 * ledOFF() - Desliga Led.
 */
 void ledOFF() {

 	digitalWrite(VERDE, LOW); digitalWrite(AZUL, LOW); digitalWrite(VERMELHO, LOW);

 }










 /* ---------------------------------------------------------------------------------------------------------------------- */
 /* #################################################### PRINT'S ######################################################### */
 /* ---------------------------------------------------------------------------------------------------------------------- */










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
 	Serial.println(sensorValues[0]);
 	Serial.print("sensorDir_1: ");
 	Serial.println(sensorValues[1]);

 	Serial.print("sensor_Cent: ");
 	Serial.println(sensorValues[2]);

  // Sensores da esquerda.
 	Serial.print("sensorEsq_1: ");
 	Serial.println(sensorValues[3]);
 	Serial.print("sensorEsq_2: ");
 	Serial.println(sensorValues[4]);

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


