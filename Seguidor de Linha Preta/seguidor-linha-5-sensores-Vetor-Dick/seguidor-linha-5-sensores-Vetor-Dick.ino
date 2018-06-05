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
 * @ 01 de dezembro de 2015 - Version 0.0.13 | Economia de memoria, removendo 5 variáveis.
 * @ 04 de dezembro de 2015 - Version 0.0.14 | ultilização de referencia/ponteiro e #define, para economia de memoria.
 *
 * ========================================================================================================================
 */

 /* Inclui as bibliotecas dependentes. */
 #include <DMPH.h>
 #include <QTRSensors.h>

 /* Pin de coneão para o LED RGB. */
 #define VERMELHO 10
 #define VERDE     9
 #define AZUL     11

 int ValVermelho = 0;
 int ValAzul = 0;
 int ValVerde = 0;

 /* Objetos motor_RIGHT, motor_LEFT que recebem por parametro(motorPin, motorPin, motorPinVel) */
 DMPH motor_RIGHT(6, 7, 5);
 DMPH motor_LEFT(2, 4, 3);

 #define speedMAX 255         /* Velocidade Máxima [255] */
 #define speedMIN   0         /* Velocidade Mínima [0]*/

 /* Intencidade p/controle de luminosidade. */
 #define lum 300

 /* Configurações dos sensores. */
 #define NUM_SENSORS             5  /* Quantidade de sensores em uso. */
 #define NUM_SAMPLES_PER_SENSOR  4  /* Média de amostra por leituras analogicas de cada sensor */

 /* Os sensores de 0 a 5 são ligado na entras analogicas de 05 respectivamente. */
 QTRSensorsAnalog qtra((unsigned char[]) { 0, 1, 2, 3, 4 }, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, QTR_NO_EMITTER_PIN);
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
 	ledRED(); /* Alerta luminoso de cor vermelha, informando o inicio de clibração dos sensores. */
 	unsigned int counter; /* usado como um simples contador */
 	
 	/* Realiza calibração em alguns segundos. (em torno de 10s )*/
 	for (counter = 0; counter < 80; counter++) { //400
 		if(counter < 30 || counter >= 60){
 			/* Gira para a esquerda */
 			motor_LEFT.move(-90);
 			motor_RIGHT.move(115);
 			} else{
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

 		delay(3000);  /* Aguarda 5 segundos para posicionar o robô na linha. */

 		ledOFF();     /* Alerta luminoso, desliga led, informa que vai seguir linha. */

 	}

/**
* ========================================================================================================================
* loop() - Coração do sistema, execulta em um loop infinito até que o sitema seja desligado.
*
*/
void loop() {
	readLine();
	controlMotor();
}

/**
 * ========================================================================================================================
 * readLine() - Realiza as intruções necessárias para ler os sensores IR.
 */
 void readLine() {
 	qtra.readLine(sensorValues); /* Ler valores dos sensores calibrados. */
 }


/**
 * ========================================================================================================================
 * controlMotor() - Recebe os valores dos sensores IR e realiza as instruções que farão as ações de mover o robo, para
 *                  frente, esquerda, direita, parar, entre outras ações que faz com que siga linha.
 *
 * Branco = [0] -> lum = <
 * Preto  = [1] -> lum = >
 */
 void controlMotor() {

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
   /* [D0][D0] [C1] [E0][E0] */
   if (sensorValues[0] < lum && sensorValues[1] < lum && sensorValues[2] > lum && sensorValues[3] < lum && sensorValues[4] < lum) {
   	motor_RIGHT.move(100);
   	motor_LEFT.move(70);
   	delay(10);
   }

  /**
   * ========================================================================================================================
   * Esquerda - move o robo para a esquerda, deacordo com cada intecidade...
   * ========================================================================================================================
   */

   /* [D0][D1] [C0] [E0][E0] */
   if (sensorValues[0] < lum && sensorValues[1] > lum && sensorValues[2] < lum && sensorValues[3] < lum && sensorValues[4] < lum) {
   	motor_RIGHT.move(-100);
   	motor_LEFT.move(120);
   }
   /* [D0][D1] [C1] [E0][E0] */
   if (sensorValues[0] < lum && sensorValues[1] > lum && sensorValues[2] > lum && sensorValues[3] < lum && sensorValues[4] < lum) {
   	motor_RIGHT.move(-100);
   	motor_LEFT.move(120);
   }
   /* [D1][D0] [C0] [E0][E0] */
   if (sensorValues[0] > lum && sensorValues[1] < lum && sensorValues[2] < lum && sensorValues[3] < lum && sensorValues[4] < lum) {
   	motor_RIGHT.move(-100);
   	motor_LEFT.move(120);
   }
   /* [D1][D1] [C0] [E0][E0] */
   if (sensorValues[0] > lum && sensorValues[1] > lum && sensorValues[2] < lum && sensorValues[3] < lum && sensorValues[4] < lum) {
   	motor_RIGHT.move(-100);
   	motor_LEFT.move(120);
   }
   /* [D1][D1] [C1] [E0][E0] */
   if (sensorValues[0] > lum && sensorValues[1] > lum && sensorValues[2] > lum && sensorValues[3] < lum && sensorValues[4] < lum) {
   	motor_RIGHT.move(-100);
   	motor_LEFT.move(120);
   }


  /**
   * ========================================================================================================================
   * Direita - move o robo para a direita, deacor com cada intencidade...
   * ========================================================================================================================
   */

   /* [D0][D0] [C0] [E1][E0] */
   if (sensorValues[0] < lum && sensorValues[1] < lum && sensorValues[2] < lum && sensorValues[3] > lum && sensorValues[4] < lum) {
   	motor_RIGHT.move(150);
   	motor_LEFT.move(-70);
   }
   /* [D0][D0] [C1] [E1][E0] */
   if (sensorValues[0] < lum && sensorValues[1] < lum && sensorValues[2] > lum && sensorValues[3] > lum && sensorValues[4] < lum) {
   	motor_RIGHT.move(150);
   	motor_LEFT.move(-70);
   }
   /* [D0][D0] [C0] [E0][E1] */
   if (sensorValues[0] < lum && sensorValues[1] < lum && sensorValues[2] < lum && sensorValues[3] < lum && sensorValues[4] > lum) {
   	motor_RIGHT.move(150);
   	motor_LEFT.move(-70);
   }
   /* [D0][D0] [C0] [E1][E1] */
   if (sensorValues[0] < lum && sensorValues[1] < lum && sensorValues[2] < lum && sensorValues[3] > lum && sensorValues[4] > lum) {
   	motor_RIGHT.move(150);
   	motor_LEFT.move(-70);
   }
   /* [D0][D0] [C1] [E1][E1] */
   if (sensorValues[0] < lum && sensorValues[1] < lum && sensorValues[2] > lum && sensorValues[3] > lum && sensorValues[4] > lum) {
   	motor_RIGHT.move(150);
   	motor_LEFT.move(-70);
   }

  /**
   * ========================================================================================================================
   * Para - para o robo.
   * ========================================================================================================================
   */
   /* [D0][D0][D0] [E0][E0][E0] */
   // if (sensorValues[0] < lum && sensorValues[1] < lum && sensorValues[2] < lum && sensorValues[3] < lum && sensorValues[4] < lum) {}
   /* [D1][D1][D1] [E1][E1][E1] */
   // if (sensorValues[0] > lum && sensorValues[1] > lum && sensorValues[2] > lum && sensorValues[3] > lum && sensorValues[4] > lum) {}

    //printSensores();    
}

/**
 * ========================================================================================================================
 * ledRED() - Alerta luminoso, variação de VERDE para VERMELHO.
 */
 void ledRED() {
 	ValVermelho = 255;
 	ValAzul = 0;
 	ValVerde = 0;

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
     delay(10);
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
     delay(10);
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
     delay(10);
 }

}


/**
 * ========================================================================================================================
 * ledOFF() - Desliga Led.
 */
 void ledOFF() { digitalWrite(VERDE, LOW); digitalWrite(AZUL, LOW); digitalWrite(VERMELHO, LOW); }



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
 * printSensores() - Imprime os valores lidos pelos sensores IR
 */
 void printSensores() {
 	/* Sensores da direita. */
 	Serial.print("sensorDir_2: ");
 	Serial.println(sensorValues[0]);
 	Serial.print("sensorDir_1: ");
 	Serial.println(sensorValues[1]);

 	Serial.print("sensor_Cent: ");
 	Serial.println(sensorValues[2]);

 	/* Sensores da esquerda. */
 	Serial.print("sensorEsq_1: ");
 	Serial.println(sensorValues[3]);
 	Serial.print("sensorEsq_2: ");
 	Serial.println(sensorValues[4]);

 	Serial.println("------------------------------------------");
 }

/**
 * ========================================================================================================================
 * printVelMotro() - Imprime os valores da velocidade dos motores
 */
 void printVelMotor(int leftMotorSpeed, int rightMotorSpeed) {
 	Serial.println(" ");
 	Serial.print("leftMotorSpeed: ");
 	Serial.println(leftMotorSpeed);
 	Serial.print("rightMotorSpeed: ");
 	Serial.println(rightMotorSpeed);
 	Serial.println("-----------------------------------");
 }


