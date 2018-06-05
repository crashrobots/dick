#include <DMPH.h>
#include <QTRSensors.h>

/* Definição de variáveis para controle do LED RGB */
int ValVermelho = 0;
int ValAzul = 0;
int ValVerde = 0;

const int delayTime = 10; /* Tempo de cor de transição. */

/* Pin de coneão para o LED RGB. */
const int VERMELHO = 10;
const int VERDE = 9;
const int AZUL = 11;

#define NUM_SENSORS                5  /* número de sensores utilizados. */
#define NUM_SAMPLES_PER_SENSOR     4  /* média de n amostras lidas pelos sensores. */

/* Sensores pin's analogicos [0][1][2][3][4], quantidade de sensores conectado, tempo de saida, sem pin de LED_RED. */ 
QTRSensorsAnalog qtra((unsigned char[]){ 0, 1, 2, 3, 4 }, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, QTR_NO_EMITTER_PIN);

/* Matriz para armazenar valores dos sensores. */
unsigned int sensorValues[NUM_SENSORS];

/* Objetos motor_RIGHT, motor_LEFT que recebem por parametro(motorPin, motorPin, motorPinVel) */
DMPH motor_RIGHT(6, 7, 5);
DMPH motor_LEFT(2, 4, 3);

int speedRightDEFAULT = 95; /* Velocidade padão motor direito. */
int speedLeftDEFAULT = 80;  /* Velocidade padão motor esquerdo. */
int speedMAX = 130;         /* Velocidade Máxima [255] */ 
int speedMIN = 0;           /* Velocidade Mínima [0]*/   

/* P. D. */
#define Kp 7       /* Proporcional [.2] */
#define Kd 2        /* Derivada [5] */
int lastError = 0;  /* Erro [0] */

/**
 * ========================================================================================================================
 * setup() - 
 */
 void setup() {
	// Inicia o Monitor Serial, pasando por paramentro o valor da sua taxa de transmição.
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
 * calibration() - 
 */
 void calibration(){
 	red(); // Alerta que inicio de clibração dos sensores

 	// Move robo para calibração de sensores.
 	motor_LEFT.move(-80);
 	motor_RIGHT.move(95);

 	int i;

 	// the calibration will take a few seconds
 	for (i = 0; i < 80; i++){
 		qtra.calibrate();
 		delay(20);
 	}

 	// Imprime configurações da calibração.
 	printCalibration();

 	// para robo..
 	motor_LEFT.move(0);
 	motor_RIGHT.move(0);

 	green(); // Alerta que finalizou calibração de sensores..

 }

 /**
 * ========================================================================================================================
 * loop() - 
 *
 */
 void loop(){
 	readLine();
 }

/**
 * ========================================================================================================================
 * readLine() - 
 */
 void readLine(){
 	int position = qtra.readLine(sensorValues);

 	// Imprime valores dos sensores...
 	printSensores(position);

 	pid(position);
 }

/**
 * ========================================================================================================================
 * pid() - 
 */
 void pid(int position){

 	int error = position - 2000;

 	int motorSpeed = Kp * error + Kd * (error - lastError);
 	lastError = error;

 	int leftMotorSpeed = speedLeftDEFAULT + motorSpeed;
 	int rightMotorSpeed = speedRightDEFAULT - motorSpeed;


 	// Envia os valores da velocidade para os motores.
 	controlMotor(leftMotorSpeed, rightMotorSpeed);
 }

/**
 * ========================================================================================================================
 * controlMotor() -
 */
 void controlMotor(int leftMotorSpeed, int rightMotorSpeed){

 	/* Limite de velocidade. */
 	 if (leftMotorSpeed > speedMAX ) leftMotorSpeed = speedMAX;
     if (rightMotorSpeed > speedMAX ) rightMotorSpeed = speedMAX;

     /* Mantem a velocidade do motor acima de 0. */
     if (leftMotorSpeed < 0) leftMotorSpeed = 0;
     if (rightMotorSpeed < 0) rightMotorSpeed = 0;

     motor_LEFT.move(leftMotorSpeed);
     motor_RIGHT.move(rightMotorSpeed);

     printVelMotor(leftMotorSpeed, rightMotorSpeed);
 } 

/**
 * ========================================================================================================================
 * red() - Variação de VERDE para VERMELHO.
 */
 void red() {

 	int ValVermelho = 255;
 	int ValAzul = 0;
 	int ValVerde = 0;

 	for(int i = 0 ; i < 255 ; i += 1) {
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
 		delay( delayTime );
  }

}

/**
 * ========================================================================================================================
 * green() - Variação de AZUL para VERDE.
 */
 void green(){

 	ValVermelho = 0;  
 	ValAzul = 0;  
 	ValVerde = 255;

 	for(int i = 0 ; i < 255 ; i += 1){
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
 * blue() - Variação de VERMELHO para AZUL.
 */
 void blue() {

 	ValVermelho = 0;  
 	ValAzul = 255;  
 	ValVerde = 0;  

 	for( int i = 0 ; i < 255 ; i += 1 ){  
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

 /**
 * ========================================================================================================================
 * printCalibration - Mostra resultado da calibração, valores MAX e MIN.
 *
 */
 void printCalibration(){  
    // Imprime os valores mínimos medidos na calibração na calibração.
 	Serial.println(">>>| Configuração dos sensores. |<<<");

 	Serial.print("Valores MIN: ");
 	for (int i = 0; i < NUM_SENSORS; i++){
 		Serial.print(qtra.calibratedMinimumOn[i]);
 		Serial.print(' ');
 	}
 	Serial.println(" ");

    // Imprime os valores máximos encontrados na calibração.
 	Serial.print("Valores MAX: ");
 	for (int i = 0; i < NUM_SENSORS; i++) {
 		Serial.print(qtra.calibratedMaximumOn[i]);
 		Serial.print(' ');
 	}
 	Serial.println(" ");
 	Serial.println("-----------------------------------");

 }

/**
 * ========================================================================================================================
 * printSensores() - 
 */
 void printSensores(int position){
    // Imprime os valores dos sensores, de 0 a 1000 [0 = reflatancia no MAX e 1000 = reflatania no MIN]
 	for (unsigned char i = 0; i < NUM_SENSORS; i++) {
 		Serial.print(sensorValues[i]);
 		Serial.print('\t');
 	}
 	Serial.println(position);
 	Serial.println();

 }

/**
 * ========================================================================================================================
 * printVelMotro() - 
 */
void printVelMotor(int leftMotorSpeed, int rightMotorSpeed){

	Serial.println(" ");
	Serial.print("leftMotorSpeed: ");
	Serial.println(leftMotorSpeed);
	Serial.print("rightMotorSpeed: ");
	Serial.println(rightMotorSpeed);
	Serial.println("-----------------------------------");

}





















