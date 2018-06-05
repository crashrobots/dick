  /*
   * ========================================================================================================================
   * @ Software - Dick - robô seguidor de linha
   * @ Version - 0.0.1
   * @ Date - 27 de agosto de 2015
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
   * @ 27 de agosto de 2015 - Versão 0.0.1 | Início projeto robô seguidor de linha.
   * @ 30 de agosto de 2015 - Versão 0.0.2 | Calibração dos sensores IR, teste.
   * @ 31 de agosto de 2015 - Versão 0.0.3 | Recalibração, teste, implementação sonar. 
   * @ 24 de setembro de 2015 - Versão 0.0.4 | Add 6 sensores IR.
   * @ 01 de outubro de 2015 - Versão 0.0.5 | Calibração de sensores.
   * @ 28 de outubro de 2015 - Versão 0.0.6 | add metodo calibration, alteração de váriavei para o inglês, add map e constrain,
                                              add lib qtr sensors, add PID.
   *
   * ========================================================================================================================
   */ 

   /**
    * Import Library
    */
   #include <DMPH.h> // Lib. p/controlar ponteH motor driver L298N.
   #include <QTRSensors.h> // Lib p/controlar Sensores IR.

   int LED_RED = 13;
   int LED_YELLOW = 12;

   #define NUM_SENSORS                5  // número de sensores utilizados.
   #define NUM_SAMPLES_PER_SENSOR     4  // média de n amostras lidas pelos sensores.

   // Sensores pin's analogicos [0][1][2][3][4], quantidade de sensores conectado, tempo de saida, sem pin de LED_RED.
   QTRSensorsAnalog qtr((unsigned char[]) { 0, 1, 2, 3, 4 }, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, QTR_NO_EMITTER_PIN);

   // Matriz para armazenar valores dos sensores.
   unsigned int sensorValues[NUM_SENSORS];

   // Objetos motor_RIGHT, motor_LEFT que recebem por parametro(motorPin, motorPin, motorPinVel)
   DMPH motor_RIGHT(6, 7, 5);
   DMPH motor_LEFT(2, 4, 3);

   // velocidade base do motor
   const int speedMAX = 150; // [255]
   const int speedMIN = 0; // [0]

   // PID
   int lastError = 0;
   int  last_proportional = 0;
   int integral = 0;

   #define KP .2
   #define KD 5

  /**
   * ========================================================================================================================
   * setup() - Configuração de sistema.
   *
   */
   void setup(){
    // Inicia o Monitor Serial, pasando por paramentro o valor da sua taxa de transmição.
    Serial.begin(9600);

    pinMode(LED_RED, OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);

    // Inicia calibração de sensores.
    calibration();
  }

  /**
   * ========================================================================================================================
   * calibration() - Função que calibra os sensores IR.
   * O robó varea a area na pista, verificando o valor máximo e minimo e salvando, para ser usado nas comparações posteriormente.  
   *
   */
   void calibration(){

    // Ligar LED_RED do Arduino para indicar que está em modo de calibração..
     digitalWrite(LED_RED, HIGH);


    motor_RIGHT.move(-115);
    motor_LEFT.move(100);

    // realiza calibração, aguarda quase 10 segundos
    for(int i = 0; i < 400; i++){
    // lê todos os sensores 10 vezes em 2,5 ms 5 sensores (ie ~ 25 ms por chamada)
    qtr.calibrate();
    delay(20);
  }

    //garante parar o motor.
    motor_RIGHT.move(0);
    motor_LEFT.move(0);

    // desligar LED_RED do Arduino para indicar que é completou a calibração.
    digitalWrite(LED_RED, LOW);   

    // Mostra resultado da calibração.
    resulCalibration();
    
    delay(2000);

    // Alerta que está pronto para seguir linha..
    digitalWrite(LED_YELLOW, HIGH);
    delay(3000);
    // Alerta inicia seguidor di linha.
    digitalWrite(LED_YELLOW, LOW);  
  }

  /**
   * ========================================================================================================================
   * loop() - Coração do sistema.
   *
   */
   void loop(){
    readIR();
  }

  /**
   * ========================================================================================================================
   * readIR - Realiza leitura dos sensores IR.
   *
   */
   void readIR(){

    // Ler os valores dos sensores...
    // Aqui não estamos interessados nos valores individuais de cada sensor
    unsigned int position = qtr.readLine(sensorValues);

    int error = position - 2000;

    int motorSpeed = KP * error + KD * (error - lastError);
    lastError = error;

    int leftMotorSpeed = 100 + motorSpeed;
    int rightMotorSpeed = 115 - motorSpeed;

    controlMotor(rightMotorSpeed, leftMotorSpeed);

    //Imprime valores lidos pelos sensores...
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
   * controlMotor - Realiza controle dos motores..
   *
   */
   void controlMotor(int speedMotorRIGHT, int speedMotorLEFT){

   if (speedMotorLEFT > 150 ) speedMotorLEFT = 150; // limit top speed
   if (speedMotorRIGHT > 150 ) speedMotorRIGHT = 150; // limit top speed

   if (speedMotorLEFT < 0) speedMotorLEFT = 0; // keep motor above 0
   if (speedMotorRIGHT < 0) speedMotorRIGHT = 0; // keep motor speed above 0
   
   motor_LEFT.move(speedMotorLEFT);     // set motor speed
   motor_RIGHT.move(speedMotorRIGHT);     // set motor speed

    // Envia valores da velocidade para o monitor serial.
    monitorSerial(speedMotorRIGHT, speedMotorLEFT);
    //cdelay(500);
  }

  /**
   * ========================================================================================================================
   * monitorSerial - Método que imprime valores dos sensores IR na tela. Recebe por parametro os valores.
   *
   */
   void monitorSerial(int speedMotorRIGHT, int speedMotorLEFT){

    Serial.print("speedMotorRIGHT: ");
    Serial.println(speedMotorRIGHT);
    Serial.print("speedMotorLEFT: ");
    Serial.println(speedMotorLEFT);
    Serial.println("-----------------------------------");
  }

  /**
   * ========================================================================================================================
   * resulCalibration - Mostra resultado da calibração, valores MAX e MIN.
   *
   */
   void resulCalibration(){  
    // Imprime os valores mínimos medidos na calibração na calibração.
    Serial.print("Valores MIN: ");
    for (int i = 0; i < NUM_SENSORS; i++){
      Serial.print(qtr.calibratedMinimumOn[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println();
    
    // Imprime os valores máximos encontrados na calibração.
    Serial.print("Valores MAX: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(qtr.calibratedMaximumOn[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println("-----------------------------------");
    Serial.println();
  }
















