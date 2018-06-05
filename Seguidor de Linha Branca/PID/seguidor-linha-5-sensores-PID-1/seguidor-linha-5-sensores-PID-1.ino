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

    int LED = 13;

   #define NUM_SENSORS                5  // número de sensores utilizados.
   #define NUM_SAMPLES_PER_SENSOR     4  // média de n amostras lidas pelos sensores.

   // Sensores pin's analogicos [0][1][2][3][4], quantidade de sensores conectado, tempo de saida, sem pin de LED.
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
   int last_proportional;
   int integral;

  /**
   * ========================================================================================================================
   * setup() - Configuração de sistema.
   *
   */
   void setup(){
    // Inicia o Monitor Serial, pasando por paramentro o valor da sua taxa de transmição.
     Serial.begin(9600);

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

    // Ligar LED do Arduino para indicar que está em modo de calibração..
     pinMode(LED, OUTPUT); //Configura LED como saída.
     digitalWrite(LED, HIGH);

    // realiza calibração, aguarda quase 10 segundos
     for(int i = 0; i < 400; i++){
    // lê todos os sensores 10 vezes em 2,5 ms 5 sensores (ie ~ 25 ms por chamada)
      qtr.calibrate();
      delay(20);
    }
    
    //garante parar o motor.
    motor_RIGHT.move(0);
    motor_LEFT.move(0);

    // desligar LED do Arduino para indicar que é completou a calibração.
    digitalWrite(LED, LOW);   

    // Mostra resultado da calibração.
    resulCalibration();
    
    delay(2000);
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

    // O termo proporcional deve ser 0 quando estamos na linha.
    int proportional = ((int)position) - 2000;

    // Calcula o termo derivativo (mudança) e o termo integral (soma) da posição
    int derivative = proportional - last_proportional;
    integral += proportional;

    // Lembrando a ultima posição
    last_proportional = proportional;

    // Calcula a diferença entre o aranjo de potência dos dois motores m1 - m2;
    // Se for um número positivo, o robot irá virar para a direita; 
    // Se for um número negativo, o robot irá virar para a esquerda;
    // E a magnetude dos números determinam a agudez com que fará as curvas/giros.
    int power_difference = proportional/10 + integral/10000 + derivative*3/2;

    // Calcula a configuração atual dos motores.
    // Nunca vamos configurar um motor com valor negativo.
    if(power_difference > speedMAX) power_difference = speedMAX;
    if(power_difference < -speedMAX) power_difference = -speedMAX;

    if(power_difference < 0){
      controlMotor(speedMAX, speedMAX+power_difference);
    }
    else {
      controlMotor(speedMAX-power_difference, speedMAX);
    }

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

    if(speedMotorRIGHT >= 0 && speedMotorLEFT >= 0){
      motor_RIGHT.move(speedMotorRIGHT);
      motor_LEFT.move(speedMotorLEFT);
    }
    if(speedMotorRIGHT >= 0 && speedMotorLEFT < 0){
      speedMotorLEFT = -speedMotorLEFT;
      
      motor_RIGHT.move(speedMotorRIGHT);
      motor_LEFT.move(speedMotorLEFT);
    }
    if(speedMotorRIGHT < 0 && speedMotorLEFT >= 0){
      speedMotorRIGHT = -speedMotorRIGHT;
      
      motor_RIGHT.move(speedMotorRIGHT);
      motor_LEFT.move(speedMotorLEFT);
    } 

    // Envia valores da velocidade para o monitor serial.
    monitorSerial(speedMotorRIGHT, speedMotorLEFT);
    delay(500);
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
















