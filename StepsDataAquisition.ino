/* TODO: Descrição geral do código */

/* ------------------- Encoders Settings ------------------------------- */
/* Dual LS7366 Quadrature Counter Pinout  
   Hardware: Arduino MEGA
   LS7366 Breakout    ---------------   Arduino MEGA
   --------------------------------------------------
        MOSI   ------------------------   SDO (51)
        MISO   ------------------------   SDI (50)
        SCK    ------------------------   SCK (52)
        SS1    ------------------------   SS1 (47)
        SS2    ------------------------   SS2 (46)
        GND    ------------------------   GND
        VDD    ------------------------   VCC (5.0V)
//====================================================*/

// Include the standard Arduino SPI Library, please ensure the SPI pins are
// connected properly for your Arduino version
#include <SPI.h>

/* -------------- Encoders Variables Initialization---------------------- */
// Slave Select pins for encoders 1 and 2
// Feel free to reallocate these pins to best suit your circuit
const int slaveSelectEnc1 = 47;
const int slaveSelectEnc2 = 46;

// These hold the current encoder count
signed long encoder1count = 0;
signed long encoder2count = 0;

// Encoder past readings
signed long encoder1countpast = 0;
signed long encoder2countpast = 0;

/* -------------- Encoders Routines ------------------------------------- */
// Enconders Initialization
void initEncoders() {
  
  // Set slave selects as outputs
  pinMode(slaveSelectEnc1, OUTPUT);
  pinMode(slaveSelectEnc2, OUTPUT);
  
  // Raise select pins
  // Communication begins when you drop the individual select signsl
  digitalWrite(slaveSelectEnc1,HIGH);
  digitalWrite(slaveSelectEnc2,HIGH);
  
  SPI.begin();
  
  // Initialize encoder 1
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    00: non-quadrature count mode. (A = clock, B = direction).
  //    01: x1 quadrature count mode (one count per quadrature cycle).
  //    10: x2 quadrature count mode (two counts per quadrature cycle).
  //    11: x4 quadrature count mode (four counts per quadrature cycle).
  // NOTE: For more information on commands, see datasheet
  digitalWrite(slaveSelectEnc1,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  //SPI.transfer(0x00);                       // Configure to non-quadrature count mode
  SPI.transfer(0x01);                       // Configure to mode x1
  //SPI.transfer(0x02);                       // Configure to mode x2
  //SPI.transfer(0x03);                       // Configure to 4 byte mode x4
  digitalWrite(slaveSelectEnc1,HIGH);       // Terminate SPI conversation 

  // Initialize encoder 2
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(slaveSelectEnc2,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  //SPI.transfer(0x00);                       // Configure to non-quadrature count mode
  SPI.transfer(0x01);                       // Configure to mode x1
  //SPI.transfer(0x02);                       // Configure to mode x2
  //SPI.transfer(0x03);                       // Configure to 4 byte mode x4
  digitalWrite(slaveSelectEnc2,HIGH);       // Terminate SPI conversation 
}

// Encoders Reading
long readEncoder(int encoder) {
  
  // Initialize temporary variables for SPI read
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;  
  
  // Read encoder 1
  if (encoder == 1) {
    digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation
    SPI.transfer(0x60);                     // Request count
    count_1 = SPI.transfer(0x00);           // Read highest order byte
    count_2 = SPI.transfer(0x00);           
    count_3 = SPI.transfer(0x00);           
    count_4 = SPI.transfer(0x00);           // Read lowest order byte
    digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation 
  }
  
  // Read encoder 2
  else if (encoder == 2) {
    digitalWrite(slaveSelectEnc2,LOW);      // Begin SPI conversation
    SPI.transfer(0x60);                      // Request count
    count_1 = SPI.transfer(0x00);           // Read highest order byte
    count_2 = SPI.transfer(0x00);           
    count_3 = SPI.transfer(0x00);           
    count_4 = SPI.transfer(0x00);           // Read lowest order byte
    digitalWrite(slaveSelectEnc2,HIGH);     // Terminate SPI conversation 
  }
  
  // Calculate encoder count
  count_value = (count_1 << 8) + count_2;
  count_value = (count_value << 8) + count_3;
  count_value = (count_value << 8) + count_4;
  
  return count_value;
}

// Clean encoders
void clearEncoderCount() {
    
  // Set encoder1's data register to 0
  digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder1's current data register to center
  digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation   
  
  // Set encoder2's data register to 0
  digitalWrite(slaveSelectEnc2,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(slaveSelectEnc2,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder2's current data register to center
  digitalWrite(slaveSelectEnc2,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(slaveSelectEnc2,HIGH);     // Terminate SPI conversation 
}
/* -------------- End of encoders settings --------------------------------- */

/* --------------------- H-Bridge Settings -------------- ------------------ */
/* ---- H-Bridge Definitions and Variables Initialization ------------------ */
// Definition of motor conditions
#define BRAKE 0
#define CW    1
#define CCW   2
// Definition of safety current (Check: "1.3 Monster Shield Example").
#define CS_THRESHOLD 15

//MOTOR 1 status pins
#define MOTOR_A1_PIN 7
#define MOTOR_B1_PIN 8

//MOTOR 2 status pins
#define MOTOR_A2_PIN 4
#define MOTOR_B2_PIN 9

// PWM pins for each motor
#define PWM_MOTOR_1 2
#define PWM_MOTOR_2 3

// Current Sensors Pins
#define CURRENT_SEN_1 A2   
#define CURRENT_SEN_2 A3

// Output enable pins
#define EN_PIN_1 30
#define EN_PIN_2 33

// Definition of each motor constant
#define MOTOR_1 0
#define MOTOR_2 1

//default motor speed
short motorSpeed = 0;
//default motor status
unsigned short usMotor_Status = BRAKE;

//Function that controls the variables: motor(0 or 1), direction (cw or ccw) and speed (from 0 to 255);
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if(motor == MOTOR_1)
  {
    if(direct == CW)
    {
      digitalWrite(MOTOR_A1_PIN, LOW); 
      digitalWrite(MOTOR_B1_PIN, HIGH);
    }
    else if(direct == CCW)
    {
      digitalWrite(MOTOR_A1_PIN, HIGH);
      digitalWrite(MOTOR_B1_PIN, LOW);      
    }
    else
    {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, LOW);            
    }
    
    analogWrite(PWM_MOTOR_1, pwm); 
  }
  else if(motor == MOTOR_2)
  {
    if(direct == CW)
    {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, HIGH);
    }
    else if(direct == CCW)
    {
      digitalWrite(MOTOR_A2_PIN, HIGH);
      digitalWrite(MOTOR_B2_PIN, LOW);      
    }
    else
    {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, LOW);            
    }
    
    analogWrite(PWM_MOTOR_2, pwm);
  }
}
/* --------------------- H-Bridge Settings ---------------------------------- */

/* --------------------- Current sensor settings ---------------------------- */
// Inclde the Current Sensor Library
#include <ACS712.h>

// Set the ACS sensor model (30A) and the analog pin to read
ACS712 sensor(ACS712_30A, A0);
/* -------------------End of Current sensor settings ------------------------ */

/* --------------------- Voltage sensors settings --------------------------- */
// Analog Input of the output A1 of the H-bridge
const int pinoSensorA1 = A4;
// Analog Input of the output B1 of the H-bridge
const int pinoSensorB1 = A5;
/* ------------ Variables Initialization ------------------------------------ */
// Sensor readings
int leituraSensor1 = 0;
int leituraSensor2 = 0;

// Sensor readings in volts
float tensaoEntrada1 = 0;
float tensaoEntrada2 = 0;

// Voltage readings according to the voltage dividers circuits
float tensaoMedida1 = 0.0; 
float tensaoMedida2 = 0.0; 

// Resistors Measured Value to the voltage convertion
float valorR1A1 = 192.5; //VALOR DO RESISTOR 1 DO DIVISOR DE TENSÃO em kOhm
float valorR2A1 = 100.0; // VALOR DO RESISTOR 2 DO DIVISOR DE TENSÃO em kOhm
float valorR1B1 = 195.0; //VALOR DO RESISTOR 1 DO DIVISOR DE TENSÃO em kOhm
float valorR2B1 = 100.0; // VALOR DO RESISTOR 2 DO DIVISOR DE TENSÃO em kOhm
/* --------------- End of Voltage sensors settings --------------------------- */

/* ----------------- Test Signal Settings ------------------------------------ */


/* ----------------- End of Test Signal Settings ----------------------------- */

/* ------------ Other Variables Initialization ------------------------------- */
// Boolean variable to indicate the interruption occurrence 
bool intboom = false;
//Time variable to record the instant the interruption triggers the measurement
unsigned long tactual = 0;

int j = 0;
int n = 0; //número de interrupções
int stepPerc = 40;
int load = 1;
/* ------------ End of Other Variables Initialization ------------------------ */

void setup() {
  /* PinMode Settings */
  // Set the motors pins to output
  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);

  pinMode(MOTOR_A2_PIN, OUTPUT);
  pinMode(MOTOR_B2_PIN, OUTPUT);

  pinMode(PWM_MOTOR_1, OUTPUT);
  pinMode(PWM_MOTOR_2, OUTPUT);

  pinMode(EN_PIN_1, OUTPUT);
  pinMode(EN_PIN_2, OUTPUT);

  //stop interrupts for till we make the settings
  cli();                      

  // Serial COM for data output
  Serial.begin(2000000);      
  Serial.print("Trial Description - Step (%):"); Serial.print(","); Serial.print(stepPerc); Serial.print(","); Serial.print("Load: "); Serial.print(","); Serial.println(load);
  delay(10);
  // Initiate the encoders and clear the count for the first measurement
  initEncoders();       //Serial.println("Encoders Initialized...");  
  clearEncoderCount();  //Serial.println("Encoders Cleared...");

  // Current Sensor Initialization
  //Serial.println("Wait for current sensor calibration..."); 
  sensor.calibrate();
  //Serial.println("End of calibration");
  
  // Interruption configuration
  /* Calculations (for 10ms): 
  System clock 16 Mhz and Prescalar 64;
  Timer 4 speed = 16Mhz/64 = 250 Khz    
  Pulse time = 1/250 Khz =  4us  
  Count up to = 10ms / 4us = 2500 (so this is the value the OCR register should have)
  Count up to = 15ms / 4us = 3750 (so this is the value the OCR register should have)
  Count up to = 20ms / 4us = 5000 (so this is the value the OCR register should have)*/
  //Serial.println("Interruption Configuration Initialized...");
    
  /*1. First we reset the control register to amke sure we start with everything disabled.*/
  TCCR4A = 0;                 // Reset entire TCCR4A to 0 
  TCCR4B = 0;                 // Reset entire TCCR4B to 0
  TCNT4  = 0;                 //initialize counter value to 0
  /*2. We set the prescalar to the desired value by changing the CS10 CS12 and CS12 bits. */  
  TCCR4B |= B00000011;        //Set CS12 to 0 so, CS11 to 1, and CS10 to 1 we get prescalar 64  
  
  /*3. We enable compare match mode on register A*/
  TIMSK4 |= B00000010;        //Set OCIE4A to 1 so we enable compare match A 
  
  /*4. Set the value of register A to 2500*/
  OCR4A = 2500;             //Finally we set compare register A to this value
  sei();                    //Enable back the interrupts

  // Description of the trial and the header of the printed variables
  //Serial.print("Trial Description - Step (%):"); Serial.print(","); Serial.println(stepPerc);
  //Serial.println("Header...");
  Serial.print("time"); Serial.print(",");
  Serial.print("encoderCount"); Serial.print(",");
  Serial.print("Speed"); Serial.print(",");
  Serial.print("A0(Rawcurrent"); Serial.print(",");
  Serial.print("Current"); Serial.print(",");
  Serial.print("A4(VoltageA1Raw)"); Serial.print(",");
  Serial.print("A5(VoltageB1Raw)"); Serial.print(",");
  Serial.print("VoltageA1"); Serial.print(",");
  Serial.print("VoltageB1"); Serial.print(",");
  Serial.print("MotorVoltage"); Serial.print(",");
  Serial.print("MotorStatus"); Serial.print(",");
  Serial.println("PWM");
}

void loop() {
   // Enable the motor to drive
   digitalWrite(EN_PIN_1, HIGH);
   digitalWrite(EN_PIN_2, HIGH); 

   // Verifies the occurrence of a timer interruption
   if (intboom == true){
    // Record the actual instant
    tactual = micros(); 

    // Read the encoder counter and converts to RPM
    encoder2count = readEncoder(2);
    signed long speed2 = -(1/17.0)*((encoder2count-encoder2countpast)/30.0)/0.01;

    // Read the current sensor
    float I = sensor.getCurrentDC();

    // Read the voltage sensors and converts the measurements to a 12 V scale
    leituraSensor1 = analogRead(pinoSensorA1);
    tensaoEntrada1 = (leituraSensor1 * 5.0) / 1024.0;
    tensaoMedida1 = tensaoEntrada1 / (valorR2A1/(valorR1A1+valorR2A1)); 

    leituraSensor2 = analogRead(pinoSensorB1);
    tensaoEntrada2 = (leituraSensor2 * 5.0) / 1024.0;
    tensaoMedida2 = tensaoEntrada2 / (valorR2B1/(valorR1B1+valorR2B1));

    // Prints the measure values according to the Header TODO: finalizar quais os valores mostrados (saída de controle do motor é a anterior à atualização a seguir) Colocar na vertical co comentários
    Serial.print(tactual); Serial.print(",");
    Serial.print(encoder2count); Serial.print(",");
    Serial.print(speed2); Serial.print(",");
    Serial.print(analogRead(A0)); Serial.print(",");
    Serial.print(I); Serial.print(",");
    Serial.print(leituraSensor1); Serial.print(",");
    Serial.print(leituraSensor2); Serial.print(",");
    Serial.print(tensaoMedida1); Serial.print(",");
    Serial.print(tensaoMedida2); Serial.print(",");
    Serial.print(tensaoMedida2-tensaoMedida1); Serial.print(",");
    Serial.print(usMotor_Status); Serial.print(",");
    Serial.println(motorSpeed);
    
    // Stores the past encoder reading to the next speed calcultation
    encoder2countpast = encoder2count;

    if (j>=10){
      motorSpeed = map(stepPerc, 0, 100, 0, 255);
    }
    else {
      motorSpeed = 0;
      j++;
    }
    
    // Reset the variable to indicate a new timer interruption
    intboom = false;
  }

  // Update the motor output
  usMotor_Status = CW;
  motorGo(MOTOR_1, usMotor_Status, motorSpeed);

}

//With the settings above, this IRS will trigger each 500ms.
ISR(TIMER4_COMPA_vect){
  //First, set the timer back to 0 so it resets for next interrupt
  TCNT4  = 0;  
  // Indicate the interruption occurrence
  if (n<=160){
    intboom = true;               
  }
  else {
    intboom = false;
    usMotor_Status = BRAKE;
    motorSpeed = 0;
  }
  n++;
}
