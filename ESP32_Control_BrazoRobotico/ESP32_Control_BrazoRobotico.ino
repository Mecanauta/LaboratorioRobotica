#include <Arduino.h>

// Definición de pines para motores DC con encoder
// Motor 1 - Base
#define MOTOR1_PWM 25
#define MOTOR1_DIR 26
#define MOTOR1_ENC_A 34
#define MOTOR1_ENC_B 35

// Motor 2 - Hombro
#define MOTOR2_PWM 27
#define MOTOR2_DIR 14
#define MOTOR2_ENC_A 36
#define MOTOR2_ENC_B 39

// Motor 3 - Codo
#define MOTOR3_PWM 12
#define MOTOR3_DIR 13
#define MOTOR3_ENC_A 4
#define MOTOR3_ENC_B 5

// Motor 4 - Muñeca
#define MOTOR4_PWM 32
#define MOTOR4_DIR 33
#define MOTOR4_ENC_A 16
#define MOTOR4_ENC_B 17

// Parámetros de los encoders
#define PPR 1024  // Pulsos por revolución del encoder
#define GEAR_RATIO 19.2  // Relación de engranajes (ajustar según el motor)

// Límites de los ángulos en grados
const int ANGLE_LIMITS[4][2] = {
  {-180, 180},  // Motor 1 (Base)
  {-120, 120},  // Motor 2 (Hombro)
  {-135, 135},  // Motor 3 (Codo)
  {-180, 180}   // Motor 4 (Muñeca)
};

// Variables globales
volatile long encoderPos[4] = {0, 0, 0, 0};  // Posición de los encoders
int targetPos[4] = {0, 0, 0, 0};             // Posición objetivo en pulsos
int currentAngle[4] = {0, 0, 0, 0};          // Ángulo actual en grados
bool moving = false;                         // Indicador de movimiento en curso
bool emergencyStop = false;                  // Estado de parada de emergencia

// Objetos para control de motores
class DCMotorEncoder {
private:
  int pwmPin;
  int dirPin;
  int encoderIndex;
  int pwmValue;
  bool direction;
  float Kp, Ki, Kd;  // Constantes PID
  float error, lastError, integral;
  unsigned long lastTime;
  
public:
  DCMotorEncoder(int pwm, int dir, int encIdx) : 
    pwmPin(pwm), dirPin(dir), encoderIndex(encIdx), 
    pwmValue(0), direction(true), 
    Kp(5.0), Ki(0.1), Kd(0.5),
    error(0), lastError(0), integral(0), lastTime(0) {
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    analogWrite(pwmPin, 0);
    digitalWrite(dirPin, HIGH);
  }
  
  void setTarget(int angle) {
    // Convertir ángulo a pulsos de encoder
    // PPR * GEAR_RATIO * (ángulo / 360)
    long pulses = (long)(PPR * GEAR_RATIO * angle / 360.0);
    targetPos[encoderIndex] = pulses;
    error = 0;
    integral = 0;
    lastError = 0;
    lastTime = millis();
  }
  
  bool update() {
    if (emergencyStop) {
      stop();
      return false;
    }
    
    long currentPos = encoderPos[encoderIndex];
    long targetPulses = targetPos[encoderIndex];
    
    // Calcular error
    error = targetPulses - currentPos;
    
    // Si está cerca del objetivo, detenerse
    if (abs(error) < 5) {
      stop();
      return true;  // Llegó al objetivo
    }
    
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    if (dt > 0) {
      // Cálculo PID
      integral += error * dt;
      float derivative = (error - lastError) / dt;
      
      // Limitar integral para evitar windup
      if (integral > 1000) integral = 1000;
      if (integral < -1000) integral = -1000;
      
      // Calcular salida PID
      float output = Kp * error + Ki * integral + Kd * derivative;
      
      // Limitar salida
      int pwm = constrain(abs(output), 0, 255);
      
      // Establecer dirección
      direction = (error > 0);
      digitalWrite(dirPin, direction ? HIGH : LOW);
      
      // Aplicar PWM
      analogWrite(pwmPin, pwm);
      
      lastError = error;
      lastTime = now;
    }
    
    return false;  // Aún en movimiento
  }
  
  void stop() {
    analogWrite(pwmPin, 0);
  }
  
  int getCurrentAngle() {
    // Convertir pulsos a ángulos
    return (int)(encoderPos[encoderIndex] * 360.0 / (PPR * GEAR_RATIO));
  }
};

// Crear objetos de motor
DCMotorEncoder motor1(MOTOR1_PWM, MOTOR1_DIR, 0);
DCMotorEncoder motor2(MOTOR2_PWM, MOTOR2_DIR, 1);
DCMotorEncoder motor3(MOTOR3_PWM, MOTOR3_DIR, 2);
DCMotorEncoder motor4(MOTOR4_PWM, MOTOR4_DIR, 3);
DCMotorEncoder* motors[4] = {&motor1, &motor2, &motor3, &motor4};

// Interrupciones para encoders
void IRAM_ATTR handleEncoder1A() {
  if (digitalRead(MOTOR1_ENC_A) == digitalRead(MOTOR1_ENC_B)) {
    encoderPos[0]--;
  } else {
    encoderPos[0]++;
  }
}

void IRAM_ATTR handleEncoder1B() {
  if (digitalRead(MOTOR1_ENC_A) == digitalRead(MOTOR1_ENC_B)) {
    encoderPos[0]++;
  } else {
    encoderPos[0]--;
  }
}

void IRAM_ATTR handleEncoder2A() {
  if (digitalRead(MOTOR2_ENC_A) == digitalRead(MOTOR2_ENC_B)) {
    encoderPos[1]--;
  } else {
    encoderPos[1]++;
  }
}

void IRAM_ATTR handleEncoder2B() {
  if (digitalRead(MOTOR2_ENC_A) == digitalRead(MOTOR2_ENC_B)) {
    encoderPos[1]++;
  } else {
    encoderPos[1]--;
  }
}

void IRAM_ATTR handleEncoder3A() {
  if (digitalRead(MOTOR3_ENC_A) == digitalRead(MOTOR3_ENC_B)) {
    encoderPos[2]--;
  } else {
    encoderPos[2]++;
  }
}

void IRAM_ATTR handleEncoder3B() {
  if (digitalRead(MOTOR3_ENC_A) == digitalRead(MOTOR3_ENC_B)) {
    encoderPos[2]++;
  } else {
    encoderPos[2]--;
  }
}

void IRAM_ATTR handleEncoder4A() {
  if (digitalRead(MOTOR4_ENC_A) == digitalRead(MOTOR4_ENC_B)) {
    encoderPos[3]--;
  } else {
    encoderPos[3]++;
  }
}

void IRAM_ATTR handleEncoder4B() {
  if (digitalRead(MOTOR4_ENC_A) == digitalRead(MOTOR4_ENC_B)) {
    encoderPos[3]++;
  } else {
    encoderPos[3]--;
  }
}

// Función para procesar comandos desde MATLAB
void processCommand(String command) {
  // Eliminar espacios y convertir a mayúsculas para normalizar
  command.trim();
  
  // Comando de prueba
  if (command == "TEST") {
    Serial.println("OK:ROBOT_READY");
    return;
  }
  
  // Comando de desconexión
  if (command == "DISCONNECT") {
    stopAllMotors();
    Serial.println("OK:DISCONNECTED");
    return;
  }
  
  // Comando de parada
  if (command == "STOP") {
    stopAllMotors();
    moving = false;
    Serial.println("OK:STOPPED");
    return;
  }
  
  // Comando de emergencia
  if (command == "EMERGENCY") {
    emergencyStop = true;
    stopAllMotors();
    Serial.println("OK:EMERGENCY_ACTIVATED");
    return;
  }
  
  // Comando para reiniciar después de emergencia
  if (command == "RESET_EMERGENCY") {
    emergencyStop = false;
    Serial.println("OK:EMERGENCY_RESET");
    return;
  }
  
  // Obtener posición actual
  if (command == "GET_POS") {
    // Actualizar ángulos actuales
    for (int i = 0; i < 4; i++) {
      currentAngle[i] = motors[i]->getCurrentAngle();
    }
    
    // Enviar posición
    Serial.print("POS:");
    Serial.print(currentAngle[0]);
    Serial.print(",");
    Serial.print(currentAngle[1]);
    Serial.print(",");
    Serial.print(currentAngle[2]);
    Serial.print(",");
    Serial.println(currentAngle[3]);
    return;
  }
  
  // Comando de movimiento
  if (command.startsWith("MOVE:")) {
    if (emergencyStop) {
      Serial.println("ERROR:EMERGENCY_ACTIVE");
      return;
    }
    
    // Formato: MOVE:ang1,ang2,ang3,ang4
    String angleStr = command.substring(5);
    
    // Separar ángulos
    int commaIndex = angleStr.indexOf(',');
    int secondCommaIndex = angleStr.indexOf(',', commaIndex + 1);
    int thirdCommaIndex = angleStr.indexOf(',', secondCommaIndex + 1);
    
    if (commaIndex == -1 || secondCommaIndex == -1 || thirdCommaIndex == -1) {
      Serial.println("ERROR:INVALID_FORMAT");
      return;
    }
    
    // Extraer valores
    int ang1 = angleStr.substring(0, commaIndex).toInt();
    int ang2 = angleStr.substring(commaIndex + 1, secondCommaIndex).toInt();
    int ang3 = angleStr.substring(secondCommaIndex + 1, thirdCommaIndex).toInt();
    int ang4 = angleStr.substring(thirdCommaIndex + 1).toInt();
    
    // Verificar límites
    if (ang1 < ANGLE_LIMITS[0][0] || ang1 > ANGLE_LIMITS[0][1] ||
        ang2 < ANGLE_LIMITS[1][0] || ang2 > ANGLE_LIMITS[1][1] ||
        ang3 < ANGLE_LIMITS[2][0] || ang3 > ANGLE_LIMITS[2][1] ||
        ang4 < ANGLE_LIMITS[3][0] || ang4 > ANGLE_LIMITS[3][1]) {
      Serial.println("ERROR:ANGLE_OUT_OF_RANGE");
      return;
    }
    
    // Establecer objetivos
    motors[0]->setTarget(ang1);
    motors[1]->setTarget(ang2);
    motors[2]->setTarget(ang3);
    motors[3]->setTarget(ang4);
    
    moving = true;
    Serial.println("MOVE_OK");
    return;
  }
  
  // Comando no reconocido
  Serial.println("ERROR:UNKNOWN_COMMAND");
}

// Detener todos los motores
void stopAllMotors() {
  for (int i = 0; i < 4; i++) {
    motors[i]->stop();
  }
}

void setup() {
  // Inicializar comunicación serial
  Serial.begin(115200);
  Serial.println("Robot 4DOF Iniciado");
  
  // Configurar pines de interrupción para encoders
  pinMode(MOTOR1_ENC_A, INPUT_PULLUP);
  pinMode(MOTOR1_ENC_B, INPUT_PULLUP);
  pinMode(MOTOR2_ENC_A, INPUT_PULLUP);
  pinMode(MOTOR2_ENC_B, INPUT_PULLUP);
  pinMode(MOTOR3_ENC_A, INPUT_PULLUP);
  pinMode(MOTOR3_ENC_B, INPUT_PULLUP);
  pinMode(MOTOR4_ENC_A, INPUT_PULLUP);
  pinMode(MOTOR4_ENC_B, INPUT_PULLUP);
  
  // Adjuntar interrupciones
  attachInterrupt(digitalPinToInterrupt(MOTOR1_ENC_A), handleEncoder1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR1_ENC_B), handleEncoder1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_ENC_A), handleEncoder2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_ENC_B), handleEncoder2B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR3_ENC_A), handleEncoder3A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR3_ENC_B), handleEncoder3B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR4_ENC_A), handleEncoder4A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR4_ENC_B), handleEncoder4B, CHANGE);
  
  // Inicializar motores
  stopAllMotors();
  
  // Restablecer posición
  for (int i = 0; i < 4; i++) {
    encoderPos[i] = 0;
    targetPos[i] = 0;
    currentAngle[i] = 0;
  }
}

void loop() {
  // Procesar comandos seriales
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }
  
  // Actualizar motores si están en movimiento
  if (moving && !emergencyStop) {
    bool allDone = true;
    
    for (int i = 0; i < 4; i++) {
      bool motorDone = motors[i]->update();
      allDone = allDone && motorDone;
    }
    
    // Si todos los motores alcanzaron su objetivo
    if (allDone) {
      moving = false;
      Serial.println("MOVE_COMPLETE");
      
      // Actualizar ángulos actuales
      for (int i = 0; i < 4; i++) {
        currentAngle[i] = motors[i]->getCurrentAngle();
      }
    }
  }
  
  // Enviar datos de feedback cada 500ms
  static unsigned long lastFeedbackTime = 0;
  if (millis() - lastFeedbackTime > 500) {
    lastFeedbackTime = millis();
    
    // Actualizar ángulos actuales
    for (int i = 0; i < 4; i++) {
      currentAngle[i] = motors[i]->getCurrentAngle();
    }
    
    // Enviar solo si no hay otro comando en proceso
    if (Serial.availableForWrite() > 40) {
      Serial.print("POS:");
      Serial.print(currentAngle[0]);
      Serial.print(",");
      Serial.print(currentAngle[1]);
      Serial.print(",");
      Serial.print(currentAngle[2]);
      Serial.print(",");
      Serial.println(currentAngle[3]);
    }
  }
}