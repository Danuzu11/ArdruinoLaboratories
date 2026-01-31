// Pines de los Encoders (DEBEN SER PINES DE INTERRUPCIÓN)
#define ENCODER_LEFT_PIN 3  // Interrupción 0
#define ENCODER_RIGHT_PIN 2 // Interrupción 1

#define BUTTON_PUSH A0 // Control analogico

// Pines del Controlador de Motor IZQUIERDO
#define MOTOR_LEFT_IN1 7
#define MOTOR_LEFT_IN2 8
#define MOTOR_LEFT_ENA 10 // PWM

// Pines del Controlador de Motor DERECHO
#define MOTOR_RIGHT_IN3 5
#define MOTOR_RIGHT_IN4 6
#define MOTOR_RIGHT_ENB 9 // PWM 

// Pines de la Interfaz de Usuario
#define BUTTON_RECORD_PIN 11 // Botón para Grabar //  naranja - azul
#define BUTTON_PLAY_PIN 12  // Botón para Reproducir //azul - blanco
#define LED_RECORD_PIN 4
#define LED_PLAY_PIN 13

// Grabación
#define SAMPLE_RATE_MS 500 // Muestrear cada 1 segundo
#define MAX_SEGMENTS 20   // Memoria máxima (10 segmentos)

// Ejecución
#define FIXED_PLAYBACK_SPEED 150 // Velocidad fija (de 0 a 255) //
#define FIXED_PLAYBACK_SPEED_L 138 // Velocidad fija (de 0 a 255) //

// Debounce
#define DEBOUNCE_DELAY 1000 // 1 segundo anti-rebote para pulsadores
#define ENCODER_DEBOUNCE_MS 50 // Debounce para encoders

// Contadores de Ticks (actualizados por las ISR)
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// Variables de tiempo para debounce de encoders
volatile unsigned long lastLeftTickTime = 0;
volatile unsigned long lastRightTickTime = 0;

// Estructura para un segmento de trayectoria
struct PathSegment
{
  long leftTicks;
  long rightTicks;
};

// Array para guardar la trayectoria
PathSegment trajectory[MAX_SEGMENTS];
int pathLength = 0; // Cuántos segmentos hemos grabado

// Máquina de Estados
enum RobotState { STATE_IDLE, STATE_RECORDING, STATE_PLAYING };
RobotState currentState;

// Variables de control del botón
unsigned long lastRecordButtonTime = 0;
unsigned long lastPlayButtonTime = 0;

// Variables de estado
unsigned long lastSampleTime = 0;
long lastRecordLeftTicks = 0;
long lastRecordRightTicks = 0;

// --- Rutinas de Servicio de Interrupción (ISR) ---

void isr_left()
{
  unsigned long now = millis();
  if (now - lastLeftTickTime > ENCODER_DEBOUNCE_MS)
  {
    leftTicks++;
    lastLeftTickTime = now;
  }
}

void isr_right()
{
  unsigned long now = millis();
  if (now - lastRightTickTime > ENCODER_DEBOUNCE_MS)
  {
    rightTicks++;
    lastRightTickTime = now;
  }
}

// --- Función Setup (Configuración Inicial) ---

void setup()
{
  Serial.begin(9600);
  Serial.println("Robot Grabador de Trayectoria INICIADO");

  // Configurar pines de motores
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_LEFT_ENA, OUTPUT);
  pinMode(MOTOR_RIGHT_IN3, OUTPUT);
  pinMode(MOTOR_RIGHT_IN4, OUTPUT);
  pinMode(MOTOR_RIGHT_ENB, OUTPUT);

  // Configurar pines de UI
  pinMode(BUTTON_RECORD_PIN, INPUT); // Asumimos pull-down externo
  pinMode(BUTTON_PLAY_PIN, INPUT);   // Asumimos pull-down externo
  pinMode(LED_RECORD_PIN, OUTPUT);
  pinMode(LED_PLAY_PIN, OUTPUT);

  pinMode(BUTTON_PUSH, INPUT); // Configura A0 como entrada

  // Apagar LEDs
  digitalWrite(LED_RECORD_PIN, LOW);
  digitalWrite(LED_PLAY_PIN, LOW);

  currentState = STATE_IDLE;
  Serial.println("Estado: IDLE (Reposo)");
}


// --- Loop Principal (Máquina de Estados) ---

void loop()
{
  // Procesar el estado actual
  if (digitalRead(BUTTON_PUSH) == HIGH) {
    setMotorSpeed(0, FIXED_PLAYBACK_SPEED_L);
    setMotorSpeed(1, FIXED_PLAYBACK_SPEED);
  }else{
    stopMotors();
  }

  switch (currentState)
  {
    case STATE_IDLE:
      handleIdle();
      break;

    case STATE_RECORDING:
      handleRecording();
      break;

    case STATE_PLAYING:
      handlePlaying();
      break;
  }
}

// --- Manejadores de Estado ---

void handleIdle()
{
  // Esperamos la presión de uno de los dos botones
  
  // Chequear botón de Grabar
  bool recordPressed = digitalRead(BUTTON_RECORD_PIN);
  if (recordPressed && (millis() - lastRecordButtonTime > DEBOUNCE_DELAY))
  {
    lastRecordButtonTime = millis();
    enterRecording();
    return; // Salir para no procesar el otro botón
  }
  
  // Chequear botón de Reproducir
  bool playPressed = digitalRead(BUTTON_PLAY_PIN);
  if (playPressed && (millis() - lastPlayButtonTime > DEBOUNCE_DELAY) && pathLength > 0)
  {
    // Solo reproducimos si hay algo grabado
    lastPlayButtonTime = millis();
    enterPlaying();
  }
}

void handleRecording()
{
  if (digitalRead(BUTTON_PUSH) == HIGH) {
    setMotorSpeed(0, FIXED_PLAYBACK_SPEED_L);
    setMotorSpeed(1, FIXED_PLAYBACK_SPEED);
  }else{
    stopMotors();
  }

  // Verificamos si es tiempo de tomar una muestra
  if (millis() - lastSampleTime >= SAMPLE_RATE_MS)
  {
    lastSampleTime = millis(); // Reinicia el temporizador

    // Leemos los contadores actuales (copia segura)
    noInterrupts();
    long currentLeftTicks = leftTicks;
    long currentRightTicks = rightTicks;
    interrupts();

    // Calculamos el delta (el movimiento en este segmento)
    long deltaLeft = currentLeftTicks - lastRecordLeftTicks;
    long deltaRight = currentRightTicks - lastRecordRightTicks;

    // Guardamos el segmento en el array
    if (pathLength < MAX_SEGMENTS)
    {
      trajectory[pathLength].leftTicks = deltaLeft;
      trajectory[pathLength].rightTicks = deltaRight;
      Serial.print("Segmento "); Serial.print(pathLength);
      Serial.print(": L="); Serial.print(deltaLeft);
      Serial.print(", R="); Serial.println(deltaRight);
      
      pathLength++;
    }
    else
    {
      // Memoria llena, salimos del modo grabación
      Serial.println("¡Memoria llena! Saliendo de grabación.");
      enterIdle();
    }

    // Actualizamos los "últimos" valores para el siguiente cálculo
    lastRecordLeftTicks = currentLeftTicks;
    lastRecordRightTicks = currentRightTicks;
  }
}

void handlePlaying()
{
  // En este diseño, la ejecución es "bloqueante".
  // Hacemos todo el recorrido y LUEGO volvemos a IDLE.

  Serial.println("Iniciando reproducción...");

  for (int i = 0; i < pathLength; i++)
  {
    Serial.print("Ejecutando segmento "); Serial.println(i);
    executeSegment(i);
    
  }

  // Al terminar, detenemos todo y volvemos a IDLE
  Serial.println("Reproducción terminada.");
  stopMotors();
  enterIdle();
}


// --- Funciones de Transición de Estado ---

void enterIdle()
{
  currentState = STATE_IDLE;
  digitalWrite(LED_RECORD_PIN, LOW);
  digitalWrite(LED_PLAY_PIN, LOW);
  stopMotors(); // Asegurarnos de que los motores están apagados

  // Desactivamos interrupciones (si estuvieran activas)
  detachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN));
  detachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN));
  
  Serial.println("Estado: IDLE (Reposo)");
}

void enterRecording()
{
  currentState = STATE_RECORDING;
  digitalWrite(LED_RECORD_PIN, HIGH); 
  digitalWrite(LED_PLAY_PIN, LOW);

  // Reiniciamos la trayectoria
  pathLength = 0;
  
  // Reseteamos contadores de grabación
  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  interrupts();
  
  lastRecordLeftTicks = 0;
  lastRecordRightTicks = 0;
  lastSampleTime = millis();

  // Reiniciamos los temporizadores de debounce de encoder
  unsigned long now = millis();
  lastLeftTickTime = now;
  lastRightTickTime = now;
  
  digitalWrite(MOTOR_LEFT_ENA, LOW);
  digitalWrite(MOTOR_RIGHT_ENB, LOW);
  
  // Activamos interrupciones SOLO al entrar a grabar
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), isr_left, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), isr_right, RISING);
  
  Serial.println("Estado: RECORDING (Grabando). Mueve el robot...");
}

void enterPlaying()
{
  currentState = STATE_PLAYING;
  digitalWrite(LED_RECORD_PIN, LOW);
  digitalWrite(LED_PLAY_PIN, HIGH);

  // Desactivamos interrupciones (no las necesitamos para reproducir)
  detachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN));
  detachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN));
  
  Serial.println("Estado: PLAYING (Reproduciendo).");
  
}

// --- Funciones de Hardware y Control ---

void stopMotors()
{
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  analogWrite(MOTOR_LEFT_ENA, 0);
  
  digitalWrite(MOTOR_RIGHT_IN3, LOW);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);
  analogWrite(MOTOR_RIGHT_ENB, 0);
}

/**
 * Controla un motor (IZQUIERDO o DERECHO)
 * @param motor 0 = IZQUIERDO, 1 = DERECHO
 * @param speed Velocidad de 0 a 255
 */
void setMotorSpeed(int motor, int speed)
{
  int spd = constrain(speed, 0, 255);
  
  if (motor == 0) // Motor Izquierdo
  {
    if (spd > 0) // Avance
    {
      digitalWrite(MOTOR_LEFT_IN1, HIGH);
      digitalWrite(MOTOR_LEFT_IN2, LOW);
      analogWrite(MOTOR_LEFT_ENA, spd);
    }
    else // Parar (Frenar)
    {
      digitalWrite(MOTOR_LEFT_IN1, LOW);
      digitalWrite(MOTOR_LEFT_IN2, LOW);
      analogWrite(MOTOR_LEFT_ENA, 0);
    }
  }
  else // Motor Derecho
  {
    if (spd > 0) // Avance
    {
      digitalWrite(MOTOR_RIGHT_IN3, HIGH);
      digitalWrite(MOTOR_RIGHT_IN4, LOW);
      analogWrite(MOTOR_RIGHT_ENB, spd);
    }
    else // Parar (Frenar)
    {
      digitalWrite(MOTOR_RIGHT_IN3, LOW);
      digitalWrite(MOTOR_RIGHT_IN4, LOW);
      analogWrite(MOTOR_RIGHT_ENB, 0);
    }
  }
}

/**
 * Función de ejecución BLOQUEANTE para un solo segmento
 * Ejecuta los motores a velocidad fija, durante SAMPLE_RATE_MS.
 */
void executeSegment(int index)
{
  // Obtener los deltas objetivo del array
  long targetLeft = trajectory[index].leftTicks;
  long targetRight = trajectory[index].rightTicks;

  // Si no hay movimiento grabado en este segmento, solo esperamos
  if (targetLeft == 0 && targetRight == 0)
  {
    stopMotors();
    delay(SAMPLE_RATE_MS);
    return;
  }

  // Lógica de Velocidad Fija
  // Simplemente vemos SI se movió, no CUÁNTO
  int speedLeft = 0;
  int speedRight = 0;

  if (targetLeft > 0 && targetRight > 0)
  {
    // Avance recto
    speedLeft = FIXED_PLAYBACK_SPEED_L;
    speedRight = FIXED_PLAYBACK_SPEED;
  }
  else if (targetLeft > 0)
  {
    // Giro a la derecha (solo rueda izq se mueve)
    speedLeft = FIXED_PLAYBACK_SPEED_L;
    speedRight = 0;
  }
  else if (targetRight > 0)
  {
    // Giro a la izquierda (solo rueda der se mueve)
    speedLeft = 0;
    speedRight = FIXED_PLAYBACK_SPEED;
  }

  // Aplicamos la velocidad (solo avance)
  setMotorSpeed(0, speedLeft);
  setMotorSpeed(1, speedRight);

  // Mantenemos la velocidad por el tiempo del segmento
  delay(SAMPLE_RATE_MS); 
  
}