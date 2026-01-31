// =========================================================================
//                           PRUEBA: 50 CM EN LÍNEA RECTA
//                 Control manual de velocidad y Debounce Dinámico
// =========================================================================

// --- CONSTANTES DE CONTROL DE VELOCIDAD (AJUSTA ESTAS) ---
// Modifica estos valores (0 a 255) para las pruebas (100, 180, 255)
// NOTA: Usamos analogWrite (PWM) en los pines ENA/ENB para variar la velocidad.
#define TEST_SPEED_LEFT 100  // Velocidad PWM para el motor IZQUIERDO
#define TEST_SPEED_RIGHT 100 // Velocidad PWM para el motor DERECHO

// --- DEFINICIONES DE HARDWARE (Pines confirmados de trayectoria_practica2.ino) ---
#define ENCODER_LEFT_PIN 3  // Interrupción 1 (para UNO)
#define ENCODER_RIGHT_PIN 2 // Interrupción 0 (para UNO)

// Pines del Controlador de Motor IZQUIERDO
#define MOTOR_LEFT_IN1 7
#define MOTOR_LEFT_IN2 8
#define MOTOR_LEFT_ENA 10 // PWM

// Pines del Controlador de Motor DERECHO
#define MOTOR_RIGHT_IN3 5
#define MOTOR_RIGHT_IN4 6
#define MOTOR_RIGHT_ENB 9 // PWM 

// --- CONSTANTES FÍSICAS DEL ROBOT (AJUSTA ESTOS VALORES) ---
const float DIAMETRO_RUEDA = 5; // cm 
const int PPR = 20; // Pulsos por Revolución (VERIFICA TU VALOR REAL)
const float DISTANCIA_OBJETIVO_CM = 120; // Distancia de prueba: 50 cm

// --- CONSTANTES Y VARIABLES DE DEBOUNCE DINÁMICO ---
const int DEBOUNCE_FACTOR = 10;
const long MIN_DEBOUNCE_US = 50;    // 50 microsegundos
const long MAX_DEBOUNCE_US = 10000; // 10 milisegundos

// Contadores de Ticks (actualizados por las ISR)
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// Variables de tiempo para Debounce Dinámico
volatile unsigned long lastLeftTickTime = 0;
volatile unsigned long lastRightTickTime = 0;
volatile long currentPeriod_L = 1000000; // Periodo entre pulsos (usado solo para cálculo)
volatile long currentPeriod_R = 1000000;
volatile long dynamicDebounce_L = 1000; // Debounce inicial
volatile long dynamicDebounce_R = 1000;  

bool objetivoAlcanzado = false;

// =========================================================================
//                          FUNCIONES DE CONTROL
// =========================================================================

/**
 * Detiene ambos motores.
 */
void stopMotors()
{
  // Desactivar PWM
  analogWrite(MOTOR_LEFT_ENA, 0);
  analogWrite(MOTOR_RIGHT_ENB, 0);
  // Poner los pines de dirección en LOW para asegurar el freno
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN3, LOW);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);
}

/**
 * Controla un motor (0=Izquierdo, 1=Derecho) con velocidad PWM (0-255).
 * Asume que siempre avanzamos (HIGH, LOW).
 */
void setMotorSpeed(int motor, int speed)
{
  int with100 = 17; // con 120

  int spd_D = 255;
  int spd_I = spd_D - with100;


  if (motor == 0) // Motor Izquierdo
  {
    digitalWrite(MOTOR_LEFT_IN1, HIGH); 
    digitalWrite(MOTOR_LEFT_IN2, LOW);
    analogWrite(MOTOR_LEFT_ENA, spd_I);
  }
  else // Motor Derecho
  {
    digitalWrite(MOTOR_RIGHT_IN3, HIGH); 
    digitalWrite(MOTOR_RIGHT_IN4, LOW);
    analogWrite(MOTOR_RIGHT_ENB, spd_D);
  }
}

// =========================================================================
//                            ISRS CON DEBOUNCE DINÁMICO
// =========================================================================

/*
 * ISR para el Encoder Izquierdo (Mantiene los nombres de tu archivo base)
 */
void isr_Encoder_L()
{
  long now_us = micros();
  
  // --- Lógica de Debounce Dinámico ---
  if (now_us - lastLeftTickTime > dynamicDebounce_L) 
  {
    // 1. Pulso Válido
    currentPeriod_L = now_us - lastLeftTickTime; 
    lastLeftTickTime = now_us;
    leftTicks++;
    
    // 2. Ajuste Dinámico
    dynamicDebounce_L = currentPeriod_L / DEBOUNCE_FACTOR;
    
    // 3. Aplicar Límites
    if (dynamicDebounce_L < MIN_DEBOUNCE_US)
    {
      dynamicDebounce_L = MIN_DEBOUNCE_US;
    }
    if (dynamicDebounce_L > MAX_DEBOUNCE_US)
    {
      dynamicDebounce_L = MAX_DEBOUNCE_US;
    }
  }
}

/*
 * ISR para el Encoder Derecho (Mantiene los nombres de tu archivo base)
 */
void isr_Encoder_R()
{
  long now_us = micros();
  
  if (now_us - lastRightTickTime > dynamicDebounce_R) 
  {
    currentPeriod_R = now_us - lastRightTickTime;
    lastRightTickTime = now_us;
    rightTicks++;
    
    dynamicDebounce_R = currentPeriod_R / DEBOUNCE_FACTOR;
    
    if (dynamicDebounce_R < MIN_DEBOUNCE_US)
    {
      dynamicDebounce_R = MIN_DEBOUNCE_US;
    }
    if (dynamicDebounce_R > MAX_DEBOUNCE_US)
    {
      dynamicDebounce_R = MAX_DEBOUNCE_US;
    }
  }
}

// =========================================================================
//                             SETUP Y LOOP
// =========================================================================

void setup()
{
  Serial.begin(9600);
  Serial.println("Robot - Modo Prueba 50 cm");
  
  // Configuración de Pines
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_LEFT_ENA, OUTPUT);
  pinMode(MOTOR_RIGHT_IN3, OUTPUT);
  pinMode(MOTOR_RIGHT_IN4, OUTPUT);
  pinMode(MOTOR_RIGHT_ENB, OUTPUT);
  pinMode(ENCODER_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_PIN, INPUT_PULLUP);
  
  stopMotors();

  // Activación de interrupciones para los encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), isr_Encoder_L, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), isr_Encoder_R, RISING);
  
  // Inicializar los timers de Debounce
  lastLeftTickTime = micros();
  lastRightTickTime = micros();

  Serial.println("========================================");
  Serial.print("Velocidad: L="); Serial.print(TEST_SPEED_LEFT); Serial.print(", R="); Serial.println(TEST_SPEED_RIGHT);
  Serial.println("Iniciando prueba de 50 cm en 3 segundos...");
  delay(3000);

  // INICIAR MOTORES AUTOMÁTICAMENTE
  setMotorSpeed(0, TEST_SPEED_LEFT); 
  setMotorSpeed(1, TEST_SPEED_RIGHT);
}

void loop()
{
  if (objetivoAlcanzado) {
    return; // Detener ejecución si ya se alcanzó el objetivo
  }
  
  // 1. Distancia por Pulso
  float circunferencia = PI * DIAMETRO_RUEDA;
  float distancia_por_pulso = circunferencia / PPR;
  
  // 2. Ticks Objetivo
  long targetTicks = (long)(DISTANCIA_OBJETIVO_CM / distancia_por_pulso); 

  // 3. Lectura de variables 'volatile'
  // Usaremos el tick del motor más rápido para asegurar que el robot se detenga.
  noInterrupts();
  long pulsos_actuales_L = leftTicks;
  long pulsos_actuales_R = rightTicks;
  long debounce_L_copy = dynamicDebounce_L;
  long debounce_R_copy = dynamicDebounce_R;
  long period_L_copy = currentPeriod_L;
  long period_R_copy = currentPeriod_R;
  interrupts();

  // Usamos el promedio de los pulsos para el cálculo de distancia.
  long pulsos_actuales = (pulsos_actuales_L + pulsos_actuales_R) / 2;
  
  // Opcional: Detener si CUALQUIER motor alcanza el objetivo (más seguro para evitar desvío excesivo)
  if (pulsos_actuales_L >= targetTicks || pulsos_actuales_R >= targetTicks) { 
    objetivoAlcanzado = true;
  }
  
  // 4. Detener motores si se alcanzó el objetivo
  if (objetivoAlcanzado) {
    stopMotors();
    
    // Imprimir Diagnósticos
    Serial.println("========================================");
    Serial.println("¡50 CM ALCANZADO! Motores detenidos.");
    Serial.print("Pulsos Finales (L,R): "); Serial.print(pulsos_actuales_L); Serial.print(", "); Serial.println(pulsos_actuales_R);
    Serial.print("Distancia calculada (L,R): "); 
    Serial.print(pulsos_actuales_L * distancia_por_pulso); Serial.print(" cm, ");
    Serial.print(pulsos_actuales_R * distancia_por_pulso); Serial.println(" cm");
    Serial.println("========================================");
    
    // Desactivamos interrupciones
    detachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN));
    detachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN));
    return;
  }
  
  // 5. Impresión de Diagnósticos (similar a tu código base)
  Serial.print("L_Ticks: ");
  Serial.print(pulsos_actuales_L);
  Serial.print(", L_Periodo(us): ");
  Serial.print(period_L_copy);
  Serial.print(", L_Debounce(us): ");
  Serial.print(debounce_L_copy);
  Serial.print("  |  R_Ticks: ");
  Serial.print(pulsos_actuales_R);
  Serial.print(", R_Periodo(us): ");
  Serial.print(period_R_copy);
  Serial.print(", R_Debounce(us): ");
  Serial.println(debounce_R_copy);

  delay(100); // Actualizar monitor 10 veces por segundo
}
  

/*
// Se declara como 'volatile' porque su valor cambia fuera del loop(), dentro de una ISR.
volatile long contador_pulsos_derecha = 0;

// Constantes físicas del robot
const int PIN_ENCODER_DERECHA = 2; // Pin digital con capacidad de interrupción
const float DIAMETRO_RUEDA = 5; // cm
const int PPR = 20; // Pulsos por revolución del disco encoder

// Variables para control de distancia
const float DISTANCIA_OBJETIVO = 100.0; // 1 metro = 100 cm
bool objetivoAlcanzado = false;

// --------------------------------------------------------------------------------------
// Rutina de Servicio de Interrupción (ISR)
// Se ejecuta cada vez que el sensor del encoder derecho detecta un cambio.
void isr_contador_derecha()
{
  contador_pulsos_derecha++;
}

// --------------------------------------------------------------------------------------
void setup()
{
  Serial.begin(9600);
  pinMode(PIN_ENCODER_DERECHA, INPUT_PULLUP);

  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);

  digitalWrite(7,LOW);
  digitalWrite(8,LOW);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_DERECHA), isr_contador_derecha, RISING);
}

// --------------------------------------------------------------------------------------
void loop()
{
  if (objetivoAlcanzado) {
    return; // Detener ejecución si ya se alcanzó el objetivo
  }

  // 1. Distancia por Pulso
  float circunferencia = PI * DIAMETRO_RUEDA;
  float distancia_por_pulso = circunferencia / PPR;

  // 2. Leer variables 'volatile'
  noInterrupts();
  long pulsos_actuales = contador_pulsos_derecha;
  interrupts();

  // 3. Cálculo de distancia total
  float distancia_recorrida = pulsos_actuales * distancia_por_pulso;

  // 4. Verificar si se alcanzó 1 metro
  if (distancia_recorrida >= DISTANCIA_OBJETIVO) {
    // Detener motores
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
    objetivoAlcanzado = true;
    
    Serial.println("========================================");
    Serial.println("¡1 METRO ALCANZADO!");
    Serial.print("Distancia calculada: ");
    Serial.print(distancia_recorrida);
    Serial.println(" cm");
    Serial.println("Motores detenidos.");
    Serial.println("========================================");
  }

  // Imprimir resultado
  Serial.print("Pulsos: ");
  Serial.print(pulsos_actuales);
  Serial.print(" | Distancia (cm): ");
  Serial.println(distancia_recorrida);

  delay(500);
}

*/