/*
 * CURSO DE INTRODUCCIÓN A LA ROBÓTICA
 * Ejemplo de Debounce Dinámico para Encoders de Rueda
 *
 * Este código lee dos encoders (Izquierdo y Derecho) usando interrupciones
 * e implementa un 'debounce delay' que se ajusta automáticamente
 * basado en la velocidad actual de la rueda para evitar lecturas
 * falsas (por ruido) y pérdida de pulsos (a alta velocidad).
 *
 * Conexiones:
 * - Encoder Izquierdo: Pin 2 (Interrupción 0 en UNO)
 * - Encoder Derecho: Pin 3 (Interrupción 1 en UNO)
 * - (Asegurar GND común)
 *
 */

// --- Pines de los Encoders (Deben ser pines de Interrupción) ---
const int ENCODER_PIN_L = 2; // Rueda Izquierda
const int ENCODER_PIN_R = 3; // Rueda Derecha

// --- Constantes del Robot ---
// (Pulsos por cada revolución de la rueda)
const int PULSOS_POR_REVOLUCION = 20;

// --- Constantes de Ajuste del Debounce ---
// Usaremos 1/N del periodo actual como tiempo de debounce
const int DEBOUNCE_FACTOR = 10;
// Límite inferior del debounce (para no ser cero)
const long MIN_DEBOUNCE_US = 50; // 50 microsegundos
// Límite superior del debounce (para filtrar ruido lento)
const long MAX_DEBOUNCE_US = 10000; // 10 milisegundos

// --- Variables Globales Volátiles ---
// (Volatile porque se usan en ISRs y en el loop principal)

// --- Izquierda ---
volatile long tickCount_L = 0;
volatile long lastPulseTime_L = 0;
volatile long currentPeriod_L = 1000000; // Default 1 seg (lento)
volatile long dynamicDebounce_L = 1000;  // Default 1ms

// --- Derecha ---
volatile long tickCount_R = 0;
volatile long lastPulseTime_R = 0;
volatile long currentPeriod_R = 1000000; // Default 1 seg (lento)
volatile long dynamicDebounce_R = 1000;  // Default 1ms


// -------------------------------------------------------------------
//                         SETUP
// -------------------------------------------------------------------
void setup()
{
  Serial.begin(9600);
  Serial.println("Iniciando Sistema de Odometría con Debounce Dinámico");

  // Configurar pines de encoder como entrada con PULLUP
  // (Asume que el encoder drena a GND)
  pinMode(ENCODER_PIN_L, INPUT_PULLUP);
  pinMode(ENCODER_PIN_R, INPUT_PULLUP);

  // Adjuntar las interrupciones
  // Se activarán en el flanco ASCENDENTE (RISING)
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_L), isr_Encoder_L, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_R), isr_Encoder_R, RISING);

  // Inicializar los timers
  lastPulseTime_L = micros();
  lastPulseTime_R = micros();
}

// -------------------------------------------------------------------
//                         LOOP PRINCIPAL
// -------------------------------------------------------------------
void loop()
{
  // --- Lectura Segura de Variables Volátiles ---
  // Copiamos las variables rápidamente con las interrupciones
  // deshabilitadas para evitar que se actualicen a mitad de la lectura.

  long period_L_copy;
  long period_R_copy;
  long debounce_L_copy;
  long debounce_R_copy;
  long ticks_L_copy;
  long ticks_R_copy;

  noInterrupts(); // Deshabilitar interrupciones
  period_L_copy = currentPeriod_L;
  period_R_copy = currentPeriod_R;
  debounce_L_copy = dynamicDebounce_L;
  debounce_R_copy = dynamicDebounce_R;
  ticks_L_copy = tickCount_L;
  ticks_R_copy = tickCount_R;
  interrupts();   // Habilitar interrupciones

  // --- Cálculos (Fuera de la ISR) ---

  // Calcular RPM (Revoluciones por Minuto)
  // (1,000,000 microsegundos / periodo) = Frecuencia (Hz)
  // (Frecuencia / Pulsos_Por_Rev) = Rev por Seg
  // (Rev por Seg * 60) = Rev por Min (RPM)
  // 1,000,000 * 60 = 60,000,000

  // Evitar división por cero si el periodo es muy largo o cero
  long rpm_L = 0;
  if (period_L_copy > 0 && period_L_copy < 2000000) // Límite de 2 seg
  {
    rpm_L = 60000000 / (period_L_copy * PULSOS_POR_REVOLUCION);
  }

  long rpm_R = 0;
  if (period_R_copy > 0 && period_R_copy < 2000000)
  {
    rpm_R = 60000000 / (period_R_copy * PULSOS_POR_REVOLUCION);
  }

  // --- Imprimir Diagnósticos ---
  Serial.print("L_Ticks: ");
  Serial.print(ticks_L_copy);
  Serial.print(", L_RPM: ");
  Serial.print(rpm_L);
  Serial.print(", L_Debounce(us): ");
  Serial.print(debounce_L_copy);

  Serial.print("  |  R_Ticks: ");
  Serial.print(ticks_R_copy);
  Serial.print(", R_RPM: ");
  Serial.print(rpm_R);
  Serial.print(", R_Debounce(us): ");
  Serial.println(debounce_R_copy);

  // NOTA: Aquí iría el código para controlar los motores (PWM)
  // por ejemplo: analogWrite(MOTOR_PIN_L, 100);

  delay(100); // Actualizar monitor 10 veces por segundo
}

// -------------------------------------------------------------------
//           RUTINAS DE SERVICIO DE INTERRUPCIÓN (ISRs)
// -------------------------------------------------------------------

/*
 * ISR para el Encoder Izquierdo
 */
void isr_Encoder_L()
{
  long now_us = micros();

  // --- Lógica de Debounce Dinámico ---
  if (now_us - lastPulseTime_L > dynamicDebounce_L)
  {
    // 1. Pulso Válido
    currentPeriod_L = now_us - lastPulseTime_L;
    lastPulseTime_L = now_us;
    tickCount_L++;

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
  // else: Es ruido, ignorar.
}

/*
 * ISR para el Encoder Derecho
 */
void isr_Encoder_R()
{
  long now_us = micros();

  // --- Lógica de Debounce Dinámico ---
  if (now_us - lastPulseTime_R > dynamicDebounce_R)
  {
    // 1. Pulso Válido
    currentPeriod_R = now_us - lastPulseTime_R;
    lastPulseTime_R = now_us;
    tickCount_R++;

    // 2. Ajuste Dinámico
    dynamicDebounce_R = currentPeriod_R / DEBOUNCE_FACTOR;

    // 3. Aplicar Límites
    if (dynamicDebounce_R < MIN_DEBOUNCE_US)
    {
      dynamicDebounce_R = MIN_DEBOUNCE_US;
    }
    if (dynamicDebounce_R > MAX_DEBOUNCE_US)
    {
      dynamicDebounce_R = MAX_DEBOUNCE_US;
    }
  }
  // else: Es ruido, ignorar.
}
