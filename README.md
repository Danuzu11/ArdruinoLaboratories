# ArdruinoLaboratories

Laboratorio de prácticas con Arduino y robot diferencial: **trayectorias** (grabar/reproducir camino) y **debounce** para encoders, usando servomotores/DC con driver L298N.

---

## Estructura del repositorio

| Carpeta | Descripción |
|--------|-------------|
| `debounceDelayoOriginal/` | Ejemplo solo de **debounce dinámico** y odometría (encoders, sin motores). |
| `debounceDelay/` | Prueba de **distancia en línea recta** con motores y debounce dinámico. |
| `trayectoria_practica1/` | Robot **grabador/reproductor de trayectoria** con reproducción a **velocidad fija**. |
| `trayectoria_practica2/` | Mismo grabador/reproductor con reproducción **proporcional** a los ticks grabados. |

Los archivos `.mp4` en las carpetas de trayectoria son solo demostrativos; la documentación se centra en el código.

---

## Hardware común

- **Placa:** Arduino UNO (u compatible).
- **Encoders:** Dos encoders de rueda (salida digital). Deben conectarse a **pines con interrupción**:
  - Encoder izquierdo: **Pin 3** (INT1 en UNO).
  - Encoder derecho: **Pin 2** (INT0 en UNO).
  - Entradas con `INPUT_PULLUP`; encoder que drena a GND.
- **Driver de motores:** Estilo L298N (o similar):
  - Motor **izquierdo:** IN1 = 7, IN2 = 8, ENA (PWM) = 10.
  - Motor **derecho:** IN3 = 5, IN4 = 6, ENB (PWM) = 9.
- **Alimentación y GND** comunes según tu montaje (no incluidos en el código).

Los sketches de trayectoria además usan:
- **A0:** botón/pulsador de control manual (avanzar solo mientras está pulsado).
- **Pin 11:** botón Grabar.
- **Pin 12:** botón Reproducir.
- **Pin 4:** LED modo Grabar.
- **Pin 13:** LED modo Reproducir.

---

## 1. debounceDelayoOriginal

**Archivo:** `debounceDelayoOriginal/debounceDelayoOriginal.ino`

### Objetivo

Demostrar **debounce dinámico** para dos encoders de rueda: filtrar ruido sin perder pulsos a alta velocidad. No controla motores; solo lee encoders y envía diagnósticos por Serial.

### Conceptos

- **Debounce:** ignorar pulsos que llegan demasiado juntos (ruido o rebotes). Un debounce **fijo** (ej. 50 ms) puede rechazar pulsos válidos cuando la rueda gira rápido.
- **Debounce dinámico:** el tiempo mínimo entre pulsos válidos se ajusta según el **periodo** reciente:
  - Rueda rápida → periodo corto → debounce corto (menos pérdida de pulsos).
  - Rueda lenta → periodo largo → debounce más largo (mejor filtrado de ruido).

### Parámetros importantes

- `PULSOS_POR_REVOLUCION`: 20 (ajustar al encoder real).
- `DEBOUNCE_FACTOR`: 10 → debounce = periodo / 10.
- `MIN_DEBOUNCE_US`: 50 µs (límite inferior).
- `MAX_DEBOUNCE_US`: 10 000 µs (10 ms, límite superior).

### Flujo

1. **setup:** Configura pines 2 y 3 como `INPUT_PULLUP`, asocia las ISR a flanco `RISING`, inicializa tiempos con `micros()`.
2. **ISR (izquierda/derecha):** En cada flanco se lee `micros()`. Si ha pasado más de `dynamicDebounce_L/R` desde el último pulso válido:
   - Se actualiza el periodo (`currentPeriod_L/R`).
   - Se incrementa el contador de ticks.
   - Se recalcula el debounce como `periodo / DEBOUNCE_FACTOR` y se limita entre `MIN_DEBOUNCE_US` y `MAX_DEBOUNCE_US`.
   - Si no ha pasado ese tiempo, el pulso se considera ruido y se ignora.
3. **loop:** Con `noInterrupts()`/`interrupts()` se copian de forma segura los contadores y periodos. Se calcula RPM con la fórmula `60 000 000 / (periodo * PULSOS_POR_REVOLUCION)` y se imprimen ticks, RPM y debounce por Serial cada 100 ms.

### Salida típica (Serial)

```
L_Ticks: 120, L_RPM: 45, L_Debounce(us): 1100  |  R_Ticks: 118, R_RPM: 44, R_Debounce(us): 1150
```

---

## 2. debounceDelay (debounceDaley.ino)

**Archivo:** `debounceDelay/debounceDaley.ino/debounceDaley.ino.ino`

### Objetivo

Llevar el robot **en línea recta** una distancia objetivo (p. ej. 120 cm) usando encoders con **debounce dinámico** y detenerlo al alcanzar esa distancia.

### Diferencias con debounceDelayoOriginal

- Incluye **control de motores** (mismos pines que en trayectoria).
- Calcula la **distancia** a partir de ticks: `circunferencia = π * DIAMETRO_RUEDA`, `distancia_por_pulso = circunferencia / PPR`, `targetTicks = DISTANCIA_OBJETIVO_CM / distancia_por_pulso`.
- Usa el **promedio** de ticks izquierdo y derecho para decidir cuándo parar (opción: parar cuando cualquiera de los dos llegue al objetivo).
- Al arrancar espera 3 s y luego pone los motores en marcha; al alcanzar el objetivo llama a `stopMotors()` y desactiva las interrupciones de encoder.

### Parámetros clave

- `DIAMETRO_RUEDA`: 5 cm.
- `PPR`: 20.
- `DISTANCIA_OBJETIVO_CM`: 120 cm.
- `TEST_SPEED_LEFT` / `TEST_SPEED_RIGHT`: PWM 0–255 (en el código se usan valores internos `spd_I`/`spd_D` para compensar diferencias entre ruedas).

### ISRs

Misma lógica de debounce dinámico que en debounceDelayoOriginal: periodo, actualización de tiempo del último pulso, incremento de ticks y actualización de `dynamicDebounce_L/R` con límites.

### Lectura segura en loop

Se copian `leftTicks`, `rightTicks`, periodos y debounces dentro de `noInterrupts()`/`interrupts()`. Se usa `(pulsos_actuales_L + pulsos_actuales_R) / 2` para comparar con `targetTicks`. Al alcanzar el objetivo se imprime distancia calculada por rueda y se detienen los motores.

Al final del archivo hay un **bloque comentado** con una versión mínima: un solo encoder, objetivo 100 cm, sin debounce dinámico ni segundo motor; sirve como referencia de evolución del código.

---

## 3. trayectoria_practica1

**Archivo:** `trayectoria_practica1/trayectoria_practica1.ino`

### Objetivo

Robot que **graba** una trayectoria moviendo el robot a mano (o con el pulsador A0) y luego **reproduce** el mismo camino. La reproducción usa **velocidad fija** por tipo de movimiento (recto, giro izquierda, giro derecha) y duración fija por segmento.

### Máquina de estados

- **STATE_IDLE:** Reposo. Botón Grabar → pasa a RECORDING; botón Reproducir (si hay trayectoria) → pasa a PLAYING.
- **STATE_RECORDING:** Cada `SAMPLE_RATE_MS` (500 ms) se toma una “muestra”: se leen los ticks actuales de ambos encoders, se calculan los deltas respecto a la última muestra y se guardan en `trajectory[]` como un segmento `{ leftTicks, rightTicks }`. Máximo `MAX_SEGMENTS` (20). El movimiento se controla con el pulsador A0 (avanzar solo mientras está HIGH).
- **STATE_PLAYING:** Reproduce todos los segmentos en secuencia con `executeSegment()` y al terminar vuelve a IDLE.

### Grabación

- Al entrar en RECORDING se resetean `leftTicks` y `rightTicks` y se activan las interrupciones en los pines de encoder.
- En cada intervalo se hace una copia segura de los contadores (`noInterrupts`/`interrupts`), se calculan `deltaLeft` y `deltaRight` y se guardan en `trajectory[pathLength]`.
- Debounce de **botones:** 1000 ms (`DEBOUNCE_DELAY`). Debounce de **encoders:** 50 ms (`ENCODER_DEBOUNCE_MS`) dentro de las ISR (solo se cuenta un tick si han pasado al menos 50 ms desde el anterior).

### Reproducción (velocidad fija)

`executeSegment(index)` lee `targetLeft` y `targetRight` del segmento:

- Si ambos son 0: motores parados durante `SAMPLE_RATE_MS`.
- Si ambos > 0: avance recto con `FIXED_PLAYBACK_SPEED_L` y `FIXED_PLAYBACK_SPEED`.
- Si solo `targetLeft > 0`: giro a la derecha (solo motor izquierdo).
- Si solo `targetRight > 0`: giro a la izquierda (solo motor derecho).

La duración de cada segmento en reproducción es siempre `SAMPLE_RATE_MS`; no se ajusta la velocidad al número de ticks grabados.

### Funciones de control

- `stopMotors()`: pone IN1/IN2 e IN3/IN4 en LOW y PWM en 0.
- `setMotorSpeed(motor, speed)`: motor 0 = izquierdo, 1 = derecho; `speed` 0–255; avance con IN1=HIGH/IN2=LOW (izq) e IN3=HIGH/IN4=LOW (der).

---

## 4. trayectoria_practica2

**Archivo:** `trayectoria_practica2/trayectoria_practica2.ino`

### Objetivo

Mismo sistema de **grabar/reproducir** trayectoria que en práctica 1, pero la reproducción usa **velocidad proporcional** a los ticks grabados en cada segmento, para intentar aproximar mejor la trayectoria real.

### Diferencias con trayectoria_practica1

- Nuevas constantes: `MIN_PLAYBACK_SPEED` (100) y `MAX_TICKS_PER_SEGMENT` (200).
- En **PLAYING** se llama a `executeSegmentProportional(i)` en lugar de `executeSegment(i)`.

### Reproducción proporcional

`executeSegmentProportional(index)`:

- Si `targetLeft` y `targetRight` son 0: motores parados durante `SAMPLE_RATE_MS`.
- Si hay movimiento:
  - `speedLeft = map(targetLeft, 1, MAX_TICKS_PER_SEGMENT, MIN_PLAYBACK_SPEED, 255)` (y análogo para la derecha).
  - Se aplica `constrain(..., 0, 255)`.
  - Si `targetLeft <= 0` se fuerza `speedLeft = 0`; si `targetRight <= 0` se fuerza `speedRight = 0`.
- Así, los segmentos con más ticks obtienen mayor PWM durante el mismo tiempo, intentando recorrer más distancia en ese intervalo y acercarse al camino grabado.

La duración de cada segmento sigue siendo `SAMPLE_RATE_MS`; solo cambia la velocidad en función de los ticks.

En el código se mantiene también la función `executeSegment()` (velocidad fija) comentada o disponible; en el flujo principal de reproducción se usa la versión proporcional.

---

## Resumen rápido

| Código | Encoders | Motores | Debounce | Función principal |
|--------|----------|---------|----------|--------------------|
| debounceDelayoOriginal | Sí (2) | No | Dinámico | Odometría + RPM por Serial |
| debounceDelay | Sí (2) | Sí | Dinámico | Avanzar X cm y parar |
| trayectoria_practica1 | Sí (2) | Sí | Fijo (50 ms encoders, 1 s botones) | Grabar y reproducir con velocidad fija por segmento |
| trayectoria_practica2 | Sí (2) | Sí | Fijo | Grabar y reproducir con velocidad proporcional a ticks |

Todos los sketches asumen el mismo mapeo de pines (encoders 2 y 3, driver L298N en 5–10, UI en A0 y 4, 11, 12, 13). Ajusta `PULSOS_POR_REVOLUCION`, `DIAMETRO_RUEDA`, velocidades y distancias según tu robot real.
