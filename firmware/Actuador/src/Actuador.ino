/**
 * @file    main.cpp
 * @brief   Nodo IoT con ESP32-C6, controlador PI discreto y servo actuador.
 *
 * @details
 * Este firmware implementa:
 *  - Un controlador PI discreto (método de Tustin) sobre un actuador tipo servo.
 *  - Recepción de consignas (SP), variables de proceso (PV) y parámetros de sintonía (Kp, Ti)
 *    a través de MQTT (payload ASCII).
 *  - Publicación de la salida lógica del controlador (0..100 %) y de la posición física del servo (grados).
 *  - Ejecución en tiempo real mediante dos tareas FreeRTOS:
 *      - @ref control_task : lazo de control PI + servo + logging.
 *      - @ref mqtt_task    : gestión de WiFi y MQTT.
 *
 * Se utiliza además el puerto serie para generar un log en formato CSV compatible con MATLAB.
 */

#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>

/**
 * @defgroup CONTROL Control del Proceso y Actuador
 * @brief   Implementa el controlador PI discreto y el actuador físico (servo).
 *
 * Este módulo:
 *  - Mantiene los parámetros y estados del controlador PI discretizado por Tustin.
 *  - Recibe SP y PV desde el módulo de comunicaciones (@ref COMMS_MQTT).
 *  - Entrega un mando lógico (0..100 %) que se mapea a grados de servo.
 */

/**
 * @defgroup COMMS_MQTT Comunicación WiFi/MQTT
 * @brief   Gestión de la conectividad WiFi y del enlace con el broker MQTT.
 *
 * Este módulo:
 *  - Establece la conexión WiFi.
 *  - Gestiona la conexión al broker MQTT y la suscripción a tópicos (SP, PV, Kp, Ti).
 *  - Publica el estado del controlador (U lógica) y la posición del servo.
 *  - Interpreta los mensajes de entrada para actualizar variables del módulo @ref CONTROL.
 */

/**
 * @defgroup RTOS_TASKS Manejo de Tareas FreeRTOS
 * @brief   Organización de la ejecución en tiempo real mediante tareas FreeRTOS.
 *
 * Este módulo:
 *  - Define la tarea de control discreto (@ref control_task).
 *  - Define la tarea de comunicaciones (@ref mqtt_task).
 *  - Crea y configura las tareas desde @ref setup().
 */

// ================================================== DEFINICIONES DE HARDWARE ==================================================

/**
 * @brief  Pin GPIO donde se conecta la señal PWM del servo.
 * @note   Debe ser un pin compatible con la generación de PWM en ESP32.
 * @ingroup CONTROL
 */
static const int SERVO_PIN = 21;    

/**
 * @brief  Ancho mínimo de pulso del servo en microsegundos.
 * @ingroup CONTROL
 */
static const int MIN_US    = 500;   

/**
 * @brief  Ancho máximo de pulso del servo en microsegundos.
 * @ingroup CONTROL
 */
static const int MAX_US    = 2400;  

/**
 * @brief  Frecuencia en Hz utilizada para controlar el servo.
 * @ingroup CONTROL
 */
static const int SERVO_HZ  = 50;    

/// Objeto servo utilizado como actuador principal.
/// @ingroup CONTROL
Servo servo;

// ------------------------------------------------- LED integrado (un solo color)

/**
 * @brief  Brillo lógico utilizado para el LED integrado (0..255).
 * @note   Actualmente solo se usa para decidir encendido/apagado,
 *         no para controlar un LED RGB real.
 */
#define RGB_BRIGHTNESS 64

/**
 * @brief  Color lógico actual del LED, empaquetado como 0xRRGGBB.
 *
 * Se utiliza para recordar el estado previo del LED y restaurarlo
 * luego de pequeños pulsos de actividad (por ejemplo, en la tarea de control).
 */
volatile uint32_t current_color = 0;

/**
 * @brief  Actualiza el color lógico del LED y enciende/apaga el LED integrado.
 *
 * @param[in] r  Componente rojo (0..255).
 * @param[in] g  Componente verde (0..255).
 * @param[in] b  Componente azul (0..255).
 *
 * @note Actualmente el hardware solo permite un LED "on/off", por lo que
 *       se enciende el pin @c RGB_BUILTIN si el color es distinto de negro.
 *       El valor de @ref current_color se mantiene para futuros usos.
 */
void setLedColor(uint8_t r, uint8_t g, uint8_t b) {
  current_color = ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
  bool on = (r != 0) || (g != 0) || (b != 0);
  digitalWrite(RGB_BUILTIN, on ? HIGH : LOW);
}

// ================================================== CONTROLADOR PI DISCRETO ==================================================

/**
 * @brief  Ganancia proporcional del controlador PI (Kp de MATLAB).
 * @note   Se puede actualizar en tiempo de ejecución vía MQTT en el tópico @c emu/kp_set.
 * @ingroup CONTROL
 */
volatile float Kp = 5.0f;

/**
 * @brief  Tiempo integral del controlador PI (Ti de MATLAB) en segundos.
 * @note   Se puede actualizar vía MQTT en el tópico @c emu/ti_set.
 * @ingroup CONTROL
 */
volatile float Ti = 250.0f;

/**
 * @brief  Tiempo de muestreo del lazo de control en segundos.
 *
 * @details
 * Corresponde al Tm utilizado en la simulación en MATLAB. El lazo de control discreto
 * se ejecuta cada @ref CONTROL_INTERVAL_MS milisegundos.
 * @ingroup CONTROL
 */
static const float Tm = 1.0f;  

/**
 * @brief  Intervalo de ejecución del lazo de control en milisegundos.
 * @ingroup CONTROL
 */
const uint32_t CONTROL_INTERVAL_MS = (uint32_t)(Tm * 1000.0f);

/**
 * @brief  Mínimo ángulo físico del servo (apertura mínima), en grados.
 * @ingroup CONTROL
 */
static const float SERVO_DEG_MIN = 0.0f;

/**
 * @brief  Máximo ángulo físico del servo (apertura máxima), en grados.
 * @ingroup CONTROL
 */
static const float SERVO_DEG_MAX = 180.0f;

/**
 * @brief  Coeficiente b0 del PI discreto en la forma de Tustin.
 * @details Los coeficientes se recalculan en @ref recalculateCoefficients().
 * @ingroup CONTROL
 */
volatile float Kpos;   // b0

/**
 * @brief  Coeficiente b1 del PI discreto en la forma de Tustin.
 * @ingroup CONTROL
 */
volatile float Kneg;   // b1

/**
 * @brief  Salida lógica actual del controlador PI, en % (0..100).
 *
 * @details
 * Esta es la variable de mando en unidades lógicas. Luego se mapea a grados de servo
 * mediante @ref scaleToDegrees() y @ref setServoDegrees().
 *
 * @note Se inicializa en 25 % y luego, tras el reset inicial (a los 5 s),
 *       se fuerza a 50 % como condición de estado estacionario.
 * @ingroup CONTROL
 */
volatile float u_logic       = 25.0f;

/**
 * @brief  Posición actual del servo en grados físicos.
 * @ingroup CONTROL
 */
volatile float pos_deg       = 0.0f;

/**
 * @brief  Temperatura de consigna (SP) recibida por MQTT.
 * @ingroup CONTROL
 */
volatile float temp_setpoint = 25.0f;

/**
 * @brief  Temperatura de proceso (PV) recibida por MQTT.
 * @ingroup CONTROL
 */
volatile float temp_actual   = 25.0f;

/**
 * @brief  Error de control en el instante k-1 (e[k-1]).
 *
 * @details
 * Se utiliza por el algoritmo recursivo del PI discreto:
 * @f[
 *    u[k] = u[k-1] + Kpos \cdot e[k] + Kneg \cdot e[k-1]
 * @f]
 * @ingroup CONTROL
 */
volatile float error_k_1     = 0.0f;

/**
 * @brief  Índice de muestra del lazo de control (k).
 *
 * @details
 * Se incrementa en @ref control_task y se utiliza para:
 *  - Logging en forma de CSV para MATLAB.
 *  - Implementar la lógica de reset inicial del PI cuando @c sampleIndex == 5.
 * @ingroup CONTROL
 */
volatile unsigned long sampleIndex = 0;

/**
 * @brief  Indica si ya se realizó el reseteo de estado inicial del controlador PI.
 *
 * @details
 * El reseteo inicial fuerza:
 *  - @ref u_logic = 50 %
 *  - @ref error_k_1 = 0
 *  - @ref temp_actual = @ref temp_setpoint
 *
 * Esto permite comenzar desde una condición de estado estacionario.
 * @ingroup CONTROL
 */
bool initial_state_reset = false;

// ================================================== CONFIGURACIÓN WIFI / MQTT ==================================================

/**
 * @brief  SSID de la red WiFi utilizada por el nodo.
 * @ingroup COMMS_MQTT
 */
static const char* WIFI_SSID = "Alumnos 2G";

/**
 * @brief  Contraseña de la red WiFi (vacía en este entorno de laboratorio).
 * @ingroup COMMS_MQTT
 */
static const char* WIFI_PASS = "";

/**
 * @brief  Dirección IP del broker MQTT.
 * @ingroup COMMS_MQTT
 */
static const char* MQTT_HOST = "172.16.131.195";

/**
 * @brief  Puerto TCP del broker MQTT.
 * @ingroup COMMS_MQTT
 */
static const uint16_t MQTT_PORT = 1883;

/**
 * @brief  Usuario de autenticación para MQTT (vacío si no se requiere).
 * @ingroup COMMS_MQTT
 */
static const char* MQTT_USER = "";

/**
 * @brief  Contraseña de autenticación para MQTT (vacía si no se requiere).
 * @ingroup COMMS_MQTT
 */
static const char* MQTT_PASS = "";

/**
 * @name Tópicos de suscripción para sintonización del PI
 * @ingroup COMMS_MQTT
 * @{
 */
static const char* MQTT_SUB_TOPIC_KP  = "emu/kp_set";  /**< Tópico para actualizar Kp. */
static const char* MQTT_SUB_TOPIC_TI  = "emu/ti_set";  /**< Tópico para actualizar Ti. */
/** @} */

/**
 * @name Tópicos de suscripción para señales de control
 * @ingroup COMMS_MQTT
 * @{
 */
static const char* MQTT_SUB_TOPIC_SP  = "emu/t_sp";   /**< Tópico para consigna de temperatura (SP). */
static const char* MQTT_SUB_TOPIC_PV  = "emu/t_act";  /**< Tópico para temperatura de proceso (PV). */
/** @} */

/**
 * @name Tópicos de publicación de estado
 * @ingroup COMMS_MQTT
 * @{
 */
static const char* MQTT_PUB_TOPIC_POS = "emu/servo_pos"; /**< Tópico donde se publica la posición del servo (grados). */
static const char* MQTT_PUB_TOPIC_U   = "emu/u_out";     /**< Tópico donde se publica la salida lógica del PI (0..100 %). */
/** @} */

/// Cliente TCP/IP para la pila WiFi.
/// @ingroup COMMS_MQTT
WiFiClient wifiClient;

/// Cliente MQTT basado en PubSubClient.
/// @ingroup COMMS_MQTT
PubSubClient mqtt(wifiClient);

// ================================================== FUNCIONES DE UTILIDAD ==================================================

/**
 * @brief  Escala el mando lógico (0..100 %) al rango de grados del servo.
 *
 * @param[in] u  Mando lógico en porcentaje (0..100).
 * @return      Ángulo en grados dentro del rango [SERVO_DEG_MIN, SERVO_DEG_MAX].
 * @ingroup CONTROL
 */
float scaleToDegrees(float u) {
  if (u < 0.0f)   u = 0.0f;
  if (u > 100.0f) u = 100.0f;

  float span = (SERVO_DEG_MAX - SERVO_DEG_MIN);
  return SERVO_DEG_MIN + (u / 100.0f) * span;
}

/**
 * @brief  Recalcula los coeficientes del PI discreto (Kpos, Kneg) usando Tustin.
 *
 * @details
 * Equivalente a la discretización realizada en MATLAB con:
 * @code
 *   controladorZ = c2d(controlador, Tm, 'tustin');
 * @endcode
 *
 * @note
 * Se llama:
 *  - En @ref setup(), para inicializar el controlador.
 *  - Cuando se actualizan @ref Kp o @ref Ti vía MQTT.
 * @ingroup CONTROL
 */
void recalculateCoefficients() {
  if (Ti <= 0.0f || Kp <= 0.0f) {
    Serial.println("[PI] ERROR: Kp o Ti no válidos. Coeficientes no actualizados.");
    return;
  }

  const float T_factor = Tm / (2.0f * Ti);  // = 1/(2*Ti) con Tm=1

  // coeficientes numéricos del PI en Tustin:
  // C(z) = (b0 + b1 z^-1) / (1 - z^-1)
  Kpos = Kp * (1.0f + T_factor);      // b0
  Kneg = -Kp * (1.0f - T_factor);     // b1

  Serial.printf("[PI] Kp=%.3f, Ti=%.3f, Tm=%.3f -> Kpos=%.6f, Kneg=%.6f\r\n",
                Kp, Ti, Tm, Kpos, Kneg);
}

/**
 * @brief  Aplica un ángulo al servo, saturando al rango permitido e invirtiendo el sentido.
 *
 * @param[in] deg  Ángulo deseado en grados.
 *
 * @details
 * La función:
 *  - Satura @p deg al rango [SERVO_DEG_MIN, SERVO_DEG_MAX].
 *  - Actualiza la variable global @ref pos_deg.
 *  - Llama a @ref servo.write() con el ángulo invertido (180 - deg) según el montaje físico.
 * @ingroup CONTROL
 */
void setServoDegrees(float deg) {
  if (deg < SERVO_DEG_MIN) deg = SERVO_DEG_MIN;
  if (deg > SERVO_DEG_MAX) deg = SERVO_DEG_MAX;

  pos_deg = deg;
  servo.write(int(180)-(int)roundf(deg));
}

/**
 * @brief  Publica el estado actual del actuador a través de MQTT.
 *
 * @details
 * Se publican:
 *  - @ref pos_deg en @ref MQTT_PUB_TOPIC_POS (con 1 decimal).
 *  - @ref u_logic en @ref MQTT_PUB_TOPIC_U (con 3 decimales).
 *
 * @note Los mensajes se publican con la marca @c retained = true.
 * @ingroup COMMS_MQTT
 */
static void publishState() {
  if (mqtt.connected()) {
    // Posición del servo EN GRADOS
    String s_deg = String(pos_deg, 1);
    mqtt.publish(MQTT_PUB_TOPIC_POS, s_deg.c_str(), true);
    
    // Mando lógico U (0..100 %)
    String s_u = String(u_logic, 3);
    mqtt.publish(MQTT_PUB_TOPIC_U, s_u.c_str(), true);
  }
}

/**
 * @brief  Aplica un paso del controlador PI discreto y actualiza el servo.
 *
 * @details
 * Implementa el algoritmo recursivo:
 * @f[
 *   u[k] = u[k-1] + Kpos \cdot e[k] + Kneg \cdot e[k-1]
 * @f]
 * donde @f$ e[k] = SP - PV @f$.
 *
 * Luego:
 *  - Se satura @f$ u[k] @f$ al rango 0..100 %.
 *  - Se actualizan @ref error_k_1 y @ref u_logic.
 *  - Se convierte @f$ u[k] @f$ a grados de servo y se llama a @ref setServoDegrees().
 * @ingroup CONTROL
 */
void applyControl() {
  float e_k = temp_setpoint - temp_actual;

  // u[k] = u[k-1] + Kpos*e[k] + Kneg*e[k-1]
  float u_new = u_logic + (Kpos * e_k) + (Kneg * error_k_1);

  // saturación 0..100 % lógico
  if (u_new < 0.0f)   u_new = 0.0f;
  if (u_new > 100.0f) u_new = 100.0f;

  // actualizar estados
  error_k_1 = e_k;
  u_logic   = u_new;

  // mapear 0..100 % a grados reales
  float deg = scaleToDegrees(u_new);
  setServoDegrees(deg);
}

// ================================================== CONEXIÓN MQTT ==================================================

/**
 * @brief  Establece la conexión con el broker MQTT y suscribe los tópicos de entrada.
 *
 * @retval true   Conexión exitosa y suscripción completada.
 * @retval false  Fallo al conectar con el broker.
 *
 * @note
 * El identificador de cliente MQTT se construye a partir de la MAC del ESP32-C6.
 * Solo se suscriben los tópicos de entrada (SP, PV, Kp, Ti). Las salidas se publican
 * mediante @ref publishState().
 * @ingroup COMMS_MQTT
 */
static bool connectMQTT() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  if (mqtt.connected()) return true;

  String clientId = String("esp32c6-servo-") + String((uint32_t)ESP.getEfuseMac(), HEX);
  Serial.printf("[MQTT] Conectando a %s:%u ...\r\n", MQTT_HOST, MQTT_PORT);

  bool ok;
  if (MQTT_USER && MQTT_USER[0] != '\0') {
    ok = mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS);
  } else {
    ok = mqtt.connect(clientId.c_str());
  }

  if (!ok) {
    Serial.printf("[MQTT] Fallo rc=%d\r\n", mqtt.state());
    return false;
  }

  Serial.println("[MQTT] Conectado, suscribiendo tópicos...");

  // Suscribiendo solo a los tópicos de entrada (SP, PV, Kp, Ti)
  mqtt.subscribe(MQTT_SUB_TOPIC_SP, 1);
  mqtt.subscribe(MQTT_SUB_TOPIC_PV, 1);
  mqtt.subscribe(MQTT_SUB_TOPIC_KP, 1);
  mqtt.subscribe(MQTT_SUB_TOPIC_TI, 1);

  // Al conectar, se publica el estado actual.
  publishState();
  return true;
}

/**
 * @brief  Callback de MQTT para procesar mensajes entrantes.
 *
 * @param[in] topic    Tópico MQTT recibido (cadena terminada en '\\0').
 * @param[in] payload  Puntero al buffer con los datos del mensaje.
 * @param[in] length   Longitud del payload en bytes.
 *
 * @details
 * La función intenta interpretar el payload como:
 *  - Número ASCII (formato texto).
 *  - Double (8 bytes) o float (4 bytes) en binario.
 *
 * Si el valor es válido y finito:
 *  - En @ref MQTT_SUB_TOPIC_KP y @ref MQTT_SUB_TOPIC_TI actualiza Kp y Ti y recalcúla
 *    los coeficientes del controlador.
 *  - En @ref MQTT_SUB_TOPIC_SP actualiza la consigna de temperatura.
 *  - En @ref MQTT_SUB_TOPIC_PV actualiza la temperatura de proceso.
 * @ingroup COMMS_MQTT
 */
static void mqttCallback(char* topic, byte* payload, unsigned int length) {
  float value = NAN;

  // 1) ¿Parece número en ASCII?
  bool isAsciiNumber = true;
  unsigned int n_check = length;
  if (n_check > 64) n_check = 64;

  for (unsigned int i = 0; i < n_check; ++i) {
    char c = (char)payload[i];
    if (!isDigit(c) && c != '.' && c != '-' && c != '+' &&
        c != 'e' && c != 'E' && c != ' ') {
      isAsciiNumber = false;
      break;
    }
  }

  if (isAsciiNumber) {
    String s;
    s.reserve(length);
    for (unsigned int i = 0; i < length; ++i) s += (char)payload[i];
    s.trim();
    value = s.toFloat();
    Serial.printf("[MQTT-CB] ASCII '%s' -> %f\r\n", s.c_str(), value);
  }
  else if (length >= 8) {
    double d;
    memcpy(&d, payload, 8);
    value = (float)d;
    Serial.printf("[MQTT-CB] Binario (>=8) -> double=%f (len=%u)\r\n", value, length);
  }
  else if (length >= 4) {
    float f;
    memcpy(&f, payload, 4);
    value = f;
    Serial.printf("[MQTT-CB] Binario (>=4) -> float=%f (len=%u)\r\n", value, length);
  }
  else {
    Serial.printf("[MQTT-CB] Longitud muy corta (%u), no interpreto.\r\n", length);
    return;
  }

  if (!isfinite(value)) {
    Serial.println("[MQTT-CB] Valor no válido (NaN/Inf)");
    return;
  }

  // --- Sintonización ---
  if (strcmp(topic, MQTT_SUB_TOPIC_KP) == 0) {
    if (value > 0.0f) {
      Kp = value;
      recalculateCoefficients();
    }
    return;
  }
  if (strcmp(topic, MQTT_SUB_TOPIC_TI) == 0) {
    if (value > 0.0f) {
      Ti = value;
      recalculateCoefficients();
    }
    return;
  }

  // --- Control ---
  if (strcmp(topic, MQTT_SUB_TOPIC_SP) == 0) {
    temp_setpoint = value;
    Serial.printf("[MQTT-CB] Nuevo SP = %.3f\r\n", temp_setpoint);
  }
  else if (strcmp(topic, MQTT_SUB_TOPIC_PV) == 0) {
    temp_actual = value;
    Serial.printf("[MQTT-CB] Nueva PV = %.3f\r\n", temp_actual);
  }
  else {
    Serial.printf("[MQTT-CB] Tópico no manejado: %s\r\n", topic);
  }
}

// ================================================== TAREAS FREERTOS ==================================================

/**
 * @brief  Tarea de control discreto (PI + servo + logging).
 *
 * @param[in] parameter  Puntero genérico no utilizado (debe ser @c NULL).
 *
 * @details
 * La tarea:
 *  - Se ejecuta periódicamente cada @ref CONTROL_INTERVAL_MS milisegundos
 *    mediante @ref vTaskDelayUntil().
 *  - Verifica que exista conexión WiFi y MQTT antes de aplicar el control.
 *  - Aplica la lógica de reseteo inicial del PI cuando @ref sampleIndex == 5:
 *      - Fuerza @ref u_logic = 50 %.
 *      - Resetea @ref error_k_1.
 *      - Igual @ref temp_actual a @ref temp_setpoint.
 *  - Llama a @ref applyControl() y @ref publishState().
 *  - Envía al puerto serie un log en formato CSV:
 *      @code
 *      k,t,SP,PV,U_logic,Deg
 *      @endcode
 *  - Genera un breve pulso del LED para indicar actividad de control.
 *
 * @note Esta tarea nunca retorna.
 * @ingroup RTOS_TASKS
 */
void control_task(void * parameter) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency     = pdMS_TO_TICKS(CONTROL_INTERVAL_MS);
  const TickType_t xLedPulseDelay = pdMS_TO_TICKS(5); // pulso LED corto

  xLastWakeTime = xTaskGetTickCount();

  while (true) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    if (WiFi.status() == WL_CONNECTED && mqtt.connected()) {
      
      // --- LÓGICA DE RESET INICIAL ---
      if (sampleIndex == 5 && !initial_state_reset) {
        Serial.println("--- [PI Reset] Forzando u_logic = 50% y e[k-1] = 0.0 para inicio limpio ---");
        
        // Resetea el integrador al valor de polarización (50%)
        u_logic = 50.0f;      
        
        // Resetea el error anterior
        error_k_1 = 0.0f;     
        initial_state_reset = true;
        
        // Lleva la PV al SP para iniciar en estado estacionario
        temp_actual = temp_setpoint; 
      }
      // ------------------------------------------

      uint8_t r_prev = (current_color >> 16) & 0xFF;
      uint8_t g_prev = (current_color >> 8)  & 0xFF;
      uint8_t b_prev =  current_color        & 0xFF;

      applyControl();
      publishState();

      // ----- LOG PARA MATLAB -----
      unsigned long k = sampleIndex++;
      float t = k * Tm;   // Tm = 1.0 s

      Serial.printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
                    k,
                    t,
                    temp_setpoint,
                    temp_actual,
                    u_logic,   // mando lógico 0..100 %
                    pos_deg);  // posición en grados

      // pulso de LED
      setLedColor(0, 0, RGB_BRIGHTNESS);
      vTaskDelay(xLedPulseDelay);
      setLedColor(r_prev, g_prev, b_prev);
    }
  }
}

/**
 * @brief  Tarea de comunicaciones (gestión de WiFi y MQTT).
 *
 * @param[in] parameter  Puntero genérico no utilizado (debe ser @c NULL).
 *
 * @details
 * La tarea:
 *  - Supervisa permanentemente el estado de la conexión WiFi.
 *  - En caso de desconexión:
 *      - Muestra el LED en rojo.
 *      - Registra mensajes de diagnóstico por serie.
 *      - Reintenta la conexión WiFi con un período definido.
 *  - Cuando hay conexión WiFi:
 *      - Muestra el LED en verde.
 *      - Verifica y restablece la conexión MQTT si es necesario.
 *      - Llama periódicamente a @ref mqtt.loop().
 *
 * @note Si se pierde la comunicación con el broker, el lazo de control
 *       se congela en el último estado conocido, ya que los datos
 *       de SP, PV y sintonía se encuentran centralizados en el broker.
 * @ingroup RTOS_TASKS
 * @ingroup COMMS_MQTT
 */
void mqtt_task(void * parameter) {
  uint32_t lastPrint = 0;
  uint32_t lastReconnect = 0;

  while (true) {
    wl_status_t st = WiFi.status();

    if (st != WL_CONNECTED) {
      setLedColor(RGB_BRIGHTNESS, 0, 0);

      if (millis() - lastPrint > 2000) {
        Serial.printf("[WiFi] No conectado. status=%d\r\n", st);
        lastPrint = millis();
      }

      if (millis() - lastReconnect > 5000) {
        Serial.println("[WiFi] Reintentando WiFi.reconnect()");
        WiFi.reconnect();
        lastReconnect = millis();
      }
    }
    else {
      setLedColor(0, RGB_BRIGHTNESS, 0);

      if (!mqtt.connected()) {
        connectMQTT();
      }
      mqtt.loop();
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ================================================== SETUP Y LOOP ARDUINO ==================================================

/**
 * @brief  Función de inicialización del firmware.
 *
 * @details
 * Realiza:
 *  - Inicialización del puerto serie y encabezado de log CSV.
 *  - Inicialización del LED integrado (apagado).
 *  - Cálculo inicial de los coeficientes del PI discreto.
 *  - Configuración y posicionamiento inicial del servo.
 *  - Conexión inicial a la red WiFi con timeout de 10 s.
 *  - Configuración del cliente MQTT y registro del callback.
 *  - Creación de las tareas FreeRTOS:
 *      - @ref control_task con prioridad 3.
 *      - @ref mqtt_task con prioridad 2.
 *
 * @note La lógica de control y comunicaciones queda delegada por completo a las tareas.
 * @ingroup RTOS_TASKS
 */
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("\r\n--- ESP32-C6 - PI + MQTT + Servo (RTOS, grados, log CSV) ---");

  // Header CSV para MATLAB
  Serial.println("k,t,SP,PV,U_logic,Deg");

  // Configuración del LED integrado (según la placa es posible que se requiera pinMode)
  //pinMode(RGB_BUILTIN, OUTPUT);
  setLedColor(0, 0, 0);

  // Inicializar coeficientes del PI
  recalculateCoefficients();

  // Configuración del servo
  servo.setPeriodHertz(SERVO_HZ);
  servo.attach(SERVO_PIN, MIN_US, MAX_US);

  // posición inicial: u_logic = 25 % (se ajustará a 50 % en el reset inicial)
  float deg0 = scaleToDegrees(u_logic);
  setServoDegrees(deg0);

  // Configuración WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[WiFi] Conectando a '%s'...\r\n", WIFI_SSID);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 10000) {
    delay(500); // NOTA: Este delay en setup() puede causar un WDT si la conexión tarda demasiado.
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] Conectado. IP: %s\r\n", WiFi.localIP().toString().c_str());
    setLedColor(0, RGB_BRIGHTNESS, 0);
  } else {
    Serial.println("[WiFi] NO se pudo conectar en el arranque (timeout).");
    setLedColor(RGB_BRIGHTNESS, 0, 0);
  }

  // Configuración MQTT
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);

  Serial.println("Inicializando tareas FreeRTOS...");

  // Tarea de control PI
  xTaskCreate(
    control_task,
    "ControlPI",
    4096,
    NULL,
    3,
    NULL
  );

  // Tarea de comunicaciones MQTT
  xTaskCreate(
    mqtt_task,
    "CommsMQTT",
    8192,
    NULL,
    2,
    NULL
  );
}

/**
 * @brief  Bucle principal de Arduino (no utilizado).
 *
 * @details
 * Todas las funcionalidades del sistema están implementadas en las tareas FreeRTOS.
 * Esta función permanece vacía.
 */
void loop() {
  // vacío: todo lo manejan las tareas
}
