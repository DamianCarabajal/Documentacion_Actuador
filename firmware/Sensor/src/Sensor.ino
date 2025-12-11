/**
 * @file    main.cpp
 * @brief   Nodo de sensado IoT basado en ESP8266 con MQTT y LWT.
 *
 * @details
 * Este firmware implementa un nodo de sensado para una sala específica
 * (Sala de Profes) utilizando:
 *  - Conexión WiFi.
 *  - Comunicación MQTT con Last Will and Testament (LWT) para indicar estado
 *    Online/Offline de forma automática.
 *  - Sensor DHT11 para medir temperatura y humedad relativa.
 *
 * El nodo publica periódicamente la medición en tópicos MQTT y actualiza
 * un tópico de estado que permite a otros componentes del sistema detectar
 * pérdidas de conexión.
 */

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

/**
 * @defgroup NODE_SENSOR Nodo de Sensado
 * @brief   Nodo IoT encargado de la adquisición de temperatura y humedad.
 *
 * Este nodo:
 *  - Se conecta a una red WiFi de laboratorio.
 *  - Resuelve dinámicamente la IP del broker MQTT (por nombre de host).
 *  - Publica periódicamente temperatura y humedad en tópicos MQTT específicos.
 *  - Utiliza LWT (Last Will and Testament) para indicar automáticamente
 *    el estado "Offline" cuando se pierde la conexión con el broker.
 */

// =======================================================================
// === BLOQUE DE CONFIGURACIÓN ===
// =======================================================================

/**
 * @name Configuración WiFi
 * @{
 */

/**
 * @brief  SSID de la red WiFi utilizada por el nodo de sensado.
 * @ingroup NODE_SENSOR
 */
const char* ssid = "Alumnos 2G";

/**
 * @brief  Contraseña de la red WiFi (vacía en este entorno de laboratorio).
 * @ingroup NODE_SENSOR
 */
const char* password = "";

/** @} */

/**
 * @name Configuración MQTT
 * @{
 */

/**
 * @brief  Nombre de host del broker MQTT (resuelto por DNS local).
 *
 * @details
 * Se utiliza @ref WiFi.hostByName() para resolver este nombre a una dirección IP
 * antes de configurar el cliente MQTT.
 * @ingroup NODE_SENSOR
 */
const char* mqtt_host = "FeDaMi.local"; 

/**
 * @brief  Puerto TCP del broker MQTT.
 * @ingroup NODE_SENSOR
 */
const int   mqtt_port = 1883;

/**
 * @brief  Identificador de cliente MQTT para este nodo (debe ser único).
 * @ingroup NODE_SENSOR
 */
const char* mqtt_client_id = "ESP01S_Sensor_Temp_Sala-de-Profes";

/** @} */

/**
 * @name Configuración de tópicos MQTT
 * @brief Tópicos utilizados por el nodo para publicar mediciones y estado.
 * @{
 */

/**
 * @brief  Tópico base asociado a la sala de profesores.
 *
 * @note Es sensible a mayúsculas/minúsculas, debe coincidir con la configuración
 *       en Node-RED.
 * @ingroup NODE_SENSOR
 */
String TOPICO_BASE = "universidad/piso3/sala_de_profes";

/**
 * @brief  Tópico donde se publica la temperatura medida (°C).
 * @ingroup NODE_SENSOR
 */
String TOPIC_TEMPERATURA = TOPICO_BASE + "/temperatura";

/**
 * @brief  Tópico donde se publica la humedad medida (%HR).
 * @ingroup NODE_SENSOR
 */
String TOPIC_HUMEDAD     = TOPICO_BASE + "/humedad";

/**
 * @brief  Tópico donde se publica el estado del nodo ("Online", "Offline", etc.).
 *
 * @details
 * Se utiliza tanto para el Last Will and Testament (LWT) como para indicar
 * estados de error del sensor.
 * @ingroup NODE_SENSOR
 */
String TOPIC_STATUS      = TOPICO_BASE + "/status";

/** @} */

/**
 * @name Configuración del sensor DHT
 * @{
 */

/**
 * @brief  Pin GPIO utilizado para el sensor DHT11.
 * @ingroup NODE_SENSOR
 */
const int DHT_PIN = 2;    // GPIO2

/**
 * @brief  Tipo de sensor DHT utilizado (DHT11 en este caso).
 * @ingroup NODE_SENSOR
 */
#define DHT_TIPO DHT11    

/** @} */

/**
 * @name Configuración de tiempos
 * @{
 */

/**
 * @brief  Intervalo entre publicaciones MQTT, en milisegundos.
 *
 * @details
 * En este firmware se utiliza un periodo de 60 s (1 minuto) entre
 * mediciones y publicaciones.
 * @ingroup NODE_SENSOR
 */
const long intervalo_publicacion = 60000; // 1 minuto

/** @} */

// =======================================================================
// === VARIABLES GLOBALES ===
// =======================================================================

/**
 * @brief  Cliente TCP/IP para la pila WiFi del ESP8266.
 * @ingroup NODE_SENSOR
 */
WiFiClient espClient;

/**
 * @brief  Cliente MQTT basado en PubSubClient.
 * @ingroup NODE_SENSOR
 */
PubSubClient client(espClient);

/**
 * @brief  Objeto de sensor DHT (temperatura y humedad).
 * @ingroup NODE_SENSOR
 */
DHT dht(DHT_PIN, DHT_TIPO);

/**
 * @brief  Marca de tiempo (millis) de la última publicación realizada.
 * @ingroup NODE_SENSOR
 */
long ultimaPublicacion = 0;

/**
 * @brief  Buffer temporal para formatear las mediciones antes de publicarlas.
 * @ingroup NODE_SENSOR
 */
char msgBuffer[10];

/**
 * @brief  Dirección IP resuelta del servidor MQTT (a partir de @ref mqtt_host).
 * @ingroup NODE_SENSOR
 */
IPAddress mqtt_server_ip;

// =======================================================================
// === FUNCIONES DE CONEXIÓN Y COMUNICACIÓN ===
// =======================================================================

/**
 * @brief  Establece la conexión con la red WiFi configurada.
 *
 * @details
 * La función:
 *  - Inicia la conexión WiFi mediante @ref WiFi.begin().
 *  - Espera hasta obtener el estado @c WL_CONNECTED.
 *  - Muestra por puerto serie la IP asignada al ESP8266.
 *
 * @ingroup NODE_SENSOR
 */
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado!");
  Serial.print("IP del ESP: ");
  Serial.println(WiFi.localIP());
}

/**
 * @brief  Resuelve la IP del servidor MQTT a partir del nombre de host.
 *
 * @details
 * Se llama a @ref WiFi.hostByName() en un bucle hasta obtener una IP válida.
 * Una vez resuelta la dirección, se configura el cliente MQTT con
 * @ref PubSubClient::setServer().
 *
 * @note Esta función bloquea mientras no se pueda resolver el host.
 * @ingroup NODE_SENSOR
 */
void resolver_ip_servidor() {
  Serial.print("Buscando IP de: ");
  Serial.println(mqtt_host);
  
  while (WiFi.hostByName(mqtt_host, mqtt_server_ip) == 0) {
    Serial.print("No se encuentra '");
    Serial.print(mqtt_host);
    Serial.println("'. Reintentando...");
    delay(2000);
  }
  
  Serial.print("¡Servidor encontrado! IP: ");
  Serial.println(mqtt_server_ip);
  client.setServer(mqtt_server_ip, mqtt_port);
}

/**
 * @brief  Establece o restablece la conexión MQTT utilizando LWT.
 *
 * @details
 * La función intenta conectarse al broker MQTT en un bucle hasta lograrlo.
 * Se configura un Last Will and Testament (LWT) sobre @ref TOPIC_STATUS
 * con el mensaje "Offline" y:
 *  - QoS = 1
 *  - Retain = true
 *
 * Una vez conectados, el nodo publica inmediatamente el estado "Online"
 * en el mismo tópico con retain, sobrescribiendo el mensaje "Offline".
 *
 * @ingroup NODE_SENSOR
 */
void reconnect_mqtt() {
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    
    // Sintaxis: connect(ID, User, Pass, TopicoWill, QoS, Retain, MensajeWill)
    if (client.connect(mqtt_client_id, NULL, NULL, TOPIC_STATUS.c_str(), 1, true, "Offline")) {
      
      Serial.println("¡Conectado!");
      // Apenas nos conectamos, pisamos el mensaje "Offline" con un "Online" (Retain=true)
      client.publish(TOPIC_STATUS.c_str(), "Online", true);
      
    } else {
      Serial.print("falló, rc=");
      Serial.print(client.state());
      Serial.println(" reintentando en 5s");
      delay(5000);
    }
  }
}

// =======================================================================
// === SETUP Y LOOP PRINCIPAL ===
// =======================================================================

/**
 * @brief  Función de inicialización del nodo de sensado.
 *
 * @details
 * Realiza:
 *  - Inicialización del puerto serie.
 *  - Inicialización del sensor DHT.
 *  - Conexión a la red WiFi.
 *  - Resolución de la IP del servidor MQTT.
 *
 * La conexión MQTT se gestiona posteriormente en @ref loop() mediante
 * @ref reconnect_mqtt().
 *
 * @ingroup NODE_SENSOR
 */
void setup() {
  Serial.begin(115200);
  dht.begin();
  setup_wifi();
  resolver_ip_servidor();
}

/**
 * @brief  Bucle principal del nodo de sensado.
 *
 * @details
 * En cada iteración:
 *  - Verifica la conexión MQTT y la restablece si es necesario.
 *  - Llama a @ref PubSubClient::loop() para mantener viva la conexión.
 *  - Cada @ref intervalo_publicacion milisegundos:
 *      - Lee temperatura y humedad del sensor DHT.
 *      - Verifica lecturas válidas.
 *      - Publica temperatura en @ref TOPIC_TEMPERATURA (con 1 decimal).
 *      - Publica humedad en @ref TOPIC_HUMEDAD (entero).
 *      - En caso de error de lectura, publica "ErrorSensor" en @ref TOPIC_STATUS.
 *
 * Todos los mensajes MQTT se envían con la marca @c retained = true
 * para que el último valor quede disponible al suscribirse.
 *
 * @ingroup NODE_SENSOR
 */
void loop() {
  if (!client.connected()) {
    reconnect_mqtt();
  }
  client.loop();

  long now = millis();
  if (now - ultimaPublicacion > intervalo_publicacion) {
    ultimaPublicacion = now;

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t)) {
      Serial.println("Error sensor DHT11");
      client.publish(TOPIC_STATUS.c_str(), "ErrorSensor");
      return;
    }

    // Temperatura (1 decimal)
    snprintf(msgBuffer, 10, "%.1f", t);
    client.publish(TOPIC_TEMPERATURA.c_str(), msgBuffer, true); // retain=true
    Serial.print("Temp: "); Serial.println(msgBuffer);

    // Humedad (entero)
    snprintf(msgBuffer, 10, "%.0f", h);
    client.publish(TOPIC_HUMEDAD.c_str(), msgBuffer, true);
    Serial.print("Hum: "); Serial.println(msgBuffer);
  }
}
