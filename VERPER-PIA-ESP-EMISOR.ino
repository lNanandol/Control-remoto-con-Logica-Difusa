#include <WiFi.h>
#include <esp_now.h>

// Estructura de datos para enviar la variable
typedef struct {
  int miVariable1;
  int miVariable2;
  int miVariable3;
  int miVariable4;
} datos_t;
.0
// Definir el receptor ESP-NOW
esp_now_peer_info_t receptor;

// Función de envío de datos
void enviarDatos() {
  datos_t datos;
  // Variables a enviar
  datos.miVariable1 = analogRead(34);  
  datos.miVariable2 = analogRead(35);
  datos.miVariable3 = analogRead(32);
  datos.miVariable4 = analogRead(33);

  // Enviar datos
  esp_now_send(receptor.peer_addr, (uint8_t*)&datos, sizeof(datos));
}

void setup() {
  // Inicializar ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error inicializando ESP-NOW");
    return;
  }

  // Agregar receptor
  memset(&receptor, 0, sizeof(receptor));
  memcpy(receptor.peer_addr, "CC:DB:A7:68:FA:54", 6);  // Dirección MAC del receptor
  receptor.channel = 0;
  receptor.encrypt = false;
  if (esp_now_add_peer(&receptor) != ESP_OK) {
    Serial.println("Error agregando el receptor");
    return;
  }

  // Configurar función de envío de datos
  esp_now_register_send_cb([](const uint8_t* mac, esp_now_send_status_t sendStatus) {
    if (sendStatus == ESP_NOW_SEND_SUCCESS) {
      Serial.println("Datos enviados con éxito");
    } else {
      Serial.println("Error al enviar los datos");
    }
  });
}

void loop() {
  enviarDatos();
  delay(50);  // Esperar 100 milisegundos antes de enviar los datos nuevamente
}
