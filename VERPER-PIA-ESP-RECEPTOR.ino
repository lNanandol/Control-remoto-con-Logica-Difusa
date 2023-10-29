#include <Fuzzy.h>
#include "WiFi.h"
#include "esp_now.h"
#include <ESP32Servo.h>

// Instancia de un objeto Fuzzy
Fuzzy *fuzzy = new Fuzzy();

// Instancia de los ServoMotores
Servo servo1;
Servo servo2;
int pinServo1 = 25;
int pinServo2 = 13;

// Declaracion de los pines para el puente H
// Motor A
int motor1Pin1 = 27; 
int motor1Pin2 = 26; 
int enable1Pin1 = 14; 

// Motor B
int motor1Pin3 = 32; 
int motor1Pin4 = 33; 
int enable1Pin2 = 12;

//Declaracion de las variables de operacion para el PWM
const int freq = 30000;
const int pwmChannel0 = 14;
const int pwmChannel2 = 15;
const int resolution = 8;

// Declaracion de las variables recibidas
int variableRecibida1 = 0;
int variableRecibida2 = 0;
int variableRecibida3 = 0;
int variableRecibida4 = 0;

// Declaracion de las variables de intensidad de corriente
int intensidad1 = 0;
int intensidad2 = 0;
int intensidad3 = 0;
int intensidad4 = 0;

// Estructura de datos para recibir la variable
typedef struct {
  int miVariable1;
  int miVariable2;
  int miVariable3;
  int miVariable4;
} datos_t;

// Función de recepción de datos
void recibirDatos(const uint8_t* mac, const uint8_t* datos, int len) {
  if (len == sizeof(datos_t)) {
    datos_t datosRecibidos;
    memcpy(&datosRecibidos, datos, sizeof(datosRecibidos));
    variableRecibida1 = datosRecibidos.miVariable1;
    variableRecibida2 = datosRecibidos.miVariable2;
    variableRecibida3 = datosRecibidos.miVariable3;
    variableRecibida4 = datosRecibidos.miVariable4;
  }
}

void setup()
{
  // Establece la salida Serial
  Serial.begin(115200);

  // Configuracion de salidas del puente H
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin1, OUTPUT);

  pinMode(motor1Pin3, OUTPUT);
  pinMode(motor1Pin4, OUTPUT);
  pinMode(enable1Pin2, OUTPUT);
  
  //Configuracion de salida para PWM de los motores
  ledcSetup(pwmChannel0, freq, resolution);
  ledcSetup(pwmChannel2, freq, resolution);

  // Configurar los canales para el control de salida 
  ledcAttachPin(enable1Pin1, pwmChannel0);
  ledcAttachPin(enable1Pin2, pwmChannel2);

  // Configuracion de los servomotores 
  servo1.attach(pinServo1, 500, 2500);
  servo2.attach(pinServo2, 500, 2500);

  // Inicializar ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error inicializando ESP-NOW");
    return;
  }

  // Configurar función de recepción de datos
  esp_now_register_recv_cb(recibirDatos);

  // Instanciando un objeto FuzzyInput
  FuzzyInput *potValue = new FuzzyInput(1);
  
  // Instanciando un objeto FuzzySet
  FuzzySet *low = new FuzzySet(0, 0, 41, 1536);  // Define un conjunto difuso "bajo"
  potValue->addFuzzySet(low);  // Incluye el conjunto difuso en la entrada difusa
  
  FuzzySet *medium = new FuzzySet(512, 2048, 2048, 3583);  // Define un conjunto difuso "medio"
  potValue->addFuzzySet(medium);  // Incluye el conjunto difuso en la entrada difusa
  
  FuzzySet *high = new FuzzySet(2560, 4056, 4096, 4096);  // Define un conjunto difuso "alto"
  potValue->addFuzzySet(high);  // Incluye el conjunto difuso en la entrada difusa
  
  fuzzy->addFuzzyInput(potValue);

  // Instanciando un objeto FuzzyOutput
  FuzzyOutput *speed = new FuzzyOutput(1);
  
  FuzzySet *slow = new FuzzySet(0, 0, 1, 38);
  speed->addFuzzySet(slow);
  
  FuzzySet *average = new FuzzySet(12, 50, 50, 87);
  speed->addFuzzySet(average);
  
  FuzzySet *fast = new FuzzySet(62, 99, 100, 100);
  speed->addFuzzySet(fast);
  
  fuzzy->addFuzzyOutput(speed);

  // Construcción de la regla difusa "IF potValue = low THEN speed = slow"
  FuzzyRuleAntecedent *ifPotValueLow = new FuzzyRuleAntecedent();
  ifPotValueLow->joinSingle(low);
  FuzzyRuleConsequent *thenSpeedSlow = new FuzzyRuleConsequent();
  thenSpeedSlow->addOutput(slow);
  FuzzyRule *fuzzyRule01 = new FuzzyRule(1, ifPotValueLow, thenSpeedSlow);
  fuzzy->addFuzzyRule(fuzzyRule01);

  // Construcción de la regla difusa "IF potValue = medium THEN speed = average"
  FuzzyRuleAntecedent *ifPotValueMedium = new FuzzyRuleAntecedent();
  ifPotValueMedium->joinSingle(medium);
  FuzzyRuleConsequent *thenSpeedAverage = new FuzzyRuleConsequent();
  thenSpeedAverage->addOutput(average);
  FuzzyRule *fuzzyRule02 = new FuzzyRule(2, ifPotValueMedium, thenSpeedAverage);
  fuzzy->addFuzzyRule(fuzzyRule02);

  // Construcción de la regla difusa "IF potValue = high THEN speed = fast"
  FuzzyRuleAntecedent *ifPotValueHigh = new FuzzyRuleAntecedent();
  ifPotValueHigh->joinSingle(high);
  FuzzyRuleConsequent *thenSpeedFast = new FuzzyRuleConsequent();
  thenSpeedFast->addOutput(fast);
  FuzzyRule *fuzzyRule03 = new FuzzyRule(3, ifPotValueHigh, thenSpeedFast);
  fuzzy->addFuzzyRule(fuzzyRule03);
}

void loop()
{ 
  ////////////////////////////////// Potenciometro1 y Motor1 ////////////////////////////////////////////
  // Imprimiendo la entrada
  /*Serial.println("\n\n\nEntrada 1: ");
  Serial.print("\t\t\tValor del Potenciómetro 1: ");
  Serial.println(variableRecibida1);*/
  
  // Estableciendo el valor del potenciómetro como entrada
  fuzzy->setInput(1, variableRecibida1);
  
  // Ejecutando la Fuzzificación
  fuzzy->fuzzify();
  
  // Ejecutando la Defuzzificación
  float Motor1 = fuzzy->defuzzify(1);
  
  // Imprimiendo el resultado
  /*Serial.println("Resultado 1: ");
  Serial.print("\t\t\tVelocidad 1: ");
  Serial.println(Motor1);*/

  ////////////////////////////////// Potenciometro2 y Motor2 ////////////////////////////////////////////
  // Imprimiendo la entrada
  /*Serial.println("\n\n\nEntrada 2: ");
  Serial.print("\t\t\tValor del  2: ");
  Serial.println(variableRecibida2);*/
  
  // Estableciendo el valor del potenciómetro como entrada
  fuzzy->setInput(1, variableRecibida2);
  
  // Ejecutando la Fuzzificación
  fuzzy->fuzzify();
  
  // Ejecutando la Defuzzificación
  float Motor2 = fuzzy->defuzzify(1);
  
  // Imprimiendo el resultado
  /*Serial.println("Resultado 2: ");
  Serial.print("\t\t\tVelocidad 2: ");
  Serial.println(Motor2);*/

  ////////////////////////////////// Potenciometro3 y ServoMotor1 ////////////////////////////////////////////
  // Imprimiendo la entrada
  /*Serial.println("\n\n\nEntrada 3: ");
  Serial.print("\t\t\tValor del Potenciómetro 3: ");
  Serial.println(variableRecibida3);*/
  
  // Estableciendo el valor del potenciómetro como entrada
  fuzzy->setInput(1, variableRecibida3);
  
  // Ejecutando la Fuzzificación
  fuzzy->fuzzify();
  
  // Ejecutando la Defuzzificación
  float ServoMotor1 = fuzzy->defuzzify(1);
  
  // Imprimiendo el resultado
  /*Serial.println("Resultado 3: ");
  Serial.print("\t\t\tVelocidad 3: ");
  Serial.println(ServoMotor1);*/

  ////////////////////////////////// Potenciometro4 y ServoMotor2 ////////////////////////////////////////////
  // Imprimiendo la entrada
  /*Serial.println("\n\n\nEntrada 4: ");
  Serial.print("\t\t\tValor del Potenciómetro 4: ");
  Serial.println(variableRecibida4);*/
  
  // Estableciendo el valor del potenciómetro como entrada
  fuzzy->setInput(1, variableRecibida4);
  
  // Ejecutando la Fuzzificación
  fuzzy->fuzzify();
  
  // Ejecutando la Defuzzificación
  float ServoMotor2 = fuzzy->defuzzify(1);
  
  // Imprimiendo el resultado
  /*Serial.println("Resultado 4: ");
  Serial.print("\t\t\tVelocidad 4: ");
  Serial.println(ServoMotor2);*/

  // Mapear la salida difusa a un rango de 0 a 255
  int intensidad1 = map(Motor1, 0, 100, 0, 255);
  int intensidad2 = map(Motor2, 0, 100, 0, 255);
  
  // Mapear la salida difusa a un rango de 0 a 180
  int intensidad3 = map(ServoMotor1, 0, 100, 0, 180);
  int intensidad4 = map(ServoMotor2, 0, 100, 80, 100);

  // Establecer los grados de los servos 
  servo1.write(intensidad3);
  servo2.write(intensidad4);

  // Control de los motores mediante el puente H
  // Giro del Motor Derecho
  if (intensidad1 > 51){ 
    Serial.println("Giro del Motor Derecho");
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH); 
    ledcWrite(pwmChannel0, 200); 
  }else{
    Serial.println("Paro del Motor Derecho");
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW); 
    ledcWrite(pwmChannel0, 0); 
  }

  // Giro del Motor Izquierdo
  if (intensidad2 > 51){ 
    Serial.println("Giro del Motor Izquierdo");
    digitalWrite(motor1Pin3, LOW);
    digitalWrite(motor1Pin4, HIGH); 
    ledcWrite(pwmChannel2, 200); 
  }else{
    Serial.println("Paro del Motor Izquierdo");
    digitalWrite(motor1Pin3, LOW);
    digitalWrite(motor1Pin4, LOW); 
    ledcWrite(pwmChannel2, 0); 
  }

  // Espera 100 milisegundos
  delay(100);
}
