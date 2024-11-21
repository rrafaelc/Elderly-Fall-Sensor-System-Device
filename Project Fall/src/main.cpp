#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <SimpleKalmanFilter.h>
#include <math.h>

const char* mqtt_server = "maqiatto.com";  // Endereço do broker MQTT
const int mqtt_port = 1883; // Porta padrão do MQTT
const char* mqtt_user = "iago.almeida2@fatec.sp.gov.br";  // Usuario do MQTT
const char* mqtt_password = "S0gek1ng@732";  // Senha do MQTT
const char* topic_queda = "iago.almeida2@fatec.sp.gov.br/queda/alerta"; // Tópico onde os dados serão publicados
const char* serialNumber = "ESP32_001";

WiFiClient espClient;
PubSubClient client(espClient);

// Pinos para o buzzer e o LED
#define BUZZER_PIN 5
#define LED_PIN 4

// Pino do botão de emergência
#define BUTTON_PIN 19

// Variáveis para o controle de queda
bool quedaDetectada = false;
int fallCount = 0;

// Contadores para avaliação
int verdadeiroPositivo = 0;
int falsoPositivo = 0;
int verdadeiroNegativo = 0;
int falsoNegativo = 0;

// Variáveis de tempo
unsigned long lastSensorUpdate = 0; // Armazena o último tempo em que os sensores foram atualizados
unsigned long sensorUpdateInterval = 50; // Intervalo para a atualização do sensor (em milissegundos)

// Intervalo entre tentativas de reconexão
const long wifiReconnectInterval = 20000;  // Intervalo de 20 segundos para reconectar ao Wi-Fi
const long mqttReconnectInterval = 10000;  // Intervalo de 10 segundos para reconectar ao MQTT

unsigned long lastWiFiReconnectTime = 0;  // Armazena o tempo da última tentativa de reconexão Wi-Fi
unsigned long lastMQTTReconnectTime = 0;  // Armazena o tempo da última tentativa de reconexão MQTT

// Definições do intervalo de amostragem
const int sampleInterval = 100; // Intervalo de amostragem em milissegundos

// Variáveis para armazenar os valores anteriores (usadas para calcular a variação brusca)
float prevAccX = 0.0, prevAccY = 0.0, prevAccZ = 0.0;
float prevGyroX = 0.0, prevGyroY = 0.0, prevGyroZ = 0.0;

// Variáveis para armazenar os offsets dos sensores
float accOffsetX = 0.0, accOffsetY = 0.0, accOffsetZ = 0.0;
float gyroOffsetX = 0.0, gyroOffsetY = 0.0, gyroOffsetZ = 0.0;

// Limites para detecção de queda
const float fallAccelThreshold = 4.0;   // Ajuste para aceleração
const float fallGyroThreshold = 6.0;    // Ajuste para giroscópio
const float fallJerkThreshold = 100000.0; // Ajuste para jerk (mudar para um valor menor se muito sensível)

Adafruit_MPU6050 mpu;
Preferences preferences;

SimpleKalmanFilter kalmanAccX(0.05, 1.3, 0.01); // (Q, R, P)
SimpleKalmanFilter kalmanAccY(0.05, 1.3, 0.01);
SimpleKalmanFilter kalmanAccZ(0.05, 1.3, 0.01);

SimpleKalmanFilter kalmanGyroX(0.1, 0.2, 0.1);
SimpleKalmanFilter kalmanGyroY(0.1, 0.2, 0.1);
SimpleKalmanFilter kalmanGyroZ(0.1, 0.1, 0.1);

// Função para piscar o buzzer e LED
void blinkBuzzerAndLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);    // Acende o LED
    tone(BUZZER_PIN, 1800);         // Emite som de 1800 Hz no buzzer
    delay(200);                     // Mantém os dois acesos por 200 ms
    digitalWrite(LED_PIN, LOW);     // Apaga o LED
    noTone(BUZZER_PIN);             // Desativa o buzzer
    delay(200);                     // Mantém os dois apagados por 200 ms
  }
}

// Função para reconectar ao MQTT
void reconnect() {
  while (!client.connected()) {
    Serial.print("Tentando conectar ao MQTT...");
    if (client.connect("ESP32_FallDetector", mqtt_user, mqtt_password)) {
      Serial.println("Conectado ao MQTT!");
    } else {
      Serial.print("Falha na conexão, rc=");
      Serial.print(client.state());
      Serial.println(" Tentando novamente em 5 segundos.");
      delay(5000);
    }
  }
}

// Função para configurar e conectar ao Wi-Fi
void setup_wifi() {
  Serial.begin(115200);

  // Conectar à rede Wi-Fi usando WiFiManager
  WiFiManager wifiManager;
  wifiManager.autoConnect("ESP32_FallDetector");

  Serial.println("Conectado ao Wi-Fi!");
  blinkBuzzerAndLED(3);  // Pisca o buzzer e LED 3 vezes quando conectado ao Wi-Fi
}

// Função para verificar a conexão Wi-Fi
void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastWiFiReconnectTime >= wifiReconnectInterval) {
      Serial.println("Wi-Fi desconectado! Tentando reconectar...");
      setup_wifi();  // Tenta reconectar ao Wi-Fi
      lastWiFiReconnectTime = millis();
    }
  } else {
    if (millis() - lastWiFiReconnectTime >= wifiReconnectInterval) {
      Serial.println("Wi-Fi conectado!");
      lastWiFiReconnectTime = millis();  // Atualiza o tempo da última mensagem
    }
  }
}

// Função para avaliar a detecção de queda
void evaluateFallDetection() {
  // Estruturas para armazenar os dados dos sensores
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp); // Obtém os dados do acelerômetro e giroscópio

  // Aceleração (já em g, sem necessidade de dividir por 8192)
  float accX = a.acceleration.x - accOffsetX;
  float accY = a.acceleration.y - accOffsetY;
  float accZ = a.acceleration.z - accOffsetZ;

  // Cálculo de rotação do giroscópio (já em °/s, sem necessidade de dividir por 65.5)
  float gyroX = g.gyro.x - gyroOffsetX;
  float gyroY = g.gyro.y - gyroOffsetY;
  float gyroZ = g.gyro.z - gyroOffsetZ;


  // Aplica o filtro de Kalman no acelerômetro
  accX = kalmanAccX.updateEstimate(accX);
  accY = kalmanAccY.updateEstimate(accY);
  accZ = kalmanAccZ.updateEstimate(accZ);

  // Aplica o filtro de Kalman no giroscópio
  gyroX = kalmanGyroX.updateEstimate(gyroX);
  gyroY = kalmanGyroY.updateEstimate(gyroY);
  gyroZ = kalmanGyroZ.updateEstimate(gyroZ);

  // Cálculo da variação brusca (jerk) para detecção de queda
  float jerkX = (accX - prevAccX) / (sampleInterval / 1000.0);  // Variação de aceleração no eixo X
  float jerkY = (accY - prevAccY) / (sampleInterval / 1000.0);  // Variação de aceleração no eixo Y
  float jerkZ = (accZ - prevAccZ) / (sampleInterval / 1000.0);  // Variação de aceleração no eixo Z
  float jerkTotal = jerkX * jerkX + jerkY * jerkY + jerkZ * jerkZ;  // Magnitude total do jerk

  // Cálculo da aceleração total (magnitude)
  float accelTotal = sqrt(accX * accX + accY * accY + accZ * accZ) / 9.81;
  
  // Cálculo da rotação total do giroscópio (magnitude)
  float gyroTotal = sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);

  // Imprime os dados do acelerômetro e giroscópio no monitor serial
  Serial.print("Aceleração: ");

  Serial.print("AccelX:");
  Serial.print(accX);
  Serial.print(",");
  Serial.print("AccelY:");
  Serial.print(accY);
  Serial.print(",");
  Serial.print("AccelZ:");
  Serial.print(accZ);


  // Serial.print(" | Giroscópio: ");

  // Serial.print("GyroX:");
  // Serial.print(gyroX);
  // Serial.print(",");
  // Serial.print("GyroY:");
  // Serial.print(gyroY);
  // Serial.print(",");
  // Serial.print("GyroZ:");
  // Serial.print(gyroZ);
  // Serial.println("");

  Serial.print(" | Aceleração Total (g): ");
  Serial.print(accelTotal);
  Serial.print(" | Giroscópio Total (°/s): ");
  Serial.println(gyroTotal);
  Serial.print(" | Variação Brusca (jerk): ");
  Serial.println(jerkTotal);

  // Verificação de queda
  if (accelTotal > fallAccelThreshold && gyroTotal > fallGyroThreshold && jerkTotal > fallJerkThreshold) {
    if (!quedaDetectada) {
      Serial.println("Queda detectada!");
      quedaDetectada = true;
      blinkBuzzerAndLED(2);
      fallCount++;
      sendEventToAPI("queda", true, false, accX, accY, accZ, gyroX, gyroY, gyroZ);
      Serial.println("Status enviado!");
    }
  } else {
    quedaDetectada = false;  // Reseta a detecção
  }

  // Atualiza os valores anteriores de aceleração e giroscópio
  prevAccX = accX;
  prevAccY = accY;
  prevAccZ = accZ;
  prevGyroX = gyroX;
  prevGyroY = gyroY;
  prevGyroZ = gyroZ;
}


// Função para enviar dados para o API MQTT (usada para queda e emergência)
void sendEventToAPI(const String& eventType, bool isFall, bool isImpact,
                    float ax, float ay, float az, 
                    float gx, float gy, float gz) {

  if (!client.connected()) {
    Serial.println("MQTT não está conectado. Tentando reconectar...");
    reconnect();
    if (!client.connected()) {
      Serial.println("Falha ao conectar ao MQTT, não será possível enviar dados.");
      return;
    }
  }

  unsigned long currentMillis = millis();
  if (currentMillis - lastMQTTReconnectTime >= 5000) {
    StaticJsonDocument<256> doc;
    doc["serial_number"] = "ESP32_001";
    doc["event_type"] = eventType;
    doc["is_fall"] = isFall;
    doc["is_impact"] = isImpact;

    JsonObject acceleration = doc.createNestedObject("acceleration");
    acceleration["ax"] = ax;
    acceleration["ay"] = ay;
    acceleration["az"] = az;

    JsonObject gyroscope = doc.createNestedObject("gyroscope");
    gyroscope["gx"] = gx;
    gyroscope["gy"] = gy;
    gyroscope["gz"] = gz;

    String output;
    serializeJson(doc, output);

    int attempts = 0;
    bool published = false;
    while (attempts < 3 && !published) {
      if (client.publish(topic_queda, output.c_str())) {
        Serial.println("Dados enviados com sucesso!");
        lastMQTTReconnectTime = currentMillis;
        published = true;
      } else {
        Serial.print("Falha ao enviar dados. Tentativa: ");
        Serial.println(attempts + 1);
        attempts++;
        delay(1000);
      }
    }

    if (!published) {
      Serial.println("Falha ao enviar dados após várias tentativas.");
    }
  }
}

// Função para verificar o botão de emergência
void checkEmergencyButton() {
  unsigned long currentMillis = millis();
  static unsigned long lastButtonPressTime = 0;

  if (digitalRead(BUTTON_PIN) == LOW && (currentMillis - lastButtonPressTime) > 500) {
    lastButtonPressTime = currentMillis;
    blinkBuzzerAndLED(1);
    sendEventToAPI("emergencia", false, false, 0, 0, 0, 0, 0, 0);
  }
}

  // void calibrateSensors() {
  //   sensors_event_t a, g, temp;
  //   int samples = 100;

  //   for (int i = 0; i < samples; i++) {
  //       mpu.getEvent(&a, &g, &temp);
  //       accOffsetX += a.acceleration.x;
  //       accOffsetY += a.acceleration.y;
  //       accOffsetZ += (a.acceleration.z - 9.81);
  //       gyroOffsetX += g.gyro.x;
  //       gyroOffsetY += g.gyro.y;
  //       gyroOffsetZ += g.gyro.z;
  //       delay(10);
  //   }

  //   accOffsetX /= samples;
  //   accOffsetY /= samples;
  //   accOffsetZ /= samples;
  //   gyroOffsetX /= samples;
  //   gyroOffsetY /= samples;
  //   gyroOffsetZ /= samples;

  //   //Salvar os offsets na memória Flash (Preferences)
  //   preferences.begin("calibration", false);  // "calibration" é o namespace
  //   preferences.putFloat("accOffsetX", accOffsetX);
  //   preferences.putFloat("accOffsetY", accOffsetY);
  //   preferences.putFloat("accOffsetZ", accOffsetZ);
  //   preferences.putFloat("gyroOffsetX", gyroOffsetX);
  //   preferences.putFloat("gyroOffsetY", gyroOffsetY);
  //   preferences.putFloat("gyroOffsetZ", gyroOffsetZ);
  //   preferences.end();  // Finaliza o uso do namespace

  //   Serial.println("Calibração salva na memória Flash!");

  // }

  void loadCalibrationData() {
    preferences.begin("calibration", true);  // Abre o armazenamento de preferências para leitura
    // Carrega os valores de offsets ou usa 0.0 como valor padrão se não existirem
    accOffsetX = preferences.getFloat("accOffsetX", 0.0);
    accOffsetY = preferences.getFloat("accOffsetY", 0.0);
    accOffsetZ = preferences.getFloat("accOffsetZ", 0.0);
    gyroOffsetX = preferences.getFloat("gyroOffsetX", 0.0);
    gyroOffsetY = preferences.getFloat("gyroOffsetY", 0.0);
    gyroOffsetZ = preferences.getFloat("gyroOffsetZ", 0.0);
    preferences.end();  // Fecha o armazenamento de preferências
    Serial.println("Dados de calibração carregados.");
  }

// Função de inicialização do MPU6050
bool initializeMPU() {
    if (!mpu.begin()) {
        // Se falhar na inicialização, retorna false
        return false;
    }
    // Configuração do acelerômetro e giroscópio
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);  // Faixa de aceleração ±4G
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);       // Faixa de giroscópio ±500°/s
    
    // Configuração do filtro passa-alta
    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(1); // Detecta movimento com limiar baixo (sensível)
    mpu.setMotionDetectionDuration(20); // O movimento deve durar 20ms
    mpu.setInterruptPinLatch(true);     // Interrupção latente (permanece até ser reiniciada)
    mpu.setInterruptPinPolarity(true);  // Polaridade da interrupção
    mpu.setMotionInterrupt(true);       // Habilita interrupção por movimento
    Serial.println("MPU6050 configurado!");

    // calibrateSensors();
    loadCalibrationData(); 
    Serial.println("Calibração concluída!");

    return true;  // Retorna true se a inicialização for bem-sucedida
}

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Wire.begin();
  setup_wifi();

  // Tenta inicializar o MPU6050
  if (!initializeMPU()) {
      Serial.println("Falha ao iniciar o MPU6050. Reiniciando...");
      delay(2000);  // Aguarda 2 segundos antes de reiniciar
      ESP.restart(); // Reinicia o ESP32
  }

  
  Serial.println("Configuração concluída!");

  client.setServer(mqtt_server, mqtt_port);
  // client.setCallback(callback);
}

void loop() {
  checkWiFiConnection();

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();
    if (currentMillis - lastSensorUpdate >= sensorUpdateInterval) {
        // Avaliar a detecção de queda
        evaluateFallDetection();
        lastSensorUpdate = currentMillis;
        // calculatePerformanceMetrics();
    }

  // Verificar botão de emergência
  checkEmergencyButton();
  yield();
}
