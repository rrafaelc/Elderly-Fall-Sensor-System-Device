#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SimpleKalmanFilter.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <TimeLib.h>
#include <wifi.h>

// Definindo constantes
#define BUTTON_PIN 19
#define BUZZER_PIN 5
#define LED_PIN 4
#define MPU 0x68

// Variáveis globais
Adafruit_MPU6050 mpu;
float accX, accY, accZ, gyroX, gyroY, gyroZ;
bool quedaDetectada = false;

String lastPublishedStatus = "offline";
//variavel global tempo
unsigned long lastFallDetection = 0; // Tempo da última detecção de queda

//Utilize um mecanismo de "debounce" para evitar múltiplas notificações de queda em um curto período.
const unsigned long debounceTime = 10000; // 10 segundos de debounce

// unsigned long previousMillis = 0;  
// const long interval = 1000; // Intervalo de atualização de 1 segundo
int fallCount = 0;

unsigned long lastSensorUpdate = 0; // Armazena o último tempo em que os sensores foram atualizados
unsigned long sensorUpdateInterval = 100; // ou outro valor que você preferir
unsigned long loopStartTime = 0; // inicializa para controle de tempo

//controle de intervalo para evitar repetições
unsigned long lastMQTTPublish = 0;
const unsigned long mqttPublishInterval = 3000; // Intervalo entre as publicações

// Contadores para avaliação
int verdadeiroPositivo = 0;
int falsoPositivo = 0;
int verdadeiroNegativo = 0;
int falsoNegativo = 0;

// Variáveis de calibração
float accOffsetX = 0.0, accOffsetY = 0.0, accOffsetZ = 0.0;
float gyroOffsetX = 0.0, gyroOffsetY = 0.0, gyroOffsetZ = 0.0;

float thresholdAccel = 1.5;  
float thresholdGyro = 2.0;

// Definições do servidor MQTT e tópico
const char* mqtt_server = "maqiatto.com";
const char* mqtt_username = "iago.almeida2@fatec.sp.gov.br";
const char* mqtt_password = "S0gek1ng@732";
const char* topic_queda = "iago.almeida2@fatec.sp.gov.br/queda/alerta";

WiFiClient espClient;
PubSubClient client(espClient);

// Criando instâncias do filtro de Kalman
SimpleKalmanFilter kalmanAccX(0.05, 1.3, 0.01); // (Q, R, P)
SimpleKalmanFilter kalmanAccY(0.05, 1.3, 0.01);
SimpleKalmanFilter kalmanAccZ(0.05, 1.3, 0.01);
SimpleKalmanFilter kalmanGyroX(0.1, 1.5, 0.01);
SimpleKalmanFilter kalmanGyroY(0.1, 1.5, 0.01);
SimpleKalmanFilter kalmanGyroZ(0.1, 1.5, 0.01);


String serialNumber = "123456"; // Substitua pelo número de série real


// Função para piscar o buzzer e o LED
void blinkBuzzerAndLED(int times) {
    for (int i = 0; i < times; i++) {
        tone(BUZZER_PIN, 1800); // Emite um tom no buzzer
        digitalWrite(LED_PIN, HIGH); // Liga o LED
        delay(250); // Mantém o tom e o LED aceso por 250ms
        noTone(BUZZER_PIN); // Para o tom
        digitalWrite(LED_PIN, LOW); // Desliga o LED
        delay(250); // Pausa entre os toques
    }
}

void setup() {
    Serial.begin(115200);
    delay(100);

    if (!initializeMPU()) {
        Serial.println("Falha ao iniciar o MPU6050. Reiniciando...");
        delay(2000); // Aguarda um momento antes de reiniciar
        ESP.restart(); // Reinicia o ESP
    }

    Wire.begin();
    setup_wifi();
    setup_mqtt();
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
}

bool initializeMPU() {
    if (!mpu.begin()) {
        return false; // Retorna falso se a inicialização falhar
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    Serial.println("MPU6050 configurado!");

    calibrateSensors();
    Serial.println("Calibração concluída!");
    return true; // Retorna verdadeiro se a inicialização for bem-sucedida
}

void calibrateSensors() {
    sensors_event_t a, g, temp;
    int samples = 100;

    for (int i = 0; i < samples; i++) {
        mpu.getEvent(&a, &g, &temp);
        accOffsetX += a.acceleration.x;
        accOffsetY += a.acceleration.y;
        accOffsetZ += (a.acceleration.z - 9.81);
        gyroOffsetX += g.gyro.x;
        gyroOffsetY += g.gyro.y;
        gyroOffsetZ += g.gyro.z;
        delay(10);
    }

    accOffsetX /= samples;
    accOffsetY /= samples;
    accOffsetZ /= samples;
    gyroOffsetX /= samples;
    gyroOffsetY /= samples;
    gyroOffsetZ /= samples;
}

void setup_wifi() {
    WiFiManager wifiManager; 
    wifiManager.setTimeout(60); 

    int attempts = 0; 
    while (!wifiManager.autoConnect("ESP32-AP") && attempts < 3) {
        Serial.println("Tentativa de conexão falhou. Tentando novamente...");
        attempts++;
        delay(1000);
    }

    if (attempts == 3) {
        Serial.println("Falha ao conectar após 3 tentativas. Reiniciando...");
        ESP.restart(); 
    }

    Serial.println("Conectado à rede WiFi!");
    Serial.print("Endereço IP: ");
    Serial.println(WiFi.localIP());
}

void checkWiFiConnection() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi desconectado. Tentando reconectar...");
        setup_wifi();
    }
}

// <<Config MQTT>>
void setup_mqtt() {
    client.setServer(mqtt_server, 1883); 
    Serial.println("MQTT configurado.");
}

void handleMQTTConnection() {
    if (!client.connected()) {
        Serial.println("Tentando reconectar ao MQTT...");
        reconnect(); // Tenta reconectar
    }
}

void reconnect() {
    while (!client.connected()) {
        // Serial.print("Tentando conexão MQTT...");
        if (client.connect("ESP32Client", mqtt_username, mqtt_password)) {
            Serial.println("Conectado ao broker MQTT!");
        } else {
            Serial.print("Falha na conexão. Estado: ");
            Serial.println(client.state());
            delay(2000);
        }
    }
}

unsigned long lastPrintTime = 0; // Tempo da última impressão
const unsigned long printInterval = 2000; // Intervalo de 2 segundos

void loop() {
    unsigned long startLoop = millis();
    checkWiFiConnection();
    handleMQTTConnection();
    client.loop();
    handleButtonPress();

    unsigned long currentMillis = millis();
    if (currentMillis - lastSensorUpdate >= sensorUpdateInterval) {
        processSensorData();
        lastSensorUpdate = currentMillis;
        calculatePerformanceMetrics();
    }

    // Monitorar o tempo de execução do loop
    unsigned long loopDuration = millis() - loopStartTime;
    Serial.print("Duração do loop: ");
    Serial.print(loopDuration);
    Serial.println(" ms");
}

void handleButtonPress() {
    // Publica o estado do botão apenas se a conexão estiver ativa
    if (digitalRead(BUTTON_PIN) == LOW) {
        if (client.connected()) {
            Serial.println("Publicando mensagem de emergência...");
            sendEmergencyAlert();
            activateBuzzer();
            digitalWrite(LED_PIN, HIGH);
        } else {
            Serial.println("Conexão MQTT não está ativa. Não é possível publicar emergência.");
        }
    } else {
        deactivateBuzzer();
        digitalWrite(LED_PIN, LOW);
    }
}


void processSensorData() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    accX = a.acceleration.x - accOffsetX;
    accY = a.acceleration.y - accOffsetY;
    accZ = a.acceleration.z - accOffsetZ;
    gyroX = g.gyro.x - gyroOffsetX;
    gyroY = g.gyro.y - gyroOffsetY;
    gyroZ = g.gyro.z - gyroOffsetZ;

    accX = kalmanAccX.updateEstimate(accX);
    accY = kalmanAccY.updateEstimate(accY);
    accZ = kalmanAccZ.updateEstimate(accZ);
    gyroX = kalmanGyroX.updateEstimate(gyroX);
    gyroY = kalmanGyroY.updateEstimate(gyroY);
    gyroZ = kalmanGyroZ.updateEstimate(gyroZ);

    // Cálculo da aceleração e giroscópio total
    float accelTotal = sqrt(pow(accX, 2) + pow(accY, 2) + pow(accZ, 2)) / 9.81;
    float gyroTotal = sqrt(pow(gyroX, 2) + pow(gyroY, 2) + pow(gyroZ, 2));

    // Imprimindo dados
    Serial.printf("Aceleração: X = %.2f, Y = %.2f, Z = %.2f, Total = %.2f g\n", accX, accY, accZ, accelTotal);
    Serial.printf("Giroscópio: X = %.2f, Y = %.2f, Z = %.2f, Total = %.2f °/s\n", gyroX, gyroY, gyroZ, gyroTotal);

    // Avaliação da detecção de queda
    evaluateFallDetection(accelTotal, gyroTotal);

    Serial.print("Aceleração Total "); Serial.print(accelTotal); Serial.println(" g");
    Serial.print("Gyro Total: "); Serial.print(gyroTotal); Serial.println(" °/s");
    Serial.printf("Contagem de quedas: %d\n", fallCount);
    Serial.printf("VP: %d, FP: %d, VN: %d, FN: %d\n", verdadeiroPositivo, falsoPositivo, verdadeiroNegativo, falsoNegativo);
}

void evaluateFallDetection(float accelTotal, float gyroTotal) {
    if (accelTotal > thresholdAccel && gyroTotal > thresholdGyro) {
        if (!quedaDetectada) {
            Serial.println("Queda detectada!");
            lastFallDetection = millis();
            fallCount++;
            quedaDetectada = true;
            sendFallDataToAPI();
            activateBuzzer();
        }
        verdadeiroPositivo++;
    } else if (accelTotal <= thresholdAccel && gyroTotal <= thresholdGyro) {
        verdadeiroNegativo++;
        quedaDetectada = false; // Reseta queda detectada
        deactivateBuzzer();
    } else {
        // Apenas um dos dois critérios está acima do limite
        if (accelTotal > thresholdAccel || gyroTotal > thresholdGyro) {
            falsoPositivo++;
            Serial.println("Falso positivo detectado.");
        } else {
            falsoNegativo++;
            Serial.println("Falso negativo detectado.");
        }
    }

    // Debounce para queda
    if (quedaDetectada && (millis() - lastFallDetection >= debounceTime)){
        quedaDetectada = false; // Reseta a detecção após o debounce
    }
}

void sendEventToAPI(const String& eventType, bool isFall, bool isImpact, 
                    float ax = 0.0, float ay = 0.0, float az = 0.0,
                    float gx = 0.0, float gy = 0.0, float gz = 0.0) {
    // Verifica se o MQTT está conectado antes de tentar publicar
    if (!client.connected()) {
        Serial.println("MQTT não está conectado. Tentando reconectar...");
        handleMQTTConnection();
        if (!client.connected()) {
            Serial.println("Falha ao conectar ao MQTT, não será possível enviar dados.");
            return;
        }
    }

    unsigned long currentMillis = millis();
    if (currentMillis - lastMQTTPublish >= mqttPublishInterval) {
        StaticJsonDocument<256> doc;
        doc["serial_number"] = serialNumber;
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

        const int maxRetries = 3;
        int attempts = 0;
        bool published = false;

        while (attempts < maxRetries && !published) {
            if (client.publish(topic_queda, output.c_str())) {
                Serial.println("Dados enviados com sucesso!");
                lastMQTTPublish = currentMillis; // Atualiza o tempo de publicação
                published = true;
            } else {
                Serial.print("Falha ao enviar dados. Tentativa: ");
                Serial.println(attempts + 1);
                attempts++;
                delay(1000); // Aguarda antes de tentar novamente
            }
        }

        if (!published) {
            Serial.println("Falha ao enviar dados após várias tentativas.");
        }
    } else {
        Serial.println("Aguardando para evitar duplicidade de publicações.");
    }
}



 // Função para enviar alerta de emergência
void sendEmergencyAlert() {
    sendEventToAPI("emergencia", false, false);
}

// Função para enviar dados de queda
void sendFallDataToAPI() {
    sendEventToAPI("queda", quedaDetectada, false,
                  accX, accY, accZ, gyroX, gyroY, gyroZ);
}


void activateBuzzer() {
    tone(BUZZER_PIN, 1800);
    Serial.println("Buzzer ativado!");
    delay(500);
}

void deactivateBuzzer() {
    noTone(BUZZER_PIN);
    Serial.println("Buzzer desativado!");
}

void calculatePerformanceMetrics() {
    float precision = (verdadeiroPositivo + falsoPositivo) > 0 
                      ? (float)verdadeiroPositivo / (verdadeiroPositivo + falsoPositivo) : 0;
    float recall = (verdadeiroPositivo + falsoNegativo) > 0 
                  ? (float)verdadeiroPositivo / (verdadeiroPositivo + falsoNegativo) : 0;
    float specificity = (verdadeiroNegativo + falsoPositivo) > 0 
                        ? (float)verdadeiroNegativo / (verdadeiroNegativo + falsoPositivo) : 0;

    float f1Score = (precision + recall > 0) 
                     ? 2 * (precision * recall) / (precision + recall) : 0;

    Serial.println("==== Métricas de Desempenho ====");
    Serial.print("Precisão: "); Serial.println(precision, 4);
    Serial.print("Recall: "); Serial.println(recall, 4);
    Serial.print("Especificidade: "); Serial.println(specificity, 4);
    Serial.print("F1-Score: "); Serial.println(f1Score, 4);
}


// // put function declarations here:
// int myFunction(int, int);

// void setup() {
//   // put your setup code here, to run once:
//   int result = myFunction(2, 3);
// }

// void loop() {
//   // put your main code here, to run repeatedly:
// }

// // put function definitions here:
// int myFunction(int x, int y) {
//   return x + y;
// }