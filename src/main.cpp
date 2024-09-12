#include <Arduino.h>
#include <BluetoothSerial.h>

#define PIN_VOUT 34          // Defina o pino ADC
#define VREF 3.3             // Defina a tens�o de refer�ncia do ESP32
#define R_Fixo 1000.0        // Defina o valor do resistor fixo (em ohms)
#define resolucao 4095.0     // Resolu��o do ADC
#define PERIODO 4000000      // Defina o per�odo em microssegundos (4 segundos)

uint64_t t_atual = 0;
uint64_t t_anterior = 0;
uint64_t t_decorrido = 0;

BluetoothSerial SerialBT;    // Instancia do BluetoothSerial

void setup() {
  // Inicializa a comunica��o serial por USB e Bluetooth
  Serial.begin(115200);

  // Tenta iniciar o Bluetooth e verifica se foi bem-sucedido
  if (!SerialBT.begin("ESP32_Bluetooth")) {
    Serial.println("Erro ao iniciar o Bluetooth");
  } else {
    Serial.println("Bluetooth iniciado com sucesso");
    Serial.println("Emparelhe o dispositivo para se conectar.");
  }

  pinMode(PIN_VOUT, INPUT);  // Define o pino ADC como entrada
}

void loop() {
  // Pausa antes da pr�xima leitura
  t_atual = micros();
  t_decorrido = t_atual - t_anterior;

  if (t_decorrido >= PERIODO) {
    t_anterior = t_atual;

    // Ler a tens�o do pino ADC
    int valor_adc = analogRead(PIN_VOUT);

    // Converter o valor lido
    float Vout = valor_adc * (VREF / resolucao);

    // Calcular a resist�ncia do potenci�metro usando a f�rmula do divisor de tens�o
    float Rpot = (R_Fixo * (VREF - Vout)) / Vout;

    // Imprimir o valor da resist�ncia no monitor serial
    Serial.print("Resistencia do Potenciometro: ");
    Serial.print(Rpot);
    Serial.println(" ohms");
    Serial.print("Tens�o de sa�da (Vout): ");
    Serial.println(Vout);


    // Enviar o valor da resist�ncia via Bluetooth, se houver cliente conectado
    if (SerialBT.hasClient()) {
      SerialBT.print("Resistencia do Potenciometro: ");
      SerialBT.print(Rpot);
      SerialBT.println(" ohms");
    }
  }
}
