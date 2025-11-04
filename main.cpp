//**************/
// Universidad del Valle de Guatemala
// BE3029 - Electronica Digital 2
// Allan Lemus | Juan Vivas
// 3/11/2025
// Laboratorio 6
// MCU: ESP32 dev kit 1.0
//**************/
//**************/
// Librerias
//**************/
#include <Arduino.h>
#include <stdint.h>
#include <LiquidCrystal.h>
#include <ESP32SPISlave.h>
#include "Wire.h"

//**************/
// Definiciones
//**************/
// Pantalla LCD 4 bits
#define rs 32
#define en 25
#define d4 26
#define d5 27
#define d6 14
#define d7 13
// // Hardware utilizado
#define pot1 36

#define ledA 17
#define ledR 16
#define ledV 4

//Configuración de esp32 como esclavo
ESP32SPISlave slave;

static constexpr uint32_t BUFFER_SIZE = 8;
uint8_t tx_buf[BUFFER_SIZE];
uint8_t rx_buf[BUFFER_SIZE];
char cmd;
String prendida;
#define QUEUE_SIZE 1

//Configuración I2C
#define I2C_DEV_ADDR 0x55
uint32_t i = 0;

//**************/
// Prototipos de funciones
//**************/
//Funciones de I2C
void onRequest();
void onReceive(int len);

void leerADC();
void initleds(void);
//**************/
// Variables globales
//**************/
LiquidCrystal lcd(rs,en,d4,d5,d6,d7);
uint8_t valBits = 0;
uint8_t valVolts = 0;
//**************/
// ISRs Rutinas de Interrupcion
//**************/
//**************/
// Configuracion
//**************/
void setup() {
  //Se establece cada una de las configuraciones necesarias tanto de SPI como de I2C, siendo las que llevan el prefijo "Wire" de I2C
  //Y las que llevan el prefijo "Slave" por parte de la comunicación SPI
  Serial.begin(115200);
  delay(2000);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);
  lcd.begin(16,2);
  lcd.setCursor(0,0);
  lcd.print("Todo bien!");
  delay(500);
  lcd.clear();
  initleds();
  slave.setDataMode(SPI_MODE0);
  slave.setQueueSize(QUEUE_SIZE); // por default
  // VSPI = CS: 5, CLK: 18, MOSI: 23, MISO:19
  slave.begin(VSPI);
  memset(tx_buf, 0, BUFFER_SIZE);
  memset(rx_buf, 0, BUFFER_SIZE);
  pinMode(pot1,INPUT);
}
//**************/
// Loop Principal
//**************/
void loop() {
  //Comienzo de la comunicación SPI
  if (slave.hasTransactionsCompletedAndAllResultsHandled()){
    slave.queue(NULL, rx_buf, BUFFER_SIZE);
    slave.trigger();
  }
  if (slave.hasTransactionsCompletedAndAllResultsReady(QUEUE_SIZE)){
    Serial.println("All queued transactions completed. Start verifying ");
    // Si se detecta información del STM32, guarda el dato el rx_buf
    size_t received_bytes = slave.numBytesReceived();
    cmd = rx_buf[0];
    //Dependiendo de que variable se guarda en el buffer, se manda a un estado en el cual se determina cual led se utilizará
    if(cmd == '1'){
      //caso 1, utilizando la led azul de la led RGV
      prendida = "A";
      lcd.setCursor(0,0);
      lcd.print("Bits Volts LED");
      leerADC();
      lcd.setCursor(0,1);
      lcd.print(valBits);
      leerADC();
      lcd.setCursor(5,1);
      lcd.print(valVolts);
      lcd.setCursor(11,1);
      lcd.print(prendida);
      delay(1000);
      lcd.clear();
      digitalWrite(ledA,LOW);
      digitalWrite(ledR,HIGH);
      digitalWrite(ledV,HIGH);
      prendida = "A";
      delay(1000);
      prendida = "---";
      digitalWrite(ledA,HIGH);
      digitalWrite(ledR,HIGH);
      digitalWrite(ledV,HIGH);
      cmd = ' ';
  }
  if (cmd == '3'){
    //caso 3, utilizando la led verde de la led RGV
    prendida = "V";
    lcd.setCursor(0,0);
    lcd.print("Bits Volts LED");
    leerADC();
    lcd.setCursor(0,1);
    lcd.print(valBits);
    leerADC();
    lcd.setCursor(5,1);
    lcd.print(valVolts);
    lcd.setCursor(11,1);
    lcd.print(prendida);
    lcd.clear();
    Serial.println("Setting LED active LOW");
    digitalWrite(ledA,HIGH);
    digitalWrite(ledR,LOW);
    digitalWrite(ledV,HIGH);
    
    delay(500);
    prendida = "---";
    digitalWrite(ledA,HIGH);
    digitalWrite(ledR,HIGH);
    digitalWrite(ledV,HIGH);
    cmd = ' ';
  }
  if (cmd == '2'){
    //caso 2, utilizando la led roja de la led RGV
    prendida = "R";
    lcd.setCursor(0,0);
    lcd.print("Bits Volts LED");
    leerADC();
    lcd.setCursor(0,1);
    lcd.print(valBits);
    leerADC();
    lcd.setCursor(5,1);
    lcd.print(valVolts);
    lcd.setCursor(11,1);
    lcd.print(prendida);
    lcd.clear();
    digitalWrite(ledA,HIGH);
    digitalWrite(ledR,HIGH);
    digitalWrite(ledV,LOW);
    delay(1500);
    prendida = "---";
    digitalWrite(ledA,HIGH);
    digitalWrite(ledR,HIGH);
    digitalWrite(ledV,HIGH);
    cmd = ' ';
  }
}
  //Parte del loop en el cual se muestra las variables en la pantalla LCD de manera constante.
  lcd.setCursor(0,0);
  lcd.print("Bits Volts LED");
  leerADC();
  lcd.setCursor(0,1);
  lcd.print(valBits);
  leerADC();
  lcd.setCursor(5,1);
  lcd.print(valVolts);
  lcd.setCursor(11,1);
  lcd.print(prendida);
  delay(1000);
  lcd.clear();
}
//**************/
// Otras funciones
//**************/
//función encargada de mandar los bits obtenidos por parte del potenciometro hacia la nucleo STM32
void onRequest() {
  leerADC();
  Wire.print(valBits);
  Wire.print(" Packets.");
  Serial.println("onRequest");
  Serial.println(valBits);
}
// Función encargada de recibir la solicitud de envio de datos 
void onReceive(int len) {
  Serial.printf("onReceive[%d]: ", len);
  while (Wire.available()) {
    Serial.write(Wire.read());
  }
  Serial.println();
}
// función que se encarga de leer la cantidad de bits por parte del potenciometro al realizar un analogRead
void leerADC() {
  const int N = 10;
  long suma = 0;
//Lectura analógica del potenciometro
  for (int i = 0; i < N; i++) {
    suma += analogRead(pot1);
    delay(1);
  }
  //Conversión a bits del resultado de la lectura analógica
  int poten = suma / N;               // promedio
  valBits = map(poten, 0, 4095, 0, 255); // mapea a 8 bits
  valVolts = (poten * 3.3) / 4095;
}
//Función inicializadora de las leds, permite comprobar el funcionamiento correcto de ellas.
void initleds(void){
  pinMode(ledR, OUTPUT);
  pinMode(ledV, OUTPUT);
  pinMode(ledA, OUTPUT);

  digitalWrite(ledR, HIGH);
  digitalWrite(ledV, HIGH);
  digitalWrite(ledA, HIGH);
}