/*BIBLIOTECAS REFERENTES A OS SENSORES
AO SERVO E AO SISTEMA DE GRAVAÇÃO*///<-TODAS AS BIBLIOTECAS UTILIZADAS NO SISTEMA FINAL DOO GDAe
#include <mySD.h>
#include <SPI.h>
#include <Wire.h>
//#include <WiFi.h>
#include <Kalman.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <ESP32Servo.h>

/*DEFINIÇÕES DOS PINOS
DO CARTÃO SD*/
#define SD_MOSI 23
#define SD_MISO 19
#define SD_SCK 18
#define SD_CS 5
#define n 30

File root;//colocar nome do arquivo dentro do SD


void setup() {
  
  pinMode (led, OUTPUT);
  
  Serial.begin(115200);
  
  /*Iniciação do processo de leitura e escrita do cartão SD*/
  Serial.print("iniciando o cartão SD...");
  if (!SD.begin(SD_CS, SD_MOSI, SD_MISO, SD_SCK)) {
    Serial.println("Inicialização falhou");
    digitalWrite(led ,HIGH);
    return;
  } else { digitalWrite(led, LOW);
  }
     
  delay(100); // estabilização do(s) sensor(es)

  while (i2cRead(0x3B, i2cData, 14));

  root = SD.open("logging.csv", FILE_WRITE);//abre o cartão SD e permite a escrita no arquivo
 
  if (root) {
    root.println("Variaveis a serem gravadas no cartão SD");
    root.flush();
    root.close();
  } else {

    Serial.println("erro logging.csv");
    digitalWrite(led ,LOW);
  }

  timer = micros();
}

void loop() {
  
  /*COLOCAR O CODIGO DE 
  OBTENÇÃO DE DADOS AQUI*/
  
  grava_SD();//Chama o void do gravador SD  
}

void grava_SD(){
  /*GRAVAÇÃO NO CARTÃO SD*/
  root = SD.open("logging.csv"/*<-COLOCAR NOME DO ARQUIVO*/, FILE_WRITE);//
  root.print(KalAngleX);//COLOCAR O NOME DA VARIAVEL PARA GRAVAÇÃO NO CARTÃO SD
  root.print(","); // SEPARAR AS VARIAVEIS EM VIRGULAS POSSIBILITANDO A DISTINÇÃO DOS DADOS PÓS GRAVAÇÃO
  //REPRETRI LINHA 68 E 69 PARA TODAS AS VARIAVEIS
  entrada++; //<- É INTERRESANTE GRAVAR O NUMERO DA ENTRADAS REALIZADAS TB
  //delay()//NÃO É NECESSARIO MAS DEPENDENDO DA APLICAÇÃO É IMPORTANTE PARA POSSIBILITAR A GRAVAÇÃO CORRETA DOS DADOS
}
