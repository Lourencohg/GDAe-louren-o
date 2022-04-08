/*BIBLIOTECAS REFERENTES A OS SENSORES
AO SERVO E AO SISTEMA DE GRAVAÇÃO*/
#include <mySD.h>
#include <SPI.h>
#include <Wire.h>
//#include <WiFi.h>
#include <Kalman.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <ESP32Servo.h>

/*DEFINIÇÕES PARA CONVERTER OS VALORES RECEBIDOS
DO ACELEROMETRO E DO GIROSCOPIO PARA FORÇA G E RAD/S*/
#define Conversor_AC 0.00059814453125
#define Conversor_Giroscopio 131.0

/*DEFINIÇÕES DOS PINOS
DO CARTÃO SD*/
#define SD_MOSI 23
#define SD_MISO 19
#define SD_SCK 18
#define SD_CS 5
#define n 30

/*DEFINIÇÕES DOS PINOS
DO BMP-280*/
#define BMP_MOSI 11
#define BMP_MISO 12
#define BMP_SKC 13
#define BMP_CS 10

Adafruit_BMP280 bmp;

/*CRIAÇÃO DE 
FILTROS KALMAN*/
Kalman KalmanX;
Kalman KalmanY;
Kalman KalmanZ;


/*VARIAVEIS*/
double comparador_angulo = 0;

//
uint8_t i2cData[14];
double AcX, AcY, AcZ;//Variaveis para armazenar os valores das acelerações
double gyroX, gyroY, gyroZ;//Variaveis para armazenar os valores das rotações

int entrada = 1;//Numero do registro no cartão SD
int led = 2;//Pino responsavel por acender o pino interno do ESP
uint32_t timer;//Reloginho

double gyroXangle;
double gyroYangle;
double gyroZangle;//Variaveis relacionados a angulação do foguete provida pelo giroscopio

double KalAngleX;
double KalAngleY;
double KalAngleZ;//Variaveis relacionados a angulação do foguete com os filtros já aplicados

double Ac_x_ms;
double Ac_y_ms;
double Ac_z_ms;//Aceleração ja "processsada"

double aceleracao_total;
double aceleracao_corpo;

File root;//colocar nome do arquivo dentro do SD


void setup() {
  
  pinMode (led, OUTPUT);
  
  Serial.begin(115200);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Freq = 400kHz.
#else
  TWBR = ((F_CPU/400000UL) - 16) / 2; // Freq = 400kHz
#endif
  
  i2cData[0] = 7;
  i2cData[1] = 0x00;
  i2cData[2] = 0x00;// escala de ± 250º para o giroscopio
  i2cData[3] = 0x00; // escala de ±2g para o acelerometro
  
  while (i2cWrite(0x19, i2cData, 4, false)); // 
  while (i2cWrite(0x6B, 0x01, true)); // Disabilita o sleep mode
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // lé o registro "WHO_AM_I"
    Serial.print("Erro sensor");
    while (1);{
      Serial.print("Erro!. Conexão com MPU 6050 não encontrada");
      }
  }

  /*espaço para o codigo referente ao barometro?*/
  Serial.println(F("BMP280 TEST"));

  //if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID))
  if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID){
    Serial.println(F("não foi possivel encontrar um sensor bmp280 valido"))
    while (1) delay(10);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL /*modo de abertura*/
                  Adafruit_BMP280::SAMPLING_X2, /*Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16, /*Presure oversampling*/
                  Adafruit_BMP280::FILTER_X16, /*filtering*/
                  Adafruit_BMP280::STANDBY_MS_500); /*Standby time*/
  
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
    root.println("ANGX, ANGY, ANGZ, ACELX, ACELY, ACELZ, ACEL_CORPO, ACEL_TOTAL, TEMPO, ENTRADA");
    root.flush();
    root.close();
  } else {

    Serial.println("erro logging.csv");
    digitalWrite(led ,LOW);
  }
  
  /*SETANDO OS ANGULOS INICIAIS DO SISTEMA*/  
  AcX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  AcY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  AcZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  double pitch = atan(AcX/sqrt(AcY * AcY + AcZ * AcZ)) * RAD_TO_DEG;
  double roll = atan(AcY/sqrt(AcX * AcX + AcZ * AcZ)) * RAD_TO_DEG;
  double yaw = atan(AcX/sqrt(AcZ * AcZ + AcY * AcY)) * RAD_TO_DEG;//angulos apartir do acelerometros

  KalmanX.setAngle(roll);
  KalmanY.setAngle(pitch);
  KalmanZ.setAngle(yaw);

  gyroXangle = roll;
  gyroYangle = pitch;
  gyroZangle = yaw;
  //Angulos iniciais do sistema

  timer = micros();
}

void loop() {

  /*FUNCIONAMENTO DO MPU6050*/
  while (i2cRead(0x3B, i2cData, 14));
  AcX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  AcY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  AcZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  //tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calcula deltaT
  timer = micros();


  double pitch = atan(AcX/sqrt(AcY * AcY + AcZ * AcZ)) * RAD_TO_DEG;
  double roll = atan(AcY/sqrt(AcX * AcX + AcZ * AcZ)) * RAD_TO_DEG;
  double yaw = atan(AcX/sqrt(AcZ * AcZ + AcY * AcY)) * RAD_TO_DEG;
  
  Ac_x_ms = AcX*Conversor_AC;
  Ac_y_ms = AcY*Conversor_AC;
  Ac_z_ms = AcZ*Conversor_AC;

  aceleracao_total = sqrt(Ac_x_ms*Ac_x_ms + Ac_y_ms*Ac_y_ms + Ac_z_ms*Ac_z_ms);
  aceleracao_corpo = aceleracao_total - 9.8; // força g?
  /*Armados com o que há de mais moderno em equipamentos de espionagem,
  um porquinho-da-índia e seu time altamente treinado  são o último recurso de defesa para proteger o mundo do caos e da destruição*/

  gyroXangle = gyroX / Conversor_Giroscopio; // converte de rad/s para deg/s
  gyroYangle = gyroY / Conversor_Giroscopio;
  gyroZangle = gyroZ / Conversor_Giroscopio;

  KalAngleX = KalmanX.getAngle(roll, gyroXangle, dt);
  KalAngleY = KalmanY.getAngle(pitch, gyroYangle, dt);
  KalAngleZ = KalmanZ.getAngle(yaw, gyroZangle, dt);

  /*SAIDAS DO VALORES OBTIDOS*/
  Serial.print("AngX: ");
  Serial.println(KalAngleX);
  //Serial.print(gyroXangle); angulo calculado por Navegação estimada
  Serial.print("AngY: ");
  Serial.println(KalAngleY);
  //Serial.print(gyroXangle); angulo calculado por Navegação estimada
  Serial.print("AngZ: ");
  Serial.println(KalAngleZ);
  //Serial.print(gyroXangle); angulo calculado por Navegação estimada

  /*CODIGO BMP 280*/  

  
  /*espaço do codigo de ativação dos paraquedas*/

  
  grava_SD();//Chama o void do gravador SD  
  
}

void grava_SD(){
  /*GRAVAÇÃO NO CARTÃO SD*/
  root = SD.open("logging.csv", FILE_WRITE);
  root.print(KalAngleX);
  root.print(",");
  root.print(KalAngleY);
  root.print(",");
  root.print(KalAngleZ);
  root.print(",");
  root.print( Ac_x_ms);
  root.print(",");
  root.print( Ac_y_ms);
  root.print(",");
  root.print( Ac_z_ms);
  root.print(",");
  root.print(aceleracao_corpo);
  root.print(",");
  root.print(aceleracao_total);
  root.print(",");
  root.print(temperatura);
  root.print(",");
  root.print(timer);
  root.print(",");
  root.print(entrada);
  root.print(",");
  root.println(""); 
  root.close();  
  entrada++;
  //delay()
}
