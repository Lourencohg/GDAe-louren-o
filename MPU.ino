
#include <Wire.h>
#include <Kalman.h>

#define RESTRICT_PITCH // Commentar essa linha caso sejpa necessario restringir para ±90deg: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

/*VALORES DAS INSTANCIAS KALMANS*/
Kalman kalmanX;
Kalman kalmanY;

/* Data */
double accX, accY, accZ; // valores da aceleração obtida em cada eixo
double gyroX, gyroY, gyroZ; // valores das velocidade angular em cada eixo
//int16_t tempRaw; //temperatura caso seja necessario

double gyroXangle, gyroYangle; // Angulo calculado somente com os valores da velocidade angular giroscopio
double kalAngleX, kalAngleY; // Valores calculados com o kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer para I2C data
int led = 2; // Setando o valor da variavel led para ser igual ao valor do pino do led interno
int porta = 19;

void setup() {
  Serial.begin(115200);
  pinMode (led, OUTPUT);  
  pinMode (porta, OUTPUT);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Setando a frenquencia I2C para 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Setando a frenquencia I2C para 400kHz
#endif

  i2cData[0] = 7; // Seta o sample rate para 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling  por meio do registradores 3B a 40  encontrado na pag x do registradores
  i2cData[2] = 0x00; // Seta a escala do giroscopio para ±250deg/s por meio do registradores 43 a 48  encontrado na pag x do registradores
  i2cData[3] = 0x00; // Seta a escala do acelerometro para ±2g caso(00)
  while (i2cWrite(0x19, i2cData, 4, false));
  while (i2cWrite(0x6B, 0x01, true)); // Disabilidantando o slepp mode do mpu-6050, olhar p.9 do mapa de registros do mpu

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // lé "WHO_AM_I" register
    Serial.print(F("Erro na leitura do sensor"));
    while (1);
  }

  delay(100); // Estabilização/ verificar a necessidade desse valor

  /* Setando os valores iniciais do filtro kalman */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]); //obtendo
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

/*CALCULA ROLL E PITCH DO COM BASE NOS VALOREZ */
  // Referencia: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs o valor de -π ate π (radianos) - http://en.wikipedia.org/wiki/Atan2
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Setando o angulo inicial do eixo X
  kalmanY.setAngle(pitch); //Setando o angulo incial do eixo y
  gyroXangle = roll;
  gyroYangle = pitch;

  timer = micros();
  
}

void loop() {
  /* Atualização de todos os valores */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);//pegando os valores da aceleração no eixo x encontradas no diretorio
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);//pegando os valores da aceleração no eixo y encontradas no diretorio
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);//pegando os valores da aceleração no eixo z encontradas no diretorio
  //tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);//pegando os valores da velocidade angular no eixo x encontradas no diretorio
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);//pegando os valores da velocidade angular no eixo x encontradas no diretorio
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;//pegando os valores da velocidade angular no eixo x encontradas no diretorio

  double dt = (double)(micros() - timer) / 1000000; // Delta t
  timer = micros();

  /*CALCULANDO OS VALORES PARA*/
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; //Convertendo para deg/s
  double gyroYrate = gyroY / 131.0; //Convertendo para deg/s

#ifdef RESTRICT_PITCH
  // esse codigo corrige o problema de trasição dos valores muito bruscamente( quando ele muda de -180 para 180 do nada)
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calcula o valor do angulo no eixo X usando o filtro kalman

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Inverter o rate para se adaptar ao valor de leitura restrita
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  /// esse codigo corrige o problema de trasição dos valores muito bruscamente( quando ele muda -180 para 10 do nada)
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calcula o angulo y usando o filtro kalman

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Inverte o rate, para caber na leitura restrita do acelerometro
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calcula o angulo x usando o filtro kalman
#endif

  gyroXangle += gyroXrate * dt; // Calcula o angulo do giroscopia sem filtrpo
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  if (kalAngleX < -90  || kalAngleX > 90)
    digitalWrite(led ,HIGH);
    digitalWrite(porta ,HIGH);
    
  if (kalAngleY < -90  || kalAngleY > 90)
    digitalWrite(led ,HIGH);
    digitalWrite(porta ,HIGH);
/*
#if 0 // Setar para 1 caso queira escrever os valores po meio da porta serial
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
#endif

  Serial.print(roll); Serial.print("\t");
  Serial.print(gyroXangle); Serial.print("\t");
  Serial.print(kalAngleX); Serial.print("\t");

  Serial.print("\t");

  Serial.print(pitch); Serial.print("\t");
  Serial.print(gyroYangle); Serial.print("\t");
  Serial.print(kalAngleY); Serial.print("\t");
*/


/*
#if 0 // Setar para 1 caso queira os valores da temperatura na porta serial
  Serial.print("\t");

  double temperature = (double)tempRaw / 340.0 + 36.53;
  Serial.print(temperature); Serial.print("\t");
#endif
*/
  Serial.print("\r\n");
  delay(2);
}
