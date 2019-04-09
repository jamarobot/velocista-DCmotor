#include <Wire.h>
#include "MCP23008.h"

// direcciones I2C de los componentes
#define DIR_I2C_GESTION   0x27
#define DIR_I2C_SENSORES  0x20

// estados para la programación basada en autómatas
#define PARADO            0
#define CARRERA           1

// pines del arduino
#define MOTOR_DER_DIR     4
#define MOTOR_DER_VEL     5
#define MOTOR_IZQ_DIR     12
#define MOTOR_IZQ_VEL     6

// constantes para el control PD y el ajuste de la velocidad de los motores
#define KP                20
#define KD                100
#define VELOCIDAD         255

// instancia de la placa de control
MCP23008 gestionI2C;
MCP23008 sensoresI2C;

// variables
unsigned char estado = PARADO;
int error = 0, error_anterior;
int sensor_error[4] = {-4, -2, 2, 4};

void setup() {

  pinMode(MOTOR_DER_DIR, OUTPUT);
  digitalWrite(MOTOR_DER_DIR, LOW);
  pinMode(MOTOR_DER_VEL, OUTPUT);
  pinMode(MOTOR_IZQ_DIR, OUTPUT);
  digitalWrite(MOTOR_IZQ_DIR, LOW);
  pinMode(MOTOR_IZQ_VEL, OUTPUT);

  gestionI2C.begin(DIR_I2C_GESTION);
  gestionI2C.pinMode(0x0F);
  gestionI2C.setPullup(0x0F);

  sensoresI2C.begin(DIR_I2C_SENSORES);
  sensoresI2C.pinMode(0xFF);
  sensoresI2C.setPullup(0x00);
}

void loop() {

  switch (estado) {
    case PARADO:
      // comprobación del botón de cambio de estado
      if (bigButtonPulsed() == true) {
        gestionI2C.write(4, HIGH);
        estado = CARRERA;                          // cambia al estado de carrera
      }
      break;
    case CARRERA:
      // comprobación del botón de cambio de estado 
      if (bigButtonPulsed() == true) {
        gestionI2C.write(4, LOW);
        estado = PARADO;                           // cambia al estado de parada
        digitalWrite(MOTOR_DER_VEL, LOW);         // detiene los dos motores
        digitalWrite(MOTOR_IZQ_VEL, LOW);
        break;                                    // se rompe el switch para que el control PD no active los motores de nuevo
      }
      
      // lectura de sensores
      char values = sensoresI2C.read();

      // cálculo del error
      error_anterior = error;                     // se guarda el error anterior para el control derivativo
      error = 0;                                  // error y número de sensores en negro se inician en 0
      int num_negros = 0;
      for (int i = 0; i < 4; i++) {
        if (bitRead(values, i) == HIGH) {         // si el sensor ha detectado negro,
          error += sensor_error[i];               // se suma el error correspondiente a ese sensor y
          num_negros++;                           // se incrementa el número de sensores que detectan negro
        }
      }
      if (num_negros != 0) {                      // sólo si se ha detectado algún negro,
        error /= num_negros;                      // se aplica la corrección al error para el caso de uno o dos sensores detectando negro
      }

      // aplicación del control PD
      int desfase = KP * error + KD* (error - error_anterior);

      // cálculo y limitación de las velocidades
      int velocidad_der = VELOCIDAD + desfase;    // se calcula la nueva velocidad para el motor derecho y
      if (velocidad_der < 0) {                    // se comprueba que la velocidad esté dentro del rango de 0 a 255.
        velocidad_der = 0;
      }
      else if (velocidad_der > 255) {
        velocidad_der = 255;
      }
      int velocidad_izq = VELOCIDAD - desfase;    // se calcula la nueva velocidad para el motor izquierdo y
      if (velocidad_izq < 0) {                    // se comprueba que la velocidad esté dentro del rango de 0 a 255.
        velocidad_izq = 0;
      }
      else if (velocidad_izq > 255) {
        velocidad_izq = 255;
      }

      //actualización de velocidad en motores
      analogWrite(MOTOR_DER_VEL, velocidad_der);
      analogWrite(MOTOR_IZQ_VEL, velocidad_izq);

      break;
  }
  delay(5);                                       // tiempo de espera hasta la próxima aplicación del control PD
}

unsigned char estado_anterior_boton = HIGH;

bool bigButtonPulsed() {

  // lectura del estado del botón
  char estado_boton = gestionI2C.read(3);
  // identificación de un flanco ascendente en base al estado anterior y al actual
  if ((estado_anterior_boton == HIGH) && (estado_boton == LOW)) {
    estado_anterior_boton = estado_boton;
    return true;
  }
  else {
    estado_anterior_boton = estado_boton;
    return false;
  }
}
