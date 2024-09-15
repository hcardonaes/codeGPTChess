/*
 Name:		codeGPTChess.ino
 Created:	13/09/2024 19:53:23
 Author:	Ofel
*/
#include <Math.h>
#include <Arduino.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

struct Coordenadas {
    double x;
    double y;
};

struct Angles {
    double theta1;
    double theta2;
};

// Longitud de los brazos del SCARA
const float l1 = 100.0;  // Longitud del primer brazo
const float l2 = 100.0;  // Longitud del segundo brazo

// Últimos ángulos (posición inicial es 0,0)
float last_theta1 = 0.0;
float last_theta2 = 0.0;

// Configuración del motor del hombro (NEMA17 con TMC2209)
constexpr auto STEP_PIN = 6;
constexpr auto DIR_PIN = 3;
constexpr auto ENABLE_PIN = 51;
constexpr auto R_SENSE = 0.11f;      // R_SENSE para cálculo de corriente
constexpr auto DRIVER_ADDRESS = 0b00;       // Dirección del driver TMC2209 según MS1 y MS2
#define SERIAL_PORT Serial3

// Configuración del motor del codo (24BYJ48 con ULN2003)
constexpr auto HALFSTEP = 8;
constexpr auto motorPin1 = 8;     // IN1 on ULN2003 ==> Blue   on 28BYJ-48
constexpr auto motorPin2 = 9;     // IN2 on ULN2004 ==> Pink   on 28BYJ-48
constexpr auto motorPin3 = 10;    // IN3 on ULN2003 ==> Yellow on 28BYJ-48
constexpr auto motorPin4 = 11;    // IN4 on ULN2003 ==> Orange on 28BYJ-48

constexpr auto LEVA_HOMBRO_PIN = 5;
constexpr auto LEVA_CODO_PIN = 4;

double angHombro = 0.0;
double angCodo = 0.0;

double posicionActualX = 0.0;
double posicionActualY = 0.0;

bool homingRealizado = false;

long pasosMotores[2]; // Array para almacenar las pasosMotores objetivo

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
AccelStepper hombro(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
AccelStepper codo(HALFSTEP, motorPin1, motorPin3, motorPin2, motorPin4);
MultiStepper motores;

Coordenadas calcularCoordenadasDesdeCentro(String comando) {
    int columna = comando[0] - 'a'; // Columna [a-h]
    int fila = comando[1] - '1';    // Fila [1-8]
    double x = (columna - 3.5) * 40; // Ajuste para centrar en el tablero
    double y = (fila - 3.5) * 40;    // Ajuste para centrar en el tablero
    return { x, y };
}

void realizarHoming() {
	Serial.println("Realizando homing para los motores...");
	// Homing para el codo
	bool estadoLeva = digitalRead(LEVA_CODO_PIN);
	if (estadoLeva == LOW) {
		// Mover el motor hasta que se active el fin de carrera
		codo.setSpeed(-800);
		while (digitalRead(LEVA_CODO_PIN) == LOW) {
			codo.runSpeed();
		}
	}
	else {
		codo.setSpeed(800);
		while (digitalRead(LEVA_CODO_PIN) == HIGH) {
			codo.runSpeed();
		}
	}
	codo.setCurrentPosition(0); // Establece la posición actual transitoria como cero

	// Homing para el hombro
	estadoLeva = digitalRead(LEVA_HOMBRO_PIN);
	//Serial.print("Estado leva hombro: "); //Serial.println(estadoLeva);
	if (estadoLeva == LOW) {
		// Mover el motor hasta que se active el fin de carrera
		hombro.setSpeed(-800);
		while (digitalRead(LEVA_HOMBRO_PIN) == LOW) {
			hombro.runSpeed();
		}
	}
	else {
		hombro.setSpeed(800);
		while (digitalRead(LEVA_HOMBRO_PIN) == HIGH) {
			hombro.runSpeed();
		}
	}
	hombro.setCurrentPosition(0); // Establece la posición actual como cero

	pasosMotores[0] = 4060;
	pasosMotores[1] = -1625;
	// Mover los motores a la posición deseada
	motores.moveTo(pasosMotores);
	motores.runSpeedToPosition();
	// Establece la posición actual como cero
	codo.setCurrentPosition(0);
	hombro.setCurrentPosition(0);

}


// Función para calcular la cinemática inversa

//void calcularCinematicaInversa(float x, float y) {
//    // Calculamos theta2 usando la fórmula de cinemática inversa
//    float cos_theta2 = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2);
//	//Serial.print("cos_theta2: "); //Serial.println(cos_theta2);
//
//    // Aseguramos que cos_theta2 esté dentro del rango válido [-1, 1] para evitar errores de dominio
//    if (cos_theta2 > 1.0) cos_theta2 = 1.0;
//    if (cos_theta2 < -1.0) cos_theta2 = -1.0;
//
//    // Solución para theta2 (ángulo del segundo brazo)
//    float theta2_1 = acos(cos_theta2);
//    float theta2_2 = -acos(cos_theta2);
//	////Serial.print("Sol. theta2_1: "); //Serial.println(theta2_1 * 180.0 / PI);
//	////Serial.print("Sol. theta2_2: "); //Serial.println(theta2_2 * 180.0 / PI);
//
//
//    // Calculamos theta1 para ambas soluciones de theta2
//    float theta1_1 = atan2(y, x) - atan2(l2 * sin(theta2_1), l1 + l2 * cos(theta2_1));
//    float theta1_2 = atan2(y, x) - atan2(l2 * sin(theta2_2), l1 + l2 * cos(theta2_2));
//
//    // Normalizamos los ángulos entre [-pi, pi] para evitar ángulos mayores a 360 grados
//    theta1_1 = atan2(sin(theta1_1), cos(theta1_1));
//    theta1_2 = atan2(sin(theta1_2), cos(theta1_2));
//    theta2_1 = atan2(sin(theta2_1), cos(theta2_1));
//    theta2_2 = atan2(sin(theta2_2), cos(theta2_2));
//
//	//Serial.print("Sol. theta1_1: "); //Serial.println(theta1_1 * 180.0 / PI);
//	//Serial.print("Sol. theta1_2: "); //Serial.println(theta1_2 * 180.0 / PI); 
//	//Serial.print("Sol. theta2_1: "); //Serial.println(theta2_1 * 180.0 / PI); 
//	//Serial.print("Sol. theta2_2: "); //Serial.println(theta2_2 * 180.0 / PI);
//
//    // Elegimos la solución que minimiza el recorrido angular
//    float delta_theta1_1 = fabs(theta1_1 - last_theta1);
//	// imprime el valor de theta1_1 y last_theta1
//	//Serial.print("theta1_1: ");
//	//Serial.println(theta1_1 * 180.0 / PI);
//	//Serial.print("last_theta1: ");
//	//Serial.println(last_theta1 * 180.0 / PI);
//
//    float delta_theta2_1 = fabs(theta2_1 - last_theta2);
//    float delta_theta1_2 = fabs(theta1_2 - last_theta1);
//    float delta_theta2_2 = fabs(theta2_2 - last_theta2);
//	//Serial.println("Deltas:");
//	//Serial.println(delta_theta1_1 * 180.0 / PI);
//	//Serial.println(delta_theta2_1 * 180.0 / PI);
//	//Serial.println(delta_theta1_2 * 180.0 / PI);
//	//Serial.println(delta_theta2_2 * 180.0 / PI);
//
//    if ((delta_theta1_1 + delta_theta2_1) < (delta_theta1_2 + delta_theta2_2)) {
//        last_theta1 = theta1_1;
//        last_theta2 = theta2_1;
//    }
//    else {
//        last_theta1 = theta1_2;
//        last_theta2 = theta2_2;
//    }
//
//	angHombro = last_theta1 * 180.0 / PI;
//	angCodo = 180 + (last_theta2 * 180.0 / PI);
//	if (angCodo >= 180) angCodo = 360 - angCodo;
//	//Serial.print("Angulo hombro: "); Serial.println(angHombro);
//	//Serial.print("Angulo codo: "); Serial.println(angCodo);
//}

Angles calcularCinematicaInversa(float x, float y) {
	// Calculamos theta2 usando la fórmula de cinemática inversa
	float cos_theta2 = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2);

	// Aseguramos que cos_theta2 esté dentro del rango válido [-1, 1] para evitar errores de dominio
	if (cos_theta2 > 1.0) cos_theta2 = 1.0;
	if (cos_theta2 < -1.0) cos_theta2 = -1.0;

	// Solución para theta2 (ángulo del segundo brazo)
	float theta2_1 = acos(cos_theta2);
	float theta2_2 = -acos(cos_theta2);

	// Calculamos theta1 para ambas soluciones de theta2
	float theta1_1 = atan2(y, x) - atan2(l2 * sin(theta2_1), l1 + l2 * cos(theta2_1));
	float theta1_2 = atan2(y, x) - atan2(l2 * sin(theta2_2), l1 + l2 * cos(theta2_2));

	// Normalizamos los ángulos entre [-pi, pi] para evitar ángulos mayores a 360 grados
	theta1_1 = atan2(sin(theta1_1), cos(theta1_1));
	theta1_2 = atan2(sin(theta1_2), cos(theta1_2));
	theta2_1 = atan2(sin(theta2_1), cos(theta2_1));
	theta2_2 = atan2(sin(theta2_2), cos(theta2_2));

	// Elegimos la solución que minimiza el recorrido angular
	float delta_theta1_1 = fabs(theta1_1 - last_theta1);
	float delta_theta2_1 = fabs(theta2_1 - last_theta2);
	float delta_theta1_2 = fabs(theta1_2 - last_theta1);
	float delta_theta2_2 = fabs(theta2_2 - last_theta2);

	Angles angulos;
	if ((delta_theta1_1 + delta_theta2_1) < (delta_theta1_2 + delta_theta2_2)) {
		angulos.theta1 = theta1_1;
		angulos.theta2 = theta2_1;
	}
	else {
		angulos.theta1 = theta1_2;
		angulos.theta2 = theta2_2;
	}

	return angulos;
}


//void moverAPosicion(String comando) {
//	Serial.print("Moviendo a la posicion "); Serial.print(comando); Serial.println("...");
//
//	// Convertir notación de ajedrez a coordenadas en mm desde el centro del tablero
//	Coordenadas coord = calcularCoordenadasDesdeCentro(comando);
//	double x = coord.x;
//	double y = coord.y;
//	calcularCinematicaInversa(x, y);
//	moverMotores();
//	// Actualizar la posición actual del efector
//	posicionActualX = x;
//	posicionActualY = y;
//}

void moverAPosicion(String comando) {
	Serial.print("Moviendo a la posicion "); Serial.print(comando); Serial.println("...");

	// Convertir notación de ajedrez a coordenadas en mm desde el centro del tablero
	Coordenadas coord = calcularCoordenadasDesdeCentro(comando);
	double xDestino = coord.x;
	double yDestino = coord.y;
	Serial.print("Destino: x = "); Serial.print(xDestino); Serial.print(", y = "); Serial.println(yDestino);

	// Mover en línea recta desde la posición actual a la posición de destino
	moverEnLineaRecta(posicionActualX, posicionActualY, xDestino, yDestino, 10);

	// Actualizar la posición actual del efector
	posicionActualX = xDestino;
	posicionActualY = yDestino;
}


void moverEnGrados(double anguloHombro, double anguloCodo) {
	// Convertir ángulos a pasos de motor (en pasos)
	long pasosHombro = calcularPasosHombro(anguloHombro);
	long pasosCodo = calcularPasosCodo(anguloCodo);

	// Imprimir los ángulos y pasos calculados
	Serial.print("Moviendo a ángulos: Hombro = "); Serial.print(anguloHombro);
	Serial.print(" grados, Codo = "); Serial.print(anguloCodo); Serial.println(" grados");
	Serial.print("Pasos: Hombro = "); Serial.print(pasosHombro);
	Serial.print(", Codo = "); Serial.println(pasosCodo);

	// Mover los motores a las posiciones calculadas
	pasosMotores[0] = pasosHombro;
	pasosMotores[1] = pasosCodo;
	motores.moveTo(pasosMotores);
	motores.runSpeedToPosition();

	// Actualizar los ángulos actuales
	last_theta1 = anguloHombro * PI / 180.0;
	last_theta2 = anguloCodo * PI / 180.0;
}

void moverMotores(double theta1, double theta2) {
	// Convertir ángulos a pasos de motor
	long pasosHombro = calcularPasosHombro(theta1);
	long pasosCodo = calcularPasosCodo(theta2);

	// Imprimir los pasos calculados
	Serial.print("Pasos: ("); Serial.print(pasosHombro);
	Serial.print(", "); Serial.print(pasosCodo); Serial.println(")");

	// Mover los motores a las posiciones calculadas
	pasosMotores[0] = pasosHombro;
	pasosMotores[1] = pasosCodo;
	motores.moveTo(pasosMotores);
	motores.runSpeedToPosition();

	// Actualizar los ángulos actuales
	last_theta1 = theta1;
	last_theta2 = theta2;
}


long calcularPasosHombro(double theta1) {
	double pasosPorGradoHombro = 16150 / 360; // 15450 pasosMotores por vuelta
	long pasos = theta1 * pasosPorGradoHombro;
	return pasos;
}

long calcularPasosCodo(double theta2) {
	double pasosPorGradoCodo = 4140.0 / 360;
	long pasos = theta2 * pasosPorGradoCodo;
	//Serial.print("theta2: "); Serial.println(theta2);
	return pasos;
}

void moverEnLineaRecta(double xInicio, double yInicio, double xFin, double yFin, int pasos) {
	double deltaX = (xFin - xInicio) / pasos;
	double deltaY = (yFin - yInicio) / pasos;

	for (int i = 0; i <= pasos; i++) {
		double xIntermedio = xInicio + i * deltaX;
		double yIntermedio = yInicio + i * deltaY;
		Serial.print("xIntermedio: "); Serial.print(xIntermedio);	
		Serial.print(", yIntermedio: "); Serial.println(yIntermedio);

		// Calcular los ángulos para las coordenadas intermedias
		Angles angulos = calcularCinematicaInversa(xIntermedio, yIntermedio);
		Serial.print("hombro: "); Serial.print(angulos.theta1 * 180.0 / PI);
		Serial.print(", codo: "); Serial.println(angulos.theta2 * 180.0 / PI);

		// Mover los motores a los ángulos calculados
		moverMotores(angulos.theta1, angulos.theta2);
	}
}

void setup() {
		Serial.begin(115200);

		SERIAL_PORT.begin(115200); // Configura la comunicación con el TMC2209

		// Configuración del driver TMC2209
		pinMode(ENABLE_PIN, OUTPUT);
		digitalWrite(ENABLE_PIN, LOW); // Habilitar el driver

		driver.begin();
		driver.toff(5);
		driver.rms_current(1000); // Corriente en mA para el NEMA17
		driver.microsteps(16);   // Configurar microstepping
		driver.en_spreadCycle(false); // Deshabilitar spreadCycle, usa StealthChop
		driver.pwm_autoscale(true); // Activar auto scale PWM
		driver.enn(); // Habilitar el driver

		// Configuración de los pines
		pinMode(LEVA_HOMBRO_PIN, INPUT_PULLUP);
		pinMode(LEVA_CODO_PIN, INPUT_PULLUP);

		// Configuración inicial de los motores
		hombro.setMaxSpeed(800);
		hombro.setAcceleration(800);
		codo.setMaxSpeed(400);
		codo.setAcceleration(200);

		// Add the motores to the MultiStepper object
		motores.addStepper(hombro); // position '0'
		motores.addStepper(codo);    // position '1'

		// Homing: Mueve los motores hasta posicionarse en el origen
		Serial.println("Realizando homing...");
		realizarHoming();
		delay(3000);

		//moverEnGrados(0, 360);
		//while (true)
		//{}
		Serial.println("Homing completado. Listo para recibir comandos.");
		homingRealizado = true;
		Serial.flush();
}



void loop() {
	if (Serial.available() > 0) {
		String input = Serial.readStringUntil('\n');
		Serial.print("Comando recibido: "); Serial.println(input);

		// Validar que el comando tenga la estructura de una casilla de ajedrez
		if (input.length() == 2 && input[0] >= 'a' && input[0] <= 'h' && input[1] >= '1' && input[1] <= '8') {
			//Serial.print("Comando válido: "); Serial.println(input);
			moverAPosicion(input);
		}
		else {
			//Serial.println("Comando inválido. Debe ser una casilla de ajedrez (a-h, 1-8).");
		}
	}
}


