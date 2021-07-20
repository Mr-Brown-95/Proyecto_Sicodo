#include <DHT.h>
#include <LiquidCrystal_I2C.h>

#define DHTPIN 2 // Definimos el pin digital donde se conecta el sensor DTH11
#define DHTTYPE DHT11 // Dependiendo del tipo de sensor
#define ventiladorPin 13 //Definimos en pin digital donde se conecta ventilador
#define ledbuttonPin 12 //Definimos en pin digital donde se conecta el led
#define buttonPin 3 //Definimos en pin digital donde se conecta el push-button
#define pirPin 4 //Definimos en pin digital donde se conecta el sensor de movimiento
#define ledPirPin 11 //Definimos en pin digital donde se conecta el led

int valueLed = 0;//valueLed se emplea para almacenar el estado del led
int state = 0;// 0 LED apagado, mientras que 1 encendido
int oldValueLed = 0; // Almacena el antiguo valor de val

int temperatura;// Almacena la temperatura
int humedad;// Almacena la humedad

int valuePir = 0;//valuePir se emplea para almacenar el estado del sensor PIR

DHT dht(DHTPIN, DHTTYPE); // Inicializamos el sensor DHT11

LiquidCrystal_I2C lcd(0x27, 20, 4); // Crear el objeto lcd  dirección 0x27 para 16 caracteres y 2 lineas en el display

void setup() {

  Serial.begin(9600); // Inicializamos comunicación serie

  dht.begin();  // Comenzamos el sensor DHT
  lcd.init();// Inicializar el LCD
  lcd.backlight(); //Encender la luz de fondo.

  pinMode (ventiladorPin, OUTPUT);

  pinMode(ledbuttonPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  pinMode(ledPirPin, OUTPUT);
  pinMode(pirPin, INPUT);

}

void sensorDTH() {

  delay(1000);

  humedad = dht.readHumidity();
  temperatura = dht.readTemperature();

  // Comprobamos si ha habido algún error en la lectura
  if (isnan(humedad) || isnan(temperatura)) {
    lcd.setCursor(0, 0);
    lcd.print("Error obteniendo");
    lcd.setCursor(0, 1);
    lcd.print("los datos del sensor DHT11");
    return;
  }

  //imprimimos los valores dentro del lcd
  lcd.setCursor(0, 0);// Ubicamos el cursor en la primera posición(columna:0) de la segunda línea(fila:1)
  lcd.print("Humedad: ");// Escribimos el Mensaje en el LCD.
  lcd.print(humedad);// Escribimos el valor de la variable humedad.
  lcd.print("   ");
  lcd.setCursor(0, 1);
  lcd.print("Temperatura: ");
  lcd.print(temperatura);

  //( humedad > 33 ) || ( temperatura > 26 )
  if (  temperatura >= 30  ) { 
    digitalWrite (ventiladorPin, HIGH);
  }
  else {
    digitalWrite (ventiladorPin, LOW);
  }
}

void manejadorLed() {

  valueLed = digitalRead(buttonPin);
  
  if ((valueLed == HIGH) && (oldValueLed == LOW)) {
    state = 1 - state;
    delay(10);
  }
  oldValueLed = valueLed;
  if (state == 1) {
    digitalWrite(ledbuttonPin, HIGH);
  }
  else {
    digitalWrite(ledbuttonPin, LOW);
  }
}

void sensorPir() {

  valuePir = digitalRead(pirPin);

  if (valuePir == HIGH)
  {
    digitalWrite(ledPirPin, HIGH);
    delay(5000);
    digitalWrite(ledPirPin, LOW);
    delay(50);
  }
  else
  {
    digitalWrite(ledPirPin, LOW);
  }
}

void loop() {

  manejadorLed();
  sensorDTH();
  sensorPir();

}
