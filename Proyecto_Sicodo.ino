#include <DHT.h>
#include <LiquidCrystal_I2C.h>

#define DHTPIN 2 // Definimos el pin digital donde se conecta el sensor DTH11
#define DHTTYPE DHT11 // Definimos el tipo de sensor

#define pirPin 3 //Definimos en pin digital donde se conecta el sensor de movimiento
#define lightSensorPin A3 //Definimos en pin analogico donde se conecta el sensor de luz
#define buttonPin 4 //Definimos en pin digital donde se conecta el push-button

#define ventiladorPin 13 //Definimos en pin digital donde se conecta ventilador
#define ledPirPin 12 //Definimos en pin digital donde se conecta el led
#define ledLightSensorPin 11 //Definimos en pin digital donde se conecta el led
#define ledButtonPin 10 //Definimos en pin digital donde se conecta el led(RELEY+FOCO)

int temperature;// Almacena la temperatura
int humidity;// Almacena la humedad

int valuePir = 0;//valuePir se emplea para almacenar el estado del sensor PIR
int valueLightSensor = 0;//valueLightSensor se emplea para almacenar las lecruras dell sensor de luz

int valueLed = 0;//valueLed se emplea para almacenar el estado del led
int state = 0;// 0 LED apagado, mientras que 1 encendido
int oldValueLed = 0; // Almacena el antiguo valor de val

DHT dht(DHTPIN, DHTTYPE); // Inicializamos el sensor DHT11

LiquidCrystal_I2C lcd(0x27, 20, 4); // Crear el objeto lcd  dirección 0x27 para 16 caracteres y 2 lineas en el display

void sensorDTH() {

  delay(1000);

  humidity = dht.readHumidity();
  temperature = dht.readTemperature();

  // Comprobamos si ha habido algún error en la lectura
  if (isnan(humidity) || isnan(temperature)) {
    lcd.setCursor(0, 0);
    lcd.print("Error obteniendo");
    lcd.setCursor(0, 1);
    lcd.print("los datos del sensor DHT11");
    return;
  }

  //imprimimos los valores dentro del lcd
  lcd.setCursor(0, 0);// Ubicamos el cursor en la primera posición(columna:0) de la segunda línea(fila:1)
  lcd.print("Humedad: ");// Escribimos el Mensaje en el LCD.
  lcd.print(humidity);// Escribimos el valor de la variable humedad.
  lcd.print("   ");
  lcd.setCursor(0, 1);
  lcd.print("Temperatura: ");
  lcd.print(temperature);

  //( humedad > 33 ) || ( temperatura > 26 )
  if (  temperature >= 30  ) {
    digitalWrite (ventiladorPin, HIGH);
  }
  else {
    digitalWrite (ventiladorPin, LOW);
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

void sensorLight() {

  valueLightSensor = analogRead(ledLightSensorPin);

  //Serial.println(valueLightSensor);

  if (valueLightSensor < 750) {
    digitalWrite(ledLightSensorPin, HIGH);
  } else {
    digitalWrite(ledLightSensorPin, LOW );
  }
}

void pushButton() {

  valueLed = digitalRead(buttonPin);

  if ((valueLed == HIGH) && (oldValueLed == LOW)) {
    state = 1 - state;
    delay(10);
  }
  
  oldValueLed = valueLed;
  
  if (state == 1) {
    digitalWrite(ledButtonPin, HIGH);
  }
  else {
    digitalWrite(ledButtonPin, LOW);
  }
}

void setup() {

  Serial.begin(9600); // Inicializamos comunicación serie

  dht.begin();  // Comenzamos el sensor DHT
  lcd.init();// Inicializar el LCD
  lcd.backlight(); //Encender la luz de fondo.

  pinMode (ventiladorPin, OUTPUT);

  pinMode(ledPirPin, OUTPUT);
  pinMode(pirPin, INPUT);

  pinMode(ledLightSensorPin, OUTPUT);
  pinMode(lightSensorPin, INPUT);

  pinMode(ledButtonPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

}

void loop() {

  sensorDTH();
  sensorPir();
  sensorLight();
  pushButton();

}
