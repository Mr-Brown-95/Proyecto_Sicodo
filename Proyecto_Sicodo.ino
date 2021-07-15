#include <DHT.h>
#include <LiquidCrystal_I2C.h>

#define DHTPIN 2 // Definimos el pin digital donde se conecta el sensor DTH11
#define DHTTYPE DHT11 // Dependiendo del tipo de sensor
#define ventiladorPin 13 //Definimos en pin digital donde se conecta ventilador
#define ledPin 12 //Definimos en pin digital donde se conecta el led
#define buttonPin 3 //Definimos en pin digital donde se conecta el push-button

int val = 0;//val se emplea para almacenar el estado del boton
int estado = 0;// 0 LED apagado, mientras que 1 encendido
int old_val = 0; // Almacena el antiguo valor de val

DHT dht(DHTPIN, DHTTYPE); // Inicializamos el sensor DHT11

LiquidCrystal_I2C lcd(0x27, 20, 4); // Crear el objeto lcd  dirección 0x27 para 16 caracteres y 2 lineas en el display

void setup() {

  Serial.begin(9600); // Inicializamos comunicación serie

  dht.begin();  // Comenzamos el sensor DHT
  lcd.init();// Inicializar el LCD
  lcd.backlight(); //Encender la luz de fondo.

  pinMode (ventiladorPin, OUTPUT);// Configuramos pin de salida
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

}

void sensorDTH() {

  delay(1000);// Esperamos 5 segundos entre medidas

  float humedad = dht.readHumidity();// Leemos la humedad relativa
  float temperatura = dht.readTemperature();  // Leemos la temperatura en grados centígrados (por defecto)

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

  if ( ( humedad > 33 ) || ( temperatura > 26 ) ) { // si la humeldad es mayor a 31ª o la temperatura es mayor a 22ª
    digitalWrite (ventiladorPin, HIGH); // enciende ventilador
  }
  else {// si no, se apaga
    digitalWrite (ventiladorPin, LOW);// apagar ventilador
  }
}

void manejadorLed() {

  val = digitalRead(buttonPin);
  if ((val == HIGH) && (old_val == LOW)) {
    estado = 1 - estado;
    delay(10);
  }
  old_val = val; // valor del antiguo estado
  if (estado == 1) {
    digitalWrite(ledPin, HIGH); // enciende el LED
  }
  else {
    digitalWrite(ledPin, LOW); // apagar el LED
  }
}


void loop() {

  manejadorLed();
  sensorDTH();

}
