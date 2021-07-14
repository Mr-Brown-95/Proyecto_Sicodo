#include <DHT.h>
#include <LiquidCrystal_I2C.h>

#define DHTPIN 2 // Definimos el pin digital donde se conecta el sensor
#define DHTTYPE DHT11 // Dependiendo del tipo de sensor
int ventiladorPin = 13; //definimos en pin del ventilador

DHT dht(DHTPIN, DHTTYPE); // Inicializamos el sensor DHT11

LiquidCrystal_I2C lcd(0x27, 20, 4); // Crear el objeto lcd  dirección 0x27 para 16 caracteres y 2 lineas en el display
void setup() {

  Serial.begin(9600); // Inicializamos comunicación serie
  dht.begin();  // Comenzamos el sensor DHT
  lcd.init();// Inicializar el LCD
  lcd.backlight(); //Encender la luz de fondo.
  pinMode (ventiladorPin, OUTPUT);// Configuramos pin de salida

}
void sensorDTH() {

  delay(5000);// Esperamos 5 segundos entre medidas

  float h = dht.readHumidity();// Leemos la humedad relativa
  float t = dht.readTemperature();  // Leemos la temperatura en grados centígrados (por defecto)

  // Comprobamos si ha habido algún error en la lectura
  if (isnan(h) || isnan(t)) {
    lcd.setCursor(0, 0);
    lcd.print("Error obteniendo");
    lcd.setCursor(0, 1);
    lcd.print("los datos del sensor DHT11");
    return;
  }

  lcd.setCursor(0, 0);
  lcd.print("Humedad: ");
  lcd.print(h);
  lcd.print("   ");
  lcd.setCursor(0, 1);
  lcd.print("Temperatura: ");
  lcd.print(t);

  if ( ( h > 33 ) || ( t > 26 ) ) { // si la humeldad (h) es mayor a 31ª o la temperatura (t) es mayor a 22ª
    digitalWrite (ventiladorPin, HIGH); // se prende ventilador
  }
  else {// si no, se apaga
    digitalWrite (ventiladorPin, LOW);
  }
}

void loop() {

  sensorDTH();

}
