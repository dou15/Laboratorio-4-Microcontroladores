//////////////////////////////////////////////////////////////////////////////////
//          Importar Bibliotecas               
//////////////////////////////////////////////////////////////////////////////////

#include <DHT.h>
#include <PCD8544.h>
#include <EEPROM.h>

//////////////////////////////////////////////////////////////////////////////////
// Definiciones, instancias de bibliotecas              
//////////////////////////////////////////////////////////////////////////////////

#define DHTPIN A1         // Digital pin connected to the DHT sensor

#define DHTTYPE DHT22     // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE); // Instancia biblioteca DHT(PUERTO,TypoSensor)

#define RG11_Pin 9// RG-11 is connected to digital pin 2

#define switchDisplay 8 // Switch encendido apagado pantalla
PCD8544 lcd;
// The dimensions of the LCD (in pixels)...
static const byte LCD_WIDTH = 84;
static const byte LCD_HEIGHT = 48;
// A custom "degrees" symbol...
static const byte DEGREES_CHAR = 1;
static const byte degrees_glyph[] = { 0x00, 0x07, 0x05, 0x07, 0x00 };

struct MyObject{
  float field1;
  float field2;
  float field3;
  float field4;
  char name[10];
};

//////////////////////////////////////////////////////////////////////////////////
//          Declaración de Variables                   
//////////////////////////////////////////////////////////////////////////////////

float rtdSensor = A0;        // Puerto A0 empleado para tomar la señal del RTD 
float V0_rtd;                    // Tension detectada en el RTD
float Vi_rtd = 5;                // Tensión suplida a R1 y Rt  
float R1_rtd = 250;              // Resistencia R1 conectada entre Vi y Rt
float Rt;                    // Resistencia calculada del RTD
float R0 = 100;              // Valor resistivo de Rt a 0 grados celcius
float a_const = 0.0039083;    // a y b son constantes provistas por el
float b_const = -0.0000005775;  // fabricante del RTD para calcular la
                             // temperatura detectada
float temp;                  // Variable función readTemp para calcular Temperatura
float temperatura;

float humedad;
float temperaturadht22;

float ldrSensor = A2;            // Sensor LDR conectado a A2
float V0_ldr;                    // Tension detectada en el LDR
float Vi_ldr = 5;                // Tensión suplida a R1 y LDR
float R1_ldr = 1000;             // Resistencia R1 conectada entre Vi y LDR
float R_ldr;
float IntensidadLuminosa;

float jlfsx2VoltageRead; // Señal registrada en A3 para el jlfsx2
float jlfsx2Voltage;     // Tensión determinada en A3
float windspeedread;     // Velocidad del viento

int RG11_read;           // Almacena lectura RG11 sensor lluvia
char rg11_rain[10];      // Almacena si llueve 

int switchDisplay_read;  // Almacena estado on/off pantalla

int counterTimer = 0;    // Contador utilizado en las interrupciones
                         // Que escriben en memoria EEPROM y envian por USART
                         
int eeAddress = 0;      // Almacena posición de la memoria a almacenar                           

//////////////////////////////////////////////////////////////////////////////////
//          SETUP                     
//////////////////////////////////////////////////////////////////////////////////

void setup(){
    clearMemory();      // Límpia la memoria EEPROM al inicio
    
    // Config TIMER1
    TCCR1A = 0;                                   // Registro de control A en 0
    TCCR1B = 0;                                   // Registrador en 0
    TCNT1  = 0;                                   // Inicio temporización
    OCR1A = 65535;                                // Cargar registro comparación: 16MHz/1024/1Hz -1 = 15624 = 0X3D08
    TCCR1B |= (1 << WGM12)|(1<<CS10)|(1 << CS12); // modo CTC, prescaler 1024: CS12 = 1 e CS10 = 1  
    TIMSK1 |= (1 << OCIE1A);                      // habilita interrupción por igualdade de comparación
    
    
    Serial.begin(9600);        // Comunicación serial
    pinMode(rtdSensor, INPUT); // configura puerto sensor RTD 
    dht.begin();               // Inicializa biblioteca DHT22
    pinMode(ldrSensor, INPUT); // configura puerto sensor LDR
    pinMode(RG11_Pin, INPUT);  // configura puerto sensor lluvia
    // Configura pantalla     
    pinMode(switchDisplay, INPUT);     
    lcd.begin(LCD_WIDTH, LCD_HEIGHT);  
    // Register the custom symbol...
    lcd.createChar(DEGREES_CHAR, degrees_glyph);
    lcd.clear();
}

//////////////////////////////////////////////////////////////////////////////////
//          LOOP                    
//////////////////////////////////////////////////////////////////////////////////

void loop(){
    
    // Lectura Temperatura RTD
    temperatura = readTemp(rtdSensor); // Registra temperatura RTD
    Serial.println(temperatura);
    
    // Lectura Humedad DHT22
    // Espera mientras realiza la medición de humedad
    delay(2000);  
    // El sensor humadad DHT22 ocupa 2 segundos para realizar la lectura
    humedad = dht.readHumidity();
    // Read temperature as Celsius (the default)
    temperaturadht22 = dht.readTemperature();  
    Serial.print("Humidity: ");
    Serial.print(humedad);
    Serial.print("%  Temperature: ");
    Serial.print(temperaturadht22);
    Serial.print("\t");
    Serial.print(char(176));
    Serial.print("C ");
    Serial.println();
    
    // Lectura intensidad luminica
    readldr(ldrSensor);
    delay(1000);
    
    // Lectura velocidad del viento
    windSpeed(jlfsx2VoltageRead);
    
    // Lectura de lluvia
    rainSensor();

    // Pantalla
    displaylcd();
    
}

//////////////////////////////////////////////////////////////////////////////////
//          FUNCIONES                     
//////////////////////////////////////////////////////////////////////////////////

// calcular temperatura RTD
float readTemp(float rtdSensor){
    V0_rtd   = analogRead(rtdSensor)*(5.0 / 1023.0); // Calcula tensión en la resistencia Rt a traves de la entrada A0
    Rt   = (V0_rtd*R1_rtd)/(Vi_rtd-V0_rtd);                      // Determina valor de Rt
    if(Rt>=100){
      temp = (-a_const*R0 + sqrt(a_const*a_const*R0*R0 - 4*b_const*R0 * (R0-Rt))) / (2*b_const*R0); // Calcula temperatura para Rt >= 100 ohms
    }
    else{
      temp = (Rt - 100) / 0.4;
    }
    return temp;
    
}

// Calcular intensidad  Intensidad luminosa
float readldr(float ldrSensor){
    V0_ldr   = analogRead(ldrSensor)*(5.0 / 1023.0); // Calcula tensión en la resistencia Rt a traves de la entrada A0
    R_ldr   = (V0_ldr*R1_ldr)/(Vi_ldr-V0_ldr);       // Determina valor de Rt
    
    if(R_ldr <= 300){
        IntensidadLuminosa = 1000;
        Serial.print("Valor resistivo LDR: ");
        Serial.println(R_ldr);
        Serial.println("1000 lux");
    }
    else if(R_ldr > 300 && R_ldr < 1500){
        IntensidadLuminosa = 50;
        Serial.print("Valor resistivo LDR: ");
        Serial.println(R_ldr);
        Serial.println("50 lux");
    }
    else if(R_ldr >= 1500 && R_ldr < 10000){
        IntensidadLuminosa = 100;
        Serial.print("Valor resistivo LDR: ");
        Serial.println(R_ldr);
        Serial.println("100 lux");
    }    
    else if(R_ldr >= 10000 && R_ldr < 70000){
        IntensidadLuminosa = 10;
        Serial.print("Valor resistivo LDR: ");
        Serial.println(R_ldr);
        Serial.println("10 lux");
    }
    else if(R_ldr >= 70000 && R_ldr < 600000){
        IntensidadLuminosa = 1;
        Serial.print("Valor resistivo LDR: ");
        Serial.println(R_ldr);
        Serial.println("1 lux");
    }
    else if(R_ldr >= 600000){
        IntensidadLuminosa = 0.1;
        Serial.print("Valor resistivo LDR: ");
        Serial.println(R_ldr);
        Serial.println("0.1 lux");
    }
}

// Calcula velocidad del viento
float windSpeed(float jlfsx2VoltageRead){
    // Calculate wind speed
    // 1 V corresponde a 6 m/s
    jlfsx2VoltageRead = analogRead(A3);
    jlfsx2Voltage     = jlfsx2VoltageRead * (5.0 / 1023.0);
    windspeedread     = 6 * jlfsx2Voltage;                  
    // Print data
    Serial.print("Velocidad del viento: ");
    Serial.print(windspeedread);
    Serial.print(" ");
    Serial.print("m/s");
    Serial.println();
}

// Determina si esta lloviendo
void rainSensor(){
    RG11_read = digitalRead(RG11_Pin);
    
    if(RG11_read==LOW){
        char rg11_sirain[10] = "Llueve   ";
        strcpy(rg11_rain, rg11_sirain);
        //rg11_rain[10] = "SinLluvia";
        Serial.println("Llueve");        
    }
    else{
        char rg11_norain[10] = "No Llueve";
        strcpy(rg11_rain, rg11_norain); 
        //rg11_rain[10] = "ConLluvia";
        Serial.println("No llueve");        
    }
} 

// Despliega datos en pantalla
void displaylcd(){
   switchDisplay_read = digitalRead(switchDisplay);
 
    lcd.clear();

    if(switchDisplay_read==HIGH){
    lcd.setPower(1);  
    // Imprime Temperatura
    lcd.setCursor(0, 0);
    lcd.print("Temp ");
    lcd.print(temperatura, 1);
    lcd.print("\001C");    
    // Imprime Humedad
    lcd.setCursor(0, 1);
    lcd.print("hum ");
    lcd.print(humedad);
    lcd.print("%");
    // Imprime Luminosidad
    lcd.setCursor(0, 2);
    lcd.print("Lum ");
    lcd.print(IntensidadLuminosa);
    lcd.print(" lux");
    // Imprime Velocidad Viento
    lcd.setCursor(0, 3);
    lcd.print("W S ");
    lcd.print(windspeedread);
    lcd.print(" m/s");
    // Imprime Lluvia
    lcd.setCursor(0, 4);
    lcd.print(rg11_rain);
    }
    else{
      lcd.setPower(0);
    }        
    delay(3000);
}    

// interrupción por igualdade de comparación en TIMER1   
ISR(TIMER1_COMPA_vect){   
  if(counterTimer == 71){ // interrupción en 5 minutos
    if(eeAddress<EEPROM.length()){ // Si la memoria EEPROM no esta llena  
        
      MyObject customVar = {
        temperatura,
        humedad,
        IntensidadLuminosa,
        windspeedread,   
        rg11_rain[10],
      };
     
      EEPROM.put(eeAddress, customVar); // Escribe los valores a almacenar en la memoria EEPROM  
      eeAddress += sizeof(MyObject);  
    }
    else{
      clearMemory();
      eeAddress = 0;
      }
    counterTimer++;        
    }    
  else if(counterTimer == 142){ // interrupción en 10 minutos
    counterTimer = 0;
    }    
  else{ 
    counterTimer++;
    }  
} // Fin ISR

/////////////////////////////////////////////////////////////////////////////////
void clearMemory(){
  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
} 
