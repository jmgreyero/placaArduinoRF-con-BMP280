
// **********************************************************************************
// 
// Test RFM69 Radio con 280.
//          
//  version 1.0 20190423
//  separando datos con <,> y con <.> para decimales
//  con Ack
//                                             
// **********************************************************************************

#include <RFM69registers.h>
#include <RFM69.h>              // https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>          // https://www.github.com/lowpowerlab/rfm69
#include <SPIFlash.h>                // Included with Arduino IDE
#include <stdlib.h>

// del ejemplo
#include <Wire.h>             //included in Arduino IDE (www.arduino.cc)
#include <SparkFunBME280.h>   //get it here: https://github.com/sparkfun/SparkFun_BME280_Arduino_Library/tree/master/src

// mensaje que envia el Pi preguntando a local_id
// destino=local_id,datos=msg_280 ("rdNode280")

// si el local_id no coincide, ya no lo lee
// si no coincide el mensaje, lo lee pero da error

// mensaje a devolver
// destino=master_id,datos="num.mensaje,NODE_TYPE,LOCAL_ID,datosTHP"

// tipos de modulo:
#define NODE_TYPE_THP280  1
  // datos: T H P
  
// Node and network config
#define MASTER_ID    1    // el que pregunta -> el del Pi
#define LOCAL_ID     2    // The ID of this node (must be different for every node on network)
#define NETWORK_ID   100  // The network ID

// Are you using the RFM69 Wing? Uncomment if you are.
//#define USING_RFM69_WING 

// The transmision frequency of the baord. Change as needed.
#define FREQUENCY      RF69_433MHZ //RF69_868MHZ // RF69_915MHZ

// Uncomment if this board is the RFM69HW/HCW not the RFM69W/CW
#define IS_RFM69HW_HCW

// Serial board rate - just used to print debug messages
#define SERIAL_BAUD   9600

// Board and radio specific config - You should not need to edit
#if defined (__AVR_ATmega32U4__) && defined (USING_RFM69_WING)
    #define RF69_SPI_CS  10   
    #define RF69_RESET   11   
    #define RF69_IRQ_PIN 2 
    #define RF69_IRQ_NUM digitalPinToInterrupt(RF69_IRQ_PIN) 
#elif defined (__AVR_ATmega32U4__)
    #define RF69_RESET    4
    #define RF69_SPI_CS   8
    #define RF69_IRQ_PIN  7
    #define RF69_IRQ_NUM  4
#elif defined(ARDUINO_SAMD_FEATHER_M0) && defined (USING_RFM69_WING)
    #define RF69_RESET    11
    #define RF69_SPI_CS   10
    #define RF69_IRQ_PIN  6
    #define RF69_IRQ_NUM  digitalPinToInterrupt(RF69_IRQ_PIN)
#elif defined(ARDUINO_SAMD_FEATHER_M0)
// esta placa es la que reconoce como Adafruit Feather M0
    #define RF69_RESET    4
    #define RF69_SPI_CS   8
    #define RF69_IRQ_PIN  3
    #define RF69_IRQ_NUM  3
    #define LED           13
#endif

RFM69 radio(RF69_SPI_CS, RF69_IRQ_PIN, false, RF69_IRQ_NUM);

BME280 bme280;

int8_t salida_display_THP=0;
int8_t salida_display_recepcion=1;
int8_t salida_display_envio=1;

// Setup

void setup() {
  Serial.begin(9600); //115200);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW );      
  
  Serial.println("prueba del node-280");
  Serial.println();
  
  // Reset the radio
  resetRadio();
  // Initialize the radio
  radio.initialize(FREQUENCY, LOCAL_ID, NETWORK_ID);
  radio.promiscuous(true);
  #ifdef IS_RFM69HW_HCW
    radio.setHighPower(); //must include this only for RFM69HW/HCW!
  #endif

  // modulo 280
  
  Wire.begin();
  if (bme280.beginI2C() == false) //Begin communication over I2C
  {
    Serial.println("The sensor did not respond. Please check wiring.");
    while(1); //Freeze
  }
}

// Main loop

float T;
float P;
float H;
char Pstr[10];
char Tstr[10];
char Hstr[10];

char buffer_tx[70];
char msg_280[] = "rdNode280";

int16_t  contador=0;
int8_t  mensajes_en_cola=0;

void loop() {
    // Receive
  if (radio.receiveDone()) {
    digitalWrite(LED, HIGH);      

    // leer BME sensor
    T=bme280.readTempC();
    dtostrf(T, 4, 1, '.',Tstr);  // usar <.>
    H=bme280.readFloatHumidity();
    dtostrf(H, 3, 1, '.',Hstr);
    P=bme280.readFloatPressure();
    dtostrf(P/100.0, 5, 1, '.',Pstr);
    if(salida_display_THP){
      Serial.print("T:");
      Serial.print(T,2);
      Serial.print(" H:");
      Serial.print(H,0);
      Serial.print(" P:");
      Serial.println(P,0);
      Serial.println();
    }

    sprintf(buffer_tx, "%i,%i,%s,%s,%s",NODE_TYPE_THP280,LOCAL_ID, Tstr, Hstr, Pstr);
    if(salida_display_THP)
      Serial.println(buffer_tx);

    if(salida_display_recepcion){
      Serial.println("\nRecibido:");
      Serial.print("SENDERID=");
        Serial.print(radio.SENDERID,DEC);
      Serial.print(" TARGETID=");
        Serial.print(radio.TARGETID,DEC);
      Serial.print(" data=\"");
        Serial.print((char*)radio.DATA);
      Serial.print("\" [RX_RSSI=");
        Serial.print(radio.readRSSI());
      Serial.println("]");
    }

    if( (radio.SENDERID==MASTER_ID) & 
          (radio.TARGETID==LOCAL_ID) &
          (strcmp((char*)radio.DATA,msg_280)==0)) {
      contador++;
      mensajes_en_cola++;
    }
    else {
      mensajes_en_cola=0;
      if(salida_display_recepcion)  Serial.println("Mensaje para otro");
    }
 
    const void* bf = ""; //OJO!! da error si no se hace esto
    if (radio.ACKRequested()) { radio.sendACK(bf,radio.SENDERID); }
    delay(100);
    digitalWrite(LED, LOW );      
  }

    // Send     
  if (mensajes_en_cola){
    char mensajeTx[RF69_MAX_DATA_LEN];
    sprintf(mensajeTx,"%i,%s", contador,buffer_tx);
    while(strlen(mensajeTx)<RF69_MAX_DATA_LEN)  strcat(mensajeTx," ");
    if (salida_display_envio){
      Serial.print("enviando:");
      Serial.println(mensajeTx);
    }
    radio.sendWithRetry(MASTER_ID, mensajeTx, sizeof(mensajeTx), 3, 200);
 //   radio.send(MASTER_ID, mensajeTx, sizeof(mensajeTx), false);
          mensajes_en_cola--;
    }
}

// Reset the Radio
void resetRadio() {
  if (Serial) Serial.print("Resetting radio...");
  pinMode(RF69_RESET, OUTPUT);
  digitalWrite(RF69_RESET, HIGH);
  delay(20);
  digitalWrite(RF69_RESET, LOW);
  delay(500);
}

// preparado para usar ',' en lugar de '.' como separador
// olver a usar <.>
char *dtostrf(double value, int width, unsigned int precision, char separador,char *result){
  int decpt, sign, reqd, pad;
  const char *s, *e;
  char *p;
  s = fcvt(value, precision, &decpt, &sign);
  if (precision == 0 && decpt == 0) {
  s = (*s < '5') ? "0" : "1";
    reqd = 1;
  } else {
    reqd = strlen(s);
    if (reqd > decpt) reqd++;
    if (decpt == 0) reqd++;
  }
  if (sign) reqd++;
  p = result;
  e = p + reqd;
  pad = width - reqd;
  if (pad > 0) {
    e += pad;
    while (pad-- > 0) *p++ = ' ';
  }
  if (sign) *p++ = '-';
  if (decpt <= 0 && precision > 0) {
    *p++ = '0';
    *p++ = separador; // '.';
    e++;
    while ( decpt < 0 ) {
      decpt++;
      *p++ = '0';
    }
  }   
  while (p < e) {
    *p++ = *s++;
    if (p == e) break;
    if (--decpt == 0) *p++ = separador; //'.';
  }
  if (width < 0) {
    pad = (reqd + width) * -1;
    while (pad-- > 0) *p++ = ' ';
  }
  *p = 0;
  return result;
}

//****************************
