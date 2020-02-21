#include <Arduino.h>
#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>
#include <WiFi.h>
#include <Wire.h>

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.

// static const u1_t PROGMEM DEVEUI[8]={ 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
// void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// // Copy the value from Application EUI from the TTN console in LSB mode
// static const u1_t PROGMEM APPEUI[8]= { 0x0B, 0x74, 0x02, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
// void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// // This key should be in big endian format (or, since it is not really a
// // number but a block of memory, endianness does not really apply). In
// // practice, a key taken from ttnctl can be copied as-is. Anyway its in MSB mode.
// static const u1_t PROGMEM APPKEY[16] ={ 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
// void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}
// // // These callbacks are only used in over-the-air activation, so they are
// // // left empty here (we cannot leave them out completely unless
// // // DISABLE_JOIN is set in config.h, otherwise the linker will complain).

static PROGMEM u1_t NWKSKEY[16] = { 0x98, 0x52, 0xA5, 0x7A, 0x8E, 0xD7, 0x0B, 0x22, 0xE1, 0x94, 0xAF, 0x3E, 0xCB, 0x70, 0x99, 0x5E };// LoRaWAN NwkSKey, network session key 
static u1_t PROGMEM APPSKEY[16] = { 0xF2, 0x7B, 0xB7, 0xC2, 0x17, 0xA7, 0x79, 0x9E, 0x0A, 0x6B, 0x12, 0x74, 0x6F, 0xC4, 0x0B, 0x91 }; // LoRaWAN AppSKey, application session key 
static const u4_t DEVADDR = 0x26021B38 ; // LoRaWAN end-device address (DevAddr)
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


static uint8_t mydata[3];
static osjob_t sendjob;
static osjob_t KeepAlive;
void do_send(osjob_t* j); // declaration of send function

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 20;
const unsigned KA_INTERVAL = 50;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 23,
  .dio = {26, 33, 32},  // PIN 33 HAS TO BE PHYSICALLY CONNECTED TO PIN Lora1 OF TTGO
};  


const int Trigger = 2;   //Pin digital 2 para el Trigger del sensor
const int Echo = 13;   //Pin digital 3 para el Echo del sensor
const int BATTERY_PIN = 35;
const int LED_OCUPADO = 14;
const int LED_LIBRE = 25;
bool cambio = false;
bool estado = false; // true ocupado, false libre
const int SENSIBILIDAD = 50;
int distancia;
ostime_t ultimoCambio = os_getTime();


float vBat;

long calcularDistancia()
{
  long t; //timepo que demora en llegar el eco
  long d; //distancia en centimetros

  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);          //Enviamos un pulso de 10us
  digitalWrite(Trigger, LOW);
  
  t = pulseIn(Echo, HIGH,140000); //obtenemos el ancho del pulso
  d = t/59;             //escalamos el tiempo a una distancia en cm
  return d;
}
void imprimirBuffer()
{
  Serial.print("Estado: ");
  Serial.println(mydata[0]);
  Serial.print("Distancia: ");
  Serial.println(mydata[1]);
  Serial.print("Bateria: ");
  Serial.println(mydata[2]);
}

float getBatteryVoltage() {
  // we've set 10-bit ADC resolution 2^10=1024 and voltage divider makes it half of maximum readable value (which is 3.3V)
  vBat = analogRead(BATTERY_PIN) * 2.0 * (3.3 / 4096.0);
  return vBat;  
}



void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            LMIC_setLinkCheckMode(0);

            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}
void keep_alive(osjob_t* j){
  mydata[0] = 1;

  LMIC_setTxData2(2, mydata, 1, 0);
  Serial.println(F("keep alive queued"));
  os_setTimedCallback(&KeepAlive, os_getTime()+sec2osticks(KA_INTERVAL), keep_alive);
}
void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        if (!estado)
        {
          mydata[0] = 1;
        }else
        {
          mydata[0] = 0;
        }
        mydata[1] = calcularDistancia();
        mydata[2] = getBatteryVoltage()*10;
        LMIC_setTxData2(1, mydata, 3, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


void setup() 
{
  Serial.begin(9600);
  Serial.println(F("Starting"));
  adcStart(BATTERY_PIN);
  pinMode(Trigger, OUTPUT); //pin como salida
  pinMode(Echo, INPUT);  //pin como entrada
  pinMode(LED_LIBRE, OUTPUT);
  pinMode(LED_OCUPADO, OUTPUT);
  digitalWrite(Trigger, LOW);//Inicializamos el pin con 0

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
   // Reset the MAC state. Session and pending data transfers will be discarded.
  //LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  #if defined(CFG_eu868)
  #elif defined(CFG_us915)
  // NA-US channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);
  #endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF9,14);

  // Start job
  // do_send(&sendjob);
  keep_alive(&KeepAlive);
}



void loop()
{
  os_runloop_once();
  distancia = calcularDistancia();
  delay(100);
  //Serial.print(calcularDistancia());
  if (estado && (ultimoCambio + sec2osticks(10) < os_getTime()) && ((distancia > SENSIBILIDAD)||( distancia = 0 )))
  {
    ultimoCambio = os_getTime();
    Serial.print("-----------");
    Serial.println(distancia);
    estado = false;
    cambio = true;
    digitalWrite(LED_LIBRE,HIGH);
    digitalWrite(LED_OCUPADO,LOW);

  }
  
  if (!estado && (ultimoCambio + sec2osticks(10) < os_getTime()) && ((distancia < SENSIBILIDAD)&&(distancia > 0)))
  {
    ultimoCambio = os_getTime();
    Serial.print("++++++++++");
    Serial.println(distancia);
    estado = true;
    cambio = true;
    digitalWrite(LED_LIBRE,LOW);
    digitalWrite(LED_OCUPADO,HIGH);    
  }

  if (cambio)
  {
    do_send(&sendjob);
    cambio = false;
  }
  
}

