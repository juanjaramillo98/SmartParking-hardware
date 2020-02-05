#include <Arduino.h>
#include <Arduino.h>
#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>
#include <WiFi.h>
#include <CayenneLPP.h>
#include <Wire.h>
#include <Adafruit_BME280.h>

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static PROGMEM u1_t NWKSKEY[16] = { 0x66, 0x52, 0x89, 0x28, 0xA5, 0x99, 0x80, 0xEF, 0xF0, 0xF8, 0xEC, 0x0E, 0x3E, 0x25, 0x96, 0x2D }; // LoRaWAN NwkSKey, network session key 
static u1_t PROGMEM APPSKEY[16] = { 0xBE, 0xEA, 0x9A, 0x4F, 0xD5, 0xC8, 0x58, 0xC9, 0xFF, 0x91, 0xF1, 0x4E, 0xAF, 0x98, 0xD3, 0x67 }; // LoRaWAN AppSKey, application session key 
static const u4_t DEVADDR = 0x26021383; 

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[4];
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
const int CONTROL_PIN = 14;

float vBat;

long calcularDistancia()
{
  long t; //timepo que demora en llegar el eco
  long d; //distancia en centimetros

  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);          //Enviamos un pulso de 10us
  digitalWrite(Trigger, LOW);
  
  t = pulseIn(Echo, HIGH); //obtenemos el ancho del pulso
  d = t/59;             //escalamos el tiempo a una distancia en cm
  return d;
}
void imprimirBuffer()
{
  Serial.print("Estado: ");
  Serial.println(mydata[0]);
  Serial.print("Distancia: ");
  Serial.println(mydata[1]);
  Serial.print("Control: ");
  Serial.println(mydata[2]);
  Serial.print("Bateria: ");
  Serial.println(mydata[3]);
}

float getBatteryVoltage() {
  // we've set 10-bit ADC resolution 2^10=1024 and voltage divider makes it half of maximum readable value (which is 3.3V)
  vBat = analogRead(BATTERY_PIN) * 2.0 * (3.3 / 1024.0);
  return vBat;  
}

int getAnalogControl()
{
  return (analogRead(CONTROL_PIN)/40.95);
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
  mydata[1] = 1;
  mydata[2] = 1;
  mydata[3] = 1;
  LMIC_setTxData2(1, mydata, 4, 0);
  Serial.println(F("keep alive queued"));
  os_setTimedCallback(&KeepAlive, os_getTime()+sec2osticks(KA_INTERVAL), keep_alive);
}
void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        if (calcularDistancia() < getAnalogControl())
        {
          mydata[0] = 1;
        }else
        {
          mydata[0] = 0;
        }
        mydata[1] = calcularDistancia();
        mydata[2] = getAnalogControl();
        mydata[3] = getBatteryVoltage();
        LMIC_setTxData2(1, mydata, 4, 0);
        imprimirBuffer();
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
  digitalWrite(Trigger, LOW);//Inicializamos el pin con 0

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  #ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
  #else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
  #endif

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
  LMIC_setDrTxpow(DR_SF7,14);

  // Start job
  // do_send(&sendjob);
  keep_alive(&KeepAlive);
}

bool cambio = false;
bool estado = false; // true ocupado, false libre
ostime_t ultimoCambio = os_getTime();

void loop()
{
  os_runloop_once();
  //Serial.print(calcularDistancia());
  if (estado && (ultimoCambio + sec2osticks(10) < os_getTime()) && (calcularDistancia() > 100))
  {
    ultimoCambio = os_getTime();
    Serial.println("ocurrio un cambio negativo");
    estado = false;
    cambio = true;
  }
  
  if (!estado && (ultimoCambio + sec2osticks(10) < os_getTime()) && (calcularDistancia() < 100))
  {
    ultimoCambio = os_getTime();
    Serial.println("ocurrio un cambio positivo");
    estado = true;
    cambio = true;
  }

  if (cambio)
  {
    do_send(&sendjob);
    Serial.println("intento do send");
    cambio = false;
  }
  
}

