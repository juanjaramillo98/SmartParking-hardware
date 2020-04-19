#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <lmic.h>
#include <hal/hal.h>

//**************************************
//*********** WIFICONFIG ***************
//**************************************
const char *ssid = "juanpablo";
const char *password = "70577893";

//**************************************
//*********** MQTT CONFIG **************
//**************************************
const char *mqtt_server = "ioticos.org";
const int mqtt_port = 1883;
const char *mqtt_user = "zoAdbG2cjLqF3bm";
const char *mqtt_pass = "Zy0DXgPoIrDEYZr";
const char *root_topic_subscribe = "JubZ2d6exOOS2CV/sub/parqueadero18_1";
const char *root_topic_publish = "JubZ2d6exOOS2CV/pub/";

//**************************************
//*********** GLOBALES   ***************
//**************************************
WiFiClient espClient;
PubSubClient client(espClient);
char msg[50];

//************************
//** F U N C I O N E S ***
//************************
void callback(char *topic, byte *payload, unsigned int length);
void reconnect();
void setup_wifi();

static uint8_t mydata[3];
const int Trigger = 2; //Pin digital 2 para el Trigger del sensor
const int Echo = 13;   //Pin digital 3 para el Echo del sensor
const int BATTERY_PIN = 35;
const int LED_OCUPADO = 14;
const int LED_LIBRE = 25;
bool cambio = false;
bool estado = false; // true ocupado, false libre
const int SENSIBILIDAD = 100;
int distancia;
ostime_t ultimoCambio = os_getTime();

float vBat;
void setup()
{
	Serial.begin(115200);
	setup_wifi();
	client.setServer(mqtt_server, mqtt_port);
	client.setCallback(callback);
	adcStart(BATTERY_PIN);
	pinMode(Trigger, OUTPUT); //pin como salida
	pinMode(Echo, INPUT);  //pin como entrada
	pinMode(LED_LIBRE, OUTPUT);
	pinMode(LED_OCUPADO, OUTPUT);
	digitalWrite(Trigger, LOW);//Inicializamos el pin con 0
}


long calcularDistancia()
{
	long t; //timepo que demora en llegar el eco
	long d; //distancia en centimetros

	digitalWrite(Trigger, HIGH);
	delayMicroseconds(10); //Enviamos un pulso de 10us
	digitalWrite(Trigger, LOW);

	t = pulseIn(Echo, HIGH, 140000); //obtenemos el ancho del pulso
	d = t / 59;						 //escalamos el tiempo a una distancia en cm
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

float getBatteryVoltage()
{
	// we've set 10-bit ADC resolution 2^10=1024 and voltage divider makes it half of maximum readable value (which is 3.3V)
	vBat = analogRead(BATTERY_PIN) * 2.0 * (3.3 / 4096.0);
	return vBat;
}

void loop()
{

	if (!client.connected())
	{
		reconnect();
	}

	//la variable str se contruye de un valo imitando 1 o 2, 1 para cambio de estado y 2 para keep alive,id del dispositivo","el estado 0 para ocupado y 1 para libre" , bateria "3.7"
	//no borrar la coma después de el id y el estado
	//un ejemplo de keep alive incluyendo la batería en este:
	//String str = String("2,")+String("parqueadero18_1,")+String("3.7");
	if (client.connected())
	{
		//*******************************************
		distancia = calcularDistancia();
		//Serial.println(distancia);
		//Serial.print(calcularDistancia());
		if (estado && (ultimoCambio + sec2osticks(10) < os_getTime()) && ((distancia > SENSIBILIDAD) || (distancia = 0)))
		{
			ultimoCambio = os_getTime();
			Serial.println("Se desocupo");
			String str = String("1,") + String("parqueadero18_1,") + String("1,") + String(getBatteryVoltage());
			str.toCharArray(msg, 50);
			estado = false;
			cambio = true;
			digitalWrite(LED_LIBRE, HIGH);
			digitalWrite(LED_OCUPADO, LOW);
		}

		if (!estado && (ultimoCambio + sec2osticks(10) < os_getTime()) && ((distancia < SENSIBILIDAD) && (distancia > 0)))
		{
			ultimoCambio = os_getTime();
			Serial.println("ocurrio un cambio positivo");
			String str = String("1,") + String("parqueadero18_1,") + String("0,") + String(getBatteryVoltage());
			str.toCharArray(msg, 50);
			estado = true;
			cambio = true;
			digitalWrite(LED_LIBRE, LOW);
			digitalWrite(LED_OCUPADO, HIGH);
		}

		if (cambio)
		{
			//do_send(&sendjob);
			client.publish(root_topic_publish, msg);
			Serial.println("intento do send");
			cambio = false;
		}
		//*******************************************
	}
	client.loop();
}

//*****************************
//***    CONEXION WIFI      ***
//*****************************
void setup_wifi()
{
	delay(10);
	// Nos conectamos a nuestra red Wifi
	Serial.println();
	Serial.print("Conectando a ssid: ");
	Serial.println(ssid);

	WiFi.begin(ssid, password);

	while (WiFi.status() != WL_CONNECTED)
	{
		delay(500);
		Serial.print(".");
	}
    digitalWrite(LED_LIBRE, HIGH);
	Serial.println("");
	Serial.println("Conectado a red WiFi!");
	Serial.println("Dirección IP: ");
	Serial.println(WiFi.localIP());
}

//*****************************
//***    CONEXION MQTT      ***
//*****************************

void reconnect()
{

	while (!client.connected())
	{
		Serial.print("Intentando conexión Mqtt...");
		// Creamos un cliente ID
		String clientId = "IOTICOS_H_W_";
		clientId += String(random(0xffff), HEX);
		// Intentamos conectar
		if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass))
		{
			Serial.println("Conectado!");
			// Nos suscribimos
			if (client.subscribe(root_topic_subscribe))
			{
				Serial.println("Suscripcion ok");
			}
			else
			{
				Serial.println("fallo Suscripciión");
			}
		}
		else
		{
			Serial.print("falló :( con error -> ");
			Serial.print(client.state());
			Serial.println(" Intentamos de nuevo en 5 segundos");
			delay(5000);
		}
	}
}

//*****************************
//***       CALLBACK        ***
//*****************************

void callback(char *topic, byte *payload, unsigned int length)
{
	String incoming = "";
	Serial.print("Mensaje recibido desde -> ");
	Serial.print(topic);
	Serial.println("");
	for (int i = 0; i < length; i++)
	{
		incoming += (char)payload[i];
	}
	incoming.trim();
	Serial.println("Mensaje -> " + incoming);
}