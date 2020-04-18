#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

//**************************************
//*********** WIFICONFIG ***************
//**************************************
const char* ssid = "ARRIS-2602";
const char* password =  "FE63B4366C140438";



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
long count=0;


//************************
//** F U N C I O N E S ***
//************************
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();
void setup_wifi();

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
}

void loop() {
  
  if (!client.connected()) {
		reconnect();
	}

	//la variable str se contruye de un valo imitando 1 o 2, 1 para cambio de estado y 2 para keep alive,id del dispositivo","el estado 0 para ocupado y 1 para libre" , bateria "3.7"
	//no borrar la coma después de el id y el estado
	//un ejemplo de keep alive incluyendo la batería en este:
	//String str = String("2,")+String("parqueadero18_1,")+String("3.7");
  if (client.connected()){
    String str = String("1,")+String("parqueadero18_1,")+String("0,")+String("3.7");
	//String str = String("2,")+String("parqueadero18_1,")+String("3.7");
    str.toCharArray(msg,50);
    client.publish(root_topic_publish,msg);
    count++;

    delay(5000);

	
  }
  client.loop();
}




//*****************************
//***    CONEXION WIFI      ***
//*****************************
void setup_wifi(){
	delay(10);
	// Nos conectamos a nuestra red Wifi
	Serial.println();
	Serial.print("Conectando a ssid: ");
	Serial.println(ssid);

	WiFi.begin(ssid, password);

	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}

	Serial.println("");
	Serial.println("Conectado a red WiFi!");
	Serial.println("Dirección IP: ");
	Serial.println(WiFi.localIP());
}



//*****************************
//***    CONEXION MQTT      ***
//*****************************

void reconnect() {

	while (!client.connected()) {
		Serial.print("Intentando conexión Mqtt...");
		// Creamos un cliente ID
		String clientId = "IOTICOS_H_W_";
		clientId += String(random(0xffff), HEX);
		// Intentamos conectar
		if (client.connect(clientId.c_str(),mqtt_user,mqtt_pass)) {
			Serial.println("Conectado!");
			// Nos suscribimos
			if(client.subscribe(root_topic_subscribe)){
        Serial.println("Suscripcion ok");
      }else{
        Serial.println("fallo Suscripciión");
      }
		} else {
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


void callback(char* topic, byte* payload, unsigned int length){
	String incoming = "";
	Serial.print("Mensaje recibido desde -> ");
	Serial.print(topic);
	Serial.println("");
	for (int i = 0; i < length; i++) {
		incoming += (char)payload[i];
	}
	incoming.trim();
	Serial.println("Mensaje -> " + incoming);

}