#include <ESP8266WiFi.h>
#include <ESP8266httpUpdate.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

//Hemos quitado el comando ADCMODE puesto que queremos usar la entrada analógica para leer el valor del sensor, no la tension de entrada

// datos para actualización   >>>> SUSTITUIR IP <<<<<
#define OTA_URL "https://iot.ac.uma.es:1880/esp8266-ota/update"// Address of OTA update server
#define HTTP_OTA_VERSION   String(__FILE__).substring(String(__FILE__).lastIndexOf('\\')+1)+".nodemcu"

// funciones para progreso de OTA
void progreso_OTA(int,int);
void final_OTA();
void inicio_OTA();
void error_OTA(int);
int LED_OTA = 16; 

// Configuración del WiFi y de MQTT
//const char* ssid = "DIGIFIBRA-HuNF";                                             
//const char* password = "eyTxC9AA3d";
const char* ssid = "infind";
const char* password = "1518wifi";
//const char* ssid = "MiFibra-FF10";
//const char* password = "4mWexwaV";

const char* mqtt_server = "iot.ac.uma.es";                                          //Servidor del departamento
const char* mqtt_user = "II11";                                                     //Usuario del grupo
const char* mqtt_pass = "jzWeZUIo";                                                 //Contraseña del grupo
const char* chip_id = "ESP0010C48E";                                                //Identificador único de la placa

// Topics usados para enviar y recibir mensajes por mqtt
const char* topic_conexion = "II11/ESP0010C48E/conexion";                           //Topic para los mensajes de conexión
const char* topic_datos_VMA407N = "II11/ESP0010C48E/datos_VMA407N";                 //Topic para datos de configuración y relacionados con la lectura del sensor VMA407N
const char* topic_FOTA = "II11/ESP0010C48E/FOTA";                                   //Topic para buscar actualización 

const char* topic_RGB_cmd = "II11/ESP0010C48E/RGB_cmd";                             //Topic para enviar comandos relacionados con el led RGB
const char* topic_modo_cmd = "II11/ESP0010C48E/modo_cmd";                           //Topic para enviar comandos sobre el modo de funcionamiento
const char* topic_switch_cmd = "II11/ESP0010C48E/switch_cmd";                       //Topic para enviar mensajes sobre el switch
const char* topic_config_cmd = "II11/ESP0010C48E/configuracion_cmd";                //Topic para enviar comandos sobre la configuración

const char* topic_RGB_status = "II11/ESP0010C48E/RGB_status";                       //Topic para conocer el estado del led RGB
const char* topic_modo_status = "II11/ESP0010C48E/modo_status";                     //Topic para concoer el modo de funcionamiento actual
const char* topic_switch_status = "II11/ESP0010C48E/switch_status";                 //Topic para conocer el estado actual del switch

unsigned long lastMsg = 0;                                                          //Variable para saber cuando se envio el ultimo mensaje
unsigned long lastAct = 0;                                                          //Variable para saber cuando se realizo la ultima actualizacion
int ledRojo = 12;       // D6                                                       //Pin asociado al led rojo
int ledVerde = 13;      // D7                                                       //Pin asociado al led verde
int ledAzul = 14;       // D5                                                       //Pin asociado al led azul
int ledInt = 5;         // D1                                                       //Pin asociado al switch
const int analogPin = A0;                                                           //Pin analógico para la lectura del sensor


//Inicializando diferentes variables                                       
int nivel_rojo = 0;                                                                 //Nivel de rojo en PWM
int nivel_verde = 0;                                                                //Nivel de verde en PWM
int nivel_azul = 0;                                                                 //Nivel de azul en PWM
int switch_inv = 1;                                                                 //Valor del interruptor(TIENE LÓGICA NEGATIVA), inicializado a OFF
int switch_prev = 0;                                                                //Valor previo del interruptor
int sensorValue = 0;                                                                //Valor del sensor

int nivel_deriva = 0;                                                               //Porcentaje de cambio en el PWM usando el modo deriva
int operador = 0;                                                                   //Variable para saber si la operacion deriva es ascendente o descendente

//Variables para saber que modo de funcionamiento está puesto y cuál ha sido el previo para poder realizar un control de prioridad
int modo_party = 0;                                                          
int modo_auto = 0;
int modo_inverso = 0;
int prev_modo_auto = 0;
int prev_modo_party = 0;
int prev_modo_inverso = 0;

char* origen = "";                                                                  //Cadena para controlar si el interruptor se apaga desde MQTT o la placa                                                      

//Variables para el pulsador mediante interrupciones
int boton_flash=0;                                                                  //GPIO0 = boton flash
int estado_int=HIGH;                                                                //Por defecto HIGH (PULLUP). Cuando se pulsa se pone a LOW
unsigned long ultima_int = 0;                                                       //Variable para saber cuando se realizo la ultima interrupcion                                                 
unsigned long puls_previa=0;                                                        //Variable para saber cuando se realizo la pulsacion previa

//Variables para identificar el tipo de pulsacion
int doble = 0;       
int simple = 0;
int prolongada = 0;

// Parametros de la configuración de la placa
int t_enviar = 5000;                                                                //Periodo de envío de datos
int t_actualizar = 300000;                                                          //Periodo de búsqueda de actualizaciones
int luz_crit = 400;                                                                 //Variable para identificar si la luz actual es crítica(es menor que esta vble)
int luz_baja = 800;                                                                 //Variable para identificar si la luz actual es baja
int NUEVA_CONFIG = 0;                                                               //Si se activa, se actualizarán todos los parámetros de configuración
int BUSC_FOTA = 0;                                                                  //Si se activa, se busca una act                                                           
    
// Para guardar los mensajes json
char msg_datos_VMA407N[256];                                                        //Hemos usado arduinojson para ver cuanta memoria necesitamos
char msg_modo_status[96]; 
char msg_RGB_status[192];
char msg_switch_status[96];

WiFiClient espClient;
PubSubClient client(espClient);
#define MSG_BUFFER_SIZE (50)

char msg[MSG_BUFFER_SIZE];


//-------------------------------------------------------------------------//
//-                                 RTI                                   -//
//-------------------------------------------------------------------------//
// Rutina de Tratamiento de la Interrupcion (RTI)                                  //Se ha tomado como referencia el archivo del campus sobre muestreo vs polling
ICACHE_RAM_ATTR void RTI() {
  int lectura=digitalRead(boton_flash);
  unsigned long ahora= millis();
  
  if(lectura==estado_int || ahora-ultima_int<50) return; // filtro rebotes 50ms
  if(lectura==LOW)
  { 
   estado_int=LOW;
   //Serial.print("Int en: ");
   //Serial.println(ahora);
   if((ahora - puls_previa < 300)&&(ahora > 350)){                                //Nos evitamos que entre al principio al ejecutar el programa
    doble = 1;
    }
    puls_previa = ahora;                                                         //Actualizamos la pulsación previa
  }
  else
  {
   estado_int=HIGH;
   //Serial.print("Int dura: ");
   //Serial.println(ahora-ultima_int);
   if(ahora-ultima_int >= 2000){                                                 //Consideraremos pulsacion prolongada a partir de unos 2 segundos
    prolongada = 1;
  }
   if(ahora-ultima_int<350){                                                     //Consideraremos una pulsación simple menor de unos 350 ms
    simple = 1;
  }
  
  }
  ultima_int = ahora;                                                            //Actualizamos la ultima interrupcion
  
}

//-------------------------------------------------------------------------//
//-                            CONEXIÓN WIFI                              -//
//-------------------------------------------------------------------------//

void setup_wifi() {
  Serial.printf("\nConnecting to%s:\n", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {                                        //Mientras que no se haya conectado no sale del bucle
    delay(200);
  }

  Serial.printf("\nWiFi connected, IP address: %s\n", WiFi.localIP().toString().c_str());
}

//-------------------------------------------------------------------------//
//-                                 FOTA                                  -//
//-------------------------------------------------------------------------//

void intenta_OTA()                                                                             
{ 
  Serial.println( "--------------------------" );  
  Serial.println( "Comprobando actualización:" );
  Serial.println(OTA_URL);
  Serial.println( "--------------------------" );  
  ESPhttpUpdate.setLedPin(LED_OTA, LOW);
  ESPhttpUpdate.onStart(inicio_OTA);
  ESPhttpUpdate.onError(error_OTA);
  ESPhttpUpdate.onProgress(progreso_OTA);
  ESPhttpUpdate.onEnd(final_OTA);
  WiFiClientSecure wClient;
  // Reading data over SSL may be slow, use an adequate timeout
  wClient.setTimeout(12); // timeout argument is defined in seconds for setTimeout
  wClient.setInsecure();
  switch(ESPhttpUpdate.update(wClient, OTA_URL, HTTP_OTA_VERSION)) {
    case HTTP_UPDATE_FAILED:
      Serial.printf(" HTTP update failed: Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println(F(" El dispositivo ya está actualizado"));
      break;
    case HTTP_UPDATE_OK:
      Serial.println(F(" OK"));
      break;
    }
}

//-----------------------------------------------------
void final_OTA()
{
  Serial.println("Fin OTA. Reiniciando...");
}

void inicio_OTA()
{
  Serial.println("Nuevo Firmware encontrado. Actualizando...");
}

void error_OTA(int e)
{
  char cadena[64];
  snprintf(cadena,64,"ERROR: %d",e);
  Serial.println(cadena);
}

void progreso_OTA(int x, int todo)
{
  char cadena[256];
  int progress=(int)((x*100)/todo);
  if(progress%10==0)
  {
    snprintf(cadena,256,"Progreso: %d%% - %dK de %dK",progress,x/1024,todo/1024);
    Serial.println(cadena);
  }
}

//-------------------------------------------------------------------------//
//-                               CALLBACK                                -//
//-------------------------------------------------------------------------//

void callback(char* topic, byte* payload, unsigned int length){

  char* cadena = (char*)malloc(length+1);

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  strncpy(cadena,(char*)payload,length);
  cadena[length]='\0';

  StaticJsonDocument<128>json;
                                        
  DeserializationError error = deserializeJson(json,cadena,length);
   if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }
  //---------------------------------------------------   MODO_CMD
   else if (strcmp(topic,topic_modo_cmd) == 0){
                                                                                 //Recibimos que modo queremos poner
      modo_auto = json["modo_auto"];
      modo_party = json["modo_party"];   
      modo_inverso = json["modo_inverso"]; 
   
      prev_modo_auto = modo_auto;                                                //Guardamos los valores para compararlos y actuar en consecuencia
      prev_modo_party = modo_party;
      prev_modo_inverso = modo_inverso;
      
    if(switch_inv == 1){                                                         //Si el interruptor está apagado, apagamos todos los modos
      modo_auto = 0;
      modo_party = 0;
      modo_inverso = 0;
      }  
    
    if(modo_auto==1){                                                            //El modo auto tiene prioridad con respecto al resto por lo que se apagan
      modo_party=0;
      modo_inverso = 0;
      switch_inv = 0;                                                            //Aqui lo apagamos para que termine encendiendose debido al inversor de nodered. Para mayor comodidad muestra y pasa los 0 a 1 ya que el interruptor funciona con logica negativa, por tanto en este unico caso en el que queremos que se encienda sin comando, hay que ponerlo al reves
      } 
    else if(modo_party==1){                                                      //Tanto party como inverso tienen la misma prioridad por lo que si está uno, se apaga el otro
      modo_inverso = 0;
      } 
     else if(modo_inverso==1){
      modo_party = 0;
     }
    
    }
    
  //---------------------------------------------------   RGB_CMD
  
  else if (strcmp(topic,topic_RGB_cmd) == 0){                                     //Recibimos el color actual y el nivel de deriva
    nivel_rojo = json["rojo"];
    nivel_verde = json["verde"];
    nivel_azul= json["azul"];
    nivel_deriva = json["deriva"];
    
    if((modo_inverso == 1)&&(nivel_deriva == 0)){                                 //El modo inverso funciona de forma normal sacando el color contrario al elegido solo si deriva vale 0%, si no, funciona para pasar a negro o a blanco
      nivel_rojo = 255 - nivel_rojo;
      nivel_verde = 255 - nivel_verde;
      nivel_azul = 255 - nivel_azul;
       } 
       
     StaticJsonDocument<192> RGB_status;                                          //Enviamos el color final
                                                                                  //Durante el modo party o el modo automatico no enviamos el color ya que no tiene mucho interes
     RGB_status["ChipID"] = chip_id;
     RGB_status["Deriva"]= nivel_deriva;
                                                               
     JsonObject RGB_Colores = RGB_status.createNestedObject("RGB_Colores");
     RGB_Colores["Red"] = nivel_rojo;
     RGB_Colores["Green"] = nivel_verde;
     RGB_Colores["Blue"] = nivel_azul;
     
     
     serializeJson(RGB_status, msg_RGB_status);
     Serial.print(msg_RGB_status);
     client.publish(topic_RGB_status,msg_RGB_status);
     
    }
  //--------------------------------------------------   SWITCH_CMD
   else if(strcmp(topic,topic_switch_cmd) == 0){                                  //Recibimos el valor del interruptor
      switch_inv = json["switch"];
      origen = "MQTT";
       
      if(switch_inv==1){                                                          //Si está apagado apagamos todos los modos de funcionamiento
        modo_auto=0;
        modo_party=0;
        modo_inverso=0;
      }
   }
   //------------------------------------------------   CONFIG_CMD

   else if (strcmp(topic,topic_config_cmd) == 0){                                 //Recibimos los parametros de configuracion nuevos
      NUEVA_CONFIG = json["config"];
  
      if (NUEVA_CONFIG==1){                                                       //Solo si nueva configuracion vale 1 actualizamos todos
       t_enviar = json["t_enviar"];
       t_actualizar = json["t_actualizar"];
       luz_crit = json["val_critico"];
       luz_baja = json["val_bajo"];
       NUEVA_CONFIG = 0;
       }
  }
  //--------------------------------------------------    FOTA
  else if (strcmp(topic,topic_FOTA) == 0){                                         //Buscamos actualizacion
      BUSC_FOTA = json["fota"];
      Serial.print(BUSC_FOTA);
       if(BUSC_FOTA ==1){                                                          //Si vale 1 intentamos actualizar
        intenta_OTA(); 
        BUSC_FOTA = 0;
        }
              
  }
  //---------------------------------------------------
}

//-------------------------------------------------------------------------//
//-                            CONEXIÓN MQTT                              -//
//-------------------------------------------------------------------------//

void reconnect() {

  String clientId = chip_id;                                                              //Se utiliza el id único de la placa como id de cliente
  while (!client.connected()) {                        //Mientras que el cliente no esté conectado, se intenta conectar

    Serial.print("Attempting MQTT connection...");

    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass,topic_conexion,2,true,"Online_False")) {                 //En caso de que el cliente se conecte al servidor con el usuario y la contraseña aportados se conecta y se envía un LWT por el topic conexión
      Serial.println("connected");
            client.subscribe(topic_RGB_cmd);                                                       //Nos suscribimos a todos los topics
            client.subscribe(topic_modo_cmd);
            client.subscribe(topic_switch_cmd);
            client.subscribe(topic_config_cmd);
            client.subscribe(topic_FOTA);
      //Se suscribe al topic de FOTA
    } else {
      Serial.printf("failed, rc=%d  try again in 5s\n", client.state());                  //En caso contrario se informa del fallo y se vuelve a intentar tras un tiempo

      delay(5000);
    }
  }
}

//-------------------------------------------------------------------------//
//-                                SETUP                                  -//
//-------------------------------------------------------------------------//

void setup() {
  //Se inicializan las GPIOs según para lo que se vayan a usar
  pinMode(BUILTIN_LED, OUTPUT);                                                            //Se inicializa el LED de la placa como output                                                                   
  digitalWrite(BUILTIN_LED, HIGH);                                                         //Se apaga el LED (lógica negativa)
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);
  pinMode(14,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(boton_flash,INPUT_PULLUP);                                                       

  attachInterrupt(digitalPinToInterrupt(boton_flash), RTI, CHANGE);                        //Activamos la interrupción para que funcione al cambiar el valor de boton_flash
  
  //Se conecta al wifi y al broker mqtt
  Serial.begin(115200);
  
  setup_wifi();                                                                             //Se llama a la función para conectar al wifi
  intenta_OTA();                                                                            //Buscamos actualizacin al conectarnos a la red
  
  client.setServer(mqtt_server, 1883);                                                      //Se fija el servidor mqtt y el puerto que se va a usar
  client.setBufferSize(512);                                                                //Se fija el tamaño del buffer
  client.setCallback(callback);                                                             //Se especifica la función a la que se llama cuando se recibe un mensaje
 //Podriamos con client.setKeepAlive(KA)cambiar el KA pero mantenemos el default de 15 segundos
                                                                           //Se llama a la función para conectarse al servidor mqtt

}
//-------------------------------------------------------------------------//
//-                                 MAIN                                  -//
//-------------------------------------------------------------------------//

void loop() {
  if (!client.connected()) {                                                                //Si se ha desconectado de MQTT
    
    reconnect();                                                                            //Intenta volver a conectar
  }
  client.loop();                                                                            //Se le devuelve el control a la librería
  //------------------------------

  sensorValue = analogRead(analogPin);                                                      // Se realiza la lectura del sensor VMA407N 
  
  unsigned long now = millis();                                                             //Se guarda el uptime actual de la placa en la variable now


 //----------------------------------------------------------------------
 // Leemos el modo en el que nos encontramos     
  if (modo_auto == 1){                                                                      
    if (sensorValue <= luz_crit){                                                           //Si el valor leído está por debajo de crítico el led debe ponerse rojo
       nivel_verde = 0;
       nivel_rojo = 255;
       nivel_azul = 0;
     }
    else if (sensorValue <= luz_baja){                                                      //Si el valor está por debajo de bajo pero no de crítico el led se pone amarillo
       nivel_verde = 214;
       nivel_rojo = 240;
       nivel_azul = 15;
     }
    else if(sensorValue <= 1023){                                                           //Si el led 
       nivel_verde = 255;
       nivel_rojo = 0;
       nivel_azul = 0;
    }                                                                                       //Al usar if y else if, si no entra en la condicion de arriba ya sabes que el limite inferior no lo cumple por lo que no hace falta especificarlo
  }
   else if (modo_party == 1){                                                               //Colores aleatorios
       nivel_verde = random(255);
       nivel_rojo = random(255);
       nivel_azul = random(255);  
       delay(100);    
  }
//----------------------------------------------------------------------
//Función para aumentar gradualmente el color
     if((nivel_deriva != 0)&&(modo_auto==0)&&(modo_party==0)){                   //Comprobamos que no esten activos ni modo auto ni modo party, si no es así se ignora la graduación
        if(modo_inverso == 1){                                                   // Si modo inverso no está activado sumamos el porcentaje y pasamos del color actual paulatinamente a blanco
          operador = nivel_deriva*(-1);                                          //Si modo inverso esta activado restamos el porcentaje y pasamos a negro
          }
        else{
          operador = nivel_deriva;
          }
         
        if(nivel_rojo < 255){                                                    //Tambien, no queremos que siga sumando a ese color si ya hemos llegado al límite
           nivel_rojo = nivel_rojo + round(2.55* operador);                      //El porcentaje de 255 pero con redondeo ya que el color pwm se representa solo con enteros
           if(nivel_rojo > 255){
              nivel_rojo = 255; 
           }
        }

         if(nivel_verde < 255){
           nivel_verde = nivel_verde + round(2.55* operador);
           if(nivel_verde > 255){
              nivel_verde = 255; 
           }
        }

          if(nivel_azul < 255){
           nivel_azul = nivel_azul + round(2.55 * operador);
           if(nivel_azul > 255){
              nivel_azul = 255; 
           }
        }
     }

//---------------------------------------------------------------------
 //Envia la lectura del sensor junto a la información de la placa cada t_enviar mlisegundos
  if( now - lastMsg > t_enviar){
    StaticJsonDocument<256> datos_VMA407N;                                        //Se crea el documento JSON

    datos_VMA407N["ChipId"] = chip_id;                                            //Se guarda el id único de la placa
    datos_VMA407N["uptime"] = now;                                                //Se guarda el tiempo que lleva activa la placa

    JsonObject VMA407N = datos_VMA407N.createNestedObject("VMA407N");             //Se crea el documento JSON anidado VMA407N
    VMA407N["iluminacion"] = sensorValue;                                         //En el que se guarda la temperatura


    JsonObject Wifi = datos_VMA407N.createNestedObject("Wifi");                   //Se crea el documento JSON anidado WiFi
    Wifi["SSID"] = ssid;                                                          //En el que se guarda el SSID
    Wifi["IP"] = WiFi.localIP();                                                  //La IP de la placa
    Wifi["RSSI"] = WiFi.RSSI();                                                   //Y el RSSI
    
    lastMsg = now;
    serializeJson(datos_VMA407N, msg_datos_VMA407N);                              //Se serializa el mensaje JSON
    client.publish(topic_datos_VMA407N, msg_datos_VMA407N);                       //Y se publica por el topic
  }
//-----------------------------------------------------------------------
  if( t_actualizar != 0){                                                         //Solo miramos si tenemos que actualizar cuando el valor de t_actualizar sea distinto de 0
    if( now - lastAct > t_actualizar){                                            //Si ha pasado t_actualizar buscamos una actualizacion posible
      intenta_OTA();
      lastAct = now;
    }
  }
//-----------------------------------------------------------------------

     if (simple == 1){                                            
     delay(300);                                                                  //Esperamos para ver si es doble o simple 300ms que es el tiempo maximo que  puede haber entre las dos pulsaciones de la doble. Ha sido elegido mediante ensayo
         if(doble==1){                                                            //Si es doble, el led se enciende de color blanco
             nivel_rojo = 255; 
             nivel_verde = 255;
             nivel_azul = 255;   
             //Serial.print("Pulsacion doble");
             simple = 0;
             doble = 0;
          }
         else{                                   //PULSACION SIMPLE
             if(switch_inv == 1){                                                //Podemos apagar y encender el led RGB, y además para que el usuario sepa que funciona se enciende y se apaga también el de la placa
                switch_inv = 0;                                                  
                digitalWrite(BUILTIN_LED, LOW);
                origen = "Placa";                                                //Manamos el origen del mensaje
                //Serial.print("Pulsacion simple");
                simple=0;
              }
              else{
                switch_inv = 1;
                digitalWrite(BUILTIN_LED, HIGH);
                origen = "Placa";
                //Serial.print("Pulsacion simple");
                simple=0;
              }
           }
     }
     else if(prolongada == 1){                                                  //Si es prolongada buscamos una actualizacion
             intenta_OTA();
             //Serial.print("Pulsacion prolongada");
             prolongada =0;
    }
//----------------------------------------------------------------
if((prev_modo_auto != modo_auto)||(prev_modo_party != modo_party)||(prev_modo_inverso != modo_inverso)){                //Solo publica cuando algun modo cambie de valor para que no se cree un bucle infinito
    StaticJsonDocument<96> modo_status;
   
    modo_status["ChipId"] = chip_id;
    modo_status["auto"] = modo_auto; 
    modo_status["party"] = modo_party; 
    modo_status["inverso"] = modo_inverso; 
    serializeJson(modo_status,msg_modo_status);
     Serial.print(msg_modo_status);
    client.publish(topic_modo_status,msg_modo_status);
    }
    
//--------------------------------------------------------------
    if(switch_prev != switch_inv){                                             //Queremos que publique el valor del switch sólo si cambia de valor

      
      switch_prev = switch_inv;                                                //Actualziamos el valor de switch
    StaticJsonDocument<96>switch_status;

      switch_status["ChipId"] = chip_id;
      switch_status["status_sw"] = switch_inv;
      switch_status["origen"] = origen;

      serializeJson(switch_status,msg_switch_status);
      client.publish(topic_switch_status,msg_switch_status);
    }

//--------------------------
                                                                            // Por ultimo, realizamos la escritura de los pines
  analogWrite(ledRojo,nivel_rojo);
  analogWrite(ledVerde,nivel_verde);
  analogWrite(ledAzul,nivel_azul);
  digitalWrite(ledInt,switch_inv);
  delay(100);

}
