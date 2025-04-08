/**
 * @author Martin Pavella
 * @date 28.3.2025
 * 
 * The ESP will automatically connect to WiFi and the MQTT broker after reset. 
 *  - 3 LED blinks indicate failed WiFi connection attempt (it is repeated untill successful connection).
 *  - 5 LED blinks indicate failed MQTT connection attempt (it is repeated untill successful connection).
 */


#include <PubSubClient.h>
#include <WiFi.h>

#define WIFI_FAIL__NUM_BLINKS 3
#define MQTT_FAIL__NUM_BLINKS 5

#define LED_PIN 2

/******** MANUALLY SETUP THE DETAILS FOR THE PROGRAMMED ESP ********/
const char* send_to_server_topic = "esp_to_server/rack0";
const char* receive_from_server_topic = "server_to_esp/rack0";  
const char* esp_client_name = "esp/rack0";  

const char* wifi_network_name = "Nitroduck-BioReactor";
const char* wifi_network_password = "REPLACE_WITH_PASSWORD";
const char* mqtt_server_uri = "bioreactor.local";


#define NUM_LAYERS 5
const int valve_pins[] = {
  27, 16, 17, 25, 26  // Next to each other on Wemos D1 R32
};
const int light_pins[] = {
  23, 5, 13, 12, 14  // Next to each other on Wemos D1 R32
};

long last_message_time = 0;

WiFiClient wifi_client;
PubSubClient pub_sub_client(wifi_client);


// ---------- The following functions manage the connected peripherals. ----------

void blink_led(unsigned int times, unsigned int duration){
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(duration);
    digitalWrite(LED_PIN, LOW); 
    delay(200);
  }
}

void open_valve(unsigned int valve_id) {
  if(valve_id >= NUM_LAYERS){
    Serial.print("!!! VALVE ID TOO LARGE (open_valve): ");
    Serial.println(valve_id);
  } else {
    digitalWrite(valve_pins[valve_id], HIGH);
  }
}

void close_valve(unsigned int valve_id) {
  if(valve_id >= NUM_LAYERS){
    Serial.print("!!! VALVE ID TOO LARGE (close_valve): ");
    Serial.println(valve_id);
  } else {
    digitalWrite(valve_pins[valve_id], LOW);
  }
}

void light_on(unsigned int light_id) {
  if(light_id >= NUM_LAYERS){
    Serial.print("!!! LAYER ID TOO LARGE (light_on): ");
    Serial.println(light_id);
  } else {
    digitalWrite(light_pins[light_id], HIGH);
  }
}

void light_off(unsigned int light_id) {
  if(light_id >= NUM_LAYERS){
    Serial.print("!!! LIGHT ID TOO LARGE (light_off): ");
    Serial.println(light_id);
  } else {
    digitalWrite(light_pins[light_id], LOW);
  }
}


void setup_wifi() {
  // TODO Maybe insert 50ms delay.

  Serial.print("\nConnecting to network: ");
  Serial.println(wifi_network_name);

  WiFi.begin(wifi_network_name, wifi_network_password);

  unsigned int attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    // Blink LED to indicate failed WiFi connection attempt.
    blink_led(WIFI_FAIL__NUM_BLINKS, 400);
    delay(1000);  // Wait before trying again.
    Serial.print(".");

    // After 10 failed attempts (cca. 15 seconds), restart the MCU in case the problem is local.
    if(++attempts > 10){
        ESP.restart(); 
    }
  }

  Serial.println("\nConnected to WiFi.");
  Serial.println("ESP IP address: ");
  Serial.println(WiFi.localIP());
  
}

void connect_to_mqtt_server() {
  // Loop until the connection is successfully established.
  while (!pub_sub_client.connected()) {

    // First, make sure WiFi connection is established.
    if(WiFi.status() != WL_CONNECTED)
      setup_wifi();

    
    Serial.print("Connecting to MQTT broker.");
    if (pub_sub_client.connect(esp_client_name)) { // TODO Change the name of client here if multiple ESP32 are connected
      // Successfull connection.
      Serial.println("Connected");

      // Subscribe to the topic specified for this ESP.
      pub_sub_client.subscribe(receive_from_server_topic);
    } 
    else {
      // Connection unsuccessful.
      Serial.print("Failed, code =");
      Serial.print(pub_sub_client.state());
      Serial.println(" trying again in 5 seconds.");

      // Blink LED to indicate unsuccessful MQTT connection.
      blink_led(MQTT_FAIL__NUM_BLINKS, 400);
      
      // Wait before the next attempt.
      delay(3000);
    }
  }
  
}

bool contains_prefix(String message, char* prefix) {
  // Determine if the `message` contains a given prefix.
  size_t prefix_len = strlen(prefix);
  return message.substring(0, prefix_len) == prefix;
}

int id_after_prefix(String message, char* prefix) {
  // Return the integer, which is stored in the string `message` after some given prefix.
  size_t prefix_len = strlen(prefix);
  int id = message.substring(prefix_len).toInt();
  return id;
}


void mqtt_callback(char* topic, byte* message, unsigned int length) {
  /**
   * Function called when a MQTT message is recieved.
   */
  Serial.print("Message arrived on topic: '");
  Serial.print(topic);
  Serial.println("'. Message: ");
  String str_message;
  
  for (int i = 0; i < length; i++) {
    char next_char = (char) message[i];
    Serial.print(next_char);
    str_message += next_char;
  }
  Serial.println();

  // Respond to the message.
  if (String(topic) == receive_from_server_topic) {
    
    if(str_message == "blink_led"){
      Serial.println("Action: blink LED.");
      blink_led(1,2000);
    }
    else if(contains_prefix(str_message, "open_valve")){
      int valve_id =  id_after_prefix(str_message, "open_valve_");
      open_valve(valve_id);

      Serial.print("Action: open valve ");
      Serial.println(valve_id);
    }
    else if(contains_prefix(str_message, "close_valve")){
      int valve_id = id_after_prefix(str_message, "close_valve_");
      close_valve(valve_id);

      Serial.print("Action: close valve ");
      Serial.println(valve_id);
    }
    else if(contains_prefix(str_message, "light_on")){
      int light_id = id_after_prefix(str_message, "light_on_");
      light_on(light_id);

      Serial.print("Action: light on ");
      Serial.println(light_id);
    }
    else if(contains_prefix(str_message, "light_off")){
      int light_id = id_after_prefix(str_message, "light_off_");
      light_off(light_id);

      Serial.print("Action: light off ");
      Serial.println(light_id);
    }
    else{
      Serial.print("!!! Unknown command: ");
      Serial.println(str_message);
    }
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  for(unsigned i = 0 ; i < NUM_LAYERS ; i++){
    pinMode(valve_pins[i], OUTPUT);
    pinMode(light_pins[i], OUTPUT);
  }

  Serial.begin(115200);

  setup_wifi();

  int mqtt_port = 1883;  // Default MQTT port.
  pub_sub_client.setServer(mqtt_server_uri, mqtt_port);
  pub_sub_client.setCallback(mqtt_callback);
}

void loop() {
  if (!pub_sub_client.connected()) {
    connect_to_mqtt_server();
  }

  pub_sub_client.loop();

  // SIMULATED SENSOR READINGS (for testing purposes)
  //
  // long now = millis();
  // if (now - last_message_time > 1000) {
  //   last_message_time = now;

  //   int reading = random(80,100);  // Simulate some sensor reading.
  //   unsigned int buf_size = 4;
  //   char payload[buf_size] = {0,};
  //   String(reading).toCharArray(payload, buf_size);
  //   Serial.println(payload);

  //   pub_sub_client.publish(send_to_server_topic, payload);
  // }
}
