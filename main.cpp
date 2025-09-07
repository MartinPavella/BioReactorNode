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

// If `MAIN_BOARD` is defined, the code will compile for the main (lights) board.
// If the symbol is not defined, the code will compile for the secondary (pump) board.
// #define MAIN_BOARD 1  

#define WIFI_FAIL__NUM_BLINKS 3
#define MQTT_FAIL__NUM_BLINKS 5

#define LED_PIN 2

/******** MANUALLY SETUP THE DETAILS FOR THE PROGRAMMED ESP ********/
const char* wifi_network_name = "Nitroduck-BioReactor";
const char* wifi_network_password = "iGEM2025";
const char* mqtt_server_uri = "bioreactor.local";

#ifdef MAIN_BOARD

const char* send_to_server_topic = "esp_to_server/rack0";
const char* receive_from_server_topic = "server_to_esp/rack0";  
const char* esp_client_name = "esp/rack0";  

#else  // !MAIN_BOARD

const char* send_to_server_topic = "esp_to_server/pump";
const char* receive_from_server_topic = "server_to_esp/pump";  
const char* esp_client_name = "esp/pump";  

#endif  // MAIN_BOARD



#define NUM_LAYERS 5

// ----------- Pins for the main board ---------------------------------------------------
#ifdef MAIN_BOARD
const int valve_pins[] = {
  // 4, 23, 1, 21, 18  // Old version. (1 is used by UART for Serial communication.)
  13, 14, 16, 17, 18  // New set.
};
const bool valve_relay_active_high[] = {
  false, false, false, false, false
};

const int light_pins[] = {
  // 15, 22, 3, 19, 5  // Pin 3 is used for UART Serial communication.
  19, 21, 22, 23, 32  // New set.
};  
const bool light_relay_active_high[] = {
  // Should be `true`, since we want the relays to be on for shorter (when light is off).
  // Switched aroud for now.
  false, false, false, false, false  
};

const int PROBE_input_pins[] = {
  // 32, 33, 25, 26, 27  // Old version. (25,26,27 cannot be used while wifi is running)
  34, 35, 36, 39, 33  // New set.
};  
#endif  // MAIN_BOARD
// ---------------------------------------------------------------------------------------

// ----------- Pins for the pump board ---------------------------------------------------
#ifndef MAIN_BOARD
// #define PUMP_PIN 19  // Old version

#define NUM_PERISTALTIC_PUMPS 4
const int peristaltic_pump_pins[] = {
  13, 14, 16, 17
};  
const int mixing_pump_pin = 18;
const int additive_mixers_pin = 19;
const int harvesting_pump_direction_pin = 21;
const int pwm_harwesting_pump_pin = 23;
const int ph_reader_pin = 34;
const int conductivity_reader_pin = 35;
#endif  // !MAIN_BOARD
// ---------------------------------------------------------------------------------------


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

#ifdef MAIN_BOARD
void open_valve(unsigned int valve_id) {
  if(valve_id >= NUM_LAYERS){
    Serial.print("!!! VALVE ID TOO LARGE (open_valve): ");
    Serial.println(valve_id);
  } else {
    if(valve_relay_active_high[valve_id])
      digitalWrite(valve_pins[valve_id], HIGH);
    else
      digitalWrite(valve_pins[valve_id], LOW);
  }
}

void close_valve(unsigned int valve_id) {
  if(valve_id >= NUM_LAYERS){
    Serial.print("!!! VALVE ID TOO LARGE (close_valve): ");
    Serial.println(valve_id);
  } else {
    if(valve_relay_active_high[valve_id])
      digitalWrite(valve_pins[valve_id], LOW);
    else
      digitalWrite(valve_pins[valve_id], HIGH);
  }
}

void light_on(unsigned int light_id) {
  if(light_id >= NUM_LAYERS){
    Serial.print("!!! LAYER ID TOO LARGE (light_on): ");
    Serial.println(light_id);
  } else {
    if(light_relay_active_high[light_id])
      digitalWrite(light_pins[light_id], HIGH);
    else
      digitalWrite(light_pins[light_id], LOW);
  }
}

void light_off(unsigned int light_id) {
  if(light_id >= NUM_LAYERS){
    Serial.print("!!! LIGHT ID TOO LARGE (light_off): ");
    Serial.println(light_id);
  } else {
    if(light_relay_active_high[light_id])
      digitalWrite(light_pins[light_id], LOW);
    else
      digitalWrite(light_pins[light_id], HIGH);
  }
}

void all_lights_on() {
  for(unsigned i = 0 ; i < NUM_LAYERS ; i++)
    light_on(i);
}
void all_lights_off() {
  for(unsigned i = 0 ; i < NUM_LAYERS ; i++)
    light_off(i);
}
void all_valves_on() {
  for(unsigned i = 0 ; i < NUM_LAYERS ; i++)
    open_valve(i);
}
void all_valves_off() {
  for(unsigned i = 0 ; i < NUM_LAYERS ; i++)
    close_valve(i);
}

void close_all_relays(){
  for(unsigned layer = 0 ; layer < NUM_LAYERS ; layer++){
    light_off(layer);
    close_valve(layer);
  }
}

#else  // !MAIN_BOARD

void start_pump(int percentage_power) {
  const int pump_dir = HIGH;  // Can be specified.
  digitalWrite(harvesting_pump_direction_pin, pump_dir);
  analogWrite(pwm_harwesting_pump_pin, 255 * percentage_power / 100);
}

void stop_pump() {
  analogWrite(pwm_harwesting_pump_pin, 0);
}
#endif  // MAIN_BOARD





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

bool contains_prefix(String message, String prefix) {
  // Determine if the `message` contains a given prefix.
  size_t prefix_len = prefix.length();
  return message.substring(0, prefix_len) == prefix;
}

int id_after_prefix(String message, String prefix) {
  // Return the integer, which is stored in the string `message` after some given prefix.
  int id = message.substring(prefix.length()).toInt();
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
#ifdef MAIN_BOARD 
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
    else if(str_message == "all_lights_on"){
      all_lights_on();
      Serial.println("Action: all lights on");
    }
    else if(str_message == "all_lights_off"){
      all_lights_off();
      Serial.println("Action: all lights off");
    }
    else if(str_message == "all_valves_on"){
      all_valves_on();
      Serial.println("Action: all valves on");
    }
    else if(str_message == "all_valves_off"){
      all_valves_off();
      Serial.println("Action: all valves off");
    }

#else  // !MAIN_BOARD

    else if(contains_prefix(str_message, "start_pump:")){
      int percentage_power = id_after_prefix(str_message, "start_pump:");
      start_pump(percentage_power);

      Serial.print("Action: start pump");
      Serial.println(percentage_power);
    }
    else if(str_message == "stop_pump"){
      stop_pump();
      Serial.println("Action: stop pump");
    }
#endif  // MAIN_BOARD

    else{
      Serial.print("!!! Unknown command: ");
      Serial.println(str_message);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Booting...");
  
  pinMode(LED_PIN, OUTPUT);

#ifdef MAIN_BOARD
  for(unsigned i = 0 ; i < NUM_LAYERS ; i++){
    pinMode(valve_pins[i], OUTPUT);
    pinMode(light_pins[i], OUTPUT);
    // analogSetPinAttenuation(PROBE_input_pins[i], ADC_11db);
  }
  
  close_all_relays();

#else  // !MAIN_BOARD
  for(unsigned i = 0 ; i < NUM_PERISTALTIC_PUMPS ; i++){
    pinMode(peristaltic_pump_pins[i], OUTPUT);
  }
  pinMode(harvesting_pump_direction_pin, OUTPUT);
  pinMode(mixing_pump_pin, OUTPUT);
  pinMode(additive_mixers_pin, OUTPUT);

  pinMode(pwm_harwesting_pump_pin, OUTPUT);  // PWM output.
  
  analogSetPinAttenuation(ph_reader_pin, ADC_11db);
  analogSetPinAttenuation(conductivity_reader_pin, ADC_11db);


#endif  // MAIN_BOARD


  
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



  
  // TODO Rework!
  // // Send PROBE sensor readings.
  // int interval_seconds = 1;
  // long now = millis();
  // if (now - last_message_time > interval_seconds * 1000) {
  //   last_message_time = now;

  //   // 9 ("PROBE[i]:") + 4 (decimal chars of read value) + 1 (zero byte). E.g. "PROBE[2]:1337\0"
  //   unsigned int buf_size = 9 + 4 + 1;  
  //   char buffer[buf_size] = {0,};
  //   for(unsigned i = 0 ; i < NUM_LAYERS ; i++){
  //     // Read the value.
  //     String reading = String(analogRead(PROBE_input_pins[i]));
      
  //     // Prepare the buffer to send.
  //     String payload = String("PROBE[") + String(i) + String("]:") + reading;
  //     payload.toCharArray(buffer, buf_size);  

  //     // Log and publish to server.
  //     Serial.print("Reading PROBE[" + String(i) + "]: " + reading);
  //     Serial.println(String("\tSending: ") + payload);

  //     // TODO 
  //     // pub_sub_client.publish(send_to_server_topic, buffer);
  //   }
  //
  // }
}
