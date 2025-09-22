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
#define MAIN_BOARD 1  

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
  16, 23, 32, 21, 18
};
const bool valve_relay_active_high[] = {
  false, false, false, false, false
};

const int light_pins[] = {
  13, 22, 14, 19, 17
};  
const bool light_relay_active_high[] = {
  // Should be changed `true`, since we want the relays to be on for shorter (when light is off).
  // Switched aroud for now.
  false, false, false, false, false  
};

const int PROBE_input_pins[] = {
   39, 33, 34, 35, 36
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

void read_and_publish_probe_measurement() {
  // TODO Change the message to PROBE_store or something silimar. Or use a separate MQTT topic.
  // Prepare the buffer to send. The format is: "PROBE[<layer_id>]:<raw_value>"
  // 6 ("PROBE[") + 1 (layer number decimal) + 2 (]:) + 4 (decimal chars of the read value) + 1 (zero byte). E.g. "PROBE[0]:1234\0"
  unsigned int buf_size = 6 +1 + 2 + 4 + 1;  
  char buffer[buf_size] = {0,};

  // Read the values.
  for(unsigned layer = 0 ; layer < NUM_LAYERS ; layer++){
    int raw_value = analogRead(PROBE_input_pins[layer]);
    if(raw_value == 0){
      // The PROBE for this layer is not conneted. Don't send any data.
      continue;
    }
    String probe_reading = String(raw_value);

    String payload = String("PROBE[") + String(layer) + String("]:") + probe_reading;
    payload.toCharArray(buffer, buf_size);  

    // Log and publish to server.
    Serial.println("Reading PROBE[" + String(layer) + "] = " + probe_reading);
    Serial.println(String("\tSending: ") + buffer);
    pub_sub_client.publish(send_to_server_topic, buffer);
  }

}

void read_and_publish_single_probe_reading(int probe_idx) {
  // CURRENTLY UNUSED!!!!!
  // Prepare the buffer to send. The format is: "PROBE_reading[<layer_id>]:<raw_value>"
  // 14 ("PROBE_reading[") + 1 (layer number decimal) + 2 (]:) + 4 (decimal chars of the read value) + 1 (zero byte). E.g. "PROBE_reading[0]:1234\0"
  unsigned int buf_size = 14 + 1 + 2 + 4 + 1;  
  char buffer[buf_size] = {0,};

  // Read the values.
  int raw_value = analogRead(PROBE_input_pins[probe_idx]);
  String probe_reading = String(raw_value);

  String payload = String("PROBE_reading[") + String(probe_idx) + String("]:") + probe_reading;
  payload.toCharArray(buffer, buf_size);  

  // Log and publish to server.
  // Serial.println("Reading single PROBE[" + String(probe_idx) + "] = " + probe_reading);
  // Serial.println(String("\tSending: ") + buffer);
  pub_sub_client.publish(send_to_server_topic, buffer);
}

void read_and_publish_single_probe_readings() {
  // Prepare the buffer to send. The format is: "PROBE_readings:<raw_value1>,<raw_value2>,<raw_value3>,<raw_value4>,<raw_value5>"
  // 14 ("PROBE_readings") + 5 * [1 (separator) + 4 (decimal chars of the read value)] + 1 (zero byte). E.g. "PROBE_readings:14,1124,15,574,2341\0"
  unsigned int buf_size = 14 + 5 * (1 + 4) + 1;  
  char buffer[buf_size] = {0,};

  // Read the values.
  int raw_values[5] = {0,};
  for(unsigned i = 0 ; i < 5 ; i++){
    raw_values[i] = analogRead(PROBE_input_pins[i]);
  }


  String payload = String("PROBE_readings:") + String(raw_values[0]);
  for(unsigned i = 1 ; i < 5 ; i++){
    payload += String(",") + String(raw_values[i]);
  }

  payload.toCharArray(buffer, buf_size);  

  // Log and publish to server.
  // Serial.println("Reading single PROBE[" + String(probe_idx) + "] = " + probe_reading);
  // Serial.println(String("\tSending: ") + buffer);
  pub_sub_client.publish(send_to_server_topic, buffer);
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

void start_peristaltic_pump(int id_) {
  digitalWrite(peristaltic_pump_pins[id_], HIGH);
}
void stop_peristaltic_pump(int id_) {
  digitalWrite(peristaltic_pump_pins[id_], LOW);
}

void additive_mixing_on() {
  digitalWrite(additive_mixers_pin, HIGH);
}
void additive_mixing_off() {
  digitalWrite(additive_mixers_pin, LOW);
}

void reservoir_mixing_on() {
  digitalWrite(mixing_pump_pin, HIGH);
}
void reservoir_mixing_off() {
  digitalWrite(mixing_pump_pin, LOW);
}

void read_and_publish_ph_and_cunductivity_measurement() {
  // Read the values.
  String ph_reading = String(analogRead(ph_reader_pin));
  String conductivity_reading = String(analogRead(conductivity_reader_pin));
  
  // Prepare the buffer to send. The format is: "ph-cond:<ph_value>:<conductivity_value>"
  // 8 ("ph-cond:") + 4 (decimal chars of ph read value) + 1 (:) + 4 (conductivity value) + 1 (zero byte). E.g. "ph-cond:1337:1234\0"
  unsigned int buf_size = 8 + 4 + 1 + 4 + 1;  
  char buffer[buf_size] = {0,};
  String payload = String("ph-cond:") + ph_reading + String(":") + conductivity_reading;
  payload.toCharArray(buffer, buf_size);  

  // Log and publish to server.
  Serial.println("Reading ph = " + ph_reading + ", conductivity = " + conductivity_reading);
  Serial.println(String("\tSending: ") + buffer);
  pub_sub_client.publish(send_to_server_topic, buffer);

}

void read_and_publish_single_ph_and_cunductivity_measurement() {
  // Read the values.
  String ph_reading = String(analogRead(ph_reader_pin));
  String conductivity_reading = String(analogRead(conductivity_reader_pin));
  
  // Prepare the buffer to send. The format is: "ph-cond_reading:<ph_value>:<conductivity_value>"
  // 16 ("ph-cond_reading:") + 4 (decimal chars of ph read value) + 1 (:) + 4 (conductivity value) + 1 (zero byte). E.g. "ph-cond_reading:1337:1234\0"
  unsigned int buf_size = 16 + 4 + 1 + 4 + 1;  
  char buffer[buf_size] = {0,};
  String payload = String("ph-cond_reading:") + ph_reading + String(":") + conductivity_reading;
  payload.toCharArray(buffer, buf_size);  

  // Log and publish to server.
  Serial.println("Reading ph = " + ph_reading + ", conductivity = " + conductivity_reading);
  Serial.println(String("\tSending: ") + buffer);
  pub_sub_client.publish(send_to_server_topic, buffer);

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

    else if(str_message == "trigger_probe_measurement"){
      Serial.println("Action: PROBE measurement");
      read_and_publish_probe_measurement();
    }

    else if(contains_prefix(str_message, "get_probe_readings")){
      read_and_publish_single_probe_readings();

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

    else if(contains_prefix(str_message, "peristaltic_on:")){
      int id_ = id_after_prefix(str_message, "peristaltic_on:");
      start_peristaltic_pump(id_);

      Serial.print("Action: peristatlic pump on ");
      Serial.println(id_);
    }
    else if(contains_prefix(str_message, "peristaltic_off:")){
      int id_ = id_after_prefix(str_message, "peristaltic_off:");
      stop_peristaltic_pump(id_);

      Serial.print("Action: peristatlic pump off ");
      Serial.println(id_);
    }

    else if(str_message == "additive_mixing_on"){
      additive_mixing_on();
      Serial.println("Action: additive mixing on");
    }
    else if(str_message == "additive_mixing_off"){
      additive_mixing_off();
      Serial.println("Action: additive mixing off");
    }

    else if(str_message == "reservoir_mixing_on"){
      reservoir_mixing_on();
      Serial.println("Action: reservoir mixing on");
    }
    else if(str_message == "reservoir_mixing_off"){
      reservoir_mixing_off();
      Serial.println("Action: reservoir mixing off");
    }

    else if(str_message == "trigger_ph_cond_measurement"){
      read_and_publish_ph_and_cunductivity_measurement();
    }

    else if(str_message == "get_ph_cond_reading"){
      read_and_publish_single_ph_and_cunductivity_measurement();
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
}
