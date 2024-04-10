/*
    Name:       7segment_ESPS3_Now.ino
    Created:	8/4/2024 5:33:50 μμ
    Author:     ROUSIS_FACTORY\user
	
	******
	Set ESP NOW to send data to 7segment_ESP32S3
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <EEPROM.h>
#include <Rousis7segment.h>
#include <Adafruit_NeoPixel.h>
#define MASTER 0
#define INVERT_DISPLAY true 
#define DISPLAY_DIGITS 4    
#define BUZZER 14

#define	REMOTE_D0 15
#define REMOTE_D1 16
#define REMOTE_D2 17
#define REMOTE_D3 18

//  ----- EEProme addresses -----
//EEPROM data ADDRESSER
#define EEP_WIFI_SSID 0
#define EEP_WIFI_PASS 32
#define EEP_USER_LOGIN 128
#define EEP_USER_PASS 160
#define EEP_SENSOR 196
#define EEP_DEFAULT_LOGIN 198 ///Same with the EEP_DEFAULT_WiFi
#define EEP_DEFAULT_WiFi 198
#define EEP_BRIGHTNESS 199
#define EEP_ADDRESS 200
#define EEP_TICKET_TYPE 201
//------------------------------------------------------------------------------
unsigned long previousMillis = 0;
uint8_t remote_button = 0;
uint8_t buzzer_cnt = 0;
uint8_t flash_cnt = 0;
bool flash_on = false;
bool Scan = false;
uint8_t Address;
uint8_t In_bytes_count = 0;
// Define variables to store Queue to be sent
uint16_t queue = 0;
uint8_t type = 1;
String out_queue;
uint8_t out_counter;
String out_category = "A";
char queue_char[5] = { 0,0,0,0,0 };

char  Line1_buf[10] = { 0 };

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN        48 
Adafruit_NeoPixel pixels(1, PIN, NEO_GRB + NEO_KHZ800);
//----------------------------------------------
//Set LED myLED
Rousis7segment myLED(4, 4, 5, 6, 7);
//----------------------------------------------
// Replace with your ESP32S3 MAC Address
//uint8_t broadcastAddress[] = { 0x48, 0x27, 0xE2, 0x0D, 0x6D, 0xC0 }; // Master MAC Address: 48:27:E2:0D:6D:C0
uint8_t broadcastAddress[] = { 0x48, 0x27, 0xE2, 0x0D, 0x6C, 0xD4 }; // Slave MAC Address: 48:27:E2:0D:6C:D4
String success;
esp_err_t result;

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
	String queue;
	String device;
	String instruction;
	uint8_t counter;
	String category;
} struct_message;

struct_message Queue_senting;
struct_message Queue_receive;
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
	memcpy(&Queue_receive, incomingData, sizeof(Queue_receive));
	Serial.print("Bytes received: ");
	Serial.println(len);
	out_queue = Queue_receive.queue;
	out_counter = Queue_receive.counter;
	out_category = Queue_receive.category;

	Serial.print("Queue Number: ");
	Serial.println(out_queue);
	Serial.print("Counter: ");
	Serial.println(out_counter);
	Serial.print("Category: ");
	Serial.println(out_category);
	Serial.print("Device: ");
	Serial.println(Queue_receive.device);
	Serial.print("Instruction: ");
	Serial.println(Queue_receive.instruction);
	Serial.println("----------------------------------");


	char C = 0xff; uint8_t i = 0;

	char  Line_rec[11] = { "          " };
	while (C)
	{
		C = Queue_receive.queue[i];
		if (C)
		{
			Line_rec[i++] = C;
		}
	}

	//Line_rec[4] = out_counter | 0x30;

	Serial.println("print q:");
	for (i = 0; i < sizeof(Line_rec); i++)
	{
		Serial.print(Line_rec[i], HEX);
		Serial.print(" ");
	}
	Serial.println();
	Serial.println("-----------------------------------");
	
	memccpy(Line1_buf, &Line_rec[0], 2, 4);
	myLED.print(Line1_buf, INVERT_DISPLAY);

	if (Queue_receive.instruction == "CALL") //REFRESH
	{
		flash_on = true;
		flash_cnt = 17;
		buzzer_cnt = 6;
		//digitalWrite(BUZZER, HIGH);
		ledcWrite(1, 128); // Παράγει ημιτονικό (b%) σήμα PWM
		pixels.setPixelColor(0, pixels.Color(0, 150, 0));
		pixels.show();
	}
	else {
		pixels.setPixelColor(0, pixels.Color(150, 0, 0));
		pixels.show();
	}
}
//-----------------------------------------------------------------------------
//Timer setup
//create a hardware timer  of ESP32
hw_timer_t* flash_timer = NULL;
portMUX_TYPE falshMux = portMUX_INITIALIZER_UNLOCKED;
//Timer callback function
void IRAM_ATTR FlashInt()
{
	portENTER_CRITICAL_ISR(&falshMux);

	if (flash_cnt)
	{
		if (flash_on)
		{
			myLED.print("    ", INVERT_DISPLAY);

			flash_on = false;
		}
		else {
			myLED.print(Line1_buf, INVERT_DISPLAY);
			flash_on = true;
		}
		flash_cnt--;
	}
	else {
		myLED.print(Line1_buf, INVERT_DISPLAY);
	}

	if (buzzer_cnt) {
		buzzer_cnt--;
	}
	else {
		digitalWrite(BUZZER, LOW);
		// digitalWrite(12, LOW);
		ledcWrite(1, 0); // Παράγει ημιτονικό (b%) σήμα PWM
		//analogWrite(12, 0, 4000, 8, 0);
	}

		//timerWrite(flash_timer, 0); // Reset the timer to prevent watchdog timeout
	portEXIT_CRITICAL_ISR(&falshMux);

}
//-----------------------------------------------------------------------------
void writeString(char add, String data, uint8_t length)
{
	int _size = length; // data.length();
	int i;
	for (i = 0; i < _size; i++)
	{
		if (data[i] == 0) { break; }
		EEPROM.write(add + i, data[i]);
	}
	EEPROM.write(add + i, '\0');   //Add termination null character for String Data
	EEPROM.commit();
}

String read_String(char add, uint8_t length)
{
	int i;
	char data[100]; //Max 100 Bytes
	int len = 0;
	unsigned char k;
	k = EEPROM.read(add);
	while (k != '\0' && len < length)   //Read until null character
	{
		k = EEPROM.read(add + len);
		data[len] = k;
		len++;
	}
	data[len] = '\0';
	return String(data);
}
//-----------------------------------------------------------------------------
void setup() {
	// Init Serial Monitor
	Serial.begin(115200);
	delay(100);
	Serial.println("Starting ESP32S3 Client");
	
	EEPROM.begin(256);

	pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
	if (MASTER)
	{
		pinMode(REMOTE_D0, INPUT_PULLUP);
		pinMode(REMOTE_D1, INPUT_PULLUP);
		pinMode(REMOTE_D2, INPUT_PULLUP);
		pinMode(REMOTE_D3, INPUT_PULLUP);
	}
	else
	{
		pinMode(REMOTE_D0, PULLDOWN);
		pinMode(REMOTE_D1, PULLDOWN);
		pinMode(REMOTE_D2, PULLDOWN);
		pinMode(REMOTE_D3, PULLDOWN);
	}

	//setup BUZZER pin for PWM 2 KHz
	ledcSetup(1, 2000, 8);
	ledcAttachPin(BUZZER, 1);
	
	//----------------------------------------------
	// Set device as a Wi-Fi Station
	WiFi.mode(WIFI_STA);

	// Init ESP-NOW
	if (esp_now_init() != ESP_OK) {
		Serial.println("Error initializing ESP-NOW");
		return;
	}

	//print MAC Address
	Serial.print("MAC Address: ");
	Serial.println(WiFi.macAddress());

	// Once ESPNow is successfully Init, we will register for Send CB to
	// get
	// the status of Trasnmitted packet
	esp_now_register_send_cb(OnDataSent);

	// Register peer
	memcpy(peerInfo.peer_addr, broadcastAddress, 6);
	peerInfo.channel = 0;
	peerInfo.encrypt = false;

	// Add peer
	if (esp_now_add_peer(&peerInfo) != ESP_OK) {
		Serial.println("Failed to add peer");
		//return;
	}
	else {
		Serial.println("Peer added");
	}
	// Register for a callback function that will be called when data is received
	esp_now_register_recv_cb(OnDataRecv);
	//----------------------------------------------
	// return the clock speed of the CPU
	uint8_t cpuClock = ESP.getCpuFreqMHz();

	flash_timer = timerBegin(1, cpuClock, true);
	timerAttachInterrupt(flash_timer, &FlashInt, true);
	timerAlarmWrite(flash_timer, 100000, true);
	//----------------------------------------------
	pixels.setPixelColor(0, pixels.Color(50, 0, 10));
	pixels.show();

	myLED.displayEnable();     // This command has no effect if you aren't using OE pin
	myLED.displayBrightness(255);
	myLED.normalMode();
	myLED.TestSegments(4);

	update_all_queue();
	buzzer_cnt = 3;
	ledcWrite(1, 128); // Παράγει ημιτονικό (b%) σήμα PWM
	timerAlarmEnable(flash_timer);
}

void loop()
{
	// Check if REMOTE_D0 is pressed high
	if (digitalRead(REMOTE_D0)) {
		call_next();
		Serial.println("Call Next Instruction");
		//loop while REMOTE_D0 is pressed
		while (digitalRead(REMOTE_D0))
		{
			//do nothing
		}
	}
	else if (digitalRead(REMOTE_D1)) {
		recall();
		Serial.println("Recall Instruction");
		//loop while REMOTE_D1 is pressed
		while (digitalRead(REMOTE_D1))
		{
			//do nothing
		}
	}
	else if (digitalRead(REMOTE_D2)) {
		pixels.setPixelColor(0, pixels.Color(150, 0, 0));
		pixels.show();

		queue--;
		if (type)
		{
			if (queue > 499) {
				queue = 499;
			}
		}
		else {
			if (queue > 9999) {
				queue = 9999;
			}
		}
		update_all_queue();
		Serial.println("Decrement Instruction");
		//loop while REMOTE_D2 is pressed
		while (digitalRead(REMOTE_D2))
		{
			//do nothing
		}
	}
	else if (digitalRead(REMOTE_D3)) {
		queue = 0;
		pixels.setPixelColor(0, pixels.Color(0, 0, 150));
		pixels.show();
		update_all_queue();
		Serial.println("Reset Instruction");
		//loop while REMOTE_D3 is pressed
		while (digitalRead(REMOTE_D3))
		{
			//do nothing
		}
	}
	
	//tongle pixels white or black every 1000 millis()
	if (millis() - previousMillis > 1000 && !flash_cnt) {
		previousMillis = millis();
		if (Scan)
		{
			pixels.setPixelColor(0, pixels.Color(50, 50, 50));
		}
		else {
			pixels.setPixelColor(0, pixels.Color(0, 0, 0));
		}
		pixels.show();

		//tongle scan
		if (Scan)
		{
			Scan = false;
		}
		else {
			Scan = true;
		}
	}

	
}

//==============================================================================

void call_next() {
	//inc_cueue();
	pixels.setPixelColor(0, pixels.Color(0, 150, 0));
	pixels.show();

	queue++;
	if (type)
	{
		if (queue > 499) {
			queue = 0;
		}
	}
	else {
		if (queue > 9999) {
			queue = 0;
		}
	}

	update_queue_string();

	for (size_t i = 0; i < sizeof(queue_char); i++)
	{
		Line1_buf[i] = queue_char[i];
	}
	flash_cnt = 17;

	myLED.print(queue_char, INVERT_DISPLAY);
	//Open the BUZZER PWM for 2 KHz
	ledcWrite(1, 128); // Παράγει ημιτονικό (b%) σήμα PWM
	buzzer_cnt = 6;

	Queue_senting.queue = String(queue_char);
	Queue_senting.device = "CONSOLE";
	Queue_senting.instruction = "CALL";
	Queue_senting.counter = 1; // address;
	Queue_senting.category = out_category;

	result = esp_now_send(broadcastAddress, (uint8_t*)&Queue_senting, sizeof(Queue_senting));
	if (result == ESP_OK) {
		Serial.println("Send CALL");
	}
	else
	{
		Serial.println("Error sending");
	}
}

void recall() {
	pixels.setPixelColor(0, pixels.Color(0, 150, 0));
	pixels.show();

	update_queue_string();

	for (size_t i = 0; i < sizeof(queue_char); i++)
	{
		Line1_buf[i] = queue_char[i];
	}
	flash_cnt = 17;

	myLED.print(queue_char, INVERT_DISPLAY);
	buzzer_cnt = 6;
	//Open the BUZZER PWM for 2 KHz
	ledcWrite(1, 128); // Παράγει ημιτονικό (b%) σήμα PWM

	Queue_senting.queue = String(queue_char);
	Queue_senting.device = "CONSOLE";
	Queue_senting.instruction = "CALL";
	Queue_senting.counter = 1; // address;
	Queue_senting.category = out_category;

	result = esp_now_send(broadcastAddress, (uint8_t*)&Queue_senting, sizeof(Queue_senting));
	if (result == ESP_OK) {
		Serial.println("Send CALL");
	}
	else
	{
		Serial.println("Error sending");
	}
}

void update_all_queue() {
	update_queue_string();
	update_queue_display();
	/*ESPUI.getControl(queue_label)->value = queue_char;
	ESPUI.updateControl(queue_label);*/

	Queue_senting.queue = String(queue_char);
	Queue_senting.device = "CONSOLE";
	Queue_senting.instruction = "REFRESH";
	Queue_senting.counter = 1; // address;
	Queue_senting.category = out_category;

	result = esp_now_send(broadcastAddress, (uint8_t*)&Queue_senting, sizeof(Queue_senting));
	if (result == ESP_OK) {
		Serial.println("Send CALL");
	}
	else
	{
		Serial.println("Error sending");
	}
}

void update_queue_display() {
	update_queue_string();
	//copy queue_char to Line1_buf
	for (size_t i = 0; i < sizeof(queue_char); i++)
	{
		Line1_buf[i] = queue_char[i];
	}
	myLED.print(queue_char, INVERT_DISPLAY);
}

void update_queue_string() {
	queue_char[0] = 0;
	queue_char[1] = 0;
	queue_char[2] = 0;
	queue_char[3] = 0;

	itoa(queue, queue_char, 10);
	if (EEPROM.read(EEP_TICKET_TYPE))
	{
		if (queue_char[1] == 0)
		{
			queue_char[2] = queue_char[0];
			queue_char[1] = '0';
			queue_char[0] = '0';

		}
		else if (queue_char[2] == 0) {
			queue_char[2] = queue_char[1];
			queue_char[1] = queue_char[0];
			queue_char[0] = '0';
		}

		switch (queue_char[0])
		{
		case '0':
			queue_char[0] = 'A';
			break;
		case '1':
			queue_char[0] = 'B';
			break;
		case '2':
			queue_char[0] = 'C';
			break;
		case '3':
			queue_char[0] = 'D';
			break;
		case '4':
			queue_char[0] = 'E';
			break;
		default:
			queue_char[0] = 'A';
			break;
		}
	}
}
