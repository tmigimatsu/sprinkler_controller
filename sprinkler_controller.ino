#include <RFM69.h>
#include <SPI.h>
#include <LowPower.h>
#include "sprinkler_controller.h" // Defines SECRET_AES

#define DEBUG

/**
 * Message format:
 *   - G.       :         Get relay status
 *   - S.x.1.t. : Set relay x ON for t milliseconds
 *   - S.x.0.   : Set relay x OFF
 */

static const uint8_t ID_NETWORK = 108;
static const uint8_t ID_CENTRAL = 1;
static const uint8_t ID_NODE    = 2;

static const char AES_ENCRYPT_KEY[] = SECRET_AES; // Defined in sprinkler_controller.h
static const uint32_t FREQUENCY = RF69_915MHZ; // RF69_433MHZ

static const uint8_t NUM_RELAYS = 8;
static const uint8_t PIN_RELAY[NUM_RELAYS] = {9, 8, 7, 6, 5, 4, 14, 15};
static const uint8_t PIN_RFM69_RESET = 3;

enum status_t {
	ON  = LOW,
	OFF = HIGH
};

static const char CHAR_ON  = '1';
static const char CHAR_OFF = '0';
static const char CHAR_GET = 'G';
static const char CHAR_SET = 'S';
static const char CHAR_SEP = '.';

static const unsigned long MAX_ON_DURATION = 30 /* min */ * 60 /* s/min */ * 1e3 /* ms/s */;

class RelayController {

public:

	RelayController() {
		for (int i = 0; i < NUM_RELAYS; i++) {
			status_relay[i] = CHAR_OFF;
		}
		status_relay[NUM_RELAYS] = '\0';
	}

	void write_relay(uint8_t num_relay, status_t command, unsigned long ms_duration = 0);
	bool parse_message(const char *message);
	void check_timers();
	bool relay_is_on();

	char status_relay[NUM_RELAYS + 1];

private:

	enum command_t {
		GET,
		SET
	};

	// write_relay() helper functions
	inline void set_relay(uint8_t num_relay, status_t command);
	inline void set_timer(uint8_t num_relay, unsigned long ms_duration);
	inline bool command_same(uint8_t num_relay, status_t command, unsigned long ms_duration) const;

	// parse_message() helper functions
	bool parse_char(const char **p_message, const char *char_set, uint8_t len_set, char *ret_char) const;
	bool parse_command_type (const char **p_message, command_t *ret_type) const;
	bool parse_num_relay    (const char **p_message, uint8_t *ret_num_relay) const;
	bool parse_command_relay(const char **p_message, status_t *ret_command) const;
	bool parse_on_duration  (const char **p_message, unsigned long *ret_duration) const;

	unsigned long ms_start_relay_[NUM_RELAYS] = { 0 };
	unsigned long ms_duration_relay_[NUM_RELAYS] = { 0 };

};

RelayController controller;
RFM69 radio;

void setup() {

	Serial.begin(115200);
#ifdef DEBUG
	Serial.println("Initialize");
#endif

	// Initialize all pins to OFF
	for (int i = 0; i < NUM_RELAYS; i++) {
		pinMode(PIN_RELAY[i], OUTPUT);
		digitalWrite(PIN_RELAY[i], OFF);
	}

	// Reset RFM69
	pinMode(PIN_RFM69_RESET, OUTPUT);
	digitalWrite(PIN_RFM69_RESET, HIGH);
	delay(100);
	digitalWrite(PIN_RFM69_RESET, LOW);
	delay(100);

	// Initialize radio
	radio.initialize(FREQUENCY, ID_NODE, ID_NETWORK);
	radio.promiscuous(true);
	radio.setHighPower(false);
	radio.encrypt(AES_ENCRYPT_KEY);

	radio.readAllRegs();

}

void loop() {

	static bool report_status = true;

	// Receive message
	// if (radio.receiveDone() && radio.SENDERID == ID_CENTRAL) {
	if (radio.receiveDone()) {
		// Return acknowledgement
		if (radio.ACKRequested()) radio.sendACK();
#ifdef DEBUG
		Serial.print("Received message ");
		Serial.print((char *)radio.DATA);
		Serial.print(" from node ");
		Serial.print(ID_CENTRAL, DEC);
		Serial.print(" with RSSI ");
		Serial.print(radio.readRSSI(), DEC);
		Serial.println(".");
#endif

		// Parse message
		report_status = controller.parse_message(radio.DATA);
	} else if (!report_status && !controller.relay_is_on()) {
		LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
	}

	// Check timers
	controller.check_timers();

	// Send status
#ifdef DEBUG
	if (report_status) {
		Serial.print("Send status ");
		Serial.print(controller.status_relay);
		Serial.print(" to node ");
		Serial.print(ID_CENTRAL, DEC);
		Serial.println(".");
	}
#endif

	// delay(100);
	if (report_status && radio.sendWithRetry(ID_CENTRAL, controller.status_relay, NUM_RELAYS + 1, 3, 100)) {
		report_status = false;
	} else {
		report_status = false;
	}

}



void RelayController::set_relay(uint8_t num_relay, status_t command) {
	// Write command
	digitalWrite(PIN_RELAY[num_relay], command);
	status_relay[num_relay] = (command == ON) ? CHAR_ON : CHAR_OFF;
}

void RelayController::set_timer(uint8_t num_relay, unsigned long ms_duration) {
	// Set timer
	ms_start_relay_[num_relay] = millis();
	ms_duration_relay_[num_relay] = ms_duration;
}

bool RelayController::command_same(uint8_t num_relay, status_t command, unsigned long ms_duration) const {
	char char_command = (command == ON) ? CHAR_ON : CHAR_OFF;
	return status_relay[num_relay] == char_command
		   && ms_duration_relay_[num_relay] == ms_duration;
}

void RelayController::write_relay(uint8_t num_relay, status_t command, unsigned long ms_duration = 0) {
	if (command_same(num_relay, command, ms_duration)) return;

	set_relay(num_relay, command);
	if (command == ON) {
		set_timer(num_relay, ms_duration);
	}
#ifdef DEBUG
	Serial.print("Set relay ");
	Serial.print(num_relay, DEC);
	Serial.print(" to ");
	Serial.print(command == ON ? "ON" : "OFF");
	Serial.print(" for ");
	Serial.print(ms_duration);
	Serial.println("ms.");
#endif
}

void RelayController::check_timers() {
	unsigned long ms_curr = millis();
	for (int i = 0; i < NUM_RELAYS; i++) {
		if (status_relay[i] == CHAR_OFF) continue;
		if (ms_curr - ms_start_relay_[i] < ms_duration_relay_[i]) continue;

		write_relay(i, OFF);
	}
}

bool RelayController::relay_is_on() {
	for (int i = 0; i < NUM_RELAYS; i++) {
		if (status_relay[i] == CHAR_ON) return true;
	}
	return false;
}

bool RelayController::parse_char(const char **p_message, const char *char_set, uint8_t len_set, char *ret_char) const {
	// Parse character
	char c;
	int num_filled = sscanf(*p_message, "%c.", &c);
	if (num_filled < 1) return false;
	
	// Check if character is in valid set
	int i;
	for (i = 0; i < len_set; i++) {
		if (c == char_set[i]) break;
	}
	if (i == len_set) return false;

	// Return values
	*p_message += 2;
	*ret_char = c;
	return true;
}

bool RelayController::parse_command_type(const char **p_message, command_t *ret_type) const {
	static const char SET_TYPES[2] = {CHAR_GET, CHAR_SET};

	// Parse command type
	char char_type;
	if (!parse_char(p_message, SET_TYPES, 2, &char_type)) return false;

	command_t type = (char_type == CHAR_GET) ? GET : SET;

	// Return values
	*ret_type = type;
	return true;
}

bool RelayController::parse_num_relay(const char **p_message, uint8_t *ret_num_relay) const {
	const char *message = *p_message;

	// Parse relay number
	uint8_t num_relay;
	int num_filled = sscanf(message, "%hhu.", &num_relay);
	if (num_filled < 1) return false;
	if (num_relay > NUM_RELAYS) return false;

	// Find position of CHAR_SEP
	message = strchr(message, CHAR_SEP);

	// Return values
	*p_message = message + 1;
	*ret_num_relay = num_relay;
	return true;
}

bool RelayController::parse_command_relay(const char **p_message, status_t *ret_command) const {
	static const char SET_COMMANDS[2] = {CHAR_ON, CHAR_OFF};

	// Parse command type
	char char_command;
	if (!parse_char(p_message, SET_COMMANDS, 2, &char_command)) return false;

	status_t command = (char_command == CHAR_ON) ? ON : OFF;

	// Return values
	*ret_command = command;
	return true;
}

bool RelayController::parse_on_duration(const char **p_message, unsigned long *ret_duration) const {
	const char *message = *p_message;

	// Parse duration and CHAR_SEP
	unsigned long duration;
	int num_filled = sscanf(message, "%lu.", &duration);
	if (num_filled < 1) return false;
	if (duration == 0 || duration > MAX_ON_DURATION) duration = MAX_ON_DURATION;

	// Find position of CHAR_SEP
	message = strchr(message, CHAR_SEP);

	// Return values
	*p_message = message + 1;
	*ret_duration = duration;
	return true;
}

bool RelayController::parse_message(const char *message) {

	// Parse command type
	command_t type;
	if (!parse_command_type(&message, &type)) return false;

	if (type == SET) {

		// Parse relay number and command
		uint8_t num_relay;
		status_t command;
		if (!parse_num_relay(&message, &num_relay)) return false;
		if (!parse_command_relay(&message, &command)) return false;

		if (command == ON) {

			// Parse duration
			unsigned long ms_duration;
			if (!parse_on_duration(&message, &ms_duration)) return false;

			// Write relay on
			write_relay(num_relay, ON, ms_duration);

		} else if (command == OFF) {

			// Write relay off
			write_relay(num_relay, OFF);
			return true;

		}

	} else if (type == GET) {

		// Ask to report status
		return true;

	}
}

