#include <Notecard.h>
#include "stm32h7xx_ll_gpio.h"
#include "thingProperties.h"
#include <ArduinoModbus.h>
#include <ArduinoRS485.h>
#include <Scheduler.h>

#define F7M24 0x21
#define pause_trigger 15

#define usbSerial Serial
#define INBOUND_QUEUE_POLL_SECS 10
#define INBOUND_QUEUE_NOTEFILE "relay.qi"
#define INBOUND_QUEUE_COMMAND_FIELD "toggle"

#define DEBUG
#define HIRES

// Energy Meter Parameters
float V_actual, V_avg, V_max, V_min;
float A_actual, A_avg, A_max, A_min;
float W_actual, W_avg, W_max, W_min;
float Var_actual;
float Va_actual, Va_avg, Va_max, Va_min;
float Wh_packet, Varh_packet, Wh_Abs_packet;

typedef struct uPWR_STRUCT {
  float uV_code;
  float uW_code;
  float uWh_code;
} PWR_STRUCT;
PWR_STRUCT user_profile;

/**
  ======================= IMPORTANT =======================
  Before using the system, the following parameters MUST be defined:
 
  - operation_safety_margin: Multiplier indicating safety margin. E.g., 1.1 implies a 10% headroom.
  - estimated_max_power: Expected maximum power (in Watts) under which the network will operate.
  - estimated_max_energy: Expected maximum active energy limit (in Wh) for the network.
  - Device_1_Limiter & Device_2_Limiter: Upper limit current (in Amperes) and power (in Watts) for Device 1 & 2 respectively.
  - Device_1_CompRef & Device_2_CompRef: Energy meter parameters required for accurate system operation.
*/
float operation_safety_margin = 1.1;
float estimated_max_power = 220;
float estimated_max_energy = 2880;
float Device_1_Limiter = 1;
float Device_2_Limiter = 100;
float Device_1_CompRef = A_actual;
float Device_2_CompRef = W_avg;
/*
  It is CRUCIAL to set these parameters accurately to ensure safe and correct operation of the system.
  =========================================================
*/

constexpr auto baudrate { 19200 };

// Calculate preDelay and postDelay in microseconds as per Modbus RTU Specification
// MODBUS over serial line specification and implementation guide V1.02
// Paragraph 2.5.1.1 MODBUS Message RTU Framing
// https://modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
constexpr auto bitduration { 1.f / baudrate };
// constexpr auto preDelayBR { bitduration * 9.6f * 3.5f * 1e6 };
constexpr auto postDelayBR { bitduration * 9.6f * 3.5f * 1e6 };
constexpr auto preDelayBR { bitduration * 10.0f * 3.5f * 1e6 };

int start_time = 0;
int rs485_counter = 0;
int counter = 0;

// Current and previous state of the button.
int buttonState     = 0;
int lastButtonState = 0;

bool relayOn = false;

// Variables to implement button debouncing.
unsigned long lastDebounceTime  = 0;
unsigned long debounceDelay     = 50; // In ms

// Arrays of user LEDs
int userLeds[] = {LED_D0, LED_D1, LED_D2, LED_D3};

int numRelays = 4;

long previousMillis = 0;  

#ifdef HIRES
long readingInterval = 1000 * 10 * 1;
#else
long readingInterval = 1000 * 60 * 1;
#endif

Notecard notecard;

/*************************************
* Energy Meter related routines
*************************************/

/**
  Control the relay output based on the user (consumption) profile input and configured power/energy target.

  @param desired_target Desired resource required to run the connected device on the relay.
  @param req_target Minimum resource required to run the connected device on the relay.
  @param relayTarget Relay to activate or deactivate.
  @return Returns 0 or 1, representing HIGH state for 1 and LOW state for 0.
*/
uint8_t relay_Trigger(int desired_target, int req_target, pin_size_t relayTarget) {
  bool isStable = (desired_target >= req_target) && (desired_target > (req_target * user_profile.uV_code));
  digitalWrite(relayTarget, isStable ? HIGH : LOW);
  usbSerial.print(F("Energy Manager: "));
  usbSerial.print(isStable ? F("Stable operation margin") : F("Unstable / possible overload"));
  usbSerial.print(F(" - Turning "));
  usbSerial.print(isStable ? F("ON: ") : F("OFF: "));
  usbSerial.println(relayTarget);

  return isStable ? 1 : 0;
}

/**
  Initial user profile setup.

  @param init_OperMargin  System operation margin for power budget represented in percentage.
  @param init_Watt  User defined Wattage limit for the system.
  @param init_WhCon User defined Energy consumption limit for the system.
*/
void consumption_profile(float init_OperMargin, float init_Watt, float init_WhCon){
  uOperMargin = init_OperMargin;
  user_profile.uV_code = uOperMargin;

  uWatt = init_Watt;
  user_profile.uW_code = uWatt;

  uWhCon = init_WhCon;
  user_profile.uWh_code = uWhCon;
}

/**
  Simple consumption profile value updater.
*/
void consumption_var_container(){
  uOperMargin = user_profile.uV_code;
  uWatt = user_profile.uW_code;
  uWhCon = user_profile.uWh_code;
}

/**
  Monitors and uses the user defined profile and retrieved information from Energy meter to manage connected devices of interest.

  @param Wh_packet Retrieved energy information from Energy meter in unit of [Wh].
  @param Device_#_f Device flag controlled by relay_Trigger() function. # specifies device number or designation.
  @param directOverride1 Direct override flag for Device #1 controlled via Cloud.
  @param W_avg Average power information retrieved from Energy meter. 
  @param Device_X_Limiter User defined upper limit value for selected device parameter.
  @param Device_X_CompRef User selected Energy Meter parameter reading to be compared as a reference for device actuation and system behavior.
*/
void energy_distro_ctrl() {
  if (Wh_packet != 0) {
      uWhOpta = Wh_packet;
  } else {
      usbSerial.println(F("Energy Manager: Energy information recollection stand-by"));
      return;
  }

  if ((Wh_packet * user_profile.uV_code) >= user_profile.uWh_code) {
      digitalWrite(D0, LOW);
      digitalWrite(D1, LOW);
      digitalWrite(D2, LOW);
      usbSerial.println(F("Energy Manager: Energy consumption is high! - Warning"));
  }

  if (directOverride1) {
      usbSerial.println(F("Energy Manager: Direct Override Request Received"));
      digitalWrite(D0, HIGH);
      digitalWrite(D1, HIGH);
      digitalWrite(D2, HIGH);
  }

  if ((W_avg * user_profile.uV_code) > user_profile.uW_code) {
      usbSerial.println(F("Energy Manager: Average Power is above profile limit! - Warning"));
      usbSerial.println(W_avg * user_profile.uV_code);
  }
}

/**
  Displays electrical, power, and energy consumption information retrieved from the energy meter.

  @param V_XXX Voltage information from Energy meter categorized in actual, average, maximum, and minimum value.
  @param A_XXX Current information from Energy meter categorized in actual, average, maximum, and minimum value.
  @param W_XXX Power information from Energy meter categorized in actual, average, maximum, and minimum value.
  @param Var_XXX Reactive power total information from Energy meter categorized in actual.
  @param Va_XXX Apparent power total information from Energy meter categorized in actual, average, maximum, and minimum value.
  @param Wh_packet Power based energy information from Energy meter.
  @param Varh_packet Reactive power total based energy information from Energy meter.
  @param Wh_Abs_packet Absolute Active total based energy information from Energy meter.
*/
struct DataGroup {
  const char* header;
  float* data;
  const char** units;
  uint8_t size;
};

const char* energyUnits[] = { "Wh", "Varh", "Wh" };
const char* actUnits[] = { "V", "A", "W", "Var", "VA" };
const char* avgUnits[] = { "V", "A", "W", "VA" };
const char* maxUnits[] = { "V", "A", "W", "VA" };
const char* minUnits[] = { "V", "A", "W", "VA" };
const char* cloudParamUnits[] = { "u", "W", "Wh" };

void modbus_com_monitor(){
  float energyData[] = { Wh_packet, Varh_packet, Wh_Abs_packet };
  float actData[] = { V_actual, A_actual, W_actual, Var_actual, Va_actual };
  float avgData[] = { V_avg, A_avg, W_avg, Va_avg };
  float maxData[] = { V_max, A_max, W_max, Va_max };
  
  DataGroup groups[] = {
    { "||| Energy  |||||||||||||||||||||", energyData, energyUnits, sizeof(energyData) / sizeof(energyData[0]) },
    { "||| Act. Data  ||||||||||||||||||", actData, actUnits, sizeof(actData) / sizeof(actData[0]) },
    { "||| AVG Data  |||||||||||||||||||", avgData, avgUnits, sizeof(avgData) / sizeof(avgData[0]) },
    { "||| Max. Data  ||||||||||||||||||", maxData, maxUnits, sizeof(maxData) / sizeof(maxData[0]) },
  };

#ifdef DEBUG
  usbSerial.println(); // Empty println for formating reasons
  for(uint8_t i = 0; i < sizeof(groups) / sizeof(groups[0]); i++) {
    usbSerial.println(F(groups[i].header));
    for(uint8_t j = 0; j < groups[i].size; j++) {
      usbSerial.print(groups[i].data[j]);
      usbSerial.print(" ");
      usbSerial.println(groups[i].units[j]);
    }
  }
  usbSerial.println(); // Empty println for formating reasons
#endif
}

/**
  Requests and retrieves actual electrical, and power information from Energy meter over Modbus RTU protocol.
*/
void modbus_com_actual(){
  float V_actual_temp, A_actual_temp, W_actual_temp, Var_actual_temp, Va_actual_temp;

  // Actual Measurements
  // Voltage (V)
  V_actual_temp = getT5(F7M24, 30107);
  if (V_actual_temp != 0.0f) {
    V_actual = V_actual_temp;
  }

  // Current (A)
  A_actual_temp = getT5(F7M24, 30126);
  if (A_actual_temp != 0.0f) {
    A_actual = A_actual_temp;
  }

  // Active Power Total - Pt (W)
  W_actual_temp = getT6(F7M24, 30140);
  if (W_actual_temp != 0.0f) {
    W_actual = W_actual_temp;
  }
  
  // Reactive Power Total - Qt (var) (IEEE 754)
  Var_actual_temp = getT_Float(F7M24, 32544);
  if (Var_actual_temp != 0.0f) {
    Var_actual = Var_actual_temp;
  }

  // Apparent Power Total - St (VA)
  Va_actual_temp = getT5(F7M24, 30156);
  if (Va_actual_temp != 0.0f) {
    Va_actual = Va_actual_temp;
  }
}

/**
  Requests and retrieves average electrical, and power information from Energy meter over Modbus RTU protocol.
*/
void modbus_com_avg(){
  float V_avg_temp, A_avg_temp, W_avg_temp, Va_avg_temp;

  // Average Measurements
  // Voltage (V)
  V_avg_temp = getT5(F7M24, 35507);
  if (V_avg_temp != 0.0f) {
    V_avg = V_avg_temp;
  }

  // Current (A)
  A_avg_temp = getT5(F7M24, 35526);
  if (A_avg_temp != 0.0f) {
    A_avg = A_avg_temp;
  }

  // Active Power Total - Pt (W)
  W_avg = getT6(F7M24, 35540);
  if (W_avg_temp != 0.0f) {
    W_avg = W_avg_temp;
  }

  // Apparent Power Total - St (VA)
  Va_avg = getT5(F7M24, 35556);
  if (Va_avg_temp != 0.0f) {
    Va_avg = Va_avg_temp;
  }
}

/**
  Requests and retrieves maximum electrical,and power information from Energy meter over Modbus RTU protocol.
*/
void modbus_com_max(){
  float V_max_temp, A_max_temp, W_max_temp, Va_max_temp;

  // Maximum Measurements
  // Voltage (V)
  V_max_temp = getT5(F7M24, 35607);
  if (V_max_temp != 0.0f) {
    V_max = V_max_temp;
  }

  // Current (A)
  A_max_temp = getT5(F7M24, 35626);
  if (A_max_temp != 0.0f) {
    A_max = A_max_temp;
  }

  // Active Power Total - Pt (W)
  W_max_temp = getT6(F7M24, 35640);
  if (W_max_temp != 0.0f) {
    W_max = W_max_temp;
  }

  // Apparent Power Total - St (VA)
  Va_max_temp = getT5(F7M24, 35656);
  if (Va_max_temp != 0.0f) {
    Va_max = Va_max_temp;
  }
}

/**
  Requests and retrieves minimum electrical, and power information from Energy meter over Modbus RTU protocol.
*/
void modbus_com_min(){
  // Minimum Measurements
  // Voltage (V)
  V_min = getT5(F7M24, 35707);

  // Current (A)
  A_min = getT5(F7M24, 35726);

  // Active Power Total - Pt (W)
  W_min = getT6(F7M24, 35740);

  // Apparent Power Total - St (VA)
  Va_min = getT5(F7M24, 35756);
}

/**
  Requests and retrieves energy information from Energy meter over Modbus RTU protocol.
*/
void modbus_com_energy(){
  float Wh_packet_temp, Varh_packet_temp, Wh_Abs_packet_temp;

  // Energy
  // Energy (Wh) - n1
  Wh_packet_temp = getT_Float(F7M24, 32752);
  if (Wh_packet_temp != 0.0f) {
    Wh_packet = Wh_packet_temp;
  }

  // Energy (varh) - n4
  Varh_packet_temp = getT_Float(F7M24, 32758);
  if (Varh_packet_temp != 0.0f) {
    Varh_packet = Varh_packet_temp;
  }

  // Total Absolute Active Energy (Wh)
  Wh_Abs_packet_temp = getT_Float(F7M24, 32760);
  if (Wh_Abs_packet_temp != 0.0f) {
    Wh_Abs_packet = Wh_Abs_packet_temp;
  }
}

/*************************************
* LED PLC Switches
*************************************/

/**
  Defaulting LED states to low when called. 
*/
void plc_led_off() {
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDB, LOW);
  digitalWrite(PIN_SPI_MISO, LOW);
  digitalWrite(PIN_SPI_MOSI, LOW);
  digitalWrite(PIN_SPI_SCK, LOW);
  digitalWrite(PIN_SPI_SS, LOW);
}

/**
  Sets up the LEDs as outputs and calls the plc_led_off() function.
*/
void plc_led_Setup(){
  pinMode(LEDG, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDB, OUTPUT);
  pinMode(PIN_SPI_MISO, OUTPUT);
  pinMode(PIN_SPI_MOSI, OUTPUT);
  pinMode(PIN_SPI_SCK, OUTPUT);
  pinMode(PIN_SPI_SS, OUTPUT);

  plc_led_off();
}

/*************************************
* Digital Port related tasks
*************************************/

void digitalIO_Setup(){
  pinMode(D0, OUTPUT);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
}

/*************************************
* Analog Port related tasks
*************************************/

/**
  Sets up analog ports with 12 bit resolution.
*/
void analogIO_Setup(){
  analogReadResolution(12);

  start_time = millis();
  SCB_DisableDCache();
  usbSerial.println("Start");
}

/*************************************
* RTU related tasks
*************************************/

/**
  Sets up Modbus RTU protocol configuration.
*/
void RTU_Setup(){
  usbSerial.println("Energy Management - Modbus RTU Client");

  RS485.setDelays(preDelayBR, postDelayBR);

  // start the Modbus RTU client
  // 7M.24 Energy meter specifies 19200 of default baudrate and 8N2 frame
  if (!ModbusRTUClient.begin(baudrate, SERIAL_8N2)) {
    usbSerial.println("Failed to start Modbus RTU Client!");
    while (1)
        ;
  }
}

/**
  Obtains T5 data type variable

  @param dev_address Device address.
  @param base_reg Register address.
*/
float getT5(int dev_address, int base_reg) {
  ModbusRTUClient.requestFrom(dev_address, INPUT_REGISTERS, base_reg - 30000, 2);

  while(!ModbusRTUClient.available()) {}

  uint32_t rawreg = ModbusRTUClient.read() << 16 | ModbusRTUClient.read();
  int8_t reg_exp = ((uint8_t*)&rawreg)[3];
  uint32_t reg_base = rawreg & 0x00FFFFFF;
  float reg = reg_base * pow(10, reg_exp);
  
  return reg;
}

/**
  Obtains T6 data type variable

  @param dev_address Device address.
  @param base_reg Register address.
*/
float getT6(int dev_address, int base_reg) {
  ModbusRTUClient.requestFrom(dev_address, INPUT_REGISTERS, base_reg - 30000, 2);
  
  while(!ModbusRTUClient.available()) {}
  
  uint32_t rawreg = ModbusRTUClient.read() << 16 | ModbusRTUClient.read();

  int8_t reg_exp = ((uint8_t*)&rawreg)[3];
  int32_t reg_base = (int32_t)rawreg & 0x007FFFFF;
  if(rawreg & 0x800000) {
    reg_base = -reg_base;
  }
  float reg = reg_base * pow(10, reg_exp);

  return reg;
}

/**
  Obtains T2 data type variable

  @param dev_address Device address.
  @param base_reg Register address.
*/
float getT2(int dev_address, int base_reg) {
  ModbusRTUClient.requestFrom(dev_address, INPUT_REGISTERS, base_reg - 30000, 1);
  while(!ModbusRTUClient.available()) {}

  int16_t rawreg = ModbusRTUClient.read();
  
  return (float)rawreg;
}

/**
  Obtains T_Float data type variable

  @param dev_address Device address.
  @param base_reg Register address.
*/
float getT_Float(int dev_address, int base_reg) {
  ModbusRTUClient.requestFrom(dev_address, INPUT_REGISTERS, base_reg - 30000, 2);
  
  while(!ModbusRTUClient.available()) {}
  
  uint32_t rawreg = ModbusRTUClient.read() << 16 | ModbusRTUClient.read();
  float reg;
  memcpy(&reg, &rawreg, sizeof(float));
  
  return reg;
}


/*****************************
* Arduino Cloud 
*****************************/

/**
  Sets up configuration for Arduino Cloud
*/
void iot_cloud_setup(){
  // Defined in thingProperties.h
  initProperties();

  // Connect to Arduino IoT Cloud
 // ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  
  /*
     The following function allows you to obtain more information
     related to the state of network and IoT Cloud connection and errors
     the higher number the more granular information you’ll get.
     The default is 0 (only errors).
     Maximum is 4
 */
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
}

/*
  Since UOperMargin is READ_WRITE variable, onUOperMarginChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onUOperMarginChange()  {
  usbSerial.println(F("Energy Manager: Operation margin updated from Arduino Cloud"));
  if (uOperMargin > 1){
    user_profile.uV_code = uOperMargin;
  }
}

/*
  Since UWatt is READ_WRITE variable, onUWattChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onUWattChange()  {
  usbSerial.println(F("Energy Manager: Power value updated from Arduino Cloud"));
  if (uWatt > 1){
    user_profile.uW_code = uWatt;
  }
}

/*
  Since UWhCon is READ_WRITE variable, onUWhConChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onUWhConChange()  {
  usbSerial.println(F("Energy Manager: Energy [Wh] value updated from Arduino Cloud"));
  if (uWhCon > 1){
    user_profile.uWh_code = uWhCon;
  }
}

/*
  Since DirectOverride1 is READ_WRITE variable, onDirectOverride1Change() is
  executed every time a new value is received from IoT Cloud.
*/
void onDirectOverride1Change()  {
  if (directOverride1 == true){
    usbSerial.println(F("Energy Manager: Device #1 Override ON from Arduino Cloud"));
    digitalWrite(D0, HIGH); 
    digitalWrite(D1, HIGH); 
    digitalWrite(D2, HIGH); 
  }
  if (directOverride1 == false){
    // D0 will operate depending on the system's resource state
    usbSerial.println(F("Energy Manager: Device #1 under system control"));
  } 
}

/*****************************
* Main
*****************************/
void setup() {
  // Initial Parameter 
  directOverride1 = false;
  uWhOpta = 0;

#ifdef DEBUG
  usbSerial.begin(115200);
  const size_t usb_timeout_ms = 3000;
  for (const size_t start_ms = millis(); !usbSerial && (millis() - start_ms) < usb_timeout_ms;)
      ;
  notecard.setDebugOutputStream(usbSerial);
#endif

  // Analog/Digital IO Port Configuration
  analogIO_Setup();
  digitalIO_Setup();

  // Modbus RTU Configuration 
  RTU_Setup();
  
  // Status LED configuration;
  plc_led_Setup();

  // IoT Cloud Setup
  iot_cloud_setup();

  // Configures user profile and controlled device parameters
  consumption_profile(operation_safety_margin, estimated_max_power, estimated_max_energy);

  notecard.begin();

  J *req = notecard.newRequest("hub.set");
  JAddStringToObject(req, "mode", "continuous");
  JAddNumberToObject(req, "outbound", 15);
  JAddNumberToObject(req, "inbound", 1440);
  JAddBoolToObject(req, "sync", true);
  JAddBoolToObject(req, "align", true);
  JAddNumberToObject(req, "duration", 15);
  notecard.sendRequestWithRetry(req, 5); // 5 seconds
  
  pinMode(BTN_USER, INPUT);
  
  for (int i = 0; i < numRelays; i++) {
    digitalWrite(userLeds[i], HIGH); 
    delay(150); 
    digitalWrite(userLeds[i], LOW); 
    delay(150); 
  }
  
  for (int i = numRelays - 1; i >= 0; i--) {
    digitalWrite(userLeds[i], HIGH); 
    delay(150); 
    digitalWrite(userLeds[i], LOW); 
    delay(150); 
  }

  digitalWrite(LED_RESET, HIGH);

  // Make Sure Relay is Open
  digitalWrite(D0, LOW); 

  // Scheduler -> ModBus
  Scheduler.startLoop(modbus_line);
}

void loop() {
  int reading = digitalRead(BTN_USER);
  unsigned long currentMillis = millis();
  static unsigned long nextPollMs = 0;
  
  // Check if button state has changed.
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if(currentMillis - previousMillis > readingInterval) {
    previousMillis = currentMillis;

    takeReading();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == HIGH) {
        digitalWrite(LED_USER, LOW);
      } else {
        digitalWrite(LED_USER, HIGH);

        toggleRelay();
      }
    }

    if (millis() > nextPollMs)
    {
        nextPollMs = millis() + (INBOUND_QUEUE_POLL_SECS * 1000);
        processInbound();
    }
  }
  
  // Save the current state as the last state, for next time through the loop.
  lastButtonState = reading;
}

/**
  Dedicated function for scheduler to retrieve Energy meter's information over Modbus RTU protocol
*/
void modbus_line(){
  modbus_com_actual();
  modbus_com_avg();
  modbus_com_max();
  modbus_com_min();
  modbus_com_energy();
}

void toggleRelay() {
  bool relayChanged = false;

  if (!relayOn) {
    digitalWrite(D0, HIGH); 
    digitalWrite(LED_D0, HIGH);

    relayOn = true;
    relayChanged = true;
  } else {
    digitalWrite(D0, LOW); 
    digitalWrite(LED_D0, LOW);
    
    relayOn = false;
    relayChanged = true;
  }

  if (relayChanged) {
    J *req = notecard.newRequest("note.add");
    if (req != NULL)
    {
      JAddStringToObject(req, "file", "relay.qo");
      JAddBoolToObject(req, "sync", true);
      J *body = JAddObjectToObject(req, "body");
      if (body != NULL)
      {
        JAddStringToObject(body, "relay", relayOn ? "OPEN" : "CLOSED");
      }
      notecard.sendRequest(req);
    }
  }
}


void takeReading() {
  // Serial print of the data from the Energy Meter
  modbus_com_monitor();

  // TODO: Send to Notecard
  J *req = notecard.newRequest("note.add");
  if (req != NULL)
  {
    JAddStringToObject(req, "file", "energy.qo");
    JAddBoolToObject(req, "sync", true);
    J *body = JAddObjectToObject(req, "body");
    if (body != NULL)
    {
      JAddNumberToObject(body, "energy_wh", Wh_packet);
      JAddNumberToObject(body, "energy_varh", Varh_packet);
      JAddNumberToObject(body, "energy_wh_abs", Wh_Abs_packet);
      JAddNumberToObject(body, "actual_V", V_actual);
      JAddNumberToObject(body, "actual_A", A_actual);
      JAddNumberToObject(body, "actual_W", W_actual);
      JAddNumberToObject(body, "actual_Var", Var_actual);
      JAddNumberToObject(body, "actual_Va", Va_actual);
      JAddNumberToObject(body, "avg_V", V_avg);
      JAddNumberToObject(body, "avg_A", A_avg);
      JAddNumberToObject(body, "avg_W", W_avg);
      JAddNumberToObject(body, "avg_Va", Va_avg);
      JAddNumberToObject(body, "max_V", V_max);
      JAddNumberToObject(body, "max_A", A_max);
      JAddNumberToObject(body, "max_W", W_max);
      JAddNumberToObject(body, "max_Va", Va_max);
    }
    notecard.sendRequest(req);
  }
}

void processInbound() {
  while (true)
  {
      J *req = notecard.newRequest("note.get");
      JAddStringToObject(req, "file", INBOUND_QUEUE_NOTEFILE);
      JAddBoolToObject(req, "delete", true);
      J *rsp = notecard.requestAndResponse(req);
      if (rsp != NULL)
      {
          if (notecard.responseError(rsp))
          {
              notecard.deleteResponse(rsp);
              break;
          }

          J *body = JGetObject(rsp, "body");
          if (body != NULL)
          {
              if (JGetString(body, INBOUND_QUEUE_COMMAND_FIELD)) {
                toggleRelay();
              }
          }
      }
      notecard.deleteResponse(rsp);
  }
}
