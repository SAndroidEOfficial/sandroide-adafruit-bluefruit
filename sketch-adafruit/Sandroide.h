/*
 * Sandroide library for Adafruit Bluefruit 32u4 devices
 * VERSION: 0.1
 * 
 * Before using this file, please install Blefruit libraries as shown in:
 * https://learn.adafruit.com/adafruit-feather-32u4-bluefruit-le/setup
 * 
 * Then include this library in your Arduino ".ino" file like shown here:
 * 
 * #include "Sandroide.h"
 * void setup(void)
 * {  
 *    // your code here
 *    setupSandroide(false);  // true to enable Sandroide library logs on serial port
 *    // your code here
 * }
 * 
 * void loop(void) 
 * {  
 *    // your code here
 *    loopSandroide();
 *    // your code here
 * }
 * 
*/

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"


// Create the bluefruit object
#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// pin configuration modes:
#define PIN_INPUT                 0  // digital input pin
#define PIN_OUTPUT                1  // digital output pin
#define PIN_ANALOG                2  // analog pin in analogInput mode
#define PIN_PWM                   3  // digital pin in PWM output mode
#define PIN_SERVO                 4  // digital pin in Servo output mode
#define PIN_NOTSET                5  // pin not set
#define NO_GROUP                  0  // no_group means that current pin is sampled and transmitted individually
#define TXRX_BUF_LEN              20 // size of ble buffer

#define ANALOG_MAX_VALUE          1023 // this is uint16 max value: 65535
#define DEFAULT_SAMPLING_INTERVAL 1000   // 1 second
#define DEFAULT_DELTA             10    // this is uint16 in range [0-65535], 655 is 1% delta

static const int maxPins = 30;
uint8_t pinTypes[maxPins];
uint8_t pinGroups[maxPins];
uint16_t prevValues[maxPins];

int pinSamplingIntervals[maxPins];
int pinTimers[maxPins];
uint16_t pinDelta[maxPins];
static int timer1_counter;
static bool sandroideDebug = false;

static volatile int gcd=-1;
static volatile bool recalcTimer = false;
static volatile bool triggerSensorPolling = false;
static volatile bool gcdChanged =false;


void array_to_string(byte array[], unsigned int len, char buffer[])
{
    for (unsigned int i = 0; i < len; i++)
    {
        byte nib1 = (array[i] >> 4) & 0x0F;
        byte nib2 = (array[i] >> 0) & 0x0F;
        buffer[i*2+0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
        buffer[i*2+1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
    }
    buffer[len*2] = '\0';
}

// function to send BLE message from Adafruit device to Smartphone using Adafruit Bluefruit Bluetooth class
void sendMessage(uint8_t *buf, int len) {
  char str[32] = "";
  if (ble.isConnected()) {
    //if (queue==toSendQueue) {
        array_to_string(&buf[0], len, str);
        if (sandroideDebug) { Serial.print("SENDING "); Serial.println(str);}
        ble.print("AT+BLEUARTTX=");
        ble.println(str);
        //ble.print(buf[0]);
        //ble.printByteArray(buf,len);
        //ble.printByteArray(&buf[1],len-1);
        //for()
        //ble.write((char*)buf, len+13);
        //ble.println((char*)buf);
    /*} else {
        NODE *item = NULL;
        item = (NODE*) malloc(sizeof (NODE));
        memcpy(item->data.payload, buf, len); 
        item->data.length = len;
        Enqueue(queue, item);
    }*/
  }
}

// Reads a pin value,  given its pin number
uint16_t readPin(uint8_t pin) {
    uint8_t mode = pinTypes[pin];
    if (mode==PIN_INPUT) { // exists and is initialized as digital output
        //Serial.print(" INPUT "); Serial.println(digitalRead(pin));
        return digitalRead(pin)==LOW?0:ANALOG_MAX_VALUE;
    } else if (mode==PIN_OUTPUT) { // exists and is initialized as digital output
        //Serial.print(" OUTPUT "); Serial.println(digitalRead(pin));
        return digitalRead(pin)==LOW?0:ANALOG_MAX_VALUE;
    } else if (mode==PIN_ANALOG) { // exists and is initialized as digital output
        //Serial.print(" ANALOG "); Serial.println(analogRead(pin));
        return analogRead(pin);
    }
    return 0;
}

// Generates ble message to send value of a single pin to smartphone
void sendPinValue(uint8_t pin, uint16_t value) {
    if (sandroideDebug) { Serial.print("sendPinValue function for pin "); Serial.println(pin);}
    uint8_t buf[TXRX_BUF_LEN];
    buf[0]='G';
    buf[1]=pin;
    buf[2]=(uint8_t)(value>>8); 
    buf[3]=  (uint8_t) ((value<<8)>>8);
    sendMessage(buf, 4);
    prevValues[pin] = value;
}

// Generates ble message to send value of a group of pins to smartphone
// Pin groups currently do not work. TOFIX
void sendGroup(uint8_t groupno) {
    uint8_t buf[TXRX_BUF_LEN], i=0;
    buf[i++]='G';
    for (uint8_t pin=0; pin<maxPins;pin++){
        if (pinGroups[pin]==groupno) {
            uint16_t value = readPin(pin);
            buf[i++] = pin;
            buf[i++] = (uint8_t)(value>>8); 
            buf[i++] = (uint8_t) ((value<<8)>>8);
            prevValues[pin] = value;
        }
    }
    if (i>1) { // at least 1 pin value to send
        sendMessage(buf, i);
    }
}

// Check if pin is an input pin (DIGITAL INPUT OR ANALOG INPUT)
bool isInputPin(uint8_t pin) {
    if (pin<maxPins){
        uint8_t type = pinTypes[pin];
        return type==PIN_INPUT||type==PIN_ANALOG;
    }
    return false;
}

// Check if any input pin with expired timer has changed value
void check_pin_changed(void) 
{
    if (sandroideDebug) { Serial.print("CHECKING WITH GCD: "); Serial.println(gcd);}
    //uint8_t buf[QUEUE_STRING_LENGTH];
    if (gcd>0) {
        for (int pin=0; pin<maxPins;pin++){
            if (isInputPin(pin)) {
                if (sandroideDebug) { Serial.print(" PIN NO: "); Serial.print(pin); Serial.print(" TIMER: "); Serial.print(pinTimers[pin]);}
                if (pinTimers[pin] < 0) {
                    pinTimers[pin] = pinSamplingIntervals[pin];
                } else {
                    pinTimers[pin]-=gcd;
                }
                if (pinTimers[pin]==0) {
                    pinTimers[pin] = pinSamplingIntervals[pin];
                    uint16_t value = readPin(pin);
                    if (sandroideDebug) { Serial.print(" OLD: "); Serial.print(prevValues[pin]); Serial.print(" NEW: "); Serial.print(value);}
                    if (abs(prevValues[pin]-value) >= pinDelta[pin]) {
                        if (sandroideDebug) { Serial.println(" CHANGED!!!!!!!!!!!!");}
                        // TOFIX: Pin groups do not work with Bluefruit
                        //if (pinGroups[pin]!=NO_GROUP) { // enqueue sending operation for group
                        //    int len = sprintf((char *)buf,"R%c",pinGroups[pin]);
                            //sendMessage( buf, len);                        
                        //} else { // send the pin
                            sendPinValue(pin,value);    
                        //}
                    } else {
                        if (sandroideDebug) { Serial.println("----");}
                    }
                }else {
                        if (sandroideDebug) { Serial.println();}
                }
            }
        }
    }  
}

// Calculate the gcd (Greatest common divisor) between two numbers
int calc_gcd(int n1,int n2) {
    int lgcd=1;
    for(int i=2; i <= n1 && i <= n2; ++i)
    {
        // Checks if i is factor of both integers
        if(n1%i==0 && n2%i==0)
            lgcd = i;
    }
    return lgcd;
}

// Calculate the GCD (Greatest common divisor) between sampling intervals. 
// This function gets called each time a new pin sampling interval configuration is sent from smartphone
void calc_timer_interval()
{
    gcd = -1;
    for (int pin=0; pin<maxPins;pin++){
        if (isInputPin(pin) && pinSamplingIntervals[pin]>0) {
            uint8_t buf[TXRX_BUF_LEN];
            int len = sprintf((char *)buf,"TIMER %d@%d",pin,pinSamplingIntervals[pin]);
            //int len = sprintf((char *)buf,"check-gcd");
            sendMessage(buf, len);
            
            if (gcd==-1) {
                gcd = pinSamplingIntervals[pin];
            } else {
                gcd = calc_gcd(gcd,pinSamplingIntervals[pin]);
            }
        }
    }

    timer1_counter = 65536- (31250/(1000/(float)gcd));
    if (sandroideDebug) { Serial.print("GCD = "); Serial.println(gcd); Serial.println(timer1_counter);}
}

// Configure a pin mode (DIGITAL, ANALOG, INPUT, OUTPUT )
// This function gets called each time a new pin configuration is sent is sent from smartphone
bool initPin(uint8_t pin, uint8_t type){
    bool ret=false,wasset=true,armTimer=false;
   
    if (pin<maxPins) {       // "initPin(): Pin number out of bounds"
        wasset = pinTypes[pin]!=PIN_NOTSET;
        if (sandroideDebug) { Serial.print("Pin "); Serial.print(pin); Serial.print(" configured as ");}
        if ((type==PIN_INPUT||type==PIN_OUTPUT)) {
            if (sandroideDebug) { Serial.println(type==PIN_INPUT?"DIGITAL INPUT":"DIGITAL OUTPUT");}
            pinMode(pin,type==PIN_INPUT?INPUT:OUTPUT);
            //if (type==PIN_INPUT) digitals[mapDigitals[pin]].input();  // initialize as input
            //if (type==PIN_OUTPUT) digitals[mapDigitals[pin]].output(); // initialize as input
            pinTypes[pin] = type; // mark the pin as initialized
            ret =true;
        } else if (type==PIN_ANALOG){ // && mapAnalogs[pin]>=0) {    TODO: check if analog is allowed on this pin
            if (sandroideDebug) { Serial.println(" ANALOG INPUT ");}
            pinMode(pin,INPUT);
            pinTypes[pin] = type; // mark the pin as initialized
            ret =true;
        }
        if (!wasset && ret && (type==PIN_INPUT||type==PIN_ANALOG)) armTimer=true;
    }
    if (armTimer) {
        pinSamplingIntervals[pin] = DEFAULT_SAMPLING_INTERVAL;
        //pinTimers[pin]=pinSamplingIntervals[pin];
        recalcTimer = true;
    }
    
    return ret;
}

// Configure a pin mode (DIGITAL, ANALOG, INPUT, OUTPUT ) and group
// Pin groups currently do not work. TOFIX
bool initPin(uint8_t pin, uint8_t type, uint8_t group){
    bool ret = initPin(pin, type);
    if (ret){
        pinGroups[pin]=group;
    }
    return ret;
}

// Change delta threshold for a pin, which is used to check if a pin reading is changed
// delta must be a number [0, 1023]
// If new pin reading differs more than delta from the last reading sent via Bluetooth
// then the value is changed and the new value must be sent via Bluetooth
// Minimum value allowed is 0: always send the reading
// Maximum value allowed for delta is 1/20 of the fullscale: 1024*0.2 = 205. 
void changeDelta(uint8_t pin, uint16_t delta) {
    uint8_t buf[TXRX_BUF_LEN];
    int len = sprintf((char *)buf,"DELTA %d@%d",pin,delta);
    sendMessage(buf, len);
    
    //float fdelta = delta / ANALOG_MAX_VALUE;
    // delta minimo è il 20% di 1023
    if (delta > ((ANALOG_MAX_VALUE+1)/20)) delta=((ANALOG_MAX_VALUE+1)/20);
    if (isInputPin(pin)) {
        pinDelta[pin] = delta;
    }
}

// Change delta threshold.
// fdelta must be a percentage
void changeDeltaPercent(uint8_t pin, float fdelta) {
    changeDelta(pin, (uint16_t)(fdelta*ANALOG_MAX_VALUE));
}

// Change sampling interval of the specified pin
// interval must be in ms [1, MAX_INT VALUE]
void changeSamplingInterval(uint8_t pin, int interval) {
    if (isInputPin(pin)) {
        pinSamplingIntervals[pin]= interval;
        recalcTimer = true;
    }
}

// Set value of a digital output pin
// value is a boolean corresponding to HIGH and LOW values
bool writeDigital(uint8_t pin, bool value){
    if (pinTypes[pin]==PIN_OUTPUT) {
        if (sandroideDebug) { Serial.print("Pin "); Serial.print(pin); Serial.print(" value set as "); Serial.println(value?"HIGH":"LOW");}
        digitalWrite(pin,value?HIGH:LOW);
    }
}

// Parse a message received from Bluetooth/Smartphone
void parseRedBearCmd(uint8_t* cmdString){
    uint8_t buf[TXRX_BUF_LEN];
    memset(buf, 0, TXRX_BUF_LEN);
    int len=0, scanned=-1, sampling=-1;
    float fdelta=-1; 
    
    uint8_t startOffset = cmdString[0]==0?1:0;
    uint8_t index = startOffset;
    uint8_t cmd = cmdString[index++], pin=cmdString[index++], mode=PIN_NOTSET, group=NO_GROUP;
    //Serial.print("PARSE PIN:");Serial.println(pin);
    pin = pin>=48?pin-48:pin;
    uint8_t value2write;

    switch (cmd) {
        case '{':
            //snprintf((char*) buf, MAX_REPLY_LEN, "ERROR: Unknown char\n");
            //m_uart_service_ptr->writeString((char*)buf);
            break;
        case 'Y':
            value2write = cmdString[index++]-48;
            value2write = value2write>=48?value2write-48:value2write;
            writeDigital(pin,value2write!=0);
            break;

        case 'M': //pc.printf("Querying pin %u mode\n",pin);
            buf[0]=cmd;buf[1]=pin;buf[2]=pinTypes[pin];
            sendMessage(buf, 3);
            break;
            
        case 'S': // set pin mode
            mode = cmdString[index++];
            mode = mode>=48?mode-48:mode;
            group = cmdString[index++];
            if (initPin(pin, mode, group)) { // analogs are already initialized
            //if (initPin(pin, mode)) { // analogs are already initialized
                sendPinValue(pin,readPin(pin));
            }
            break;
            
        case 'D': // delta to consider value changed (as percentage [0-1] of Voltage range)
            scanned = sscanf( (char *)&cmdString[2], "%f", &fdelta);
            
            if (scanned==1 && fdelta>=0 && fdelta<=1) {
                len = sprintf((char *)buf,"DELTA%d@%f",(int)pin,fdelta);
                sendMessage(buf, len);
                changeDeltaPercent(pin, fdelta);
                /*changeDelta           ( pin,((uint16_t)cmdString[index+0]) << 8  |
                                            ((uint16_t)cmdString[index+1]) );*/
            } else {
                len = sprintf((char *)buf,"DELTA%d@ERR",(int)pin);
                sendMessage(buf, len);
            }
            break;
            
        case 'I': // sampling interval
            scanned = sscanf( (char *)&cmdString[2], "%d", &sampling);

            if (scanned==1 && sampling>=0) {
                len = sprintf((char *)buf,"SAMPL%d@%d",(int)pin,sampling);
                sendMessage(buf, len);
                changeSamplingInterval( pin, sampling);
            /*changeSamplingInterval( pin,((int)cmdString[index+0]) << 24 | 
                                        ((int)cmdString[index+1]) << 16 | 
                                        ((int)cmdString[index+2]) << 8  |
                                        ((int)cmdString[index+3]) );*/
            } else {
                len = sprintf((char *)buf,"SAMPL%d@ERR",(int)pin);
                sendMessage(buf, len);
            }
            break;
            
        case 'G': //pc.printf("Reading pin %u\n",pin);
            switch (pinTypes[pin]) {
                case PIN_INPUT:
                case PIN_ANALOG:
                    sendPinValue(pin,readPin(pin));
                    break;
                case PIN_OUTPUT: // TODO: send warning pin not readable (is an output)
                default:  // TODO: send warning pin not initialized
                    buf[0]=PIN_NOTSET;buf[1]=PIN_NOTSET;buf[2]=PIN_NOTSET;
                    sendMessage(buf, 3);
                    break; 
            }
            break; 
            
        case 'T':
            switch (pinTypes[pin]) {
                case PIN_OUTPUT:
                    value2write = cmdString[index++];
                    writeDigital(pin,value2write!=0);
                    sendPinValue(pin,readPin(pin));
                    break;
                case PIN_INPUT: // TODO: send warning pin not writable (is an input) 
                case PIN_ANALOG: // TODO: send warning pin not writable (is an input) 
                default:  // TODO: send warning pin not initialized
                    buf[0]='T';buf[1]='T';buf[2]='T';
                    sendMessage(buf, 3);
                    break; 
            }
            break; 
        case 'R':
            // pin variable contains the group, not the pin number
            sendGroup(pin);
            break;
        default:
            // echo received buffer
            sendMessage(&cmdString[startOffset], strlen((char *)&cmdString[startOffset]));
            break;
    }   
}


void triggerSensor(){
    triggerSensorPolling=true;
}

static bool toggleled=false;

//Timer interrupt
ISR(TIMER1_OVF_vect)        // interrupt service routine
{
  /*if (!ble.isConnected()) {
    if (sandroideDebug) {
    
    
  }*/
    // per il led:
   digitalWrite(13,toggleled=!toggleled);  // cambio stato led
  
   // per scrivere sulla seriale un messaggio ad ogni interrupt:
   //cnt1++;                  // conteggio numero interrupt ricevuti 
   triggerSensorPolling = true;            // notifico che è scattato l'interrupt 

   // resetto il registro del timer per far ripartire il conteggio dei cicli di clock
   TCNT1 = timer1_counter;
}


// Sandroide initialization
// To be called from the Arduino "setup" function
void setupSandroide(bool debug) {

  sandroideDebug=debug;
  
  // Bluefruit Board Led is used to show sampling: 
  // Led state is toggled at each GCD sampling interval
  pinMode(13, OUTPUT);

  /* Initialise Bluefruit BLE module */
  if (sandroideDebug) { Serial.print(F("Initialising the Bluefruit LE module: "));}
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  if (sandroideDebug) { Serial.println( F("OK!") );}

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    if (sandroideDebug) { Serial.println(F("Performing a factory reset: "));}
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  if (sandroideDebug) { 
    Serial.println("Requesting Bluefruit info:");
    /* Print Bluefruit information */
    ble.info();
    Serial.println(F("Please check Sandroide wiki for how to"));
    Serial.println(F("connect Bluefruit to Android: "));
    Serial.println(F("https://github.com/SAndroidEOfficial/framework/wiki/Using-Bluefruit-32u4-with-SandroidE"));
    Serial.println();
  }

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  // Initializing pin arrays with default values
  for (int i=0;i<maxPins;i++) {
      pinTypes[i] = PIN_NOTSET;
      prevValues[i] = 0;
      pinSamplingIntervals[i] = -1;
      pinTimers[i]=-1;
      pinDelta[i]=DEFAULT_DELTA;
      pinGroups[i]=NO_GROUP;
  }
  
  // Initializing timer interrupt used for pin sampling
  TCCR1A = 0;
  TCCR1B = 0;
  timer1_counter = 34286;   // preload timer 65536-8MHz/256/1Hz

  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
}

// Run Sandroide loop
// To be called from the Arduino "loop" function
void loopSandroide(){
  // Read pins if sampling interval is expired and send them via bluetooth if they are changed
  if (recalcTimer) { 
    // a new sampling interval configuration is arrived from Bluetooth/Smartphone... 
    // Let's recalculate GCD ..
     if (sandroideDebug) { Serial.println("RECALC TIMER");}
     recalcTimer =false;
     calc_timer_interval();
  } else if (triggerSensorPolling) {
      triggerSensorPolling = false;
      check_pin_changed();
  } 
  
  // Check for incoming messages from Smartphone and process them
  ble.println("AT+BLEUARTRX");
  ble.readline();
  if (strcmp(ble.buffer, "OK") == 0) {
    // no data
    return;
  }
  // Some data was found, its in the buffer
  if (sandroideDebug) { Serial.print(F("[Recv] ")); Serial.println(ble.buffer); }
  parseRedBearCmd(ble.buffer);
  
  ble.waitForOK();
}


