/*
Code is based on the the ABP example of the arduino-lmic library
which is licensed under the MIT license
Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
Copyright (c) 2018 Terry Moore, MCCI
https://github.com/mcci-catena/arduino-lmic/blob/master/examples/ttn-abp/ttn-abp.ino
*/


#include <lmic.h>
#include <hal/hal.h>
#include <Wire.h>
#include <SPI.h>
#include "LowPower.h"


#ifdef BME280
    #include <Adafruit_Sensor.h>
    #include <Adafruit_BME280.h>
    Adafruit_BME280 bme; // I2C, depending on your BME, you have to use address 0x77 (default) or 0x76, see below
#endif

#include <Credentials.h>



#define SOIL_MOISTURE_SENSOR_PIN_1 A6
#define SOIL_MOISTURE_SENSOR_PIN_2 A7
#define SWITCH_PIN 9

#ifdef DEBUG
  #define print(x) Serial.print(x);
  #define println(x) Serial.println(x);
#else
  #define print(x)
  #define println(x)
#endif


void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

const unsigned TX_INTERVAL = 20 * 60; // in seconds
const int SLEEP_CYCLES = (int) (TX_INTERVAL / 8);

// Pin mapping 
const lmic_pinmap lmic_pins = {
        .nss = 10, // ulm node 10
        .rxtx = LMIC_UNUSED_PIN,
        .rst = LMIC_UNUSED_PIN,
        .dio = {4, 5, 6}, // TTN Ulm Minster node {4, 5 ,6}
};

void printValues() {
    #ifdef BME280
        print("Temperature = ");
        print(bme.readTemperature());
        println(" *C");

        print("Pressure = ");

        print(bme.readPressure());
        println(" hPa");

        print("Humidity = ");
        print(bme.readHumidity());
        println(" %");

        println();
    #endif
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
      println(F("OP_TXRXPEND, not sending"));
    } else {
        // soil moisture sensor 1   ->  2 byte
        // soil moisture sensor 2   ->  2 byte
        // temp                     ->  2 byte
        // pressure                 ->  2 byte
        // humidity                 ->  2 byte
        // =====================================
        // sum                      -> 10 byte
        #ifdef BME280
            byte payload[10];
        #else
            byte payload[4];
        #endif

        // switch on soil moisture pin and wait 3s to calibrate sensors
        digitalWrite(SWITCH_PIN, HIGH);
        delay(3000);

        // read sensors
        int soilMoisture1 = analogRead(SOIL_MOISTURE_SENSOR_PIN_1);
        delay(100);
        soilMoisture1 = analogRead(SOIL_MOISTURE_SENSOR_PIN_1);

        payload[0] = highByte(soilMoisture1);
        payload[1] = lowByte(soilMoisture1);

        int soilMoisture2 = analogRead(SOIL_MOISTURE_SENSOR_PIN_2);
        delay(100);
        soilMoisture2 = analogRead(SOIL_MOISTURE_SENSOR_PIN_2);
        payload[2] = highByte(soilMoisture2);
        payload[3] = lowByte(soilMoisture2);

        print("Sensor1: ");
        println(soilMoisture1);
        print("Sensor2: ");
        println(soilMoisture2);
        println();

        // switch of sensors
        digitalWrite(SWITCH_PIN, LOW);


        #ifdef BME280
            // Only needed in forced mode. Force update of BME values
            bme.takeForcedMeasurement();

            #ifdef DEBUG
                printValues();
            #endif

            // temp
            int temp = round(bme.readTemperature() * 100);
            payload[4] = highByte(temp);
            payload[5] = lowByte(temp);

            // pressure
            int pressure = round(bme.readPressure()/100);
            payload[6] = highByte(pressure);
            payload[7] = lowByte(pressure);

            // humidity
            int humidity = round(bme.readHumidity() * 100);
            payload[8] = highByte(humidity);
            payload[9] = lowByte(humidity);
        #endif 

        LMIC_setTxData2(1, (uint8_t*)payload, sizeof(payload), 0);

        println(F("Packet queued"));
    }
}

/**
 * Adjusts the Arduino's internal timer by the given time.
 * Code adopted from JackGruber
 * https://github.com/JackGruber/Arduino-Pro-Mini-LoRa-Sensor-Node/blob/master/src/powerdown.cpp
 */
void updateMicros(int seconds) {
    extern volatile unsigned long timer0_overflow_count;
    cli();
    // LMIC uses micros() to keep track of the duty cycle, so
    // hack timer0_overflow for a rude adjustment:  
    timer0_overflow_count+= seconds * 64 * clockCyclesPerMicrosecond();
    sei();
    os_getTime();
}

void onEvent (ev_t ev) {
    print(os_getTime());
    print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            println(F("EV_JOINED"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              println(F("Received ack"));
            if (LMIC.dataLen) {
              println(F("Received "));
              println(LMIC.dataLen);
              println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);


            // Now preparing to go into sleep mode. The LMIC library already
            // powers down the RFM95, see
            // https://www.thethingsnetwork.org/forum/t/how-to-put-rfm95-to-sleep/9427

            // Following code is adapted by
            // https://github.com/rocketscream/MiniUltraPro/blob/master/ttn-otaa-sleep.ino

            // Ensure all debugging messages are sent before sleep
            #ifdef DEBUG
              Serial.flush();
            #endif

            // Going into sleep for more than 8 s – any better idea?
            for(int i = 0; i < SLEEP_CYCLES; i++) {
              LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
              updateMicros(8);
            }

            // Schedule next transmission to be immediately after this
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(1), do_send);


            break;
        case EV_LOST_TSYNC:
            println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;
        default:
            print(F("Unknown event: "));
            println((unsigned) ev);
            break;
    }
}


void setup() {
    #ifdef DEBUG
      Serial.begin(9600);
    #endif
    println(F("Starting"));

    #ifdef BME280
        // Setup BME280, use address 0x77 (default) or 0x76
        if (!bme.begin(0x76)) {
        println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
        }
        

        // Set BME in force mode to reduce power consumption
        // force mode = measure, store results, and go into sleep mode
        // until next measurement, see
        // - http://tinkerman.cat/low-power-weather-station-bme280-moteino/
        // - https://github.com/adafruit/Adafruit_BME280_Library/blob/master/examples/advancedsettings/advancedsettings.ino
        bme.setSampling(Adafruit_BME280::MODE_FORCED,
                        Adafruit_BME280::SAMPLING_X1, // temperature
                        Adafruit_BME280::SAMPLING_X1, // pressure
                        Adafruit_BME280::SAMPLING_X1, // humidity
                        Adafruit_BME280::FILTER_OFF   );
    #endif


    // Set sensor switch port to switch on/off soil moisture sensors
    pinMode(SWITCH_PIN, OUTPUT);
    digitalWrite(SWITCH_PIN, LOW);

    pinMode(SOIL_MOISTURE_SENSOR_PIN_1, INPUT);
    pinMode(SOIL_MOISTURE_SENSOR_PIN_2, INPUT);

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set. The LMIC doesn't let you change
    // the three basic settings, but we show them here.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915) || defined(CFG_au915)
    // NA-US and AU channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #elif defined(CFG_as923)
    // Set up the channels used in your country. Only two are defined by default,
    // and they cannot be changed.  Use BAND_CENTI to indicate 1% duty cycle.
    // LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    // LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);

    // ... extra definitions for channels 2..n here
    #elif defined(CFG_kr920)
    // Set up the channels used in your country. Three are defined by default,
    // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
    // BAND_MILLI.
    // LMIC_setupChannel(0, 922100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(1, 922300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(2, 922500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

    // ... extra definitions for channels 3..n here.
    #elif defined(CFG_in866)
    // Set up the channels used in your country. Three are defined by default,
    // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
    // BAND_MILLI.
    // LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

    // ... extra definitions for channels 3..n here.
    #else
    # error Region not supported
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
