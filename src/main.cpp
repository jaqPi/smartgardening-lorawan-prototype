/*
Code is based on the the ABP example of the arduino-lmic library
which is licensed under the MIT license
Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
https://github.com/matthijskooijman/arduino-lmic/blob/master/examples/ttn-abp/ttn-abp.ino
*/

#include <lmic.h>
#include <hal/hal.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "LowPower.h"


#include <Credentials.h> // TODO

//#define DEBUG // toggle serial output


#ifdef DEBUG
  #define log(x) Serial.print(x);
  #define logln(x) Serial.println(x);
#else
  #define log(x)
  #define logln(x)
#endif

Adafruit_BME280 bme; // I2C, depending on your BME, you have to use address 0x77 (default) or 0x76, see below

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

const unsigned TX_INTERVAL = 900; // in seconds
const int SLEEP_CYCLES = (int) (TX_INTERVAL / 8);

// Pin mapping
const lmic_pinmap lmic_pins = {
        .nss = 10, // ulm node 10
        .rxtx = LMIC_UNUSED_PIN,
        .rst = LMIC_UNUSED_PIN,
        .dio = {4, 5, 6}, // TTN Ulm Minster node {4, 5 ,6}
};

void printValues() {
    log("Temperature = ");
    log(bme.readTemperature());
    logln(" *C");

    log("Pressure = ");

    log(bme.readPressure());
    logln(" hPa");

    log("Humidity = ");
    log(bme.readHumidity());
    logln(" %");

    logln();
}


void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
      logln(F("OP_TXRXPEND, not sending"));
    } else {
        // temp -> 2 byte
        // pressure -> 2 byte
        // humidity -> 2 byte
        // sum -> 6 byte
        byte payload[6];

        // Only needed in forced mode. Force update of BME values
        bme.takeForcedMeasurement();

        #ifdef DEBUG
          printValues();
        #endif

        // temp
        int temp = round(bme.readTemperature() * 100);
        payload[0] = highByte(temp);
        payload[1] = lowByte(temp);

        // pressure
        int pressure = round(bme.readPressure()/100);
        payload[2] = highByte(pressure);
        payload[3] = lowByte(pressure);

        // humidity
        int humidity = round(bme.readHumidity() * 100);
        payload[4] = highByte(humidity);
        payload[5] = lowByte(humidity);


        LMIC_setTxData2(1, (uint8_t*)payload, sizeof(payload), 0);

        logln(F("Packet queued"));
    }
}

void onEvent (ev_t ev) {
    log(os_getTime());
    log(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            logln(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            logln(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            logln(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            logln(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            logln(F("EV_JOINING"));
            break;
        case EV_JOINED:
            logln(F("EV_JOINED"));
            break;
        case EV_RFU1:
            logln(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            logln(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            logln(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            logln(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
                logln(F("Received ack"));
            if (LMIC.dataLen) {
                logln(F("Received "));
                logln(LMIC.dataLen);
                logln(F(" bytes of payload"));
            }

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
            }

            // Schedule next transmission to be immediately after this
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(1), do_send);

            break;
        case EV_LOST_TSYNC:
            logln(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            logln(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            logln(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            logln(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            logln(F("EV_LINK_ALIVE"));
            break;
        default:
            logln(F("Unknown event"));
            break;
    }
}

void setup() {
    #ifdef DEBUG
      Serial.begin(9600);
    #endif
    logln(F("Starting"));


    // Setup BME280, use address 0x77 (default) or 0x76
    if (!bme.begin(0x76)) {
      logln("Could not find a valid BME280 sensor, check wiring!");
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

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

#ifdef PROGMEM
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
#elif defined(CFG_us915)
    LMIC_selectSubBand(1);
#endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
