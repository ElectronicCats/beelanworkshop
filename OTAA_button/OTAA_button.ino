#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

int flag_button=1;
int flag_join=0;
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70. 00 00 00 00 00 00 0A E2
static const u1_t PROGMEM APPEUI[8] = {0x20,0x34,0x00,0xF0,0x7E,0xD5,0xB3,0x70};
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = {0x23,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}
// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = {0xB1,0x83,0xBC,0x89,0xE8,0xAF,0x9D,0x71,0xE4,0x2A,0x65,0x9D,0x2C,0xA7,0xF9,0xCA};

void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

static uint8_t mydata[] = "Boton presionado";
static osjob_t sendjob;

//Tiempo de espera entre cada transmision
const unsigned TX_INTERVAL = 30;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 3, 4},  //{DI0, DIO1, DIO2}
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            flag_join=1;
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            //Programa la siguiente transmision
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
       
        if(flag_button==1)
        {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
        Serial.println(F("Packet queued"));
        //flag_button=0;
        }
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(9600);
    Serial.println(F("Starting"));

    //Pin Button pullup
    pinMode(8,INPUT_PULLUP);

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    
    //Configuration for SubBand 7
      for (int channel=0; channel<72; ++channel) {
      LMIC_disableChannel(channel);
    }

      LMIC_enableChannel(48);
      LMIC_enableChannel(49);
      LMIC_enableChannel(50);
      LMIC_enableChannel(51);
      LMIC_enableChannel(52);
      LMIC_enableChannel(53);
      LMIC_enableChannel(54);
      LMIC_enableChannel(55);
      LMIC_enableChannel(70);

      // TTN uses SF9 for its RX2 window. This is configured in the
      // join accept message, but the LMIC library does not currently
      // process this part of the join accept yet (see Arduino-LMIC issue #20).
      LMIC.dn2Dr = DR_SF9;

      // Use a fixed data rate of SF9 (not sure if tx power is
      // actually used). SF9 is the lowest datarate that (withing the
      // TTN fair-usage-policy of 30 seconds of airtime per day)
      // allows us to send at least 4 packets every hour.
      LMIC_setDrTxpow(DR_SF7, 14);

      // Let LMIC compensate for +/- 1% clock error
      LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
  if(flag_join==1)
  {
  if(digitalRead(8)==0)
  {
   flag_button=1;
   Serial.println("boton presionado");
    }
  }
    os_runloop_once();
}
