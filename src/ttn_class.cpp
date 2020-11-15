#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <vector>
#include <Preferences.h>
#include "ttn_class.h"

#define TTN_DEBUG

TTNClass TTN;

static RTC_DATA_ATTR uint32_t LoRaMessageCount = 0;

int TTNClass::getActMethod()
{
    return actMethod;
}
void TTNClass::getArtEui(u1_t *buf)
{
    if (getActMethod() == ACT_OTAA)
        memcpy_P(buf, appEUI, 8);
}
void TTNClass::getDevEui(u1_t *buf)
{
    if (getActMethod() == ACT_OTAA)
        memcpy(buf, devEUI, 8);
}
void TTNClass::getDevKey(u1_t *buf)
{
    if (getActMethod() == ACT_OTAA)
        memcpy_P(buf, appKey, 16);
}
void TTNClass::generateLoRaDevEUI(uint8_t *pdeveui)
{
    uint8_t *p = pdeveui, dmac[6];
    int i = 0;
    esp_efuse_mac_get_default(dmac);
    // deveui is LSB, we reverse it so TTN DEVEUI display
    // will remain the same as MAC address
    // MAC is 6 bytes, devEUI 8, set first 2 ones
    // with an arbitrary value
    *p++ = 0xFF;
    *p++ = 0xFE;
    // Then next 6 bytes are mac address reversed
    for (i = 0; i < 6; i++)
    {
        *p++ = dmac[5 - i];
    }
}
bool TTNClass::begin(int sck, int miso, int mosi, int nss, int reset, int dio0, int dio1, int dio2)
{
    lmic_pins = {
        .nss = nss,
        .rxtx = LMIC_UNUSED_PIN,
        .rst = reset,
        .dio = {dio0, dio1, dio2},
    };

    spi_pins.miso = miso;
    spi_pins.mosi = mosi;
    spi_pins.nss = nss;
    spi_pins.sck = sck;
    initCount();

    if (actMethod == ACT_OTAA)
        initDevEUI();

    // SPI interface
    SPI.begin(spi_pins.sck, spi_pins.miso, spi_pins.mosi, spi_pins.nss);

    // LMIC init
    return (1 == os_init_ex((const void *)&lmic_pins));
}
void TTNClass::join()
{

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

#ifdef CLOCK_ERROR
    LMIC_setClockError(MAX_CLOCK_ERROR * CLOCK_ERROR / 100);
#endif

#if defined(CFG_eu868)

    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);   // g2-band

#elif defined(CFG_us915)

    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    // in the US, with TTN, it saves join time if we start on subband 1
    // (channels 8-15). This will get overridden after the join by
    // parameters from the network. If working with other networks or in
    // other regions, this will need to be changed.
    LMIC_selectSubBand(1);

#elif defined(CFG_au915)

    // set sub band for AU915
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/AU-global_conf.json
    LMIC_selectSubBand(1);

#endif

    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    if (singleChannel >= 0)
    {
        setSingleChannel(singleChannel);
        // Set default rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    }
    setSpreadingFactor(spreadingFactor);

    if (actMethod == ACT_ABP)
    {
        // Set static session parameters. Instead of dynamically establishing a session
        // by joining the network, precomputed session parameters are be provided.
        uint8_t appskey[sizeof(appSessionKey)];
        uint8_t nwkskey[sizeof(networkSessionKey)];
        memcpy_P(appskey, appSessionKey, sizeof(appSessionKey));
        memcpy_P(nwkskey, networkSessionKey, sizeof(networkSessionKey));

#ifdef TTN_DEBUG
        Serial.print("App Session Key: ");
        for (size_t i = 0; i < sizeof(appskey); ++i)
        {
            if (i != 0)
                Serial.print("-");
            printHex2(appskey[i]);
        }
        Serial.println("");
        Serial.print("Network Session Key: ");
        for (size_t i = 0; i < sizeof(networkSessionKey); ++i)
        {
            if (i != 0)
                Serial.print("-");
            printHex2(networkSessionKey[i]);
        }
        Serial.println("");
#endif
        LMIC_setSession(0x1, deviceAddress, nwkskey, appskey);

        // TTN uses SF9 for its RX2 window.
        LMIC.dn2Dr = DR_SF9;

        // Trigger a false joined
        TTN_callback(EV_JOINED);
    }
    else if (actMethod == ACT_OTAA)
    {
        // Make LMiC initialize the default channels, choose a channel, and
        // schedule the OTAA join
        LMIC_startJoining();

        if (singleChannel >= 0)
        {
            // LMiC will already have decided to send on one of the 3 default
            // channels; ensure it uses the one we want
            LMIC.txChnl = singleChannel;
        }
    }

    Preferences p;
    p.begin("lora", true); // we intentionally ignore failure here
    uint32_t netId = p.getUInt("netId", UINT32_MAX);
    uint32_t devAddr = p.getUInt("devAddr", UINT32_MAX);
    uint8_t nwkKey[16], artKey[16];
    bool keysgood = p.getBytes("nwkKey", nwkKey, sizeof(nwkKey)) == sizeof(nwkKey) &&
                    p.getBytes("artKey", artKey, sizeof(artKey)) == sizeof(artKey);
    p.end(); // close our prefs

    if (!keysgood)
    {
        // We have not yet joined a network, start a full join attempt
        // Make LMiC initialize the default channels, choose a channel, and
        // schedule the OTAA join
        Serial.println("No session saved, joining from scratch");
        LMIC_startJoining();
    }
    else
    {
        Serial.println("Rejoining saved session");
        LMIC_setSession(netId, devAddr, nwkKey, artKey);

        // Trigger a false joined
        TTN_callback(EV_JOINED);
    }
}

#ifdef TTN_DEBUG
void TTNClass::printHex2(unsigned v)
{
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}
#endif
void TTNClass::onEvent(ev_t event)
{
    switch (event)
    {
    case EV_JOINED:
    {
        if (singleChannel >= 0)
            forceTxSingleChannel();

        if (!adrEnabled)
        {
            LMIC_setLinkCheckMode(0); // Link check problematic if not using ADR. Must be set after join
        }

#ifdef TTN_DEBUG
        Serial.println(F("EV_JOINED"));
#endif
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
#ifdef TTN_DEBUG
        Serial.print("netid: ");
        Serial.println(netid, DEC);
        Serial.print("devaddr: ");
        Serial.println(devaddr, HEX);
        Serial.print("AppSKey: ");
        for (size_t i = 0; i < sizeof(artKey); ++i)
        {
            if (i != 0)
                Serial.print("-");
            printHex2(artKey[i]);
        }
        Serial.println("");
        Serial.print("NwkSKey: ");
        for (size_t i = 0; i < sizeof(nwkKey); ++i)
        {
            if (i != 0)
                Serial.print("-");
            printHex2(nwkKey[i]);
        }
        Serial.println();
#endif
        Preferences p;
        if (p.begin("lora", false))
        {
            p.putUInt("netId", netid);
            p.putUInt("devAddr", devaddr);
            p.putBytes("nwkKey", nwkKey, sizeof(nwkKey));
            p.putBytes("artKey", artKey, sizeof(artKey));
            p.end();
        }
        break;
    }
    case EV_TXCOMPLETE:
#ifdef TTN_DEBUG
        Serial.println(F("EV_TXCOMPLETE (inc. RX win. wait)"));
#endif
        if (LMIC.txrxFlags & TXRX_ACK)
        {
#ifdef TTN_DEBUG
            Serial.println(F("Received ack"));
#endif
            TTN_callback(EV_MSG_ACK);
        }
        if (LMIC.dataLen)
        {
#ifdef TTN_DEBUG
            Serial.print(F("Data Received: "));
            Serial.print(LMIC.dataLen);
            Serial.print(F(" bytes of payload: 0x"));
            for (int i = 0; i < LMIC.dataLen; i++)
            {
                if (LMIC.frame[LMIC.dataBeg + i] < 0x10)
                {
                    Serial.print(F("0"));
                }
                Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
            }
            Serial.println();
#endif
            TTN_callback(EV_MSG_RESPONSE);
        }
        break;
    default:
        break;
    }

    // Send message callbacks
    TTN_callback(event);
}
void TTNClass::setSingleChannel(int channel)
{
    singleChannel = channel;
    forceTxSingleChannel();
}
void TTNClass::forceTxSingleChannel()
{
    int channels = 8;
#ifdef CFG_us915
    channels = 70;
#endif
#ifdef CFG_au915
    channels = 70;
#endif
    for (int i = 0; i < channels; i++)
    {
        if (i != singleChannel)
        {
#ifdef TTN_DEBUG
            Serial.println("Disabling channel: " + String(i));
#endif
            LMIC_disableChannel(i);
        }
    }
    setSpreadingFactor(spreadingFactor);
}
void TTNClass::initDevEUI()
{
    bool needInit = true;
    for (int i = 0; i < sizeof(devEUI); i++)
        if (devEUI[i])
            needInit = false;

    if (needInit)
        generateLoRaDevEUI(devEUI);
#ifdef TTN_DEBUG
    Serial.print("DevEUI: ");
    for (int i = 0; i < sizeof(devEUI); i++)
    {
        if (i != 0)
            Serial.print("-");
        printHex2(devEUI[i]);
    }
    Serial.println();
#endif
}
void TTNClass::initCount()
{
    if (LoRaMessageCount == 0)
    {
        Preferences p;
        if (p.begin("lora", true))
        {
            LoRaMessageCount = p.getUInt("count", 0);
            p.end();
        }
    }
}
void TTNClass::setSpreadingFactor(unsigned char sf)
{
    spreadingFactor = sf;
    LMIC_setDrTxpow(sf, 14);
}
void TTNClass::setAdrEnabled(bool enabled)
{
    adrEnabled = enabled;
    LMIC_setAdrMode(enabled);
    LMIC_setLinkCheckMode(!enabled);
}
void TTNClass::setFrameCount(int count)
{
    LoRaMessageCount = count;
    LMIC_setSeqnoUp(LoRaMessageCount);
    static uint32_t lastWriteMsec = UINT32_MAX; // Ensure we write at least once
    uint32_t now = millis();
    if (now < lastWriteMsec || (now - lastWriteMsec) > 5 * 60 * 1000L)
    { // write if we roll over (50 days) or 5 mins
        lastWriteMsec = now;

        Preferences p;
        if (p.begin("lora", false))
        {
            p.putUInt("count", LoRaMessageCount);
            p.end();
        }
    }
}
void TTNClass::erasePrefs()
{
    Preferences p;
    if (p.begin("lora", false))
    {
        p.clear();
        p.end();
    }
}
void TTNClass::TTN_callback(uint8_t message)
{
    for (uint8_t i = 0; i < _lmic_callbacks.size(); i++)
    {
        (_lmic_callbacks[i])(message);
    }
}
void TTNClass::loop()
{
    os_runloop_once();
}
void TTNClass::sendMessage(uint8_t *data, uint8_t data_size, uint8_t port, bool confirmed)
{
    setFrameCount(LoRaMessageCount); // we are about to send using the current packet count

    // Check if there is not a current TX/RX job running

    if (LMIC.opmode & OP_TXRXPEND)
    {
        TTN_callback(EV_MSG_PENDING);
        return;
    }

    // Prepare upstream data transmission at the next possible time.
    // Parameters are port, data, length, confirmed
    LMIC_setTxData2(port, data, data_size, confirmed ? 1 : 0);

    TTN_callback(EV_MSG_QUEUED);
    LoRaMessageCount++;
}
void TTNClass::registerCallback(void (*callback)(uint8_t message))
{
    _lmic_callbacks.push_back(callback);
}
void TTNClass::getResponse(uint8_t *buffer, size_t len)
{
    for (uint8_t i = 0; i < LMIC.dataLen; i++)
    {
        buffer[i] = LMIC.frame[LMIC.dataBeg + i];
    }
}
void TTNClass::useABP(u4_t devAddress, u1_t *appSKey, u1_t *networkSKey)
{
    deviceAddress = devAddress;
    memcpy(&appSessionKey, appSKey, 16);
    memcpy(&networkSessionKey, networkSKey, 16);
    actMethod = ACT_ABP;
}
void TTNClass::useOTAA(u1_t *aEUI, u1_t *dEUI, u1_t *aKey)
{
    memcpy(&appEUI, aEUI, 8);
    memcpy(&devEUI, dEUI, 8);
    memcpy(&aKey, appKey, 16);
}
size_t TTNClass::responseLen()
{
    return LMIC.dataLen;
}

//LMIC_Callbacks
void onEvent(ev_t event)
{
    TTN.onEvent(event);
}
void os_getArtEui(u1_t *buf)
{
    TTN.getArtEui(buf);
}
void os_getDevEui(u1_t *buf)
{
    TTN.getDevEui(buf);
}
void os_getDevKey(u1_t *buf)
{
    TTN.getDevKey(buf);
}