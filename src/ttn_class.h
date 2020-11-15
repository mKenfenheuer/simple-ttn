#pragma once
#ifndef __TTN_CLASS_H_INCLUDED__   
#define __TTN_CLASS_H_INCLUDED__
#include <hal/hal.h>
#include <SPI.h>
#include <vector>
#include <Preferences.h>
#include "lmic.h"

#define TTN_868EU 1
#define TTN_915US 2
#define TTN_915AU 3
#define ACT_OTAA 1
#define ACT_ABP 2

#define EV_MSG_QUEUED       100
#define EV_MSG_PENDING      101
#define EV_MSG_ACK          102
#define EV_MSG_RESPONSE     103

typedef struct t_spi_pins {
    int sck;
    int miso;
    int mosi;
    int nss;
};

class TTNClass {
    public:
        bool begin(int sck, int miso, int mosi, int nss, int reset, int dio0, int dio1, int dio2);

        //ABP
        void useABP(u4_t deviceAddress, u1_t appSessionKey[16], u1_t networkSessionKey[16]);
        //OTAA
        void useOTAA(u1_t appEUI[8], u1_t devEUI[8], u1_t appKey[16]);

        //Start the network connection
        void join();

        //LMIC os callbacks
        void getArtEui (u1_t* buf);
        void getDevEui (u1_t* buf);
        void getDevKey (u1_t* buf);  
        void setSingleChannel(int channel);
        void onEvent(ev_t event);
        void setSpreadingFactor(unsigned char sf);
        void setAdrEnabled(bool enabled);
        void setFrameCount(int count);
        void erasePrefs();
        void loop();
        void sendMessage(uint8_t * data, uint8_t data_size, uint8_t port, bool confirmed);
        void registerCallback(void (*callback)(uint8_t message));
        void getResponse(uint8_t * buffer, size_t len);
        void TTN_callback(uint8_t message);

        int getActMethod();
        size_t responseLen();
        
    private:
        //Vars
        int singleChannel = -1;
        int actMethod = ACT_ABP;
        unsigned char spreadingFactor = DR_SF7;
        bool adrEnabled = false;
        bool packetQueued = false;
        bool packetSent = false;
        lmic_pinmap lmic_pins;
        t_spi_pins spi_pins;
        u4_t deviceAddress;
        u1_t appSessionKey[16];
        u1_t networkSessionKey[16];
        u1_t appEUI[8];
        u1_t devEUI[8];
        u1_t appKey[16];
        std::vector<void(*)(uint8_t message)> _lmic_callbacks;

        //Methods
        void initCount();
        void initDevEUI();
        void beginInit();
        void generateLoRaDevEUI(uint8_t *pdeveui);
        void forceTxSingleChannel();
        void setLMICFrameCount();
        void printHex2(uint hex);

};
#endif