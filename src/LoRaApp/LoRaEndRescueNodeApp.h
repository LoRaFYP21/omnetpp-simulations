//
// LoRaEndRescueNodeApp.h
// Thin subclass of LoRaEndNodeApp for specialised rescue node behaviour (placeholder).
//
#ifndef __LORA_OMNET_LORAENDRESCUENODEAPP_H_
#define __LORA_OMNET_LORAENDRESCUENODEAPP_H_

#include "LoRaEndNodeApp.h"

namespace inet {

class INET_API LoRaEndRescueNodeApp : public LoRaEndNodeApp {
  protected:
    virtual void initialize(int stage) override {
        LoRaEndNodeApp::initialize(stage);
        if (stage == INITSTAGE_LOCAL) {
            // Placeholder for rescue-node specific local init.
        }
    }
};

}

#endif
