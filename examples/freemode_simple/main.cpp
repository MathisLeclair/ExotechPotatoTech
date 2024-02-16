#include "gladiator.h"
Gladiator* gladiator;
void reset();
void setup() {
    //instanciation de l'objet gladiator
    gladiator = new Gladiator();
    //enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset); // GFA 4.4.1
    gladiator->game->enableFreeMode(RemoteMode::OFF); //GFA 4.3.2
}

void reset() {
    //fonction de reset:
    //initialisation de toutes vos variables avant le début d'un match
    gladiator->log("Call of reset function"); // GFA 4.5.1
}

void loop() {
    if(gladiator->game->isStarted()) { //tester si un match à déjà commencer
        //code de votre stratégie
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, 0.5);
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, -0.5);
    }else {
       
    }
    delay(300);
}