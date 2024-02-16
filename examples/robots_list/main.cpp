#include "gladiator.h"
Gladiator* gladiator;
void reset();
void setup() {
    //instanciation de l'objet gladiator
    gladiator = new Gladiator();
    //enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset); // GFA 4.4.1
}

void reset() {
    //fonction de reset:
    //initialisation de toutes vos variables avant le début d'un match
    gladiator->log("Call of reset function"); // GFA 4.5.1
}

void loop() {
    if(gladiator->game->isStarted()) { //tester si un match à déjà commencer
        //code de votre stratégie
        RobotList robotList = gladiator->game->getPlayingRobotsId(); // GFA 4.3.5
        gladiator->log("Robots playing : %d, %d, %d, %d ---", robotList.ids[0], robotList.ids[1], robotList.ids[2], robotList.ids[3]); // GFA 4.5.1
    }
    delay(1000);
}