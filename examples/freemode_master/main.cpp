#include "gladiator.h"
Gladiator* gladiator;
uint cycle = 0;
int dir = 1;
unsigned long lasttime = 0;
void reset();
void setup() {
    //instanciation de l'objet gladiator
    gladiator = new Gladiator();
    //enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset); //GFA 4.4.1
    // Changement du code PIN pour se connecter à MInotor
    gladiator->game->setMinotorPin(1234); //GFA 4.3.6
    //positionner le robot aux coordonnées de la première base (0.1; 1.6)
    Position initialPosition = {0.1, 1.6, 0};
    gladiator->game->enableFreeMode(RemoteMode::OFF,initialPosition); //GFA 4.3.2
    //lorsque le robot commencera en free mode sa position de départ sera la valeur de initalPosition
    //Seuls les encodeurs seront utilisés pour calculer la position du robot (elle sera moins précise que celle lorsque le robot est connecté à l'arène)
    //IL est toujours possible de connecter l'outil de debugage minotor au robot lorque celui ci est en free mode
    //Le free mode n'est pas utilisable en simulation

    //modifier les coéfficients du PID :
    //gladiator->control->setWheelPidCoefs(WheelAxis::LEFT, -1, 0, 0);

    cycle = 0;
}
 
void reset() {
    //fonction de reset:
    //initialisation de toutes vos variables avant le début d'un match
}

void loop() {
    if(gladiator->game->isStarted()) { // Cette fonction est inutile en free mode
        //code de votre stratégie
        //Voici les fonctions utilisables dans ce mode :
        // - Toutes les fonctions de control (contrôler le robot, voir GFA chapitre 4.2)
        // - Toutes les fonctions de robot (réccupérer les données du robot, voir GFA chapitre 4.6)
        // - Les fonction de debugs (chapitre 4.5 du GFA)
        // - Toutes les fonctions de Weapon (Contrôler l'arme du robot, voir GFA chapitre 4.8)
        if (cycle == 0) {
            delay(2000);
            cycle++;
            lasttime = millis();
        }else if (cycle == 1) {
            gladiator->control->setWheelSpeed(WheelAxis::LEFT, dir*0.7, true); //GFA 4.2.1
            gladiator->control->setWheelSpeed(WheelAxis::RIGHT, dir*0.7, true); //GFA 4.2.1
            RobotData data = gladiator->robot->getData(); // GFA 4.6.1  
            gladiator->log("vl = %f; vr = %f", data.vl, data.vr); //GFA 4.5.1

            if(millis() - lasttime > 3500) {
                cycle++;
            }
            
        }else if (cycle == 2) {
            gladiator->control->setWheelSpeed(WheelAxis::LEFT, 0, true); //GFA 4.2.1
            gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 0, true); //GFA 4.2.1
            dir*=-1;
            delay(1000);
            lasttime = millis();
            cycle--;
        }
        delay(100);

    }
}