#include "gladiator.h"
Gladiator* gladiator;
bool robot1 = false;
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
    gladiator->log("Robot reset successfully"); // GFA 4.5.1
    Position startPosition = gladiator->robot->getData().position; // GFA 4.6.1  
    //Est-ce le premier robot scanné ?
    robot1 = (startPosition.x < 0.4);
}

void loop() {
    
    if(gladiator->game->isStarted()) { //tester si un match à déjà commencé
        //code de votre stratégie

        //Obtenir les données du robot
        RobotData data = gladiator->robot->getData(); // GFA 4.6.1

        //si ce n'est pas le robot scanné en premier
        if (!robot1) {
            //on récupère la liste des robots actuellement dans la partie
            RobotList robotList = gladiator->game->getPlayingRobotsId(); // GFA 4.3.5
            RobotData ret {};
            for(uint8_t id : robotList.ids) {
                
                if (id!=0 && id!=data.id) {
                    //on prend celui dont l'id est différent de 0 et dont l'id est différent du robot actuel
                    ret = gladiator->game->getOtherRobotData(id); // 4.3.3
                }
            }
            if(ret.id!=0) data = ret;
        }
        
        //affichage des données du robot
        
        // position du robot (filtré)
        Position position = data.position;
        // position du robot (brut venant de la caméra)
        Position cposition = data.cposition;
        // vitesse du robot
        double vl = data.vl; // vitesse roue gauche
        double vr = data.vr; // vitesse roue droite
        //vie du robot
        unsigned char lifes = data.lifes;
        //limite en vitesse
        double speedLimit = data.speedLimit;
        //Id du robto 
        unsigned char robotId = data.id;
        //score du robot
        unsigned char score = data.score;
        //address du robot
        String macAddress = data.macAddress;

        gladiator->log("Robot id=%d; position=(%f, %f, %f); speed[vl=%f; vr=%f]; score=%d -- ", robotId, position.x, position.y, position.a, vl, vr, score); // GFA 4.5.1

        delay(500);

        
    }
}