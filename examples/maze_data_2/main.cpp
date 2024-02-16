#include "gladiator.h"
Gladiator* gladiator;
bool robot1 = false;
void reset();
void setup() {
    //instanciation de l'objet gladiator
    gladiator = new Gladiator();
    //enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset); // 4.4.1
}

void reset() {
    //fonction de reset:
    //initialisation de toutes vos variables avant le début d'un match
    gladiator->log("Robot reset successfully"); //4.5.1
    Position startPosition = gladiator->robot->getData().position; // GFA 4.6.1
    //Est-ce le premier robot scanné ?
    robot1 = (startPosition.x < 0.4);
}

void loop() {
    if(gladiator->game->isStarted()) { //tester si un match à déjà commencer
        //code de votre stratégie
        gladiator->log("The game started"); // GFA 4.5.1

        // A savoir avant de lire le code ci-sessous
        // - La méthode des graphes est utilisés pour représenter le labyrinthe
        // - Cette représentation est expliquée en détails dans le GFA chapitre 4.1.6
        // - Il faut être à l'aise avec les pointeurs ;)

        //obtenir la case ou se trouve le robot
        const MazeSquare* square = gladiator->maze->getNearestSquare(); // GFA 4.7.2
        if (!robot1) {
            unsigned char i = 5, j = 10;
            square = gladiator->maze->getSquare(i, j); // GFA 4.7.1
        }
        
        gladiator->log("----- WALL i=%d; j=%d -------", square->i, square->j); // GFA 4.5.1
        //vérifier s'il a un mur au dessus
        if (square->northSquare==nullptr) {
            gladiator->log("There a TOP wall"); // GFA 4.5.1
        }
        //vérifier s'il a un mur à droite
        if (square->eastSquare==nullptr) {
            gladiator->log("There a RIGHT wall"); // GFA 4.5.1
        }
        //vérifier s'il a un mur en dessous
        if (square->southSquare==nullptr) {
            gladiator->log("There a BOTTOM wall"); // GFA 4.5.1
        }
        //vérifier s'il a un mur à gauche
        if (square->westSquare==nullptr) {
            gladiator->log("There a LEFT wall"); // GFA 4.5.1
        }
        gladiator->log("----------------------------");
        delay(200);
    }
}
