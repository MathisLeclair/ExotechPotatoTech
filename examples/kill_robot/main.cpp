
/*
* CODE DE TEST, PERMETTANT DE TESTER LE CONTROL DU ROBOT ET D'ATTAQUER UN ENEMI.
*/
#include <gladiator.h>


Gladiator* gladiator;

void reset();


void setup() {
  gladiator = new Gladiator();
  //gladiator->game->enableFreeMode(RemoteMode::OFF, {1.5,1.5,0.1});
  gladiator->game->onReset(&reset); //GFA 3.4.1
}

void reset() {
  gladiator->log("Game reset successfully"); //GFA 4.5.1
}


void loop() {
  // Faire en sorte que les deux robots se croisent pour que l'un explose l'autre
  if(gladiator->game->isStarted()) { //GFA 3.3.1
    RobotData data = gladiator->robot->getData(); //GFA 4.6.1
    if (data.position.x < 0.300) {
      while(data.position.x < 1.500) {
        data = gladiator->robot->getData(); // 4.6.1
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, 0.6); //GFA 4.2.1
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 0.6); //GFA 4.2.1
        delay(30);
      }
      gladiator->control->setWheelSpeed(WheelAxis::LEFT, 0); //GFA 4.2.1
      gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 0); //GFA 4.2.1
    }else if (data.position.y < 0.300){
      delay(10000);
      while(data.position.y < 1.880) {
        data = gladiator->robot->getData(); // 4.6.1
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, 1); //GFA 4.2.1
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 1); //GFA 4.2.1
        delay(30);
      }
      gladiator->control->setWheelSpeed(WheelAxis::LEFT, 0); //GFA 4.2.1
      gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 0); //GFA 4.2.1

    }
    
  }
  
  
}

