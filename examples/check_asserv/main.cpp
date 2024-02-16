#include "gladiator.h"
Gladiator *gladiator;

float consl = 0.f;
float consr = 0.f;
int time_cons = 2000;
int last_time = 0;
float increase = 0.1;

void reset();
void setup()
{
    // instanciation de l'objet gladiator
    gladiator = new Gladiator();
    gladiator->game->enableFreeMode(RemoteMode::OFF);
    // enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset); // GFA 4.4.1
}

void reset()
{
    consl = 0.f;
    consr = 0.f;
}

void loop()
{
    if (gladiator->game->isStarted())
    {
        if (millis() - last_time > time_cons)
        {
            last_time = millis();
            consr += increase;
            consl += increase;
            gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consr);
            gladiator->control->setWheelSpeed(WheelAxis::LEFT, consl);
            
            if (abs(consl) > 0.9)
            {
                increase = -increase;
            }
        }

        RobotData data = gladiator->robot->getData();
        gladiator->log("%f %f %f %f", consl, consr, data.vl, data.vr);
        delay(20);
    }
}