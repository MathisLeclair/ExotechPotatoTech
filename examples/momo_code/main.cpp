#include "gladiator.h"
Gladiator* gladiator;

float kw = 2.f;
float kv = 1.f;
float wlimit = 4.f;
float vlimit = 0.5;
float erreurPos = 0.07;

bool has_goal = false;
bool turned = false;
MazeSquare* last = nullptr;
Position goal {0, 0, 0};
uint8_t targets[4][2] = {{1,1}, {10,1}, {10,10}, {1,10}};
int i = 0;


double reductionAngle(double x)
{
    x = fmod(x + PI, 2 * PI);
    if (x < 0)
        x += 2 * PI;
    return x - PI;
}
bool cmp_s(MazeSquare* s1, MazeSquare* s2) {
    if (s1!=nullptr && s2!=nullptr) {
        if (s1->i == s2->i && s2->j == s1->j) {
            return true;
        }
    }
    return false;
}

bool turn(Position cons, Position pos) {
    double dx = cons.x - pos.x;
    double dy = cons.y - pos.y;
    double rho = atan2(dy, dx);
    gladiator->log("turn dx=%f dy=%f rho=%f d=%f cx=%f cy=%f x=%f y=%f", dx, dy, rho, reductionAngle(rho - pos.a), cons.x, cons.y, pos.x, pos.y);
    if (abs(reductionAngle(rho - pos.a)) > 0.1) {
        float consw = kw * reductionAngle(rho - pos.a);
        consw = consw > wlimit ? (consw > 0 ? 1 : -1) * wlimit : consw;

        float consvl = -gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
        float consvr = gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consvr, false); // GFA 3.2.1
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, consvl, false);  // GFA 3.2.1
        return false;
    } else {
        return true;
    }
}
bool go_to(Position cons, Position pos)
{
    double consvl, consvr;
    double dx = cons.x - pos.x;
    double dy = cons.y - pos.y;
    double d = sqrt(dx * dx + dy * dy);
    gladiator->log("goto dx=%f dy=%f d=%f cx=%f cy=%f x=%f y=%f", dx, dy, d, cons.x, cons.y, pos.x, pos.y);
    bool arrived = false;
    if (d > erreurPos)
    {
        double rho = atan2(dy, dx);
        double consw = kw * reductionAngle(rho - pos.a);

        double consv = kv * d * cos(reductionAngle(rho - pos.a));
        consw = abs(consw) > wlimit ? (consw > 0 ? 1 : -1) * wlimit : consw;
        consv = abs(consv) > vlimit ? (consv > 0 ? 1 : -1) * vlimit : consv;

        consvl = consv - gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
        consvr = consv + gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
        arrived = false;
    }
    else
    {
        consvr = 0;
        consvl = 0;
        arrived = true;
    }

    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consvr, false); // GFA 3.2.1
    gladiator->control->setWheelSpeed(WheelAxis::LEFT, consvl, false);  // GFA 3.2.1
    return arrived;
}

void reset() {
    //fonction de reset:
    //initialisation de toutes vos variables avant le début d'un match
    gladiator->log("Call of reset function"); // GFA 4.5.1
    i = 0;
    has_goal = false;
    turned =false;
}

void setup() {
    //instanciation de l'objet gladiator
    gladiator = new Gladiator();
    //enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset); // GFA 4.4.1
}


void loop() {
    if(gladiator->game->isStarted()) { //tester si un match à déjà commencer
        //code de votre stratégie
        Position myPosition = gladiator->robot->getData().position;
        
        //get 
        if(!has_goal) {
            //gladiator->log("There is no goal, define a new goal");
            const MazeSquare* square = gladiator->maze->getNearestSquare();
            MazeSquare* target = nullptr;
            /*if (square->northSquare != nullptr && !cmp_s(last, square->northSquare)) {
                target = square->northSquare;
                gladiator->log("Can go north ");
            }else if (square->eastSquare!= nullptr && !cmp_s(last, square->eastSquare)) {
                target = square->eastSquare;
                gladiator->log("Can go east ");
            }else if (square->southSquare != nullptr && !cmp_s(last, square->southSquare)) {
                target = square->southSquare;
                gladiator->log("Can go east ");
            }else if (square->westSquare != nullptr && !cmp_s(last, square->westSquare)) {
                target = square->westSquare;
                gladiator->log("Can go east ");
            }*/
            target = (MazeSquare*) gladiator->maze->getSquare(targets[i][0], targets[i][1]);
            i=(i+1)%4;
            if (target==nullptr) {
                //gladiator->log("Go nowhere, go back ! ");
                target = last;
            }

            if (target!=nullptr) {
                //get the position of the target
                float square_size = gladiator->maze->getSquareSize();
                last = (MazeSquare*) square;
                goal.x =  (target->i + 0.5) * square_size;
                goal.y = (target->j + 0.5) * square_size;
                gladiator->saveUserWaypoint(goal.x, goal.y);
                //gladiator->log("Go to new target x=%f; y=%f", goal.x, goal.y);
                has_goal = true;
            }
        } else {
            bool reached_target = false;
            if (!turned) {
                bool finish_turning = turn(goal, myPosition);
                if (finish_turning) {
                    turned = true;
                }
            }else {
                reached_target = go_to(goal, myPosition);
                delay(100);
            }

            if (reached_target)
            {
                has_goal = false;
                turned = false;
                //gladiator->log("Target reached ! ");
                delay(100);
            } 
        }
            

    }
    delay(10);
}





