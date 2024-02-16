#include "gladiator.h"
Gladiator *gladiator;
unsigned int cycle = 0;
int DIR = 1;
unsigned long lasttime = 0;
void reset();
void initTrajectory();
void setupTrajectory(Position pos);
// planning
Position cons;
Position lastCons;
float kw = 1;
float kv = 1;
float vlimit = 0.8;
float wlimit = 2.5;

// trajectory
const uint8_t traj_n = 40;
Position trajectory[traj_n];
uint8_t traj_from = 0;
uint8_t traj_to = 21;
uint8_t traj_start = 0;
double leash = 0.02;

long lastPositionTime = 0;
long nextPositionTime = 0;
uint8_t step = 0;
uint8_t lastStep = 0;
int moduleTime = 1200; // 1,2s
int accelTime = 400;
float vmax = 0.65;
double dirAngle = 0;

long debTime = 0;
int linesNb = 0;

float moduleWidth;
bool sentRocket = false;

bool firstWait = true;

bool shooter = false;
void setup()
{
  gladiator = new Gladiator();
  gladiator->robot->setCalibrationOffset(-0.005, 0.020, 0);
  gladiator->game->setMinotorPin(1111);
  //gladiator->game->enableFreeMode(RemoteMode::ON, {1.5,1.5,0.1});
  gladiator->game->onReset(&reset);               // GFA 3.4.1
  moduleWidth = gladiator->maze->getSquareSize(); // GFA 3.7.4
}

void reset()
{
  RobotData data = gladiator->robot->getData();
  shooter = data.position.x < 0.3;
  firstWait = true;
  sentRocket = false;
}

void loop()
{
  // gladiator->log("loop ok ");
  if (gladiator->game->isStarted())
  {
    if (shooter)
    {
      if(firstWait) {
        delay(5000);
        firstWait = false;
      } else {
        delay(200);
      }
      
      gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 0.3);
      gladiator->control->setWheelSpeed(WheelAxis::LEFT, 0.3);

      RobotData data = gladiator->robot->getData();
      uint8_t myTeam = data.teamId;
      gladiator->log("--------------------------------------");
      for (uint8_t j = 0; j < 14; j++)
      {
        String lineLog = "";
        for (uint8_t i = 0; i < 14; i++)
        {
          const MazeSquare *sq = gladiator->maze->getSquare(i, j);
          lineLog += String(sq->possession) + ";";
        }
        gladiator->log(lineLog.c_str());
      }
      gladiator->log("--------------------------------------");
      // Position pos = gladiator->robot->getData().cposition;
      // gladiator->log("%d;%f;%f;%f", millis(), pos.x, pos.y, pos.a);

      
        if (gladiator->weapon->canLaunchRocket()) {
          gladiator->weapon->launchRocket();
          gladiator->log("After launch rocket call");
        }
      
    }
    else
    {
      RobotData data = gladiator->robot->getData();
      if (data.position.y < 1.6)
      {
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 0.5);
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, 0.5);
      }
      else
      {
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 0);
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, 0);
      }
    }
  }
}