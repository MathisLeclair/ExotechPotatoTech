#include "gladiator.h"
#include <cmath>
#include <chrono>
// x,y représentent des coordonnées en m
// Vector{1.5,1.5} représente le point central
// Pour convertir une cordonnée de cellule (i,j) (0<=i<=13, 0<=j<=13) :
// x = i * CELL_SIZE + 0.5*CELL_SIZE
// y = j * CELL_SIZE + 0.5*CELL_SIZE
// avec CELL_SIZE = 3.0/14 (~0.214)
class Vector2
{
public:
    Vector2() : _x(0.), _y(0.) {}
    Vector2(float x, float y) : _x(x), _y(y) {}

    float norm1() const { return abs(_x) + abs(_y); }
    float norm2() const { return std::sqrt(_x * _x + _y * _y); }
    void normalize()
    {
        _x /= norm2();
        _y /= norm2();
    }
    Vector2 normalized() const
    {
        Vector2 out = *this;
        out.normalize();
        return out;
    }

    Vector2 operator-(const Vector2 &other) const { return {_x - other._x, _y - other._y}; }
    Vector2 operator+(const Vector2 &other) const { return {_x + other._x, _y + other._y}; }
    Vector2 operator*(float f) const { return {_x * f, _y * f}; }

    bool operator==(const Vector2 &other) const { return abs(_x - other._x) < 1e-5 && abs(_y - other._y) < 1e-5; }
    bool operator!=(const Vector2 &other) const { return !(*this == other); }

    float x() const { return _x; }
    float y() const { return _y; }

    float dot(const Vector2 &other) const { return _x * other._x + _y * other._y; }
    float cross(const Vector2 &other) const { return _x * other._y - _y * other._x; }
    float angle(const Vector2 &m) const { return std::atan2(cross(m), dot(m)); }
    float angle() const { return std::atan2(_y, _x); }

private:
    float _x, _y;
};

using namespace std;

Gladiator *gladiator;

const MazeSquare *maze[12][12];
const Coin *coins[42];

bool initiated = false;
long timestamp;

void fillMap()
{
    const Coin **c = coins;
    for (int x = 0; x < 12; x++)
    {
        for (int y = 0; y < 12; y++)
        {
            maze[x][y] = gladiator->maze->getSquare(x, y);
            if (maze[x][y]->coin.value == 1)
            {
                *c = &(maze[x][y]->coin);
                c++;
            }
        }
    }
    *c = nullptr;
}

const MazeSquare *history[12][12];
void clearHistory()
{
    for (int x = 0; x < 12; x++)
        for (int y = 0; y < 12; y++)
            history[y][x] = NULL;
}

inline float moduloPi(float a) // return angle in [-pi; pi]
{
    return (a < 0.0) ? (std::fmod(a - M_PI, 2 * M_PI) + M_PI) : (std::fmod(a + M_PI, 2 * M_PI) - M_PI);
}

inline bool aim(Gladiator *gladiator, const Vector2 &target, bool showLogs, bool emergency)
{
    constexpr float POS_REACHED_THRESHOLD = 0.08;

    auto posRaw = gladiator->robot->getData().position;

    Vector2 pos{posRaw.x, posRaw.y};

    Vector2 posError = target - pos;

    float targetAngle = posError.angle();
    float angleError = moduloPi(targetAngle - posRaw.a);

    bool targetReached = false;
    float leftCommand = 0.f;
    float rightCommand = 0.f;

    if (posError.norm2() < POS_REACHED_THRESHOLD) //
    {
        targetReached = true;
    }
    else if (posError.norm2() > 0.8)
    {
        gladiator->log("angle: %f", angleError);
        if (abs(angleError) < 0.3)
        {
            rightCommand += .8;
            leftCommand += .8;
        }
        else
        {
            // float K = .7;
            float K = .2;
            rightCommand = angleError * K;
            leftCommand = -angleError * K;
        }
        gladiator->log("rightCommand: %f", rightCommand);
        gladiator->log("leftCommand: %f", leftCommand);
    }
    else
    {
        // float K1 = .7;
        // float K2 = 1.5;
        float K1 = .4;
        float K2 = 1;

        // rotate
        rightCommand = angleError * K1;
        leftCommand = -angleError * K1;

        float factor = posError.norm2() * K2;
        rightCommand += factor + .1; //+angleError*0.1  => terme optionel, "pseudo correction angulaire";
        leftCommand += factor + .1;  //-angleError*0.1   => terme optionel, "pseudo correction angulaire";
    }

    gladiator->control->setWheelSpeed(WheelAxis::LEFT, leftCommand);
    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, rightCommand);

    if (showLogs || targetReached)
    {
        // gladiator->log("ta %f, ca %f, ea %f, tx %f cx %f ex %f ty %f cy %f ey %f", targetAngle, posRaw.a, angleError, target.x(), pos.x(), posError.x(), target.y(), pos.y(), posError.y());
    }

    return targetReached;
}

const MazeSquare *q[144];
const MazeSquare **path;
void findPath(int x, int y)
{
    clearHistory();
    const MazeSquare *start = gladiator->maze->getNearestSquare();
    const MazeSquare *square;
    const MazeSquare *depopSquare;
    const MazeSquare *goal = maze[x][y];

    const MazeSquare **qstart = q;
    const MazeSquare **qend = q + 1;
    *q = start;
    while (qstart != qend)
    {
        depopSquare = *qstart;
        qstart++;
        square = depopSquare->eastSquare;
        if (square && !history[square->i][square->j])
        {
            *qend = square;
            qend++;
            history[square->i][square->j] = depopSquare;
            if (square == goal)
                break;
        }
        square = depopSquare->westSquare;
        if (square && !history[square->i][square->j])
        {
            *qend = square;
            qend++;
            history[square->i][square->j] = depopSquare;
            if (square == goal)
                break;
        }
        square = depopSquare->southSquare;
        if (square && !history[square->i][square->j])
        {
            *qend = square;
            qend++;
            history[square->i][square->j] = depopSquare;
            if (square == goal)
                break;
        }
        square = depopSquare->northSquare;
        if (square && !history[square->i][square->j])
        {
            *qend = square;
            qend++;
            history[square->i][square->j] = depopSquare;
            if (square == goal)
                break;
        }
    }

    path = q + 143;
    *path = nullptr;
    if (qstart != qend)
    {
        while (square != start)
        {
            --path;
            *path = square;
            square = history[square->i][square->j];
        }
    }
}

bool followPath()
{
    if (*path == nullptr)
        return true;
    float x = ((float)((*path)->i) + 0.5) / 4.0;
    float y = ((float)((*path)->j) + 0.5) / 4.0;
    Vector2 pathToAim{x, y};
    if (aim(gladiator, pathToAim, false, false))
        path++;
    return false;
}

uint64_t timeSinceEpochMillisec()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

bool squareIsOutsideOfMap(int i, int j)
{
    long actualTime = timeSinceEpochMillisec();
    int reduc = (actualTime - timestamp) / 20000;

    if (i > 12 - reduc || j > 12 - reduc || i < reduc || j < reduc)
        return true;
    return false;
}

bool isDangerous(int i, int j)
{
    return squareIsOutsideOfMap(i, j);
}

bool pathIsDangerous()
{
    const MazeSquare **p = path;
    while (*p != nullptr)
    {
        if (isDangerous((*p)->i, (*p)->j))
        {
            gladiator->log("See this");
            return true;
        }
        p++;
    }
    return false;
}

int destX, destY = -1;
void setRandomPath()
{
    gladiator->log("Searching for destination...");
    destX = rand() % 12;
    destY = rand() % 12;
    while (destX == -1 || destY == -1)
    {
        destX = rand() % 12;
        destY = rand() % 12;
        findPath(destX, destY);
        if (isDangerous(destX, destY) || pathIsDangerous())
            destX = -1;
    }
    gladiator->log("New destination: x:%d y:%d", destX, destY);
}

bool setCoinPath()
{
    // First try to get coin
    const Coin **c = coins;
    while (*c != nullptr)
    {
        if ((*c)->value == 1)
        {
            destX = (int)((*c)->p.x * 4);
            destY = (int)((*c)->p.y * 4);
            findPath(destX, destY);
            if (isDangerous(destX, destY) || pathIsDangerous() || *path == nullptr)
            {
                (*c)++;
                continue;
            }
            gladiator->log("Going to coin: x:%f y:%f", (*c)->p.x, (*c)->p.y);
            gladiator->log("Going to space: x:%d y:%d", destX, destY);
            break;
        }
        c++;
    }
    // If no coin, random
    if ((*c) == nullptr)
        return false;
    return true;
}

void setBestPath()
{
    if (!setCoinPath())
        setRandomPath();
}

bool changeDest = false;
bool handleDanger = false;
Vector2 safePos{1.5, 1.5};
void checkDanger()
{
    const MazeSquare *robSquare = gladiator->maze->getNearestSquare();
    // OOB
    if (robSquare == nullptr)
    {
        gladiator->log("Robot square outside of map");
        handleDanger = true;
        changeDest = true;
        return;
    }
    // OOM
    if (isDangerous(robSquare->i, robSquare->j))
    {
        gladiator->log("Robot is in danger");
        handleDanger = true;
        changeDest = true;
        return;
    }

    handleDanger = false;

    if (pathIsDangerous())
    {
        gladiator->log("Path is in danger");
        changeDest = true;
    }
}

// RobotData allyData = RobotData{}
// void getNearestEnemy()
// {
//     RobotData data = gladiator->robot->getData();
//     RobotList l = gladiator->game->getPlayingRobotsId();
//     for (int i = 0; i < 4; i++)
//     {
//         if (l.ids[i] == 0)
//             continue;
//         RobotData enemy = gladiator->game->getOtherRobotData(l.ids[i]);
//         // Skip dead enemies and teammates
//         if (enemy.lifes == 0 || enemy.teamId == data.teamId)
//             continue;
//         if (!enemyExists)
//         {
//             enemyExists = true;
//             enemyData = enemy;
//         }
//         else
//         {

//         }
//     }
// }

uint64_t tick = 0;
void reset()
{
    tick = 0;
    timestamp = timeSinceEpochMillisec();
    initiated = false;
    path = q + 143;
    *path = nullptr;
}

void initialize()
{
    fillMap();
    setBestPath();
    initiated = true;
}

void setup()
{
    // instanciation de l'objet gladiator
    gladiator = new Gladiator();
    // enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset);
    // gladiator->game->enableFreeMode(RemoteMode::ON);
}

enum Strat
{
    OOB,
    DEST,
    ATTACK
};

void loop()
{
    if (gladiator->game->isStarted())
    {
        if (!initiated)
            initialize();
        tick++;

        // Check if bot is in danger
        // This can toggle handleDanger or changeDest
        if (tick % 50 == 0)
            checkDanger();

        if (handleDanger)
        {
            Position pos = gladiator->robot->getData().position;
            if (tick % 500 == 0)
                gladiator->log("Going to center: %f %f", pos.x, pos.y);
            aim(gladiator, safePos, false, true);
        }
        else
        {
            // Set new dest if requested
            // This can be set by followPath or handleDanger
            if (changeDest)
            {
                gladiator->log("Changing destination");
                setBestPath();
                changeDest = false;
            }
            // New destination if destination reached
            changeDest = followPath();
            // gladiator->control->setWheelSpeed(WheelAxis::LEFT, 0.0);
            // gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 0.0);
        }
    }
    // delay(1); // boucle à 100Hz
}