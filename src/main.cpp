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

enum Strat
{
    NONE,
    OOB,
    GOTO,
    ESCAPE,
    ATTACK,
    SPIN,
    SHOOT
};

Strat strat = Strat::NONE;

Gladiator *gladiator;

const MazeSquare *maze[12][12];
const Coin *coins[42];

bool initiated = false;
uint64_t timestamp;
uint64_t tick = 0;
RobotData initRobotData;

Vector2 deads[4];

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

bool coinsExist()
{
    const Coin **c = coins;
    while (*c != nullptr)
    {
        if ((*c)->value)
            return true;
        c++;
    }
    return false;
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

    bool direction = true;
    if (angleError > M_PI_2)
    {
        direction = false;
        angleError -= M_PI;
    }
    else if (angleError < -M_PI_2)
    {
        direction = false;
        angleError += M_PI;
    }

    bool targetReached = false;
    float leftCommand = 0.f;
    float rightCommand = 0.f;

    if (posError.norm2() < POS_REACHED_THRESHOLD) //
    {
        targetReached = true;
    }
    else if (posError.norm2() > 1.2)
    {
        if (abs(angleError) < 0.3)
        {
            rightCommand += .8;
            leftCommand += .8;
        }
        else if (abs(angleError) > M_PI - 0.3)
        {
            rightCommand -= .8;
            leftCommand -= .8;
        }
        else
        {
            float K = .2;
            rightCommand = angleError * K;
            leftCommand = -angleError * K;
        }
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

    if (direction)
    {
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, leftCommand);
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, rightCommand);
    }
    else
    {
        // Inversed
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, rightCommand * -1.0);
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, leftCommand * -1.0);
    }

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
        strat = Strat::GOTO;
    }
    else
    {
        strat = Strat::OOB;
    }
}

void gotoPoints(bool getCoin)
{
    clearHistory();
    const MazeSquare *start = gladiator->maze->getNearestSquare();
    const MazeSquare *square;
    const MazeSquare *depopSquare;

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
            if (getCoin)
                if (square->coin.value == 1)
                    break;
            else if (square->possession != initRobotData.teamId)
                break;
        }
        square = depopSquare->westSquare;
        if (square && !history[square->i][square->j])
        {
            *qend = square;
            qend++;
            history[square->i][square->j] = depopSquare;
            if (getCoin)
                if (square->coin.value == 1)
                    break;
            else if (square->possession != initRobotData.teamId)
                break;
        }
        square = depopSquare->southSquare;
        if (square && !history[square->i][square->j])
        {
            *qend = square;
            qend++;
            history[square->i][square->j] = depopSquare;
            if (getCoin)
                if (square->coin.value == 1)
                    break;
            else if (square->possession != initRobotData.teamId)
                break;
        }
        square = depopSquare->northSquare;
        if (square && !history[square->i][square->j])
        {
            *qend = square;
            qend++;
            history[square->i][square->j] = depopSquare;
            if (getCoin)
                if (square->coin.value == 1)
                    break;
            else if (square->possession != initRobotData.teamId)
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
        strat = Strat::GOTO;
    }
    else
    {
        strat = Strat::OOB;
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
    uint64_t actualTime = timeSinceEpochMillisec();
    int reduc = (actualTime - timestamp) / 20000;

    if (i > 11 - reduc || j > 11 - reduc || i < reduc || j < reduc)
    {
        return true;
    }
    return false;
}

bool squareWithDeadBodies(int i, int j)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        if (deads[i] == Vector2(i, j))
        {
            return true;
        }
    }
    return false;
}

bool isDangerous(int i, int j)
{
    bool danger = squareIsOutsideOfMap(i, j);
    danger = squareWithDeadBodies(i, j);
    return danger;
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
        if (isDangerous(destX, destY))
            destX = -1;
    }
    gladiator->log("New destination: x:%d y:%d", destX, destY);
}

bool weFucked(const MazeSquare *robSquare)
{
    return (!robSquare->eastSquare && !robSquare->westSquare && !robSquare->southSquare && !robSquare->westSquare);
}

bool changeDest = false;
bool handleDanger = false;
Vector2 MiddlePos{1.5, 1.5};
void checkOOB()
{
    const MazeSquare *robSquare = gladiator->maze->getNearestSquare();
    // OOB
    if (robSquare == nullptr || isDangerous(robSquare->i, robSquare->j) || weFucked(robSquare))
    {
        strat = Strat::OOB;
        return;
    }
    else if (strat == Strat::OOB)
        strat = Strat::NONE;
}

Vector2 enemies[2];
float enemiesDist[2];
void getEnemies()
{
    RobotData data = gladiator->robot->getData();
    RobotList l = gladiator->game->getPlayingRobotsId();
    int a = 0;
    enemies[0] = Vector2{255, 255};
    enemies[1] = Vector2{255, 255};
    for (int i = 0; i < 4; i++)
    {
        if (l.ids[i] == 0)
            continue;
        RobotData enemy = gladiator->game->getOtherRobotData(l.ids[i]);
        // Skip dead enemies and teammates
        if (enemy.lifes == 0)
        {
            deads[i] = Vector2((int)(enemy.position.x * 4), (int)(enemy.position.y * 4));
            gladiator->log("new deads[%d][x]=%f deads[%d][y]=%f, raw: %f %f", i, deads[i].x(), i, deads[i].y(), enemy.position.x, enemy.position.y);
        }
        else if (enemy.teamId == data.teamId)
            continue;
        else
        {
            enemies[a] = Vector2{enemy.position.x, enemy.position.y};
            ++a;
        }
    }

    auto robotPos = gladiator->robot->getData().position;
    Vector2 posUs{robotPos.x, robotPos.y};

    enemiesDist[0] = (enemies[0] - posUs).norm2();
    enemiesDist[1] = (enemies[1] - posUs).norm2();
}

bool canSee(Vector2 v)
{
    return v.angle() < M_PI / 8;
}

void checkEnemies()
{
    Position robotPos = gladiator->robot->getData().position;
    Vector2 posUs{robotPos.x, robotPos.y};

    if (gladiator->weapon->canLaunchRocket() && enemiesDist[0] < .9 &&
        abs(moduloPi((enemies[0] - posUs).angle() - robotPos.a)) < M_PI / 32)
    {
        gladiator->weapon->launchRocket();
    }
    else if (gladiator->weapon->canLaunchRocket() && enemiesDist[1] < .9 &&
             abs(moduloPi((enemies[1] - posUs).angle() - robotPos.a)) < M_PI / 32)
    {
        gladiator->weapon->launchRocket();
    }
    else if (enemiesDist[0] < 0.5 || enemiesDist[1] < 0.5)
    {
        if (enemiesDist[0] < 0.17 || enemiesDist[1] < 0.17)
            strat = Strat::SPIN;
        else
            strat = Strat::ATTACK;
    }
    else
    {
        if (strat == Strat::ATTACK || strat == Strat::SPIN)
            strat = NONE;
    }
}

void reset()
{
    tick = 0;
    timestamp = timeSinceEpochMillisec() - 2000;
    initiated = false;
    path = q + 143;
    *path = nullptr;
    strat = Strat::NONE;
}

void initialize()
{
    fillMap();
    initRobotData = gladiator->robot->getData();
    gladiator->weapon->initWeapon(WeaponPin::M1, WeaponMode::SERVO);
    gladiator->weapon->setTarget(WeaponPin::M1, 27);
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

bool veryFastLoop()
{
    return !(tick % 10);
}

bool fastLoop()
{
    return !(tick % 50);
}

bool slowLoop()
{
    return !(tick % 200);
}

bool verySlowLoop()
{
    return !(tick % 500);
}

void attack(int i)
{
    auto robotPos = gladiator->robot->getData().position;
    Vector2 posUs{robotPos.x, robotPos.y};

    if (abs(moduloPi((enemies[i] - posUs).angle() - robotPos.a)) > M_PI / 4)
    {
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, (moduloPi((enemies[i] - posUs).angle() - robotPos.a)) / 3);
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, (moduloPi((enemies[i] - posUs).angle() - robotPos.a)) / 3);
    }
    else
        aim(gladiator, enemies[i], false, true);
}

// todo: esquive de missiles
// todo: esquive de morts
// todo: esquive d'allié
// todo: si angle tir almost exact, tourner tirer
// todo: opti code

void loop()
{
    if (gladiator->game->isStarted())
    {
        if (!initiated)
            initialize();
        tick++;

        if (strat != Strat::OOB && fastLoop())
            checkOOB();
        if (slowLoop())
        {
            getEnemies();
            checkEnemies();
        }

        if (strat == Strat::OOB)
        {
            if (verySlowLoop())
                checkOOB();
            aim(gladiator, MiddlePos, false, true);
        }
        else if (strat == Strat::ESCAPE)
        {
            aim(gladiator, MiddlePos, false, true);
        }
        else if (strat == Strat::SPIN)
        {
            gladiator->control->setWheelSpeed(WheelAxis::LEFT, 1);
            gladiator->control->setWheelSpeed(WheelAxis::RIGHT, -1);
        }
        else if (strat == Strat::ATTACK)
        {
            if (enemiesDist[0] < enemiesDist[1])
                attack(0);
            else
                attack(1);
        }
        else if (strat == Strat::GOTO)
        {
            // checkEscape();

            if (followPath())
                strat = Strat::NONE;
        }
        
        if (strat == Strat::NONE)
        {
            // Check if robot has rocket
            if (!gladiator->weapon->canLaunchRocket() && coinsExist())
                gotoPoints(true);
            else
                gotoPoints(false);
        }
    }
}