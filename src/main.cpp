#include "gladiator.h"
#include <cmath>

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

inline bool aim(Gladiator *gladiator, const Vector2 &target, bool showLogs)
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
    else
    {
        float K1 = .7;
        float K2 = 1.5;

        // rotate
        rightCommand = angleError * abs(angleError) * K1;
        leftCommand = -angleError * abs(angleError) * K1;

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
    if (aim(gladiator, pathToAim, false))
        path++;
    return false;
}

bool isDangerous(int i, int j)
{
    float size = gladiator->maze->getSize();
    if (i > size * 4 || j > size * 4 || i < (12 - size * 4) || j < (12 - size * 4))
        return true;
    return false;
}

int destX, destY = -1;
void setRandomDestination()
{
    gladiator->log("Searching for destionation...");
    destX = rand() % 12;
    destY = rand() % 12;
    while (destX == -1 || destY == -1 || isDangerous(destX, destY))
    {
        destX = rand() % 12;
        destY = rand() % 12;
    }
    gladiator->log("New destination: x:%d y:%d", destX, destY);
}

void setBestDestination()
{
    // First try to get coin
    const Coin *coin = *coins;
    while (coin != nullptr)
    {
        if (coin->value == 1)
        {
            gladiator->log("Going to coin: x:%f y:%f", coin->p.x, coin->p.y);
            destX = (int)(coin->p.x * 4);
            destY = (int)(coin->p.y * 4);
            gladiator->log("Going to space: x:%d y:%d", destX, destY);
            break;
        }
        coin++;
    }
    // If no coin, random
}

bool changeDest = false;
bool handleDanger = false;
Vector2 safePos{1.5, 1.5};
void checkDanger()
{
    const MazeSquare *robSquare = gladiator->maze->getNearestSquare();
    if (robSquare == nullptr)
    {
        gladiator->log("Robot square outside of map");
        handleDanger = true;
        return;
    }
    if (isDangerous(robSquare->i, robSquare->j))
    {
        gladiator->log("Robot is in danger");
        handleDanger = true;
    }
    else
        handleDanger = false;
}

int loopTick = 0;
void reset()
{
    initiated = false;
    loopTick = 0;
    path = q + 143;
    *path = nullptr;
}

void initialize()
{
    fillMap();
    setBestDestination();
    findPath(destX, destY);
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
    loopTick++;
    if (gladiator->game->isStarted())
    {
        if (!initiated)
            initialize();

        // Check if bot is in danger
        // This can toggle handleDanger or changeDest
        if (loopTick % 50 == 0)
            checkDanger();

        if (handleDanger)
        {
            aim(gladiator, safePos, false);
        }
        else
        {
            // Set new dest if requested
            // This can be set by followPath or handleDanger
            if (changeDest)
            {
                gladiator->log("Changing destination");
                setRandomDestination();
                findPath(destX, destY);
                changeDest = false;
            }
            // New destination if destination reached
            changeDest = followPath();
        }
    }
    // delay(10); // boucle à 100Hz
}