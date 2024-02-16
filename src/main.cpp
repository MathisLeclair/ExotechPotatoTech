#include "gladiator.h"
#include <cmath>
#include <list>
#undef abs

// x,y représentent des coordonnées en m
// Vector{1.5,1.5} représente le point central
// Pour convertir une cordonnée de cellule (i,j) (0<=i<=13, 0<=j<=13) :
// x = i * CELL_SIZE + 0.5*CELL_SIZE
// y = j * CELL_SIZE + 0.5*CELL_SIZE
// avec CELL_SIZE = 3.0/14 (~0.214)
using namespace std;
class Vector2
{
public:
    Vector2() : _x(0.), _y(0.) {}
    Vector2(float x, float y) : _x(x), _y(y) {}

    float norm1() const { return std::abs(_x) + std::abs(_y); }
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

    bool operator==(const Vector2 &other) const { return std::abs(_x - other._x) < 1e-5 && std::abs(_y - other._y) < 1e-5; }
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

Gladiator *gladiator;

MazeSquare maze[12][12];
MazeSquare *history[12][12];
void init()
{
    for (int x = 0; x < 12; x++)
        for (int y = 0; y < 12; y++)
            maze[y][x] = gladiator->maze->getSquare(y, x)[0];
}

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
    constexpr float ANGLE_REACHED_THRESHOLD = 0.1;
    constexpr float POS_REACHED_THRESHOLD = 0.05;

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
    else if (std::abs(angleError) > ANGLE_REACHED_THRESHOLD)
    {
        float factor = 0.2;
        if (angleError < 0)
            factor = -factor;
        rightCommand = factor;
        leftCommand = -factor;
    }
    else
    {
        float factor = 0.8;
        rightCommand = factor; //+angleError*0.1  => terme optionel, "pseudo correction angulaire";
        leftCommand = factor;  //-angleError*0.1   => terme optionel, "pseudo correction angulaire";
    }

    gladiator->control->setWheelSpeed(WheelAxis::LEFT, leftCommand);
    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, rightCommand);

    if (showLogs || targetReached)
    {
        gladiator->log("ta %f, ca %f, ea %f, tx %f cx %f ex %f ty %f cy %f ey %f", targetAngle, posRaw.a, angleError, target.x(), pos.x(), posError.x(), target.y(), pos.y(), posError.y());
    }

    return targetReached;
}

list<MazeSquare *> findPath(int x, int y)
{
    clearHistory();
    bool stop = false;
    auto currentPos = gladiator->robot->getData().position;
    MazeSquare *start = &(maze[(int)currentPos.x][(int)currentPos.y]);
    MazeSquare *square;
    MazeSquare *depopSquare;
    MazeSquare *goal = &(maze[x][y]);

    list<MazeSquare *> q;
    q.push_back(start);
    while (q.size() != 0 && stop)
    {
        depopSquare = q.front();
        q.pop_front();
        square = depopSquare->eastSquare;
        if (square && !history[square->i][square->j])
        {
            q.push_back(square);
            if (square == goal)
                break;
            history[square->i][square->j] = depopSquare;
        }
        square = depopSquare->westSquare;
        if (square && !history[square->i][square->j])
        {
            q.push_back(square);
            if (square == goal)
                break;
            history[square->i][square->j] = depopSquare;
        }
        square = depopSquare->southSquare;
        if (square && !history[square->i][square->j])
        {
            q.push_back(square);
            if (square == goal)
                break;
            history[square->i][square->j] = depopSquare;
        }
        square = depopSquare->northSquare;
        if (square && !history[square->i][square->j])
        {
            q.push_back(square);
            if (square == goal)
                break;
            history[square->i][square->j] = depopSquare;
        }
    }
    list<MazeSquare *> result;
    if (q.size() != 0)
    {
        square = goal;
        result.push_back(goal);
        while (square != start)
        {
            square = history[square->i][square->j];
            result.push_front(square);
        }
    }
    return result;
}

void followPath(list<MazeSquare *> path)
{
    if (&maze[(int)gladiator->robot->getData().position.x][(int)gladiator->robot->getData().position.y] == path.front())
    {
        path.pop_front();
        if (path.size() == 0)
            return;
    }
    float x = ((float)path.front()->i + 0.5) / 4.0;
    float y = ((float)path.front()->j + 0.5) / 4.0;
    Vector2 pathToAim{x, y};
    aim(gladiator, pathToAim, false);
}

list<MazeSquare *> path;
void reset()
{
    init();
    path = findPath(11, 11);
}

void setup()
{
    // instanciation de l'objet gladiator
    gladiator = new Gladiator();
    // enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset);
    // gladiator->game->enableFreeMode(RemoteMode::ON);
}

void loop()
{
    if (gladiator->game->isStarted())
    {
        gladiator->log("Loop start");
        followPath(path);
    }
    delay(10); // boucle à 100Hz
}