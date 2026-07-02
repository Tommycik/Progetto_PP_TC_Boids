#ifndef PROGETTOOPENMP_BOID_H
#define PROGETTOOPENMP_BOID_H

class Boid {
private:
    // position
    float x, y;

    // velocity
    float vx, vy;

    // gruppo scout
    int scout_group;
    float biasval;

    // variabili di buffer
    float newX, newY, newVx, newVy;

public:
    Boid(float startX, float startY, float startVx, float startVy,
         int group, float bias);

    void commit();

    // getter
    float getX() const;
    float getY() const;
    float getVx() const;
    float getVy() const;
    int getScoutGroup() const;
    float getBiasval() const;

    // setter
    void setNewX(float x);
    void setNewY(float y);
    void setNewVx(float vx);
    void setNewVy(float vy);
    void setBiasval(float bias);
};

#endif // PROGETTOOPENMP_BOID_H