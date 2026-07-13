//
// Created by tommy on 02/07/2026.
//

#include "Boid.h"

Boid::Boid(float startX, float startY, float startVx,
           float startVy, int group, float bias)
    : x(startX),
      y(startY),
      vx(startVx),
      vy(startVy),
      scout_group(group),
      biasval(bias) {}

void Boid::commit() {
    x = newX;
    y = newY;
    vx = newVx;
    vy = newVy;
}

float Boid::getX() const {
    return x;
}

float Boid::getY() const {
    return y;
}

float Boid::getVx() const {
    return vx;
}

float Boid::getVy() const {
    return vy;
}

int Boid::getScoutGroup() const {
    return scout_group;
}

float Boid::getBiasval() const {
    return biasval;
}

void Boid::setNewX(float x) {
    newX = x;
}

void Boid::setNewY(float y) {
    newY = y;
}

void Boid::setNewVx(float vx) {
    newVx = vx;
}

void Boid::setNewVy(float vy) {
    newVy = vy;
}

void Boid::setBiasval(float bias) {
    biasval = bias;
}
