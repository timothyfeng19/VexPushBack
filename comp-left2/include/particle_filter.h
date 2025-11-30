#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <cmath>
#include <cstdlib>
#include <ctime>

// =========================
// Particle Structure
// =========================
struct Particle {
    double x;
    double y;
    double heading;
    double weight;
};

// Number of particles
const int NUM_PARTICLES = 500;

// Particle array
static Particle particles[NUM_PARTICLES];

// =========================
// Helper random functions
// =========================
double randDouble(double min, double max) {
    return min + (max - min) * ((double)rand() / RAND_MAX);
}

// =========================
// Initialize Particles
// =========================
inline void initParticles() {
    srand(time(NULL));
    for (int i = 0; i < NUM_PARTICLES; i++) {
        particles[i].x = randDouble(0, 3000);
        particles[i].y = randDouble(0, 3000);
        particles[i].heading = randDouble(0, 360);
        particles[i].weight = 1.0 / NUM_PARTICLES;
    }
}

// =========================
// Motion Update
// =========================
inline void motionUpdate(double forward, double turnDegrees) {
    for (int i = 0; i < NUM_PARTICLES; i++) {
        particles[i].heading += turnDegrees + randDouble(-2, 2);
        double rad = particles[i].heading * M_PI / 180.0;
        particles[i].x += forward * cos(rad) + randDouble(-5, 5);
        particles[i].y += forward * sin(rad) + randDouble(-5, 5);
    }
}

// =========================
// Sensor Update
// =========================
inline void sensorUpdate(double measuredDist) {
    for (int i = 0; i < NUM_PARTICLES; i++) {
        double expectedDist = hypot(3000 - particles[i].x, 1500 - particles[i].y);
        double error = fabs(measuredDist - expectedDist);
        particles[i].weight = 1.0 / (1.0 + error);
    }
}

// =========================
// Resampling
// =========================
inline void resampleParticles() {
    Particle newSet[NUM_PARTICLES];

    double weightSum = 0;
    for (int i = 0; i < NUM_PARTICLES; i++) weightSum += particles[i].weight;

    for (int i = 0; i < NUM_PARTICLES; i++) {
        double r = randDouble(0, weightSum);
        double cumulative = 0;

        for (int j = 0; j < NUM_PARTICLES; j++) {
            cumulative += particles[j].weight;
            if (cumulative >= r) {
                newSet[i] = particles[j];
                break;
            }
        }
    }

    for (int i = 0; i < NUM_PARTICLES; i++) particles[i] = newSet[i];
}

#endif
