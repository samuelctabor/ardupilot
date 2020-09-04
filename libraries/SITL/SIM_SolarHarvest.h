#pragma once

namespace SITL {

// Test function
int runTest();

// Main entry point
double  estimateSolarHarvest(double roll, double pitch, double yaw, double latitude, double longitude, double elevation, double timeUTC, double solarPowerRef, int cloudOctas);

// Shell method to be completed at a later date to account for cloud cover effect.
double  cloudCoverFactor(int cloudOctas, double sunElevationAngle);

// Compute aircraft z axis in earth frame.
void    calculatezVec(double roll, double pitch, double yaw, double zVec[]);

// Calculate sun vector in earth frame.
void    calculateSunVector(double outArr[], double latitude, double longitude, double timeUTC);

// Calculate solar harvest from nominal solar power, direct normal solar power and angle of incidence.
double  calculateHarvest(double solarPowerRef, double normalSolarPower, double angleOfIncidence);

double  deg2rad(double deg);
double  rad2deg(double rad);
double  max(double in1, double in2);

} // namespace SITL
