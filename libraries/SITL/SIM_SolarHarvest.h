#pragma once

namespace SITL {

class SIM_SolarHarvest {

public:

	// Function Prototypes
	int runTest();

	double  estimateSolarHarvest(double roll, double pitch, double yaw, double latitude, double longitude, double elevation, double timeUTC, double solarPowerRef, int cloudOctas);

private:
	double  cloudCoverFactor(int cloudOctas, double sunElevationAngle);
	double  deg2rad(double deg);
	double  rad2deg(double rad);
	double  max(double in1, double in2);
	void    calculatezVec(double roll, double pitch, double yaw, double zVec[]);
	void    calculateSunVector(double outArr[], double latitude, double longitude, double timeUTC);
	double  calculateHarvest(double solarPowerRef, double normalSolarPower, double angleOfIncidence);

}; //class SIM_SolarHarvest

} // namespace SITL