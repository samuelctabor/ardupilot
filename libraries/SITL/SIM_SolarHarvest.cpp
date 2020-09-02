#include <stdio.h>
#include <math.h>
#include <time.h>
#include "SIM_SolarHarvest.h"

namespace SITL {

int runTest()
{
    //Declare Variables
    double  roll, pitch, yaw;
    double  latitude, longitude, elevation;       
    double  solarPowerRef; 
    double  solarHarvest;
    double printTime;
    int     cloudOctas;
    int     year, month, day, hour, minute;
    struct  tm * timeinfo;
    time_t  rawtime;
    time_t  timeUTCstatic;
    

    //Input values for test
    roll            = 0;
    pitch           = 0;
    yaw             = 0;
    latitude        = 33;       //deg
    longitude       = -114.3;   //deg
    longitude       = 0;        //deg
    elevation       = 250;      //m 
    cloudOctas      = 0;    
    solarPowerRef   = 173;      // W - the harvest from the whole aircraft in 1000W/m^2 normal to the surface
    

    year            = 2020;
    month           = 07;
    day             = 22;
    hour            = 15;
    minute          = 0;
    
    time(&rawtime);                         //get current UTC
    timeinfo            = gmtime(&rawtime); // convert time from time_t to struct in UTC
    timeinfo->tm_year   = year - 1900;      // The number of years since 1900 
    timeinfo->tm_mon    = month - 1;        //month, range 0 to 11 not 1-12 so needs modifier
    timeinfo->tm_mday   = day;
    timeinfo->tm_hour   = hour ;
    timeinfo->tm_min    = minute;
    timeinfo->tm_sec    = 0;


    // do set time
    // call mktime: timeinfo->tm_wday will be set
    timeUTCstatic = mktime(timeinfo) - timezone + daylight*60*60; //timezone is time from utc in seconds, daylight is flag of if DST is on/off
    printTime = timeUTCstatic / 86400.0;    
    printf("timeinfo %f\n", printTime);     // print in days

    struct tm convertedtiminfo = *gmtime(&timeUTCstatic);
    printf("target: %d-%02d-%02d %02d:%02d:%02d\n", convertedtiminfo.tm_year + 1900, convertedtiminfo.tm_mon + 1, convertedtiminfo.tm_mday, convertedtiminfo.tm_hour, convertedtiminfo.tm_min, convertedtiminfo.tm_sec);


    
    solarHarvest = estimateSolarHarvest( roll,  pitch,  yaw,  latitude,  longitude,  elevation, timeUTCstatic, solarPowerRef, cloudOctas);
    printf("The solar estimate is %f\n", solarHarvest);
        
    return 0; // succesfully ran
}

double estimateSolarHarvest(double roll, double pitch, double yaw, double latitude, double longitude, double elevation, double timeUTC, double solarPowerRef, int cloudOctas)
{
    double solarHarvest;   
    double solarPower;

    // Calculate zVec, which is the aircraft down vector
    // I.E. The angle sunlight must travel to be normal to the solar panels.
    double zVec[3] = { 0 };
    calculatezVec(roll, pitch, yaw,  zVec);


    // Calculate the sun direction
    double sunVeccalculateSunVectorOutput[4] = { 0 }; // Array of N, E, D vector with "solarElevationAngleDeg" as final value in array    
    calculateSunVector(sunVeccalculateSunVectorOutput, latitude, longitude, timeUTC);
    double sunVec[3] = { sunVeccalculateSunVectorOutput[0],  sunVeccalculateSunVectorOutput[1],  sunVeccalculateSunVectorOutput[2] }; //N E D
    double solarElevationAngleDeg = sunVeccalculateSunVectorOutput[3];    
    // now know the sun's location N,E,D and solarElevationAngleDeg 
        
    if (solarElevationAngleDeg < 0) {
        // sun is below horizon so no sun
        solarHarvest = 0;
        return solarHarvest;
    }

    // estimate the solar intensity(direct solar insolation)
    // from Kastenand young(1989)
    // https://www.sku.ac.ir/Datafiles/BookLibrary/45/John%20A.%20Duffie,%20William%20A.%20Beckman(auth.)-Solar%20Engineering%20of%20Thermal%20Processes,%20Fourth%20Edition%20(2013).pdf footnote 3 

    double solarZenithAngleDeg = 90 - solarElevationAngleDeg;
    printf("solarZenithAngleDeg %f\n", solarZenithAngleDeg);     
            
    double airMassFactor = exp(-0.0001184 * elevation) / (cos(deg2rad(solarZenithAngleDeg)) + 0.5057 * pow(96.080 - solarZenithAngleDeg, -1.634));
    printf("airMassFactor %f\n", airMassFactor);

    //done using forumla found on https ://en.wikipedia.org/wiki/Direct_insolation    
    solarPower = 1353 * pow(0.7 , pow(airMassFactor,0.678));    
    printf("solarPower %f\n", solarPower);
    // now we know the Solar Power

/*
    if (solarPower == 0) {
        solarHarvest = 0;
        return solarHarvest;
    }
  */

    //Find the effective solar power that is normal to the solar panels
    double solarPowerFac, angleOfIncidence, normalSolarPower;
    //Dot product between the sun vector(N,E,D) and the z vector (aircraft down vector). Gives sun effective fraction
    solarPowerFac = zVec[0] * sunVec[0] + zVec[1] * sunVec[1] + zVec[2] * sunVec[2];
    printf("solarPowerFac %f\n", solarPowerFac);

    angleOfIncidence = acos(solarPowerFac);         // this assumes zVecand sunVec have magnitude of 1, 
    solarPowerFac = max(0, solarPowerFac);          // a negative power fraction indicates sun is shining on underside of wings.
    normalSolarPower = solarPowerFac * solarPower;  // W/m^2
                        
    //calculate the power generated by the solar panels in the given conditions
    solarHarvest = calculateHarvest(solarPowerRef, normalSolarPower, angleOfIncidence);
    printf("solarHarvest %f\n", solarHarvest);


    //solarHarvest = solarHarvest * cloudCoverFactor(cloudOctas, rad2deg(asin(D))); // apply cloud cover reduction

    return solarHarvest;
}


//Function to take in roll,pitch,yaw Euler angles and return the aircraft down vector.
void calculatezVec(double roll, double pitch, double yaw, double zVec[])
{
    int i, j, k; //steppers used in for loop

    //Euler (roll, pitch, yaw) To DCM (zVec)
    double   T1[3][3] = { {1, 0, 0},{0, cos(roll), -sin(roll)},{0, sin(roll), cos(roll)} };
    double   T2[3][3] = { {cos(pitch), 0, sin(pitch)},{0, 1, 0,},{-sin(pitch), 0, cos(pitch)} };
    double   T3[3][3] = { {cos(yaw), -sin(yaw), 0},{sin(yaw), cos(yaw), 0},{0,0,1} };
    double  T23[3][3] = { 0 }; //Create empty 3x3 array for result of multipliyng T2 and T3
    double    T[3][3] = { 0 }; //Create empty 3x3 array for result of multipliyng T1 and T23 (i.e. T=T1*T2*T3)   

    // multiply T23=T2*T3
    printf("T: \n");
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            for (k = 0; k < 3; k++) {
                T23[i][j] += T2[i][k] * T3[k][j];
            }
        }
    }
    // multiply T=T1*T23 (i.e. T=T1*T2*T3)
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            for (k = 0; k < 3; k++) {
                T[i][j] += T1[i][k] * T23[k][j];
            }
            printf("%f ", T[i][j]);
        }
        printf("\n");
    }
    printf("\n");

    // Calculate zVec, which is the aircraft down vector
    // I.E. The angle sunlight must travel to be normal to the solar panels.
    for (i = 0; i < 3; i++) {
        zVec[i] = T[i][2];
        printf("zVec %f\n", zVec[i]);
    }

    
}

// Functotion to calculate the sun vector
void calculateSunVector(double outArr[], double latitude, double longitude, double timeUTC)
{
    // This is based on the NOAA excel sheet https://www.esrl.noaa.gov/gmd/grad/solcalc/calcdetails.html


    double days = timeUTC / 86400.0; //convert timeUTC to days from seconds, because excel and matlab use days
    double timeOnly = fmod(days, 1); //Get fraction of current day (this ignores leap seconds encountered since epoch)
    double julianDay = days + 719529 + 1721058.5; // find number of days since start (about 4715BC), first convert to year 0 the count backwards from there.
    printf("julianDay %f\n", julianDay);
    double julianCentury = (julianDay - 2451545) / 36525;    
    double geomMeanLongSunDeg = fmod(280.46646 + julianCentury * (36000.76983 + julianCentury * 0.0003032), 360);
    double geomMeanAnomSunDeg = 357.52911 + julianCentury * (35999.05029 - 0.0001537 * julianCentury);
    double eccentEarthOrbit = 0.016708634 - julianCentury * (0.000042037 + 0.0000001267 * julianCentury);
    double sunEqOfCtr = sin(deg2rad(geomMeanAnomSunDeg)) * (1.914602 - julianCentury * (0.004817 + 0.000014 * julianCentury)) + sin(deg2rad(2 * geomMeanAnomSunDeg)) * (0.019993 - 0.000101 * julianCentury) + sin(deg2rad(3 * geomMeanAnomSunDeg)) * 0.000289;
    printf("sunEqOfCtr %f\n", sunEqOfCtr);
    double sunTrueLongDeg = geomMeanLongSunDeg + sunEqOfCtr;
    // double sunTrueAnomDeg = geomMeanAnomSunDeg + sunEqOfCtr;
    // double sunRadVectorAUs = (1.000001018 * (1 - eccentEarthOrbit * eccentEarthOrbit)) / (1 + eccentEarthOrbit * cos(deg2rad(sunTrueAnomDeg)));
    double sunAppLongDeg = sunTrueLongDeg - 0.00569 - 0.00478 * sin(rad2deg(125.04 - 1934.136 * julianCentury));
    double meanObliqEclipticDeg = 23 + (26 + ((21.448 - julianCentury * (46.815 + julianCentury * (0.00059 - julianCentury * 0.001813)))) / 60) / 60;
    double obliqCorrDeg = meanObliqEclipticDeg + 0.00256 * cos(deg2rad(125.04 - 1934.136 * julianCentury));
    // double sunRtAscenDeg = rad2deg(atan2(cos(deg2rad(obliqCorrDeg)) * sin(deg2rad(sunAppLongDeg)), cos(deg2rad(sunAppLongDeg))));// atan2 is x, y in exceland y, x in matlab
    double sunDeclinDeg = rad2deg(asin(sin(deg2rad(obliqCorrDeg)) * sin(deg2rad(sunAppLongDeg))));
    double varY = tan(deg2rad(obliqCorrDeg / 2)) * tan(deg2rad(obliqCorrDeg / 2));
    printf("varY %f\n", varY);
    double eqOfTimeMin = 4 * rad2deg(varY * sin(2 * deg2rad(geomMeanLongSunDeg)) - 2 * eccentEarthOrbit * sin(deg2rad(geomMeanAnomSunDeg)) + 4 * eccentEarthOrbit * varY * sin(deg2rad(geomMeanAnomSunDeg)) * cos(2 * deg2rad(geomMeanLongSunDeg)) - 0.5 * varY * varY * sin(4 * deg2rad(geomMeanLongSunDeg)) - 1.25 * eccentEarthOrbit * eccentEarthOrbit * sin(2 * deg2rad(geomMeanAnomSunDeg)));
    // double haSunriseDeg = rad2deg(acos(cos(deg2rad(90.833)) / (cos(deg2rad(latitude)) * cos(deg2rad(sunDeclinDeg))) - tan(deg2rad(latitude)) * tan(deg2rad(sunDeclinDeg))));
    // double solarNoonLST = (720 - 4 * longitude - eqOfTimeMin) / 1440;
    // double sunriseTimeLST = solarNoonLST - haSunriseDeg * 4 / 1440;
    // double sunsetTimeLST = solarNoonLST + haSunriseDeg * 4 / 1440;
    // double sunlightDurationMin = 8 * haSunriseDeg;
    double trueSolarTimeMin = fmod(timeOnly * 1440 + eqOfTimeMin + 4 * longitude, 1440);
    printf("trueSolarTimeMin %f\n", trueSolarTimeMin);

    double hourAngleDeg;
    if (trueSolarTimeMin / 4 < 0) {
        hourAngleDeg = trueSolarTimeMin / 4 + 180;
    }
    else {
        hourAngleDeg = trueSolarTimeMin / 4 - 180;
    }

    double solarZenithAngleDeg = rad2deg(acos(sin(deg2rad(latitude)) * sin(deg2rad(sunDeclinDeg)) + cos(deg2rad(latitude)) * cos(deg2rad(sunDeclinDeg)) * cos(deg2rad(hourAngleDeg))));
    double solarElevationAngleDeg = 90 - solarZenithAngleDeg;
    printf("solarElevationAngleDeg %f\n", solarElevationAngleDeg);

    double solarAzimuthAngleDegCWfromN;
    if (hourAngleDeg > 0) {
        solarAzimuthAngleDegCWfromN = fmod(rad2deg(acos(((sin(deg2rad(latitude)) * cos(deg2rad(solarZenithAngleDeg))) - sin(deg2rad(sunDeclinDeg))) / (cos(deg2rad(latitude)) * sin(deg2rad(solarZenithAngleDeg))))) + 180, 360);
    }
    else {
        solarAzimuthAngleDegCWfromN = fmod(540 - rad2deg(acos(((sin(deg2rad(latitude)) * cos(deg2rad(solarZenithAngleDeg))) - sin(deg2rad(sunDeclinDeg))) / (cos(deg2rad(latitude)) * sin(deg2rad(solarZenithAngleDeg))))), 360);
    }

    // Now let's calculate the vector of the sun beam in NED coordinate system
    double N = -cos(deg2rad(solarElevationAngleDeg)) * cos(deg2rad(solarAzimuthAngleDegCWfromN));
    double E = -cos(deg2rad(solarElevationAngleDeg)) * sin(deg2rad(solarAzimuthAngleDegCWfromN));
    double D = sin(deg2rad(solarElevationAngleDeg));

    printf("N %f\n", N);
    printf("E %f\n", E);
    printf("D %f\n", D);

    // now know the sun's location N,E,D and solarElevationAngleDeg 
    outArr[0] = N;
    outArr[1] = E;
    outArr[2] = D;
    outArr[3] = solarElevationAngleDeg;
}

//calculates the harvest given the sun power and direction
double calculateHarvest(double solarPowerRef, double normalSolarPower,  double angleOfIncidence)
{
    //total power is the number of cells * the cell power at 1000W / m ^ 2
    //then modified by the actual solar power / 1000W / m ^ 2
    double estimatedSolarHarvest = solarPowerRef * normalSolarPower / 1000;

    //calculate the 3 efficiencies (temperature, irradiance, incidence)
    double cellTemperatureEfficiency = 1; // not using this currently      
    double solarIrradianceEfficiency = 1; // not using this currently    

    // index of refraction modifier based on https://pvpmc.sandia.gov/modeling-steps/1-weather-design-inputs/shading-soiling-and-reflection-losses/incident-angle-reflection-losses/physical-model-of-iam/
    double indexOfRefraction = 1.35; // 1.526 for glass, 1.35 for teflon
    double glazingExtinction = 0; // 4 for glass
    double glazingThickness = 2.54e-5; // m thickness 0.002 is normal, 2.54e-5 is 1mil


    printf("normalSolarPower %f\n", normalSolarPower);
    printf("angleOfIncidence %f\n", angleOfIncidence);
    double theta_r = asin(1 / indexOfRefraction * sin(angleOfIncidence));
    double incidenceModifier;

    printf("theta_r %f\n", theta_r);

    incidenceModifier = (exp(-(glazingExtinction * glazingThickness) / cos(theta_r)) * (1 - 0.5 * (pow(sin(theta_r - angleOfIncidence), 2) / pow(sin(theta_r + angleOfIncidence), 2) + pow(tan(theta_r - angleOfIncidence), 2) / pow(tan(theta_r + angleOfIncidence), 2)))) / (exp(-glazingExtinction * glazingThickness) * (1 - pow((1 - indexOfRefraction) / (1 + indexOfRefraction), 2)));

    printf("incidenceModifier %f\n", incidenceModifier);

    double finalEfficiency = solarIrradianceEfficiency * cellTemperatureEfficiency * incidenceModifier;

    //Additional 10 % loss seems to match the data and models from Atlantik Solar
    return 0.95 * estimatedSolarHarvest * finalEfficiency;
   

}

//cloudCoverReduction Estimate the radiation fraction reaching the surface through cloud cover.
double cloudCoverFactor(int cloudOctas, double sunElevationAngle)
{
     // Based on https ://rmets.onlinelibrary.wiley.com/doi/full/10.1002/joc.2432 table 1

    


    return 0;
}

// converts angle from degrees to radians
double deg2rad(double deg)
{
    

    return (M_PI / 180) * deg;
}

// converts angle from radians to degrees
double rad2deg(double rad)
{
    

    return (180 / M_PI) * rad;
}

// returns largest of 2 doubles
double max(double in1, double in2)
{
    // returns largest of 2 doubles

    if (in1 > in2) {
        return in1;
    }
    else {
        return in2;
    }

}

} // namespace SITL