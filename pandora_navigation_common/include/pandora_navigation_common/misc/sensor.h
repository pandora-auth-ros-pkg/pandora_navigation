#ifndef SENSOR_H
#define SENSOR_H

/*! \struct Sensor
 *     \brief Holds a Sensor information - used in Coverage
 *     
*/
class Sensor{

public:
	char sensorID; //!< Sensor ID (declared in defines.headMaxY)
	float angle;   //!< Angle of Sensor in accordance to the robot's x accordancexis
	float DOV;     //!< Distance of sensing
	float AOV;     //!< Angle of sensing
};


#endif

