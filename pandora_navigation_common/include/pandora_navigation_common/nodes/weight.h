/*! \class Weight
 *     \brief Holds the weight of a voronode
 *     */
class Weight{

public:
	float angle;//!< The angle from the robot
	float dist;//!< The distance from the robot
	float relativeWallDist; 					//!< The distance from the closest wall
	float pathCoverage;//!< The 			path coverage from robot to node

	template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
			ar & angle;
			ar & dist;
			ar & relativeWallDist;
			ar & pathCoverage;

	}

};
