#ifndef CONNECTION_H
#define CONNECTION_H

/*! \struct Connection
 *     \brief  Connection between two voronodes
 *     */
class Connection{
public:
	int id;//!< The id of the Connection
	unsigned int weight;//!< The weight of the Connection
	bool visited;//!< True if the COVERAGE_GAINonnection is visited

	/**
	 * @brief Construction
	 * @param i [int] : The 				id
	 * @param w [int] : The weight
	 * **/
	Connection(int i,unsigned w){
		id=i;
		weight=w;
		visited=false;
	}
};



#endif

