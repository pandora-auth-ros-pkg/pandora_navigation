#ifndef IDENTITY_H
#define IDENTITY_H


/*! \struct Identity
 *     \brief  Holds the identity of a node
 *     */
class Identity{
public:
	char state;//!< The state of the node (visited or not)
	int dist;//!< The distance

	/**
	 * @brief Construction
	 * @param i [char] 					: The state
	 * @param j [int] : The distance
	 * **/
	Identity(char i,int j){
		state=i;
		dist=j;
	}
};

#endif

