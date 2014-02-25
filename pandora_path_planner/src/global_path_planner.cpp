//~ #include "navigation/path_planner/global_path_planner.h"

//~ 
//~ PixelCoords GlobalPathPlanner::transformPoseStamped2PixelCoords(geometry_msgs::PoseStamped point){
	//~ //TODO:implementation
//~ }
//~ 
//~ 
//~ geometry_msgs::PoseStamped GlobalPathPlanner::transformPixelCoords2PoseStamped(PixelCoords point){
	//~ //TODO:implementation
//~ }

//TODO: edit this. needs robot pose
//~ void GlobalPathPlanner::slamMapCallback(const actionlib::SimpleClientGoalState& state, const slam_communications::slamMapResultConstPtr& result){
	//~ 
	//~ int width=result->xsize;
	//~ int height=result->ysize;
	//~ 
	//~ prevxMin=result->xmin;
	//~ prevxMax=result->xmax;
	//~ prevyMin=result->ymin;
	//~ prevyMax=result->ymax;
	//~ 
	//~ for(int i=0;i<width;i++){
		//~ for(int j=0;j<height;j++){
			//~ map[i+prevxMin][j+prevyMin]=result->slamMap[i*height+j];
		//~ }
	//~ }
//~ }

