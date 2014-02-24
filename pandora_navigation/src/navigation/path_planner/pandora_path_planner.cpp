#include "navigation/path_planner/pandora_path_planner.h"

//~ PandoraPathPlanner::PandoraPathPlanner(void):  partitionGraphPathGenerator( &(_mapUpdater.getMapAttributes() ) ),
											   //~ treePathGenerator( &(_mapUpdater.getMapAttributes() ) ),
											   //~ voronoiPathGenerator( &(_mapUpdater.getMapAttributes() ) )
//~ {
	//~ mapAttributes = & (_mapUpdater.getMapAttributes() );
	//~ 
	//~ _mapUpdater.startSubscriber();
	
//~ }

PandoraPathPlanner::PandoraPathPlanner(MapAttributes* mapAttr): partitionGraphPathGenerator(mapAttr,&voronoi), treePathGenerator(mapAttr,&voronoi), voronoiPathGenerator(mapAttr,&voronoi), voronoi(mapAttr,false) {
	
	mapAttributes = mapAttr;
	
}


bool PandoraPathPlanner::makePlan( const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan ){
	
	voronoi.fixVoronoi();
	
	plan.clear();
	
	//move with partitionGraph
	ROS_INFO_NAMED("path_planner", "[path_planner %d]: Calculating path with partition graph ", __LINE__);
	if ( !partitionGraphPathGenerator.generatePath( goal, plan ) ){
		ROS_INFO_NAMED("path_planner", "[path_planner %d]: Partition path generator CAN NOT find path ", __LINE__);
		
		
		//move with RRT tree
		ROS_INFO_NAMED("path_planner", "[path_planner %d]: Alternative: move with  RRT tree ", __LINE__);
		std::cout<<"Alternative: move with  RRT tree\n";
		if ( !treePathGenerator.generatePath( goal, plan) ){
			ROS_INFO_NAMED("path_planner", "[path_planner %d]: RRT tree path generator CAN NOT find path ", __LINE__);
			
			
			//move with Voronoi
			ROS_INFO_NAMED("path_planner", "[path_planner %d]: Final solution: move with Voronoi ", __LINE__);
			if ( !voronoiPathGenerator.generatePath( goal, plan) ){
				ROS_INFO_NAMED("path_planner", "[path_planner %d]: voronoiPathGenerator CAN NOT find path ", __LINE__);
				ROS_ERROR_NAMED("path_planner", "[path_planner %d]: RUN FOR YOUR LIVES!!! ", __LINE__);
				
				plan.clear();
				return false;
			}
			else
				ROS_INFO_NAMED("path_planner", "[path_planner %d]: Path found with Voronoi! ", __LINE__);
		}
		else
			ROS_INFO_NAMED("path_planner", "[path_planner %d]: Path found with RRT Tree! ", __LINE__);
	}
	else
		ROS_INFO_NAMED("path_planner", "[path_planner %d]: Path found with Partition Graph! ", __LINE__);
	
	//~ pixelPlan.clear();
	//~ for (unsigned int i=0; i, plan.size(); i++)
		//~ pixelPlan.push_back( transformPoseStamped2PixelCoords(plan[i]) );
	
	return true;
}


//to be used ONLY in TargetSelector
void PandoraPathPlanner::getTargetSelectorPaths(NodesVector* nodes){
	voronoi.fixVoronoi();
	ROS_INFO_NAMED("path_planner", "[path_planner %d]: Generating paths for multiple targets ", __LINE__);

	partitionGraphPathGenerator.generatePath( nodes );

}




