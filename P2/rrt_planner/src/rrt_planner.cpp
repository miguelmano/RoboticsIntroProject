
#include <rrt_planner/rrt_planner.h>

namespace rrt_planner {

    RRTPlanner::RRTPlanner(costmap_2d::Costmap2DROS *costmap, 
            const rrt_params& params) : params_(params), collision_dect_(costmap) {

        costmap_ = costmap->getCostmap();
        map_width_  = costmap_->getSizeInMetersX();
        map_height_ = costmap_->getSizeInMetersY();

        random_double_x.setRange(-map_width_, map_width_);
        random_double_y.setRange(-map_height_, map_height_);

        nodes_.reserve(params_.max_num_nodes);
    }

    bool RRTPlanner::planPath() {

        // clear everything before planning
        nodes_.clear();

        // Start Node
        createNewNode(start_, -1);

        double *p_rand, *p_new;
        Node nearest_node;

        for (unsigned int k = 1; k <= params_.max_num_nodes; k++) {

            p_rand = sampleRandomPoint();
            nearest_node = nodes_[getNearestNodeId(p_rand)];
            p_new = extendTree(nearest_node.pos, p_rand); // new point and node candidate
            if (p_new == nullptr) {
                continue;
            }
            if (!collision_dect_.obstacleBetween(nearest_node.pos, p_new)) {
                createNewNode(p_new, nearest_node.node_id);

            } else {
                continue;
            }

            if(k > params_.min_num_nodes) {
                
                if(computeDistance(p_new, goal_) <= params_.goal_tolerance){
                    return true;
                }
            }
        }

        return false;
    }

    
    int RRTPlanner::getNearestNodeId(const double* point) {
    	/**************************
	    * Implement your code here
	    **************************/
        if (nodes_.empty()) {
            ROS_ERROR("No nodes available in RRT tree.");
            return -1;  // Return an invalid ID if the tree is empty
        }

        int nearest_id = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (int i = 0; i < nodes_.size(); ++i) {
            double dist = computeDistance(nodes_[i].pos, point);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_id = i;
            }
        }

        return nearest_id;
    }

    void RRTPlanner::createNewNode(const double* pos, int parent_node_id) {
    	/**************************
	    * Implement your code here
	    **************************/

	    Node new_node;

	    // Set the position of the new node
	    new_node.pos[0] = pos[0];
	    new_node.pos[1] = pos[1];

	    // Set the parent node ID
	    new_node.parent_id = parent_node_id;

	    // Assign a unique node ID based on the current size of the nodes_ vector
	    new_node.node_id = nodes_.size();

	    // Add the new node to the tree
        nodes_.emplace_back(new_node);
        
	}


	    
    double* RRTPlanner::sampleRandomPoint() {
        /**************************
	    * Implement your code here
	    **************************/
        // Loop until a valid free space point is found that is far enough from obstacles
        do {
        rand_point_[0] = random_double_x.generate();
        rand_point_[1] = random_double_y.generate();

            // Add a buffer distance to keep nodes away from the inflated obstacles
        } while (!collision_dect_.inFreeSpace(rand_point_));

        return rand_point_;
    }



    double* RRTPlanner::extendTree(const double* point_nearest, const double* point_rand) {
    
    	/**************************
	    * Implement your code here
	    **************************/
	    
        double theta = atan2(point_rand[1] - point_nearest[1], point_rand[0] - point_nearest[0]);

        // Step towards the random point
        candidate_point_[0] = point_nearest[0] + params_.step * cos(theta);
        candidate_point_[1] = point_nearest[1] + params_.step * sin(theta);

        // Check if the new point is in free space and at a safe distance from obstacles
        if (!collision_dect_.inFreeSpace(candidate_point_)) {
            //ROS_INFO("extendTree: Point (%.2f, %.2f) is not in free space, rejecting.", candidate_point_[0], candidate_point_[1]);
            return nullptr;  // If not free space, don't extend the tree
        }

        //ROS_INFO("extendTree: Point (%.2f, %.2f) is valid, extending the tree.", candidate_point_[0], candidate_point_[1]);
        return candidate_point_;  // Return the new candidate point if valid
    }


    const std::vector<Node>& RRTPlanner::getTree() {

        return nodes_;
    }

    void RRTPlanner::setStart(double *start) {

        start_[0] = start[0];
        start_[1] = start[1];
    }

    void RRTPlanner::setGoal(double *goal) {

        goal_[0] = goal[0];
        goal_[1] = goal[1];
    }

};
