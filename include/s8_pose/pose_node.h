#ifndef __POSE_NODE_H
#define __POSE_NODE_H

#include <string>

namespace s8 {
	namespace pose_node {
		const std::string NODE_NAME 				= 	"pose_node";
		const std::string TOPIC_POSE 				=	"/s8/pose";
		const std::string TOPIC_POSE_SIMPLE 		= 	"/s8/pose_simple";
		const std::string SERVICE_SET_ORIENTATION 	=	"/s8/pose/setOrientation";
		const std::string SERVICE_SET_POSITION	  	=   "/s8/pose/setPosition";
		enum FrontFacing {
		    EAST = 0,
		    NORTH = 90,
		    WEST = 180,
		    SOUTH = 270
		};

		std::string to_string(FrontFacing front_facing) {
			switch(front_facing) {
				case FrontFacing::EAST: return "EAST";
				case FrontFacing::NORTH: return "NORTH";
				case FrontFacing::WEST: return "WEST";
				case FrontFacing::SOUTH: return "SOUTH";
			}
		}
	}
}

#endif
