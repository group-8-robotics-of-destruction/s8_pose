#ifndef __POSE_NODE_H
#define __POSE_NODE_H

#include <string>

namespace s8 {
	namespace pose_node {
		const std::string NODE_NAME = 			"pose_node";

		const std::string TOPIC_POSE = 			"/s8/pose";

		enum FrontFacing {
		    NORTH = 1,
		    EAST = 1 << 1,
		    SOUTH = 1 << 2,
		    WEST = 1 << 3
		};
	}
}

#endif
