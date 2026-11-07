#include <active_perception/heading_planner.h>
#include <plan_env/raycast.h>  // complete type needed for casters_'s implicit destructor

// HeadingPlanner is never instantiated (see the header's NOTE) - its
// construction site in planner_manager.cpp is commented out even in the
// original ROS1 code. But FastPlannerManager still owns a
// unique_ptr<HeadingPlanner> member, and unique_ptr's destructor requires a
// complete type's destructor to be defined *somewhere* in the link graph -
// the original ROS1 build satisfied this by compiling the full 632-line
// heading_planner.cpp unconditionally despite it being dead code. Rather
// than port that unused implementation, only the ctor/dtor are defined here
// to satisfy the linker; every other declared method remains unimplemented,
// exactly as unreachable as it was upstream.

namespace c2_expl {

HeadingPlanner::HeadingPlanner(rclcpp::Node::SharedPtr node) {
  (void)node;
}

HeadingPlanner::~HeadingPlanner() {
}

}  // namespace c2_expl
