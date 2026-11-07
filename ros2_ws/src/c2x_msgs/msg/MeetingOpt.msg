# Contiguity-driven task allocation handshake (Proposal -> Commit -> Finalize,
# plus Cancel to abort), unicast from the temporary host to one participant.
int32 from_drone_id
int32 to_drone_id

float64 stamp
# 0: proposal, 1: commit, 2: cancel, 3: finalize
int8 phase
int32[] ego_ids
int32[] other_ids
geometry_msgs/Point[] ego_centers
geometry_msgs/Point[] other_centers

# Convex hulls of split task units, flattened: hull k spans points
# [offsets[k], offsets[k+1]).
int32[] ego_hull_offsets
geometry_msgs/Point[] ego_hull_points
int32[] other_hull_offsets
geometry_msgs/Point[] other_hull_points
