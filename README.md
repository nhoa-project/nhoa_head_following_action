# nhoa_head_following_action 
A ROS action to perform a following of an indicated tracked person by moving the ARI's head.


## Actions implemented

* **HeadFollowing**. A client of this action can sent an *ApproachGoal* with the id of the desired person to be followed. If the target id string is equals to "-1", the action will try to follow the first person indicated in the list of *hri_msgs/IdsList* message published in the topic */humans/bodies/tracked* (if any person is detected).
