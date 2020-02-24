#!/usr/bin/env python
import rospy
from visualization_classes import DesiredWaypointsVisualization, RoadNetworkVisualization

if __name__ == "__main__":
    try:
        rospy.init_node("viz_pub", anonymous=True)
        waypointsviz = DesiredWaypointsVisualization()
        road_topo_viz = RoadNetworkVisualization()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
