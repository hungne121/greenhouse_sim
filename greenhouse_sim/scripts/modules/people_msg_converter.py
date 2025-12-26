#!/usr/bin/env python3
"""
People Msg Converter
Bridges leg_detector (PositionMeasurementArray) to social_navigation_layers (People)
"""
import rospy
from people_msgs.msg import PositionMeasurementArray, People, Person

class PeopleConverter:
    def __init__(self):
        rospy.init_node('people_converter')
        
        # Input from leg_detector
        self.sub = rospy.Subscriber('/leg_tracker_measurements', PositionMeasurementArray, self.callback)
        
        # Output to social_layer
        self.pub = rospy.Publisher('/people', People, queue_size=10)
        
        rospy.loginfo("[PeopleConverter] Bridge Started: /people_tracker_measurements -> /people")

    def callback(self, msg):
        people_msg = People()
        people_msg.header = msg.header
        
        for idx, pm in enumerate(msg.people):
            person = Person()
            person.name = f"human_{idx}"
            person.position = pm.pos
            person.velocity.x = 0.0 
            person.velocity.y = 0.0
            person.velocity.z = 0.0
            person.reliability = pm.reliability
            person.tagnames = pm.tagnames
            person.tags = pm.tags
            
            people_msg.people.append(person)
            
        self.pub.publish(people_msg)

if __name__ == '__main__':
    try:
        PeopleConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
