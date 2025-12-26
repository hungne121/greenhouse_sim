#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import LaserScan
from people_msgs.msg import People, Person
from geometry_msgs.msg import Point
import tf

class SimpleLegDetector:
    def __init__(self):
        rospy.init_node('simple_leg_detector')
        
        # Params
        self.fixed_frame = rospy.get_param('~fixed_frame', 'base_link')
        self.cluster_dist_thresh = 0.05  # STRICTER: 5cm max gap (prevents merging wall + leg)
        self.min_points = 4             # STRICER: Noise filtering
        self.max_width = 0.45           # STRICER: Human leg width limit (0.6 is too wide)
        
        self.listener = tf.TransformListener()
        
        # Publishers
        self.people_pub = rospy.Publisher('/people', People, queue_size=10)
        
        # Subscribers
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        
        # Tracking variables
        self.prev_people = [] # List of {'pos': (x,y), 'time': t}
        self.prev_time = None
        
        rospy.loginfo("Simple Leg Detector Started. Waiting for scan...")

    def scan_cb(self, msg):
        # 1. Convert visible points to Cartesian coordinates (in Laser frame)
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append((x, y))
            angle += msg.angle_increment
            
        if not points:
            return

        # 2. Clustering (Euclidean Distance)
        clusters = []
        if points:
            current_cluster = [points[0]]
            for i in range(1, len(points)):
                p1 = points[i-1]
                p2 = points[i]
                dist = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
                
                if dist < self.cluster_dist_thresh:
                    current_cluster.append(p2)
                else:
                    clusters.append(current_cluster)
                    current_cluster = [p2]
            clusters.append(current_cluster)

        # 3. Filter and Convert to People
        people_msg = People()
        people_msg.header = msg.header
        people_msg.header.frame_id = self.fixed_frame # We will transform
        



        # Re-implementation with transformPoint
        people_msg.people = []
        for cluster in clusters:
            if len(cluster) < self.min_points: continue
            
            # Width check
            width = math.hypot(cluster[-1][0]-cluster[0][0], cluster[-1][1]-cluster[0][1])
            if width > self.max_width: continue

            # Centroid
            cx = sum(p[0] for p in cluster) / len(cluster)
            cy = sum(p[1] for p in cluster) / len(cluster)
            
            # Transform
            # Create a PointStamped in Laser Frame
            from geometry_msgs.msg import PointStamped
            ps = PointStamped()
            ps.header = msg.header
            ps.point.x = cx
            ps.point.y = cy
            ps.point.z = 0.0
            
            try:
                ps_out = self.listener.transformPoint(self.fixed_frame, ps)
                
                # Check distance (ignore far objects)
                dist_from_robot = math.hypot(ps_out.point.x, ps_out.point.y)
                if dist_from_robot > 3.0: # Ignore > 3m
                    continue
                
                person = Person()
                person.name = "human_leg"
                person.position = ps_out.point
                person.reliability = 1.0 
                # (Optional) Velocity could be calc from tracking, but static for now
                people_msg.people.append(person)
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        
                people_msg.people.append(person)
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        
        # 4. TRACKING & VELOCITY CALCULATION
        current_time = msg.header.stamp.to_sec()
        current_people_data = [] # For next iteration
        
        if self.prev_time is not None:
            dt = current_time - self.prev_time
            if dt > 0:
                for person in people_msg.people:
                    # Find closest match in prev_people
                    best_dist = 100.0
                    best_match = None
                    
                    px, py = person.position.x, person.position.y
                    
                    for prev in self.prev_people:
                        prev_x, prev_y = prev['pos']
                        dist = math.hypot(px - prev_x, py - prev_y)
                        if dist < best_dist:
                            best_dist = dist
                            best_match = prev
                    
                    # Match found within reasonable distance (e.g. 0.5m moved in 0.1s is 5m/s - fast but possible)
                    if best_match and best_dist < 0.5:
                        vx = (px - best_match['pos'][0]) / dt
                        vy = (py - best_match['pos'][1]) / dt
                        
                        # Simple low-pass filter (optional, but 10Hz is noisy)
                        # For now, raw velocity is fine for "Moving vs Stopped" check
                        person.velocity.x = vx
                        person.velocity.y = vy
                        person.velocity.z = 0.0
                    else:
                        person.velocity.x = 0.0
                        person.velocity.y = 0.0
        
        # Update history
        for person in people_msg.people:
            current_people_data.append({
                'pos': (person.position.x, person.position.y),
                'time': current_time
            })
            
        self.prev_people = current_people_data
        self.prev_time = current_time

        if len(people_msg.people) > 0:
            self.people_pub.publish(people_msg)

if __name__ == '__main__':
    try:
        SimpleLegDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
