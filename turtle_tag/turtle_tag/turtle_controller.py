#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robo_interfaces.msg import Turtle
from my_robo_interfaces.msg import TurtleArray
from my_robo_interfaces.srv import CaughtTurtle
from functools import partial



class TurtleController(Node):  
    def __init__(self):
        super().__init__("turtle_controller")  
        self.pose_ = None
        #self.target_x = 10.9
        #self.target_y = 2.6
        self.declare_parameter("catch_closest_turtle_first",True)

        self.turtle_to_catch = None
        self.closest_turtle_flag = self.get_parameter("catch_closest_turtle_first").value
      
        self.publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel",10)
        self.subscriber_ = self.create_subscription(Pose,"turtle1/pose",self.turtle_pose,10)
        self.turtle_array_subscriber_ = self.create_subscription(TurtleArray, "turtles_alive", self.turtle_target,10)

        self.turtle_timer =self.create_timer(0.01, self.control_loop)
        

    def turtle_target(self,msg):
        
        if len(msg.turtles)>0:
            if self.closest_turtle_flag:
                closest_turtle = None
                closest_turtle_dist = None
                for turtle in msg.turtles:
                    dist_x = turtle.x - self.pose_.x
                    dist_y = turtle.y - self.pose_.y 
                    distance = math.sqrt(dist_x*dist_x + dist_y*dist_y)
                    if closest_turtle == None or distance < closest_turtle_dist: 
                        closest_turtle = turtle
                        closest_turtle_dist = distance
                self.turtle_to_catch = closest_turtle
            else:
                self.turtle_to_catch = msg.turtles[0]
    
    def turtle_pose(self,msg):
        self.pose_ = msg
    
    def control_loop(self):
        if self.pose_ == None or self.turtle_to_catch == None:
            return 
        
        error = math.sqrt(math.pow(self.turtle_to_catch.x - self.pose_.x,2)+math.pow(self.turtle_to_catch.y - self.pose_.y,2))
        msg = Twist()
        if error > 0.5:
            #position
            msg.linear.x = 2*error

            #orientation
            goal_theta = math.atan2((self.turtle_to_catch.y - self.pose_.y),(self.turtle_to_catch.x - self.pose_.x))
            delta_theta = goal_theta - self.pose_.theta

            if delta_theta > math.pi:
                delta_theta -= 2*math.pi
            elif delta_theta < -math.pi:
                delta_theta += 2*math.pi
            
            msg.angular.z = 5*delta_theta
        
        else:
            #target reached
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.callback_caught_turtle_srv(self.turtle_to_catch.name)
            self.turtle_to_catch = None
        
        self.publisher_.publish(msg)
    
    def callback_caught_turtle_srv(self,turtle_name):
        client = self.create_client(CaughtTurtle,"turtle_caught") #type_name, service_name (MUST be same as defined in the server code),
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server to be online")

        request = CaughtTurtle.Request()
        request.name = turtle_name
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_caught_turtle,turtle_name=turtle_name))
    
    def callback_call_caught_turtle(self,future,turtle_name):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error("Turtle " + str(turtle_name)+" couldn't be caught! ")
        except Exception as e:
            self.get_logger().error("Service call failed {}".format(e))

         



 
def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()  
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()