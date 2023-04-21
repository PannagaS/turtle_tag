#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.srv import Kill

from functools import partial
import random
import math
from my_robo_interfaces.msg import Turtle 
from my_robo_interfaces.msg import TurtleArray
from my_robo_interfaces.srv import CaughtTurtle
from functools import partial



class TurtleSpawn(Node):  
    def __init__(self):
        super().__init__("turtle_spawnner")
        self.declare_parameter("spawn_frequency",1.0)
        self.spawn_frequency = self.get_parameter("spawn_frequency").value

        self.timer_ = self.create_timer(1.0/self.spawn_frequency, self.spawn_new_turtle)
        self.alive_turtles = []
        self.turtle_counter = 0
        self.turtle_name_prefix = "turtle"
        self.publish_turtles_ = self.create_publisher(TurtleArray, "turtles_alive",10)
        self.catch_turtle_srv = self.create_service(CaughtTurtle, 'turtle_caught', self.callback_caught_turtle)

        self.get_logger().info("Spawn turtle node started ... ")

    
    def callback_caught_turtle(self,request,response):
        self.callback_kill_srv(request.name)
        response.success = True
        return response

    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles
        self.publish_turtles_.publish(msg)

    def spawn_new_turtle(self):
        self.turtle_counter+=1
        self.name = self.turtle_name_prefix + str(self.turtle_counter)
        self.x_ = random.uniform(0.0,11.0)
        self.y_ = random.uniform(0.0,11.0)
        self.theta_ = random.uniform(0,2*math.pi)
           
        self.callback_spawn_srv(self.name,self.x_, self.y_, self.theta_)
        

    
        #self.server_ = self.create_service(Spawn,"Spawn",self.callback_spawn_turtle) #service_type,name_of_the_service,callback_function


    def callback_spawn_srv(self,turtle_name,x,y,theta):
        client = self.create_client(Spawn,"spawn") #type_name, service_name (MUST be same as defined in the server code),
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server to be online")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_spawn_turtle,turtle_name=turtle_name,x=x,y=y,theta=theta))
    
    def callback_call_spawn_turtle(self,future,turtle_name,x,y,theta):
        try:
            response = future.result()
            
            
            self.get_logger().info("Successfully spawned a new turtle at x:{}, y:{}, theta:{}".format(x,y,theta))
            new_turtle = Turtle()
            new_turtle.x = x
            new_turtle.y = y
            new_turtle.theta = theta
            new_turtle.name = response.name
            self.alive_turtles.append(new_turtle)
            
            self.publish_alive_turtles()

            self.get_logger().info("Return message " + str(response.name))
        except Exception as e:
            self.get_logger().error("Service call failed {}".format(e))
    
    
    
    def callback_kill_srv(self, turtle_name):
        client = self.create_client(Kill,"kill") 
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server to be online")

        request = Kill.Request()
        
        request.name = turtle_name
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_kill_turtle,turtle_name=turtle_name))
    
    def callback_call_kill_turtle(self,future,turtle_name):
        try:
            future.result()
            for (i,turtle) in enumerate(self.alive_turtles):
                if turtle.name == turtle_name:
                    del self.alive_turtles[i]
                    self.publish_alive_turtles()
                    break

            
            
        except Exception as e:
            self.get_logger().error("Service call failed {}".format(e))



        
 
 
def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawn() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()