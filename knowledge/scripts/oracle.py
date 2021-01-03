#!/usr/bin/env python3

"""
    ROS service used to simulate robot motion
"""

# Import of libraries
import rospy

from geometry_msgs.msg import Point
from knowledge.srv import OracleReq,OracleReqResponse

class Location:

    def __init__(self,name,visited,position):

        self.name = name
        self.visited = visited
        self.position = position
    
    def getName(self):
        return self.name

    def getVisited(self):
        return self.visited

    def getPosition(self):
        return self.position

    def setVisited(self,visited):
        self.visited = visited

    def setPosition(self,position):
        self.position = position
    
class Oracle:

    def __init__(self):

        entrance = Location("entrance",False,Point())
        closet = Location("closet",False,Point())
        living_room = Location("living_room",False,Point())
        kitchen = Location("kitchen",False,Point())
        bathroom = Location("bathroom",False,Point())
        bedroom = Location("bedroom",False,Point())

        self.house = {
            "blue": entrance,
            "red": closet,
            "green": living_room,
            "yellow": kitchen,
            "magenta": bathroom,
            "black": bedroom,
        }

        rospy.init_node('oracle_server')

    
        s = rospy.Service('oracle_req', OracleReq, self.handle_req)

    def handle_req(self,req):

        resp = OracleReqResponse()

        request = req.req_type.split()

        if(request[0] == "prevDetect"):
            ball = request[1]
            resp.location = str(self.house[ball].getVisited())
        elif(request[0] == "setPos"):
            ball = request[1]
            position = Point(float(request[2]),float(request[3]),0)
            self.house[ball].setPosition(position)
            self.house[ball].setVisited(True)
        elif(request[0] == "isVisited"):
            req_location = request[1]
            for ball, location in self.house.items():
                if location.getName() == req_location:
                    req_ball = ball
                    req_location = location
                    break
            resp.location = str(req_location.getVisited())+" "+req_ball
        elif(request[0] == "getPos"):
            ball = request[1]
            resp.location = str(self.house[ball].getPosition().x)+" "+str(self.house[ball].getPosition().y)

        return resp


if __name__ == "__main__":

    o = Oracle()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Oracle module")