#!/usr/bin/env python3

"""
    ROS service that represent the oracle
"""

# Import of libraries
import rospy

from geometry_msgs.msg import Point
from knowledge.srv import OracleReq,OracleReqResponse

class Location:

    """
        A class used to represent the room in the house

        Attributes
        -----
        @param name: name of the room
        @type name: string

        @param visited: whether it has been visited or not
        @type visited: bool

        @param position: x and y position
        @type position: Point

        Methods
        -----
        getName()):
            Get the name of the room
        getVisited()
            Get if it was visited or not
        getPosition()
            Get the x and y position
        setVisited()
            Set that the room is visited
        setPosition()
            Set the x and y position
    """

    def __init__(self,name,visited,position):

        """
            Constuctor. Initialize the attributes with the given one

            @param name: name of the room
            @type name: string

            @param visited: whether it has been visited or not
            @type visited: bool

            @param position: x and y position
            @type position: Point
        """

        self.name = name
        self.visited = visited
        self.position = position
    
    def getName(self):

        """
            Get the name of the room

            @returns: name of the room
            @rtype: string
        """

        return self.name

    def getVisited(self):

        """
            Get if it was visited or not

            @returns: if it was visited or not
            @rtype: bool
        """

        return self.visited

    def getPosition(self):

        """
            Get the x and y position

            @returns: x and y position
            @rtype: Point
        """

        return self.position

    def setVisited(self,visited):

        """
            Set that the room is visited

            @param visited: room is visited
            @type visited: bool
        """

        self.visited = visited

    def setPosition(self,position):

        """
            Set the x and y position

            @param position: x and y position
            @type position: pOINT
        """

        self.position = position
    
class Oracle:

    """
        A class used to represent the oracle

        Attributes
        -----
        @param house: map the balls to the rooms
        @type house: Map

        Methods
        -----
        handle_req(self,req)
            Handle the different requests
    """

    def __init__(self):

        """
            Constuctor. Create some rooms and initialize the map. Start the service
        """

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

        """
            Handle the different requests

            @param request: request message
            @type request: OracleReq

            @returns: response to the request
            @rtype: OracleReqResponse
        """

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