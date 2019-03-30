from afrl.cmasi.searchai import HazardZone
from amase.TCPClient import AmaseTCPClient
from amase.TCPClient import IDataReceived
from afrl.cmasi.searchai.HazardZoneEstimateReport import HazardZoneEstimateReport
from afrl.cmasi.Circle import Circle
from afrl.cmasi.Polygon import Polygon
from afrl.cmasi.Waypoint import Waypoint
from afrl.cmasi.VehicleActionCommand import VehicleActionCommand
from afrl.cmasi.LoiterAction import LoiterAction
from afrl.cmasi.LoiterType import LoiterType
from afrl.cmasi.LoiterDirection import LoiterDirection
from afrl.cmasi.CommandStatusType import CommandStatusType
from afrl.cmasi.AltitudeType import AltitudeType
from afrl.cmasi.searchai.HazardZoneDetection import HazardZoneDetection
from afrl.cmasi.searchai.HazardType import HazardType
from afrl.cmasi.Location3D import Location3D
from afrl.cmasi.AirVehicleState import AirVehicleState
from afrl.cmasi.AirVehicleConfiguration import AirVehicleConfiguration
from afrl.cmasi.KeepInZone import KeepInZone

# Librarys that haven't come with the default hackathon package
from scipy.special import softmax
import math
import numpy as np

GRID_SIZE = 100

class Node():  # Class defined for each point in the matrix
    def __init__(self, id_in, location_in):
        self.ID = id_in
        self.Location = location_in
        self.value = 1


class PrintLMCPObject(IDataReceived):
    def dataReceived(self, lmcpObject):
        print(lmcpObject.toXMLStr(""))


class Summit(IDataReceived):

    def __init__(self, tcpClient):
        self.__client = tcpClient
        self.__uavsLoiter = {}
        self.__estimatedHazardZone = Polygon()
        self.node = [[[] for i in range(GRID_SIZE)] for j in range(GRID_SIZE)] # Creates empty array 10 x 10
        self.grid = np.full((GRID_SIZE, GRID_SIZE), 1.0)
        self.p_grid = np.empty((GRID_SIZE, GRID_SIZE))
        self.zoneCenter = Location3D()

    def tick(self):
        print(self.grid)
        alpha = 0.5
        # x  current drone position in grid
        # y  position in grid
        x = y = 0
        # adjust the grid based on current observation
        # no fire in the area of the drone decrease the p value by alpha
        #self.grid[x][y] *= alpha

        # pick a random waypoint based on probability grid

        # Credrics random point selector with probability ---
#        coords = [[a, b] for a in range(GRID_SIZE) for b in range(GRID_SIZE)]
#        pvalues = softmax(self.grid.flatten())
#        r = np.random.choice(range(GRID_SIZE * GRID_SIZE), 1, p=pvalues)[0]
#        print(coords[r])
    #Another implementaion has been made down the bottom


    # Get position
    # Compute distance to refuel
    #   distance =
    # if distance to refuel > fuel * consumption * safety_coefficient:
    # set heading to / waypoint to refuel position
    # else:
    #      r = rand() # between 0 and 1
    #     epsilon = 0.8
    #    if r < epsilon: # Exploration
    #       random_angle = rand() * 360
    # set random heading
    #  else:
    # use knowledge
    #     if fire_detected:
    #
    def dataReceived(self, lmcpObject):
        # self.tick()
        if isinstance(lmcpObject, KeepInZone):
            zone = lmcpObject
            self.zoneCenter = zone.Boundary.CenterPoint  # Stores the Zones bounding box geopoint into Zone Center
            w = zone.Boundary.Width
            h = zone.Boundary.Height
            print("Zone Lat: " + str(self.zoneCenter.get_Latitude()))
            print("Zone Long: " + str(self.zoneCenter.get_Longitude()))

            for a in range(GRID_SIZE):     # Fill array with Node Objects (Is instanciated here because we can get the zone data here)
                for b in range(GRID_SIZE):
                    self.node[a][b] = Node(str(a) + str(b), self.meterCoordsFromCenter((w/GRID_SIZE)*a - w/2,(h/GRID_SIZE)*b - h/2, 700))
            self.tick()

        if isinstance(lmcpObject, AirVehicleState):
            vehicleState = lmcpObject

        if isinstance(lmcpObject, AirVehicleConfiguration):
            vehicleInfo = lmcpObject
            print(str(vehicleInfo.EntityType))

            locNode = self.getNode()
            loc = self.node[locNode[0]][locNode[1]]

            if (str(vehicleInfo.EntityType) == "b'FixedWing'"):
                self.performAction(vehicleInfo.get_ID(), loc.Location, "loiter")    # Setting the beggining location for the drones to loiter

            elif (str(vehicleInfo.EntityType) == "b'Multi'"):
                self.performAction(vehicleInfo.get_ID(), loc.Location, "loiter")

        if isinstance(lmcpObject, HazardZoneDetection):
            hazardDetected = lmcpObject
            # Get location where zone first detected
            detectedLocation = hazardDetected.get_DetectedLocation()
            # Get entity that detected the zone
            detectingEntity = hazardDetected.get_DetectingEnitiyID()
            # Check if the UAV has already been sent the loiter command and proceed if it hasn't
            if not detectingEntity in self.__uavsLoiter:
                # Send the loiter command
                self.sendLoiterCommand(detectingEntity, detectedLocation)

                # Note: Polygon points must be in clockwise or counter-clockwise order to get a shape without intersections
                self.__estimatedHazardZone.get_BoundaryPoints().append(detectedLocation)

                # Send out the estimation report to draw the polygon
                self.sendEstimateReport()

                self.__uavsLoiter[detectingEntity] = True
                print('UAV' + str(detectingEntity) + ' detected hazard at ' + str(
                    detectedLocation.get_Latitude()) + ',' + str(
                    detectedLocation.get_Longitude()) + '. Sending loiter command.');
                self.detected = True

    def sendLoiterCommand(self, vehicleId, location):
        # Setting up the mission to send to the UAV
        vehicleActionCommand = VehicleActionCommand()
        vehicleActionCommand.set_VehicleID(vehicleId)
        vehicleActionCommand.set_Status(CommandStatusType.Pending)
        vehicleActionCommand.set_CommandID(1)

        # Setting up the loiter action
        loiterAction = LoiterAction()
        loiterAction.set_LoiterType(LoiterType.Circular)
        loiterAction.set_Radius(250)
        loiterAction.set_Axis(0)
        loiterAction.set_Length(0)
        loiterAction.set_Direction(LoiterDirection.Clockwise)
        loiterAction.set_Duration(100000)
        loiterAction.set_Airspeed(15)

        # Creating a 3D location object for the stare point
        loiterAction.set_Location(location)

        # Adding the loiter action to the vehicle action list
        vehicleActionCommand.get_VehicleActionList().append(loiterAction)

        # Sending the Vehicle Action Command message to AMASE to be interpreted
        self.__client.sendLMCPObject(vehicleActionCommand)

    def sendEstimateReport(self):
        # Setting up the mission to send to the UAV
        hazardZoneEstimateReport = HazardZoneEstimateReport()
        hazardZoneEstimateReport.set_EstimatedZoneShape(self.__estimatedHazardZone)
        hazardZoneEstimateReport.set_UniqueTrackingID(1)
        hazardZoneEstimateReport.set_EstimatedGrowthRate(0)
        hazardZoneEstimateReport.set_PerceivedZoneType(HazardType.Fire)
        hazardZoneEstimateReport.set_EstimatedZoneDirection(0)
        hazardZoneEstimateReport.set_EstimatedZoneSpeed(0)

        # Sending the Vehicle Action Command message to AMASE to be interpreted
        self.__client.sendLMCPObject(hazardZoneEstimateReport)

    def performAction(self, vehicleId, location, action):  # Action that is called that tells the drone what to do
        # Setting up the mission to send to the UAV
        vehicleActionCommand = VehicleActionCommand()
        vehicleActionCommand.set_VehicleID(vehicleId)
        vehicleActionCommand.set_Status(CommandStatusType.Pending)
        vehicleActionCommand.set_CommandID(1)

        if (action == "loiter"):
            loiter = LoiterAction()

            loiter.set_LoiterType(LoiterType.Circular)
            loiter.set_Radius(1000)
            loiter.set_Axis(0)
            loiter.set_Length(0)
            loiter.set_Direction(LoiterDirection.Clockwise)
            loiter.set_Duration(100000)
            loiter.set_Airspeed(15)

            loiter.set_Location(location)
            vehicleActionCommand.get_VehicleActionList().append(loiter)

        # Sending the Vehicle Action Command message to AMASE to be interpreted
        self.__client.sendLMCPObject(vehicleActionCommand)

    def meterCoordsFromCenter(self, lat, lon, alt):
        # Following code block allows us to get meters from coordinates and add them to each other. which is faaaaantastic
        earth = 6378.137
        pi = 3.1415
        m = (1 / ((2 * math.pi / 360) * earth)) / 1000
        coords = Location3D()

        latitude = self.zoneCenter.get_Latitude() + (lat * m)  # The '0' here is the amount of meters we would like to shift left or right
        longitude = self.zoneCenter.get_Longitude() + (lon * m) / math.cos(self.zoneCenter.get_Latitude() * (math.pi / 180))
        # ----------------------------------------------------------------------------------------------------------------

        coords.set_Latitude(latitude)
        coords.set_Longitude(longitude)
        coords.set_Altitude(alt)
        return coords

    def getNode(self):  #Cedrics get coordinates with probability method
        coords = [[a, b] for a in range(GRID_SIZE) for b in range(GRID_SIZE)]
        pvalues = softmax(self.grid.flatten())
        r = np.random.choice(range(GRID_SIZE * GRID_SIZE), 1, p=pvalues)[0]
        return coords[r]

#################
## Main
#################

if __name__ == '__main__':
    myHost = 'localhost'
    myPort = 5555
    amaseClient = AmaseTCPClient(myHost, myPort)
    # amaseClient.addReceiveCallback(PrintLMCPObject())
    amaseClient.addReceiveCallback(Summit(amaseClient))

    try:
        # make a threaded client, listen until a keyboard interrupt (ctrl-c)
        # start client thread
        amaseClient.start()
        print("running")
        while True:
            # wait for keyboard interrupt
            pass
    except KeyboardInterrupt as ki:
        print("Stopping amase tcp client")
    except Exception as ex:
        print(ex)
    amaseClient.stop()

"""
    def tick()
        # Get position
        # Compute distance to refuel
        distance = 
        # if distance to refuel > fuel * consumption * safety_coefficient:
            # set heading to / waypoint to refuel position
        # else:
            r = rand() # between 0 and 1
            epsilon = 0.8
            if r < epsilon: # Exploration
                random_angle = rand() * 360
                # set random heading
            else:
                # use knowledge
                if fire_detected:
                    # 
    def dataReceived(self, lmcpObject):

        tick()

        if isinstance(lmcpObject, AirVehicleState):
            vehicleState = lmcpObject

            vehicleState.Location
"""
