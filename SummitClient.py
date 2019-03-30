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

import numpy as np
from scipy.special import softmax

GRID_SIZE = 10

class PrintLMCPObject(IDataReceived):
    def dataReceived(self, lmcpObject):
        print(lmcpObject.toXMLStr(""))

class Summit(IDataReceived):

    def __init__(self, tcpClient):
        self.__client = tcpClient
        self.__uavsLoiter = {}
        self.__estimatedHazardZone = Polygon()
        self.grid = np.full((GRID_SIZE,GRID_SIZE), 1.0)
        self.p_grid = np.empty((GRID_SIZE,GRID_SIZE))

    def tick(self):
        #print(self.grid)

        alpha = 0.5
        # x  current drone position in grid
        # y  position in grid
        x = y = 0
        # adjust the grid based on current observation
        # no fire in the area of the drone decrease the p value by alpha
        self.grid[x][y] *= alpha

        # pick a random waypoint based on probability grid
        coords = [[a, b] for a in range(GRID_SIZE) for b in range(GRID_SIZE)]
        r = np.random.choice( range(GRID_SIZE*GRID_SIZE), 1, p = softmax(self.grid.flatten()))[0]
        print(coords[r])


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
        self.tick()
        if isinstance(lmcpObject, AirVehicleState):
            vehicleState = lmcpObject
            vehicleState.Location

        if isinstance(lmcpObject, AirVehicleConfiguration):
            vehicleInfo = lmcpObject
            print(str(vehicleInfo.EntityType))
        
        if isinstance(lmcpObject, HazardZoneDetection):
            hazardDetected = lmcpObject
            #Get location where zone first detected
            detectedLocation = hazardDetected.get_DetectedLocation()
            #Get entity that detected the zone
            detectingEntity = hazardDetected.get_DetectingEnitiyID()
            #Check if the UAV has already been sent the loiter command and proceed if it hasn't
            if not detectingEntity in self.__uavsLoiter:
                #Send the loiter command
                self.sendLoiterCommand(detectingEntity, detectedLocation)

                #Note: Polygon points must be in clockwise or counter-clockwise order to get a shape without intersections
                self.__estimatedHazardZone.get_BoundaryPoints().append(detectedLocation)

                #Send out the estimation report to draw the polygon
                self.sendEstimateReport();

                self.__uavsLoiter[detectingEntity] = True
                print('UAV' + str(detectingEntity) + ' detected hazard at ' + str(detectedLocation.get_Latitude()) + ',' + str(detectedLocation.get_Longitude()) + '. Sending loiter command.');
                self.detected = True

    def sendLoiterCommand(self, vehicleId, location):
        # Setting up the mission to send to the UAV
        vehicleActionCommand = VehicleActionCommand()
        vehicleActionCommand.set_VehicleID(vehicleId)
        vehicleActionCommand.set_Status(CommandStatusType.Pending)
        vehicleActionCommand.set_CommandID(1)

        #Setting up the loiter action
        loiterAction = LoiterAction()
        loiterAction.set_LoiterType(LoiterType.Circular)
        loiterAction.set_Radius(250)
        loiterAction.set_Axis(0)
        loiterAction.set_Length(0)
        loiterAction.set_Direction(LoiterDirection.Clockwise)
        loiterAction.set_Duration(100000)
        loiterAction.set_Airspeed(15)

        #Creating a 3D location object for the stare point
        loiterAction.set_Location(location)

        #Adding the loiter action to the vehicle action list
        vehicleActionCommand.get_VehicleActionList().append(loiterAction)

        #Sending the Vehicle Action Command message to AMASE to be interpreted
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

        #Sending the Vehicle Action Command message to AMASE to be interpreted
        self.__client.sendLMCPObject(hazardZoneEstimateReport)

#################
## Main
#################

if __name__ == '__main__':
    myHost = 'localhost'
    myPort = 5555
    amaseClient = AmaseTCPClient(myHost, myPort)
    #amaseClient.addReceiveCallback(PrintLMCPObject())
    amaseClient.addReceiveCallback(Summit(amaseClient))

    try:
        # make a threaded client, listen until a keyboard interrupt (ctrl-c)
        #start client thread
        amaseClient.start()
        print("running")
        while True:
            #wait for keyboard interrupt
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
    