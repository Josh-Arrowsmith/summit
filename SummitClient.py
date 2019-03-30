from afrl.cmasi.searchai import HazardZone
from amase.TCPClient import AmaseTCPClient
from amase.TCPClient import IDataReceived
from afrl.cmasi.searchai.HazardZoneEstimateReport import HazardZoneEstimateReport
from afrl.cmasi.Circle import Circle
from afrl.cmasi.Polygon import Polygon
from afrl.cmasi.Waypoint import Waypoint
from afrl.cmasi.VehicleActionCommand import VehicleActionCommand
from afrl.cmasi.MissionCommand import MissionCommand
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
import time


class Node():  # Class defined for each point in the matrix
    def __init__(self, id_in, location_in):
        self.ID = id_in
        self.Location = location_in


class PrintLMCPObject(IDataReceived):
    def dataReceived(self, lmcpObject):
        print(lmcpObject.toXMLStr(""))


class Summit(IDataReceived):

    GRID_SIZE = 10
    ALTITUDE = 150


    def __init__(self, tcpClient):
        self.waypoints = {}
        self.__client = tcpClient
        self.__uavsLoiter = {}
        self.__estimatedHazardZone = []
        self.grid = [[0 for a in range(self.GRID_SIZE)] for b in range(self.GRID_SIZE)] #np.ones([self.GRID_SIZE, self.GRID_SIZE])
        self.p_grid = np.ones([self.GRID_SIZE, self.GRID_SIZE])
        self.zoneCenter = Location3D()
        self.starttime = time.time()


    def checkNode(self):
        coords = [[a, b] for a in range(self.GRID_SIZE) for b in range(self.GRID_SIZE)]
        pvalues = softmax(self.p_grid.flatten())
        r = np.random.choice(range(self.GRID_SIZE * self.GRID_SIZE), 1, p=pvalues)[0]
        return coords[r]

    def dataReceived(self, lmcpObject):

        # Scenario initialised
        if isinstance(lmcpObject, KeepInZone):
            zone = lmcpObject
            self.zoneCenter = zone.Boundary.CenterPoint
            w = zone.Boundary.Width
            h = zone.Boundary.Height
            print("Zone Lat: " + str(self.zoneCenter.get_Latitude()))
            print("Zone Long: " + str(self.zoneCenter.get_Longitude()))

            for a in range(self.GRID_SIZE):
                for b in range(self.GRID_SIZE):
                    pos = self.meterCoordsFromCenter((w/self.GRID_SIZE)*a - w/2,(h/self.GRID_SIZE)*b - h/2, self.ALTITUDE)
                    self.grid[a][b] = Node(str(a) + str(b), pos)

        # Aircraft initialised
        if isinstance(lmcpObject, AirVehicleConfiguration):
            vehicleInfo = lmcpObject
            print(str(vehicleInfo.EntityType))
            locNode = self.checkNode()
            loc = self.grid[locNode[0]][locNode[1]]

            self.performAction(vehicleInfo.get_ID(), loc.Location, "waypoint")
            #
            # if (str(vehicleInfo.EntityType) == "b'FixedWing'"):
            #     self.performAction(vehicleInfo.get_ID(), loc.Location, "waypoint")
            #
            # elif (str(vehicleInfo.EntityType) == "b'Multi'"):
            #     self.performAction(vehicleInfo.get_ID(), loc.Location, "waypoint")

        # AirVehicleState - ~2Hz
        if isinstance(lmcpObject, AirVehicleState):
            vehicleState = lmcpObject
            lat = vehicleState.get_Location().get_Latitude()
            lon = vehicleState.get_Location().get_Longitude()
            vehicle_id = vehicleState.get_ID()

            c = vehicleState.get_CurrentWaypoint()
            if self.waypoints != {}:
                w_lat, w_lon = self.waypoints[vehicle_id]
                if (abs(w_lat - lat) <  .009) and (abs(lon - w_lon) < .009):
                    # redeploy
                    locNode = self.checkNode()
                    loc = self.grid[locNode[0]][locNode[1]]
                    self.performAction(vehicle_id, loc.Location, "waypoint")
                    print("redeploy ", vehicle_id)

                # adjust the pgrid values when there is no fire
                x, y = self.grid_coords_from_loc(lat, lon)
                beta = .5
                self.p_grid[x][y] = beta

        # Hazard detected
        if isinstance(lmcpObject, HazardZoneDetection):
            hazardDetected = lmcpObject
            detectedLocation = hazardDetected.get_DetectedLocation()
            detectingEntity = hazardDetected.get_DetectingEnitiyID()

            if True == True: # not detectingEntity in self.__uavsLoiter:
                # Send the loiter command
                #self.sendLoiterCommand(detectingEntity, detectedLocation)

                # test if new fire
                self.__estimatedHazardZone.append(Polygon())

                # Note: Polygon points must be in clockwise or counter-clockwise order to get a shape without intersections
                self.__estimatedHazardZone[-1].get_BoundaryPoints().append(detectedLocation)

                # Send out the estimation report to draw the polygon
                self.sendEstimateReport()

                # find x y of fire, calculated from lat and lon
                f_lat = detectedLocation.get_Latitude()
                f_lon = detectedLocation.get_Longitude()

                self.__uavsLoiter[detectingEntity] = True
                print('UAV' + str(detectingEntity) + ' detected hazard at ' + str(
                    f_lat) + ',' + str(
                    f_lon) + '. Sending loiter command.')

                # adjust the pgrid values when there is fire
                x, y = self.grid_coords_from_loc(f_lat, f_lon)
                alpha = 2
                self.p_grid[x][y] = alpha
                print(x,y)
                print('fire at ', f_lat, f_lon)
                self.performAction(detectingEntity, detectedLocation, "loiter")

    def grid_coords_from_loc(self,i_lat,i_lon):
        last_diff_lat = 360
        last_diff_lon = 360
        x = y = 0
        for a in range(self.GRID_SIZE):
            for b in range(self.GRID_SIZE):
                lat = self.grid[a][b].Location.get_Latitude()
                lon = self.grid[a][b].Location.get_Longitude()
                diff_lat = abs(i_lat - lat)
                diff_lon = abs(i_lon - lon)
                if diff_lat <= last_diff_lat and diff_lon <= last_diff_lon:
                    x = a
                    y = b
                    last_diff_lat = diff_lat
                    last_diff_lon = diff_lon
        return (x,y)

    def sendLoiterCommand(self, vehicleId, location):
        # Setting up the mission to send to the UAV
        vehicleActionCommand = VehicleActionCommand()
        vehicleActionCommand.set_VehicleID(vehicleId)
        vehicleActionCommand.set_Status(CommandStatusType.Pending)
        vehicleActionCommand.set_CommandID(1)

        # Setting up the loiter action
        loiterAction = LoiterAction()
        loiterAction.set_LoiterType(LoiterType.Circular)
        loiterAction.set_Radius(50)
        loiterAction.set_Axis(0)
        loiterAction.set_Length(0)
        loiterAction.set_Direction(LoiterDirection.Clockwise)
        loiterAction.set_Duration(10000)
        loiterAction.set_Airspeed(15)

        # Creating a 3D location object for the stare point
        loiterAction.set_Location(location)

        # Adding the loiter action to the vehicle action list
        vehicleActionCommand.get_VehicleActionList().append(loiterAction)


        # Sending the Vehicle Action Command message to AMASE to be interpreted
        self.__client.sendLMCPObject(vehicleActionCommand)



    def performAction(self, vehicleId, location, action):  # Action that is called that tells the drone what to do
        # Setting up the mission to send to the UAV
        vehicleActionCommand = VehicleActionCommand()
        vehicleActionCommand.set_VehicleID(vehicleId)
        vehicleActionCommand.set_Status(CommandStatusType.Pending)
        vehicleActionCommand.set_CommandID(0)

        if (action == "waypoint"):
            vehicleActionCommand = MissionCommand()
            vehicleActionCommand.set_VehicleID(vehicleId)
            vehicleActionCommand.set_Status(CommandStatusType.Pending)
            vehicleActionCommand.set_CommandID(0)
            waypoint = Waypoint()
            waypoint.set_Latitude(location.get_Latitude())
            waypoint.set_Longitude(location.get_Longitude())
            # Avoid collision by flying at different altitudes
            waypoint.set_Altitude(self.ALTITUDE + vehicleId*10)
            waypoint.set_AltitudeType(0)
            waypoint.set_Speed(10)
            waypoint.set_SpeedType(0)
            vehicleActionCommand.get_WaypointList().append(waypoint)
            self.waypoints[vehicleId] = (location.get_Latitude(), location.get_Longitude())

        if (action == "loiter"):
            loiter = LoiterAction()
            loiter.set_LoiterType(LoiterType.Circular)
            loiter.set_Radius(500)
            loiter.set_Axis(0)
            loiter.set_Length(0)
            loiter.set_Direction(LoiterDirection.Clockwise)
            loiter.set_Duration(100000)
            loiter.set_Airspeed(15)
            loiter.set_Location(location)
            vehicleActionCommand.get_VehicleActionList().append(loiter)
            #rint(vehicleActionCommand.toString())

        # Sending the Vehicle Action Command message to AMASE to be interpreted
        self.__client.sendLMCPObject(vehicleActionCommand)


    # HELPER FUNCTIONS

    def meterCoordsFromCenter(self, lat, lon, alt):
        # Following code block allows us to get meters from coordinates and add them to each other. which is faaaaantastic
        earth = 6378.137
        m = (1 / ((2 * math.pi / 360) * earth)) / 1000
        coords = Location3D()

        latitude = self.zoneCenter.get_Latitude() + (lat * m)
        longitude = self.zoneCenter.get_Longitude() + (lon * m) / math.cos(self.zoneCenter.get_Latitude() * (math.pi / 180))

        coords.set_Latitude(latitude)
        coords.set_Longitude(longitude)
        coords.set_Altitude(alt)
        return coords



    # FOREIGN FUNCTIONS
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
