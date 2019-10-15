__author__ = 'Mariusz Slabicki, Konrad Polys'

from pyltes import devices
from pyltes import generator
from pyltes import printer

import math
import random
import pickle
import copy

import numpy as np

import pymap3d

from shapely.geometry import Point, Polygon
from shapely.ops import cascaded_union


class CellularNetwork:
    """Class describing cellular network"""
    def __init__(self):
        self.ue = []
        self.bs = []
        self.obstacles = []
        self.constraintAreaMaxX = []
        self.constraintAreaMaxY = []
        self.radius = []
        self.minTxPower = 10
        self.maxTxPower = 40
        self.minFemtoTxPower = 3
        self.maxFemtoTxPower = 10
        self.optimizationFunctionResults = None
        self.Generator = generator.Generator(self)
        self.Printer = printer.Printer(self)
        self.powerConfigurator = []
        self.colorConfigurator = []
        
    def loadPowerConfigurator(self):
        from pyltes import powerConfigurator
        self.powerConfigurator = powerConfigurator.pygmoPowerConfigurator(self)

    def loadColorConfigurator(self):
        from modules import colorConfigurator
        self.colorConfigurator = colorConfigurator.pygmoColorConfigurator(self)

    def saveNetworkToFile(self, filename):
        with open(filename+".pnf", 'wb') as f:
            pickle.dump(self, f)

    @classmethod
    def loadNetworkFromFile(cls, filename):
        with open(filename+".pnf", 'rb') as f:
            return pickle.load(f)
        
    def addOneBSTower(self, x_pos, y_pos, omnidirectional = False):
        if omnidirectional == False:
            for i in range(3):
                bs = devices.BS()
                bs.x = x_pos
                bs.y = y_pos
                bs.insidePower = 37
                bs.outsidePower = 40
                bs.angle = i * 120
                bs.ID = len(self.bs)
                bs.turnedOn = True
                self.bs.append(copy.deepcopy(bs))
        
    def printPowersInBS(self):
        powers = []
        for bs in self.bs:
            powers.append(bs.outsidePower)
        print(powers)

    def connectUsersToNearestBS(self):
        for ue in self.ue:
            ue.connectToNearestBS(self.bs)

    def connectUsersToTheBestBS(self):
        for ue in self.ue:
            ue.connectToTheBestBS(self.bs)

    def setPowerInAllBS(self, outsidePowerLevel, insidePowerLevel=None):
        if (insidePowerLevel==None):
            insidePowerLevel = outsidePowerLevel - 3
        for bs in self.bs:
            if bs.useSFR:
                bs.insidePower = insidePowerLevel
                bs.outsidePower = outsidePowerLevel
            else:
                bs.insidePower = outsidePowerLevel
                bs.outsidePower = outsidePowerLevel

    def setRandomPowerInAllBS(self, powerLevel):
        for bs in self.bs:
            if bs.useSFR:
                bs.insidePower = random.randint(0, powerLevel) - 3
                bs.outsidePower = bs.insidePower + 3
            else:
                bs.outsidePower = random.randint(0, powerLevel)
                bs.insidePower = bs.outsidePower

    def setSmallestPossiblePowerInAllBS(self):
        for bs in self.bs:
            if bs.type == "MakroCell":
                if bs.useSFR:
                    bs.insidePower = self.minTxPower - 3
                    bs.outsidePower = self.minTxPower
                else:
                    bs.insidePower = self.minTxPower
                    bs.outsidePower = self.minTxPower
            if bs.type == "FemtoCell":
                bs.power == self.minFemtoTxPower

    def setHighestPossiblePowerInAllBS(self):
        for bs in self.bs:
            if bs.type == "MakroCell":
                bs.outsidePower = self.maxTxPower
            if bs.type == "FemtoCell":
                bs.power == self.maxFemtoTxPower

    def setMiInAllBS(self, mi):
        for bs in self.bs:
            bs.mi = mi

    def setColorRandomlyInAllBS(self):
        for bs in self.bs:
            bs.color = random.randint(1,3)

    def setColorInAllBS(self, color):
        for bs in self.bs:
            bs.color = color

    def getColorInAllBS(self):
        for bs in self.bs:
            print(bs.ID, bs.color)

    def setColorInBS(self, bs, color):
        self.bs[bs].color = color

    def setRcInAllBS(self, Rc):
        for bs in self.bs:
            bs.Rc = Rc

    def calculateSINRVectorForAllUE(self):
        temp_measured_vector = []
        for ue in self.ue:
            for bs in self.bs:
                if bs.ID == ue.connectedToBS:
                    calculatedSINR = ue.calculateSINR(self.bs)
                    temp_measured_vector.append(calculatedSINR)
        return temp_measured_vector

    def returnRealUEThroughputVectorRR(self):
        numberOfConnectedUEToBS = []
        max_UE_throughput_vector = []
        real_UE_throughput_vector = []
        for i in range(len(self.bs)):
            numberOfConnectedUEToBS.append([0,0])
        for ue in self.ue:
            max_UE_throughput = ue.calculateMaxThroughputOfTheNode(self.bs) # need to be first to know where UE is
            if (ue.inside):
                numberOfConnectedUEToBS[ue.connectedToBS][0] += 1
            else:
                numberOfConnectedUEToBS[ue.connectedToBS][1] += 1
            max_UE_throughput_vector.append(max_UE_throughput)
            real_UE_throughput_vector.append(max_UE_throughput)
        for i in range(len(self.ue)):
            if (self.ue[i].inside):
                real_UE_throughput_vector[i] = max_UE_throughput_vector[i] / numberOfConnectedUEToBS[self.ue[i].connectedToBS][0]
            else:
                real_UE_throughput_vector[i] = max_UE_throughput_vector[i] / numberOfConnectedUEToBS[self.ue[i].connectedToBS][1]
        return real_UE_throughput_vector

    def returnRealUEThroughputVectorFS(self):
        sumOfInvThroughputPerBS = []
        real_UE_throughput_vector = []
        for i in range(len(self.bs)):
            sumOfInvThroughputPerBS.append([0,0])
        for ue in self.ue:
            ue_throughput = ue.calculateMaxThroughputOfTheNode(self.bs)
            if ue_throughput == 0:
                if (ue.inside):
                    sumOfInvThroughputPerBS[ue.connectedToBS][0] += 1
                else:
                    sumOfInvThroughputPerBS[ue.connectedToBS][1] += 1
            else:
                if (ue.inside):
                    sumOfInvThroughputPerBS[ue.connectedToBS][0] += 1.0 / ue_throughput
                else:
                    sumOfInvThroughputPerBS[ue.connectedToBS][1] += 1.0 / ue_throughput
        for ue in self.ue:
            ue_throughput = ue.calculateMaxThroughputOfTheNode(self.bs)
            if ue_throughput == 0:
                if (ue.inside):
                    weight = 1.0 / sumOfInvThroughputPerBS[ue.connectedToBS][0]
                else:
                    weight = 1.0 / sumOfInvThroughputPerBS[ue.connectedToBS][1]
            else:
                if (ue.inside):
                    weight = ((1.0 / ue_throughput) / sumOfInvThroughputPerBS[ue.connectedToBS][0])
                else:
                    weight = ((1.0 / ue_throughput) / sumOfInvThroughputPerBS[ue.connectedToBS][1])
            real_UE_throughput_vector.append(weight * ue_throughput)
        return real_UE_throughput_vector

    def returnNumberOfUEperBS(self):
        numberOfConnectedUEToBS = []
        for i in range(len(self.bs)):
            zero = 0
            numberOfConnectedUEToBS.append(zero)
        for ue in self.ue:
            numberOfConnectedUEToBS[ue.connectedToBS] += 1
        return numberOfConnectedUEToBS

    def returnAllBSinRange(self, x, y, txrange):
        choosen_BS_vector = []
        for bs in self.bs:
            if math.sqrt((x-bs.x)**2+(y-bs.y)**2) <= txrange:
                choosen_BS_vector.append(copy.deepcopy(bs.ID))
        return choosen_BS_vector

    def returnSumOfThroughput(self, bsnumber, step):
        ue = devices.UE()
        sumOfInternalThroughput = 0
        internalBS = 0
        sumOfExternalThroughput = 0
        externalBS = 0
        for x in range(0, round(self.constraintAreaMaxX), step):
            for y in range(0, round(self.constraintAreaMaxY), step):
                ue.x = x
                ue.y = y
                ue.connectToNearestBS(self.bs)
                if ue.connectedToBS == bsnumber:
                    #if ue.distanceToBS(self.bs[bsnumber]) < self.bs[bsnumber].mi * self.bs[bsnumber].Rc:
                    if ue.inside:
                        sumOfInternalThroughput = sumOfInternalThroughput + ue.calculateMaxThroughputOfTheNode(self.bs)
                        internalBS = internalBS + 1
                    else:
                        sumOfExternalThroughput = sumOfExternalThroughput + ue.calculateMaxThroughputOfTheNode(self.bs)
                        externalBS = externalBS + 1
        if externalBS != 0:
            sumOfExternalThroughput = sumOfExternalThroughput/externalBS
        if internalBS != 0:
            sumOfInternalThroughput = sumOfInternalThroughput/internalBS
        sumOfThroughput = sumOfExternalThroughput + sumOfInternalThroughput
        return sumOfThroughput



class GeoCellularNetwork(CellularNetwork):
    """ A class that creates a Cellular Network object
    that is defined as a geographical place. 
    
    The object has base stations that are read from a cell-db file,
    and a list of polygons that define the boundaries of the place
    and the buildings inside the place. No deep checks are made.

    Oprionally we can add the transmission power of each cell, if that 
    has been previously computed.
    """
    def __init__(self, lat0, lon0, alt0):
        """(lat0, lon0, alt0) are GRS84 coordinates in degrees relative to
        which the coordinates of the local tangential system are computed."""
        CellularNetwork.__init__(self)
        self.lat0 = lat0
        self.lon0 = lon0
        self.alt0 = alt0
        self.origin = (0,0)
        
        self.boundary = None
        self.buildings = None #optional

    def loadBoundary(self, polygon):
        """
        Polygon is a list of tuples (lat, lon) in degrees that
        define the boundary of the place in geodetic coordinates.
        
        The boundary has to be closed to make sense in this case."""
        coordinates = []
        for (lat, lon) in polygon:
            x, y, _ = pymap3d.geodetic2enu(lat, lon, 1.0, self.lat0, self.lon0, self.alt0)
            coordinates.append((x,y))

        if len(coordinates) > 2:
            self.boundary = Polygon(coordinates)

        if not self.boundary.is_valid:
            raise ValueError("Boundary of place not closed. coordinates={}".format(coordinates))

    def loadBuildings(self, buildings):
        """
        Buildings is a list of closed polygons that
        define each building that is located inside the place.
        
        Each polygon is a list of tuples (lat, lon) in degrees
        that define the boundary of the building. Note that each
        polygon has to be closed for the calculations to make sense.
        """
        self.buildings = []
        for building in buildings:
            coordinates = []
            for (lat, lon) in building:
                x, y, _ = pymap3d.geodetic2enu(lat, lon, 1.0, self.lat0, self.lon0, self.alt0)
                coordinates.append((x,y))

            polygon = Polygon(coordinates)
            if not polygon.is_valid:
                raise ValueError("Invalid polygon for building, coordinates={}".format(coordinates))

            self.buildings.append(polygon)
            # self.buildings_union = cascaded_union(self.buildings) #one representative polygon for all buildings

    def loadCells(self, cells, delta_dist=1500):
        """
        Instantiates the base stations that are inside the boundary of the place.
        If we need multiple base stations with a different azimuth, then they
        should be present in the input table.
        
        The input variable is an iterable where each row has the following fields:
        'cellname': string,
        'bslat': float,
        'bslon': float,
        'antennaheight': float,
        'azimuth': int,
        'horizbeamwidth': int
        (not all of the fields are currently used)
        """
        self.bs = []
        for cell in cells:
            cellname = cell['cellname']
            bslat = cell['bslat']
            bslon = cell['bslon']
            bsalt = cell['antennaheight']
            azimuth = cell['azimuth']
            beamwidth = cell['horizbeamwidth'] # full beam-width

            transform_angle = lambda angle: (90 - angle) % 360 # to trigonometric system (angles counter counter-clockwise from the x-axis)
            start_angle = transform_angle(azimuth + beamwidth / 2)
            end_angle = transform_angle(azimuth - beamwidth / 2)
            if end_angle < start_angle:
                end_angle + 360

            # convert the Cell's coordinates to our local tangential system
            x, y, _ = pymap3d.geodetic2enu(bslat, bslon, bsalt, self.lat0, self.lon0, self.alt0)

            if self.boundary.contains(Point(x,y)):
                # now create the base station
                bs = devices.BS()
                bs.x = x
                bs.y = y
                bs.insidePower = 37
                bs.outsidePower = 40
                bs.angle = transform_angle(azimuth)
                bs.beam_start = start_angle
                bs.beam_end = end_angle
                bs.ID = len(self.bs)
                bs.turnedOn = True
                bs.beamwidth = beamwidth
                bs.cell = cell
                self.bs.append(copy.deepcopy(bs))
            else:
                print("Warning: skipped cell with coordinates: ({:11.7f},{:11.7f}). Outside of boundary.".format(bslat, bslon))

        # finds "the bounding box" of the place
        x_all = [bs.x for bs in self.bs]
        y_all = [bs.y for bs in self.bs]

        xmax = np.max(x_all)
        xmin = np.min(x_all)

        ymax = np.max(y_all)
        ymin = np.min(y_all)

        # the rectange defined by these specifies the max. size of the place
        # adds delta_dist to not get too close to the base stations
        self.constraintAreaMaxX = (xmax - xmin) + delta_dist
        self.constraintAreaMaxY = (ymax - ymin) + delta_dist

        # lowest most left point on the plot
        self.origin = (xmin - delta_dist/2, ymin - delta_dist/2)
                     # (xmax + delta_dist/2, ymax + delta_dist/2))

    def generateDevices(self, N=100, frac=0.8):
        """
        Generates N number of UE devices so that N*frac of them
        (set by parameter frac) are outside and N*(1-frac) of the
        UE's are inside buidings.
        """
        gen_coord_x = lambda: random.uniform(self.origin[0],self.origin[0] + self.constraintAreaMaxX)
        gen_coord_y = lambda: random.uniform(self.origin[1],self.origin[1] + self.constraintAreaMaxY)
        
        n_outside_max = N * frac
        n_inside_max = N * (1 - frac)
        
        n_outside = 0
        n_inside = 0
        n_total = 0
        number = 0
        while n_total < N:
            x = gen_coord_x()
            y = gen_coord_y()
            p = Point(x,y)

            if self.boundary.contains(p):
                if np.any([building.contains(p) for building in self.buildings]):
                # if self.buildings_union.contains(p):
                    if n_inside < n_inside_max:
                        ue = devices.UE()
                        ue.ID = number
                        ue.x = x
                        ue.y = y
                        self.ue.append(ue)
                        number += 1
                        n_inside += 1
                else:
                    if n_outside < n_outside_max:
                        ue = devices.UE()
                        ue.ID = number
                        ue.x = x
                        ue.y = y
                        self.ue.append(ue)
                        number += 1
                        n_outside += 1
                n_total = n_inside + n_outside

    def outputRF(self, max_neighbors=7, bandwidth=20, method='sinr'):
        """
        After all UE's have been generated and connected to the best BS,
        we can compute the SINR, RSSI, RSRP, and RSRQ for the base station,
        to which the UE is connected and a number of nearest neighbours, 
        which is set by the input parameter max_neighbors.

        The input (channel) bandwidth is used for the computattion of RSRP & RSRQ.
        They both depend on the number of resource blocks per channel.

        Acceptable valules for bandwidth are :
            [1.4, 3.0, 5.0, 10.0, 15.0, 20.0], which correspond to
            [6, 15, 25, 50, 75, 100] resource blocks, accordingly.

        RSRP = RSSI – 10LOG(12*N)
        RSRQ = N*(RSRP/RSSI)

        The neighbor finding can be performed by distance (method='dist') or
        by sinr (method='sinr').
        """

        # These are for the base station to which the phone is attached
        all_radio_measurements = []
        for ue in self.ue:
            # returns ta, rssi, rsrp, rsrq, sinr, ue.ID for the serving cell
            ue_radio_meas = ue.radio_quantities(self.bs, bandwidth=bandwidth)

            # only neighbors that are visible from the UE
            neighbors = []
            for bs in self.bs:
                if ue.isSeenFromBS(bs) and bs.ID != ue.connectedToBS:
                    neighbors.append(bs.ID)

            # copy for safe-keeping
            serving_cell = ue.connectedToBS

            # we can select the best neighbors based on distance or sinr
            if method.lower().strip() == 'dist':
                distances = []
                for neighbor in neighbors:
                    distances.append(ue.distanceToBS(bs))
                idx = np.argsort(distances) ## closest stations come first
            else: ## method == 'sinr'
                sinr = []
                for neighbor in neighbors:
                    ue.connectedToBS = neighbor
                    sinr.append(ue.calculateSINR(self.bs))
                idx = np.argsort(sinr)[::-1] ## we need the strongest signal first

            ineighbor = 0
            ue_radio_meas['neighbors'] = []
            for idx_ in idx:
                ue.connectedToBS = neighbors[idx_]
                radio_meas = ue.radio_quantities(self.bs, bandwidth=bandwidth)
                radio_meas.pop('rssi')
                ue_radio_meas['neighbors'].append(radio_meas)
                ineighbor += 1

                if ineighbor >= max_neighbors:
                    break

            ue.connectedToBS = serving_cell

            # adding the information about the position of the device
            lat, lon, alt = pymap3d.enu2geodetic(ue.x, ue.y, 1.0, self.lat0, self.lon0, self.alt0)
            ue_radio_meas['lat'] = np.round(lat,7)
            ue_radio_meas['lon'] = np.round(lon,7)
            ue_radio_meas['alt'] = np.round(alt,1)

            # ue_radio_meas['n_neighbors'] = ineighbor

            all_radio_measurements.append(ue_radio_meas)
        return all_radio_measurements



