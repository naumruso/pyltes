__author__ = 'Mariusz Slabicki, Konrad Polys'

import math
import csv
import numpy as np

class NetworkDevice:
    """Network device, needed for inheritance"""
    def __init__(self):
        self.x = 0
        self.y = 0

class UE(NetworkDevice):
    """UE"""
    def __init__(self):
        self.ID = 0
        self.connectedToBS = 0
        self.inside = True
        self.visibility = {}
        self.distances = {}

    def distanceToBS(self, BS):
        """
        Returns a precomputed distance for the given BS
        or computes the distance, saves it, and then returns it
        """
        if self.distances is not None:
            if BS.ID in self.distances:
                return self.distances[BS.ID]
            else:
                self.distances[BS.ID] = self.get_distance_to_bs(BS)
                return self.distances[BS.ID]
        else:
            self.distances = {}
            self.distances[BS.ID] = self.get_distance_to_bs(BS)
            return self.distances[BS.ID]

    def get_distance_to_bs(self, BS):
        return math.sqrt((self.x-BS.x)**2+(self.y-BS.y)**2)

    def isSeenFromBS(self, BS):
        """
        tries to access visibility if it has been created, 
        otherwise computes the visibility for the current BS,
        saves it in a dictionary and returns later this value.
        """
        if self.visibility is not None:
            if BS.ID in self.visibility:
                return self.visibility[BS.ID]
            else:
                self.visibility[BS.ID] = self.is_visible(BS)
                return self.visibility[BS.ID]
        else:
            self.visibility = {}
            self.visibility[BS.ID] = self.is_visible(BS)
            return self.visibility[BS.ID]

    def __is_visible(self, BS):
        """obsolete"""
        if BS.omnidirectionalAntenna == True:
            return True
        #returns true if angle allow signal receive, else False
        a_y = BS.y-self.y
        distance_bs_ue = self.distanceToBS(BS)
        if distance_bs_ue == 0 or BS.turnedOn == False:
            return False
        ue_angle_rad = math.acos(a_y/distance_bs_ue)
        ue_angle = math.degrees(ue_angle_rad)
        if self.x <= BS.x:
            ue_angle = 360 - ue_angle
        if BS.angle > ue_angle:
            alpha_diff = BS.angle - ue_angle
        else:
            alpha_diff = ue_angle - BS.angle
        if alpha_diff <= BS.beamwidth or alpha_diff >= 360-BS.beamwidth:
            return True
        else:
            return False
    
    def is_visible(self, BS):
        if BS.omnidirectionalAntenna:
            return True
        
        dist = self.distanceToBS(BS)
        if dist == 0 or not BS.turnedOn:
            return False
        
        return self.is_inside(BS.beam_start, BS.beam_end, x0=BS.x, y0=BS.y)

    def is_inside(self, start, end, x0=0, y0=0):
        """
        Checks whether the device is between the angles start and end.
        The angles are measured counter-clockwise from the x-axis.
        
        The check is done in the coordinate system of the UE unless
        (x0,y0) are passed, which shift the coordinates of the UE,
        x' -> x - x0,
        y' -> y - y0.
        """
        if start < 0:
            start = (start+360)%360
        if end < 0:
            end = (end+360)%360

        angle = math.degrees(math.atan2(self.y - y0, self.x - x0))
        if angle < 0:
            angle = angle + 360

        if start < end:
            if start < angle < end:
                return True
            else:
                return False
        elif start > end:
            if angle > end and angle > start:
                return True
            elif angle < end and angle < start:
                return True
            else:
                return False
        else:
            return False

    def connectToNearestBS(self, BS_vector):
        closestDistance = -1
        foundBS = -1
        for bs in BS_vector:
            if self.isSeenFromBS(bs):
                currentDistance = self.distanceToBS(bs)
                if currentDistance < closestDistance or foundBS == -1:
                    closestDistance = currentDistance
                    foundBS = bs.ID
        self.connectedToBS = foundBS

    def connectToTheBestBS(self, BS_vector, obstacleVector = None):
        theBestSINR = -1000
        foundBS = -1
        for bs in BS_vector:
            if self.isSeenFromBS(bs):
                self.connectedToBS = bs.ID
                currentSINR = self.calculateSINR(BS_vector, obstacleVector)
                if theBestSINR < currentSINR or foundBS == -1:
                    theBestSINR = currentSINR
                    foundBS = bs.ID
        self.connectedToBS = foundBS

    def calculateWallLoss(self, BS_vector, obstacleVector):
        wallLoss = 0
        for obstacle in obstacleVector:
            s10_x = self.x - BS_vector[self.connectedToBS].x
            s10_y = self.y - BS_vector[self.connectedToBS].y
            s32_x = obstacle[2] - obstacle[0]
            s32_y = obstacle[3] - obstacle[1]

            denom = s10_x * s32_y - s32_x * s10_y

            if denom == 0 :
                continue

            denom_is_positive = denom > 0

            s02_x = BS_vector[self.connectedToBS].x - obstacle[0]
            s02_y = BS_vector[self.connectedToBS].y - obstacle[1]

            s_numer = s10_x * s02_y - s10_y * s02_x

            if (s_numer < 0) == denom_is_positive:
                continue

            t_numer = s32_x * s02_y - s32_y * s02_x

            if (t_numer < 0) == denom_is_positive:
                continue

            if (s_numer > denom) == denom_is_positive or (t_numer > denom) == denom_is_positive :
                continue


            wallLoss = wallLoss + obstacle[4]
        return wallLoss

    def calculateReceivedPower(self, pSend, distance):
        R = distance
        lambda_val = 0.142758313333
        a = 4.0
        b = 0.0065
        c = 17.1
        d = 10.8
        s = 15.8

        ht = 40
        hr = 1.5
        f = 2.1
        gamma = a - b*ht + c/ht
        Xf = 6 * math.log10( f/2 )
        Xh = -d * math.log10( hr/2 )

        R0 = 100.0
        R0p = R0 * pow(10.0,-( (Xf+Xh) / (10*gamma) ))

        if(R>R0p):
            alpha = 20 * math.log10( (4*math.pi*R0p) / lambda_val )
            PL = alpha + 10*gamma*math.log10( R/R0 ) + Xf + Xh + s
        else:
            PL = 20 * math.log10( (4*math.pi*R) / lambda_val ) + s

        pRec = pSend - PL
        if(pRec > pSend):
            pRec = pSend
        return pRec

    def calculateNoise(self, bandwidth=20):
        k = 1.3806488 * math.pow(10, -23)
        T = 293.0
        BW = bandwidth * 1000 * 1000
        N = 10*math.log10(k*T) + 10*math.log10(BW)
        return N

    def calculateRSSI(self, BS_vector, bandwidth=20):
        """
        BS_vector is the list of base stations in the network.

        Computes RSSI = signal + interference + noise.
        """
        # distance to serving cell
        R = self.distanceToBS(BS_vector[self.connectedToBS])

        # received power is the transmission power of the serving cell minus path loss due to distance
        receivedPower_connectedBS=self.calculateReceivedPower(BS_vector[self.connectedToBS].outsidePower, R)

        # need to go through all other stations to collect the interference
        receivedPower_otherBS_mw = 0
        for BS in BS_vector:
            # skip the serving cell and if the cell is not visible from the direction of the UE
            if self.connectedToBS == BS.ID:
                continue
            if not self.isSeenFromBS(BS):
                continue

            receivedPower_one = self.calculateReceivedPower(BS.outsidePower, self.distanceToBS(BS))
            receivedPower_otherBS_mw = receivedPower_otherBS_mw + math.pow(10, receivedPower_one/10)

        I_mw = receivedPower_otherBS_mw
        S_mw = math.pow(10, receivedPower_connectedBS/10)
        N_mw = math.pow(10, self.calculateNoise(bandwidth)/10)

        RSSI_mw = I_mw + S_mw + N_mw
        return 10*math.log10(RSSI_mw) # to dBm

    def calculateRSRP(self, BS, RB):
        R = self.distanceToBS(BS)
        receivedPower_connectedBS = self.calculateReceivedPower(BS.outsidePower, R)
        S_mw = math.pow(10, receivedPower_connectedBS/10)
        rsrp_mw =  S_mw / (12*RB)
        return 10*math.log10(rsrp_mw)

    def calculateSINRfor(self, where, BS_vector, obstacleVector = None, bandwidth=20):
        if (where not in ["in", "out"]):
            raise Exception("wrong argument")

        R = self.distanceToBS(BS_vector[self.connectedToBS])
        if (where=="in"):
            receivedPower_connectedBS=self.calculateReceivedPower(BS_vector[self.connectedToBS].insidePower, R)
        else: # where=="out"
            receivedPower_connectedBS=self.calculateReceivedPower(BS_vector[self.connectedToBS].outsidePower, R)

        if len(BS_vector[self.connectedToBS].characteristic) != 0:
            a_x = 10
            a_y = 0
            b_x = self.x - BS_vector[self.connectedToBS].x
            b_y = self.y - BS_vector[self.connectedToBS].y
            aob = a_x * b_x + a_y * b_y
            cos_alpha = aob / (R * 10)
            ue_angle_rad = math.acos(cos_alpha)
            ue_angle = math.trunc(math.degrees(ue_angle_rad))

            if self.y - BS_vector[self.connectedToBS].y < 0:
                ue_angle = 359 - ue_angle
            receivedPower_connectedBS += float(BS_vector[self.connectedToBS].characteristic[ue_angle])

        if obstacleVector != None:
            receivedPower_connectedBS -= self.calculateWallLoss(BS_vector, obstacleVector)

        myColor = BS_vector[self.connectedToBS].color
        receivedPower_otherBS_mw = 0
        for bs_other in BS_vector:
            if self.connectedToBS == bs_other.ID:
                continue
            if self.isSeenFromBS(bs_other) is False:
                continue

            if (where=="in" and BS_vector[self.connectedToBS].useSFR):
                sum_power_mw = 0
                for i in range(1,4):
                    if (myColor == i):
                        continue
                    if(bs_other.color == i):
                        bs_other_power = bs_other.outsidePower
                    else:
                        bs_other_power = bs_other.insidePower

                    sum_power_mw += math.pow(10, self.calculateReceivedPower(bs_other_power, self.distanceToBS(bs_other))/10)
                receivedPower_one = 10*math.log10(sum_power_mw/2.0)
            else: # where=="out"
                if(bs_other.color == myColor):
                    bs_other_power = bs_other.outsidePower
                else:
                    bs_other_power = bs_other.insidePower
                receivedPower_one = self.calculateReceivedPower(bs_other_power, self.distanceToBS(bs_other))

            if obstacleVector != None:
                receivedPower_one = receivedPower_one - self.calculateWallLoss(BS_vector, obstacleVector)
            receivedPower_otherBS_mw = receivedPower_otherBS_mw + math.pow(10, receivedPower_one/10)

        I_mw = receivedPower_otherBS_mw
        S_mw = math.pow(10, receivedPower_connectedBS/10)
        N_mw = math.pow(10, self.calculateNoise(bandwidth)/10)

        SINR_mw = S_mw/(I_mw+N_mw)
        SINR = 10*math.log10(SINR_mw)

        return SINR

    def calculateSINR(self, BS_vector, obstacleVector = None):
        if BS_vector[self.connectedToBS].useSFR:
            SINRin = self.calculateSINRfor("in", BS_vector, obstacleVector)
            if(SINRin > BS_vector[self.connectedToBS].mi):
                SINR=SINRin
                self.inside = True
            else:
                SINR=self.calculateSINRfor("out", BS_vector, obstacleVector)
                self.inside = False
        else:
            SINR=self.calculateSINRfor("out", BS_vector, obstacleVector)
            self.inside = False
        return SINR

    def calculateMaxThroughputOfTheNode(self, bs_vector, obstacles = None):
        r_i = 0.0
        M_i = 0.0
        sinr = self.calculateSINR(bs_vector, obstacles)
        if sinr < -5.45:
            r_i = 0
            M_i = 1
        elif -5.45 <= sinr < -3.63:
            r_i = 78/1024
            M_i = 4
        elif -3.63 <= sinr < -1.81:
            r_i = 120/1034
            M_i = 4
        elif -1.81 <= sinr < 0:
            r_i = 193/1024
            M_i = 4
        elif 0 <= sinr < 1.81:
            r_i = 308/1024
            M_i = 4
        elif 1.81 <= sinr < 3.63:
            r_i = 449/1024
            M_i = 4
        elif 3.63 <= sinr < 5.45:
            r_i = 602/1024
            M_i = 4
        elif 5.45 <= sinr < 7.27:
            r_i = 378/1024
            M_i = 16
        elif 7.27 <= sinr < 9.09:
            r_i = 490/1024
            M_i = 16
        elif 9.09 <= sinr < 10.90:
            r_i = 616/1024
            M_i = 16
        elif 10.90 <= sinr < 12.72:
            r_i = 466/1024
            M_i = 64
        elif 12.72 <= sinr < 14.54:
            r_i = 567/1024
            M_i = 64
        elif 14.54 <= sinr < 16.36:
            r_i = 666/1024
            M_i = 64
        elif 16.36 <= sinr < 18.18:
            r_i = 772/1024
            M_i = 64
        elif 18.18 <= sinr < 20:
            r_i = 873/1024
            M_i = 64
        elif 20 <= sinr:
            r_i = 948/1024
            M_i = 64

        if bs_vector[self.connectedToBS].useSFR == True:
            if self.inside:
                capacityForUE_ms = r_i * math.log2(M_i) * 12 * 7 * ((200*(2/3))/1)
                capacityForUE_s = capacityForUE_ms * 1000
            else:
                capacityForUE_ms = r_i * math.log2(M_i) * 12 * 7 * ((200*(1/3))/1)
                capacityForUE_s = capacityForUE_ms * 1000
        else:
            capacityForUE_ms = r_i * math.log2(M_i) * 12 * 7 * ((200)/1)
            capacityForUE_s = capacityForUE_ms * 1000
        return capacityForUE_s

    def radio_quantities(self, BS_vector, bandwidth=20):
        """
        Assuming the UE is attached to one of the base stations in BS_vectors,
        the function returns a dict of the following values:

        'ta': timing advance (distance) to the base station
        'rssi': received signal strength indicator (RSS in dBm)
        'rsrp': reference signal received power (RSRP in dBm)
        'rsrq': reference signal received quality (RSRQ in dBm)
        'sinr': signal to noise & interference ratio (SINR in dBm)

        The bandwidth is required to compute the noise level and
        to determine the number of resource blocks per channel bandwidth.
        bandwidths = [1.4, 3.0, 5.0, 10.0, 15.0, 20.0]
        RBs = [6, 15, 25, 50, 75, 100]
        """
        bandwidths = [1.4, 3.0, 5.0, 10.0, 15.0, 20.0]
        RBs = [6, 15, 25, 50, 75, 100]

        RB = None
        for (bandwidth_, RB_) in zip(bandwidths, RBs):
            if np.abs(bandwidth_ - bandwidth) < 0.01:
                RB = RB_
                break # found a match

        if RB is None:
            raise ValueError("input parameter bandwidth={} could not be recognised.".format(bandwidth))

        sinr = self.calculateSINR(BS_vector)
        rssi = self.calculateRSSI(BS_vector)
        rsrp = self.calculateRSRP(BS_vector[self.connectedToBS], RB) # in dBm, formula 1
        # rsrp = rssi - 10*math.log10(12*RB) # in dBm, formula 2
        rsrq = 10*math.log10(RB) + rsrp - rssi # in dBm
        ta = self.distanceToBS(BS_vector[self.connectedToBS]) # "Timing Advance" (in meters)
        return {'id':self.connectedToBS, 'sinr':np.round(sinr,1), 'rssi':np.round(rssi,1), 'rsrp':np.round(rsrp,1), 'rsrq':np.round(rsrq,1), 'ta':np.round(ta,1)}


class BS(NetworkDevice):
    """Base Station"""
    def __init__(self):
        self.ID = 0
        self.insidePower = 0
        self.outsidePower = 0
        self.mi = 0
        self.Rc = 1666.3793
        self.color = 1
        self.angle = 0
        self.turnedOn = False
        self.type = "MakroCell"
        self.omnidirectionalAntenna = False
        self.useSFR = False
        self.characteristic = []

    def loadCharacteristic(self, filename):
        readCharacteristic = csv.reader(open(filename), delimiter=';', quotechar='|')
        for oneAngle in readCharacteristic:
            self.characteristic.append(float(oneAngle[1]))
