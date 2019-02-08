__author__ = 'Mariusz'

from pyltes import devices
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import math

from matplotlib.patches import Wedge

from shapely.geometry import Point

class Printer:
    """Class that prints network deployment"""
    def __init__(self,parent):
        self.parent = parent

    def drawHistogramOfUEThroughput(self, filename):
        thr_vector = self.parent.returnRealUEThroughputVectorRR()
        thr_MBit = [x / (1024*1024) for x in thr_vector]
        plt.hist(thr_MBit)
        # plt.savefig(filename, format="pdf", dpi=300)
        plt.savefig(filename+".png", format="png", dpi=300)
        plt.clf()

    def drawHistogramOfSetPowers(self, filename):
        power_vector = []
        for bs in self.parent.bs:
            power_vector.append(bs.outsidePower)
        plt.hist(power_vector, bins=np.arange(self.parent.minFemtoTxPower, self.parent.maxTxPower + 1, 1))
        plt.xlim(0, 100)
        # plt.savefig(filename, format="pdf", dpi=300)
        plt.savefig(filename+".png", format="png", dpi=300)
        plt.clf()

    def drawNetwork(self, filename, BS=False, UE=False, links=False, obstacles=False, 
                    fillMethod="SINR", colorMap = None, drawLegend=True, tilesInLine = 100,
                    figSize = (8, 8), colorMinValue = None, colorMaxValue = None, 
                    outputFileFormat = ["png"], WedgeRadius=65):
        main_draw = plt.figure(1, figsize=figSize)
        ax = main_draw.add_subplot(111)
        if fillMethod == "SINR":
            if colorMap == None:
                cm = plt.cm.get_cmap("viridis")
            else:
                cm = plt.cm.get_cmap(colorMap)
            imageMatrix = np.zeros((tilesInLine, tilesInLine))
            d_x = round(self.parent.constraintAreaMaxX/tilesInLine)
            d_y = round(self.parent.constraintAreaMaxY/tilesInLine)
            for ix in range(0, tilesInLine):
                for iy in range(0, tilesInLine):
                    ue = devices.UE()
                    ue.x = self.parent.origin[0] + ix * d_x
                    ue.y = self.parent.origin[1] + iy * d_y
                    if self.parent.boundary.contains(Point(ue.x,ue.y)):
                        ue.connectToTheBestBS(self.parent.bs, self.parent.obstacles)
                        imageMatrix[iy][ix] = ue.calculateSINR(self.parent.bs, self.parent.obstacles)
            if colorMinValue != None:
                colorMin = colorMinValue
            else:
                colorMin = imageMatrix.min()
            if colorMaxValue != None:
                colorMax = colorMaxValue
            else:
                colorMax = imageMatrix.max()
            extent = [self.parent.origin[0], self.parent.origin[0]+self.parent.constraintAreaMaxX, 
                      self.parent.origin[1], self.parent.origin[1]+self.parent.constraintAreaMaxY]
            image = plt.imshow(imageMatrix, vmin=colorMin, vmax=colorMax, origin='lower', extent=extent, interpolation='bilinear', cmap=cm)
            if drawLegend == True:
                from mpl_toolkits.axes_grid1 import make_axes_locatable
                divider = make_axes_locatable(ax)
                cax1 = divider.append_axes("right", size="5%", pad=0.05)
                cbar = plt.colorbar(image, cax = cax1)
                #cbar.set_clim(-60, 50)
                #cbar.ax.set_yticklabels(['0','1','2','>3'])
                #cbar.set_label('# of contacts', rotation=270)

        elif fillMethod == "Sectors":
            if colorMap == None:
                cm = plt.cm.get_cmap("Paired")
            else:
                cm = plt.cm.get_cmap(colorMap)
            imageMatrix = np.zeros((tilesInLine, tilesInLine))
            d_x = round(self.parent.constraintAreaMaxX/tilesInLine)
            d_y = round(self.parent.constraintAreaMaxY/tilesInLine)
            for ix in range(0, tilesInLine):
                for iy in range(0, tilesInLine):
                    RSSI_best = -1000
                    BS_best = -1
                    ue = devices.UE()
                    ue.x = self.parent.origin[0] + ix * d_x
                    ue.y = self.parent.origin[1] + iy * d_y
                    if self.parent.boundary.contains(Point(ue.x,ue.y)):
                        for bs in self.parent.bs:
                            if not ue.isSeenFromBS(bs):
                                continue
                            ue.connectedToBS = bs.ID
                            temp_RSSI = ue.calculateSINR(self.parent.bs)
                            if temp_RSSI > RSSI_best:
                                RSSI_best = temp_RSSI
                                BS_best = bs.ID

                    imageMatrix[iy][ix] = BS_best
            extent = [self.parent.origin[0], self.parent.origin[0]+self.parent.constraintAreaMaxX, 
                      self.parent.origin[1], self.parent.origin[1]+self.parent.constraintAreaMaxY]
            # plt.imshow(imageMatrix, origin='lower', extent=extent, interpolation='bilinear', cmap=cm)
            vmin = np.min(imageMatrix) - 0.5
            vmax = np.max(imageMatrix) + 0.5
            cmap = plt.cm.get_cmap('viridis', vmax-vmin)
            plt.imshow(imageMatrix, interpolation=None, extent=extent, origin='lower', vmin=vmin, vmax=vmax, cmap=cmap)

        if BS == True:
            bs_x_locations = [bs.x for bs in self.parent.bs]
            bs_y_locations = [bs.y for bs in self.parent.bs]
            ax.plot(bs_x_locations, bs_y_locations, 'r^', color="black", markersize=5)

        if UE == True:
            ue_x_locations = [ue.x for ue in self.parent.ue]
            ue_y_locations = [ue.y for ue in self.parent.ue]
            ax.plot(ue_x_locations, ue_y_locations, 'b*', color="black", markersize=2.5)

        if links == True:
            for ue in self.parent.ue:
                ax.arrow(ue.x, ue.y, self.parent.bs[ue.connectedToBS].x - ue.x, self.parent.bs[ue.connectedToBS].y - ue.y, linewidth=0.25)

        if obstacles == True:
            for obstacle in self.parent.obstacles:
                ax.arrow(obstacle[0], obstacle[1], obstacle[2] - obstacle[0], obstacle[3] - obstacle[1], linewidth=0.25)

        for bs in self.parent.bs:
            patch = Wedge([bs.x, bs.y], WedgeRadius, bs.angle-60, bs.angle+60, linewidth=0.25, linestyle='-', edgecolor='black', fill=True, alpha=0.25, facecolor='blue')
            ax.add_patch(patch)

        networkBorder = plt.Rectangle(self.parent.origin, self.parent.constraintAreaMaxX, self.parent.constraintAreaMaxY, color='black', fill=False)
        ax.add_patch(networkBorder)
        ax.axis('equal')

        extent = [self.parent.origin[0], self.parent.origin[0]+self.parent.constraintAreaMaxX, 
                  self.parent.origin[1], self.parent.origin[1]+self.parent.constraintAreaMaxY]
        ax.axis(extent)
        ax.axis('off')
        ax.xaxis.set_visible(False)
        ax.yaxis.set_visible(False)
        for outputFormat in outputFileFormat:
            if outputFormat == "png":
                main_draw.savefig(filename+".png", format="png", dpi=300, bbox_inches='tight')
            if outputFormat == "pdf":
                main_draw.savefig(filename+".pdf", format="pdf", dpi=300, bbox_inches='tight')
        
        plt.clf()
        return imageMatrix
