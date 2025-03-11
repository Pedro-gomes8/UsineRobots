#!/usr/bin/env python3

import math

class TurtleBot:
    def __init__(self, maxObjects = 5, radiusObjectPlacement = 0.06):
        self.isThere = False
        self.x = 0
        self.y = 0
        self.objectsColors = ""
        self.id = ""
        self.numbOfObjects = 0
        self.maxObjects = maxObjects
        self.radiusObjectPlacement = radiusObjectPlacement

    def turtleBotArrived(self, x, y, objectsColors, id):
        self.isThere = True
        self.x = x
        self.y = y
        self.objectsColors = objectsColors
        self.id = id
        self.numbOfObjects = 0


    def setObjectColor(self, objectColor):
        self.objectsColors = objectColor

    def turtleBotLeft(self):
        self.isThere = False
        self.x = 0
        self.y = 0
        self.objectsColors = ""
        self.id = ""
        self.numbOfObjects = 0
        
    def getCoordonatesNextObject(self):
        if self.numbOfObjects < self.maxObjects:
            self.numbOfObjects += 1
            return [self.x + self.radiusObjectPlacement * math.cos(2*math.pi*self.numbOfObjects/self.maxObjects), self.y + self.radiusObjectPlacement * math.sin(2*math.pi*self.numbOfObjects/self.maxObjects)]
        else:
            return [self.x, self.y]
