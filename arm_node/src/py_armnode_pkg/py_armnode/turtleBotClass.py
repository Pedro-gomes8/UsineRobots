#!/usr/bin/env python3

class TurtleBot:
    def _init_(self):
        self.isThere = False
        self.x = 0
        self.y = 0
        self.objectsColors = ""
        self.id = ""

    def turtleBotArrived(self, x, y, objectsColors, id):
        self.isThere = True
        self.x = x
        self.y = y
        self.objectsColors = objectsColors
        self.id = id


    def addObjectColor(self, objectColor):
        self.objectsColors = objectColor

    def turtleBotLeft(self):
        self.isThere = False
        self.x = 0
        self.y = 0
        self.objectsColors = ""
        self.id = ""
