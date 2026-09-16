from math import pi
import numpy as np
import swift
import time
import matplotlib.pyplot as plt
from spatialmath.base import *
from spatialmath import SE3
from roboticstoolbox import DHLink, DHRobot
from ir_support import CylindricalDHRobotPlot, UR3
from ir_support.plyprocess import *
import roboticstoolbox as rtb
from spatialgeometry import Cuboid, Cylinder
from ir_support_extra_robots import Turtlebot3Waffle

# The project will have 3 Omron TM5-700 robotic arms that will make 
# the user a cup of tea, with the user being able to select aspects 
# relating to the cup of tea such as teabag type, milk type, 
# quantities of milk/water, and temperature the water heats up to. 
# The project will utilise objects in a digital setting, 
# adhering to safety requirements such as a emergency stop, 
# as well as robotic modelling.

# safety means integrated
    # show strategically placed safety-equipment models appropriate to the risk assessment
    #   ->barrier
    # REQUIRED e Stops
    # REQUIRED react when a controlled simulated object is deliberately placed in a planned robot path
    # REQUIRED react to an asynchronous user or simulated sensor signal representing entry into an unsafe zone

#Scene objects
#objects that STILL need to be made
    # teabag(s)
    # where the teabag(s) are held
    # milk(s)
    # where the milk(s) are held
    # tap for water
    # table/sink for tap to be on
    # stove or some way to heat the tea
    # eStop
    # could also maybe do different colour/style cups

#Tea Cup
#adding a handle to the Tea Cup might be a good idea, might do later
teaCupSizeX = 0.25
teaCupSizeY = 0.25
teaCupSizeZ = 0.1

teaCupLocationX = 0 
teaCupLocationY = 0
teaCupLocationZ = 0
teaCupWallThickness = 0.025

teaCupColour = "White"

#Safety Barrier
barrierZoneX = [-2.5, 2.7]
barrierZoneY = [-2.5, 2.5]
barrierHeight = 0.5
barrierWidth = 0.025

barrierColour = "red"

#GUI Sliders
setWaterTemperature = 0
amountOfTea = 0
proportionOfMilkInTea = 0

# GUI selectors
selectedTeaBag = ""
selectedMilkType = ""
teaBagOptions = ["Earl Gray",
                 "English Breakfast",
                 "Peppermint",
                 "Lemon and Ginger",
                 "Camomile",
                 "Green Tea"]

milkTypeOptions = ["Full Cream Milk",
                   "Low Fat Milk",
                   "Skim Milk",
                   "Almond Milk",
                   "Oat Milk",
                   "Rice Milk"]

def sliders():
    waterTemperatureSlider = swift.Slider(
        setWaterTemperature,
        min=5,
        max=95,
        step=1,
        value=5,
        desc="Water Temperature",
        unit=" °C",
    )
    env.add(waterTemperatureSlider)

    amountOfTeaSlider = swift.Slider(
        amountOfTea,
        min=0,
        max=500,
        step=1,
        value=0.0,
        desc="Amount of Tea",
        unit=" mL",
    )
    env.add(amountOfTeaSlider)

    proportionOfMilkInTeaSlider = swift.Slider(
        proportionOfMilkInTea,
        min=0,
        max=100,
        step=1,
        value=0.0,
        desc="Proportion of Milk in Tea",
        unit=" %",
    )
    env.add(proportionOfMilkInTeaSlider)


def selectors():
    teaBagSelector = swift.Select(
        selectedTeaBag,
        options=teaBagOptions,
        desc="Select Tea Bag"
    )
    env.add(teaBagSelector)

    milkTypeSelector = swift.Select(
        selectedMilkType,
        options=milkTypeOptions,
        desc="Select Milk Type"
    )
    env.add(milkTypeSelector)

#make some function that generates a cuboid in the teacup the height of amountOfTea, 
# and maybe change the colour according to tea type and milk type 
# (maybe take into account transperacy of the cup too)

def constructBarrier():
    barrierLengthX = abs(barrierZoneX[1]) + abs(barrierZoneX[0])
    barrierLengthY = abs(barrierZoneY[1]) + abs(barrierZoneY[0])

    barrierMiddleX = ((barrierZoneX[0]+barrierZoneX[1])/2)
    barrierMiddleY = ((barrierZoneY[0]+barrierZoneY[1])/2)

    barrierLeft = Cuboid([barrierWidth, barrierLengthY, barrierHeight], pose = SE3(barrierZoneX[0], barrierMiddleY, (barrierHeight/2)), color=barrierColour)
    barrierRight = Cuboid([barrierWidth, barrierLengthY, barrierHeight], pose = SE3(barrierZoneX[1], barrierMiddleY, (barrierHeight/2)), color=barrierColour)
    barrierTop = Cuboid([barrierLengthX, barrierWidth, barrierHeight], pose = SE3(barrierMiddleX, barrierZoneY[0], (barrierHeight/2)), color=barrierColour)
    barrierBottom = Cuboid([barrierLengthX, barrierWidth, barrierHeight], pose = SE3(barrierMiddleX, barrierZoneY[1], (barrierHeight/2)), color=barrierColour)

    env.add(barrierLeft)
    env.add(barrierRight)
    env.add(barrierTop)
    env.add(barrierBottom)

def constructTeaCup():
    teaCupBase = Cuboid([teaCupSizeX, teaCupSizeY, teaCupWallThickness], pose = SE3(teaCupLocationX,teaCupLocationY ,(teaCupLocationZ+teaCupWallThickness/2)), color=teaCupColour)
    teaCupLeftWall = Cuboid([teaCupWallThickness, teaCupSizeY, teaCupSizeZ], pose = SE3((teaCupLocationX - teaCupSizeX/2 + teaCupWallThickness/2),teaCupLocationY, (teaCupLocationZ + teaCupSizeZ/2)), color=teaCupColour)
    teaCupRightWall = Cuboid([teaCupWallThickness, teaCupSizeY, teaCupSizeZ], pose = SE3((teaCupLocationX + teaCupSizeX/2 + teaCupWallThickness/2),teaCupLocationY, (teaCupLocationZ + teaCupSizeZ/2)), color=teaCupColour)
    teaCupTopWall = Cuboid([teaCupSizeX, teaCupWallThickness, teaCupSizeZ], pose = SE3(teaCupLocationX,(teaCupLocationX + teaCupSizeX/2 - teaCupWallThickness/2), (teaCupLocationZ + teaCupSizeZ/2)), color=teaCupColour)
    teaCupBottomWall = Cuboid([teaCupSizeX, teaCupWallThickness, teaCupSizeZ], pose = SE3(teaCupLocationX,(teaCupLocationX - teaCupSizeX/2 + teaCupWallThickness/2), (teaCupLocationZ + teaCupSizeZ/2)), color=teaCupColour)

    env.add(teaCupBase)
    env.add(teaCupLeftWall)
    env.add(teaCupRightWall)
    env.add(teaCupTopWall)
    env.add(teaCupBottomWall)

# Create Swift environment
env = swift.Swift()
env.launch(realtime=True)

#setting camera pose
env.set_camera_pose([-3, 3, 3.5], [0.0, 0.0, 1.2])

#Calling all functions
constructBarrier()
sliders()
constructTeaCup()
selectors()

env.hold()
