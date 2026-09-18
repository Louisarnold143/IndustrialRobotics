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

#might optimise vairables use so that location is where it starts not the middle as it is by default in Cuboid()

#Scene objects
#objects that STILL need to be made
    # fridge
    # eStop
    # could also maybe do different colour/style cups
    # JAKA MiniCobo
    # backsplash

# = Cuboid([, , ], pose = SE3(, , ), color= )

#Tea Cup
#adding a handle to the Tea Cup might be a good idea, might do later
teaCupSizeX = 0.05
teaCupSizeY = 0.05
teaCupSizeZ = 0.1
teaCupWallThickness = 0.005

teaCupLocationX = 0.75
teaCupLocationY = -1.5
teaCupLocationZ = 0.9
teaCupColour = "White"

#Bench (including sink and tap)
benchSizeX = 4
benchSizeY = 1
benchSizeZ = 0.9

benchLocationX = 0
benchLocationY = -1.5

backSplashDepth = 0.1
backSplashHeight = 1.75

backSplashColour = (95, 75, 67)

sinkSizeX = 0.4
sinkSizeY = 0.4
sinkSizeZ = 0.2

sinkLocationX = 1.5
sinkLocationY = -1.5

tapSetbackFromsink = 0.05

tapRadius = 0.025
tapBaseHeight = 0.2
tapTipLength = 0.3
#could do a thing where if the tap tip isnt made inside the sink it shits itself

#put all the colour variables together
#put all the colour variables together
#put all the colour variables together
#put all the colour variables together
#put all the colour variables together
#put all the colour variables together

benchColour = [166,128,100]
sinkColour = "White"
tapColour = "White"

#Safety Barrier
barrierZoneX = [-2.5, 2.7]
barrierZoneY = [-2.5, 2.5]
barrierHeight = 1.5
barrierWidth = 0.025

barrierColour = "Yellow"

#stove
    # might make into a kettle base later - depends how we want to make the tea
stoveRadius = 0.175
stoveHeight = 0.05
stoveColour = "Red"

stoveLocationX = -1.5
stoveLocationY = -1.5

#milk carton
milkCartonSizeX = 0.07
milkCartonSizeY = 0.07
milkCartonSizeZ = 0.2
leftMostMilkCarton = 1.5
gapBetweenMilks = 0.1

#tea bags 
#will make a better looking tea bag later, plus boxes, placeholder atm
teaBagBagSizeX = 0.03
teaBagBagSizeY = 0.005
teaBagBagSizeZ = 0.05
leftMostTeaBag = -0.5
gapBetweenTeaBagBoxes = 0.05
gapBetweenTeaBagsInBoxes = 0.01

teaBagBoxSizeX = 0.06
teaBagBoxSizeY = 0.15
teaBagBoxSizeZ = 0.05
teaBagBoxWallThickness = 0.001

teaBagsPerColumn = 14

#GUI Sliders
setWaterTemperature = 0
amountOfTea = 0
proportionOfMilkInTea = 0

# GUI selectors
selectedTeaBag = ""
selectedMilkType = ""

teaBagOptionsWithColours = [["Earl Gray"            , "yellow"],
                            ["English Breakfast"    , "red"],
                            ["Peppermint"           , "green"],
                            ["Lemon and Ginger"     , "orange"],
                            ["Camomile"             , "white"]]

milkTypeOptionsWithColours = [["Full Cream Milk", "blue"],
                              ["Low Fat Milk"   , "cyan"],
                              ["Skim Milk"      , "black"],
                              ["Almond Milk"    , "orange"],
                              ["Oat Milk"       , "white"],
                              ["Rice Milk"      , "yellow"]]

milkCartonWorkingNumber = 0
teaBagWorkingNumber = 0

milkTypeOptions = [milk[0] for milk in milkTypeOptionsWithColours]
teaBagOptions = [tea[0] for tea in teaBagOptionsWithColours]

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

def constructBench():
    backSplashBack = Cuboid([benchSizeX + 2*backSplashDepth, backSplashDepth, backSplashHeight], pose = SE3(benchLocationX, benchLocationY - benchSizeY/2 - backSplashDepth/2, backSplashHeight/2), color= backSplashColour)
    backSplashLeft = Cuboid([backSplashDepth, benchSizeY, backSplashHeight], pose = SE3(benchLocationX - benchSizeX/2 - backSplashDepth/2, benchLocationY, backSplashHeight/2), color= backSplashColour)
    backSplashRight = Cuboid([backSplashDepth, benchSizeY, backSplashHeight], pose = SE3(-(benchLocationX - benchSizeX/2 - backSplashDepth/2), benchLocationY, backSplashHeight/2), color= backSplashColour)

    benchBase = Cuboid([benchSizeX, benchSizeY, benchSizeZ-sinkSizeZ], pose = SE3(benchLocationX, benchLocationY, (benchSizeZ-sinkSizeZ)/2), color= benchColour)

    benchLeft = Cuboid([(benchLocationX + benchSizeX/2) - (sinkLocationX + sinkSizeX/2), benchSizeY, sinkSizeZ], pose = SE3(((benchLocationX + benchSizeX/2) + (sinkLocationX + sinkSizeX/2))/2, benchLocationY, benchSizeZ - sinkSizeZ/2), color= benchColour)

    benchRight = Cuboid([(benchLocationX + benchSizeX/2) + (sinkLocationX - sinkSizeX/2), benchSizeY, sinkSizeZ], pose = SE3(-((benchLocationX + benchSizeX/2) - (sinkLocationX - sinkSizeX/2))/2, benchLocationY, benchSizeZ - sinkSizeZ/2), color= benchColour)

    benchFront = Cuboid([benchSizeX, (benchLocationY + benchSizeY/2) - (sinkLocationY + sinkSizeY/2), sinkSizeZ], pose = SE3(benchLocationX, ((benchLocationY + benchSizeY/2) + (sinkLocationY + sinkSizeY/2))/2, benchSizeZ - sinkSizeZ/2), color= benchColour)

    benchBack = Cuboid([sinkSizeX, -(-(benchLocationY + benchSizeY/2) + (sinkLocationY + sinkSizeY/2)), sinkSizeZ], pose = SE3(sinkLocationX, ((benchLocationY - benchSizeY/2) + (sinkLocationY - sinkSizeY/2))/2, benchSizeZ - sinkSizeZ/2 + 0.000001), color= benchColour)

    print(-(benchLocationY + benchSizeY/2) + (sinkLocationY + sinkSizeY/2))
    env.add(benchBase)
    env.add(benchLeft)
    env.add(benchRight)
    env.add(benchFront)
    env.add(benchBack)
    
    env.add(backSplashBack)
    env.add(backSplashLeft)
    env.add(backSplashRight)

    #MAKE A LINING FOR THE SINK AND MAKE IT SILVER
    #MAKE SURE THE TEABAGS ARE SMALLER THAN THE TEACUP


def constructTap():
    tapBase = Cylinder(length=tapBaseHeight, radius=tapRadius, pose=SE3(sinkLocationX, sinkLocationY - sinkSizeY/2 - tapSetbackFromsink, benchSizeZ + tapBaseHeight/2), color=tapColour)
    tapTip = Cylinder(length=tapTipLength, radius=tapRadius, pose=SE3(sinkLocationX, sinkLocationY - sinkSizeY/2 - tapSetbackFromsink + tapTipLength/2 , benchSizeZ + tapBaseHeight)* SE3.Rx(pi / 2), color=tapColour)
    tapTop =Cuboid([tapRadius*2, tapRadius*2, tapRadius*2], pose = SE3(sinkLocationX, sinkLocationY - sinkSizeY/2 - tapSetbackFromsink, benchSizeZ + tapBaseHeight), color= tapColour)
    env.add(tapBase)
    env.add(tapTip)
    env.add(tapTop)

def constructTeaCup():
    teaCupBase = Cuboid([teaCupSizeX, teaCupSizeY, teaCupWallThickness], pose = SE3(teaCupLocationX,teaCupLocationY ,(teaCupLocationZ+teaCupWallThickness/2)), color=teaCupColour)
    teaCupLeftWall = Cuboid([teaCupWallThickness, teaCupSizeY, teaCupSizeZ], pose = SE3((teaCupLocationX - teaCupSizeX/2 + teaCupWallThickness/2),teaCupLocationY, (teaCupLocationZ + teaCupSizeZ/2)), color=teaCupColour)
    teaCupRightWall = Cuboid([teaCupWallThickness, teaCupSizeY, teaCupSizeZ], pose = SE3((teaCupLocationX + teaCupSizeX/2 + teaCupWallThickness/2),teaCupLocationY, (teaCupLocationZ + teaCupSizeZ/2)), color=teaCupColour)
    teaCupTopWall = Cuboid([teaCupSizeX, teaCupWallThickness, teaCupSizeZ], pose = SE3(teaCupLocationX,(teaCupLocationY + teaCupSizeY/2 - teaCupWallThickness/2), (teaCupLocationZ + teaCupSizeZ/2)), color=teaCupColour)
    teaCupBottomWall = Cuboid([teaCupSizeX, teaCupWallThickness, teaCupSizeZ], pose = SE3(teaCupLocationX,(teaCupLocationY - teaCupSizeY/2 + teaCupWallThickness/2), (teaCupLocationZ + teaCupSizeZ/2)), color=teaCupColour)

    env.add(teaCupBase)
    env.add(teaCupLeftWall)
    env.add(teaCupRightWall)
    env.add(teaCupTopWall)
    env.add(teaCupBottomWall)

def constructStove():
    stove = Cylinder(length=stoveHeight, radius=stoveRadius, pose=SE3(stoveLocationX, stoveLocationY, benchSizeZ+stoveHeight/2), color=stoveColour)
    env.add(stove)

def constructTeaBagBox(teaBagBoxLocationX, teaBagBoxLocationY, teaBagBoxLocationZ, teaBagBoxColour):
    teaBagBoxBase = Cuboid([teaBagBoxSizeX, teaBagBoxSizeY, teaBagBoxWallThickness], pose = SE3(teaBagBoxLocationX,teaBagBoxLocationY ,(teaBagBoxLocationZ+teaBagBoxWallThickness/2)), color=teaBagBoxColour)
    teaBagBoxLeftWall = Cuboid([teaBagBoxWallThickness, teaBagBoxSizeY, teaBagBoxSizeZ], pose = SE3((teaBagBoxLocationX - teaBagBoxSizeX/2 + teaBagBoxWallThickness/2),teaBagBoxLocationY, (teaBagBoxLocationZ + teaBagBoxSizeZ/2)), color=teaBagBoxColour)
    teaBagBoxRightWall = Cuboid([teaBagBoxWallThickness, teaBagBoxSizeY, teaBagBoxSizeZ], pose = SE3((teaBagBoxLocationX + teaBagBoxSizeX/2 + teaBagBoxWallThickness/2),teaBagBoxLocationY, (teaBagBoxLocationZ + teaBagBoxSizeZ/2)), color=teaBagBoxColour)
    teaBagBoxTopWall = Cuboid([teaBagBoxSizeX, teaBagBoxWallThickness, teaBagBoxSizeZ], pose = SE3(teaBagBoxLocationX,(teaBagBoxLocationY + teaBagBoxSizeY/2 - teaBagBoxWallThickness/2), (teaBagBoxLocationZ + teaBagBoxSizeZ/2)), color=teaBagBoxColour)
    teaBagBoxBottomWall = Cuboid([teaBagBoxSizeX, teaBagBoxWallThickness, teaBagBoxSizeZ], pose = SE3(teaBagBoxLocationX,(teaBagBoxLocationY - teaBagBoxSizeY/2 + teaBagBoxWallThickness/2), (teaBagBoxLocationZ + teaBagBoxSizeZ/2)), color=teaBagBoxColour)

    env.add(teaBagBoxBase)
    env.add(teaBagBoxLeftWall)
    env.add(teaBagBoxRightWall)
    env.add(teaBagBoxTopWall)
    env.add(teaBagBoxBottomWall)

def milkCartonMaker(milkType, cartonColour):
    milkType = Cuboid([milkCartonSizeX, milkCartonSizeY, milkCartonSizeZ], pose = SE3(leftMostMilkCarton - milkCartonWorkingNumber*(milkCartonSizeX+gapBetweenMilks), benchLocationY-benchSizeY/2 + milkCartonSizeY/2, benchSizeZ+milkCartonSizeZ/2), color= cartonColour)
    env.add(milkType)

def teaBagMaker(teaBag, bagColour):
    constructTeaBagBox(teaBagBoxLocationX = leftMostTeaBag - teaBagWorkingNumber*(teaBagBoxSizeX+gapBetweenTeaBagBoxes),
                       teaBagBoxLocationY = benchLocationY-benchSizeY/2 + teaBagBoxSizeY/2,
                       teaBagBoxLocationZ = benchSizeZ,
                       teaBagBoxColour = bagColour)
    for i in range(teaBagsPerColumn):    
        teaBag = Cuboid([teaBagBagSizeX, teaBagBagSizeY, teaBagBagSizeZ], pose = SE3(leftMostTeaBag - teaBagWorkingNumber*(teaBagBoxSizeX+gapBetweenTeaBagBoxes), benchLocationY-benchSizeY/2 + gapBetweenTeaBagsInBoxes + i*gapBetweenTeaBagsInBoxes, benchSizeZ + teaBagBagSizeZ/2), color= bagColour)
        env.add(teaBag)

def constructMilkCartons():
    global milkCartonWorkingNumber
    for row in milkTypeOptionsWithColours:
        milkCartonMaker(milkType = row[0], cartonColour = row[1])
        milkCartonWorkingNumber = milkCartonWorkingNumber + 1
        print(milkCartonWorkingNumber)

def constructTeaBags():
    global teaBagWorkingNumber
    for row in teaBagOptionsWithColours:
        teaBagMaker(teaBag = row[0], bagColour = row[1])
        teaBagWorkingNumber = teaBagWorkingNumber + 1
        print(teaBagWorkingNumber)

#make some function that generates a cuboid in the teacup the height of amountOfTea, 
# and maybe change the colour according to tea type and milk type 
# (maybe take into account transperacy of the cup too)

# Create Swift environment
env = swift.Swift()
env.launch(realtime=True)

#setting camera pose
env.set_camera_pose([-3, 3, 3.5], [0.0, -2, 0])

def constructObjects():
    #constructBarrier()
    #PUT THE BARRIER BACK INNNNN
    constructStove()
    constructTeaCup()
    constructBench()
    constructMilkCartons()
    constructTap()
    constructTeaBags()

#Calling all functions
constructObjects()
sliders()
selectors()

#MAKE THE TEA COLOUR SLOWLY FADE INTO THE TEA
#a lot of the objects wont stay in expected spots if flipped around axis, could fix this, depends on what assesment needs

env.hold()
