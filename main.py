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
import threading

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
    # rangehood
    # water for tap
    # plug for the sink

# things that need to be done
    # get sliders, selectors and buttons to work
    # configure eStop to work
    # 

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

#Bench (including sink and tap)
benchSizeX = 4
benchSizeY = 1
benchSizeZ = 0.9

benchLocationX = 0
benchLocationY = -1.5

backSplashDepth = 0.1
backSplashHeight = 1.75

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

benchColour             = [166,128,100] #medium brown wood
backSplashColour        = (95, 75, 67) #dark brown wood
teaCupColour            = "White"
sinkColour              = (192, 192, 192) #silver
tapColour               = (192, 192, 192) #silver
barrierColour           = "Yellow"
barrierDoorColour       = (255, 200, 0) 
barrierDoorFrameColour  = "white"
stoveBaseColour         = (192, 192, 192) #silver
stoveDialColour         = (120, 112, 110) #dark silver

stoveColourBottomLeft   = (210, 210, 210) # light silver
stoveColourBottomRight  = (210, 210, 210) # light silver
stoveColourTopLeft      = (210, 210, 210) # light silver
stoveColourTopRight     = "red"

emergencyStopColourBase     = "yellow"
emergencyStopColourButton   = "red"

sinkWallThickness = 0.005

#Safety Barrier
barrierZoneX = [-2.5, 2.5]
barrierZoneY = [-2.5, 2.5]
barrierHeight = 2.25
barrierWidth = 0.025
barrierDoorWidth = 0.82
barrierDoorHeight = 2.040
barrierDoorFrameWidth = 0.05

barrierMiddleX = ((barrierZoneX[0]+barrierZoneX[1])/2)

#stove
    # might make into a kettle base later - depends how we want to make the tea
stoveRadius = 0.1
stoveHeight = 0.05
stoveBaseSizeX = 0.8
stoveBaseSizeY = 0.7

stoveLocationX = -1.4
stoveLocationY = -1.5

stoveSpacingX = 2
stoveSpacingY = 1.75

stoveDialRadius = 0.025
stoveDialHeight = 0.01

global stoveBottomLeft
global stoveBottomRight
global stoveTopLeft
global stoveTopRight

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
leftMostTeaBag = -0.25
gapBetweenTeaBagBoxes = 0.05
gapBetweenTeaBagsInBoxes = 0.01

teaBagBoxSizeX = 0.06
teaBagBoxSizeY = 0.15
teaBagBoxSizeZ = 0.05
teaBagBoxWallThickness = 0.001

teaBagsPerColumn = 14

# emergency Stop
eStopBaseSizeX = 0.05
eStopBaseSizeY = 0.025
eStopBaseSizeZ = 0.05

eStopBaseLocationX = [-1.95,    0,      1.95]
eStopBaseLocationY = [-1,       -1,     -1]
eStopBaseLocationZ = [0.85,     0.85,   0.85]

eStopButtonRadius = 0.0125
eStopButtonHeight = 0.0125

#GUI Sliders
setWaterTemperature = 0
amountOfTea = 0
proportionOfMilkInTea = 0

# GUI selectors
selectedTeaBag = ""
selectedMilkType = ""
selectedStove = ""

#GUI buttons
openDoor = bool(0)
begin = 0

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

stoves = ["Bottom Left", "Bottom Right", "Top Left", "Top Right"]

milkCartonWorkingNumber = 0
teaBagWorkingNumber = 0

milkTypeOptions = [milk[0] for milk in milkTypeOptionsWithColours]
teaBagOptions = [tea[0] for tea in teaBagOptionsWithColours]

def wait_for_enter(stop_event):
    input("Press Enter in this terminal when you are finished.\n")
    stop_event.set()

def buttons():
    env.add(swift.Button(lambda _: openBarrierDoor(1), desc="Open Door"))
    #env.add(swift.Button(begin, desc="START MAKING TEA!"))
    
def constructEmergencyStop():
    for n in range(len(eStopBaseLocationX)):
        emergencyStopBase = Cuboid([eStopBaseSizeX, eStopBaseSizeY, eStopBaseSizeZ], pose = SE3(eStopBaseLocationX[n], eStopBaseLocationY[n], eStopBaseLocationZ[n]), color=emergencyStopColourBase)
        emergencyStopButton = Cylinder(length = eStopButtonHeight, radius = eStopButtonRadius , pose = SE3(eStopBaseLocationX[n], eStopBaseLocationY[n] + eStopBaseSizeY/2 + eStopButtonHeight/2, eStopBaseLocationZ[n]) * SE3.Rx(pi / 2), color=emergencyStopColourButton)
        env.add(emergencyStopBase)
        env.add(emergencyStopButton)



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

    stoveSelector = swift.Select(
        selectedStove,
        options=stoves,
        desc="Select Stove"
    )
    env.add(stoveSelector)

def constructBarrier():
    barrierLengthX = abs(barrierZoneX[1]) + abs(barrierZoneX[0])
    barrierLengthY = abs(barrierZoneY[1]) + abs(barrierZoneY[0])

    barrierMiddleX = ((barrierZoneX[0]+barrierZoneX[1])/2)
    barrierMiddleY = ((barrierZoneY[0]+barrierZoneY[1])/2)

    barrierLeft = Cuboid([barrierWidth, barrierLengthY + barrierWidth, barrierHeight], pose = SE3(barrierZoneX[0], barrierMiddleY, (barrierHeight/2)), color=barrierColour)
    barrierRight = Cuboid([barrierWidth, barrierLengthY + barrierWidth, barrierHeight], pose = SE3(barrierZoneX[1], barrierMiddleY, (barrierHeight/2)), color=barrierColour)
    barrierTop = Cuboid([barrierLengthX + barrierWidth, barrierWidth, barrierHeight], pose = SE3(barrierMiddleX, barrierZoneY[0], (barrierHeight/2)), color=barrierColour)
    barrierBottomLeft = Cuboid([(barrierLengthX - barrierDoorWidth - 2*barrierDoorFrameWidth)/2, barrierWidth, barrierHeight], pose = SE3(-(barrierLengthX/2 - (barrierLengthX-barrierDoorWidth - 2*barrierDoorFrameWidth)/4), barrierZoneY[1], (barrierHeight/2)), color=barrierColour)
    barrierBottomRight = Cuboid([(barrierLengthX - barrierDoorWidth - 2*barrierDoorFrameWidth)/2, barrierWidth, barrierHeight], pose = SE3(barrierLengthX/2 - (barrierLengthX-barrierDoorWidth - 2*barrierDoorFrameWidth)/4, barrierZoneY[1], (barrierHeight/2)), color=barrierColour)
    barrierBottomTop = Cuboid([barrierDoorWidth + 2*barrierDoorFrameWidth, barrierWidth, barrierHeight - barrierDoorHeight - barrierDoorFrameWidth], pose = SE3(barrierMiddleX, barrierZoneY[1], barrierHeight - (barrierHeight-barrierDoorHeight-barrierDoorFrameWidth)/2), color=barrierColour)

    doorFrameLeft = Cuboid([barrierDoorFrameWidth, barrierWidth, barrierHeight - (barrierHeight - barrierDoorHeight - barrierDoorFrameWidth)], pose = SE3(-(barrierMiddleX - barrierDoorWidth/2 - barrierDoorFrameWidth/2), barrierZoneY[1],  ((barrierHeight-(barrierHeight - (barrierDoorHeight+barrierDoorFrameWidth)))/2)), color=barrierDoorFrameColour)
    doorFrameRight = Cuboid([barrierDoorFrameWidth, barrierWidth, barrierHeight - (barrierHeight - barrierDoorHeight - barrierDoorFrameWidth)], pose = SE3(barrierMiddleX - barrierDoorWidth/2 - barrierDoorFrameWidth/2, barrierZoneY[1], ((barrierHeight-(barrierHeight - (barrierDoorHeight+barrierDoorFrameWidth)))/2)), color=barrierDoorFrameColour)
    doorFrameTop = Cuboid([barrierDoorWidth, barrierWidth, barrierDoorFrameWidth], pose = SE3(barrierMiddleX, barrierZoneY[1], barrierHeight - (barrierHeight - barrierDoorHeight - barrierDoorFrameWidth/2)), color=barrierDoorFrameColour)

    env.add(barrierLeft)
    env.add(barrierRight)
    env.add(barrierTop)

    env.add(barrierBottomLeft)
    env.add(barrierBottomRight)
    env.add(barrierBottomTop)

    env.add(doorFrameLeft)
    env.add(doorFrameRight)
    env.add(doorFrameTop)

def constructBarrierDoor():
    global barrierDoor

    barrierDoor = Cuboid([barrierDoorWidth, barrierWidth, barrierDoorHeight], pose = (SE3(barrierMiddleX, barrierZoneY[1], barrierDoorHeight/2)), color=barrierDoorColour)

    env.add(barrierDoor)

def openBarrierDoor(status):
    global barrierDoor

    #env.remove(barrierDoor)

    if status == 1:
        doorAngle = pi/2
        displacementDoor = barrierDoorWidth/2
    else:
        doorAngle = 0
        displacementDoor = 0

    barrierDoor.pose = (
        SE3(barrierMiddleX, barrierZoneY[1], barrierDoorHeight / 2)
        * SE3.Rz(doorAngle)
        * SE3.Tx(displacementDoor)
        * SE3.Ty(-displacementDoor)
    )

    #env.add(barrierDoor)
    
def constructSink():
    sinkBase = Cuboid([sinkSizeX, sinkSizeY, sinkWallThickness], pose = SE3(sinkLocationX,sinkLocationY ,(benchSizeZ - (sinkSizeZ/2)+sinkWallThickness/2)), color=sinkColour)
    sinkLeftWall = Cuboid([sinkWallThickness, sinkSizeY, sinkSizeZ], pose = SE3((sinkLocationX - sinkSizeX/2 + sinkWallThickness/2),sinkLocationY, (benchSizeZ - (sinkSizeZ/2))), color=sinkColour)
    sinkRightWall = Cuboid([sinkWallThickness, sinkSizeY, sinkSizeZ], pose = SE3((sinkLocationX + sinkSizeX/2 - sinkWallThickness/2),sinkLocationY, (benchSizeZ - (sinkSizeZ/2))), color=sinkColour)
    sinkTopWall = Cuboid([sinkSizeX, sinkWallThickness, sinkSizeZ], pose = SE3(sinkLocationX,(sinkLocationY + sinkSizeY/2 - sinkWallThickness/2), (benchSizeZ - (sinkSizeZ/2))), color=sinkColour)
    sinkBottomWall = Cuboid([sinkSizeX, sinkWallThickness, sinkSizeZ], pose = SE3(sinkLocationX,(sinkLocationY - sinkSizeY/2 + sinkWallThickness/2), (benchSizeZ - (sinkSizeZ/2))), color=sinkColour)
    
    env.add(sinkBase)
    env.add(sinkLeftWall)
    env.add(sinkRightWall)
    env.add(sinkTopWall)
    env.add(sinkBottomWall)

def constructBacksplash():
    backSplashBack = Cuboid([benchSizeX + 2*backSplashDepth, backSplashDepth, backSplashHeight], pose = SE3(benchLocationX, benchLocationY - benchSizeY/2 - backSplashDepth/2, backSplashHeight/2), color= backSplashColour)
    backSplashLeft = Cuboid([backSplashDepth, benchSizeY, backSplashHeight], pose = SE3(benchLocationX - benchSizeX/2 - backSplashDepth/2, benchLocationY, backSplashHeight/2), color= backSplashColour)
    backSplashRight = Cuboid([backSplashDepth, benchSizeY, backSplashHeight], pose = SE3(-(benchLocationX - benchSizeX/2 - backSplashDepth/2), benchLocationY, backSplashHeight/2), color= backSplashColour)

    env.add(backSplashBack)
    env.add(backSplashLeft)
    env.add(backSplashRight)

def constructBench():
    benchBase = Cuboid([benchSizeX, benchSizeY, benchSizeZ-sinkSizeZ], pose = SE3(benchLocationX, benchLocationY, (benchSizeZ-sinkSizeZ)/2), color= benchColour)
    benchLeft = Cuboid([(benchLocationX + benchSizeX/2) - (sinkLocationX + sinkSizeX/2), benchSizeY, sinkSizeZ], pose = SE3(((benchLocationX + benchSizeX/2) + (sinkLocationX + sinkSizeX/2))/2, benchLocationY, benchSizeZ - sinkSizeZ/2), color= benchColour)
    benchRight = Cuboid([(benchLocationX + benchSizeX/2) + (sinkLocationX - sinkSizeX/2), benchSizeY, sinkSizeZ], pose = SE3(-((benchLocationX + benchSizeX/2) - (sinkLocationX - sinkSizeX/2))/2, benchLocationY, benchSizeZ - sinkSizeZ/2), color= benchColour)
    benchFront = Cuboid([benchSizeX, (benchLocationY + benchSizeY/2) - (sinkLocationY + sinkSizeY/2), sinkSizeZ], pose = SE3(benchLocationX, ((benchLocationY + benchSizeY/2) + (sinkLocationY + sinkSizeY/2))/2, benchSizeZ - sinkSizeZ/2), color= benchColour)
    benchBack = Cuboid([sinkSizeX, -(-(benchLocationY + benchSizeY/2) + (sinkLocationY + sinkSizeY/2)), sinkSizeZ], pose = SE3(sinkLocationX, ((benchLocationY - benchSizeY/2) + (sinkLocationY - sinkSizeY/2))/2, benchSizeZ - sinkSizeZ/2 + 0.000001), color= benchColour)

    env.add(benchBase)
    env.add(benchLeft)
    env.add(benchRight)
    env.add(benchFront)
    env.add(benchBack)

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
    stoveBase = Cuboid([stoveBaseSizeX, stoveBaseSizeY, 0.001 ], pose = SE3(stoveLocationX, stoveLocationY, benchSizeZ), color= stoveBaseColour)

    stoveBottomLeft = Cylinder(length=stoveHeight, radius=stoveRadius, pose=SE3(stoveLocationX + stoveSpacingX * stoveRadius, stoveLocationY + stoveSpacingY * stoveRadius, benchSizeZ - stoveHeight/2 + 0.0011), color=stoveColourBottomLeft)
    stoveBottomRight = Cylinder(length=stoveHeight, radius=stoveRadius, pose=SE3(stoveLocationX - stoveSpacingX * stoveRadius, stoveLocationY + stoveSpacingY * stoveRadius, benchSizeZ- stoveHeight/2 + 0.0011), color=stoveColourBottomRight)
    stoveTopLeft = Cylinder(length=stoveHeight, radius=stoveRadius, pose=SE3(stoveLocationX + stoveSpacingX * stoveRadius, stoveLocationY - stoveSpacingY * stoveRadius, benchSizeZ- stoveHeight/2 + 0.0011), color=stoveColourTopLeft)
    stoveTopRight = Cylinder(length=stoveHeight, radius=stoveRadius, pose=SE3(stoveLocationX - stoveSpacingX * stoveRadius, stoveLocationY - stoveSpacingY * stoveRadius, benchSizeZ- stoveHeight/2 + 0.0011), color=stoveColourTopRight)

    stoveDial = Cylinder(length=stoveDialHeight, radius=stoveDialRadius, pose=SE3(stoveLocationX, stoveLocationY + stoveSpacingX * 1.4 * stoveRadius, benchSizeZ + stoveDialHeight/2), color=stoveDialColour)

    env.add(stoveDial)
    env.add(stoveBase)
    env.add(stoveTopLeft)
    env.add(stoveTopRight)
    env.add(stoveBottomLeft)
    env.add(stoveBottomRight)

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
#make better stove

#MAKE SURE THE TEABAGS ARE SMALLER THAN THE TEACUP

# Create Swift environment
env = swift.Swift()
env.launch(realtime=True)

#setting camera pose
env.set_camera_pose([-3, 3, 3.5], [0.0, -2, 0])

def constructObjects():
    constructBarrier()

    constructBarrierDoor()

    constructSink()
    constructBacksplash()
    constructStove()
    constructTeaCup()
    constructBench()
    constructMilkCartons()
    constructTap()
    constructTeaBags()
    constructEmergencyStop()

#Calling all functions
constructObjects()
sliders()
selectors()
buttons()
openBarrierDoor(1)

#OPENBARRIERDOORISBROKEN
#OPENBARRIERDOORISBROKEN
#OPENBARRIERDOORISBROKEN
#OPENBARRIERDOORISBROKEN
#OPENBARRIERDOORISBROKEN
#OPENBARRIERDOORISBROKEN
#OPENBARRIERDOORISBROKEN
#OPENBARRIERDOORISBROKEN
#OPENBARRIERDOORISBROKEN
#OPENBARRIERDOORISBROKEN

env.hold()

openBarrierDoor(openDoor)
x=1

while x == 1:
    
    env.step(0.03)
    print(openDoor)



'''
stop_event = threading.Event()
threading.Thread(target=wait_for_enter, args=(stop_event,), daemon=True).start()


try:
    if x == 1:
        buttons()
        openBarrierDoor(openDoor)
        env.step(0.03)
#except KeyboardInterrupt:
#    pass
finally:
    env.close()
'''

#MAKE THE TEA COLOUR SLOWLY FADE INTO THE TEA
#a lot of the objects wont stay in expected spots if flipped around axis, could fix this, depends on what assesment needs

env.hold()
