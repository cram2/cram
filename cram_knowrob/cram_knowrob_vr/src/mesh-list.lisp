;;;
;;; Copyright (c) 2018, Alina Hawkin <hawkin@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :kvr)

(defparameter *mesh-path-whitelist-unreal-kitchen*
  '("package://robcog/Oven.dae"
    "package://robcog/FridgeDoor.dae"
    "package://robcog/FridgeDoor.dae"
    "package://robcog/FridgeDoor.dae"
    "package://robcog/FridgeDrawer.dae"
    "package://robcog/OvenDoor.dae"
    "package://robcog/OvenDrawerMiddleBottom.dae"
    "package://robcog/OvenDrawerMiddleTop.dae"
    "package://robcog/Oven.dae"
    "package://robcog/SinkDrawerLeftBottom.dae"
    "package://robcog/SinkDrawerLeftMiddle.dae"
    "package://robcog/SinkDrawerLeftTop.dae"
    "package://robcog/BaerenMarkeFrischeAlpenmilch18.dae"
    "package://robcog/BaerenMarkeFrischeAlpenmilch38.dae"
    "package://robcog/BaerenMarkeFrischeKakao.dae"
    "package://robcog/BowlLarge.dae"
    "package://robcog/BowlSmall.dae"
    "package://robcog/Bowl.dae"
    "package://robcog/BrandtVollkornZwieback.dae"
    "package://robcog/ButtonCooktopBottomLeft.dae"
    "package://robcog/ButtonCooktopBottomRight.dae"
    "package://robcog/ButtonCooktopTopLeft.dae"
    "package://robcog/ButtonCooktopTopRight.dae"
    "package://robcog/ButtonOven.dae"
;;   "package://robcog/CeilingLight.dae"
;;   "package://robcog/CharacterCamera.dae"
    "package://robcog/CooktopHeatingElementBottomLeft.dae"
    "package://robcog/CooktopHeatingElementBottomRight.dae"
    "package://robcog/CooktopHeatingElementTopLeft.dae"
    "package://robcog/CooktopHeatingElementTopRight.dae"
    "package://robcog/Cooktop.dae"
;;    "package://robcog/CupEcoOrange.dae"
    "package://robcog/CupEspresso.dae"
    "package://robcog/Cup.dae"
    "package://robcog/DiningTable.dae"
    "package://robcog/DishwasherDoorHandle.dae"
    "package://robcog/DishwasherDoor.dae"
    "package://robcog/DishwasherLowerRack.dae"
    "package://robcog/DishwasherUpperRack.dae"
    "package://robcog/Dishwasher.dae"
;;    "package://robcog/DoubleWindow.dae"
    "package://robcog/FaucetBody.dae"
    "package://robcog/FaucetHandle.dae"
    "package://robcog/FaucetSpout.dae"
    "package://robcog/ForkDessert.dae"
    "package://robcog/ForkDinner.dae"
    "package://robcog/FridgeArea.dae"
    "package://robcog/FridgeDoorBottomShelf.dae"
    "package://robcog/FridgeDoorHandle.dae"
    "package://robcog/FridgeDoorTopShelf.dae"
    "package://robcog/FridgeDoor.dae"
    "package://robcog/FridgeDrawerHandle.dae"
    "package://robcog/FridgeDrawer.dae"
    "package://robcog/FridgeGlassShelf.dae"
    "package://robcog/FridgeHumidityDrawer.dae"
    "package://robcog/FridgeShelf.dae"
    "package://robcog/Fridge.dae"
    "package://robcog/GlassRedwine.dae"
    "package://robcog/GlassRound.dae"
    "package://robcog/GlassTall.dae"
    "package://robcog/GlassWhitewine.dae"
    "package://robcog/HohesCOrange.dae"
;;    "package://robcog/IkeaBowl.dae"
    "package://robcog/IslandArea.dae"
    "package://robcog/IslandDrawerBottomLeftHandle.dae"
    "package://robcog/IslandDrawerBottomLeft.dae"
    "package://robcog/IslandDrawerBottomMiddleHandle.dae"
    "package://robcog/IslandDrawerBottomMiddle.dae"
    "package://robcog/IslandDrawerBottomRightHandle.dae"
    "package://robcog/IslandDrawerBottomRight.dae"
    "package://robcog/IslandDrawerTopLeftHandle.dae"
    "package://robcog/IslandDrawerTopLeft.dae"
    "package://robcog/IslandDrawerTopMiddleHandle.dae"
    "package://robcog/IslandDrawerTopMiddle.dae"
    "package://robcog/IslandDrawerTopRightHandle.dae"
    "package://robcog/IslandDrawerTopRight.dae"
    "package://robcog/JaNougatBits.dae"
    "package://robcog/KellogsCornFlakesOriginal.dae"
    "package://robcog/KitchenSink.dae"
    "package://robcog/KnifeButter.dae"
    "package://robcog/KnifeTable.dae"
    "package://robcog/KoellnMuesliCranberry.dae"
;;    "package://robcog/KoellnMuesliKnusperHonigNuss.dae"
;;    "package://robcog/LabFloor.dae"
;;    "package://robcog/LabWalls.dae"
;;    "package://robcog/LargeDoubleWindow.dae"
;;    "package://robcog/LargeSingleWindow.dae"
;;    "package://robcog/LeftHand.dae"
;;    "package://robcog/MilramButtermilchErdbeere.dae"
    "package://robcog/MuellerFruchtButterMilchErdbeere.dae"
    "package://robcog/MuellerFruchtButterMilchHimbeere.dae"
    "package://robcog/MuellerFruchtButterMilchMultiVitamin.dae"
    "package://robcog/MuellerReineButterMilch.dae"
    "package://robcog/OvenArea.dae"
    "package://robcog/OvenDoorHandle.dae"
    "package://robcog/OvenDoor.dae"
    "package://robcog/OvenDrawerLeftHandle.dae"
    "package://robcog/OvenDrawerLeft.dae"
    "package://robcog/OvenDrawerMiddleBottomHandle.dae"
    "package://robcog/OvenDrawerMiddleBottom.dae"
    "package://robcog/OvenDrawerMiddleTopHandle.dae"
    "package://robcog/OvenDrawerMiddleTop.dae"
    "package://robcog/OvenDrawerRightHandle.dae"
    "package://robcog/OvenDrawerRight.dae"
    "package://robcog/OvenRack.dae"
    "package://robcog/Oven.dae"
;;    "package://robcog/PlasticBlueFork.dae"
    "package://robcog/PlateClassic16.dae"
    "package://robcog/PlateClassic26.dae"
    "package://robcog/PlateClassic28.dae"
;;    "package://robcog/RightHand.dae"
    "package://robcog/Saucer.dae"
    "package://robcog/SinkAirGap.dae"
    "package://robcog/SinkArea.dae"
    "package://robcog/SinkDrawerLeftBottomHandle.dae"
    "package://robcog/SinkDrawerLeftBottom.dae"
    "package://robcog/SinkDrawerLeftMiddleHandle.dae"
    "package://robcog/SinkDrawerLeftMiddle.dae"
    "package://robcog/SinkDrawerLeftTopHandle.dae"
    "package://robcog/SinkDrawerLeftTop.dae"
    "package://robcog/SinkStrainer.dae"
    "package://robcog/SpoonDessert.dae"
    "package://robcog/SpoonSoup.dae"
    "package://robcog/TrashDrawerHandle.dae"
    "package://robcog/TrashDrawer.dae"
    "package://robcog/Tray.dae"
    "package://robcog/WasaDelicateCrisp.dae"))
