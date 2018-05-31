;;;
;;; Copyright (c) 2018, Christopher Pollok <cpollok@uni-bremen.de>
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

(in-package :pr2-emd)

(defparameter *container*
  '(:fridge
    :fridge-drawer
    :island-left-drawer
    :island-middle-drawer
    :island-right-drawer
    :trash-drawer
    :dishwasher
    :kitchen-drawer
    :oven
    :oven-left-drawer
    :oven-right-drawer
    :oven-drawer
    ))

(defparameter *container-links*
  '((:fridge . "iai_fridge_main")
    (:fridge-drawer . "fridge_area_lower_drawer_main")
    (:island-left-drawer . "kitchen_island_left_upper_drawer_main")
    (:island-middle-drawer . "kitchen_island_middle_upper_drawer_main")
    (:island-right-drawer . "kitchen_island_right_upper_drawer_main")
    (:trash-drawer . "sink_area_trash_drawer_main")
    (:dishwasher . "sink_area_dish_washer_door_main")
    (:kitchen-drawer . "sink_area_left_upper_drawer_main")
    (:oven . "oven_area_oven_door_main")
    (:oven-left-drawer . "oven_area_area_left_drawer_main")
    (:oven-right-drawer . "oven_area_area_right_drawer_main")
    (:oven-drawer . "oven_area_area_middle_upper_drawer_main")
    ))

(defparameter *container-link-syms*
  '((:fridge-drawer . 'fridge_area_lower_drawer_main)
    (:island-left-drawer . 'kitchen_island_left_upper_drawer_main)
    (:island-middle-drawer . 'kitchen_island_middle_upper_drawer_main)
    (:island-right-drawer . 'kitchen_island_right_upper_drawer_main)
    (:trash-drawer . 'sink_area_trash_drawer_main)
    (:kitchen-drawer . 'sink_area_left_upper_drawer_main)
    (:oven-left-drawer . 'oven_area_area_left_drawer_main)
    (:oven-right-drawer . 'oven_area_area_right_drawer_main)
    (:oven-drawer . 'oven_area_area_middle_upper_drawer_main)
    ))

(defparameter *container-handle-links*
  '((:fridge . "iai_fridge_door_handle")
    (:fridge-drawer . "fridge_area_lower_drawer_drawer_handle")
    (:island-left-drawer . "kitchen_island_left_upper_drawer_handle")
    (:island-middle-drawer . "kitchen_island_middle_upper_drawer_handle")
    (:island-right-drawer . "kitchen_island_right_upper_drawer_handle")
    (:trash-drawer . "sink_area_trash_drawer_handle")
    (:dishwasher . "sink_area_dish_washer_door_handle")
    (:kitchen-drawer . "sink_area_left_upper_drawer_handle")
    (:oven . "oven_area_oven_door_handle")
    (:oven-left-drawer . "oven_area_area_left_drawer_handle")
    (:oven-right-drawer . "oven_area_area_right_drawer_handle")
    (:oven-drawer . "oven_area_area_middle_upper_drawer_handle")
    ))

(defparameter *container-joints*
  '((:fridge . "iai_fridge_door_joint")
    (:fridge-drawer . "fridge_area_lower_drawer_main_joint")
    (:island-left-drawer . "kitchen_island_left_upper_drawer_main_joint")
    (:island-middle-drawer . "kitchen_island_middle_upper_drawer_main_joint")
    (:island-right-drawer . "kitchen_island_right_upper_drawer_main_joint")
    (:trash-drawer . "sink_area_trash_drawer_main_joint")
    (:dishwasher . "sink_area_dish_washer_door_joint")
    (:kitchen-drawer . "sink_area_left_upper_drawer_main_joint")
    (:oven . "oven_area_oven_door_joint")
    (:oven-left-drawer . "oven_area_area_left_drawer_main_joint")
    (:oven-right-drawer . "oven_area_area_right_drawer_main_joint")
    (:oven-drawer . "oven_area_area_middle_upper_drawer_main_joint")
    ))

(defparameter *container-angles*
  `((:fridge . ,(/ pi 2))
    (:fridge-drawer . 0.4)
    (:island-left-drawer . 0.4)
    (:island-middle-drawer . 0.4)
    (:island-right-drawer . 0.4)
    (:trash-drawer . 0.4)
    (:dishwasher . ,(/ pi 2))
    (:kitchen-drawer . 0.48)
    (:oven . ,(/ pi 2))
    (:oven-left-drawer . 0.4)
    (:oven-right-drawer . 0.4)
    (:oven-drawer . 0.4)
    ))


