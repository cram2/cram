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

(defun execute-pick-and-place (type)
  "Executes the pick and place plan on an object of the given type.
The positions of where the robot looks for the object and where he is placing
it down are the ones extracted from Virtual Reality.
TYPE: the given type of an object on which the pick and place action should be
executed.
RETURNS: "
  (transport
   (map-T-camera->map-P-base (umap-T-ucamera-through-object type "Start"))
   (umap-P-uobj-through-surface type "Start")
   (map-T-camera->map-P-base (umap-T-ucamera-through-surface type "End"))
   (umap-P-uobj-through-surface type "End")
   (umap-P-uobj-through-surface type "End")
   type))

(defun execute-pick-up-object (type)
  "Executes only the picking up action on an object given the type of the object.
TYPE: The type of the object. Could be 'muesli or 'cup etc. The name is internally
set to CupEcoOrange in a string.
RETURNS:"
  (fetch-object
   (map-T-camera->map-P-base (umap-T-ucamera-through-object type "Start"))
   (umap-P-uobj-through-surface type "Start")
   type))

(defun execute-place-object (?obj-desig type)
  "Executes the placing action given the object designator of the picked up and
held in hand object. The placing pose is the one used in VR for that kind of 
object.
?OBJ-DESIG: The object designator of the object the robot is currently holding
and which should be placed down."
  (place-object
   (map-T-camera->map-P-base (umap-T-ucamera-through-surface type "End"))
   (umap-P-uobj-through-surface type "End")
   (umap-P-uobj-through-surface type "End")
   ;; (cram-projection::projection-environment-result-result ?obj-desig)
   ?obj-desig))

(defun execute-move-to-object (type &optional (event-time :start))
  "Moves the robot to the position where the human was standing in order to
grasp the object or in order to place it."
  (ecase event-time
    (:start
     (move-to-object
      (map-T-camera->map-P-base (umap-T-ucamera-through-object type "Start"))
      (umap-P-uobj-through-surface type "Start")))
    (:end
     (move-to-object
      (map-T-camera->map-P-base (umap-T-ucamera-through-surface type "End"))
      (umap-P-uobj-through-surface type "End")))))
