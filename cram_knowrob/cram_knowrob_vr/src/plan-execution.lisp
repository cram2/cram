;;;
;;; Copyright (c) 2018, Alina Hawkin <hawkin@cs.uni-bremen.de>
;;;                     Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(defun execute-search-for-object (type)
  "Moves the robot to the position where the human was standing to grasp the object."
  (let ((?look-poses (look-poses-ll-for-searching type))
        (?base-poses (base-poses-ll-for-searching type))
        (?bullet-type (object-type-filter-bullet type)))
   (exe:perform
    (desig:an action
              (type searching)
              (object (desig:an object (type ?bullet-type)))
              (location (desig:a location (poses ?look-poses)))
              (robot-location (desig:a location (poses ?base-poses)))))))

(defun execute-pick-up-object (type)
  "Executes only the picking up action on an object given the type of the object.
TYPE: The type of the object. Could be 'muesli or 'cup etc. The name is internally
set to CupEcoOrange in a string."
  (let ((object-designator
          (execute-search-for-object type)))
    (let ((?bullet-type (object-type-filter-bullet type))
          (?base-poses (base-poses-ll-for-fetching-based-on-object-desig
                        object-designator))
          (?grasps (object-grasped-faces-ll-from-kvr-type type))
          (?arms (arms-for-fetching-ll type)))
      (exe:perform
       (desig:an action
                 (type fetching)
                 (object (desig:an object (type ?bullet-type)))
                 (arms ?arms)
                 (grasps ?grasps)
                 (robot-location (desig:a location (poses ?base-poses))))))))

(defun execute-place-object (?obj-desig type)
  "Executes the placing action given the object designator of the picked up and
held in hand object. The placing pose is the one used in VR for that kind of object.
?OBJ-DESIG: The object designator of the object the robot is currently holding
and which should be placed down."
  (let ((?placing-poses (object-poses-ll-for-placing type))
        (?base-poses (base-poses-ll-for-placing type)))
    (exe:perform
     (desig:an action
               (type delivering)
               (object ?obj-desig)
               (target (desig:a location (poses ?placing-poses)))
               (robot-location (desig:a location (poses ?base-poses)))))))

(defun execute-pick-and-place (type)
  "Executes the pick and place plan on an object of the given type.
The positions of where the robot looks for the object and where he is placing
it down are the ones extracted from Virtual Reality.
`type' is a simple symbol for the type of the object to transport, e.g., 'milk."
  (let ((?bullet-type (object-type-filter-bullet type))
        (?search-poses (look-poses-ll-for-searching type))
        (?search-base-poses (base-poses-ll-for-searching type))
        (?fetch-base-poses (base-poses-ll-for-searching type)
                           ;; (base-poses-ll-for-fetching-based-on-object-desig
                           ;;  object-designator)
                           )
        (?grasps (object-grasped-faces-ll-from-kvr-type type))
        (?arms (arms-for-fetching-ll type))
        (?delivering-poses (object-poses-ll-for-placing type))
        (?delivering-base-poses (base-poses-ll-for-placing type)))
    (exe:perform
     (desig:an action
               (type transporting)
               (object (desig:an object (type ?bullet-type)))
               (location (desig:a location (poses ?search-poses)))
               (search-robot-location (desig:a location (poses ?search-base-poses)))
               (fetch-robot-location (desig:a location (poses ?fetch-base-poses)))
               (arms ?arms)
               (grasps ?grasps)
               (target (desig:a location (poses ?delivering-poses)))
               (deliver-robot-location (desig:a location (poses ?delivering-base-poses)))))))
