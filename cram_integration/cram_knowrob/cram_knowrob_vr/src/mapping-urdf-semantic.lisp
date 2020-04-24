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

;;; Mapping from the object of the urdf kitchen to the semantic map.
;;; The semantic map kitchen is the one in OpenEase where the data is from.
;;; The URDF kitchen is the one the robot interacts with in the bullet world.
;;; The semantic map has a lot more objects then the URDF since all the cuttlery
;;; the robot interacts with is not present in the semantic map but spawned
;;; additionally.
(in-package :kvr)

(defparameter *semantic-map-offset-x* 0.0
  "Used in spawning semantic map and offsetting its objects")
(defparameter *semantic-map-offset-y* -3.0
  "Used in spawning semantic map and offsetting its objects")

(defparameter  *semantic-to-urdf*
  '((|''IslandArea''| . :|ENVIRONMENT.kitchen_island|)
    (|''SinkArea''| . :|ENVIRONMENT.sink_area|))
  "semantic map . urdf")

(defun match-kitchens (name)
  (cdr (assoc name *semantic-to-urdf*)))

(defun get-all-objects-urdf ()
  (btr:rigid-body-names
   (btr:get-environment-object)))

(defun get-all-objects-semantic ()
  (btr:rigid-body-names
   (btr:object btr:*current-bullet-world* :semantic-map-kitchen)))


(defun object-type-filter-prolog (object-type)
  "Maps the simple name of an object, e.g. cup to the one known in the database
for that object, e.g. CupEcoOrange."
  (if (stringp object-type)
      object-type
      (ecase object-type
        (muesli "KoellnMuesliKnusperHonigNuss")
        (cup "CupEcoOrange")
        (bowl "IkeaBowl")
        (milk "MilramButtermilchErdbeere")
        (fork "PlasticBlueFork")
        (spoon "PlasticBlueSpoon"))))

(defun object-type-filter-bullet (object-type)
  "Maps the simple name of an object, e.g. cup to the one known in the database
for that object, e.g. CupEcoOrange."
  (ecase object-type
    (muesli :breakfast-cereal)
    (cup :cup)
    (bowl :bowl)
    (milk :milk)
    (fork :fork)
    (spoon :spoon)))

(defun object-type-fixer (object-type)
  "Takes care of the few cases where the name of the object within the recorded
VR data and the object within the bullet world, are different."
  (case object-type
    (:milk :milram-buttermilch-erdbeere)
    (:bowl :ikea-bowl)
    (:fork :plastic-blue-fork)
    (:spoon :plastic-blue-spoon)
    (:breakfast-cereal :koelln-muesli-knusper-honig-nuss)
    (:cup :cup-eco-orange)
    (t object-type)))
