;;;
;;; Copyright (c) 2019, Thomas Lipps <tlipps@uni-bremen.de>
;;;
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

(in-package :learning-vr)

(defvar *learning-vr-on* T)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; GET-SYMBOLIC-LOCATION ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; Helper function

(defun location-on-p (urdf-name)
  (or (equal urdf-name :kitchen-island-surface)
      (equal urdf-name :kitchen-island)
      (equal urdf-name :sink-area)
      (equal urdf-name :oven-area-area)))


;; Translates the OWL Names of the environment objects returned from
;; service GetSymbolicLocation into its URDF names.

(let ((owl->urdf-table (make-hash-table :test 'equalp)))
  (setf (gethash "SinkDrawerLeftTop" owl->urdf-table)
        :sink-area-left-upper-drawer-main
        (gethash "IslandDrawerBottomLeft" owl->urdf-table)
        :kitchen-island-left-lower-drawer-main
        (gethash "IslandDrawerBottomMiddle" owl->urdf-table)
        :kitchen-island-middle-lower-drawer-main
        (gethash "SinkDrawerLeftMiddle" owl->urdf-table)
        :sink-area-left-bottom-drawer-main
        (gethash "OvenDrawerRight" owl->urdf-table)
        :oven-area-area-right-drawer-main
        (gethash "FridgeDoorBottomShelf" owl->urdf-table)
        :iai-fridge-main
        (gethash "FridgeGlassShelf" owl->urdf-table)
        :iai-fridge-main
        (gethash "IslandArea" owl->urdf-table)
        :kitchen-island-surface)
        
  (defun owl->urdf (owl-name)
    (gethash owl-name owl->urdf-table)))

;; Translates the OWL types of the bullet items into its CRAM types.

(let ((owl->type-table (make-hash-table :test 'equalp)))
  (setf (gethash "SpoonSoup" owl->type-table)
        :spoon
        (gethash "SpoonDessert" owl->type-table)
        :spoon
        (gethash "KnifeTable" owl->type-table)
        :knife
        (gethash "BowlLarge" owl->type-table)
        :bowl
        (gethash "PlateClassic28" owl->type-table)
        :plate
        (gethash "GlassTall" owl->type-table)
        :cup
        (gethash "GlassRound" owl->type-table)
        :cup
        (gethash "Cup" owl->type-table)
        :cup
        (gethash "KoellnMuesliKnusperHonigNuss" owl->type-table)
        :breakfast-cereal
        (gethash "JaNougatBits" owl->type-table)
        :breakfast-cereal
        (gethash "KoellnMuesliCranberry" owl->type-table)
        :breakfast-cereal
        (gethash "BaerenMarkeFrischeAlpenmilch38" owl->type-table)
        :milk
        (gethash "HohesCOrange" owl->type-table)
        :bottle
        (gethash "Tray" owl->type-table)
        :tray)
  
  (defun owl->type (owl-name)
    (gethash owl-name owl->type-table)))

(defmethod costmap:costmap-generator-name->score ((name (eql 'vr-learned-grid)))
  10)


;; Designators returning the storage and destination locations from
;; the VR experiments encoded in location designators, which are later
;; resolved and inputted in `get-location-poses'.

(defmethod man-int:get-object-likely-location :vr-owl 30 (?object-type ?kitchen-name ?human-name ?context)
  "Returns a location designator representing the storage location of an object
with the given `?object-type' in the environment called `?kitchen-name' by using
the `GetSymbolicLocation' service with the given `?human-name' and `?context'.

The priority of this function should be lower than the priority of the
:heuristics functions, unless the objects are storaged like in the
VR experiment."
  (let* ((?vr-owl (get-object-storage-location ?object-type
                                               ?context
                                               ?human-name
                                               ?kitchen-name
                                               *table-id*))
         (?pseudo-urdf (owl->urdf ?vr-owl)))
    (if (location-on-p ?pseudo-urdf)
        (desig:a location
                 (on (desig:an object
                               (urdf-name ?pseudo-urdf)
                               (owl-name ?vr-owl)
                               (part-of ?kitchen-name))))
        (desig:a location
                 (in (desig:an object
                               (urdf-name ?pseudo-urdf)
                               (owl-name ?vr-owl)
                               (part-of ?kitchen-name)))))))

(defmethod man-int:get-object-destination :vr-owl 10 (?object-type ?kitchen-name ?human-name ?context)
"Returns a location designator representing the destination location of an object
with the given `?object-type' in the environment called `?kitchen-name' by using
the `GetSymbolicLocation' service the given `?human-name' and `?context'. 

The priority of this function should be higher than the priority of
The :heuristics functions, if the destination location from the VR
experiments should be used."
  (let* ((?vr-owl (get-object-destination-location ?object-type
                                                   ?context
                                                   ?human-name
                                                   ?kitchen-name
                                                   *table-id*))
         (?pseudo-urdf (owl->urdf ?vr-owl)))
    (if (location-on-p ?pseudo-urdf)
        (desig:a location
                 (on (desig:an object
                               (urdf-name ?pseudo-urdf)
                               (owl-name ?vr-owl)
                               (part-of ?kitchen-name)))
                 (context ?context)
                 (for (desig:an object (type ?object-type))))
        (desig:a location
                 (in (desig:an object
                               (urdf-name ?pseudo-urdf)
                               (owl-name ?vr-owl)
                               (part-of ?kitchen-name)))
                 (context ?context)
                 (for (desig:an object (type ?object-type)))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; GET-COSTMAP-LOCATION ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; Helper function

(defun get-already-placed-object ()
"Returns every bullet object placed on the kitchen island table."
  (remove-if-not (lambda (obj)
                   (and (typep obj 'btr:item)
                        (equalp
                         "kitchen_island"
                         (cdr (find (btr:name obj)
                                    ;; get all links contacting items
                                    ;; in the environment
                                    (btr:link-contacts
                                     (btr:get-environment-object))
                                    :key (lambda (item-and-link-name-cons)
                                           (btr:name (car item-and-link-name-cons)))
                                    :test #'equal)))))
                 (btr:objects btr:*current-bullet-world*)))

(defmethod man-int:get-location-poses :learning 10 (location-designator)
"Samples from the costmaps returned by the service `GetCostmap'."
  (if *learning-vr-on*
      (let (object-type kitchen-name context urdf-name on-p)
        ;; If the location designator contains a :for and :on/:in
        ;; property and a :context is specifed too...
        (if (and (desig:desig-prop-value location-designator :for) ;; (for (desig:an object (type ...)))
                 (or (desig:desig-prop-value location-designator :on)
                     (desig:desig-prop-value location-designator :in));; (on (desig:an object (part-of kitchen ...)))
                 (desig:desig-prop-value location-designator :context)
                 (not (desig:desig-prop-value location-designator :reachable-for)))

            ;; ... we get the object-designator and the environment designator.
            (let* ((object-designator
                     (desig:current-desig
                      (desig:desig-prop-value location-designator :for)))
                   (kitchen-object-designator
                     (if (desig:current-desig
                          (desig:desig-prop-value location-designator :on))
                         (desig:current-desig
                          (desig:desig-prop-value location-designator :on))
                         (desig:current-desig
                          (desig:desig-prop-value location-designator :in)))))

              ;; From these designators we get the needed values like...
              ;; ... object type of the object designator
              (when (desig:desig-prop-value object-designator :type)
                (setf object-type
                      (desig:desig-prop-value object-designator :type)))
              ;; ... name of the whole environment
              (when (desig:desig-prop-value kitchen-object-designator :part-of)
                (setf kitchen-name
                      (desig:desig-prop-value kitchen-object-designator :part-of)))
              ;; ... context of the location designator
              (when (desig:desig-prop-value location-designator :context)
                (setf context
                      (desig:desig-prop-value location-designator :context)))
              ;; ... urdf name of the environment designator
              (when (desig:desig-prop-value kitchen-object-designator :urdf-name)
                (setf urdf-name
                      (desig:desig-prop-value kitchen-object-designator :urdf-name)))
              ;; ... and if the object should be placed on or in the
              ;; given environment object (encoded with environment designator).
              (setf on-p
                    (member :on (desig:properties location-designator)
                            :key #'first
                            :test #'equal)))

            ;; Otherwise, we use the defined heuristics.
            (progn
              (roslisp:ros-debug (cvr costmap) "Using heuristics for the costmap.")
              (return-from man-int:get-location-poses
                (desig:resolve-location-designator-through-generators-and-validators
                 location-designator))))

        ;; After that we get the already placed objects and its
        ;; coordinates and call the the function `get-costmap-for' to
        ;; sample from the returned costmap.
        (let* ((placed-object-positions
                 (mapcar 
                  (alexandria:compose 
                   #'cl-transforms:origin
                   #'btr:pose)
                   (get-already-placed-object)))
               (x-placed-object-positions 
                 (mapcar 
                  #'cl-transforms:x
                  placed-object-positions))
               (y-placed-object-positions
                 (mapcar 
                  #'cl-transforms:y
                  placed-object-positions))
               (placed-object-types
                 (mapcar 
                  (lambda (obj)
                    (first (btr:item-types obj)))
                  (get-already-placed-object)))
               (learned-costmap
                 (get-costmap-for
                  object-type 
                  x-placed-object-positions y-placed-object-positions placed-object-types
                  context *human-name* kitchen-name *table-id* urdf-name on-p))
               (heuristics-costmaps
                 (mapcar (lambda (bindings)
                           (cut:var-value '?cm bindings))
                         (cut:force-ll
                          (prolog:prolog
                           `(costmap:desig-costmap ,location-designator ?cm)))))
               (merged-costmap
                 (apply #'costmap:merge-costmaps (cons learned-costmap
                                                       heuristics-costmaps))))

          (roslisp:ros-info (cvr costmap) "Visualizing learned costmap.")
          (cpl:sleep 1.0)
          (costmap:costmap-samples merged-costmap)))))
