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
          :iai-fridge-main)
        
  (defun owl->urdf (owl-name)
    (gethash owl-name owl->urdf-table)))

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

(defmethod man-int:get-location-poses :learning 8 (location-designator)
  (print "get-location-poses w/ vr-learning called")
  (if T ;;(and *learning-framework-on* (rob-int:reachability-designator-p location-designator))
      (let (object-type kitchen-name context)
        (if (and (desig:desig-prop-value location-designator :for) ;; (for (desig:an object (type ...)))
                 (desig:desig-prop-value location-designator :on) ;; (on (desig:an object (part-of kitchen ...)))
                 (desig:desig-prop-value location-designator :context))
            
            (let* ((object-designator
                     (desig:current-desig
                      (desig:desig-prop-value location-designator :for)))
                   (kitchen-object-designator
                     (desig:current-desig
                      (desig:desig-prop-value location-designator :on))))

              (when (desig:desig-prop-value object-designator :type)
                (setf object-type
                      (desig:desig-prop-value object-designator :type)))
              (when (desig:desig-prop-value kitchen-object-designator :part-of)
                (setf kitchen-name
                      (desig:desig-prop-value kitchen-object-designator :part-of)))
              (setf context
                    (desig:desig-prop-value location-designator :context))
              (print object-type)
              (print kitchen-name)
              (print context))
            (progn
              ;;(when (desig:desig-prop-value location-designator :location)
              ;; any other location designator
              (print "nvm - use heuristics")
              (return-from man-int:get-location-poses
                (desig:resolve-location-designator-through-generators-and-validators
                 location-designator))))
        (let* ((learned-costmap
                 (get-costmap-for
                  object-type context *human-name* kitchen-name *table-id*))
               (heuristics-costmaps
                 (mapcar (lambda (bindings)
                           (cut:var-value '?cm bindings))
                         (cut:force-ll
                          (prolog:prolog
                           `(costmap:desig-costmap ,location-designator ?cm)))))
               (merged-costmap
                 (apply #'costmap:merge-costmaps (cons learned-costmap
                                                       heuristics-costmaps))))
          (print "test")
          ;; TODO: check for invalid-probability-distribution
          (costmap:costmap-samples learned-costmap)
          (roslisp:ros-info (cvr costmap) "Visualizing learned costmap.")
          (cpl:sleep 1.0)
          (costmap:costmap-samples learned-costmap)))

        ;;(desig:resolve-location-designator-through-generators-and-validators
        ;;location-designator)
      ))

(defmethod costmap:costmap-generator-name->score ((name (eql 'vr-learned-grid)))
  8)

(defmethod man-int:get-object-likely-location :vr-owl 30 (?kitchen-name ?human-name ?context ?object-type)
  (let* ((?vr-owl (get-object-storage-location ?object-type
                                               ?context
                                               ?human-name
                                               ?kitchen-name
                                               "rectangular_table"))
         (?pseudo-urdf (owl->urdf ?vr-owl)))
    (desig:a location
             (on (desig:an object
                           ;; (type ?pseudo-type)
                           (urdf-name ?pseudo-urdf)
                           (owl-name ?vr-owl)
                           (part-of ?kitchen-name))))))

(defmethod man-int:get-object-likely-destination :vr-owl 7 (?kitchen-name ?human-name ?context ?object-type)
  (let* ((?vr-owl (get-object-destination-location ?object-type
                                                   ?context
                                                   ?human-name
                                                   ?kitchen-name
                                                   "rectangular_table"))
         (?pseudo-urdf (owl->urdf ?vr-owl))
         ;;(?pseudo-btr-type (owl->type ?object-type)
         )
    (desig:a location
             (on (desig:an object
                           ;; (type counter-top)
                           (urdf-name ?pseudo-urdf)
                           (owl-name ?vr-owl)
                           (part-of ?kitchen-name)))
             (context ?context)
             (for (desig:an object (type ?object-type))))))
