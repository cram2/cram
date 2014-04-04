;;; Copyright (c) 2014, Gayane Kazhoyan <kazhoyan@in.tum.de>
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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors
;;;       may be used to endorse or promote products derived from this software
;;;       without specific prior written permission.
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

(in-package :spatial-relations-demo)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;; PANCAKES! ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun spawn-pancake-scenario ()
  (btr::clear-current-costmap-function-object)
  (detach-all-objects (object *current-bullet-world* 'cram-pr2-knowledge::pr2))  
  (prolog `(and (bullet-world ?w)
                (robot ?robot)
                (robot-arms-parking-joint-states ?joint-parking-state)
                (assert (joint-state ?w ?robot ?joint-parking-state))
                (assert (joint-state ?w ?robot (("torso_lift_joint" 0.33))))
                (assert (object ?w btr::pancake-maker oven-1
                                ((-1.0 -0.4 0.765) (0 0 0 1))
                               :mass 0.2 :color (0.15 0.15 0.15) :size (0.15 0.15 0.035)))
                (assert (object ?w btr::mesh spatula-1
                                ((1.4 1.08 0.9119799601336841d0) (0 0 0 1))
                                :mesh btr::spatula :mass 0.2 :color (0 0 0)))
                (assert (object ?w btr::mesh spatula-2
                                ((1.4 1.08 0.9119799601336841d0) (0 0 0 1))
                                :mesh btr::spatula :mass 0.2 :color (0 0 0)))
                (assert (object ?w btr::mesh mondamin-1
                                ((1.35 1.11 0.9119799601336841d0) (0 0 0 1))
                                :mesh mondamin :mass 0.2 :color (0.8 0.4 0.2)))))
  (move-object 'spatula-1 `((1.43 0.6 0.86) (0.0d0 0.0d0 -0.4514496d0 0.89229662d0)))
  (move-object 'spatula-2 `((1.45 0.95 0.86) (0 0 0.2 1)))
  (move-object 'mondamin-1 `((1.35 1.11 0.958) (0 0 0 1)))
  (move-object 'cram-pr2-knowledge::pr2 `((0 0 0) (0 0 0 1))))

(defun spawn-spheres ()
  (btr::set-object-pose (object *current-bullet-world* 'cram-pr2-knowledge::pr2)
                        '((-0.2 1.3 0) (0 0 1 0)))
  (dolist (pair *position-scatter*)
    ;; (add-object *current-bullet-world* 'sphere (gensym) (first pair)
    ;;             :radius 0.01 :mass 0.1 :color '(0 0 0))
    (add-object *current-bullet-world* 'sphere (gensym)
                (cl-transforms:make-pose
                 (cl-transforms:make-3d-vector
                  (cl-transforms:x (cl-transforms:origin (first pair)))
                  (cl-transforms:y (cl-transforms:origin (first pair)))
                  0.862)
                 (cl-transforms:orientation (first pair)))
                :radius 0.01 :mass 0.1 :color '(0 0 0))))

(defun execute-pancake-scenario ()
  (sb-ext:gc :full t)
  (spawn-pancake-scenario)
  (cram-projection:with-projection-environment
      projection-process-modules::pr2-bullet-projection-environment
    (cpl-impl:top-level
      (let ((spatula-designator
              (find-object-on-counter 'btr::spatula "kitchen_sink_block")))
        (sb-ext:gc :full t)
        (cram-language-designator-support:with-designators
            ((spatula-location (location `((on "Cupboard")
                                           (name "pancake_table")
                                           (centered-with-padding 0.6)
                                           (for ,spatula-designator)
                                           (right-of oven-1)
                                           (near oven-1)))))
          (format t "now trying to achieve the location of spatula on kitchen-island~%")
          (plan-knowledge:achieve
           `(plan-knowledge:loc ,spatula-designator ,spatula-location))))
      (sb-ext:gc :full t)
      (let ((mondamin-designator
              (find-object-on-counter 'btr::mondamin "kitchen_sink_block")))
        (sb-ext:gc :full t)
        (cram-language-designator-support:with-designators
            ((on-kitchen-island (location `((on "Cupboard")
                                            (name "pancake_table")
                                            (centered-with-padding 0.35)
                                            (for ,mondamin-designator)
                                            (right-of oven-1)
                                            (far-from oven-1)))))
          (format t "now trying to achieve the location of mondamin on kitchen-island~%")
          (plan-knowledge:achieve
           `(plan-knowledge:loc ,mondamin-designator ,on-kitchen-island))
          (sb-ext:gc :full t)))
      (sb-ext:gc :full t)
      (let ((spatula-2-designator
              (find-object-on-counter 'btr::spatula "kitchen_sink_block")))
        (sb-ext:gc :full t)
        (cram-language-designator-support:with-designators
            ((spatula-location (location `((on "Cupboard")
                                           (name "pancake_table")
                                           (centered-with-padding 0.6)
                                           (for ,spatula-2-designator)
                                           (left-of oven-1)
                                           (near oven-1)))))
          (format t "now trying to achieve the location of spatula on kitchen-island~%")
          (plan-knowledge:achieve
           `(plan-knowledge:loc ,spatula-2-designator ,spatula-location)))))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;; restricting area for the robot ;;;;;;;;;;;;;;;;;

(defun make-restricted-area-cost-function ()
  (lambda (x y)
    (if (> x 1.0) 0.0
        (if (and (> x 0.0) (> y -1.0) (< y 1.0)) 1.0
            (if (and (< x 0.0) (> x -1.0) (> y -1.0) (< y 2.0)) 1.0
                0.0)))))

(defmethod costmap-generator-name->score ((name (eql 'restricted-area))) 5)

(def-fact-group pancakes (desig-costmap)
  (<- (desig-costmap ?designator ?costmap)
    (or (desig-prop ?designator (to see))
        (desig-prop ?designator (to reach)))
    (costmap ?costmap)
    (costmap-padding ?padding)
    (costmap-add-function restricted-area (make-restricted-area-cost-function)
                          ?costmap)))

(defun test-costmap ()
  (prolog `(and (costmap-padding ?pad)
              (costmap ?cm)
              (occupancy-grid-costmap::drivable-location-costmap ?cm ?pad)
              (semantic-map-objects ?objects)
              (costmap-add-function semantic-map-costmap::semantic-map-free-space
                                    (semantic-map-costmap::make-semantic-map-costmap
                                     ?objects :invert t :padding ?pad)
                                    ?cm)
              (costmap-add-function restricted-area
                                    (make-restricted-area-cost-function)
                                    ?cm)
              (debug-costmap ?cm))))