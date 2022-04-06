;;; Copyright (c) 2020, Amar Fayaz <amar@uni-bremen.de>
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

(in-package :spatial-cm-tests)

(defun setup-test ()
  (roslisp-utilities:startup-ros)
  (coe:clear-belief)
  (setf cram-tf:*tf-default-timeout* 2.0)
  (setf prolog:*break-on-lisp-errors* t)
  (setf proj-reasoning::*projection-reasoning-enabled* nil)

  (btr-utils:kill-all-objects)
  (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo"))

(defun spawn-object (pose object-type)
  (btr-utils:spawn-object
   (intern (format nil "~a-1" object-type) :keyword)
   object-type
   :pose pose
   :mass 0.0)
  (btr:simulate btr:*current-bullet-world* 100))

(defun make-pose-stamped (pose-list)
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (apply #'cl-transforms:make-3d-vector (first pose-list))
   (apply #'cl-transforms:make-quaternion (second pose-list))))


(define-test check-spoon-orientation-and-height-on-dining-table
  (setup-test)
  ;; bowl pose is on the dining table
  (let ((bowl-pose (make-pose-stamped '((-3.368202972412109d0
                                         -0.15089993476867675d0
                                         0.7991479237874349d0)
                                        (3.7803240502398694d-6
                                         -5.186260023037903d-5
                                         0.9513682126998901d0
                                         0.30805596709251404d0))))
        ;; Spoon just needs to exist, the pose doesn't matter
        (spoon-pose (make-pose-stamped '((0 0 0) (0 0 0 1)))))
    (spawn-object bowl-pose :bowl)
    (spawn-object spoon-pose :spoon)
    (let* ((?other-object-designator
             (desig:an object
                       (type bowl)
                       (location (desig:a location
                                          (on (desig:an object
                                                        (type counter-top)
                                                        (urdf-name dining-area-jokkmokk-table-main)
                                                        (part-of iai-kitchen)))
                                          (for (desig:an object (type bowl)))
                                          (side right)
                                          (context table-setting)
                                          (object-count 2)))))
           (location-right-of-bowl (desig:reference (desig:a location
                                                             (right-of ?other-object-designator)
                                                             (near ?other-object-designator)
                                                             (for (desig:an object (type spoon)))
                                                             (orientation support-aligned))))
           (angle-for-spoon (cram-tf:angle-around-map-z
                             (cl-transforms:orientation location-right-of-bowl)))
           (height-for-spoon (cl-transforms:z (cl-transforms:origin location-right-of-bowl)))
           (height-of-dining-table (btr-spatial-cm::get-rigid-body-aabb-top-z
                                    (btr:rigid-body
                                     (btr:object btr:*current-bullet-world* :iai-kitchen)
                                     (btr::make-rigid-body-name
                                      (string-upcase :iai-kitchen)
                                      (roslisp-utilities:rosify-underscores-lisp-name
                                       'dining-area-jokkmokk-table-main))))))

      (assert-true (< (- height-for-spoon height-of-dining-table) 0.05))
      (assert-true (< (abs (- 1.5708 angle-for-spoon)) 0.02))))) ;; ~1 degrees
