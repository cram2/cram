;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :demo)

(defun spawn-random-there-and-back-again-objects ()
  (btr-utils:kill-all-objects)
  (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
  (let ((object-types '(:cup :bowl :spoon)))
    ;; spawn at default location
    (let ((objects (mapcar (lambda (object-type)
                             (btr-utils:spawn-object
                              (intern (format nil "~a-1" object-type) :keyword)
                              object-type))
                           object-types)))
      ;; move on top of counter tops
      (mapcar (lambda (btr-object)
                (let* ((aabb-z (cl-transforms:z
                                (cl-bullet:bounding-box-dimensions (btr:aabb btr-object))))
                       (new-pose (cram-tf:rotate-pose
                                  (cram-tf:translate-pose
                                   (desig:reference
                                    (desig:a location
                                             (side left)
                                             (side front)
                                             (on (desig:an object
                                                           (type
                                                            counter-top)
                                                           (urdf-name
                                                            sink-area-surface)
                                                           (owl-name
                                                            "kitchen_sink_block_counter_top")
                                                           (part-of
                                                            kitchen)))))
                                   :z-offset (/ aabb-z 2.0))
                                  :z (/ pi (random 10.0)))))
                  (btr-utils:move-object (btr:name btr-object) new-pose)))
              objects)

      ;; stabilize world
      (btr:simulate btr:*current-bullet-world* 100)
      (btr:simulate btr:*current-bullet-world* 100)

      (let (respawn)
        (dolist (object objects)
          (when (< (cl-transforms:z (cl-transforms:origin (btr:pose object))) 0.5)
            (setf respawn t)))
        (when respawn
          (spawn-random-there-and-back-again-objects))))))

(defun random-there-and-back-again ()
  (initialize)
  (spawn-random-there-and-back-again-objects)
  (park-robot)

  (dolist (?object-type '(:bowl :cup :spoon))
    (let* ((?arm-to-use (nth (random 2) '(:left :right)))
           (?color (cdr (assoc ?object-type *object-colors*))))
      (cpl:with-failure-handling
          ((common-fail:high-level-failure (e)
             (roslisp:ros-warn (pr2-demo random-taba) "Failure happened: ~a~%Skipping..." e)
             (return)))
        (let* ((?object
                 (desig:an object
                           (type ?object-type)
                           (desig:when ?color
                             (color ?color))))
               (?first-location
                 (desig:a location
                          (side left)
                          (side front)
                          (on (desig:an object
                                        (type counter-top)
                                        (urdf-name sink-area-surface)
                                        (owl-name "kitchen_sink_block_counter_top")
                                        (part-of kitchen)))
                          (for ?object)))
               (?second-location
                 (desig:a location
                          (side right)
                          (on (desig:an object
                                        (type counter-top)
                                        (urdf-name kitchen-island-surface)
                                        (owl-name "kitchen_island_counter_top")
                                        (part-of kitchen)))
                          (for ?object))))
          (exe:perform
           (desig:an action
                     (type transporting)
                     (object ?object)
                     (location ?first-location)
                     (target ?second-location)
                     (arm ?arm-to-use)))
          (exe:perform
           (desig:an action
                     (type positioning-arm)
                     (left-configuration park)
                     (right-configuration park)))
          (let ((?new-object (exe:perform
                              (desig:an action
                                        (type detecting)
                                        (object ?object)))))
            (exe:perform
             (desig:an action
                       (type transporting)
                       (object ?new-object)
                       (location ?second-location)
                       (target ?first-location)
                       (arm ?arm-to-use))))))))

  (park-robot)
  (finalize)
  cpl:*current-path*)

