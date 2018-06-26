;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :projection-process-modules)

(defun set-tf-from-robot-state (transformer robot
                                &key (base-frame *robot-base-frame*)
                                  (time (roslisp:ros-time)))
  (let ((reference-transform-inv (cl-transforms:transform-inv
                                  (cl-transforms:reference-transform
                                   (link-pose robot base-frame)))))
    (dolist (link (link-names robot))
      (unless (equal link base-frame)
        (let ((transform (cl-transforms:transform*
                          reference-transform-inv
                          (cl-transforms:reference-transform
                           (link-pose robot link)))))
          (add-new-transform transformer
                             (make-transform-stamped
                              base-frame link time
                              (cl-transforms:translation transform)
                              (cl-transforms:rotation transform))
                             :suppress-callbacks t))))
    (execute-new-transform-callbacks transformer)))

(defun update-tf (&key (base-frame *robot-base-frame*)
                    (odom-frame *odom-frame*)
                    (map-frame *fixed-frame*))
  (cut:with-vars-bound (?robot-instance ?robot-pose)
      (cut:lazy-car
       (prolog:prolog `(and (robot ?robot)
                            (bullet-world ?world)
                            (%object ?world ?robot ?robot-instance)
                            (pose ?world ?robot ?robot-pose))))
    (assert (not (cut:is-var ?robot-instance)))
    (add-new-transform
     *transformer*
     (make-transform-stamped
      map-frame odom-frame (roslisp:ros-time)
      (cl-transforms:make-identity-vector)
      (cl-transforms:make-identity-rotation))
     :suppress-callbacks t)
    (add-new-transform
     *transformer*
     (make-transform-stamped
      odom-frame base-frame (roslisp:ros-time)
      (cl-transforms:origin ?robot-pose)
      (cl-transforms:orientation ?robot-pose))
     :suppress-callbacks t)
    (set-tf-from-robot-state
     *transformer* ?robot-instance)))

(defmethod cram-occasions-events:on-event update-tf
    ((event cram-plan-occasions-events:robot-state-changed))
  (update-tf))

(defmethod cram-robot-interfaces:compute-iks :before (pose-stamped
                                                      &key link-name arm robot-state
                                                        seed-state pose-stamped-frame
                                                        tcp-in-ee-pose)
  (declare (ignore pose-stamped link-name arm seed-state pose-stamped-frame tcp-in-ee-pose))
  (let ((time (roslisp:ros-time)))
    ;; tell the tf transformer where the robot currently is in the global
    ;; fixed coordinate system
    (cut:with-vars-bound (?robot-instance ?robot-pose)
        (cut:lazy-car
         (prolog:prolog `(and (robot ?robot)
                              (bullet-world ?world)
                              (%object ?world ?robot ?robot-instance)
                              (pose ?world ?robot ?robot-pose))))
      (assert (not (cut:is-var ?robot-instance)))

      (add-new-transform *transformer*
                         (transform->transform-stamped
                          *fixed-frame* *robot-base-frame* time
                          (cl-transforms:pose->transform ?robot-pose)))
      ;; tell the tf transformer the current configuration of robot's joints
      (set-tf-from-robot-state *transformer* robot-state
                               :base-frame *robot-base-frame* :time time))))

(defun add-new-transform (transformer transform &key suppress-callbacks)
  (cl-tf:set-transform transformer transform :suppress-callbacks suppress-callbacks))

(defun execute-new-transform-callbacks (transformer)
  (cl-tf:execute-changed-callbacks transformer))
