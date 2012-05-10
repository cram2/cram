;;; Copyright (c) 2012, Georg Bartels <georg.bartels@in.tum.de>
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

(in-package :perception-process-module)

(defvar *handle-detector-action* nil
  "Action client for handle detector.")

(defclass handle-perceived-object (perceived-object)
  ((name :initarg :name :initform nil :reader handle-name)))

(defun init-handle-detector ()
  (setf *handle-detector-action*
        (actionlib:make-action-client
         "handle_detector/detect"
         "handle_detection/HandleDetectionAction")))

(register-ros-init-function init-handle-detector)

(defmethod make-new-desig-description ((old-desig object-designator)
                                       (perceived-object handle-perceived-object))
  (let ((description (call-next-method)))
    (if (member 'name description :key #'car)
        description
        (cons `(name ,(name perceived-object)) description))))

(defun point-to-box-distance (pose dimensions point)
  "Returns the minimum distance of `point' to the box described by
  `dimensions' with its center at `pose'."
  (let ((point-in-box (cl-transforms:transform-point
                       (cl-transforms:transform-inv
                        (cl-transforms:pose->transform pose))
                       point)))
    ;; TODO(moesenle): this algorithm is _wrong_ and needs to be
    ;; fixed. However, it seems to be a working approximation.
    (with-slots ((size-x cl-transforms:x)
                 (size-y cl-transforms:y)
                 (size-z cl-transforms:z))
        dimensions
      (sqrt
       (+ (let ((distance-x (abs (cl-transforms:x point-in-box))))
            (if (< distance-x size-x)
                0.0 (expt (- distance-x (/ size-x 2)) 2)))
          (let ((distance-y (abs (cl-transforms:y point-in-box))))
            (if (< distance-y size-y)
                0.0 (expt (- distance-y (/ size-y 2)) 2)))
          (let ((distance-z (abs (cl-transforms:z point-in-box))))
            (if (< distance-z size-z)
                0.0 (expt (- distance-z (/ size-z 2)) 2))))))))

(defun find-handle-name (designator pose)
  "Returns the handle's name that is closest to `pose'. Takes into
account the shape of the handle."
  (assert (eql (desig-prop-value designator 'type) 'handle))
  (let* ((handles (sem-map-utils:designator->semantic-map-objects designator))
         (closest-handle
           (cut:minimum handles :key (lambda (handle)
                                       (point-to-box-distance
                                        (sem-map-utils:pose handle)
                                        (sem-map-utils:dimensions handle)
                                        (cl-transforms:origin pose))))))
    (when closest-handle
      (sem-map-utils:name closest-handle))))

(def-object-search-function handle-search-function handle-detector
    (((type handle)) desig perceived-object)
  (declare (ignore perceived-object))
  (with-fields (handles)
      (actionlib:send-goal-and-wait
       *handle-detector-action*
       (actionlib:make-action-goal *handle-detector-action*
         number_of_handles 3))
    (map 'list (lambda (handle-pose)
                 (let ((pose (tf:msg->pose-stamped handle-pose)))
                   (make-instance 'handle-perceived-object
                     :name (find-handle-name desig pose)
                     :pose (tf:copy-pose-stamped
                            (tf:transform-pose
                             ;; hack(moesenle): remove the time stamp because
                             ;; tf fails to transform for some weird reason
                             *tf* :pose (tf:copy-pose-stamped pose :stamp 0.0)
                             :target-frame *fixed-frame*)
                            :stamp 0.0)
                     :probability 1.0)))
         handles)))
