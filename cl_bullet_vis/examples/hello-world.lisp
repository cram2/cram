;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;

(in-package :bt-vis-ex)

(defclass hello-world-window (bullet-world-window)
  ((last-update :accessor last-update :initform 0)
   (paused :accessor paused :initform t)
   (stored-state :accessor stored-state :initform nil)))

(defun hello-world ()
  (let* ((world (make-instance 'bt-world))
         (plane-body (make-instance
                      'rigid-body
                      :collision-shape (make-instance
                                        'static-plane-shape
                                        :normal (cl-transforms:make-3d-vector 0 0 1)
                                        :constant 0)))
         (box-body (make-instance
                    'rigid-body
                    :collision-shape (make-instance
                                      'box-shape
                                      :half-extents (cl-transforms:make-3d-vector 3 3 0.1))
                    :mass 0.0
                    :pose (cl-transforms::make-pose
                           (cl-transforms:make-3d-vector 0 0 1.5)
                           (cl-transforms:axis-angle->quaternion
                            (cl-transforms:make-3d-vector 1 0 0)
                            (/ pi 7)))))
         (sphere-body (make-instance 'rigid-body
                                     :collision-shape (make-instance 'sphere-shape
                                                                     :radius 0.3)
                                     :mass 0.01
                                     :pose (cl-transforms:make-pose
                                            (cl-transforms:make-3d-vector 0.0 1.0 4.0)
                                            (cl-transforms:make-quaternion 0 0 0 1))))
         (window (make-instance 'hello-world-window
                                :camera-transform (cl-transforms:make-transform
                                                   (cl-transforms:make-3d-vector -5 0 3)
                                                   (cl-transforms:axis-angle->quaternion
                                                    (cl-transforms:make-3d-vector 0 1 0)
                                                    (/ pi 8)))
                                :world world)))
    (add-rigid-body world plane-body)
    (add-rigid-body world box-body)
    (add-rigid-body world sphere-body)
    (glut:display-window window)))


(defmethod glut:display-window :after ((window hello-world-window))
  (setf (stored-state window) (get-state (world window))))

(defmethod glut:keyboard ((window hello-world-window) key x y)
  (declare (ignore x y))
  (case key
    (#\p (setf (paused window) (not (paused window))))
    (#\s (setf (stored-state window) (get-state (world window))))
    (#\r (when (stored-state window)
           (restore-world-state (stored-state window) (world window))))
    (t (call-next-method))))

(defmethod glut:tick :before ((window hello-world-window))
  (when (and (not (paused window))
             (>=  (- (get-internal-real-time) (last-update window))
                  (* 0.015 internal-time-units-per-second)))
    (setf (last-update window) (get-internal-real-time))
    (step-simulation (world window) 0.015)))
