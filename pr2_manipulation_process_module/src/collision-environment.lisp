;;; Copyright (c) 2011, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :pr2-manipulation-process-module)

(defvar *collision-object-pub* nil)
(defvar *attached-object-pub* nil)

(defvar *known-collision-objects* (tg:make-weak-hash-table :weakness :key))

(defun get-collision-object-name (desig)
  "returns the name of a known collision object or NIL"
  (let ((obj (find-desig-collision-object desig)))
    (when obj
      (roslisp:with-fields (id) obj
        id))))

(defun find-desig-collision-object (desig)
  (labels ((find-collision-object (curr)
             (when curr
               (or (gethash curr *known-collision-objects*)
                   (find-collision-object (parent curr))))))
    (find-collision-object (current-desig desig))))

(defun desig-bounding-box (desig)
  "Returns the bounding box of the object that is bound to `desig' if
  the object is a point cloud. Otherwise, returns NIL. The result is
  of type CL-TRANSFORMS:3D-VECTOR"
  (let ((obj (find-desig-collision-object desig)))
    (when obj
      (roslisp:with-fields (shapes)
          obj
        ;; This operation only makes sense if we have only one shape
        ;; in the collision object. Otherwise we return NIL
        (unless (> (length shapes) 1)
          (roslisp:with-fields (type dimensions)
              (elt shapes 0)
            ;; We need a BOX
            (when (eql type 1)
              (apply #'cl-transforms:make-3d-vector
                     (map 'list #'identity dimensions)))))))))

(defun remove-collision-object (desig)
  (let ((collision-object (find-desig-collision-object desig)))
    (when collision-object
      (roslisp:with-fields (id) collision-object
        (moveit:remove-collision-object id)))))

(defun clear-collision-objects ()
  (moveit:clear-collision-objects))

(defun point->msg (point &optional (msg-type "geometry_msgs/Point"))
  (declare (type cl-transforms:3d-vector point))
  (roslisp:make-msg
   msg-type
   x (cl-transforms:x point)
   y (cl-transforms:y point)
   z (cl-transforms:z point)))

(defun points->point-cloud (pose points)
  (let ((pose-tf (cl-transforms:reference-transform pose)))
    (roslisp:make-msg
     "sensor_msgs/PointCloud"
     (stamp header) (cl-tf-datatypes:stamp pose)
     (frame_id header) (cl-tf-datatypes:frame-id pose)
     points (map 'vector
                 (lambda (p)
                   (roslisp:with-fields (x y z) p
                     (point->msg
                      (cl-transforms:transform-point
                       pose-tf (cl-transforms:make-3d-vector x y z))
                      "geometry_msgs/Point32")))
                 points))))

(defun collision-environment-set-laser-period ()
  "Sets the tilting laser period to work best with collision
  environment"
  (roslisp:call-service
   "/laser_tilt_controller/set_periodic_cmd" "pr2_msgs/SetPeriodicCmd"
   :command (roslisp:make-msg
             "pr2_msgs/PeriodicCmd"
             (stamp header) (ros-time)
             profile "linear"
             ;period 0
             period 3
             amplitude 0;0.75
             offset 0.25)))
