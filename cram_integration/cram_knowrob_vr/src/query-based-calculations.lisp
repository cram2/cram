;;;
;;; Copyright (c) 2018, Alina Hawkin <hawkin@cs.uni-bremen.de>
;;;                     Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :kvr)

(defun map-T-camera->map-P-base (map-T-camera)
  "Calculates the transform of where the robot should stand to grasp the object.
Based on data from Virtual Reality.
Removes the Z component of the Camera pose of the human
and also fixes the quaternion so that the robot won't tilt into the pane of the floor.
`transform': the transform of the Camera (head position of the human) from VR.
RETURNS: A pose stamped for the robot base."
  (let* ((map-R-camera
           (cl-transforms:quaternion->matrix
            (cl-transforms:rotation map-T-camera)))
         (camera-Z-axis-in-map ; the axis that looks from the camera to object is Z
           (cl-transforms:make-3d-vector ; it's the 3rd column of map-R-camera
            (aref map-R-camera 0 2)
            (aref map-R-camera 1 2)
            (aref map-R-camera 2 2)))
         (camera-Z-axis-in-map-projected-onto-floor
           (cl-transforms:copy-3d-vector camera-Z-axis-in-map :z 0))
         (base-orientation ; base should look in the same direction as camera-Z-axis
           (cl-transforms:axis-angle->quaternion
            (cl-transforms:make-3d-vector 0 0 1)
            (atan (cl-transforms:y camera-Z-axis-in-map-projected-onto-floor)
                  (cl-transforms:x camera-Z-axis-in-map-projected-onto-floor))))
         (base-origin ; base origin is the camera coordinate projected onto floor
           (cl-transforms:make-3d-vector
            (cl-transforms:x (cl-transforms:translation map-T-camera))
            (cl-transforms:y (cl-transforms:translation map-T-camera))
            0)))
    (cl-transforms-stamped:make-pose-stamped
     cram-tf:*fixed-frame*
     0.0
     base-origin
     base-orientation)))

(defun umap-P-uobj-through-surface-ll (type start-or-end)
  "Calculates the pose of the object in map relative to its supporting surface.
Formula: umap-T-uobj = umap-T-usurface * inv(smap-T-ssurface) * smap-T-sobj.
`type' is a simple symbol such as 'milk."
  (let ((name-and-surface-T-object-ll
           (query-name-and-surface-T-object-by-object-type
            (object-type-filter-prolog type)
            start-or-end
            :table-setting)))
    (cut:lazy-mapcar
     (lambda (name-and-surface-T-object)
       (let* ((surface-name
                (car name-and-surface-T-object))
              (ssurface-T-sobject
                (cdr name-and-surface-T-object))
              (umap-T-usurface
                (cl-transforms:pose->transform
                 (btr:pose
                  (btr:rigid-body
                   (btr:get-environment-object)
                   (match-kitchens surface-name)))))
              (umap-T-uobj
                (cl-transforms:transform*
                 umap-T-usurface ssurface-T-sobject)))
         (cl-transforms-stamped:make-pose-stamped
          cram-tf:*fixed-frame*
          0.0
          (cl-transforms:translation umap-T-uobj)
          (cl-transforms:rotation umap-T-uobj))))
     name-and-surface-T-object-ll)))


(defun umap-T-ucamera-through-surface-ll (type time)
  "Calculates the pose of the robot's 'camera' in map
relative to object supporting surface.
Formula: umap-T-ucamera = umap-T-usurface * inv(smap-T-ssurface) * smap-T-scamera
                        = umap-T-usurface * ssurface-T-scamera."
  (assert (or (equal time "Start") (equal time "End")))
  (let ((name-and-surface-T-camera-ll
          (query-name-and-surface-T-camera-by-object-type
           (object-type-filter-prolog type)
           time
           :table-setting)))
    (cut:lazy-mapcar
     (lambda (name-and-surface-T-camera)
       (let* ((surface-name
                (car name-and-surface-T-camera))
              (ssurface-T-scamera
                (cdr name-and-surface-T-camera))
              (umap-T-usurface
                (cl-transforms:pose->transform
                 (btr:pose
                  (btr:rigid-body
                   (btr:get-environment-object)
                   (match-kitchens surface-name)))))
              (umap-T-ucamera
                (cl-transforms:transform*
                 umap-T-usurface ssurface-T-scamera)))
         umap-T-ucamera))
     name-and-surface-T-camera-ll)))


(defun umap-T-ucamera-through-object-ll-based-on-object-pose (type time
                                                              umap-P-uobj)
  (assert (or (equal time "Start") (equal time "End")))
  (let ((umap-T-uobj
          (cl-transforms:pose->transform umap-P-uobj))
        (sobj-T-scamera-lazy-list
          (query-object-T-camera-by-object-type
           (object-type-filter-prolog type)
           time)))
    (cut:lazy-mapcar (lambda (sobj-T-scamera)
                       (cl-transforms:transform*
                        umap-T-uobj sobj-T-scamera))
                     sobj-T-scamera-lazy-list)))

(defun umap-T-ucamera-through-object-ll (type time)
  "Calculates the pose of the robot's 'camera' in map relative to object location.
Formula: umap-T-ucamera = umap-T-uobj * inv(smap-T-sobj) * smap-T-scamera
                        = umap-T-uobj * sobj-T-scamera."
  (assert (or (equal time "Start") (equal time "End")))
  (let ((umap-P-uobj
          (btr:pose
           (btr:object btr:*current-bullet-world*
                       (object-type-filter-bullet type)))))
    (umap-T-ucamera-through-object-ll-based-on-object-pose
     type time umap-P-uobj)))


(defun base-poses-ll-for-searching (type)
  (let ((umap-T-ucamera-ll
          (umap-T-ucamera-through-surface-ll type "Start")))
    (cut:lazy-mapcar
     (lambda (umap-T-ucamera)
       (map-T-camera->map-P-base umap-T-ucamera))
     umap-T-ucamera-ll)))

(defun base-poses-ll-for-searching-based-on-object-pose (bullet-type umap-P-uobj)
  (let* ((prolog-type
           (roslisp-utilities:rosify-lisp-name
            (object-type-fixer bullet-type)))
         (umap-T-ucamera-ll
           (umap-T-ucamera-through-object-ll-based-on-object-pose
            prolog-type "Start" umap-P-uobj)))
    (cut:lazy-mapcar
     (lambda (umap-T-ucamera)
       (map-T-camera->map-P-base umap-T-ucamera))
     umap-T-ucamera-ll)))

(defun base-poses-ll-for-picking-up (type)
  (let ((umap-T-ucamera-ll
          (umap-T-ucamera-through-object-ll type "Start")))
    (cut:lazy-mapcar
     (lambda (umap-T-ucamera)
       (map-T-camera->map-P-base umap-T-ucamera))
     umap-T-ucamera-ll)))

(defun base-poses-ll-for-placing (type)
  (let ((umap-T-ucamera-ll
          (umap-T-ucamera-through-object-ll type "End")))
    (cut:lazy-mapcar
     (lambda (umap-T-ucamera)
       (map-T-camera->map-P-base umap-T-ucamera))
     umap-T-ucamera-ll)))

(defun look-poses-ll-for-searching (type)
  (umap-P-uobj-through-surface-ll type "Start"))

(defun look-poses-ll-for-placing (type)
  (umap-P-uobj-through-surface-ll type "End"))

(defun object-poses-ll-for-placing (type)
  (umap-P-uobj-through-surface-ll type "End"))



(defun human-to-robot-hand-transform ()
  "Defines the offset between the human hand from the virtual reality to the
robot standart gripper, which has been calculated manually.
RETURNS: a cl-transform."
  (let ((alpha 0)) ; (/ pi 4)
    (cl-transforms:make-transform
     (cl-transforms:make-3d-vector 0.0 -0.07 0.2)
     (cl-transforms:matrix->quaternion
      (make-array '(3 3)
                  :initial-contents
                  `((0                1 0)
                    (,(- (cos alpha)) 0 ,(- (sin alpha)))
                    (,(- (sin alpha)) 0 ,(cos alpha))))))))

(defun calculate-transform-object-type-T-gripper (object-type)
  (let ((prolog-object-type
          (roslisp-utilities:rosify-lisp-name (object-type-fixer object-type))))
    (cl-transforms:transform*
     (cl-transforms:transform-inv
      (car (query-object-location-by-object-type prolog-object-type "Start")))
     (car (query-hand-location-by-object-type prolog-object-type "Start"))
     (human-to-robot-hand-transform))))
