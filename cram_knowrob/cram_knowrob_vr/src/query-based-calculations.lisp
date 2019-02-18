;;;
;;; Copyright (c) 2018, Alina Hawkin <hawkin@cs.uni-bremen.de>
;;;                      Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(defun umap-P-uobj-through-surface (type start-or-end)
  "Clauclates the placing pose of the object relative to its supporting surface.
Formula: umap-P-uobj = umap-T-usurface * inv(smap-T-ssurface) * smap-T-sobj.
`type' is a simple symbol such as 'milk."
  (let* ((prolog-type
           (object-type-filter-prolog type))
         (smap-T-ssurface
           (query-contact-surface-place-transform prolog-type))
         (umap-T-usurface
           (cl-transforms:pose->transform
            (btr:pose
             (btr:rigid-body
              (btr:get-environment-object)
              (match-kitchens (query-contact-surface-place-name prolog-type))))))
         (place-transform ; calculate place pose relative to bullet table
           (cl-transforms:transform*
            umap-T-usurface
            (cl-transforms:transform-inv smap-T-ssurface)
            (query-object-location-by-object-type prolog-type start-or-end))))
    (cl-transforms:transform->pose place-transform)))


(defun umap-T-ucamera-through-surface (type time)
  "Calculates the transform of robot 'camera' in map based on supporting surface.
Formula: umap-T-ucamera = umap-T-usurface * inv(smap-T-ssurface) * smap-T-scamera."
  (assert (or (equal time "Start") (equal time "End")))
  (let* ((prolog-type (object-type-filter-prolog type))
         (umap-T-usurface
           (cl-transforms:pose->transform
            (btr:pose
             (btr:rigid-body
              (btr:get-environment-object)
              (match-kitchens
               (if (equal time "Start")
                   (query-contact-surface-pick-name prolog-type)
                   (query-contact-surface-place-name prolog-type)))))))
         (smap-T-ssurface
           (if (equal time "Start")
               (query-contact-surface-pick-transform prolog-type)
               (query-contact-surface-place-transform prolog-type)))
         (smap-T-scamera (query-camera-location-by-object-type prolog-type time))
         (umap-T-ucamera
           (cl-transforms:transform*
            smap-T-scamera
            (cl-transforms:transform-inv smap-T-ssurface)
            umap-T-usurface)))
    umap-T-ucamera))

(defun umap-T-ucamera-through-object (type time)
  "Calculates the transform urdfmap T camera going through object location.
umap-T-ucamera = umap-T-uobj * inv(smap-T-sobj) * smap-T-scamera"
  (assert (or (equal time "Start") (equal time "End")))
  (let* ((umap-T-uobj
           (cl-transforms:pose->transform
            (btr:pose
             (btr:object btr:*current-bullet-world*
                         (object-type-filter-bullet type)))))
         (prolog-type (object-type-filter-prolog type))
         (smap-T-sobj
           (if (eql time :start)
               (query-object-location-by-object-type prolog-type "Start")
               (query-object-location-by-object-type prolog-type "End")))
         (smap-T-scamera
           (if (eql time :start)
               (query-camera-location-by-object-type prolog-type "Start")
               (query-camera-location-by-object-type prolog-type "End")))
         (umap-T-ucamera
           (cl-transforms:transform*
            umap-T-uobj
            (cl-transforms:transform-inv smap-T-sobj)
            smap-T-scamera)))
    umap-T-ucamera))



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
      (query-object-location-by-object-type prolog-object-type "Start"))
     (query-hand-location-by-object-type prolog-object-type "Start")
     (human-to-robot-hand-transform))))
