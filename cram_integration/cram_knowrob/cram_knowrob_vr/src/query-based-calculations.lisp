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

(defparameter *human-feet-offset* 0.2 "in meters")

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
            0))
         (map-T-base
           (cl-transforms:make-transform
            base-origin
            base-orientation))
         (base-T-offset-base
           (cl-transforms:make-transform
            (cl-transforms:make-3d-vector (- *human-feet-offset*) 0 0)
            (cl-transforms:make-identity-rotation)))
         (map-T-base-offset
           (cl-transforms:transform*
            map-T-base base-T-offset-base)))
    (cl-transforms-stamped:make-pose-stamped
     cram-tf:*fixed-frame*
     0.0
     (cl-transforms:translation map-T-base-offset)
     (cl-transforms:rotation map-T-base-offset))))

(defun umap-P-uobj-through-surface-ll (type start-or-end)
  "Calculates the pose of the object in map relative to its supporting surface.
Formula: umap-T-uobj = umap-T-usurface * inv(smap-T-ssurface) * smap-T-sobj.
`type' is a simple symbol such as 'milk."
  (let ((name-dim-and-surface-T-object-ll
          (query-surface-name-dim-and-surface-T-object-by-object-type
           (object-type-filter-prolog type)
           start-or-end
           :table-setting)))
    (cut:lazy-mapcar
     (lambda (name-dim-and-surface-T-object)
       (print name-dim-and-surface-T-object)
       (destructuring-bind (surface-name ssurface-dimensions ssurface-T-sobject)
           name-dim-and-surface-T-object
         (let* ((usurface-object
                  (btr:rigid-body
                   (btr:get-environment-object)
                   (match-kitchens surface-name)))
                (usurface-dimensions
                  (btr:calculate-bb-dims usurface-object))
                (scale-vector
                  ;; uncomment this instead if you want to turn off scaling
                  ;; (cl-transforms:make-3d-vector 1 1 1)
                  (cl-transforms:v/-pairwise
                   usurface-dimensions ssurface-dimensions))
                (scaled-ssurface-T-sobject
                  (cl-transforms:copy-transform
                   ssurface-T-sobject
                   :translation (cl-transforms:v*-pairwise
                                 (cl-transforms:translation ssurface-T-sobject)
                                 scale-vector)))
                (umap-T-usurface
                  (cl-transforms:pose->transform
                   (btr:pose usurface-object)))
                (umap-T-uobj
                  (cl-transforms:transform*
                   umap-T-usurface scaled-ssurface-T-sobject)))
           (cl-transforms-stamped:make-pose-stamped
            cram-tf:*fixed-frame*
            0.0
            (cl-transforms:translation umap-T-uobj)
            (cl-transforms:rotation umap-T-uobj)))))
     name-dim-and-surface-T-object-ll)))












;;;;;;; COPY PASTED FROM SPATIAL RELATIONS CM
(defun get-closest-edge-and-distance (obj-transform supp-obj-transform supp-obj-dims)
  "The supp-obj is supposed to be rectangular (and have 4 edges obviously)
with y axis (in table coordinate system) pointing towards its left edge
and x - to the back. `obj-transform' should be in the world frame.
The function returns one of the following keys: :front, :back, :left, :right."
  (declare (type cl-transforms:transform obj-transform supp-obj-transform)
           (type cl-transforms:3d-vector supp-obj-dims))
  (flet ((check-relation-p (dimensions/2 coords pred-1 pred-2 ratio-x ratio-y)
           (let ((edge-x-dist
                   (funcall pred-1
                            (cl-transforms:x dimensions/2)
                            (cl-transforms:x coords)))
                 (edge-y-dist
                   (funcall pred-2
                            (cl-transforms:y dimensions/2)
                            (cl-transforms:y coords))))
             (if (< (* edge-x-dist ratio-x)
                    (* edge-y-dist ratio-y))
                 (values T edge-x-dist)
                 (values NIL edge-y-dist))))
         (get-quarter-in-supp-obj (coords)
           (if (> (cl-transforms:x coords) 0)
               (if (> (cl-transforms:y coords) 0)
                   :back-left :back-right)
               (if (> (cl-transforms:y coords) 0)
                   :front-left :front-right))))
    (let* ((world->supp-transform
             supp-obj-transform)
           (supp->world-transform
             (cl-transforms:transform-inv world->supp-transform))
           (obj-coords-in-supp
             (cl-transforms:transform-point
              supp->world-transform
              (cl-transforms:translation obj-transform)))
           (dimensions/2
             (cl-transforms:v* supp-obj-dims 0.5))
           (ratio-x
             1.0d0)
           (ratio-y
             1.0d0)
           (quarter
             (get-quarter-in-supp-obj obj-coords-in-supp)))
      ;; find which edges of supp-obj are longer
      ;; longer edges are more preferred, ratio decides how much more preferred
      (if (> (cl-transforms:x dimensions/2) (cl-transforms:y dimensions/2))
          (setf ratio-x (/ (cl-transforms:x dimensions/2)
                           (cl-transforms:y dimensions/2)))
          (setf ratio-y (/ (cl-transforms:y dimensions/2)
                           (cl-transforms:x dimensions/2))))
      ;; find the edge of supp-obj to which obj is the closest
      ;; first find in which quarter of supp obj it is and then compare the 2 edges
      (let (pred-x pred-y)
        (ecase quarter
          (:back-left                  ; obj.x > 0, obj.y > 0
           (setf pred-x #'-
                 pred-y #'-))
          (:back-right                 ; obj.y < 0
           (setf pred-x #'-
                 pred-y #'+))
          (:front-left                 ; obj.x < 0
           (setf pred-x #'+
                 pred-y #'-))
          (:front-right                ; obj.x < 0 and obj.y < 0
           (setf pred-x #'+
                 pred-y #'+)))
        (multiple-value-bind (condition distance)
            (check-relation-p dimensions/2 obj-coords-in-supp
                              pred-x pred-y ratio-x ratio-y)
          (list (ecase quarter
                  (:back-left (if condition :back :left))
                  (:back-right (if condition :back :right))
                  (:front-left (if condition :front :left))
                  (:front-right (if condition :front :right)))
                distance))))))





(defun umap-P-uobj-through-surface-edge-ll (type start-or-end)
  "Calculates the pose of the object in map
relative to its supporting surface's closest edge.
Formula: umap-T-uobj = umap-T-usurface * ssurface-T-obj
                     = umap-T-usurface * ssurface-T-smap * smap-T-sobj
                     = umap-T-usurface * inv(smap-T-ssurface) * smap-T-sobj.
`type' is a simple symbol such as 'milk."
  (let ((surface-name-dim-transform-and-object-transform-ll
          (query-surface-name-dim-and-surface-T-object-by-object-type
           (object-type-filter-prolog type)
           start-or-end
           :table-setting)))
    (cut:lazy-mapcar
     (lambda (surface-name-dim-transform-and-object-transform)
       (let* ((surface-name
                (first surface-name-dim-transform-and-object-transform))
              ;; (surface-dimensions
              ;;   (second surface-name-dim-transform-and-object-transform))
              (smap-T-ssurface
                (third surface-name-dim-transform-and-object-transform))
              (smap-T-sobject
                (fourth surface-name-dim-transform-and-object-transform))

              ;; (closest-edge-and-distance
              ;;   (get-closest-edge-and-distance
              ;;    smap-T-sobject smap-T-ssurface surface-dimensions))
              ;; (closest-edge
              ;;   (first closest-edge-and-distance))
              ;; (closest-edge-distance
              ;;   (second closest-edge-and-distance))

              (ssurface-T-sobject
                (cl-transforms:transform*
                 (cl-transforms:transform-inv smap-T-ssurface)
                 smap-T-sobject))

              (usurface-obj
                (btr:rigid-body
                 (btr:get-environment-object)
                 (match-kitchens surface-name)))

              (umap-T-usurface
                (cl-transforms:pose->transform
                 (btr:pose usurface-obj)))
              (umap-T-uobj
                (cl-transforms:transform*
                 umap-T-usurface ssurface-T-sobject))

              ;; (usurface-dimensions
              ;;   (btr:calculate-bb-dims usurface-obj))

              ;; (uclosest-edge-and-distance
              ;;   (get-closest-edge-and-distance
              ;;    umap-T-uobj umap-T-usurface usurface-dimensions))
              )

         ;; (print closest-edge-and-distance)
         ;; (print uclosest-edge-and-distance)
         (cl-transforms-stamped:make-pose-stamped
          cram-tf:*fixed-frame*
          0.0
          (cl-transforms:translation umap-T-uobj)
          (cl-transforms:rotation umap-T-uobj))))
     surface-name-dim-transform-and-object-transform-ll)))









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

(defun umap-P-object-through-other-object-ll (type umap-T-uother-object)
  "Calculates the pose for an object to be placed at a surface,
relative to another object (in our case bowl) and the human actor.
Formula:
  part 1
  umap-T-uobject = umap-T'-ubowl * ubowl-T'-uobject =
                   [umap-R-urobot | umap-t-ubowl] * [I | sbowl-t'-sobject],
  part 2
  where sbowl-t'-sobject = translation(inv(smap-T'-sbowl) * smap-T'-sobject) =
                           translation(inv([smap-R-scamera-projected | smap-t-sbowl]) *
                                       [smap-R-scamera-projected | smap-t-sobject]))
  part 3
  and umap-R-urobot = rotation(umap-T-usurface * usurface-T-urobot),
      where usurface-T-urobot = [usurface-R-urobot | 0 0 0 ],
            where usurface-R-urobot = axis-angle->quaternion
                                         axis = (0 0 1)
                                         angle = (ecase closest-edge
                                                    (:front 0)
                                                    (:back pi)
                                                    (:left -pi/2)
                                                    (:right pi/2))"
  ;; part 2
  (let ((camera-bowl-object-transforms-ll
          (query-obj-and-camera-pose-depending-on-bowl
           (object-type-filter-prolog type))))
    (cut:lazy-mapcar
     (lambda (camera-bowl-object-transforms)
       (destructuring-bind (smap-T-scamera smap-T-sbowl smap-T-sobject ssurface-name)
           camera-bowl-object-transforms
         (let* ((smap-T-scamera-projected
                  (map-T-camera->map-P-base smap-T-scamera))
                (smap-R-scamera-projected
                  (cl-transforms:orientation smap-T-scamera-projected))
                (smap-tt-sobject
                  (cl-transforms:translation smap-T-sobject))
                (smap-T-prime-sobject
                  (cl-transforms:make-transform
                   smap-tt-sobject smap-R-scamera-projected))
                (smap-tt-sbowl
                  (cl-transforms:translation smap-T-sbowl))
                (smap-T-prime-sbowl
                  (cl-transforms:make-transform
                   smap-tt-sbowl smap-R-scamera-projected))
                (sbowl-T-prime-sobject
                  (cl-transforms:transform*
                   (cl-transforms:transform-inv smap-T-prime-sbowl)
                   smap-T-prime-sobject))
                (sbowl-tt-prime-sobject
                  (cl-transforms:translation sbowl-T-prime-sobject)))
           ;; part 3
           (let* ((umap-T-ubowl
                    umap-T-uother-object)
                  (usurface-obj
                    (btr:rigid-body
                     (btr:get-environment-object)
                     (match-kitchens ssurface-name)))
                  (umap-T-usurface
                    (cl-transforms:pose->transform
                     (btr:pose usurface-obj)))
                  (usurface-dimensions
                    (btr:calculate-bb-dims usurface-obj))
                  (uclosest-edge-and-distance
                    (get-closest-edge-and-distance
                     umap-T-ubowl umap-T-usurface usurface-dimensions))
                  (closest-edge
                    (first uclosest-edge-and-distance))
                  (usurface-R-urobot
                    (cl-transforms:axis-angle->quaternion
                     (cl-transforms:make-3d-vector 0 0 1)
                     (ecase closest-edge
                       (:front 0)
                       (:back pi)
                       (:left (/ pi -2))
                       (:right (/ pi 2)))))
                  (usurface-T-urobot
                    (cl-transforms:make-transform
                     (cl-transforms:make-identity-vector)
                     usurface-R-urobot))
                  (umap-T-urobot
                    (cl-transforms:transform* umap-T-usurface usurface-T-urobot))
                  (umap-R-urobot
                    (cl-transforms:rotation umap-T-urobot)))
             ;; part 1
             (let* ((umap-tt-ubowl
                      (cl-transforms:translation umap-T-ubowl))
                    (umap-T-prime-ubowl
                      (cl-transforms:make-transform
                       umap-tt-ubowl umap-R-urobot))
                    (ubowl-T-prime-uobject
                      (cl-transforms:make-transform
                       sbowl-tt-prime-sobject (cl-transforms:make-identity-rotation)))
                    (umap-T-uobject
                      (cl-transforms:transform* umap-T-prime-ubowl ubowl-T-prime-uobject)))
               (cl-transforms:transform->pose umap-T-uobject))))))
     camera-bowl-object-transforms-ll)))




(defun base-poses-ll-for-searching (type)
  (let ((umap-T-ucamera-ll
          (umap-T-ucamera-through-surface-ll type "Start")))
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

(defun base-poses-ll-for-fetching-based-on-object-pose (bullet-type umap-P-uobj)
  (let* ((prolog-type
           (roslisp-utilities:rosify-lisp-name
            (object-type-fixer bullet-type)))
         (umap-T-ucamera-ll
           (umap-T-ucamera-through-object-ll-based-on-object-pose
            prolog-type "Start" umap-P-uobj))
         (umap-T-ucamera-end-ll
           (umap-T-ucamera-through-object-ll-based-on-object-pose
            prolog-type "End" umap-P-uobj)))
    (cut:lazy-mapcar
     (lambda (umap-T-ucamera)
       (map-T-camera->map-P-base umap-T-ucamera))
     (append (cut:force-ll umap-T-ucamera-ll)
             (cut:force-ll umap-T-ucamera-end-ll)))))

(defun base-poses-ll-for-fetching-based-on-object-desig (object-designator)
  (let ((bullet-type (desig:desig-prop-value object-designator :type))
        (umap-P-uobj (man-int:get-object-pose-in-map object-designator)))
    (base-poses-ll-for-fetching-based-on-object-pose bullet-type umap-P-uobj)))

(defun base-poses-ll-for-placing (type)
  (let ((umap-T-ucamera-ll
          (umap-T-ucamera-through-surface-ll type "End")))
    (cut:lazy-mapcar
     (lambda (umap-T-ucamera)
       (map-T-camera->map-P-base umap-T-ucamera))
     umap-T-ucamera-ll)))

(defun look-poses-ll-for-searching (type)
  (umap-P-uobj-through-surface-ll type "Start"))




(defun object-poses-ll-for-placing (type umap-T-uother-object)
  (print umap-T-uother-object)
  (umap-P-object-through-other-object-ll type umap-T-uother-object)
  ;; (umap-P-uobj-through-surface-ll type "End")
  ;; (umap-P-uobj-through-surface-edge-ll type "End")
  )


(defun arms-for-fetching-ll (type)
  (query-hand (object-type-filter-prolog type)))


(defun object-grasped-faces-ll-from-prolog-type (prolog-type)
  (let ((object-T-hand-ll
          (query-object-T-hand-by-object-type prolog-type "Start")))
    (cut:lazy-mapcar
     (lambda (object-T-hand)
       (let* ((object-translation-hand
                (cl-transforms:translation object-T-hand))
              (x
                (cl-transforms:x object-translation-hand))
              (y
                (cl-transforms:y object-translation-hand))
              (z
                (cl-transforms:z object-translation-hand))
              (object-grasped-face
                (man-int::calculate-vector-face
                 (list x y z))))
         object-grasped-face))
     object-T-hand-ll)))

(defun object-grasped-faces-ll-from-kvr-type (kvr-type)
  (let ((prolog-type
          (object-type-filter-prolog kvr-type)))
    (object-grasped-faces-ll-from-prolog-type prolog-type)))

(defun object-grasped-faces-ll (bullet-type)
  (let ((prolog-type
          (roslisp-utilities:rosify-lisp-name
           (object-type-fixer bullet-type))))
    (object-grasped-faces-ll-from-prolog-type prolog-type)))
