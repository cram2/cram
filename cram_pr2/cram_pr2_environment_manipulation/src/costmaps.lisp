;;;
;;; Copyright (c) 2018, Christopher Pollok <cpollok@uni-bremen.de>
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

(in-package :pr2-em)

;;; DRAWER COSTMAP

(defun make-poses-reachable-cost-function (poses)
  "`poses' are the poses according to which the relation is resolved."
  (let ((meancovs (location-costmap:2d-pose-covariance poses 0.75)))
    (location-costmap:make-gauss-cost-function (first meancovs)
                                               (second meancovs))))

(defun make-container-handle-reachable-cost-function (container-desig)
  (let* ((handle-link (get-handle-link (car (alexandria:assoc-value (desig:description container-desig) :name))))
         (handle-pose (get-urdf-link-pose (cl-urdf:name handle-link)))
         (manipulated-handle-pose (get-manipulated-pose (cl-urdf:name handle-link) 1 :relative T)))
    (make-poses-reachable-cost-function (list handle-pose manipulated-handle-pose))))

(defun make-opened-drawer-cost-function (?container-desig)
  "Resolve the relation according to the poses of the handle of `container-desig' in neutral and manipulated form."
  (let* ((handle-link (get-handle-link (car (alexandria:assoc-value (desig:description ?container-desig) :name))))
         (?handle-pose (get-urdf-link-pose (cl-urdf:name handle-link)))
         (?manipulated-handle-pose (get-manipulated-pose (cl-urdf:name handle-link) 1 :relative T))
         (width 0.7)
         (neutral-x (cl-tf:x (cl-tf:origin ?handle-pose)))
         (neutral-y (cl-tf:y (cl-tf:origin ?handle-pose)))
         (manipulated-x (cl-tf:x (cl-tf:origin ?manipulated-handle-pose)))
         (manipulated-y (cl-tf:y (cl-tf:origin ?manipulated-handle-pose)))
         (neutral-point (cl-tf:make-3d-vector neutral-x neutral-y 0))
         (manipulated-point (cl-tf:make-3d-vector manipulated-x manipulated-y 0))
         (V (cl-tf:v- manipulated-point neutral-point)))
    (lambda (x y)
      (multiple-value-bind (a b c)
          (line-equation-in-xy neutral-point
                               manipulated-point)
        (let* ((P (cl-tf:make-3d-vector x y 0))
               (dist (line-p-dist a b c P))
               (dist-p (line-p-dist-point a b c P)))
          (if (and
               (< dist (/ width 2))
               (< (cl-tf:v-norm (cl-tf:v- dist-p neutral-point)) (cl-tf:v-norm V)))
              0
              1))))))

(defun line-p-dist (a b c p)
  "Return the disctance between the line described by ax + bx + c and the point p (in the x-y plane)."
  (let ((x (cl-tf:x p))
        (y (cl-tf:y p)))
    (/
     (abs (+ (* a x) (* b y) c))
     (sqrt (+ (expt a 2) (expt b 2))))))

(defun line-p-dist-point (a b c p)
  "Return the point on the line described by ax + bx + c from which the distance to the point (x,y) is minimal."
  (let ((px (cl-tf:x p))
        (py (cl-tf:y p)))
    (multiple-value-bind (x y)
        (distance-point a b c px py)
      (cl-tf:make-3d-vector x y 0))))

(defmethod location-costmap:costmap-generator-name->score ((name (eql 'poses-reachable-cost-function))) 10)

(defmethod location-costmap:costmap-generator-name->score ((name (eql 'container-handle-reachable-cost-function))) 10)

(defmethod location-costmap:costmap-generator-name->score ((name (eql 'opened-drawer-cost-function))) 10)

(def-fact-group environment-manipulation-costmap (location-costmap:desig-costmap)
  (<- (location-costmap:desig-costmap ?designator ?costmap)
    ;;(or (cram-robot-interfaces:visibility-designator ?desig)
    ;;    (cram-robot-interfaces:reachability-designator ?desig))
    (desig:desig-prop ?designator (:poses ?poses))
    (location-costmap:costmap ?costmap)
    (location-costmap:costmap-add-function
     poses-reachable-cost-function
     (make-poses-reachable-cost-function ?poses)
     ?costmap))

  (<- (location-costmap:desig-costmap ?designator ?costmap)
    (desig:desig-prop ?designator (:container ?container-designator))
    (desig:desig-prop ?container-designator (:type :container))
    (location-costmap:costmap ?costmap)
    (location-costmap:costmap-add-function
     container-handle-reachable-cost-function
     (make-container-handle-reachable-cost-function ?container-designator)
     ?costmap)
    (location-costmap:costmap-add-function
     opened-drawer-cost-function
     (make-opened-drawer-cost-function ?container-designator)
     ?costmap)
    )
  )
