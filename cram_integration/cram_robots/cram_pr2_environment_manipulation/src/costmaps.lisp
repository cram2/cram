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
  (let ((meancovs (location-costmap:2d-pose-covariance poses 0.09)))
    (location-costmap:make-gauss-cost-function (first meancovs)
                                               (second meancovs))))

;; NOTE(cpo): Maybe this should take the current position and get a desired position of the handle, so the
;; costmap is more precise. Now it calculates the min and max position, so everything should be reachable.
(defun make-container-handle-reachable-cost-function (container-desig)
  "Make a cost function for the the PR2 to reach a containers handle in neutral and manipulated position."
  (let* ((handle-link (get-handle-link (car (alexandria:assoc-value (desig:description container-desig) :name))))
         (neutral-handle-pose (get-manipulated-pose (cl-urdf:name handle-link) 0 :relative T))
         (manipulated-handle-pose (get-manipulated-pose (cl-urdf:name handle-link) 1 :relative T)))
    (make-poses-reachable-cost-function (list neutral-handle-pose manipulated-handle-pose))))

(defun get-aabb (container-name)
  (btr:aabb (btr:rigid-body (btr:object btr:*current-bullet-world* :kitchen) (btr::make-rigid-body-name "KITCHEN" container-name :pr2-em))))

(defun get-width (container-name direction)
  "Return the width of the container with name `container-name', appropiate for the joint `direction' given."
  (let ((dimensions (cl-bullet:bounding-box-dimensions (get-aabb container-name)))
        (norm-direction (cl-tf:normalize-vector direction)))
    (if (> (abs (cl-tf:x norm-direction)) (abs (cl-tf:y norm-direction)))
        (cl-tf:y dimensions)
        (cl-tf:x dimensions))))

;; NOTE(cpo): Same as above.
(defun make-opened-drawer-cost-function (?container-desig &optional (padding 0.2))
  "Resolve the relation according to the poses of the handle of `container-desig' in neutral and manipulated form."
  (let* ((container-name (string-downcase
                          (car (alexandria:assoc-value
                                (desig:description ?container-desig)
                                :name))))
         (handle-link (get-handle-link container-name))
         (handle-pose (get-manipulated-pose (cl-urdf:name handle-link) 0 :relative T))
         (manipulated-handle-pose (get-manipulated-pose (cl-urdf:name handle-link) 1 :relative T))
         (neutral-point (cl-tf:make-3d-vector (cl-tf:x (cl-tf:origin handle-pose))
                                              (cl-tf:y (cl-tf:origin handle-pose))
                                              0))
         (manipulated-point (cl-tf:make-3d-vector (cl-tf:x (cl-tf:origin manipulated-handle-pose))
                                                  (cl-tf:y (cl-tf:origin manipulated-handle-pose))
                                                  0))
         (V (cl-tf:v- manipulated-point neutral-point))
         (width (get-width container-name V)))
    (lambda (x y)
      (multiple-value-bind (a b c)
          (line-equation-in-xy neutral-point
                               manipulated-point)
        (let* ((P (cl-tf:make-3d-vector x y 0))
               (dist (line-p-dist a b c P))
               (dist-p (line-p-dist-point a b c P)))
          (if (and
               (< dist (+ (/ width 2) padding))
               (< (cl-tf:v-norm (cl-tf:v- dist-p neutral-point)) (+ (cl-tf:v-norm V) padding)))
              0
              1))))))

(defun make-opened-drawer-side-cost-function (?container-desig arm)
  (let* ((container-name (string-downcase
                          (car (alexandria:assoc-value
                                (desig:description ?container-desig)
                                :name))))
         (handle-link (get-handle-link container-name))
         (handle-pose (get-manipulated-pose (cl-urdf:name handle-link) 0 :relative T))
         (manipulated-handle-pose (get-manipulated-pose (cl-urdf:name handle-link) 1 :relative T))
         (neutral-point (cl-tf:make-3d-vector (cl-tf:x (cl-tf:origin handle-pose))
                                              (cl-tf:y (cl-tf:origin handle-pose))
                                              0))
         (manipulated-point (cl-tf:make-3d-vector (cl-tf:x (cl-tf:origin manipulated-handle-pose))
                                                  (cl-tf:y (cl-tf:origin manipulated-handle-pose))
                                                  0))
         (AB (cl-tf:v- manipulated-point neutral-point)))
    (lambda (x y)
      (let ((d (cl-tf:z (cl-tf:cross-product AB
                                             (cl-tf:v- (cl-tf:make-3d-vector x y 0)
                                                       neutral-point)))))
        (if (and (< d 0) (eql arm :right))
            1.0
            (if (and (> d 0) (eql arm :left))
                1.0
                0.0))))))

(defmethod location-costmap:costmap-generator-name->score ((name (eql 'poses-reachable-cost-function))) 10)

(defmethod location-costmap:costmap-generator-name->score ((name (eql 'container-handle-reachable-cost-function))) 10)

(defmethod location-costmap:costmap-generator-name->score ((name (eql 'opened-drawer-cost-function))) 10)

(defmethod location-costmap:costmap-generator-name->score ((name (eql 'opened-drawer-side-cost-function))) 10)

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
    (location-costmap:costmap-reach-minimal-distance ?padding)
    (location-costmap:costmap-add-function
     opened-drawer-cost-function
     (make-opened-drawer-cost-function ?container-designator ?padding)
     ?costmap)
    (desig:desig-prop ?designator (:arm ?arm))
    (location-costmap:costmap-add-function
     opened-drawer-side-cost-function
     (make-opened-drawer-side-cost-function ?container-designator ?arm)
     ?costmap)
    )
  )
