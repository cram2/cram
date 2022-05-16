;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :demos)

(defun make-dm-room-restricted-area-cost-function ()
  (lambda (x y)
    (if (> x 4.0)
        0.0
        (if (< x 0.0)
            0.0
            (if (> y 1.0)
                0.0
                (if (< y -1.0)
                    0.0
                    1.0))))))

(defun make-small-retail-restricted-area-cost-function ()
  (lambda (x y)
    (if (> x 2.0)
        0.0
        (if (< x -5.0)
            0.0
            (if (> y 1.0)
                0.0
                (if (< y -1.0)
                    0.0
                    1.0))))))

(defun make-iai-kitchen-assembly-restricted-area-cost-function ()
  (lambda (x y)
    (declare (ignore y))
    (if (> x -1.2)
        0.0
        1.0)))

(defun make-iai-kitchen-household-restricted-area-cost-function ()
  (lambda (x y)
    (if (> x 1.2)
        0.0
        (if (and (> x 0.5) (> y -1.5) (< y 2.0))
            1.0
            (if (and (> x 0.0) (> y -1.5) (< y 1.0))
                1.0
                (if (and (> x -1.5) (> y -1.5) (< y 2.5))
                    1.0
                    (if (and (> x -4.0) (> y -1.0) (< y 1.0))
                        1.0
                        0.0)))))))

(defun make-restricted-area-cost-function ()
  (ecase (rob-int:get-environment-name)
    (:iai-kitchen
     (if (btr:object btr:*current-bullet-world* :big-wooden-plate)
         (make-iai-kitchen-assembly-restricted-area-cost-function)
         (make-iai-kitchen-household-restricted-area-cost-function)))
    ((or :dm-room :store)
     (make-dm-room-restricted-area-cost-function))
    (:dm-shelves
     (make-small-retail-restricted-area-cost-function))))

(defmethod costmap:costmap-generator-name->score ((name (eql 'restricted-area)))
  5)

(def-fact-group demo-restricted-ground-costmap (costmap:desig-costmap)
  (<- (costmap:desig-costmap ?designator ?costmap)
    (or (rob-int:visibility-designator ?designator)
        (rob-int:reachability-designator ?designator))
    ;; make sure that the location is not on the robot itself
    ;; if it is, don't generate a costmap
    (once (or (and (desig:desig-prop ?designator (:object ?some-object))
                   (desig:current-designator ?some-object ?object)
                   (lisp-fun man-int:get-object-pose-in-map ?object ?to-reach-pose)
                   (lisp-pred identity ?to-reach-pose)
                   (-> (desig:desig-prop ?object (:location ?loc))
                       (not (man-int:location-always-reachable ?loc))
                       (true)))
              (and (desig:desig-prop ?designator (:location ?some-location))
                   (desig:current-designator ?some-location ?location)
                   ;; if the location is on the robot itself,
                   ;; don't use the costmap
                   (not (man-int:location-always-reachable ?location)))))
    (costmap:costmap ?costmap)
    (costmap:costmap-add-function
     restricted-area
     (make-restricted-area-cost-function)
     ?costmap)))
