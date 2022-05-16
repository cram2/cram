;;;
;;; Copyright (c) 2020, Gayane Kazhoyan <kazhoyan [at] cs.uni-bremen.de>
;;;                     Sebastian Koralewski <seba [at] uni-bremen.de>
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

(in-package :ralf-cm)

(defmethod costmap:costmap-generator-name->score
    ((name (eql 'learned-pose-distribution)))
  3)

(def-fact-group ralf-costmaps (costmap:desig-costmap)
  (<- (costmap:desig-costmap ?some-designator ?costmap)
    (desig:current-designator ?some-designator ?designator)
    (once (or (and (rob-int:reachability-designator ?designator)
                   (equal ?location-type :reachable-for))
              (and (rob-int:visibility-designator ?designator)
                   (equal ?location-type :visible-for))))
    (spec:property ?designator (:object ?some-object-designator))
    (desig:current-designator ?some-object-designator ?object-designator)
    (spec:property ?object-designator (:type ?object-type))
    (rob-int:environment-name ?environment-name)
    (costmap:costmap ?costmap)
    (-> (man-int:object-type-subtype :container ?object-type)
        ;; opening/closing doors/drawers
        (and (spec:property ?object-designator (:urdf-name ?container-name))
             ;; (spec:property ?object-designator (:part-of ?btr-environment))
             ;; (spec:property ?designator (:arm ?arm))
             (lisp-fun calculate-learned-mean-and-covariance
                       ?environment-name ?location-type ?container-name nil
                       (?learned-mean ?learned-covariance))
             (lisp-pred identity ?learned-mean)
             (costmap:costmap-add-function
              learned-pose-distribution
              (costmap:make-gauss-cost-function
               ?learned-mean ?learned-covariance)
              ?costmap))
        ;; fetching or placing items
        (and
         (once
          (or
           ;; if the object has a pose property, it's a recently perceived obj
           (and (spec:property ?object-designator (:pose ?_))
                (spec:property ?object-designator (:location ?some-obj-loc)))
           ;; otherwise, it's an object in hand or somewhere else on the robot
           ;; or a yet unperceived object (latter only for visibility cms)
           (spec:property ?designator (:location ?some-obj-loc))))
         (desig:current-designator ?some-obj-loc ?object-location)
         (once (or (spec:property ?object-location (:on ?obj-loc-object))
                   (spec:property ?object-location (:in ?obj-loc-object))))
         (desig:current-designator ?obj-loc-object ?object-location-object)
         (spec:property ?object-location-object (:urdf-name ?ref-loc-name))
         (lisp-fun calculate-learned-mean-and-covariance
                   ?environment-name ?location-type
                   ?object-type ?ref-loc-name
                   (?learned-mean ?learned-covariance))
         (format "means: ~a~%~%" ?learned-mean)
         (format "cov: ~a~%~%" ?learned-covariance)
         (lisp-pred identity ?learned-mean)
         (costmap:costmap-add-function
          learned-pose-distribution
          (costmap:make-gauss-cost-function
           ?learned-mean ?learned-covariance)
          ?costmap)))))
