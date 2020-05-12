;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :gaussian-costmap)

(defmethod costmap-generator-name->score ((name (eql 'pose-distribution))) 5)
(defmethod costmap-generator-name->score ((name (eql 'reachable-from-space))) 5)
(defmethod costmap-generator-name->score ((name (eql 'reachable-from-weighted))) 4)

(defclass pose-distribution-range-include-generator () ())

(defclass pose-distribution-range-exclude-generator () ())

(defmethod costmap-generator-name->score
    ((name pose-distribution-range-include-generator))
  7)

(defmethod costmap-generator-name->score
    ((name pose-distribution-range-exclude-generator))
  6)

(def-fact-group robot-pose-gaussian-costmap (desig-costmap)

  (<- (desig-costmap ?desig ?cm)
    (cram-robot-interfaces:visibility-designator ?desig)
    ;; (bagof ?pose (desig-location-prop ?desig ?pose) ?poses)
    (once (or (and (desig:desig-prop ?desig (:object ?some-object))
                   (desig:current-designator ?some-object ?object)
                   ;; (btr-belief:object-designator-name ?object ?object-name)
                   ;; (btr:bullet-world ?world)
                   ;; (btr:object-pose ?world ?object-name ?to-see-pose)
                   (lisp-fun man-int:get-object-pose-in-map ?object ?to-see-pose)
                   (lisp-pred identity ?to-see-pose))
              (and (desig:desig-prop ?desig (:location ?some-location))
                   (desig:current-designator ?some-location ?location)
                   (desig:designator-groundings ?location ?location-poses)
                   ;; have to take one pose from all possibilities
                   ;; as later we have a FORCE-LL on ?TO-SEE-POSES
                   ;; and that can be an infinitely long list
                   (member ?to-see-pose ?location-poses))))
    (lisp-fun list ?to-see-pose ?to-see-poses)
    (lisp-fun 2d-pose-covariance ?to-see-poses 0.5 (?mean ?covariance))
    (costmap ?cm)
    (costmap-add-function
     pose-distribution
     (make-gauss-cost-function ?mean ?covariance)
     ?cm)
    (costmap:visibility-costmap-size ?distance)
    (instance-of pose-distribution-range-include-generator ?include-generator-id)
    (costmap-add-function
     ?include-generator-id
     (make-range-cost-function ?mean ?distance)
     ?cm)
    (costmap:orientation-samples ?samples)
    (costmap:orientation-sample-step ?sample-step)
    (costmap-add-orientation-generator
     (make-angle-to-point-generator ?mean :samples ?samples :sample-step ?sample-step)
     ?cm)
    (costmap-add-height-generator
     (make-constant-height-function 0.0)
     ?cm))

  (<- (desig-costmap ?desig ?cm)
    (cram-robot-interfaces:reachability-designator ?desig)
    ;; (bagof ?pose (cram-robot-interfaces:designator-reach-pose ?desig ?pose ?_) ?poses)
    (once (or (and (desig:desig-prop ?desig (:object ?some-object))
                   (desig:current-designator ?some-object ?object)
                   (lisp-fun man-int:get-object-pose-in-map ?object ?to-reach-pose)
                   (lisp-pred identity ?to-reach-pose))
              (and (desig:desig-prop ?desig (:location ?some-location))
                   (desig:current-designator ?some-location ?location)
                   (desig:designator-groundings ?location ?location-poses)
                   ;; have to take one pose from all possibilities
                   ;; as later we have a FORCE-LL on ?TO-REACH-POSES
                   ;; and that can be an infinitely long list
                   (member ?to-reach-pose ?location-poses))))
    (lisp-fun list ?to-reach-pose ?to-reach-poses)
    (lisp-fun cut:force-ll ?to-reach-poses ?poses-list)
    (lisp-fun 2d-pose-covariance ?to-reach-poses 0.5 (?mean ?covariance))
    (costmap-in-reach-distance ?distance)
    (costmap-reach-minimal-distance ?minimal-distance)
    (costmap ?cm)
    (forall
     (member ?pose ?to-reach-poses)
     (and
      (instance-of pose-distribution-range-include-generator
                   ?include-generator-id)
      (costmap-add-function
       ?include-generator-id
       (make-range-cost-function ?pose ?distance) ?cm)
      (instance-of pose-distribution-range-exclude-generator
                   ?exclude-generator-id)
      (costmap-add-function
       ?exclude-generator-id
       (make-range-cost-function ?pose ?minimal-distance :invert t)
       ?cm)))
    (costmap:orientation-samples ?samples)
    (costmap:orientation-sample-step ?sample-step)
    (once (or (costmap:reachability-orientation-offset ?offset)
              (equal ?offset 0.0)))
    (costmap-add-orientation-generator
     (make-angle-to-point-generator ?mean :samples ?samples
                                          :sample-step ?sample-step
                                          :sample-offset ?offset)
     ?cm)
    (costmap-add-height-generator
     (make-constant-height-function 0.0)
     ?cm))

  ;; poses reachable-from ?pose for the robot
  ;; ?pose should usually be robot's current pose I suppose
  (<- (desig-costmap ?desig ?cm)
    (costmap ?cm)
    (desig-prop ?desig (:reachable-from ?from-what))
    (or (and (lisp-type ?from-what symbol)
             ;; (cram-robot-interfaces:robot ?from-what)
             (lisp-fun cram-tf:robot-current-pose ?pose))
        (and (lisp-type ?from-what cl-transforms:pose)
             (equal ?pose ?from-what)))
    (lisp-fun cl-transforms:origin ?pose ?point)
    (costmap-in-reach-distance ?distance)
    (costmap-add-function reachable-from-space
                          (make-range-cost-function ?point ?distance)
                          ?cm)
    (costmap-add-function reachable-from-weighted
                          (make-location-cost-function ?pose ?distance)
                          ?cm)))
