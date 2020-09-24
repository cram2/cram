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

(defun calculate-learned-mean-and-covariance (object-type
                                              reference-location-name)
  (if reference-location-name
      (case object-type
        (:breakfast-cereal
         (case reference-location-name
           (:oven-area-area-right-drawer-main
            (list
             (cl-transforms:make-3d-vector 0.75288949 0.75126507 0.0)
             #2a((0.00334272 -0.00167905) (-0.00167905 0.01173699))))
           (t
            '(nil nil))))
        (:bowl
         (case reference-location-name
           (:sink-area-left-middle-drawer-main
            (list
             (cl-transforms:make-3d-vector 0.48262422 0.60007345 0.0)
             #2a((0.01696884 -0.02503274) (-0.02503274 0.18154158))))
           (:dining-area-jokkmokk-table-main
            (list
             (cl-transforms:make-3d-vector -2.58749167 -0.17260023 0.0)
             #2a((0.0004593 0.00107521) (0.00107521 0.01146658))))
           (t
            '(nil nil))))
        (t
         '(nil nil)))
      (case object-type
        (:sink-area-left-middle-drawer-main
         (list
          (cl-transforms:make-3d-vector 0.3703701 1.28296277 0.0)
          #2a((0.02345756 0.03769117) (0.03769117 0.0812215))))
        (:iai-fridge-door
         (list
          (cl-transforms:make-3d-vector 0.39157908 -0.65701128 0.0)
          #2a((0.02198436 0.01780009) (0.01780009 0.031634))))
        (:oven-area-area-right-drawer-main
         (list
          (cl-transforms:make-3d-vector 0.52883482 2.06610992 0.0)
          #2a((0.01591575 0.01717804) (0.01717804 0.02525052))))
        (t
         '(nil nil)))))
  
(defmethod costmap-generator-name->score ((name (eql 'pose-distribution))) 5)
(defmethod costmap-generator-name->score ((name (eql 'learned-pose-distribution))) 3)
(defmethod costmap-generator-name->score ((name (eql 'reachable-from-space))) 5)
(defmethod costmap-generator-name->score ((name (eql 'reachable-from-weighted))) 4)

(defclass pose-distribution-range-include-generator () ())

(defclass pose-distribution-range-exclude-generator () ())

(defmethod costmap-generator-name->score
    ((name pose-distribution-range-include-generator))
  3)

(defmethod costmap-generator-name->score
    ((name pose-distribution-range-exclude-generator))
  4)

(def-fact-group robot-pose-gaussian-costmap (desig-costmap)

  (<- (desig-costmap ?desig ?cm)
    (rob-int:visibility-designator ?desig)
    ;; (bagof ?pose (desig-location-prop ?desig ?pose) ?poses)
    (once (or (and (desig:desig-prop ?desig (:object ?some-object))
                   (desig:current-designator ?some-object ?object)
                   (lisp-fun man-int:get-object-pose-in-map ?object ?to-see-pose)
                   (lisp-pred identity ?to-see-pose)
                   (-> (desig:desig-prop ?object (:location ?loc))
                       (not (man-int:location-always-reachable ?loc))
                       (true)))
              (and (desig:desig-prop ?desig (:location ?some-location))
                   (desig:current-designator ?some-location ?location)
                   (not (man-int:location-always-reachable ?location))
                   (desig:designator-groundings ?location ?location-poses)
                   ;; have to take one pose from all possibilities
                   ;; as later we have a FORCE-LL on ?TO-SEE-POSES
                   ;; and that can be an infinitely long list
                   (member ?to-see-pose ?location-poses))))
    (lisp-fun list ?to-see-pose ?to-see-poses)
    (lisp-fun 2d-pose-covariance ?to-see-poses 0.5 (?mean ?covariance))
    (rob-int:robot ?robot-name)
    (costmap:costmap ?cm)
    (costmap:costmap-add-function
     pose-distribution
     (costmap:make-gauss-cost-function ?mean ?covariance)
     ?cm)
    (costmap:visibility-costmap-size ?robot-name ?distance)
    (instance-of pose-distribution-range-include-generator ?include-generator-id)
    (costmap:costmap-add-function
     ?include-generator-id
     (costmap:make-range-cost-function ?mean ?distance)
     ?cm)
    (costmap:orientation-samples ?robot-name ?samples)
    (costmap:orientation-sample-step ?robot-name ?sample-step)
    (costmap:costmap-add-orientation-generator
     (costmap:make-angle-to-point-generator
      ?mean :samples ?samples :sample-step ?sample-step)
     ?cm)
    (costmap:costmap-add-height-generator
     (costmap:make-constant-height-function 0.0)
     ?cm))


  ;;;; RALF
  (<- (desig-costmap ?designator ?costmap)
    (rob-int:reachability-designator ?designator)
    (costmap:costmap ?costmap)
    (spec:property ?designator (:object ?some-object-designator))
    (desig:current-designator ?some-object-designator ?object-designator)
    (spec:property ?object-designator (:type ?object-type))
    (-> (man-int:object-type-subtype :container ?object-type)
        ;; opening/closing doors/drawers
        (and (spec:property ?object-designator (:urdf-name ?container-name))
             ;; (spec:property ?object-designator (:part-of ?btr-environment))
             ;; (spec:property ?designator (:arm ?arm))
             (lisp-fun calculate-learned-mean-and-covariance
                       ?container-name nil
                       (?learned-mean ?learned-covariance))
             (lisp-pred identity ?learned-mean)
             (costmap:costmap-add-function
              learned-pose-distribution
              (costmap:make-gauss-cost-function
               ?learned-mean ?learned-covariance)
              ?costmap))
        ;; fetching or placing items
        (and (spec:property ?object-designator (:pose ?container-type))
             (spec:property ?object-designator (:location ?some-obj-loc))
             (desig:current-designator ?some-obj-loc ?object-location)
             (once
              (or (spec:property ?object-location (:on ?obj-loc-object))
                  (spec:property ?object-location (:in ?obj-loc-object))))
             (desig:current-designator ?obj-loc-object ?object-location-object)
             (spec:property ?object-location-object (:urdf-name ?ref-loc-name))
             (lisp-fun calculate-learned-mean-and-covariance
                       ?object-type ?ref-loc-name
                       (?learned-mean ?learned-covariance))
             (lisp-pred identity ?learned-mean)
             (costmap:costmap-add-function
              learned-pose-distribution
              (costmap:make-gauss-cost-function
               ?learned-mean ?learned-covariance)
              ?costmap))))


  (<- (desig-costmap ?desig ?cm)
    (rob-int:reachability-designator ?desig)
    ;; (bagof ?pose (cram-robot-interfaces:designator-reach-pose ?desig ?pose ?_) ?poses)
    (once (or (and (desig:desig-prop ?desig (:object ?some-object))
                   (desig:current-designator ?some-object ?object)
                   (lisp-fun man-int:get-object-pose-in-map ?object ?to-reach-pose)
                   (lisp-pred identity ?to-reach-pose)
                   (-> (desig:desig-prop ?object (:location ?loc))
                       (not (man-int:location-always-reachable ?loc))
                       (true)))
              (and (desig:desig-prop ?desig (:location ?some-location))
                   (desig:current-designator ?some-location ?location)
                   ;; if the location is on the robot itself,
                   ;; don't use the costmap
                   (not (man-int:location-always-reachable ?location))
                   (desig:designator-groundings ?location ?location-poses)
                   ;; have to take one pose from all possibilities
                   ;; as later we have a FORCE-LL on ?TO-REACH-POSES
                   ;; and that can be an infinitely long list
                   (member ?to-reach-pose ?location-poses))))
    (lisp-fun list ?to-reach-pose ?to-reach-poses)
    (lisp-fun cut:force-ll ?to-reach-poses ?poses-list)
    (lisp-fun costmap:2d-pose-covariance ?to-reach-poses 0.5 (?mean ?covariance))
    (costmap:costmap-in-reach-distance ?robot-name ?distance)
    (costmap:costmap-reach-minimal-distance ?robot-name ?minimal-distance)
    (costmap:costmap ?cm)
    (forall (member ?pose ?to-reach-poses)
            (and (instance-of pose-distribution-range-include-generator
                              ?include-generator-id)
                 (costmap:costmap-add-function
                  ?include-generator-id
                  (costmap:make-range-cost-function ?pose ?distance) ?cm)
                 (instance-of pose-distribution-range-exclude-generator
                              ?exclude-generator-id)
                 (costmap:costmap-add-function
                  ?exclude-generator-id
                  (costmap:make-range-cost-function
                   ?pose ?minimal-distance :invert t)
                  ?cm)))
    (rob-int:robot ?robot-name)
    (costmap:orientation-samples ?robot-name ?samples)
    (costmap:orientation-sample-step ?robot-name ?sample-step)
    (once (or (costmap:reachability-orientation-offset ?robot-name ?offset)
              (equal ?offset 0.0)))
    (costmap:costmap-add-orientation-generator
     (costmap:make-angle-to-point-generator
      ?mean :samples ?samples :sample-step ?sample-step :sample-offset ?offset)
     ?cm)
    (costmap:costmap-add-height-generator
     (make-constant-height-function 0.0)
     ?cm))

  ;; poses reachable-from ?pose for the robot
  ;; ?pose should usually be robot's current pose I suppose
  (<- (costmap:desig-costmap ?desig ?cm)
    (costmap:costmap ?cm)
    (desig-prop ?desig (:reachable-from ?from-what))
    (or (and (lisp-type ?from-what symbol)
             ;; (cram-robot-interfaces:robot ?from-what)
             (lisp-fun cram-tf:robot-current-pose ?pose))
        (and (lisp-type ?from-what cl-transforms:pose)
             (equal ?pose ?from-what)))
    (lisp-fun cl-transforms:origin ?pose ?point)
    (rob-int:robot ?robot-name)
    (costmap:costmap-in-reach-distance ?robot-name ?distance)
    (costmap:costmap-add-function
     reachable-from-space
     (costmap:make-range-cost-function ?point ?distance)
     ?cm)
    (costmap:costmap-add-function
     reachable-from-weighted
     (costmap:make-location-cost-function ?pose ?distance)
     ?cm)))
