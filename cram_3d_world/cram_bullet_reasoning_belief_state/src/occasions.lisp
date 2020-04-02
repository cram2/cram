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

(in-package :cram-bullet-reasoning-belief-state)

(def-fact-group occasions (cpoe:object-in-hand cpoe:object-picked cpoe:object-placed-at cpoe:loc cpoe:torso-at cpoe:arms-positioned cpoe:ees-at cpoe:looking-at)
  (<- (cpoe:object-in-hand ?object ?side ?grasp)
    (btr:bullet-world ?world)
    (cram-robot-interfaces:robot ?robot)
    (btr:attached ?world ?robot ?link ?object-name ?grasp)
    (once
     (and (object-designator-name ?object ?object-name)
          (desig:obj-desig? ?object)))
    (cram-robot-interfaces:end-effector-link ?robot ?side ?link))

  (<- (cpoe:object-in-hand ?object ?side)
    (cpoe:object-in-hand ?object ?side ?_))

  (<- (cpoe:object-in-hand ?object)
    (setof ?object (cpoe:object-in-hand ?object ?_) ?objects)
    (member ?object ?objects))

  (<- (cpoe:object-picked ?object)
    (cpoe:object-in-hand ?object))

  (<- (cpoe:object-placed-at ?object ?location)
    (cpoe:loc ?object ?location))

  (<- (cpoe:loc ?robot ?location)
    (cram-robot-interfaces:robot ?robot)
    (object-at-location ?_ ?robot ?location))

  (<- (cpoe:loc ?object ?location)
    (desig:obj-desig? ?object)
    (object-designator-name ?object ?object-name)
    (object-at-location ?_ ?object-name ?location))

  ;; This goal check is defined in the cram_urdf_environment_manipulation package
  ;; (<- (cpoe:container-state ?container-designator ?distance)
  ;;   ...)

  (<- (cpoe:torso-at ?joint-state)
    (cpoe:torso-at ?joint-state 0.01))

  (<- (cpoe:torso-at ?joint-state ?delta)
    (lisp-pred typep ?joint-state keyword)
    (rob-int:robot ?robot)
    (rob-int:robot-torso-link-joint ?robot ?_ ?joint)
    (rob-int:joint-lower-limit ?robot ?joint ?lower-limit)
    (rob-int:joint-upper-limit ?robot ?joint ?upper-limit)
    (-> (equal ?joint-state :upper-limit)
        (cpoe:torso-at ?upper-limit ?delta)
        (-> (equal ?joint-state :lower-limit)
            (cpoe:torso-at ?lower-limit ?delta)
            (-> (equal ?joint-state :middle)
                (and
                 (lisp-fun - ?upper-limit ?lower-limit ?middle-diff)
                 (lisp-fun / ?middle-diff 2 ?middle-half-diff)
                 (lisp-fun + ?lower-limit ?middle-half-diff ?middle)
                 (cpoe:torso-at ?middle ?delta))
                (fail)))))

  (<- (cpoe:torso-at ?joint-state ?delta)
    (lisp-pred typep ?joint-state number)
    (btr:bullet-world ?world)
    (rob-int:robot ?robot)
    (symbol-value cram-tf:*robot-torso-joint* ?torso-joint)
    (btr:joint-state ?world ?robot ?torso-joint ?torso-joint-state)
    (lisp-fun - ?torso-joint-state ?delta ?lower)
    (lisp-fun + ?torso-joint-state ?delta ?upper)
    (< ?lower ?joint-state)
    (> ?upper ?joint-state))

  (<- (cpoe:arms-positioned ?left-configuration ?right-configuration)
    (cpoe:arms-positioned ?left-configuration ?right-configuration 0.01))

  (<- (cpoe:arms-positioned ?left-configuration ?right-configuration ?delta)
    (rob-int:robot ?robot)
    (-> (lisp-pred identity ?left-configuration)
        (and (man-int:configuration-joint-states :left ?left-configuration ?left-goal-states)
             (lisp-pred joint-states-converged ?left-goal-states ?delta))
        (true))
    (-> (lisp-pred identity ?right-configuration)
        (and (man-int:configuration-joint-states :right ?right-configuration ?right-goal-states)
             (lisp-pred joint-states-converged ?right-goal-states ?delta))
        (true)))

  (<- (cpoe:ees-at ?left-poses ?right-poses)
    (cpoe:ees-at ?left-poses ?right-poses 0.03 0.09)) ;; about 5 degrees for delta-rot

  (<- (cpoe:ees-at ?left-poses ?right-poses ?delta-pos ?delta-rot)
    (-> (lisp-pred typep ?left-poses list)
        (and
         (lisp-fun last ?poses ?left-pose-list)
         (lisp-fun car ?left-pose-list ?left-pose))
        (equal ?left-poses (?left-pose)))
    (-> (lisp-pred typep ?right-poses list)
        (and
         (lisp-fun last ?poses ?right-pose-list)
         (lisp-fun car ?right-pose-list ?right-pose))
        (equal ?right-poses (?right-pose)))
    (cpoe:ees-at ?left-pose ?right-pose ?delta-pos ?delta-rot))

  (<- (cpoe:ees-at ?left-pose ?right-pose ?delta-pos ?delta-rot)
    (not (lisp-pred typep ?left-pose 'cl-tf:pose-stamped))
    (not (lisp-pred typep ?right-pose 'cl-tf:pose-stamped))
    (lisp-pred typep ?left-pose 'cl-tf:pose)
    (lisp-pred typep ?right-pose 'cl-tf:pose)
    ;; We have to assume the pose is in the fixed-frame
    (symbol-value cram-tf:*fixed-frame* ?fixed-frame)
    (lisp-fun cl-tf:pose->pose-stamped ?fixed-frame 0.0 ?left-pose-stamped)
    (lisp-fun cl-tf:pose->pose-stamped ?fixed-frame 0.0 ?right-pose-stamped)
    (cpoe:ees-at ?left-pose-stamped ?right-pose-stamped ?delta-pos ?delta-rot))

  (<- (cpoe:ees-at ?left-pose-stamped ?right-pose-stamped ?delta-pos ?delta-rot)
    (lisp-pred typep ?left-pose-stamped 'cl-tf:pose-stamped)
    (lisp-pred typep ?right-pose-stamped 'cl-tf:pose-stamped)
    (rob-int:robot ?robot)
    (rob-int:robot-tool-frame ?robot :left ?left-tool-frame)
    (rob-int:robot-tool-frame ?robot :right ?right-tool-frame)
    (symbol-value cram-tf:*transformer* ?transformer)
    (symbol-value cram-tf:*fixed-frame* ?fixed-frame)
    (lisp-fun cl-tf:lookup-transform ?transformer ?fixed-frame ?left-tool-frame
              ?left-tool-transform-stamped)
    (lisp-fun cl-tf:lookup-transform ?transformer ?fixed-frame ?right-tool-frame
              ?right-tool-transform-stamped)
    (lisp-fun cl-tf:stamp ?left-tool-transfrom-stamped ?left-stamp)
    (lisp-fun cl-tf:stamp ?right-tool-transfrom-stamped ?right-stamp)
    (lisp-fun cram-tf:transform->pose-stamped ?fixed-frame ?left-stamp ?left-tool-transform-stamped
              ?left-tool-pose-stamped)
    (lisp-fun cram-tf:transform->pose-stamped ?fixed-frame ?right-stamp ?right-tool-transform-stamped
              ?right-tool-pose-stamped)
    (lisp-pred pose-stampeds-similar? ?left-pose-stamped ?left-tool-pose-stamped ?delta-pos ?delta-rot)
    (lisp-pred pose-stampeds-similar? ?right-pose-stamped ?right-tool-pose-stamped ?delta-pos ?delta-rot))

  (<- (cpoe:looking-at ?target)
    (cpoe:looking-at ?target 0.1))

  (<- (cpoe:looking-at ?object ?delta)
    (desig:obj-desig? ?object)
    (desig:current-designator ?object ?current-object-desig)
    (lisp-fun man-int:get-object-pose ?current-object-desig ?object-pose)
    (cpoe:looking-at ?object-pose ?delta))

  (<- (cpoe:looking-at ?location-designator ?delta)
    (desig:loc-desig? ?location-designator)
    (desig:current-designator ?location-designator ?current-location-designator)
    (desig:designator-groundings ?current-location-designator ?poses)
    (member ?pose ?poses)
    (cpoe:looking-at ?pose ?delta))

  (<- (cpoe:looking-at ?frame ?delta)
    (lisp-pred typep ?frame string)
    (symbol-value cram-tf:*transformer* ?transformer)
    (symbol-value cram-tf:*fixed-frame* ?fixed-frame)

    (-> (lisp-pred cl-tf:wait-for-transform ?transformer
                   :source-frame ?frame
                   :target-frame ?fixed-frame
                   :timeout 0.1)
        (and
         (lisp-fun lookup-transform-handled ?transformer ?fixed-frame ?frame
                   ?frame-transform)
         (lisp-fun cl-tf:stamp ?frame-transform ?stamp)
         (lisp-fun cram-tf:transform->pose-stamped ?fixed-frame ?stamp ?frame-transform
                   ?frame-pose-stamped)
         (cpoe:looking-at ?frame-pose-stamped ?delta))
        (fail)))

  (<- (cpoe:looking-at ?direction ?delta)
    (lisp-pred typep ?direction cl-tf:3d-vector)
    (rob-int:robot ?robot)
    (rob-int:camera-frame ?robot ?camera-frame)
    (rob-int:camera-horizontal-angle ?robot ?camera-angle-h)
    (rob-int:camera-vertical-angle ?robot ?camera-angle-v)
    (lisp-pred looking-in-direction ?camera-frame ?camera-angle-h ?camera-angle-v ?direction))

  (<- (cpoe:looking-at ?pose ?delta)
    (not (lisp-pred typep ?pose cl-tf:pose-stamped))
    (lisp-pred typep ?pose cl-tf:pose)
    (symbol-value cram-tf:*fixed-frame* ?fixed-frame)
    (lisp-fun cl-tf:pose->pose-stamped ?fixed-frame 0.0 ?pose ?pose-stamped)
    (cpoe:looking-at ?pose-stamped ?delta))

  (<- (cpoe:looking-at ?pose-stamped ?delta)
    (lisp-pred typep ?pose-stamped cl-tf:pose-stamped)
    (rob-int:robot ?robot)
    (rob-int:camera-frame ?robot ?camera-frame)
    (-> (lisp-pred looking-at-pose-p ?camera-frame ?pose-stamped)
        (true)
        (and (symbol-value cram-tf:*transformer* ?transformer)
             (symbol-value cram-tf:*fixed-frame* ?fixed-frame)
             (-> (lisp-pred cl-tf:wait-for-transform ?transformer
                            :source-frame ?camera-frame
                            :target-frame ?fixed-frame
                            :timeout 0.1)
                 (and
                  (lisp-fun cl-tf:lookup-transform ?transformer ?fixed-frame ?camera-frame ?camera-transform)
                  (lisp-fun cl-tf:translation ?camera-transform ?camera-point)
                  (lisp-fun cl-tf:origin ?pose-stamped ?target-point)
                  (lisp-fun cl-tf:v- ?target-point ?camera-point ?direction)
                  (rob-int:camera-horizontal-angle ?robot ?camera-angle-h)
                  (rob-int:camera-vertical-angle ?robot ?camera-angle-v)
                  (lisp-pred looking-in-direction ?camera-frame ?camera-angle-h ?camera-angle-v
                             ?direction))
                 (fail))))))

;; put this into cram-tf?
(defun pose-stampeds-similar? (pose other-pose delta-pos delta-rot)
  (let ((pos (cl-tf:origin pose))
        (other-pos (cl-tf:origin other-pose)))
    (and (< (cl-tf:v-norm
             (cl-tf:v- pos other-pos))
            delta-pos)
         (cram-tf:values-converged
          (cl-tf:quaternion->euler (cl-tf:orientation pose) :just-values T)
          (cl-tf:quaternion->euler (cl-tf:orientation other-pose) :just-values T)
          delta-rot))))

(defun joint-states-converged (goal-states delta)
  (let ((arm-joints (loop for (name value) in goal-states collect name))
        (goal-values (loop for (name value) in goal-states collect value)))
    (cram-tf:values-converged
     (mapcar (alexandria:curry 'btr:joint-state (btr:get-robot-object))
             arm-joints)
     goal-values
     delta)))

(defun looking-at-pose-p (camera-frame pose)
  (unwind-protect
       (handler-case
           (let* ((sphere (btr-utils:spawn-object :vis-obj :cup :pose pose))
                  (camera-transform
                    (cl-tf:lookup-transform cram-tf:*transformer* cram-tf:*fixed-frame* camera-frame))
                  (camera-pose
                    (cram-tf:transform->pose-stamped cram-tf:*fixed-frame*
                                                     (cl-tf:stamp camera-transform)
                                                     camera-transform))
                  (result (btr:looking-at-object-p btr:*current-bullet-world* camera-pose sphere)))
             result)
         (cl-transforms-stamped:timeout-error ()
           nil))
    (btr-utils:kill-object :vis-obj)))

(def-fact-group occasion-utilities (object-designator-name desig:desig-location-prop)
  (<- (object-designator-name ?name ?name)
    (lisp-type ?name symbol))

  (<- (object-designator-name ?object-designator ?object-name)
    (or (and (bound ?object-designator)
             (desig:obj-desig? ?object-designator))
        (and (not (bound ?object-designator))
             (lisp-fun unique-object-designators ?object-designators)
             (member ?one-object-designator-from-chain ?object-designators)
             (desig:current-designator ?one-object-designator-from-chain ?object-designator)))
    (lisp-fun get-designator-object-name ?object-designator ?belief-name)
    (-> (lisp-pred identity ?belief-name)
        (equal ?object-name ?belief-name)
        (and (desig:desig-prop ?object-designator (:type ?object-type))
             (btr:bullet-world ?world)
             (btr:item-type ?world ?object-name ?object-type))))

  (<- (desig:desig-location-prop ?designator ?location)
    (desig:obj-desig? ?designator)
    (desig:desig-prop ?designator (:type ?type))
    (not (desig:desig-prop ?designator (:name ?name)))
    (not (desig:desig-prop ?designator (:pose ?pose)))
    (btr:bullet-world ?world)
    (btr:item-type ?world ?name ?type)
    (btr:pose ?world ?name ?location))

  (<- (desig:desig-location-prop ?desig ?loc)
    (desig:loc-desig? ?desig)
    (desig:desig-prop ?desig (:object ?o))
    (btr:object ?_ ?o)
    (btr:pose ?_ ?o ?loc))

  (<- (desig:desig-location-prop ?o ?loc)
    (btr:object ?_ ?o)
    (btr:pose ?_ ?o ?loc))

  (<- (object-at-location ?world ?object-name ?location-designator)
    (lisp-type ?location-designator desig:location-designator)
    (btr:bullet-world ?world)
    (lisp-fun desig:current-desig ?location-designator ?current-location)
    (lisp-pred identity ?current-location)
    (desig:designator-groundings ?current-location ?_)
    (or (btr:object-pose ?world ?object-name ?object-pose)
        (btr:object-bottom-pose ?world ?object-name ?object-pose))
    (lisp-pred desig:validate-location-designator-solution ?current-location ?object-pose))

  (<- (object-at-location ?world ?object-name ?location-designator)
    (not (bound ?location-designator))
    (btr:bullet-world ?world)
    (btr:object-pose ?world ?object-name ?object-pose)
    (symbol-value cram-tf:*fixed-frame* ?fixed-frame)
    (lisp-fun cl-transforms-stamped:pose->pose-stamped ?fixed-frame 0.0 ?object-pose
              ?object-pose-stamped)
    (desig:designator :location ((:pose ?object-pose-stamped))
                      ?location-designator))

  (<- (looking-in-direction ?direction ?delta)
    (rob-int:robot ?robot)
    (rob-int:camera-horizontal-angle ?robot ?cam-angle-h)
    (rob-int:camera-vertical-angle ?robot ?cam-angle-v)
    (rob-int:camera-frame ?robot ?cam-frame)
    (lisp-pred looking-in-direction ?cam-frame ?cam-angle-h ?cam-angle-v ?direction ?delta)))

(defun looking-in-direction (camera-frame angle-horizontal angle-vertical direction)
  (handler-case
      (let* ((camera-transform
               (cl-tf:lookup-transform cram-tf:*transformer* cram-tf:*fixed-frame* camera-frame))
             (camera-up-in-cam
               (cl-tf:make-transform-stamped camera-frame "up" 0.0
                                             (cl-tf:make-3d-vector 0 1 0)
                                             (cl-tf:make-identity-rotation)))
             (camera-up-in-map
               (cram-tf:multiply-transform-stampeds cram-tf:*fixed-frame* "up"
                                                    camera-transform
                                                    camera-up-in-cam))
             (camera-up-in-map-vector (cl-tf:v- (cl-tf:translation camera-up-in-map)
                                                (cl-tf:translation camera-transform)))
             (camera-left-in-cam
               (cl-tf:make-transform-stamped camera-frame "left" 0.0
                                             (cl-tf:make-3d-vector 1 0 0)
                                             (cl-tf:make-identity-rotation)))
             (camera-left-in-map
               (cram-tf:multiply-transform-stampeds cram-tf:*fixed-frame* "left"
                                                    camera-transform
                                                    camera-left-in-cam))
             (camera-left-in-map-vector (cl-tf:v- (cl-tf:translation camera-left-in-map)
                                                  (cl-tf:translation camera-transform)))
             (angle-h (asin (/ (cl-tf:dot-product camera-left-in-map-vector direction)
                               (cl-tf:v-norm direction))))
             (angle-v (asin (/ (cl-tf:dot-product camera-up-in-map-vector direction)
                               (cl-tf:v-norm direction))))
             (max-angle-h (/ angle-horizontal 2))
             (max-angle-v (/ angle-vertical 2)))
        (and (< (abs angle-h) max-angle-h)
             (< (abs angle-v) max-angle-v)))
    (cl-transforms-stamped:timeout-error ()
      nil)))

(defun unique-object-designators ()
  "Returns all designators. For equated designators, only one instance
is returned."
  (remove-duplicates
   (remove-if-not (lambda (designator)
                    (and
                     (typep designator 'desig:object-designator)))
                  (reverse (desig:get-all-designators)))
   :test #'desig:desig-equal))
