;;;
;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
;;;               2020, Christopher Pollok <cpollok@uni-bremen.de>
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

(defparameter *torso-convergence-delta* 0.01 "In meters")
(defparameter *arm-joints-convergence-delta* 0.0174 "In radians, about 1 deg.")
(defparameter *ee-position-convergence-delta* 0.02 "In meters")
(defparameter *ee-rotation-convergence-delta* 0.07 "In radians, about 4 deg.")
(defparameter *looking-convergence-delta* 0.01 "In meters")
(defparameter *looking-convergence-joints-delta* 0.07 "In radians, about 4 deg.")

(def-fact-group occasions (cpoe:object-in-hand
                           cpoe:object-at-location
                           cpoe:robot-at-location
                           cpoe:torso-at
                           cpoe:arms-positioned-at
                           cpoe:tool-frames-at
                           cpoe:looking-at)

  ;; This occasion is defined in cram_urdf_environment_manipulation
  ;; (<- (cpoe:container-state ?container-designator ?distance) ...)

  ;; if we want the arm, we get it from the link
  (<- (cpoe:object-in-hand ?object ?arm ?grasp)
    (btr:bullet-world ?world)
    (rob-int:robot ?robot)
    (btr:attached ?world ?robot ?link ?object-name ?grasp)
    (once (and (object-designator-name ?object ?object-name)
               (desig:obj-desig? ?object)))
    (-> (bound ?arm)
        (rob-int:end-effector-link ?robot ?arm ?link)
        (once (or (rob-int:end-effector-link ?robot ?arm ?link)
                  (true)))))
  ;;
  ;; if we only want to know the link and don't care about the arm
  ;; it can be that the arm is not even given in the attachments
  ;; so we need a bit of copy paste here...
  (<- (cpoe:object-in-hand ?object ?_ ?grasp ?link)
    (btr:bullet-world ?world)
    (rob-int:robot ?robot)
    (btr:attached ?world ?robot ?link ?object-name ?grasp)
    (once (and (object-designator-name ?object ?object-name)
               (desig:obj-desig? ?object))))
  ;;
  (<- (cpoe:object-in-hand ?object ?arm)
    (cpoe:object-in-hand ?object ?arm ?_))
  ;;
  (<- (cpoe:object-in-hand ?object)
    (setof ?object (cpoe:object-in-hand ?object ?_) ?objects)
    (member ?object ?objects))


  (<- (cpoe:robot-at-location ?robot ?location)
    (rob-int:robot ?robot)
    (%object-at-location ?_ ?robot ?location))


  (<- (cpoe:object-at-location ?object ?location)
    (desig:obj-desig? ?object)
    (object-designator-name ?object ?object-name)
    (%object-at-location ?_ ?object-name ?location))


  (<- (cpoe:torso-at ?joint-state)
    (symbol-value *torso-convergence-delta* ?torso-delta)
    (cpoe:torso-at ?joint-state ?torso-delta))
  ;;
  (<- (cpoe:torso-at ?joint-state ?delta)
    (lisp-type ?joint-state keyword)
    (rob-int:robot ?robot)
    (rob-int:robot-torso-link-joint ?robot ?_ ?joint)
    (rob-int:joint-lower-limit ?robot ?joint ?lower-limit)
    (rob-int:joint-upper-limit ?robot ?joint ?upper-limit)
    (-> (equal ?joint-state :upper-limit)
        (cpoe:torso-at ?upper-limit ?delta)
        (-> (equal ?joint-state :lower-limit)
            (cpoe:torso-at ?lower-limit ?delta)
            (-> (equal ?joint-state :middle)
                (and (lisp-fun - ?upper-limit ?lower-limit ?middle-diff)
                     (lisp-fun / ?middle-diff 2 ?middle-half-diff)
                     (lisp-fun + ?lower-limit ?middle-half-diff ?middle)
                     (cpoe:torso-at ?middle ?delta))
                (fail)))))
  ;;
  (<- (cpoe:torso-at ?joint-state ?delta)
    (lisp-type ?joint-state number)
    (rob-int:robot ?robot)
    (rob-int:robot-torso-link-joint ?robot ?_ ?torso-joint)
    (btr:bullet-world ?world)
    (btr:joint-state ?world ?robot ?torso-joint ?torso-joint-state)
    (lisp-pred cram-tf:values-converged ?torso-joint-state ?joint-state ?delta))


  (<- (cpoe:arms-positioned-at ?left-configuration ?right-configuration)
    (symbol-value *arm-joints-convergence-delta* ?delta)
    (cpoe:arms-positioned-at ?left-configuration ?right-configuration ?delta))
  ;;
  (<- (cpoe:arms-positioned-at ?left-configuration ?right-configuration ?delta)
    (rob-int:robot ?robot)
    (-> (lisp-pred identity ?left-configuration)
        (and (rob-int:robot-joint-states ?robot :arm :left ?left-configuration
                                         ?left-goal-states)
             (lisp-pred btr:robot-converged-to-goal-joint-states
                        ?left-goal-states ?delta))
        (true))
    (-> (lisp-pred identity ?right-configuration)
        (and (rob-int:robot-joint-states ?robot :arm :right ?right-configuration
                                         ?right-goal-states)
             (lisp-pred btr:robot-converged-to-goal-joint-states
                        ?right-goal-states ?delta))
        (true)))
  ;; For checking other than arm configurations, e.g., neck configuration
  (<- (cpoe:arms-positioned-at ?effector-type ?effector-name
                               ?effector-configuration-name ?delta)
    (rob-int:robot ?robot)
    (rob-int:robot-joint-states ?robot ?effector-type ?effector-name
                                ?effector-configuration-name
                                ?effector-joint-states)
    (lisp-pred btr:robot-converged-to-goal-joint-states
               ?effector-joint-states ?delta))


  (<- (cpoe:tool-frames-at ?left-poses ?right-poses)
    (symbol-value *ee-position-convergence-delta* ?delta-position)
    (symbol-value *ee-rotation-convergence-delta* ?delta-rotation)
    (cpoe:tool-frames-at ?left-poses ?right-poses ?delta-position ?delta-rotation))
  ;;
  (<- (cpoe:tool-frames-at ?left-poses ?right-poses ?delta-pos ?delta-rot)
    (or (and (lisp-pred identity ?left-poses)
             (lisp-type ?left-poses list))
        (and (lisp-pred identity ?right-poses)
             (lisp-type ?right-poses list)))
    (-> (and (lisp-pred identity ?left-poses)
             (lisp-type ?left-poses list))
        (and (lisp-fun last ?left-poses ?left-pose-list)
             (lisp-fun car ?left-pose-list ?left-pose))
        (equal ?left-poses ?left-pose))
    (-> (and (lisp-pred identity ?right-poses)
             (lisp-type ?right-poses list))
        (and (lisp-fun last ?right-poses ?right-pose-list)
             (lisp-fun car ?right-pose-list ?right-pose))
        (equal ?right-poses ?right-pose))
    (cpoe:tool-frames-at ?left-pose ?right-pose ?delta-pos ?delta-rot))
  ;;
  (<- (cpoe:tool-frames-at ?left-pose ?right-pose ?delta-pos ?delta-rot)
    (not (lisp-type ?left-pose cl-transforms-stamped:pose-stamped))
    (not (lisp-type ?right-pose cl-transforms-stamped:pose-stamped))
    (or (lisp-type ?left-pose cl-transforms:pose)
        (lisp-type ?right-pose cl-transforms:pose))
    ;; If the pose doesn't have a frame, we have to assume that it's in fixed frame
    (symbol-value cram-tf:*fixed-frame* ?fixed-frame)
    (-> (lisp-type ?left-pose cl-transforms:pose)
        (lisp-fun cl-transforms-stamped:pose->pose-stamped ?fixed-frame 0.0
                  ?left-pose ?left-pose-stamped)
        (equal ?left-pose ?left-pose-stamped))
    (-> (lisp-type ?right-pose cl-transforms:pose)
        (lisp-fun cl-transforms-stamped:pose->pose-stamped ?fixed-frame 0.0
                  ?right-pose ?right-pose-stamped)
        (equal ?right-pose ?right-pose-stamped))
    (cpoe:tool-frames-at ?left-pose-stamped ?right-pose-stamped
                         ?delta-pos ?delta-rot))
  ;;
  (<- (cpoe:tool-frames-at ?left-pose-stamped ?right-pose-stamped
                           ?delta-pos ?delta-rot)
    (or (lisp-type ?left-pose-stamped cl-transforms-stamped:pose-stamped)
        (lisp-type ?right-pose-stamped cl-transforms-stamped:pose-stamped))
    (rob-int:robot ?robot)
    (btr:bullet-world ?world)
    (symbol-value cram-tf:*fixed-frame* ?fixed-frame)
    (-> (lisp-type ?left-pose-stamped cl-transforms-stamped:pose-stamped)
        (and (rob-int:robot-tool-frame ?robot :left ?left-tool-frame)
             (btr:link-pose ?world ?robot ?left-tool-frame ?left-tool-pose)
             (lisp-fun cl-transforms-stamped:pose->pose-stamped ?fixed-frame 0.0
                       ?left-tool-pose ?left-tool-pose-stamped)
             (lisp-pred cram-tf:pose-stampeds-converged
                        ?left-tool-pose-stamped ?left-pose-stamped
                        ?delta-pos ?delta-rot))
        (true))
    (-> (lisp-type ?right-pose-stamped cl-transforms-stamped:pose-stamped)
        (and (rob-int:robot-tool-frame ?robot :right ?right-tool-frame)
             (btr:link-pose ?world ?robot ?right-tool-frame ?right-tool-pose)
             (lisp-fun cl-transforms-stamped:pose->pose-stamped ?fixed-frame 0.0
                       ?right-tool-pose ?right-tool-pose-stamped)
             (lisp-pred cram-tf:pose-stampeds-converged
                        ?right-tool-pose-stamped ?right-pose-stamped
                        ?delta-pos ?delta-rot))
        (true)))


  (<- (cpoe:looking-at ?location-or-object-or-frame-or-direction-or-pose)
    (symbol-value *looking-convergence-delta* ?delta)
    (cpoe:looking-at ?location-or-object-or-frame-or-direction-or-pose ?delta))
  ;;
  (<- (cpoe:looking-at ?object-designator ?delta)
    (desig:obj-desig? ?object-designator)
    (desig:current-designator ?object-designator ?current-object-desig)
    (lisp-fun man-int:get-object-pose-in-map ?current-object-desig ?object-pose)
    (cpoe:looking-at ?object-pose ?delta))
  ;;
  (<- (cpoe:looking-at ?location-designator ?delta)
    (desig:loc-desig? ?location-designator)
    (desig:current-designator ?location-designator ?current-location-designator)
    (desig:designator-groundings ?current-location-designator ?poses)
    (member ?pose ?poses)
    (cpoe:looking-at ?pose ?delta))
  ;;
  (<- (cpoe:looking-at ?frame ?delta)
    (lisp-type ?frame string)
    (symbol-value cram-tf:*fixed-frame* ?fixed-frame)
    ;; Don't want to use the transformer in goal proving, so just assume TRUE
    ;; (symbol-value cram-tf:*transformer* ?transformer)
    ;; (-> (lisp-pred cl-tf:wait-for-transform ?transformer
    ;;                :source-frame ?frame
    ;;                :target-frame ?fixed-frame
    ;;                :timeout 0.1)
    ;;     (and (lisp-fun lookup-transform-handled ?transformer ?fixed-frame ?frame
    ;;                    ?frame-transform)
    ;;          (lisp-fun cl-transforms-stamped:stamp ?frame-transform ?stamp)
    ;;          (lisp-fun cram-tf:transform->pose-stamped ?fixed-frame ?stamp
    ;;                    ?frame-transform ?frame-pose-stamped)
    ;;          (cpoe:looking-at ?frame-pose-stamped ?delta))
    ;;     (fail))
    )
  ;;
  (<- (cpoe:looking-at ?direction ?delta)
    (lisp-type ?direction keyword)
    (rob-int:robot ?robot)
    ;; btr:looking-in-direction-p needs a vector, here the direction is a keyword
    (or (and (rob-int:robot-pose ?robot :neck ?_ ?direction ?pose-stamped)
             (cpoe:looking-at ?pose-stamped ?delta))
        (and (symbol-value *looking-convergence-joints-delta* ?joints-delta)
             (cpoe:arms-positioned-at :neck ?_ ?direction ?joints-delta))))
  ;;
  (<- (cpoe:looking-at ?pose ?delta)
    (not (lisp-type ?pose cl-transforms-stamped:pose-stamped))
    (lisp-type ?pose cl-transforms:pose)
    (symbol-value cram-tf:*fixed-frame* ?fixed-frame)
    (lisp-fun cl-transforms-stamped:pose->pose-stamped ?fixed-frame 0.0 ?pose
              ?pose-stamped)
    (cpoe:looking-at ?pose-stamped ?delta))
  ;;
  (<- (cpoe:looking-at ?pose-stamped-in-map ?delta)
    (lisp-type ?pose-stamped-in-map cl-transforms-stamped:pose-stamped)
    (lisp-fun cl-transforms-stamped:frame-id ?pose-stamped-in-map ?frame)
    (symbol-value cram-tf:*fixed-frame* ?fixed-frame)
    (rob-int:robot ?robot)
    (rob-int:camera-frame ?robot ?camera-frame)
    (btr:bullet-world ?world)
    (btr:link-pose ?world ?robot ?camera-frame ?camera-pose)
    (-> (lisp-pred string-equal ?frame ?fixed-frame)
        (or (lisp-pred btr:looking-at-pose-p ?world ?camera-pose ?pose-stamped-in-map)
            (and (rob-int:camera-horizontal-angle ?robot ?camera-angle-h)
                 (rob-int:camera-vertical-angle ?robot ?camera-angle-v)
                 (btr:%object ?world ?robot ?robot-object)
                 (lisp-fun cl-transforms:origin ?pose-stamped-in-map ?target-point)
                 (lisp-fun cl-transforms:origin ?camera-pose ?camera-point)
                 (lisp-fun cl-transforms:v- ?target-point ?camera-point ?direction)
                 (lisp-pred btr:looking-in-direction-p ?robot-object ?camera-frame
                            ?camera-angle-h ?camera-angle-v)))
        (true))
    ;; Don't want to use the transformer if the pose is not in map. Assume TRUE.
    ;; (and (symbol-value cram-tf:*transformer* ?transformer)
    ;;      (symbol-value cram-tf:*fixed-frame* ?fixed-frame)
    ;;      (-> (lisp-pred cl-tf:wait-for-transform ?transformer
    ;;                     :source-frame ?camera-frame
    ;;                     :target-frame ?fixed-frame
    ;;                     :timeout 0.1)
    ;;          (and
    ;;           (lisp-fun cl-tf:lookup-transform ?transformer ?fixed-frame
    ;;                     ?camera-frame ?camera-transform)
    ;;           (lisp-fun cl-tf:translation ?camera-transform ?camera-point)
    ;;           (lisp-fun cl-tf:origin ?pose-stamped ?target-point)
    ;;           (lisp-fun cl-tf:v- ?target-point ?camera-point ?direction)
    ;;           (rob-int:camera-horizontal-angle ?robot ?camera-angle-h)
    ;;           (rob-int:camera-vertical-angle ?robot ?camera-angle-v)
    ;;           (lisp-pred looking-in-direction ?camera-frame ?camera-angle-h
    ;;                      ?camera-angle-v ?direction))
    ;;          (fail)))
    ))



(def-fact-group occasion-utilities (object-designator-name
                                    desig:desig-location-prop)

  (<- (object-designator-name ?name ?name)
    (lisp-type ?name symbol))

  (<- (object-designator-name ?object-designator ?object-name)
    (or (and (bound ?object-designator)
             (desig:obj-desig? ?object-designator)
             (desig:current-designator ?object-designator
                                       ?current-object-designator)
             (desig:desig-prop ?current-object-designator (:name ?object-name)))
        (and (not (bound ?object-designator))
             ;; all object designators who have the same name should be
             ;; perceptions of the same exact object, and, thus,
             ;; they should be equated into one chain
             (lisp-fun unique-object-designators ?object-designators)
             (member ?one-desig-from-chain ?object-designators)
             (desig:current-designator ?one-desig-from-chain ?object-designator)
             (desig:desig-prop ?object-designator (:name ?object-name)))))

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

  (<- (%object-at-location ?world ?object-name ?location-designator)
    (lisp-type ?location-designator desig:location-designator)
    (btr:bullet-world ?world)
    (lisp-fun desig:current-desig ?location-designator ?current-location)
    (lisp-pred identity ?current-location)
    (desig:designator-groundings ?current-location ?_)
    (or (btr:object-pose ?world ?object-name ?object-pose)
        (btr:object-bottom-pose ?world ?object-name ?object-pose))
    (lisp-pred desig:validate-location-designator-solution ?current-location ?object-pose))

  (<- (%object-at-location ?world ?object-name ?location-designator)
    (not (bound ?location-designator))
    (btr:bullet-world ?world)
    (btr:object-pose ?world ?object-name ?object-pose)
    (symbol-value cram-tf:*fixed-frame* ?fixed-frame)
    (lisp-fun cl-transforms-stamped:pose->pose-stamped ?fixed-frame 0.0 ?object-pose
              ?object-pose-stamped)
    (desig:designator :location ((:pose ?object-pose-stamped))
                      ?location-designator)))

(defun unique-object-designators ()
  "Returns all designators. For equated designators, only one instance
is returned."
  (remove-duplicates
   (remove-if-not (lambda (designator)
                    (and
                     (typep designator 'desig:object-designator)))
                  (reverse (desig:get-all-designators)))
   :test #'desig:desig-equal))
