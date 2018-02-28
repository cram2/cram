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

(def-fact-group environment-manipulation (desig:action-grounding)
  
  (<- (desig:action-grounding ?action-designator (open-container ;; ?container-name
                                                                 ?arm
                                                                 ?gripper-opening
                                                                 ?left-reach-poses ?right-reach-poses
                                                                 ?left-lift-poses ?right-lift-poses
                                                                 ?joint-name ?environment-obj))
    (spec:property ?action-designator (:type :opening))
    (spec:property ?action-designator (:object ?container-designator))
    (spec:property ?container-designator (:type :container))
    (spec:property ?container-designator (:name ?container-name))
    (spec:property ?container-designator (:part-of ?environment))
    (-> (spec:property ?action-designator (:arm ?arm))
        (true)
        (and (cram-robot-interfaces:robot ?robot)
             (cram-robot-interfaces:arm ?robot ?arm)))
    ;; infer joint information
    ;; joint-name
    (lisp-fun get-container-link ?container-name ?container-link)
    (lisp-fun get-connecting-joint ?container-link ?connecting-joint)
    (lisp-fun cl-urdf:name ?connecting-joint ?joint-name)
    ;; environment
    (btr:bullet-world ?world)
    (lisp-fun btr:object ?world ?environment ?environment-obj)
    ;; infer missing information like ?gripper-opening, opening trajectory
    (lisp-fun obj-int:get-object-type-gripper-opening ?container-type ?gripper-opening)
    (lisp-fun obj-int:get-object-transform ?container-designator ?container-transform)
    (lisp-fun obj-int:get-object-grasping-poses ?container-name :container :left :front ?container-transform ?left-poses)
    (lisp-fun obj-int:get-object-grasping-poses ?container-name :container :right :front ?container-transform ?right-poses)
    (lisp-fun cram-mobile-pick-place-plans::extract-pick-up-manipulation-poses ?arm ?left-poses ?right-poses
              (?left-reach-poses ?right-reach-poses ?left-lift-poses ?right-lift-poses))
    )

  (<- (desig:action-grounding ?action-designator (drive-to-and-open-container ?container-designator))
    (spec:property ?action-designator (:type :driving-and-opening))
    (spec:property ?action-designator (:object ?container-designator))
    (spec:property ?container-designator (:type :container))
    )

  
  )
