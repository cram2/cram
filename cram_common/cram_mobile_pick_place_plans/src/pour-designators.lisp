;;;
;;; Copyright (c) 2016, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :pp-plans)

(def-fact-group pou-designator (desig:action-grounding)

  (<- (desig:action-grounding ?action-designator (pour ?resolved-designator))
    (spec:property ?action-designator (:type :pouring))
    (spec:property ?action-designator (:on-object ?_))
    ;;pec:property ?action-designator (:object ?_)))

  (spec:property ?action-designator (:arm ?_))
  ;; (once (or (spec:property ?action-designator (:arm ?_))
  ;;           (spec:property ?action-designator (:object ?_))))
  (once (or (spec:property ?action-designator (:wait-duration ?_))
            (true)))
  (-> (spec:property ?action-designator (:sides ?sides))
      (eqaul ?resolved-designator ?action-designator)
      (and (desig:desig-description ?action-designator ?description)
           (append ?description ((:sides (:top-left :top-right :top-front)))
                   ?new-description)
           (desig:designator :action ?new-description ?resolved-designator))))




  (<- (desig:action-grounding ?action-designator (pour-without-retries
                                                  ?resolved-action-designator))
    (spec:property ?action-designator (:type :pouring-without-retries))
    
    (-> (spec:property ?action-designator (:configuration ?side))
        (true)
        (format "WARNINGL: please specify a side"))
    (-> (spec:property ?action-designator (:arm ?arm))
        (true)
        (format "WARNINGL: please specify an arm"))
    ;; ;; source
    ;; (-> (spec:property ?action-designator (:arm ?arm))
    ;;     (-> (spec:property ?action-designator (:object
    ;;                                            ?source-designator))
    ;;         (once (or (cpoe:object-in-hand ?source-designator ?arm ?grasp)
    ;;                   (format "WARNING: Wanted to pour from object ~a ~
    ;;                            with arm ~a, but it's not in the arm.~%"
    ;;                           ?source-designator ?arm)))
    ;;         (cpoe:object-in-hand ?source-designator ?arm ?grasp))
    ;;     (-> (spec:property ?action-designator (:object
    ;;                                            ?source-designator))
    ;;         (once (or (cpoe:object-in-hand ?source-designator ?arm ?grasp)
    ;;                   (format "WARNING: Wanted to pour from object ~a ~
    ;;                            but it's not in any of the hands.~%"
    ;;                           ?source-designator)))
    ;;         (cpoe:object-in-hand ?source-designator ?arm ?grasp)))
    ;; (desig:current-designator ?source-designator ?current-source-designator)
    ;;destination / target
    (spec:property ?action-designator (:on-object ?target-designator))
    (desig:current-designator ?target-designator ?current-target-designator)
    ;; angle
    (-> (spec:property ?action-designator (:tilt-angle ?tilt-angle))
        (true)
        (lisp-fun man-int:get-tilt-angle-for-pouring ?source-type ?target-type
                  ?tilt-angle))
    ;; cartesian pouring trajectory
    (equal ?objects-acted-on (;; ?current-source-designator
                              ?current-target-designator))
    (-> (equal ?arm :left)
        (and (lisp-fun man-int:get-action-trajectory :pouring
                       ?arm ?side nil ?objects-acted-on
                       :tilt-angle ?tilt-angle :side ?side
                       ?left-trajectory)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :reaching
                       ?left-reach-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :tilting
                       ?left-tilt-poses)
             ;; (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :retracting
             ;;           ?left-retract-poses)
             )
        
        (and (equal ?left-reach-poses NIL)
             (equal ?left-tilt-poses NIL)))
    
    (-> (equal ?arm :right)
        (and (lisp-fun man-int:get-action-trajectory :pouring
                       ?arm ?side nil ?objects-acted-on
                       :tilt-angle ?tilt-angle :side ?side
                       ?right-trajectory)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :reaching
                       ?right-reach-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :tilting
                       ?right-tilt-poses)
             ;; (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :retracting
             ;;           ?right-retract-poses)
             )
        ;; (and (equal ?right-reach-poses t)
        ;;      (equal ?right-tilt-down-poses t)
        ;;      (equal ?right-tilt-up-poses t)
        ;;      (equal ?right-tilt-second-poses t)
        ;;      (equal ?right-tilt-third-poses t)
        ;;      (equal ?right-retract-poses t))
        
        (and (equal ?right-reach-poses NIL)
             (equal ?right-tilt-poses NIL)))
    
    
    ;; wait duration
    ;; (-> (spec:property ?action-designator (:wait-duration ?wait-duration))
    ;;     (true)
    ;;     (lisp-fun man-int:get-wait-duration-for-pouring
    ;;               ?source-type ?taget-type ?tilt-angle
    ;;               ?wait-duration))
    ;; look pose
    ;; (-> (lisp-pred identity ?left-trajectory)
    ;;     (equal ?left-tilt-down-poses (?look-pose . ?_))
    ;;     (equal ?right-tilt-down-poses (?look-pose . ?_)))
    ;; ;; ;; should look or not?
    ;; (rob-int:robot ?robot)
    ;; (-> (man-int:robot-arm-is-also-a-neck ?robot ?arm)
    ;;     (equal ?robot-arm-is-also-a-neck T)
    ;;     (equal ?robot-arm-is-also-a-neck NIL))

    
    ;;put together resulting designator
    (desig:designator :action ((:type :pouring)
                               ;;(:object ?current-source-designator)
                               (:arm ?arm)
                               (:configuration ?side)
                               ;;(:grasp ?grasp)
                               (:on-object ?current-target-designator)
                               ;; ;; (:other-object-is-a-robot ?other-object-is-a-robot)
                               ;;(:look-pose ?look-pose)
                               ;(:robot-arm-is-also-a-neck ?robot-arm-is-also-a-neck)
                               ;;(:wait-duration ?wait-duration)
                               
                               (:left-reach-poses ?left-reach-poses)
                               (:left-tilt-poses ?left-tilt-poses)
                               
                               (:right-reach-poses ?right-reach-poses)
                               (:right-tilt-poses ?right-tilt-poses)
                               )
                      
                      ?resolved-action-designator)))


  ;; (<- (action-grounding ?action-designator (move-arms-in-sequence
  ;;                                           ?resolved-action-designator))
  ;;   (spec:property ?action-designator (:type :tilting))
  ;;   (once (or (spec:property ?action-designator (:left-poses ?left-poses))
  ;;             (equal ?left-poses nil)))
  ;;   (once (or (spec:property ?action-designator (:right-poses ?right-poses))
  ;;             (equal ?right-poses nil)))
  ;;   (once (or (spec:property ?action-designator (:collision-mode ?collision))
  ;;             (equal ?collision :avoid-all)))
  ;;   (desig:designator :action ((:type ?action-type)
  ;;                              (:left-poses ?left-poses)
  ;;                              (:right-poses ?right-poses)
  ;;                              (:collision-mode ?collision)
  ;;                              (:move-base nil)
  ;;                              (:align-planes-left nil)
  ;;                              (:align-planes-right nil))
  ;;                     ?resolved-action-designator)))
