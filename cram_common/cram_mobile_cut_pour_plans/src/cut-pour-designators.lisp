;;;
;;; Copyright (c) 2019, Vanessa Hassouna <kazhoyan@cs.uni-bremen.de>
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

(in-package :cp-plans)

(def-fact-group cut-and-pour-plans (desig:action-grounding)
    

   (<- (desig:action-grounding ?action-designator (pour ?resolved-action-designator))
    (spec:property ?action-designator (:type :pouring))

    ;; extract info from ?action-designator
    (spec:property ?action-designator (:object ?object-designator))
    (desig:current-designator ?object-designator ?current-object-desig)
    (spec:property ?current-object-desig (:type ?object-type))
    (spec:property ?current-object-desig (:name ?object-name))
    (-> (spec:property ?action-designator (:arm ?arm))
        (true)
        (man-int:robot-free-hand ?_ ?arm))
     (lisp-fun man-int:get-object-transform ?current-object-desig ?object-transform)

   
    ;; infer missing information like ?grasp type, gripping ?maximum-effort, manipulation poses
    (lisp-fun man-int:calculate-object-faces ?object-transform (?facing-robot-face ?bottom-face))
    (-> (man-int:object-rotationally-symmetric ?object-type)
        (equal ?rotationally-symmetric t)
        (equal ?rotationally-symmetric nil))
    (-> (spec:property ?action-designator (:grasp ?grasp))
        (true)
        (and (lisp-fun man-int:get-action-grasps ?object-type ?arm ?object-transform ?grasps)
             (member ?grasp ?grasps)))
    (lisp-fun man-int:get-action-gripping-effort ?object-type ?effort)
    (lisp-fun man-int:get-action-gripper-opening ?object-type ?gripper-opening)

    ;; calculate trajectory
    (equal ?objects (?current-object-desig))
    (-> (equal ?arm :left)
        (and (lisp-fun man-int:get-action-trajectory :pouring ?arm ?grasp ?objects 
                       ?left-pouring-pose)
             (lisp-fun man-int:get-traj-poses-by-label ?left-pouring-pose :approach
                       ?left-approach-poses)
             (lisp-fun man-int:get-action-trajectory :tilting ?arm ?grasp ?objects
                       :tilt-approach-poses ?left-approach-poses ?left-tilt-poses))
             
        (and (equal ?left-approach-poses NIL)
             (equal ?left-tilt-poses NIL)))
     
    (-> (equal ?arm :right)
        (and (lisp-fun man-int:get-action-trajectory :pouring ?arm ?grasp ?objects 
                       ?right-pouring-pose)
             (lisp-fun man-int:get-traj-poses-by-label ?right-pouring-pose :approach
                       ?right-approach-poses)
             (lisp-fun man-int:get-action-trajectory :tilting ?arm ?grasp ?objects
                       :tilt-approach-poses ?right-approach-poses ?right-tilt-poses))
             
        (and (equal ?right-approach-poses NIL)
             (equal ?right-tilt-poses NIL)))
    
    ;; put together resulting action designator
    (desig:designator :action ((:type :pouring)
                               (:object ?current-object-desig)
                               (:object-type ?object-type)
                               (:object-name  ?object-name)
                               (:arm ?arm)
                               (:grasp ?grasp)
                               (:left-approach-poses ?left-approach-poses)
                               (:right-approach-poses ?right-approach-poses)
                               (:left-tilt-poses ?left-tilt-poses)
                               (:right-tilt-poses ?right-tilt-poses))
                      ?resolved-action-designator))
    

  ;;###############################################################################
  ;;                                    SLICING
  ;;###############################################################################
  
  (<- (desig:action-grounding ?action-designator (slice ?resolved-action-designator))
    (spec:property ?action-designator (:type :slicing))

    ;; extract info from ?action-designator
    (spec:property ?action-designator (:object ?object-designator))
    (desig:current-designator ?object-designator ?current-object-desig)
    (spec:property ?current-object-desig (:type ?object-type))
    (spec:property ?current-object-desig (:name ?object-name))
    
    (-> (spec:property ?action-designator (:arm ?arm))
        (true)
        (man-int:robot-free-hand ?_ ?arm))
    

    (lisp-fun man-int:get-object-transform ?current-object-desig ?object-transform)


    ;; infer missing information like ?grasp type, gripping ?maximum-effort, manipulation poses
    (lisp-fun man-int:calculate-object-faces ?object-transform (?facing-robot-face ?bottom-face))
    (-> (man-int:object-rotationally-symmetric ?object-type)
        (equal ?rotationally-symmetric t)
        (equal ?rotationally-symmetric nil))
    
    (-> (spec:property ?action-designator (:grasp ?grasp))
        (true)
        (and (lisp-fun man-int:get-action-grasps ?object-type ?arm ?object-transform ?grasps)
             (member ?grasp ?grasps)))

    
    
    (lisp-fun man-int:get-action-gripping-effort ?object-type ?effort)
    (lisp-fun man-int:get-action-gripper-opening ?object-type ?gripper-opening)

    ;; calculate trajectory
    (equal ?objects (?current-object-desig))
    
    
    (-> (equal ?arm :left)
        (and (lisp-fun man-int:get-action-trajectory :slicing ?arm ?grasp ?objects
                       ?left-slicing-pose)
             (lisp-fun man-int:get-traj-poses-by-label ?left-slicing-pose :slice-up
                       ?left-slice-up-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-slicing-pose :slice-down
                       ?left-slice-down-poses))
        (and (equal ?left-slice-up-poses NIL)
             (equal ?left-slice-down-poses NIL)))
    
    (-> (equal ?arm :right)
        (and (lisp-fun man-int:get-action-trajectory :slicing ?arm ?grasp ?objects
                       ?right-slicing-pose)
             (lisp-fun man-int:get-traj-poses-by-label ?right-slicing-pose :slice-up
                       ?right-slice-up-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?right-slicing-pose :slice-down
                       ?right-slice-down-poses))
        (and(equal ?right-slice-up-poses NIL)
            (equal ?right-slice-down-poses NIL)))

    
    ;;put together resulting action designator
    (desig:designator :action ((:type :slicing)
                               (:object ?current-object-desig)
                               (:object-name  ?object-name)
                               (:arm ?arm)
                               (:gripper-opening ?gripper-opening)
                               (:effort ?effort)
                               (:grasp ?grasp)
                               (:left-slice-up-poses ?left-slice-up-poses)
                               (:right-slice-up-poses ?right-slice-up-poses)
                               (:left-slice-down-poses ?left-slice-down-poses)
                               (:right-slice-down-poses ?right-slice-down-poses))
                      ?resolved-action-designator))


  ;;###############################################################################
  ;;                                    HOLDING
  ;;###############################################################################

  (<- (desig:action-grounding ?action-designator (hold ?resolved-action-designator))
      (spec:property ?action-designator (:type :holding))

      ;; extract info from ?action-designator
      (spec:property ?action-designator (:object ?object-designator))
      (desig:current-designator ?object-designator ?current-object-desig)
      (spec:property ?current-object-desig (:type ?object-type))
      (spec:property ?current-object-desig (:name ?object-name))
      
      (-> (spec:property ?action-designator (:arm ?arm))
          (true)
          (man-int:robot-free-hand ?_ ?arm))
      

      (lisp-fun man-int:get-object-transform ?current-object-desig ?object-transform)


      ;; infer missing information like ?grasp type, gripping ?maximum-effort, manipulation poses
      (lisp-fun man-int:calculate-object-faces ?object-transform (?facing-robot-face ?bottom-face))
      (-> (man-int:object-rotationally-symmetric ?object-type)
          (equal ?rotationally-symmetric t)
          (equal ?rotationally-symmetric nil))
      
      (-> (spec:property ?action-designator (:grasp ?grasp))
          (true)
          (and (lisp-fun man-int:get-action-grasps ?object-type ?arm ?object-transform ?grasps)
               (member ?grasp ?grasps)))

      
      
      (lisp-fun man-int:get-action-gripping-effort ?object-type ?effort)
      (lisp-fun man-int:get-action-gripper-opening ?object-type ?gripper-opening)

      ;; calculate trajectory
      (equal ?objects (?current-object-desig))
      
      
      (-> (equal ?arm :left)
          (and (lisp-fun man-int:get-action-trajectory :picking-up ?arm ?grasp ?objects
                         ?left-trajectory)
               (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :reaching
                         ?left-reach-poses)
               (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :grasping
                         ?left-grasp-poses))
          
          (and (equal ?left-reach-poses NIL)
               (equal ?left-grasp-poses NIL)))


      (-> (equal ?arm :right)
          (and (lisp-fun man-int:get-action-trajectory :picking-up ?arm ?grasp ?objects
                         ?right-trajectory)
               (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :reaching
                         ?right-reach-poses)
               (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :grasping
                         ?right-grasp-poses))
          (and (equal ?right-reach-poses NIL)
               (equal ?right-grasp-poses NIL)))
      
      ;;put together resulting action designator
      (desig:designator :action ((:type :holding)
                                 (:object ?current-object-desig)
                                 (:object-name  ?object-name)
                                 (:arm ?arm)
                                 (:gripper-opening ?gripper-opening)
                                 (:effort ?effort)
                                 (:grasp ?grasp)
                                 (:left-reach-poses ?left-reach-poses)
                                 (:right-reach-poses ?right-reach-poses)
                                 (:left-grasp-poses ?left-grasp-poses)
                                 (:right-grasp-poses ?right-grasp-poses))
                        ?resolved-action-designator)))







