;;;
;;; Copyright (c) 2019, Vanessa Hassouna <hassouna@uni-bremen.de>
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

;; NOTE: unfortunately, cpl:def-cram-function doesn't use the specified lambda list.
;; Because of that declare type statements do not work on variables from the lambda list,
;; and the auto completion of arguments is useless as well.
;; As we would really like to have declare statements, our plans are simple defuns.
;; If in the future one would want to use def-cram-function for plan transformations,
;; one can always def-cram-function that calls a normal function.
(defun pour (&key
               ((:object ?object-designator))
               ((:object-name  ?object-name))
               ((:object-type ?object-type))
               ((:arms ?arms))
               ((:grasp ?grasp))
               ((:left-approach-poses ?left-approach-poses))
               ((:right-approach-poses ?right-approach-poses))
               ((:left-tilt-poses ?left-tilt-poses))
               ((:right-tilt-poses ?right-tilt-poses))
             &allow-other-keys)
  ;; (declare (type desig:object-designator ?object-designator)
  ;;          (type keyword ?arm ?grasp)
  ;;          (type (or null list) ; yes, null is also list, but this is better reachability
  ;;                ?left-approach-poses ?right-approach-poses
  ;;                ?left-tilt-poses ?right-tilt-poses))
  "Object already in hand, approach 2nd object, tilt 100degree, tilt back"
  
  ;; (let* ((?a-object-in-hand
  ;;          (if (eq ?arm :left)
  ;;              (cdr (first (car (prolog:prolog '(cpoe:object-in-hand ? :left)))))
  ;;              (cdr (first (car (prolog:prolog '(cpoe:object-in-hand ? :right)))))))             

         
    ;;      (?description-hand
    ;;        (roslisp:with-fields (description) ?a-object-in-hand description))

    ;;      (?object-in-hand-name
    ;;        (second (second ?description-hand)))
    ;;      (?object-in-hand-type
    ;;        (second (first ?description-hand)))

         
    ;;      (x-dim-object
    ;;        (cl-transforms:x
    ;;         (cl-bullet::bounding-box-dimensions
    ;;          (btr::aabb(btr:object btr:*current-bullet-world*  ?object-in-hand-name)))))
         
    ;;      (z-dim-object
    ;;        (cl-transforms:z
    ;;         (cl-bullet::bounding-box-dimensions
    ;;          (btr::aabb(btr:object btr:*current-bullet-world*  ?object-in-hand-name)))))
         
    ;;      (y-gripper-position-offset
    ;;        (/ x-dim-object 2))
    ;;      (z-gripper-position-offset
    ;;        (/ z-dim-object 5)))

    
    ;; (when (not (eq ?object-type ?object-in-hand-type)) 
      
      
    ;;   (setf ?left-approach-poses
    ;;         (mapcar (lambda (slice-left)
    ;;                   (translate-pose slice-left
    ;;                                   :y-offset (+ y-gripper-position-offset)
    ;;                                   :z-offset z-gripper-position-offset))
    ;;                 ?left-approach-poses))
      
    ;;   (setf ?left-tilt-poses
    ;;         (mapcar (lambda (slice-left)
    ;;                   (translate-pose slice-left
    ;;                                   :y-offset (+ y-gripper-position-offset)
    ;;                                   :z-offset z-gripper-position-offset))
    ;;                 ?left-tilt-poses))

    ;;   (setf ?right-approach-poses
    ;;         (mapcar (lambda (slice-left)
    ;;                   (translate-pose slice-left
    ;;                                   :y-offset (- y-gripper-position-offset)
    ;;                                   :z-offset z-gripper-position-offset))
    ;;                 ?right-approach-poses))
      
    ;;   (setf ?right-tilt-poses
    ;;         (mapcar (lambda (slice-left)
    ;;                   (translate-pose slice-left
    ;;                                   :y-offset (- y-gripper-position-offset)
    ;;                                   :z-offset z-gripper-position-offset))
    ;;                 ?right-tilt-poses)))
    
    

    (roslisp:ros-info (cut-pour pour) "Approaching")
    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (cut-and-pour-plans pour)
                             "Manipulation messed up: ~a~%Ignoring."
                             e)
           ;; (return)
           ))
      (exe:perform
       (desig:an action
                 (type approaching)
                 (left-poses ?left-approach-poses)
                 (right-poses ?right-approach-poses))))
  (cpl:sleep 2)
  ;;(break)
    
    (roslisp:ros-info (cut-pour pour) "Tilting")
    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (cut-and-pour-plans pour)
                             "Manipulation messed up: ~a~%Ignoring."
                             e)
           ;; (return)
           ))
      (exe:perform
       (desig:an action
                 (type tilting)
                 (left-poses ?left-tilt-poses)
                 (right-poses ?right-tilt-poses))))
  (cpl:sleep 2)
  ;;(break)
    
    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (cut-and-pour-plans pour)
                             "Manipulation messed up: ~a~%Ignoring."
                             e)
           ;; (return)
           ))
      (exe:perform
       (desig:an action
                 (type approaching)
                 (left-poses ?left-approach-poses)
                 (right-poses ?right-approach-poses))))

  ;;(break)
    
    ;; (print ?object-type) (print '?object-in-hand-type)
  
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;                                      SLICE                                   ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun slice (&key
                ((:object ?object-designator))
                ((:object-name  ?object-name))
                ((:arm ?arm))
                ((:arm-support ?arm-support))
                ((:gripper-opening ?gripper-opening))
                ((:effort ?grip-effort))
                ((:grasp ?grasp))
                ((:grasp-support ?grasp-support))
                ((:left-slice-up-poses ?left-slice-up-poses))
                ((:right-slice-up-poses ?right-slice-up-poses))
                ((:left-slice-down-poses ?left-slice-down-poses))
                ((:right-slice-down-poses ?right-slice-down-poses))
                ((:collision-mode ?collision-mode))
              &allow-other-keys)
  
  ;; (declare (type desig:object-designator ?object-designator)
  ;;          (type keyword ?arm ?grasp)
  ;;          (type (or null list) ; yes, null is also list, but this is better reachability
  ;;                ?left-slice-up-poses ?left-slice-up-poses
  ;;                ?left-slice-down-poses ?left-slice-down-poses))
  "Object already in hand, approach 2nd object, tilt 100degree, tilt back"
 
  
       

    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;first cut
    

  (loop while (or ?left-slice-up-poses
                  ?left-slice-down-poses
                  ?right-slice-up-poses
                  ?right-slice-down-poses)
        do
           
           (let ((?current-left-slice-up-poses `(,(pop ?left-slice-up-poses)))
                 (?current-left-slice-down-poses `(,(pop ?left-slice-down-poses)))
                 (?current-right-slice-up-poses `(,(pop ?right-slice-up-poses)))
                 (?current-right-slice-down-poses `(,(pop ?right-slice-down-poses))))

             (roslisp:ros-info (cut-pour pour) "approach")
             (cpl:with-failure-handling
                 ((common-fail:manipulation-low-level-failure (e)
                    (roslisp:ros-warn (cut-and-pour-plans slice)
                                      "Manipulation messed up: ~a~%Ignoring."
                                      e)))
               (exe:perform
                (desig:an action
                          (type approaching)
                          (left-poses ?current-left-slice-up-poses)
                          (right-poses ?current-right-slice-up-poses)
                          (desig:when ?collision-mode
                            (collision-mode ?collision-mode)))))
             
             (roslisp:ros-info (cut-pour pour) "slice-down")
             (cpl:with-failure-handling
                 ((common-fail:manipulation-low-level-failure (e)
                    (roslisp:ros-warn (cut-and-pour-plans slice)
                                      "Manipulation messed up: ~a~%Ignoring."
                                      e)))
               (exe:perform
                (desig:an action
                          (type approaching)
                          (left-poses ?current-left-slice-down-poses)
                          (right-poses ?current-right-slice-down-poses)
                          (desig:when ?collision-mode
                            (collision-mode ?collision-mode)))))
             
             (roslisp:ros-info (cut-pour pour) "slice-up")
             (cpl:with-failure-handling
                 ((common-fail:manipulation-low-level-failure (e)
                    (roslisp:ros-warn (cut-and-pour-plans slice)
                                      "Manipulation messed up: ~a~%Ignoring."
                                      e)))
      (exe:perform
       (desig:an action
                 (type approaching)
                 (left-poses ?current-left-slice-up-poses)
                 (right-poses ?current-right-slice-up-poses)
                 (desig:when ?collision-mode
                   (collision-mode ?collision-mode))))))))


;; (defun hold (&key
;;                ((:object ?object-designator))
;;                ((:object-name ?object-name))
;;                ((:arm ?arm))
;;                ((:arm-support ?arm-support))
;;                ((:gripper-opening ?gripper-opening))
;;                ((:effort ?grip-effort))
;;                ((:grasp ?grasp))
;;                ((:left-reach-poses ?left-reach-poses))
;;                ((:right-reach-poses ?right-reach-poses))
;;                ((:left-grasp-poses ?left-grasp-poses))
;;                ((:right-grasp-poses ?right-grasp-poses))
;;              &allow-other-keys)
  
;;   ;; (declare (type desig:object-designator ?object-designator)
;;   ;;          (type keyword ?arm ?grasp)
;;   ;;          (type (or null list) ; yes, null is also list, but this is better reachability
;;   ;;                ?left-slice-up-poses ?left-slice-up-poses
;;   ;;                ?left-slice-down-poses ?left-slice-down-poses))
;;   "Object already in hand, approach 2nd object, tilt 100degree, tilt back"

;;   (cpl:par
;;     (roslisp:ros-info (pick-place pick-up) "Opening gripper")
;;     (exe:perform
;;      (desig:an action
;;                (type setting-gripper)
;;                (gripper ?arm)
;;                (position ?gripper-opening)))
;;     (roslisp:ros-info (cut-pour slicing) "Approaching")
;;     (cpl:with-failure-handling
;;         ((common-fail:manipulation-low-level-failure (e)
;;            (roslisp:ros-warn (cp-plans slicing)
;;                              "Manipulation messed up: ~a~%Ignoring."
;;                              e)
;;            ;; (return)
;;            ))
;;       (exe:perform
;;        (desig:an action
;;                  (type reaching)
;;                  (left-poses ?left-reach-poses)
;;                  (right-poses ?right-reach-poses)))))
  
;;   (roslisp:ros-info (cut-pour slicing) "Grasping")
;;   (cpl:with-failure-handling
;;       ((common-fail:manipulation-low-level-failure (e)
;;          (roslisp:ros-warn (cp-plans slicing)
;;                            "Manipulation messed up: ~a~%Ignoring."
;;                            e)
;;          (return)
;;          ))
;;     (exe:perform
;;      (desig:an action
;;                (type grasping)
;;                (object ?object-designator)
;;                (left-poses ?left-grasp-poses)
;;                (right-poses ?right-grasp-poses))))
  
;;   (roslisp:ros-info (cut-pour slicing) "Gripping")
;;   (exe:perform
;;    (desig:an action
;;              (type gripping)
;;              (gripper ?arm)
;;              (effort ?grip-effort)
;;              (object ?object-designator)))
  
;;   (roslisp:ros-info (cut-pour slicing) "Assert grasp into knowledge base")
;;   (cram-occasions-events:on-event
;;    (make-instance 'cpoe:object-attached-robot
;;      :object-name (desig:desig-prop-value ?object-designator :name)
;;      :arm ?arm
;;      :grasp ?grasp))
  
;; )




