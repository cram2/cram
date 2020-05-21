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
               ((:collision-mode ?collision-mode))
             &allow-other-keys)
  "Object already in hand, approach 2nd object, tilt 100degree, tilt back"
  
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
               (right-poses ?right-approach-poses)
               (desig:when ?collision-mode
                 (collision-mode ?collision-mode))))
    (cpl:sleep 2)
    
    (roslisp:ros-info (cut-pour pour) "Tilting")
    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (cut-and-pour-plans pour)
                             "Manipulation messed up: ~a~%Ignoring."
                             e)))
      (exe:perform
       (desig:an action
                 (type tilting)
                 (left-poses ?left-tilt-poses)
                 (right-poses ?right-tilt-poses)
                 (desig:when ?collision-mode
                   (collision-mode ?collision-mode)))))

    (cpl:sleep 2)
    
    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (cut-and-pour-plans pour)
                             "Manipulation messed up: ~a~%Ignoring."
                             e)))
      (exe:perform
       (desig:an action
                 (type approaching)
                 (left-poses ?left-approach-poses)
                 (right-poses ?right-approach-poses)
                 (desig:when ?collision-mode
                   (collision-mode ?collision-mode)))))))

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


