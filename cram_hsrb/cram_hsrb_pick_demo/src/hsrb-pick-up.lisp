;;;
;;; Copyright (c) 2020, Vanessa Hassouna <hassouna@uni-bremen.de>
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

(in-package :hsrb-demo)

(defun pick-up-object (object-name object-type)
"move hsr and grasp the object"
  (mirror-object-pose object-name)
  (grasp-object object-name object-type))

(defun mirror-object-pose (object-name)
  "moves the robot as the object while ignoring the enviroment"
  (let* ((?object-pose (btr:object-pose object-name)))
    (roslisp:with-fields (x y) (cl-tf:origin ?object-pose)
      (btr-utils:move-robot 
       (cl-tf:make-pose 
        (cl-tf:make-3d-vector x y 0)
        (cl-tf:orientation ?object-pose)))))
  ;;update that the robot-state changed otherwise the transformer dosen't update
  (coe::on-event (make-instance 'cpoe::robot-state-changed)))

(defun grasp-object (object-name object-type)
  "places the robot in front-/left-/right-/back-side of the object and tries 
to grasp the object, if that fails the next position in list will be tried" 
  (let* 
      ((nav-pose 
         ;;applying append function to the successive subsets of the list
         (reduce #'append 
                 ;;goes through each element in the sequence, and returns a new list
                 (mapcar
                  (lambda (list-poses)
                    (let ((tmp-list nil))
                      (loop for a from 0.0 to 0.4 by 0.01
                            do
                               ;;push element to list
                               (push
                                (roslisp:with-fields (x y)
                                    (cl-tf:origin (car list-poses))
                                  ;;check if the offset needs to be summed or subtracted
                                  (if (or (eq (car (cdr list-poses)) :front)
                                          (eq (car (cdr list-poses)) :back))
                                      (setf x (+ x 
                                                 (if (> x 0) 
                                                     a
                                                     (- a))))
                                      (setf y (+ y 
                                                 (if (> y 0) 
                                                     a
                                                     (- a)))))
                                  ;;create new list with a stamped-pose and x/y w
                                  ;;with edited offset and the grasping-side
                                  (list
                                   (cl-tf:transform-pose-stamped 
                                    cram-tf:*transformer* 
                                    :pose 
                                    (cl-tf:make-pose-stamped 
                                     "base_footprint" 0
                                     (cl-tf:make-3d-vector x y 0)
                                     (cl-tf:orientation (car list-poses)))
                                    :target-frame "map")

                                   ;;get grasping-side
                                   (cdr list-poses)))
                                ;;push to my-list
                                tmp-list))
                      ;;loop done and reverse tmp-list such that the lower value is first
                      (reverse tmp-list))) 
                  ;;the basic list that goes through the mapcar 
                  (list 
                   (list (cl-tf:make-pose 
                          (cl-tf:make-3d-vector 0.40  0.07769999504089356d0 0)
                          (cl-tf:euler->quaternion :az pi)) 
                         :front)
                   (list (cl-tf:make-pose 
                          (cl-tf:make-3d-vector -0.40  -0.07769999504089356d0 0)
                          (cl-tf:make-identity-rotation))
                         :back)
                   
                   (list (cl-tf:make-pose 
                          (cl-tf:make-3d-vector -0.07769999504089356d0 0.40 0)
                          (cl-tf:euler->quaternion :az (- (/ pi 2))))
                         :left-side)
                   
                   (list (cl-tf:make-pose 
                          (cl-tf:make-3d-vector +0.07769999504089356d0 -0.4 0)
                          (cl-tf:euler->quaternion :az  (/ pi 2)))
                         :right-side)))))
       ;;sets object-type to prolog variable 
       (?object-type object-type))
    
    ;;retrying 78 times since the list has as many entries
    (cpl:with-retry-counters ((going-retry 163))
      (cpl:with-failure-handling
          (((or common-fail:low-level-failure 
                cl::simple-error
                cl::simple-type-error)

               (e)
             (roslisp:ros-warn (grasp-object fail)
                               "~%Failed with given msgs ~a~%" e)
             ;;get rid of head of nav-pose by setting only the rest
             (setf nav-pose (cdr nav-pose))

             
             (cpl:do-retry going-retry
               (roslisp:ros-warn (grasp-object fail)
                                 "~%Failed with given msgs ~a~%" e)
               (cpl:retry))
             (roslisp:ros-warn (grasp-object fail)
                               "~%No more retries~%")))
        ;;park-robot (percieve pose)
        (btr-utils:park-robot)
        (let* ((?nav-pose
                 (car (car nav-pose)))
               (?object-pose (cl-tf:pose->pose-stamped
                              "map" 0 (btr:object-pose object-name))))
          
          ;;perform a action type going to first pose
          (cram-executive:perform
           (desig:an action
                     (type going)
                     (target (desig:a location (pose ?nav-pose)))))

          ;;perform a action type looking towards the object  
          (exe:perform
           (desig:an action
                     (type looking)
                     (target (desig:a location (pose ?object-pose))))))
        
        (let* ((?object-desig
                 (exe:perform (desig:a motion
                                       (type detecting)
                                       (object (desig:an object 
                                                         (type ?object-type))))))
               (?grasp (caadar  nav-pose)))
   
          ;;perform a action type picking-up with given grasp and object
          (exe:perform (desig:an action
                                 (type picking-up)
                                 (arm :left)
                                 (grasp ?grasp)
                                 (object ?object-desig))))))))


        

