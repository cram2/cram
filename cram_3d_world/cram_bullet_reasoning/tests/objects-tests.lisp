;;;
;;; Copyright (c) 2019, Thomas Lipps <tlipps@uni-bremen.de>
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

(in-package :btr-tests)

;; Help functions to convert objects to lists, so these can be
;; compared with equals
(defun orientation->list (orient)
  (list (cl-transforms:w orient)
        (cl-transforms:x orient)
        (cl-transforms:y orient)
        (cl-transforms:z orient)))

(defun vector->list (vector)
  (list (cl-transforms:x vector)
        (cl-transforms:y vector)
        (cl-transforms:z vector)))

(defun pose->list (pose)
  (list (vector->list (cl-transforms:origin pose))
        (orientation->list (cl-transforms:orientation pose))))

(defun pose-equal (other-pose pose)
  (equal (pose->list pose)
         (pose->list other-pose)))

(defun spawn-robot ()
  (setf rob-int:*robot-urdf*
        (cl-urdf:parse-urdf
         (roslisp:get-param "robot_description")))
  (prolog:prolog
   `(and (btr:bullet-world ?world)
         (rob-int:robot ?robot)
         (assert (btr:object ?world :urdf ?robot ((0 0 0) (0 0 0 1))
                             :urdf ,rob-int::*robot-urdf*))
         (assert (btr:joint-state ?world ?robot (("torso_lift_joint"
                                                  0.15d0))))))
  (btr:detach-all-objects (btr:get-robot-object)))

(defun spawn-kitchen ()
  (let ((kitchen-urdf
          (cl-urdf:parse-urdf
           (roslisp:get-param "kitchen_description"))))
    (prolog:prolog
     `(and (btr:bullet-world ?world)
           (man-int:environment-name ?environment-name)
           (assert (btr:object ?world :urdf ?environment-name ((0 0 0) (0 0 0 1))
                                            :urdf ,kitchen-urdf)))))
  (btr:detach-all-objects (btr:get-environment-object)))


(define-test create-static-collision-information-works
  ;; Tests if the collision information is properly saved and if it
  ;; has an effect on the object by simulating the bullet world
  (btr-utils:spawn-object 'o1 :mug :pose
                          '((-1 0.0 0.92)(0 0 0 1)))
  (let* ((pose-o1 (btr:pose (btr:object btr:*current-bullet-world* 'o1))))

    (btr::create-static-collision-information
     (btr:object btr:*current-bullet-world* 'o1))
    (lisp-unit:assert-equal :CF-STATIC-OBJECT (car
                                               (cl-bullet:collision-flags
                                                (first
                                                 (btr:rigid-bodies (btr:object
                                                                    btr:*current-bullet-world* 'o1))))))
    (btr:simulate btr:*current-bullet-world* 1)
    (let ((new-pose-o1
            (btr:pose (btr:object btr:*current-bullet-world* 'o1))))
      (lisp-unit:assert-true (pose-equal pose-o1 new-pose-o1))))

  ;; recreate begin state for next test case
  (btr:remove-object btr:*current-bullet-world* 'o1))

(define-test create-static-collision-information-works-with-more-objects
  ;; Tests if the collision information is properly saved and if it
  ;; has an effect on the object by simulating the bullet world
  (btr-utils:spawn-object 'o1 :mug :pose
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'o2 :mug :pose
                          '((-1 0.0 0.92)(0 0 0 1)))
  (let* ((pose-o1 (btr:pose (btr:object btr:*current-bullet-world* 'o1)))
         (pose-o2 (btr:pose (btr:object btr:*current-bullet-world* 'o2))))

    (lisp-unit:assert-equal
     NIL
     (car
      (btr::collision-information-flags
       (car
        (btr::create-static-collision-information
         (btr:object btr:*current-bullet-world* 'o1))))))
    (lisp-unit:assert-equal :CF-STATIC-OBJECT (car
                                               (cl-bullet:collision-flags
                                                (first
                                                 (btr:rigid-bodies (btr:object
                                                                    btr:*current-bullet-world* 'o1))))))
    (lisp-unit:assert-equal
     NIL
     (car
      (btr::collision-information-flags
       (car
        (btr::create-static-collision-information
         (btr:object btr:*current-bullet-world* 'o2))))))
    (lisp-unit:assert-equal :CF-STATIC-OBJECT (car
                                               (cl-bullet:collision-flags
                                                (first
                                                 (btr:rigid-bodies (btr:object
                                                                    btr:*current-bullet-world* 'o2))))))
    (btr:simulate btr:*current-bullet-world* 1)
    (let ((new-pose-o1
            (btr:pose (btr:object btr:*current-bullet-world* 'o1)))
          (new-pose-o2
            (btr:pose (btr:object btr:*current-bullet-world* 'o2))))
      (lisp-unit:assert-true (pose-equal pose-o1 new-pose-o1))
      (lisp-unit:assert-true (pose-equal pose-o2 new-pose-o2))))

  ;; recreate begin state for next test case
  (btr:remove-object btr:*current-bullet-world* 'o1)
  (btr:remove-object btr:*current-bullet-world* 'o2))

(define-test create-static-collision-information-works-with-attached-objects
  ;; Tests if the collision information is properly saved and if it
  ;; has an effect on the object by simulating the bullet world
  (spawn-robot)
  (btr-utils:spawn-object 'o1 :mug :pose
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'o2 :mug :pose
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'o3 :mug :pose
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr:attach-object 'o1 'o3)
  (let* ((pose-o1 (btr:pose (btr:object btr:*current-bullet-world* 'o1)))
         (pose-o2 (btr:pose (btr:object btr:*current-bullet-world* 'o2))))
    
    (lisp-unit:assert-equal
     NIL
     (car
      (btr::collision-information-flags
       (car
        (btr::create-static-collision-information
         (btr:object btr:*current-bullet-world* 'o1))))))
    (lisp-unit:assert-equal :CF-STATIC-OBJECT (car
                                               (cl-bullet:collision-flags
                                                (first
                                                 (btr:rigid-bodies 
                                                  (btr:object
                                                   btr:*current-bullet-world* 
                                                   'o1))))))
    (lisp-unit:assert-equal
     NIL
     (car
      (btr::collision-information-flags
       (car
        (btr::create-static-collision-information
         (btr:object btr:*current-bullet-world* 'o2))))))
    (lisp-unit:assert-equal :CF-STATIC-OBJECT (car
                                               (cl-bullet:collision-flags
                                                (first
                                                 (btr:rigid-bodies 
                                                  (btr:object
                                                   btr:*current-bullet-world*
                                                   'o2))))))
    (btr:simulate btr:*current-bullet-world* 1)
    (let ((new-pose-o1
            (btr:pose (btr:object btr:*current-bullet-world* 'o1)))
          (new-pose-o2
            (btr:pose (btr:object btr:*current-bullet-world* 'o2))))
      (lisp-unit:assert-true (pose-equal pose-o1 new-pose-o1))
      (lisp-unit:assert-true (pose-equal pose-o2 new-pose-o2))))
  ;; recreate begin state for next test case
  (btr:remove-object btr:*current-bullet-world* 'o1)
  (btr:remove-object btr:*current-bullet-world* 'o2)
  (btr:remove-object btr:*current-bullet-world* 'o3))

(define-test create-static-collision-information-works-with-static-objects
  ;; Tests if the collision information is properly saved and if it
  ;; has an effect on the object by simulating the bullet world
  (btr-utils:spawn-object 'o1 :mug :pose
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'o2 :mug :pose
                          '((-1 0.0 0.92)(0 0 0 1)))
  (loop for body in (apply 'concatenate 'list
                           (list (btr:rigid-bodies (btr:object
                                                    btr:*current-bullet-world*
                                                    'o1))
                                 (btr:rigid-bodies (btr:object
                                                    btr:*current-bullet-world*
                                                    'o2))))
        do
           (setf
            (btr::collision-flags body)
            :CF-STATIC-OBJECT))
  (let* ((pose-o1 (btr:pose (btr:object btr:*current-bullet-world* 'o1)))
         (pose-o2 (btr:pose (btr:object btr:*current-bullet-world* 'o2))))
    
    (lisp-unit:assert-equal
     :CF-STATIC-OBJECT
     (car
      (btr::collision-information-flags
       (car
        (btr::create-static-collision-information
         (btr:object btr:*current-bullet-world* 'o1))))))
    (lisp-unit:assert-equal :CF-STATIC-OBJECT (car
                                               (cl-bullet:collision-flags
                                                (first
                                                 (btr:rigid-bodies 
                                                  (btr:object
                                                   btr:*current-bullet-world* 
                                                   'o1))))))
    (lisp-unit:assert-equal
     :CF-STATIC-OBJECT
     (car
      (btr::collision-information-flags
       (car
        (btr::create-static-collision-information
         (btr:object btr:*current-bullet-world* 'o2))))))
    (lisp-unit:assert-equal :CF-STATIC-OBJECT (car
                                               (cl-bullet:collision-flags
                                                (first
                                                 (btr:rigid-bodies 
                                                  (btr:object
                                                   btr:*current-bullet-world*
                                                   'o2))))))
    (btr:simulate btr:*current-bullet-world* 1)
    (let ((new-pose-o1
            (btr:pose (btr:object btr:*current-bullet-world* 'o1)))
          (new-pose-o2
            (btr:pose (btr:object btr:*current-bullet-world* 'o2))))
      (lisp-unit:assert-true (pose-equal pose-o1 new-pose-o1))
      (lisp-unit:assert-true (pose-equal pose-o2 new-pose-o2))))

  ;; recreate begin state for next test case
  (btr:remove-object btr:*current-bullet-world* 'o1)
  (btr:remove-object btr:*current-bullet-world* 'o2))

(define-test create-static-collision-information-works-with-attached-static-objects
  ;; Tests if the collision information is properly saved and if it
  ;; has an effect on the object by simulating the bullet world
  (spawn-robot)
  (btr-utils:spawn-object 'o1 :mug :pose
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'o2 :mug :pose
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'o3 :mug :pose
                          '((-1 0.0 0.92)(0 0 0 1)))
  (loop for body in (apply 'concatenate 'list
                           (list (btr:rigid-bodies (btr:object
                                                    btr:*current-bullet-world*
                                                    'o1))
                                 (btr:rigid-bodies (btr:object
                                                    btr:*current-bullet-world*
                                                    'o2))
                                 (btr:rigid-bodies (btr:object
                                                    btr:*current-bullet-world*
                                                    'o3))))
        do
           (setf
            (btr::collision-flags body)
            :CF-STATIC-OBJECT))
  (btr:attach-object (btr:object btr:*current-bullet-world* 'o1)
                     (btr:object btr:*current-bullet-world* 'o3))
  (let* ((pose-o1 (btr:pose (btr:object btr:*current-bullet-world* 'o1)))
         (pose-o2 (btr:pose (btr:object btr:*current-bullet-world* 'o2))))
    
    (lisp-unit:assert-equal
     :CF-STATIC-OBJECT
     (car
      (btr::collision-information-flags
       (car
        (btr::create-static-collision-information
         (btr:object btr:*current-bullet-world* 'o1))))))
    (lisp-unit:assert-equal :CF-STATIC-OBJECT (car
                                               (cl-bullet:collision-flags
                                                (first
                                                 (btr:rigid-bodies 
                                                  (btr:object
                                                   btr:*current-bullet-world* 
                                                   'o1))))))
    (lisp-unit:assert-equal
     :CF-STATIC-OBJECT
     (car
      (btr::collision-information-flags
       (car
        (btr::create-static-collision-information
         (btr:object btr:*current-bullet-world* 'o2))))))
    (lisp-unit:assert-equal :CF-STATIC-OBJECT (car
                                               (cl-bullet:collision-flags
                                                (first
                                                 (btr:rigid-bodies 
                                                  (btr:object
                                                   btr:*current-bullet-world*
                                                   'o2))))))
    (btr:simulate btr:*current-bullet-world* 1)
    (let ((new-pose-o1
            (btr:pose (btr:object btr:*current-bullet-world* 'o1)))
          (new-pose-o2
            (btr:pose (btr:object btr:*current-bullet-world* 'o2))))
      (lisp-unit:assert-true (pose-equal pose-o1 new-pose-o1))
      (lisp-unit:assert-true (pose-equal pose-o2 new-pose-o2))))
  ;; recreate begin state for next test case
  (btr:remove-object btr:*current-bullet-world* 'o1)
  (btr:remove-object btr:*current-bullet-world* 'o2)
  (btr:remove-object btr:*current-bullet-world* 'o3))


(define-test reset-collision-information-works-with-attached-objects
  ;; Tests if the collision information is properly reseted and if it has
  ;; an effect on the object by simulating the bullet world
  (spawn-robot)
  (btr-utils:spawn-object 'o1 :mug :pose
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'o2 :mug :pose
                          '((-1 0.0 0.92)(0 0 0 1)))
  
  (let* ((pose-o1 (btr:pose (btr:object btr:*current-bullet-world* 'o1))))
    (btr:attach-object 'o1 'o2)
    ;;simulate detachment
    (setf (slot-value (btr:object btr:*current-bullet-world* 'o1) 'attached-objects)
          nil)
    (btr::reset-collision-information (btr:object btr:*current-bullet-world* 'o1)
                                      (list (btr::make-collision-information
                                             :rigid-body-name 'o1 :flags NIL)))
    (lisp-unit:assert-equal NIL (car
                                 (cl-bullet:collision-flags
                                  (first
                                   (btr:rigid-bodies
                                    (btr:object btr:*current-bullet-world* 'o1))))))
    (btr:simulate btr:*current-bullet-world* 1)
    (let ((new-pose-o1
            (btr:pose (btr:object btr:*current-bullet-world* 'o1))))
      
      (lisp-unit:assert-false (pose-equal pose-o1 new-pose-o1))))
  
  ;; recreate begin state for next test case
  (btr:remove-object btr:*current-bullet-world* 'o1)
  (btr:remove-object btr:*current-bullet-world* 'o2))

(define-test reset-collision-information-fails
  ;; Tests if nothing happens if nil is passed
  (btr-utils:spawn-object 'o1 :mug :pose
                          '((-1 0.0 0.92)(0 0 0 1)))
  (lisp-unit:assert-false (btr::reset-collision-information
                           (btr:object btr:*current-bullet-world* 'o1) nil))
  (btr:remove-object btr:*current-bullet-world* 'o1))

(define-test attach-object-called-with-item-symbols
  ;; Tests if the attach-object function gets properly called if it was
  ;; called with the object item names
  (spawn-robot)
  (btr-utils:spawn-object 'oo1 :mug :pose
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'oo2 :mug :pose
                          '((-1 0.0 0.92)(0 0 0 1)))
  
  (btr:attach-object 'oo1 'oo2)
  (lisp-unit:assert-equal
   'oo1
   (car (assoc 'oo1
               (btr:attached-objects
                (btr:object btr:*current-bullet-world* 'oo2)))))
  (lisp-unit:assert-equal
   'oo2
   (car (assoc 'oo2 (btr:attached-objects
                     (btr:object btr:*current-bullet-world* 'oo1)))))
  
  (btr:remove-object btr:*current-bullet-world* 'oo1)
  (btr:remove-object btr:*current-bullet-world* 'oo2))

(define-test attach-object-called-with-more-items-connecting-to-one-item-in-one-call
  ;; Attaches three objects and 'o1 is connected with both objects in
  ;; one call
  (btr-utils:spawn-object 'o1 :mug :pose
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'o2 :mug :pose
                          '((-1 0.75 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'o3 :mug :pose
                          '((-1 0.75 0.92)(0 0 0 1)))

  ;;connect objects o2 <-> o1 <-> o3
  (btr:attach-object (list 'o2 'o3) 'o1)

  ;;these are connected o2 <-> o1 <-> o3
  (lisp-unit:assert-equal
   'o2
   (car (assoc 'o2 (btr:attached-objects
                    (btr:object btr:*current-bullet-world* 'o1)))))
  (lisp-unit:assert-equal
   'o3
   (car (assoc 'o3 (btr:attached-objects
                    (btr:object btr:*current-bullet-world* 'o1)))))
  (lisp-unit:assert-equal
   'o1
   (car (assoc 'o1 (btr:attached-objects
                    (btr:object btr:*current-bullet-world* 'o2)))))
  (lisp-unit:assert-equal
   'o1
   (car (assoc 'o1 (btr:attached-objects
                    (btr:object btr:*current-bullet-world* 'o3)))))
  ;;these are not connected to each other: o2 o3
  (lisp-unit:assert-false
   (equal 'o2 (car (assoc 'o2 (btr:attached-objects
                               (btr:object btr:*current-bullet-world* 'o3))))))
  (lisp-unit:assert-false
   (equal 'o3 (car (assoc 'o3 (btr:attached-objects
                               (btr:object btr:*current-bullet-world* 'o2))))))

  ;; attach-object-more-objects-connected-bidirectional-to-one-object-in-one-call begin state for next test case
  (btr:remove-object btr:*current-bullet-world* 'o1)
  (btr:remove-object btr:*current-bullet-world* 'o2)
  (btr:remove-object btr:*current-bullet-world* 'o3))

(define-test attach-object-called-with-item-symbol-and-nil-as-parameter
  ;; Tests if the attach-object function does nothing if called with
  ;; nil and an valid item object
  (btr-utils:spawn-object 'oo1 :mug :pose
                          '((-1 0.0 0.92)(0 0 0 1)))

  (lisp-unit:assert-false (btr:attach-object 'oo1 nil))
  (lisp-unit:assert-number-equal
   0
   (list-length (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo1))))
  (lisp-unit:assert-false (btr:attach-object nil 'oo1))
  (lisp-unit:assert-number-equal
   0
   (list-length (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo1))))
  (lisp-unit:assert-false (btr:attach-object nil nil))

  (btr:remove-object btr:*current-bullet-world* 'oo1))

(define-test detach-object-called-with-item-symbols
  ;; Tests if the detach-object function gets properly called if it
  ;; was called with the object item names
  (btr-utils:spawn-object 'oo1 :mug :pose
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'oo2 :mug :pose
                          '((-1 0.0 0.92)(0 0 0 1)))

  (btr:attach-object 'oo1 'oo2)
  (lisp-unit:assert-equal
   'oo1
   (car (assoc 'oo1 (btr:attached-objects
                     (btr:object btr:*current-bullet-world* 'oo2)))))
  (lisp-unit:assert-equal
   'oo2
   (car (assoc 'oo2 (btr:attached-objects
                     (btr:object btr:*current-bullet-world* 'oo1)))))

  (btr:detach-object 'oo1 'oo2)
  (lisp-unit:assert-false
   (equal 'oo2 (car (assoc 'oo2 (btr:attached-objects
                                 (btr:object btr:*current-bullet-world* 'oo1))))))
  (lisp-unit:assert-false
   (equal 'oo1 (car (assoc 'oo1 (btr:attached-objects
                                 (btr:object btr:*current-bullet-world* 'oo2))))))

  (btr:remove-object btr:*current-bullet-world* 'oo1)
  (btr:remove-object btr:*current-bullet-world* 'oo2))


(define-test detach-object-called-with-item-symbol-and-nil-as-parameter
  ;; Tests if the detach-object function does nothing if called with
  ;; nil and an valid item object
  (btr-utils:spawn-object 'oo1 :mug :pose
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'oo2 :mug :pose
                          '((-1 0.0 0.92)(0 0 0 1)))

  (btr:attach-object 'oo1 'oo2)
  (lisp-unit:assert-equal
   'oo1
   (car (assoc 'oo1 (btr:attached-objects
                     (btr:object btr:*current-bullet-world* 'oo2)))))
  (lisp-unit:assert-equal
   'oo2
   (car (assoc 'oo2 (btr:attached-objects
                     (btr:object btr:*current-bullet-world* 'oo1)))))

  (lisp-unit:assert-false (btr:detach-object nil 'oo1))
  (lisp-unit:assert-equal
   'oo1
   (car (assoc 'oo1 (btr:attached-objects
                     (btr:object btr:*current-bullet-world* 'oo2)))))
  (lisp-unit:assert-equal
   'oo2
   (car (assoc 'oo2 (btr:attached-objects
                     (btr:object btr:*current-bullet-world* 'oo1)))))

  (lisp-unit:assert-false (btr:detach-object 'oo1 nil))
  (lisp-unit:assert-equal
   'oo1
   (car (assoc 'oo1 (btr:attached-objects
                     (btr:object btr:*current-bullet-world* 'oo2)))))
  (lisp-unit:assert-equal
   'oo2
   (car (assoc 'oo2 (btr:attached-objects
                     (btr:object btr:*current-bullet-world* 'oo1)))))

  (lisp-unit:assert-false (btr:detach-object nil nil))

  (btr:remove-object btr:*current-bullet-world* 'oo1)
  (btr:remove-object btr:*current-bullet-world* 'oo2))


