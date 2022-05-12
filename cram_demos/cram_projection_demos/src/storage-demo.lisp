;;;
;;; Copyright (c) 2022, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :demos)

(defparameter *storage-poses*
  `(("board0"
     (:dish-washer-tabs . ((0 0.4 0.0858) (-0.5 0.5 0.5 0.5)))
     (:balea-bottle . ((-0.35 0.45 0.1061) (0 0 0.5 0.5)))
     (:chassis . ((-0.7 0.45 0.078) (0 0 0 1)))
     (:front-wheel . ((0.4 0.43 0.0386) (0 0 0 1)))
     (:propeller . ((0.8 0.41 0.02635) (0 0 0 1)))

     (:mug . ((0.8 0.4 0.07434) (0 0 0.8 0.2)))
     (:spatula . ((0.5 0.41 0.03343) (-0.001785 0.01511 0.112743 0.99351)))
     (:bowl-round . ((0.2 0.4 0.05541) (0 0 0 1)))
     (:pot . ((-0.35 0.35 0.106334) (0 0 0 1)))
     (:fork . ((-0.7 0.41 0.03) (0 0 0.93 0.37))))

    ("board2"
     (:bowl . ((-0.8 0.4 0.07655) (0 0 0 1)))
     (:cup . ((-0.5 0.4 0.10155) (0 0 0 1)))
     (:spoon . ((-0.1 0.4 0.03274) (0 0 0.3 0.7)))
     (:breakfast-cereal . ((0.2 0.4 0.1279) (0 0 0 1)))
     (:milk . ((0.6 0.4 0.11369) (0 0 0 1)))

     (:denkmit-entkalker . ((-0.8 0.43 0.11383) (0 0 0 1)))
     (:heitmann-citronensaeure . ((-0.3 0.42 0.121446) (0 0 0 1)))
     (:kuehne-essig-essenz . ((-0.1 0.4 0.112) (0 0 0.3 0.7)))
     (:domestos-allzweckreiniger . ((0.2 0.4 0.17265) (0 0 0 1)))
     (:shoe . ((0.6 0.4 0.11755) (0 0 0.1 0.9))))))

(defun spawn-storage-objects (&key (attach t) (spawning-poses-relative *storage-poses*)
                                objects-list (weight 0.0))
  (kill-and-detach-all)
  (let ((poses (make-poses-relative-multiple spawning-poses-relative)))
    (mapcar (lambda (type-and-pose)
              (destructuring-bind (type . pose) type-and-pose
                (when (or (null objects-list) (member type objects-list))
                  (spawn-object-n-times type pose 1
                                        ;; color
                                        (case type
                                          (:dish-washer-tabs '(0 1 0))
                                          (:balea-bottle '(1 0.5 0))
                                          ((:chassis :propeller) *yellow-plane*)
                                          (:front-wheel *black-plane*)
                                          (t nil))
                                        nil nil
                                        (case type
                                          ;; dishwasher tabs are unstable, so mass=0
                                          ;; (:dish-washer-tabs 0.0)
                                          (t weight)))
                  ;; attach object to environment
                  (when attach
                    (btr:attach-object (btr:get-environment-object)
                                       (btr:object btr:*current-bullet-world*
                                                   (intern (format nil "~a-0" type) :keyword))
                                       :link (car (find type spawning-poses-relative
                                                        :key (lambda (urdf-name-and-list)
                                                               (car (find type
                                                                          (cdr urdf-name-and-list)
                                                                          :key #'car))))))))))
            poses)))




(defun storage-demo (&optional (objects-list '(:dish-washer-tabs :balea-bottle
                                               :chassis :front-wheel :propeller
                                               :bowl :cup :spoon :milk :breakfast-cereal)))
  ;; (setf cram-tf:*tf-broadcasting-enabled* t)
  (urdf-proj:with-simulated-robot
    (spawn-storage-objects :objects-list objects-list)
    (let ((old-visibility btr:*visibility-threshold*))
      (setf btr:*visibility-threshold*
            (case (rob-int:get-robot-name)
              ;; perceiving with an object in hand is hard, and perceiving the pot
              ;; even without objects in hand is also too hard
              ((:iai-donbot :kmr-iiwa) 0.4)
              (t 0.7)))
      (unwind-protect

           (let ((?environment-name
                   (rob-int:get-environment-name)))

             ;; bring objects to table
             (dolist (object-type-and-urdf-name
                      (apply
                       (alexandria:curry #'concatenate 'list)
                       (mapcar (lambda (urdf-name-and-stuff)
                                 (mapcar (lambda (type-and-pose)
                                           (cons (car type-and-pose)
                                                 (car urdf-name-and-stuff)))
                                         (cdr urdf-name-and-stuff)))
                               *storage-poses*)))

               (let ((?object-type (car object-type-and-urdf-name))
                     (?urdf-name (cdr object-type-and-urdf-name)))
                 (when (member ?object-type objects-list)
                   (cpl:with-failure-handling
                       ((cpl:simple-plan-failure (e)
                          (declare (ignore e))
                          (return)))
                     (exe:perform
                      (desig:an action
                                (type transporting)
                                (object (desig:an object
                                                  (type ?object-type)
                                                  (location (desig:a location
                                                                     (on (desig:an object
                                                                                   (type shelf)
                                                                                   (urdf-name ?urdf-name)
                                                                                   (part-of ?environment-name)))
                                                                     (side left)))))
                                (target (desig:a location
                                                 (on (desig:an object
                                                               (type table)
                                                               (urdf-name top)
                                                               (part-of ?environment-name)))
                                                 (for (desig:an object
                                                                (type ?object-type)))))))))))

             ;; put objects back
             (dolist (object-type-and-urdf-name
                      (reverse
                       (apply
                        (alexandria:curry #'concatenate 'list)
                        (mapcar (lambda (urdf-name-and-stuff)
                                  (mapcar (lambda (type-and-pose)
                                            (cons (car type-and-pose)
                                                  (if (string= (car urdf-name-and-stuff)
                                                               "board0")
                                                      "board2"
                                                      "board0")))
                                          (cdr urdf-name-and-stuff)))
                                *storage-poses*))))

               (let* ((?object-type (car object-type-and-urdf-name))
                      (?urdf-name (cdr object-type-and-urdf-name))
                      (?grasps (case ?object-type
                                 (:bowl '(:top-left-tilted :top-right-tilted))
                                 (:dish-washer-tabs '(:back :front))
                                 (:balea-bottle '(:back :front))
                                 ((:shoe :mug) '(:back :front :left-side :right-side))
                                 ;; (:pot '(:left-side :right-side))
                                 (t (cdr (assoc ?object-type *object-grasps*))))))
                 (when (member ?object-type objects-list)
                   (cpl:with-failure-handling
                       ((cpl:simple-plan-failure (e)
                          (declare (ignore e))
                          (return)))
                     (exe:perform
                      (desig:an action
                                (type transporting)
                                (object (desig:an object
                                                  (type ?object-type)
                                                  (location (desig:a location
                                                                     (on (desig:an object
                                                                                   (type table)
                                                                                   (urdf-name top)
                                                                                   (part-of ?environment-name)))))))
                                (target (desig:a location
                                                 (on (desig:an object
                                                               (type shelf)
                                                               (urdf-name ?urdf-name)
                                                               (part-of ?environment-name)))
                                                 (side left)
                                                 (for (desig:an object
                                                                (type ?object-type)))))
                                (grasps ?grasps))))))))

        (setf btr:*visibility-threshold* old-visibility)))))

(defun storage-demo-new-objects ()
  (storage-demo '(:mug :pot :bowl-round :fork :spatula
                  :denkmit-entkalker :heitmann-citronensaeure :kuehne-essig-essenz
                  :domestos-allzweckreiniger :shoe)))
