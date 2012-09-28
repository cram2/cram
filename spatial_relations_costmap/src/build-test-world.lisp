;;; Copyright (c) 2012, Gayane Kazhoyan <kazhoyan@in.tum.de>
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

(in-package :spatial-relations-costmap)

;; run bullet launch files (bullet_reasoning_demo)
;; ros-load-system btr-demo

;; TODO: comments
;; TODO: declares everywhere!

(defvar *bdgs* nil)
(defparameter *num-of-sets-on-table* 4)
;; (defvar *pr2* nil)
;; (defvar *sem-map* nil)

(defun start-ros-and-bullet ()
  (setf *bdgs* nil)
  ;; (setf *pr2* nil)
  ;; (setf *sem-map* nil)
  
  (cram-roslisp-common:startup-ros :anonymous nil)
  (let* ((urdf (cl-urdf:parse-urdf (roslisp:get-param "robot_description_lowres")))
         (kitchen-urdf (cl-urdf:parse-urdf (roslisp:get-param "kitchen_description"))))
    (setf *bdgs*
          (car
           (force-ll (prolog `(and
                               (clear-bullet-world)
                               (bullet-world ?w)
                               (assert (object ?w static-plane floor ((0 0 0) (0 0 0 1)) :normal (0 0 1) :constant 0))
                               (debug-window ?w)
                               (assert (object ?w semantic-map sem-map ((1.4 2.8 0) (0 0 0.9994 -0.0342)) :urdf ,kitchen-urdf))
                               (robot ?robot)
                               (assert (object ?w urdf ?robot ((0 0 0) (0 0 0 1)) :urdf ,urdf))
                               (robot-arms-parking-joint-states ?joint-states)
                               (assert (joint-state ?w ?robot ?joint-states))
                               (assert (joint-state ?w ?robot (("torso_lift_joint" 0.33))))

                               ;; (assert (object ?w mesh pot-1
                               ;;                 ((-2.15 2.14 0.945) (0 0 0 1)) ; y = 2.14
                               ;;                 :mesh pot :mass 0.2))
                               ;; (assert (object ?w mesh bowl-1 ((1.2 2 2.0) (0 0 0 1))
                               ;;                 :mesh bowl :mass 0.2))
                               ;; (assert (object ?w mesh mondamin-1 ((1.2 2 2.5) (0 0 0 1))
                               ;;                 :mesh mondamin :mass 0.2 :color (1 1 0)))

                               ,@(loop for i from 1 to *num-of-sets-on-table* collect
                                       `(assert (object ?w mesh ,(new-symbol "PLATE-" i) ((2 0 0) (0 0 0 1))
                                                        :mesh plate :mass 0.2 :color (0 1 1))))

                               ,@(loop for i from 1 to *num-of-sets-on-table* collect
                                       `(assert (object ?w mesh ,(new-symbol "MUG-" i) ((2 0 0) (0 0 0 1))
                                                        :mesh mug :mass 0.2 :color (1 0.5 0))))
                               
                               ,@(loop for i from 1 to *num-of-sets-on-table* collect
                                       `(assert (object ?w cutlery ,(new-symbol "FORK-" i) ((2 0 0) (0 0 0 1))
                                                        :color (1 0 1) :mass 0.2 :cutlery-type fork)))
                            
                               ,@(loop for i from 1 to *num-of-sets-on-table* collect
                                       `(assert (object ?w cutlery ,(new-symbol "KNIFE-" i) ((2 0 0) (0 0 0 1))
                                                        :color (1 0 0) :mass 0.2 :cutlery-type knife)))

                               
                               ;; ,@(loop for i from 1 to 7 collect
                               ;;         `(make-on-counter-top-desig ,(new-symbol "?DES" i)))

                               ;; (assign-object-pos pot-1 ?des1)
                               ;; (assign-object-pos bowl-1 ?des2)
                               ;; (assign-object-pos mondamin-1 ?des3)
                               ;; (assign-object-pos mug-1 ?des4)
                               ;; (assign-object-pos plate-1 ?des5) 
                               ;; (assign-object-pos fork-1 ?des6)
                               ;; (assign-object-pos knife-1 ?des7)
                               ;; (assign-pos-on-table-near-world-right-edge plate-1)
                               ;; (assign-pos-on-table plate-1)

                               ;; (assert (object-pose ?w plate-1 ,pose-on-table-1))
                               ;; (assert (object-pose ?w plate-3 ,pose-on-table-2))
                               )))))

    ;; (setf pr2 (var-value '?pr2 (lazy-car (prolog `(and (robot ?robot) (%object ?w ?robot ?pr2)) *bdgs*))))
    ;; (setf sem-map (var-value '?sem-map (lazy-car (prolog `(%object ?w sem-map ?sem-map) *bdgs*))))

    ;; (simulate *current-bullet-world* 50)
    ))




;; ;; to-reach object
;; (setf to-reach-desig (desig:make-designator 'desig-props:location '((desig-props:to desig-props:reach) (desig-props:obj pot-1))))

;; ;; left-of pose
;; (setf my-awesome-desig (desig:make-designator 'desig-props:location `((desig-props:left-of ,(cl-transforms:make-identity-pose)))))

;; ;; left-of object
;; (setf des (desig:make-designator 'desig-props:location '((desig-props:left-of pot-1))))

;; ;; combination
;; (setf des (desig:make-designator 'desig-props:location '((desig-props:left-of mug-1) (desig-props:right-of mondamin-1) (desig-props:on counter-top) (desig-props:name kitchen-island))))

;; ;; for a glass near a plate
;; (setf des (desig:make-designator 'desig-props:location '((desig-props:right-of plate-1) (desig-props:behind plate-1) (desig-props:near plate-1) (desig-props:for mug-1) (desig-props:on counter-top) (desig-props:name kitchen-island))))

;; ;; for a fork to the left of plate
;; (setf des (desig:make-designator 'desig-props:location '((desig-props:left-of plate-1)  (desig-props:near plate-1) (desig-props:for fork-1))))
;; (force-ll (prolog `(and (symbol-value des ?des) (assign-object-pos fork-1 ?des))))

(defun put-plates-on-table (&optional (number-of-plates 4))
  (let ((pose-on-table-1 (cl-transforms:make-pose (cl-transforms:make-3d-vector -2.2 2.14 0.8574701697205431d0) (cl-transforms:make-identity-rotation)))
        (pose-on-table-2 (cl-transforms:make-pose (cl-transforms:make-3d-vector -1.75 2.14 0.8574701697205431d0) (cl-transforms:make-identity-rotation)))
        (des-for-plate-2 (desig:make-designator 'desig-props:location '((desig-props:right-of plate-1) (desig-props:far-from plate-1) (desig-props:for plate-2))))
        (des-for-plate-4 (desig:make-designator 'desig-props:location '((desig-props:left-of plate-3) (desig-props:far-from plate-3) (desig-props:for plate-4)))))
    (when (> number-of-plates 0)
      (prolog `(assert (object-pose ?_ plate-1 ,pose-on-table-1)))
      (when (> number-of-plates 1)
        (prolog `(assert (object-pose ?_ plate-3 ,pose-on-table-2)))
        (when (> number-of-plates 2)
          (prolog `(assign-object-pos-on plate-2 ,des-for-plate-2))
          (when (> number-of-plates 3)
            (prolog `(assign-object-pos-on plate-4 ,des-for-plate-4))))))))

(defun set-table ()
  (time
  (let ((des-for-fork-1 (make-fork-desig 'fork-1 'plate-1))
        (des-for-fork-2 (make-fork-desig 'fork-2 'plate-2))
        (des-for-fork-3 (make-fork-desig 'fork-3 'plate-3))
        (des-for-fork-4 (make-fork-desig 'fork-4 'plate-4))

        (des-for-knife-1 (make-knife-desig 'knife-1 'plate-1))
        (des-for-knife-2 (make-knife-desig 'knife-2 'plate-2))
        (des-for-knife-3 (make-knife-desig 'knife-3 'plate-3))
        (des-for-knife-4 (make-knife-desig 'knife-4 'plate-4))

        (des-for-mug-1 (make-glass-desig 'mug-1 'plate-1))
        (des-for-mug-2 (make-glass-desig 'mug-2 'plate-2))
        (des-for-mug-3 (make-glass-desig 'mug-3 'plate-3))
        (des-for-mug-4 (make-glass-desig 'mug-4 'plate-4)))
    (put-plates-on-table)
    
    (force-ll (prolog `(assign-object-pos fork-1 ,des-for-fork-1)))
    (prolog `(assign-object-pos fork-2 ,des-for-fork-2))
    (prolog `(assign-object-pos fork-3 ,des-for-fork-3))
    (prolog `(assign-object-pos fork-4 ,des-for-fork-4))
    
    (force-ll (prolog `(assign-object-pos-on knife-1 ,des-for-knife-1)))
    (force-ll (prolog `(assign-object-pos-on knife-2 ,des-for-knife-2)))
    (force-ll (prolog `(assign-object-pos-on knife-3 ,des-for-knife-3)))
    (force-ll (prolog `(assign-object-pos-on knife-4 ,des-for-knife-4)))
              
    (force-ll (prolog `(and
                        (assign-object-pos mug-1 ,des-for-mug-1)
                        (assign-object-pos mug-2 ,des-for-mug-2)
                        (assign-object-pos mug-3 ,des-for-mug-3)
                        (assign-object-pos mug-4 ,des-for-mug-4)))))))

(defun staple-plates-on-counter ()
  (prolog `(assert (object-pose ?_ ,(new-symbol "PLATE-" 1) ((0.9 1 0.86d0) (0 0 0 1)))))
  (prolog `(assert (object-pose ?_ ,(new-symbol "PLATE-" 2) ((0.9 1 0.889d0) (0 0 0 1)))))
  (prolog `(assert (object-pose ?_ ,(new-symbol "PLATE-" 3) ((0.9 1 0.91d0) (0 0 0 1))))))

(defun put-n-sets-on-table (&optional (n 1))
  (loop for i from 1 to n do
    (put-nth-set-on-counter :n i)
    (let ((plate (put-plate-from-counter-on-table)))
      (mapcar (alexandria:rcurry #'put-object-from-counter-near-plate plate)
              '(btr::fork btr::knife btr::mug)))))

(defun put-stuff-on-table ()
  ;; (put-stuff-on-counter)
  (loop for i from 1 to *num-of-sets-on-table* do
    (format t "Put PLATE from counter on table~%")
    (let ((plate (put-plate-from-counter-on-table)))
      (mapcar (alexandria:rcurry #'put-object-from-counter-near-plate plate)
              '(btr::fork btr::knife btr::mug)))))

(declaim (inline new-symbol))
(defun new-symbol (string number)
  (intern (concatenate 'string string (write-to-string number)) "SPATIAL-RELATIONS-COSTMAP"))

(defun put-nth-set-on-counter (&key (n 1) (plate t) (mug t) (fork t) (knife t))
   (and plate
        (prolog `(assert (object-pose ?_ ,(new-symbol "PLATE-" n) ((0.9 1 0.8594702033075609d0) (0 0 0 1))))))
   (and mug
        (prolog `(assert (object-pose ?_ ,(new-symbol "MUG-" n) ((0.8 1.4 0.9119799601336841d0) (0 0 0 1))))))
   (and fork
        (prolog `(assert (object-pose ?_ ,(new-symbol "FORK-" n) ((1 0.8 0.8569999588202436d0) (0 0 0 1))))))
   (and knife
        (prolog `(assert (object-pose ?_ ,(new-symbol "KNIFE-" n) ((1 0.7 0.8569999867081037d0) (0 0 0 1)))))))

(defun put-stuff-on-counter ()
  (prolog `(and
            (assert (object-pose ?_ plate-4 ((0 0 0) (0 0 0 1))))
            (assert (object-pose ?_ plate-1 ((0.88 1 0.86d0) (0 0 0 1))))
            (assert (object-pose ?_ plate-2 ((0.88 1 0.887d0) (0 0 0 1))))
            (assert (object-pose ?_ plate-3 ((0.88 1 0.914d0) (0 0 0 1))))
            (assert (object-pose ?_ plate-4 ((0.88 1 0.941d0) (0 0 0 1))))
            ;;
            (assert (object-pose ?_ fork-1 ((0.8 0.7 0.8569999588202436d0) (0 0 1 1))))
            (assert (object-pose ?_ fork-2 ((0.84 0.7 0.8569999588202436d0) (0 0 1 1))))
            (assert (object-pose ?_ fork-3 ((0.88 0.7 0.8569999588202436d0) (0 0 1 1))))
            (assert (object-pose ?_ fork-4 ((0.92 0.7 0.8569999588202436d0) (0 0 1 1))))
            ;; (assert (object-pose ?_ fork-5 ((0.96 0.7 0.8569999588202436d0) (0 0 1 1))))
            ;;
            (assert (object-pose ?_ knife-1 ((1.04 0.7 0.8569999588202436d0) (0 0 1 1))))
            (assert (object-pose ?_ knife-2 ((1.08 0.7 0.8569999588202436d0) (0 0 1 1))))
            (assert (object-pose ?_ knife-3 ((1.12 0.7 0.8569999588202436d0) (0 0 1 1))))
            (assert (object-pose ?_ knife-4 ((1 0.7 0.8569999588202436d0) (0 0 1 1))))
            ;; (assert (object-pose ?_ knife-5 ((1.2 0.7 0.8569999588202436d0) (0 0 1 1))))
            ;;
            (assert (object-pose ?_ mug-1 ((0.8 1.48 0.9119799601336841d0) (0 0 0 1))))
            (assert (object-pose ?_ mug-2 ((0.78 1.32 0.9119799601336841d0) (0 0 0 1))))
            (assert (object-pose ?_ mug-3 ((0.9 1.4 0.9119799601336841d0) (0 0 0 1))))
            (assert (object-pose ?_ mug-4 ((1 1.29 0.9119799601336841d0) (0 0 0 1))))
            ;; (assert (object-pose ?_ mug-5 ((0.98 1.47 0.9119799601336841d0) (0 0 0 1))))
            )))



(defun put-plate-from-counter-on-table ()
  (sb-ext:gc :full t)
  (let ((plate (find-object 'btr:plate "CounterTop205")))
    (sb-ext:gc :full t)
    (put-plate-from-counter-on-corresponding-place-on-table plate)
    plate))

(cpl-impl:def-top-level-plan put-plate-from-counter-on-corresponding-place-on-table (the-plate)
  ;; (format t "global shit = ~a~%" *num-of-sets-on-table*)
  ;; (format t "typep integer?: ~a~%" (typep *num-of-sets-on-table* 'integer))
  (cram-projection:with-projection-environment 
      projection-process-modules::pr2-bullet-projection-environment
    (with-designators
        ((on-kitchen-island (desig-props:location `((desig-props:on counter-top)
                                                    (desig-props:name kitchen-island)
                                                    (desig-props:context table-setting) 
                                                    (desig-props:object-count 4)
                                                    (desig-props:for ,the-plate)))))
      (plan-knowledge:achieve `(plan-knowledge:loc ,the-plate ,on-kitchen-island)))))

(cpl-impl:def-top-level-plan put-object-from-counter-on-table (the-object)
  (cram-projection:with-projection-environment 
      projection-process-modules::pr2-bullet-projection-environment
    (with-designators
        ((on-kitchen-island (desig-props:location `((desig-props:on counter-top)
                                                    (desig-props:name kitchen-island)))))
      (plan-knowledge:achieve `(plan-knowledge:loc ,the-object ,on-kitchen-island)))))



(cpl-impl:def-top-level-plan find-object (object-type counter-to-search-on)
  (cram-projection:with-projection-environment 
      projection-process-modules::pr2-bullet-projection-environment
    (with-designators
        ((the-place (desig-props:location `((desig-props:on counter-top)
                                            (desig-props:name ,counter-to-search-on))))
         (the-object (desig-props:object `((type ,object-type)
                                           (desig-props:at ,the-place)))))
      (plan-lib:perceive-object 'cram-plan-library:a the-object))))


(cpl-impl:def-top-level-plan put-object-near-plate (object-to-put
                                                                 spatial-relations
                                                                 plate-obj)
  (cram-projection:with-projection-environment 
      projection-process-modules::pr2-bullet-projection-environment
    (with-designators
        ((put-down-location (desig-props:location `(,@(loop for property in spatial-relations
                                                            collecting `(,property ,plate-obj))
                                                    (desig-props:near ,plate-obj)
                                                    (desig-props:for ,object-to-put)
                                                    (desig-props:on counter-top)))))
      (plan-knowledge:achieve `(plan-knowledge:loc ,object-to-put ,put-down-location)))))


(defun put-object-from-counter-near-plate (object-type plate-obj)
  (format t "Put ~a from counter on table near ~a~%" object-type
          (desig-prop-value plate-obj 'name))
  (sb-ext:gc :full t)
  (let ((obj (find-object object-type "CounterTop205")))
    (sb-ext:gc :full t)
    (put-object-near-plate obj
                           (ecase object-type
                             (btr::fork '(desig-props:left-of))
                             (btr::knife '(desig-props:right-of))
                             (btr::mug '(desig-props:right-of desig-props:behind)))
                           plate-obj)
    (sb-ext:gc :full t)))


;; (defun set-obj-pose (obj-name pose)
;;   (let ((obj (gethash obj-name (slot-value *current-bullet-world* 'objects))))
;;     (btr::set-object-pose obj pose)))

(declaim (inline make-fork-desig))
(defun make-fork-desig (fork-name plate-name)
  (make-designator 'location `((left-of ,plate-name) (near ,plate-name) (for ,fork-name))))

(declaim (inline make-knife-desig))
(defun make-knife-desig (knife-name plate-name)
  (desig:make-designator 'desig-props:location `((desig-props:right-of ,plate-name)
                                                 (desig-props:near ,plate-name)
                                                 (desig-props:for ,knife-name))))

(declaim (inline make-glass-desig))
(defun make-glass-desig (glass-name plate-name)
  (desig:make-designator 'desig-props:location `((desig-props:right-of ,plate-name)
                                                 (desig-props:behind ,plate-name)
                                                 (desig-props:near ,plate-name)
                                                 (desig-props:for ,glass-name))))

(def-fact-group build-test-world ()
  (<- (make-on-counter-top-desig ?desig)
    (designator location ((desig-props:on counter-top)
                          (desig-props:name kitchen-island)) ?desig))
  
  (<- (assign-object-pos ?obj-name ?desig)
    (once
     (bound ?obj-name)
     (bound ?desig)
     (bullet-world ?w)
     (desig-solutions ?desig ?solutions)
     (take 1 ?solutions ?8-solutions) 
     (btr::generate ?poses-on (btr::obj-poses-on ?obj-name ?8-solutions ?w))
     (member ?solution ?poses-on)
     (assert (object-pose ?w ?obj-name ?solution))))

  (<- (assign-object-pos-on ?obj-name ?desig)
    (once
     (bound ?obj-name)
     (bound ?desig)
     (bullet-world ?w)
     (desig-solutions ?desig ?solutions)
     ;; (format "solutions for ~a:~% ~a~%" ?obj-name ?solutions)
     (take 8 ?solutions ?8-solutions)
     (member ?solution ?8-solutions)
     (assert (btr::object-pose-on ?w ?obj-name ?solution))))

  (<- (assign-pos-on-table ?obj-name)
    (once
     (bound ?obj-name)
     (bullet-world ?w)
     (lisp-fun pose-on-table ?pose)
     (assert (object-pose ?w ?obj-name ?pose))))

  (<- (assign-pos-on-table-near-world-right-edge ?obj-name)
    (once
     (bound ?obj-name)
     (bullet-world ?w)
     (lisp-fun pose-on-table-near-world-right-edge ?pose)
     (assert (object-pose ?w ?obj-name ?pose)))))