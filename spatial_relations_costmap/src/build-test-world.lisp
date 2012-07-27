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
                               ;;                 ((-2.15 3.44 0.945) (0 0 0 1)) ; y = 2.14
                               ;;                 :mesh pot :mass 0.2))
                               ;; (assert (object ?w mesh bowl-1 ((1.2 2 2.0) (0 0 0 1))
                               ;;                 :mesh bowl :mass 0.2))
                               ;; (assert (object ?w mesh mondamin-1 ((1.2 2 2.5) (0 0 0 1))
                               ;;                 :mesh mondamin :mass 0.2 :color (1 1 0)))
                               
                               (assert (object ?w mesh plate-1 ((0 0 0) (0 0 0 1))
                                               :mesh plate :mass 0.2 :color (0 1 1)))
                               (assert (object ?w mesh plate-2 ((0 0 0) (0 0 0 1))
                                               :mesh plate :mass 0.2 :color (0 1 1)))
                               (assert (object ?w mesh plate-3 ((0 0 0) (0 0 0 1))
                                               :mesh plate :mass 0.2 :color (0 1 1)))
                               (assert (object ?w mesh plate-4 ((0 0 0) (0 0 0 1))
                                               :mesh plate :mass 0.2 :color (0 1 1)))

                               (assert (object ?w mesh mug-1 ((0 0 0) (0 0 0 1))
                                               :mesh mug :mass 0.2 :color (1 0.5 0)))
                               (assert (object ?w mesh mug-2 ((0 0 0) (0 0 0 1))
                                               :mesh mug :mass 0.2 :color (1 0.5 0)))
                               (assert (object ?w mesh mug-3 ((0 0 0) (0 0 0 1))
                                               :mesh mug :mass 0.2 :color (1 0.5 0)))
                               (assert (object ?w mesh mug-4 ((0 0 0) (0 0 0 1))
                                               :mesh mug :mass 0.2 :color (1 0.5 0)))
                               
                               (assert (object ?w cutlery fork-1 ((0 0 0) (0 0 0 1))
                                         :color (1 0 1) :mass 0.2 :cutlery-type fork))
                               (assert (object ?w cutlery fork-2 ((0 0 0) (0 0 0 1))
                                         :color (1 0 1) :mass 0.2 :cutlery-type fork))
                               (assert (object ?w cutlery fork-3 ((0 0 0) (0 0 0 1))
                                         :color (1 0 1) :mass 0.2 :cutlery-type fork))
                               (assert (object ?w cutlery fork-4 ((0 0 0) (0 0 0 1))
                                         :color (1 0 1) :mass 0.2 :cutlery-type fork))
                               
                               (assert (object ?w cutlery knife-1 ((0 0 0) (0 0 0 1))
                                         :color (1 0 0) :mass 0.2 :cutlery-type knife))
                               (assert (object ?w cutlery knife-2 ((0 0 0) (0 0 0 1))
                                         :color (1 0 0) :mass 0.2 :cutlery-type knife))
                               (assert (object ?w cutlery knife-3 ((0 0 0) (0 0 0 1))
                                         :color (1 0 0) :mass 0.2 :cutlery-type knife))
                               (assert (object ?w cutlery knife-4 ((0 0 0) (0 0 0 1))
                                         :color (1 0 0) :mass 0.2 :cutlery-type knife))

                               ;; TODO: use a loop instead of copy-pasting
                               ;; (make-on-counter-top-desig ?des1)
                               ;; (make-on-counter-top-desig ?des2)
                               ;; (make-on-counter-top-desig ?des3)
                               ;; (make-on-counter-top-desig ?des4)
                               ;; (make-on-counter-top-desig ?des5)
                               ;; (make-on-counter-top-desig ?des6)
                               ;; (make-on-counter-top-desig ?des7)

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

(defun put-first-set-on-counter (&key (plate t) (mug t) (fork t) (knife t))
  (when plate
    (prolog `(assert (object-pose ?_ plate-1 ((0.9 1 0.8594702033075609d0) (0 0 0 1))))))
  (when mug
    (prolog `(assert (object-pose ?_ mug-1 ((1 1.4 0.9119799601336841d0) (0 0 0 1))))))
  (when fork
    (prolog `(assert (object-pose ?_ fork-1 ((1 0.8 0.8569999588202436d0) (0 0 0 1))))))
  (when knife
    (prolog `(assert (object-pose ?_ knife-1 ((1 0.7 0.8569999867081037d0) (0 0 0 1)))))))
    
#|

(sb-ext:gc :full t)

(top-level
 (cram-projection:with-projection-environment 
     projection-process-modules::pr2-bullet-projection-environment
   (with-designators
       ((on-counter (location `((desig-props:on counter-top) (desig-props:name "CounterTop205"))))
        (on-kitchen-island (location `((desig-props:on counter-top) (desig-props:name kitchen-island))))
        (plate (object `((type btr:plate) (desig-props:at ,on-counter)))))
     (achieve `(loc ,plate ,on-kitchen-island)))))



(setf object (top-level
                         (cram-projection:with-projection-environment 
                             projection-process-modules::pr2-bullet-projection-environment
                           (with-designators
                               ((on-kitchen-island (location `((desig-props:on counter-top) (desig-props:name kitchen-island))))
                                (plate (object `((type btr:plate) (desig-props:at ,on-kitchen-island)))))
                             (perceive-object 'a plate)))))
(sb-ext:gc :full t)

(top-level
 (cram-projection:with-projection-environment 
     projection-process-modules::pr2-bullet-projection-environment
   (with-designators
       ((on-counter (location `((desig-props:on counter-top) (desig-props:name "CounterTop205")))) 
        (on-kitchen-island (location `((desig-props:on counter-top) (desig-props:name kitchen-island))))
        (knife (object `((type btr:knife) (desig-props:at ,on-counter))))
        (put-down-location (location `((desig-props:right-of ,object) 
                                       (desig-props:near ,object) 
                                       (desig-props:for ,knife) 
                                       (desig-props:on counter-top)))))
     (achieve `(loc ,knife ,put-down-location)))))

(sb-ext:gc :full t)

(top-level
 (cram-projection:with-projection-environment 
     projection-process-modules::pr2-bullet-projection-environment
   (with-designators
       ((on-counter (location `((desig-props:on counter-top) (desig-props:name "CounterTop205")))) 
        (on-kitchen-island (location `((desig-props:on counter-top) (desig-props:name kitchen-island))))
        (fork (object `((type btr:fork) (desig-props:at ,on-counter))))
        (put-down-location (location `((desig-props:left-of ,object) 
                                       (desig-props:near ,object) 
                                       (desig-props:for ,fork) 
                                       (desig-props:on counter-top)))))
     (achieve `(loc ,fork ,put-down-location)))))

(sb-ext:gc :full t)

(sb-ext:gc :full t)
|#

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