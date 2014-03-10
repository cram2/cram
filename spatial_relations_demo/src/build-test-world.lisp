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

(disable-location-validation-function 'btr-desig::validate-designator-solution)
(disable-location-validation-function 'btr-desig::check-ik-solution)

;; run bullet launch files (bullet_reasoning_demo)
;; ros-load-system btr-demo

;; TODO: comments
;; TODO: declares everywhere!

(defvar *bdgs* nil)
(defparameter *num-of-sets-on-table* 2)

(defvar *items* (make-hash-table :test 'equal)
  "TYPE -> color")
(setf (gethash "PLATE" *items*) '(0.8 0.58 0.35))
(setf (gethash "FORK" *items*) '(0.2 0.1 0.3))
(setf (gethash "KNIFE" *items*) '(0.5 0 0))
(setf (gethash "MUG" *items*) '(0.8 0.3 0))

;; TODO collision detection for knives!!!

(defun start-ros-and-bullet ()
  (setf *bdgs* nil)
  (roslisp-utilities:startup-ros :anonymous nil)
  (let ((urdf (cl-urdf:parse-urdf (roslisp:get-param "robot_description_lowres")))
        (kitchen-urdf (cl-urdf:parse-urdf (roslisp:get-param "kitchen_description")))
        (pi-rotation '(0 0 1 0)))
    (setf *bdgs*
          (car
           (force-ll
            (prolog
             `(and
               (clear-bullet-world)
               (bullet-world ?w)
               (debug-window ?w)
               (robot ?robot)
               (assert (object ?w btr::static-plane floor ((0 0 0.001) (0 0 0 1))
                               :normal (0 0 1) :constant 0 :disable-collisions-with (?robot)))
               (assert (object ?w btr::semantic-map my-kitchen ((-3.45 -4.35 0) ,pi-rotation)
                               :urdf ,kitchen-urdf))
               (assert (object ?w urdf ?robot ((0 0 0) (0 0 0 1)) :urdf ,urdf))
               (robot-arms-parking-joint-states ?joint-states)
               (assert (joint-state ?w ?robot ?joint-states))
               (assert (joint-state ?w ?robot (("torso_lift_joint" 0.33))))
               (assert (object ?w btr::cylinder oven-1 ((-1.0 1.26 0.8883) (0 0 0 1))
                               :mass 0.2 :color (0 0 0) :size (0.3 0.3 0.07))))))))))

;; (setf sem-map (var-value '?sem-map (lazy-car (prolog `(%object ?w my-kitchen ?sem-map) *bdgs*))))

;; ;; desig for a glass near a plate
;; (setf des (desig:make-designator 'desig-props:location '((desig-props:right-of plate-1) (desig-props:behind plate-1) (desig-props:near plate-1) (desig-props:for mug-1) (desig-props:on counter-top) (desig-props:name kitchen-island))))

;; desig for a plate on a table
;; (setf des (desig:make-designator 'desig-props:location '((desig-props:on counter-top) (desig-props:name kitchen-island) (desig-props:context table-setting) (desig-props:for plate-1) (desig-props:object-count 4))))

;; ;; desig for a fork to the left of plate
;; (setf des (desig:make-designator 'desig-props:location '((desig-props:left-of plate-1)  (desig-props:near plate-1) (desig-props:for fork-1))))
;; (force-ll (prolog `(and (symbol-value des ?des) (assign-object-pos fork-1 ?des))))


(declaim (inline new-symbol-with-id))
(defun new-symbol-with-id (string number)
  (intern (concatenate 'string (string-upcase string) "-" (write-to-string number)) "SPATIAL-RELATIONS-COSTMAP"))

(declaim (inline new-symbol-from-strings))
(defun new-symbol-from-strings (&rest strings)
  (intern (string-upcase (reduce (alexandria:curry #'concatenate 'string) strings)) "SPATIAL-RELATIONS-COSTMAP"))


(defun spawn-stuff ()
  (loop for object-type being the hash-keys of *items*
        using (hash-value color)
        do (prolog
            `(and
              (bullet-world ?w)
              ,@(loop for i from 1 to *num-of-sets-on-table*
                      collect `(assert (object
                                        ?w mesh ,(new-symbol-with-id object-type i)
                                        ((2 0 0) (0 0 0 1))
                                        :mesh ,(intern (string-upcase object-type))
                                        :mass 0.2
                                        :color ,color)))))))

(defun kill-stuff ()
  (loop for object-type being the hash-keys of *items*
        using (hash-value color)
        do (prolog
            `(and (bullet-world ?w)
                  ,@(loop for i from 1 to *num-of-sets-on-table*
                          collect `(retract (object ?w ,(new-symbol-with-id object-type i))))))))

(defun move-object (object-name new-pose)
  (prolog `(and
            (bullet-world ?w)
            (assert (object-pose ?w ,object-name ,new-pose)))))

(defun put-stuff-away ()
  (loop for object-type being the hash-keys of *items*
        do (loop for i from 1 to *num-of-sets-on-table*
                 do (move-object (new-symbol-with-id object-type i)
                                 '((2.0 0 0) (0 0 0 1)))))
  (mapcar (alexandria:rcurry #'move-object '((2.0 0 0) (0 0 0 1)))
          '(pot-1 bowl-1 bowl-2 bowl-3 bowl-4 mondamin-1 mondamin-2)))

(defun put-noise-on-table ()
  "For trying out on cluttered scenes"
  (prolog `(and (bullet-world ?w)
                (assert (object ?w mesh pot-1 ((2.0 0 0) (0 0 0 1))
                                :mesh pot :mass 0.2 :color (0.1 0.2 0.3)))
                (assert (object ?w mesh bowl-1 ((2.0 0 0) (0 0 0 1))
                                :mesh bowl :mass 0.2 :color (0 0.3 0)))
                (assert (object ?w mesh bowl-2 ((2.0 0 0) (0 0 0 1))
                                :mesh bowl :mass 0.2 :color (0 0.3 0)))
                (assert (object ?w mesh bowl-3 ((2.0 0 0) (0 0 0 1))
                                :mesh bowl :mass 0.2 :color (0 0.3 0)))
                (assert (object ?w mesh bowl-4 ((2.0 0 0) (0 0 0 1))
                                :mesh bowl :mass 0.2 :color (0 0.3 0)))
                (assert (object ?w mesh mondamin-1 ((2.0 0 0) (0 0 0 1))
                                :mesh mondamin :mass 0.2 :color (0.5 0.1 0)))
                (assert (object ?w mesh mondamin-2 ((2.0 0 0) (0 0 0 1))
                                :mesh mondamin :mass 0.2 :color (0.5 0.1 0)))))

  (mapcar #'move-object '(pot-1 bowl-1 mondamin-1 mondamin-2 bowl-2 bowl-3 bowl-4)
          '(((-2.0 1.65 0.9413339835685429d0) (0 0 0 1))
            ((-1.9 1.95 0.8911207699875103d0) (0 0 0 1))
            ((-2.0 1.06 0.9573383588498887d0) (0 0 0 1))
            ((-2.0 2.1 0.9573383588498887d0) (0 0 0 1))
            ((-1.9 1.3 0.8911207699875103d0) (0 0 0 1))
            ((-2.0 2.36 0.8911207699875103d0) (0 0 0 1))
            ((-1.95 1.16 0.8911207699875103d0) (0 0 0 1))))

  ;; (simulate *current-bullet-world* 50)
  )

(defun put-stuff-on-counter ()
  (spawn-stuff)
  (put-stuff-away)

  (loop for i from 1 to *num-of-sets-on-table*
        for plate-coord = 0.86 then (+ plate-coord 0.027)
        ;; for plate-coord = 0.96 then (+ plate-coord 0.027)
        for fork-coord = 1.35 then (+ fork-coord 0.05)
        for knife-coord = (+ fork-coord (* (+ *num-of-sets-on-table* 1) 0.04))
        do (move-object (new-symbol-with-id "PLATE" i)
                        `((1.45 0.8 ,plate-coord) (0 0 0 1)))
           (move-object (new-symbol-with-id "FORK" i)
                        `((,fork-coord 0.5 0.865) (0 0 1 1)))
           (move-object (new-symbol-with-id "KNIFE" i)
                        `((,knife-coord 0.5 0.857) (0 0 1 1))))

  (move-object 'mug-1 '((1.5 1.08 0.9119799601336841d0) (0 0 0 1)))
  (move-object 'mug-3 '((1.65 1.02 0.9119799601336841d0) (0 0 0 1)))
  (move-object 'mug-2 '((1.35 1.11 0.9119799601336841d0) (0 0 0 1)))
  (move-object 'mug-4 '((1.55 1.19 0.9119799601336841d0) (0 0 0 1)))
  (move-object 'mug-5 '((1.5 1.17 0.9119799601336841d0) (0 0 0 1))))

;;;;;;;;;;;;;;;;;;;;;;;;; ONLY DESIGS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun put-plates-on-table-with-far (&optional (number-of-plates *num-of-sets-on-table*))
  (spawn-stuff)
  (cpl-impl:top-level
    (cram-language-designator-support:with-designators
       ((des-for-plate-2 (location `((right-of plate-1) (far-from plate-1)
                                                        (for plate-2))))
        (des-for-plate-4 (location `((left-of plate-3) (far-from plate-3)
                                                       (for plate-4)))))
     (when (> number-of-plates 0)
       (prolog `(assert (object-pose ?_ plate-1 ((-1.5 1.84 0.85747d0) (0 0 0 1)))))
       (when (> number-of-plates 1)
         (prolog `(assert (object-pose ?_ plate-3 ((-1.0 1.84 0.85747d0) (0 0 0 1)))))
         (when (> number-of-plates 2)
           (prolog `(assign-object-pos-on plate-2 ,des-for-plate-2))
           (when (> number-of-plates 3)
             (prolog `(assign-object-pos-on plate-4 ,des-for-plate-4)))))))))

(defun make-plate-desig (plate-id &optional (counter-name "kitchen_island")
                                    (plate-num *num-of-sets-on-table*))
  (let ((plate-name (new-symbol-with-id "PLATE" plate-id)))
    (make-designator 'location `((on "Cupboard") (name ,counter-name)
                                 (for ,plate-name) (context table-setting)
                                 (object-count ,plate-num)))))

(defun make-object-near-plate-desig (object-type object-id &optional (plate-id object-id))
  (let ((plate-name (new-symbol-with-id "PLATE" plate-id))
        (object-name (new-symbol-with-id object-type object-id)))
    (make-designator 'location (append (string-case object-type
                                         ("FORK" `((desig-props:left-of ,plate-name)))
                                         ("KNIFE" `((desig-props:right-of ,plate-name)))
                                         ("MUG" `((desig-props:right-of ,plate-name)
                                                  (desig-props:behind ,plate-name))))
                                       `((desig-props:near ,plate-name)
                                         (desig-props:for ,object-name))))))

(defun assign-multiple-obj-pos (object-type &optional (object-number *num-of-sets-on-table*))
  (prolog `(and ,@(loop for i from 1 to object-number collect
                        `(assign-object-pos ,(new-symbol-with-id object-type i)
                                            ,(string-case object-type
                                               ("PLATE" (make-plate-desig i))
                                               (t (make-object-near-plate-desig object-type i))))))))

(defun set-table-without-robot ()
  (spawn-stuff)
  (put-stuff-away)
  (time
   (loop for object-type being the hash-keys of *items*
         do (assign-multiple-obj-pos object-type))))

(def-fact-group build-test-world ()
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
     (take 8 ?solutions ?8-solutions)
     (member ?solution ?8-solutions)
     (assert (btr::object-pose-on ?w ?obj-name ?solution)))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; PROJECTION ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(cpl-impl:def-cram-function find-object-on-counter (object-type counter-name)
  "Returns an object designator."
  (cram-language-designator-support:with-designators
      ((on-counter (desig-props:location `((desig-props:on "Cupboard")
                                           (desig-props:name ,counter-name))))
       (the-object (desig-props:object `((desig-props:type ,object-type)
                                         (desig-props:at ,on-counter)))))
    (reference on-counter)
    (format t "trying to perceive an object ~a~%" the-object)
    (plan-lib:perceive-object 'cram-plan-library:a the-object)))

(cpl-impl:def-cram-function put-plate-on-table (plate-obj-desig)
  (cram-language-designator-support:with-designators
      ((on-kitchen-island (location `((on "Cupboard")
                                      (name "kitchen_island")
                                      (for ,plate-obj-desig) (context table-setting) 
                                      (object-count ,*num-of-sets-on-table*)))))
    (format t "now trying to achieve the location of plate on kitchen-island~%")
    (plan-knowledge:achieve `(plan-knowledge:loc ,plate-obj-desig ,on-kitchen-island))))

(cpl-impl:def-cram-function put-plate-from-counter-on-table ()
  (sb-ext:gc :full t)
  (format t "Put a PLATE from counter on table~%")
  (let ((plate (find-object-on-counter 'btr:plate "kitchen_sink_block")))
    (sb-ext:gc :full t)
    (put-plate-on-table plate)
    plate))

(cpl-impl:def-cram-function put-object-from-counter-on-table (obj-type)
  (sb-ext:gc :full t)
  (format t "Put a PLATE from counter on table~%")
  (let ((object (find-object-on-counter obj-type "kitchen_sink_block")))
    (sb-ext:gc :full t)
    (put-plate-on-table object)
    object))

(cpl-impl:def-cram-function put-object-near-plate (object-to-put plate-obj
                                                   spatial-relations)
  (cram-language-designator-support:with-designators
      ((put-down-location (location `(,@(loop for property in spatial-relations
                                              collecting `(,property ,plate-obj))
                                      (near ,plate-obj) (for ,object-to-put)
                                      (on "Cupboard")))))
    (plan-knowledge:achieve `(plan-knowledge:loc ,object-to-put ,put-down-location))))

(cpl-impl:def-cram-function put-object-from-counter-near-plate (object-type plate-obj)
  (format t "Put ~a from counter on table near ~a~%"
          object-type (desig-prop-value plate-obj 'name))
  (sb-ext:gc :full t)
  (let ((obj (find-object-on-counter object-type "kitchen_sink_block")))
    (sb-ext:gc :full t)
    (put-object-near-plate obj plate-obj
                           (ecase object-type
                             (btr::fork '(desig-props:left-of))
                             (btr::knife '(desig-props:right-of))
                             (btr::mug '(desig-props:right-of desig-props:behind))))
    (sb-ext:gc :full t)))

(cpl-impl:def-top-level-cram-function put-stuff-on-table ()
  (cram-projection:with-projection-environment
      projection-process-modules::pr2-bullet-projection-environment
  (loop for i from 1 to *num-of-sets-on-table* do
    (let ((plate (put-plate-from-counter-on-table)))
      (mapcar (alexandria:rcurry #'put-object-from-counter-near-plate plate)
              '(btr::fork btr::knife btr::mug))))))


(defun set-table-in-projection ()
  (put-stuff-on-counter)
  (put-stuff-on-table))

(defun teleport-a-plate ()
  (prolog `(and (assert (object-pose ?_ plate-1 ((-1.2 1.14 0.85747016972d0) (0 0 0 1)))))))

(defun bring-robot-to-table ()
  (put-stuff-away)
  (prolog `(and
            (assert (object-pose ?_ plate-1 ((-2.2 2.14 0.85747016972d0) (0 0 0 1))))
            (assert (object-pose ?_ plate-2 ((-1.75 2.14 0.85747016972d0) (0 0 0 1))))
            (assert (object-pose ?_ plate-3 ((-2.2 1.34 0.85747016972d0) (0 0 0 1))))
            (assert (object-pose ?_ plate-4 ((-1.75 1.34 2.85747016972d0) (0 0 0 1))))))
  (put-stuff-on-table))

(defun pick-and-place (type)
  (let ((obj-id (gensym)))
    (format t "id: ~a~%" obj-id)
    (prolog `(and (bullet-world ?w)
                  (robot ?robot)
                  (assert (object ?w mesh ,obj-id
                                  ((2 0 0) (0 0 0 1))
                                  :mesh ,type :mass 0.2 :color (0.8 0.3 0)
                                  :disable-collisions-with (?robot)
                                  ))))
    (move-object obj-id '((1.3 0.8 1.0) (0 0 0 1)))

    (cpl-impl:top-level
      (cram-projection:with-projection-environment
          projection-process-modules::pr2-bullet-projection-environment
        (let ((obj (put-object-from-counter-on-table type)))
          obj)))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;; PANCAKES! ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun spawn-pancake-scenario ()
  (prolog `(and (bullet-world ?w) (assert (object ?w btr::mesh spatula-2
                                                  ((1.4 1.08 0.9119799601336841d0) (0 0 0 1))
                                                  :mesh btr::spatula :mass 0.2 :color (0 0 0)))))
  (prolog `(and (bullet-world ?w) (assert (object ?w btr::mesh mondamin-1
                                                  ((1.35 1.11 0.9119799601336841d0) (0 0 0 1))
                                                  :mesh mondamin :mass 0.2 :color (0.5 0.1 0)))))
  (move-object 'spatula-2 `((1.5 0.8 0.86) (0.0d0 0.0d0 0.19611613794814378d0 0.9805806751289282d0)))
  (move-object 'mondamin-1 `((1.35 1.11 0.958) (0 0 0 1))))

(defun execute-pancake-scenario ()
  (spawn-pancake-scenario)
  (cram-projection:with-projection-environment
      projection-process-modules::pr2-bullet-projection-environment
    (cpl-impl:top-level
      (let ((mondamin-designator (find-object-on-counter 'btr::mondamin "kitchen_sink_block")))
        (cram-language-designator-support:with-designators
            ((on-kitchen-island (location `((on "Cupboard")
                                            (name "kitchen_island")
                                            (for ,mondamin-designator)
                                            (right-of oven-1)
                                            (near oven-1)
                                            (behind oven-1)))))
          (format t "now trying to achieve the location of mondamin on kitchen-island~%")
          (plan-knowledge:achieve `(plan-knowledge:loc ,mondamin-designator ,on-kitchen-island)))
        (let ((spatula-designator (find-object-on-counter 'btr::spatula "kitchen_sink_block")))
          (cram-language-designator-support:with-designators
              ((spatula-location (location `((on "Cupboard")
                                             (name "kitchen_island")
                                             (for ,spatula-designator)
                                             (right-of oven-1)
                                             (near oven-1)
                                             (in-front-of oven-1)))))
            (format t "now trying to achieve the location of spatula on kitchen-island~%")
            (plan-knowledge:achieve `(plan-knowledge:loc ,spatula-designator ,spatula-location))))))))
