;;; Copyright (c) 2014, Gayane Kazhoyan <kazhoyan@in.tum.de>
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

(in-package :spatial-relations-demo)

(defmethod parameterize-demo ((demo-name (eql :pancake-making)))
  (setf *demo-object-types*
        '((:main . (:spatula :mondamin))
          (:non-mesh . (:pancake-maker))))
  (setf *demo-objects-initial-poses*
        '((:pancake-maker ((-1.5 -1.05 0.765) (0 0 0 1)))
          (:spatula
           ((-0.9215973409016927d0 -1.0200214258829752d0 0.73530556233723958d0)
            ;; (0.0d0 0.0d0 0.009999499814012982d0 0.9999500037519226d0)
            (0 0 0.7071 0.7071))
           ((-1.9215973409016927d0 -1.05d0 0.73530556233723958d0)
            ;; (0.0d0 0.0d0 0.019996000752833393d0 0.9998000599889424d0)
            (0 0 0.7071 0.7071)))
          (:mondamin ((-0.77d0 0.7d0 0.9586666742960612d0) (0.0d0 0.0d0 0.0d0 0.99d0))))))

(defmethod execute-demo ((demo-name (eql :pancake-making)) &key set)
  (declare (ignore set))
  (with-projection-environment pr2-bullet-projection-environment
    (top-level
      (let ((pancake-maker-designator
              (find-object-on-counter :pancake-maker "Cupboard" "pancake_table")))
        (let ((spatula-designator
                (find-object-on-counter :spatula "Cupboard"
                                        "pancake_table")))
          (with-designators
              ((spatula-location :location `((:on "Cupboard")
                                             (:name "pancake_table")
                                             ;; (:centered-with-padding 0.6)
                                             (:for ,spatula-designator)
                                             (:right-of ,pancake-maker-designator)
                                             (:near ,pancake-maker-designator))))
            (format t "now trying to achieve the location of spatula on kitchen-island~%")
            (achieve `(loc ,spatula-designator ,spatula-location))))
        (let ((mondamin-designator
                (find-object-on-counter :mondamin "Cupboard"
                                        "kitchen_island")))
          (with-designators
              ((on-kitchen-island :location `((:on "Cupboard")
                                              (:name "pancake_table")
                                              ;; (:centered-with-padding 0.35)
                                              (:for ,mondamin-designator)
                                              (:right-of ,pancake-maker-designator)
                                              (:far-from ,pancake-maker-designator))))
            (format t "now trying to achieve the location of mondamin on kitchen-island~%")
            (achieve `(loc ,mondamin-designator ,on-kitchen-island))))
        (let ((spatula-2-designator
                (find-object-on-counter :spatula "Cupboard"
                                        "pancake_table")))
          (with-designators
              ((spatula-location :location `((:on "Cupboard")
                                             (:name "pancake_table")
                                             (:centered-with-padding 0.6)
                                             (:for ,spatula-2-designator)
                                             (:left-of ,pancake-maker-designator)
                                             (:near ,pancake-maker-designator))))
            (format t "now trying to achieve the location of spatula on kitchen-island~%")
            (achieve `(loc ,spatula-2-designator ,spatula-location))))))))

(def-top-level-cram-function put-down-object-in-hand ()
  (with-designators
      ((on-cupboard :location `((:on "Cupboard"))))
    (let ((obj-in-hand (cut:var-value '?obj (car (prolog '(object-in-hand ?obj))))))
      (with-retry-counters ((navigation-retries 10))
        (with-failure-handling
            ((manipulation-pose-unreachable (f)
               (declare (ignore f))
               (format t "Could not reach target pose.~%")
               (do-retry navigation-retries
                 (format t "Re-positioning base.~%")
                 (retry-with-updated-location
                  on-cupboard
                  (next-different-location-solution on-cupboard)))))
          (achieve `(object-placed-at ,obj-in-hand ,on-cupboard)))))))

(def-top-level-cram-function achieve-mondamin-in-hand ()
  (with-designators
      ((on-cupboard :location '((:on "Cupboard")))
       (mondamin-on-cupboard :object `((:at ,on-cupboard) (:type :mondamin))))
    (with-retry-counters ((perception-retries 100))
      (with-failure-handling
          ((object-not-found (f)
             (declare (ignore f))
             (format t "Object ~a was not found.~%" mondamin-on-cupboard)
             (do-retry perception-retries
               (format t "Re-perceiving object.~%")
               (plan-lib::retry-with-updated-location
                on-cupboard (plan-lib::next-different-location-solution on-cupboard)))))
        (perceive-object :a mondamin-on-cupboard)))
    (format t "Now trying to achieve mondamin object in hand.~%")
    (achieve `(object-in-hand ,mondamin-on-cupboard))
    (format t "Object is in hand.~%")))

(def-top-level-cram-function achieve-hand-with-mondamin-above-maker ()
  (with-designators
      ((on-pancake-table :location `((:on "Cupboard")
                                     (:name "pancake_table")))
       (pancake-maker :object `((:at ,on-pancake-table)
                                (:type :pancake-maker)))

       (location-in-gripper :location `((:in :gripper)))
       (pouring-container :object `((:at ,location-in-gripper)))

       (pouring-target-location :location `((:on ,pancake-maker))))
    (reference-object pouring-container)
    (perceive-object :a pancake-maker)
    (format t "now trying to achieve hand above maker~%")
    (with-retry-counters ((navigation-retries 10))
      (with-failure-handling
          ((manipulation-pose-unreachable (f)
             (declare (ignore f))
             (format t "Could not reach target pose.~%")
             (do-retry navigation-retries
               (format t "Re-positioning base.~%")
               (retry))))
        (achieve `(stuff-poured-at ,pouring-container ,pouring-target-location))))))

(def-top-level-cram-function pick-up-and-pour ()
  (with-designators
      ((on-counter-top :location '((:on "Cupboard")))
       (mondamin-bottle :object `((:at ,on-counter-top) (:type :mondamin)))
       (pancake-maker :object `((:at ,on-counter-top) (:type :pancake-maker)))
       (pouring-target-location :location `((:on ,pancake-maker))))
    (with-retry-counters ((perception-retries 100))
      (with-failure-handling
          ((object-not-found (f)
             (declare (ignore f))
             (format t "Object ~a was not found.~%" mondamin-bottle)
             (do-retry perception-retries
               (format t "Re-perceiving object.~%")
               (plan-lib:retry-with-updated-location
                on-counter-top
                (plan-lib:next-different-location-solution on-counter-top)))))
        (perceive-object :a mondamin-bottle)))
    (format t "Now trying to achieve mondamin object in hand.~%")
    (format t "BEFORE SLEEP~%~%")
    (sleep* 10)
    (achieve `(object-in-hand ,mondamin-bottle))
    (format t "Object is in hand.~%")
    (format t "BEFORE SLEEP~%~%")
    (sleep* 10)
    (with-retry-counters ((perception-retries 100))
      (with-failure-handling
          ((object-not-found (f)
             (declare (ignore f))
             (format t "Object ~a was not found.~%" pancake-maker)
             (do-retry perception-retries
               (format t "Re-perceiving object.~%")
               (plan-lib:retry-with-updated-location
                on-counter-top
                (plan-lib:next-different-location-solution on-counter-top)))))
        (perceive-object :a pancake-maker)))
    (format t "now trying to pour above pancake maker~%")
    (format t "BEFORE SLEEP~%~%")
    (sleep* 10)
    (reference pouring-target-location)
    (format t "this was the destination~%")
    (with-retry-counters ((navigation-retries 10))
      (with-failure-handling
          ((manipulation-pose-unreachable (f)
             (declare (ignore f))
             (format t "Could not reach target pose.~%")
             (do-retry navigation-retries
               (format t "Re-positioning base.~%")
               (retry))))
        (achieve `(stuff-poured-at ,mondamin-bottle ,pouring-target-location))))
    (format t "BEFORE SLEEP~%~%")
    (sleep* 10)
    (with-retry-counters ((navigation-retries 10))
      (with-failure-handling
          ((manipulation-pose-unreachable (f)
             (declare (ignore f))
             (format t "Could not reach target pose.~%")
             (do-retry navigation-retries
               (format t "Re-positioning base.~%")
               (retry-with-updated-location
                on-counter-top
                (next-different-location-solution on-counter-top)))))
        (achieve `(object-placed-at ,mondamin-bottle ,on-counter-top))))))


(defun reference-object (object-designator)
  (when (eql (desig-prop-value
              (desig-prop-value object-designator
               :at)
              :in) :gripper)
    (let ((new-desig
            (newest-effective-designator
             (cut:var-value '?obj (car (prolog '(object-in-hand ?obj)))))))
      (unless (is-var new-desig)
        (unless (desig-equal object-designator new-desig)
          (equate object-designator new-desig))
        new-desig))))

(def-goal (achieve (stuff-poured-at ?container ?loc))
  (roslisp:ros-info (achieve plan-lib) "(achieve (stuff-poured-at))")
  (let ((obj (current-desig ?container)))
    (assert
     (cram-occasions-events:holds `(object-in-hand ,obj)) ()
     "The container `~a' needs to be in the hand before being able to pour from it."
     obj)
    (with-retry-counters ((goal-pose-retries 3)
                          (manipulation-retries 6))
      (with-failure-handling
          ((manipulation-failure (f)
             (declare (ignore f))
             (roslisp:ros-warn
              (achieve plan-lib)
              "Got unreachable putdown pose. Trying different put-down location")
             (do-retry goal-pose-retries
               (plan-lib::retry-with-updated-location
                ?loc (plan-lib::next-different-location-solution ?loc)))))
        (with-designators ((pouring-loc :location `((:to :reach) (:location ,?loc))))
          (plan-lib::reset-counter manipulation-retries)
          (with-failure-handling
              ((manipulation-failure (f)
                 (declare (ignore f))
                 (roslisp:ros-warn
                  (achieve plan-lib)
                  "Got unreachable pouring pose. Trying alternatives.")
                 (plan-lib::do-retry manipulation-retries
                   (plan-lib::retry-with-updated-location
                    pouring-loc
                    (plan-lib::next-different-location-solution pouring-loc)))))
            (plan-lib::try-reference-location pouring-loc)
            (at-location (pouring-loc)
              (achieve `(stuff-poured ,?container ,?loc)))))))))

(def-goal (achieve (stuff-poured ?container ?loc))
  (roslisp:ros-info (achieve plan-lib) "(achieve (stuff-poured))")
  (let ((container (current-desig ?container)))
    (assert
     (cram-occasions-events:holds `(object-in-hand ,container)) ()
     "The container `~a' needs to be in the hand before being able to pour from it."
     container)
    (with-designators ((pour-action
                        :action `((:type :trajectory) (:to :pour)
                                  (:container ,container) (:at ,?loc)))
                       (park-action
                        :action `((:type :trajectory) (:to :park) (:obj ,container))))
      (with-failure-handling
          (((or manipulation-failed manipulation-pose-unreachable) (f)
             (declare (ignore f))
             ;; Park arm first
             (perform park-action)
             (monitor-action park-action)
             (roslisp:ros-warn
              (achieve plan-lib)
              "Got unreachable putdown pose.")
             (cpl:fail 'manipulation-pose-unreachable)))
        (cram-plan-library::try-reference-location ?loc)
        (achieve `(looking-at ,(reference ?loc)))
        (perform pour-action)
        (monitor-action pour-action))
      (with-retry-counters ((park-retry-counter 5))
        (with-failure-handling
            ((manipulation-failure (f)
               (declare (ignore f))
               (roslisp:ros-warn (achieve plan-lib) "Unable to park.")
               (do-retry park-retry-counter
                 (roslisp:ros-warn (achieve plan-lib) "Retrying.")
                 (retry))
               (cpl:fail 'manipulation-pose-unreachable)))
          (perform park-action)
          (monitor-action park-action))))))


(def-fact-group manipulations (object-type-grasp orientation-matters)
  ;; (<- (object-type-grasp :blue-metal-plate :front (:left)))

  ;; (<- (object-type-grasp :knife-plastic :top (:right)))

  ;; (<- (object-type-grasp :cup :front (:right)))
  ;; (<- (object-type-grasp :cup :front (:left)))

  (<- (orientation-matters ?object-designator)
    (lisp-fun desig:current-desig ?object-designator ?current-object-designator)
    (desig:desig-prop ?current-object-designator (:type :spatula))))
