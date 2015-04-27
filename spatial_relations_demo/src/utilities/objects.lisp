;;;
;;; Copyright (c) 2014, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :spatial-relations-demo)

(defun spawn-object (name type &optional pose)
  (var-value
   '?object-instance
   (car (prolog-?w
          (if pose
              `(equal ?pose ,pose)
              `(scenario-objects-init-pose ?pose))
          `(scenario-object-shape ,type ?shape)
          `(scenario-object-color ?_ ,type ?color)
          `(scenario-object-extra-attributes ?_ ,type ?attributes)
          `(append (object ?w ?shape ,name ?pose :mass 0.2 :color ?color) ?attributes
                   ?object-description)
          `(assert ?object-description)
          `(%object ?w ,name ?object-instance)))))

(defun kill-object (name)
  (prolog-?w `(retract (object ?w ,name))))

(defun kill-all-objects ()
  (prolog-?w `(household-object-type ?w ?obj ?type) `(retract (object ?w ?obj)) '(fail)))

(defun move-object (object-name &optional new-pose)
  (if new-pose
      (prolog-?w
        `(assert (object-pose ?w ,object-name ,new-pose)))
      (prolog-?w
        '(scenario-objects-init-pose ?pose)
        `(assert (object-pose ?w ,object-name ?pose)))))

(defun move-object-onto (object-name onto-type onto-name)
  (let* ((on-designator (make-designator 'location `((on ,onto-type)
                                                     (name ,onto-name)
                                                     (for ,object-name))))
         (location (reference on-designator)))
    (move-object object-name location)
    (simulate *current-bullet-world* 10)))

(defun object-instance (object-name)
  (var-value '?instance
             (car (prolog-?w `(%object ?w ,object-name ?instance)))))

(defun object-pose (object-name)
  (pose (object-instance object-name)))

(declaim (inline kill-object kill-all-objects move-object object-pose))


;;;;;;;;;;;;;;;;;;;; PROLOG ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(def-fact-group spatial-relations-demo-utilities ()
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

