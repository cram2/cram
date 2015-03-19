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

(defparameter *init-pose* '((2 0 0) (0 0 0 1)) "Pose where objects are spawned.")
(defparameter *default-color* '(0.5 0.5 0.5) "Default color to assign to object meshes.")

(defstruct item
  type
  (color *default-color*)
  (pose-stored *init-pose*)
  (pose-used *init-pose*))

(defun items-slots-hash-table (slots-lists)
  "A list of lists turns into a hash-table. See *item-types* for example."
  (let ((table (make-hash-table :test #'eq)))
    (dolist (slots-list slots-lists table)
      (setf (gethash (first slots-list) table)
            (make-item :type (or (first slots-list)
                                 (error "An item should have a slot"))
                       :color (second slots-list)
                       :pose-stored (third slots-list)
                       :pose-used (fourth slots-list))))))

(defparameter *item-types*
  (items-slots-hash-table
   `((plate (0.8 0.58 0.35))
     (fork  (0.2 0.1 0.3))
     (knife (0.5 0 0))
     (mug   (0.8 0.3 0))
     (pot (0.1 0.2 0.3))
     (bowl (0 0.3 0))
     (mondamin (0.5 0.1 0))))
  "Hash table: type -> item(type,color,pose-stored,pose-used).")

(defparameter *item-sizes*
  '((pancake-maker (0.15 0.15 0.035)))
  "AList item -> size list.")

(defun item-types ()
  (alexandria:hash-table-keys *item-types*))

(defun type-color (type)
  (or (ignore-errors (item-color (gethash type *item-types*))) *default-color*))

(defun type-pose-stored (type)
  (or (ignore-errors (item-pose-stored (gethash type *item-types*))) *init-pose*))

(defun type-pose-used (type)
  (or (ignore-errors (item-pose-used (gethash type *item-types*))) *init-pose*))

(defun type-size (type)
  (when (assoc type *item-sizes*)
    (car (cdr (assoc type *item-sizes*)))))

(defun move-object (object-name &optional (new-pose *init-pose*))
  (prolog-?w `(assert (object-pose ?w ,object-name ,new-pose))))

(defun move-object-onto (object-name onto-type onto-name)
  (let* ((on-designator (make-designator 'location `((on ,onto-type)
                                                     (name ,onto-name)
                                                     (for ,object-name))))
         (location (reference on-designator)))
    (move-object object-name location)
    (simulate *current-bullet-world* 10)))

(defun spawn-object (name type &optional pose)
  (let ((mesh? (assoc 'pancake btr::*mesh-files*)))
    (var-value
     '?object-instance
     (car (prolog-?w
            `(assert (object ?w ,(if mesh? 'mesh type)
                             ,name ,(or pose (type-pose-stored type))
                             :mass 0.2 :color ,(type-color type)
                             ,@(if mesh? `(:mesh ,type) `(:size ,(type-size type)))))
            `(%object ?w ,name ?object-instance))))))

(defun kill-object (name)
  (prolog-?w `(retract (object ?w ,name))))

(defun kill-all-objects ()
  (prolog-?w `(household-object-type ?w ?obj ?type) `(retract (object ?w ?obj)) '(fail)))

(defun object-instance (object-name)
  (var-value '?instance
             (car (prolog-?w `(%object ?w ,object-name ?instance)))))

(defun object-pose (object-name)
  (pose (object-instance object-name)))

(declaim (inline type-color move-object spawn-object kill-object))


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

