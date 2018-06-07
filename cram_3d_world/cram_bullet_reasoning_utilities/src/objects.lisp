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

(in-package :bullet-reasoning-utilities)

(defun object-instance (object-name)
  (var-value '?instance
             (car (prolog-?w `(%object ?w ,object-name ?instance)))))

(defun object-pose (object-name)
  (pose (object-instance object-name)))

(defun object-exists (object-name)
  (typep (object-instance object-name) 'btr:object))


(defgeneric spawn-object (name type &key pose color mass world)
  (:method (name type &key pose color mass world)
    (if (btr:object (or world *current-bullet-world*) name)
        (when pose
          (move-object name pose))
        (var-value
         '?object-instance
         (car (prolog
               `(and
                 ,(if pose
                      `(equal ?pose ,pose)
                      `(scenario-objects-init-pose ?pose))
                 ,(if color
                      `(equal ?color ,color)
                      `(scenario-object-color ?_ ,type ?color))
                 ,(if world
                      `(equal ?world ,world)
                      `(bullet-world ?world))
                 (scenario-object-shape ,type ?shape)
                 (scenario-object-extra-attributes ?_ ,type ?attributes)
                 (append (object ?world ?shape ,name ?pose :mass ,(or mass 0.2) :color ?color)
                         ?attributes
                         ?object-description)
                 (assert ?object-description)
                 (%object ?world ,name ?object-instance))))))))

(defgeneric kill-object (name)
  (:method (name)
    (prolog-?w `(retract (object ?w ,name)))))

(defgeneric kill-all-objects ()
  (:method ()
    (prolog-?w `(item-type ?w ?obj ?type) `(retract (object ?w ?obj)) '(fail))))

(defun respawn-object (object)
  (typecase object
    (btr:item (let ((name (btr:name object))
                    (type (car (slot-value object 'btr::types)))
                    (pose (btr:pose object))
                    (color (slot-value (slot-value (car (btr:rigid-bodies object))
                                                   'cl-bullet::collision-shape)
                                       'cl-bullet-vis::color))
                    (mass (slot-value (car (btr:rigid-bodies object)) 'cl-bullet:mass)))
                (btr:remove-object btr:*current-bullet-world* name)
                (btr:add-object btr:*current-bullet-world* :mesh name pose
                                :color color :mass mass :mesh type)))
    (t nil)))


(defun move-object (object-name &optional new-pose)
  (if new-pose
      (prolog-?w
        `(assert (object-pose ?w ,object-name ,new-pose)))
      (prolog-?w
        '(scenario-objects-init-pose ?pose)
        `(assert (object-pose ?w ,object-name ?pose)))))

(defun translate-object (object-name &optional (x-delta 0.0) (y-delta 0.0) (z-delta 0.0))
  (let* ((object-pose (object-pose object-name))
         (new-pose (cl-transforms:copy-pose
                    object-pose
                    :origin (cl-transforms:copy-3d-vector
                             (cl-transforms:origin object-pose)
                             :x (+ x-delta (cl-transforms:x (cl-transforms:origin object-pose)))
                             :y (+ y-delta (cl-transforms:y (cl-transforms:origin object-pose)))
                             :z (+ z-delta (cl-transforms:z (cl-transforms:origin object-pose)))))))
    (move-object object-name new-pose)))

;; (defun move-object-onto (object-name onto-type &optional onto-name)
;;   (let* ((size
;;            (cl-bullet:bounding-box-dimensions (aabb (object-instance object-name))))
;;          (obj-diagonal-len
;;            (sqrt (+ (expt (cl-transforms:x size) 2) (expt (cl-transforms:y size) 2))))
;;          (on-designator
;;            (make-designator :location `((:on ,onto-type)
;;                                         ,(when onto-name (list :name onto-name))
;;                                         (:centered-with-padding ,obj-diagonal-len)))))
;;     (prolog
;;      `(assert-object-pose-on ,object-name ,on-designator))))


(defun item-exists (object-name)
  (typep (object-instance object-name) 'btr:item))


(defun spawn-object-aabb-box (object-name
                              &key
                                (box-name (gensym (concatenate
                                                   'string
                                                   (symbol-name object-name)
                                                   "-BOX")))
                                (world *current-bullet-world*))
  (let* ((aabb (aabb (object-instance object-name)))
         (aabb-pose (cl-transforms:make-pose
                     (cl-bullet:bounding-box-center aabb)
                     (cl-transforms:make-identity-rotation)))
         (aabb-size (with-slots (cl-transforms:x cl-transforms:y cl-transforms:z)
                        (cl-bullet:bounding-box-dimensions aabb)
                      (list cl-transforms:x cl-transforms:y cl-transforms:z))))
    (add-object world :box box-name aabb-pose :mass 0.0 :size aabb-size)
    box-name))


;; (declaim (inline move-object object-instance object-pose object-exists item-exists))


;;;;;;;;;;;;;;;;;;;; PROLOG ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(def-fact-group bullet-reasoning-utilities ()
  (<- (assert-object-pose ?obj-name ?desig)
    (once
     (bound ?obj-name)
     (bound ?desig)
     (bullet-world ?w)
     (designator-groundings ?desig ?solutions)
     (take 1 ?solutions ?8-solutions)
     (generate-values ?poses-on (obj-poses-on ?obj-name ?8-solutions ?w))
     (member ?solution ?poses-on)
     (assert (object-pose ?w ?obj-name ?solution))))

  (<- (assert-object-pose-on ?obj-name ?desig)
    (once
     (bound ?obj-name)
     (bound ?desig)
     (bullet-world ?w)
     (designator-groundings ?desig ?solutions)
     (take 8 ?solutions ?8-solutions)
     (member ?solution ?8-solutions)
     (assert (object-pose-on ?w ?obj-name ?solution)))))

