;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;

(in-package :bullet)

(defgeneric get-state (obj)
  (:documentation "Returns a state object that allows to completely
  restore `obj'"))

(defgeneric restore-state (state world)
  (:documentation "Re-creates the object that is described by state
  and re-adds it to the world."))

(defgeneric copy-world (world &optional destination)
  (:method ((world bt-world) &optional destination)
    (restore-world-state (get-state world) (or destination
                                               (make-instance
                                                (class-of world)
                                                :gravity-vector (gravity-vector world))))))

(defclass world-state ()
  ((bodies :initarg :bodies :reader bodies)
   (constraints :initarg :constraints :reader constraints)
   (gravity-vector :initarg :gravity-vector :reader gravity-vector)
   (debug-drawer :initarg :debug-drawer :reader debug-drawer)))

(defclass rigid-body-state ()
  ((name :initarg :name :reader name)
   (mass :initarg :mass :reader mass)
   (pose :initarg :pose :reader pose)
   (force :initarg :force :reader force)
   (torque :initarg :torque :reader torque)
   (linear-velocity :initarg :linear-velocity :reader linear-velocity)
   (angular-velocity :initarg :angular-velocity :reader angular-velocity)
   (activation-state :initarg :activation-state :reader activation-state)
   (collision-flags :initarg :collision-flags :reader collision-flags)
   (collision-shape :initarg :collision-shape :reader collision-shape)
   (collision-group :initarg :collision-group :reader collision-group)
   (collision-mask :initarg :collision-mask :reader collision-mask)))

(defclass constraint-state ()
  ((body-1 :initarg :body-1 :reader body-1)
   (body-2 :initarg :body-2 :reader body-2)))

(defclass motor-state ()
  ((enabled :initarg :enabled :reader enabled)
   (limits :initarg :limits :reader limits)
   (max-impulse :initarg :max-impulse :reader max-impulse)))

(defclass point-2-point-constraint-state (constraint-state)
  ((point-in-1 :initarg :point-in-1 :reader point-in-1)
   (point-in-2 :initarg :point-in-2 :reader point-in-2)))

(defclass hinge-constraint-state (constraint-state motor-state)
  ((frame-in-1 :initarg :frame-in-1 :reader frame-in-1)
   (frame-in-2 :initarg :frame-in-2 :reader frame-in-2)))

(defclass slider-constraint-state (constraint-state motor-state)
  ((frame-in-1 :initarg :frame-in-1 :reader frame-in-1)
   (frame-in-2 :initarg :frame-in-2 :reader frame-in-2)))

(defmethod get-state ((world bt-world))
  (with-world-locked world
    (make-instance 'world-state
      :bodies (mapcar #'get-state (bodies world))
      :constraints (mapcar #'get-state (constraints world))
      :gravity-vector (gravity-vector world)
      :debug-drawer (get-debug-drawer world))))

(defmethod get-state ((body rigid-body))
  (make-instance 'rigid-body-state
    :name (name body)
    :mass (mass body)
    :pose (pose body)
    :force (total-force body)
    :torque (total-torque body)
    :linear-velocity (linear-velocity body)
    :angular-velocity (angular-velocity body)
    :activation-state (activation-state body)
    :collision-flags (collision-flags body)
    :collision-shape (get-state (collision-shape body))
    :collision-group (collision-group body)
    :collision-mask (collision-mask body)))

(defmethod get-state ((constraint point-2-point-constraint))
  (make-instance 'point-2-point-constraint-state
    :body-1 (name (body-1 constraint))
    :body-2 (name (body-2 constraint))
    :point-in-1 (point-in-1 constraint)
    :point-in-2 (point-in-2 constraint)))

(defmethod get-state ((constraint hinge-constraint))
  (make-instance 'hinge-constraint-state
    :body-1 (name (body-1 constraint))
    :body-2 (name (body-2 constraint))
    :frame-in-1 (frame-in-1 constraint)
    :frame-in-2 (frame-in-2 constraint)
    :enabled (enabled constraint)
    :limits `((:lower . ,(limit constraint :lower))
              (:upper . ,(limit constraint :upper)))
    :max-impulse (max-impulse constraint)))

(defmethod get-state ((constraint slider-constraint))
  (make-instance 'hinge-constraint-state
    :body-1 (name (body-1 constraint))
    :body-2 (name (body-2 constraint))
    :frame-in-1 (frame-in-1 constraint)
    :frame-in-2 (frame-in-2 constraint)
    :enabled (enabled constraint)
    :limits `((:lower-lin . ,(limit constraint :lower-lin))
              (:upper-lin . ,(limit constraint :upper-lin))
              (:lower-ang . ,(limit constraint :lower-ang))
              (:upper-ang . ,(limit constraint :upper-ang)))
    :max-impulse (max-impulse constraint)))

(defmethod get-state ((shape collision-shape))
  "We don't need to store the collision shape since it is not changed
  by physics simulation. Storing and restoring it would be pretty
  expensive, in particular when we have meshes with lots of vertices."
  shape)

(defun restore-world-state (world-state &optional world)
  "Restores the state captured in `world-state' If `world' is
specified, updates it so that it exactly matches the state in
`world-state', otherwise a new world is created. Returns updated
world."
  (restore-state
   world-state
   (or world
       (make-instance 'bt-world
         :gravity-vector (gravity-vector world-state)))))

(defmethod restore-state ((world-state world-state) (world bt-world))
  (with-world-locked world
    ;; First clear the world
    (dolist (constraint (constraints world))
      (remove-constraint world constraint))
    (dolist (body (bodies world))
      (remove-rigid-body world body))
    (setf (gravity-vector world) (gravity-vector world-state))
    ;; generate a new unique world identifier
    (setf (slot-value world 'id) (gensym "WORLD-"))
    ;; re-initialize the world
    (with-slots (bodies constraints debug-drawer) world-state
      (when debug-drawer
        (set-debug-drawer world debug-drawer))
      (dolist (body bodies)
        (restore-state body world))
      (dolist (constraint constraints)
        (restore-state constraint world))))
  world)

(defmethod restore-state ((body rigid-body-state) (world bt-world))
  (with-slots (name
               mass
               pose
               force torque
               linear-velocity angular-velocity
               activation-state collision-flags
               collision-shape
               collision-group
               collision-mask)
      body
    (let ((body
            (make-instance 'rigid-body
              :name name
              :pose pose
              :mass mass
              :collision-shape collision-shape
              :activation-state activation-state
              :collision-flags collision-flags
              :group collision-group
              :mask collision-mask)))
      (add-rigid-body world body)
      (clear-forces body)
      (apply-central-force body force)
      (apply-torque body torque)
      (setf (linear-velocity body) linear-velocity)
      (setf (angular-velocity body) angular-velocity)
      body)))

(defmethod restore-state ((constraint point-2-point-constraint-state) (world bt-world))
  (with-slots (body-1 body-2 point-in-1 point-in-2) constraint
    (let ((constraint (make-instance 'point-2-point-constraint
                        :body-1 (find body-1 (bodies world) :key #'name)
                        :body-1 (find body-2 (bodies world) :key #'name)
                        :point-in-1 point-in-1
                        :point-in-2 point-in-2)))
      (add-constraint world constraint)
      constraint)))

(defun make-motor-constraint (class world state)
  (with-slots (body-1
               body-2
               enabled limits max-impulse
               frame-in-1 frame-in-2)
      state
    (let ((constraint (make-instance class
                        :body-1 (find body-1 (bodies world) :key #'name)
                        :body-2 (find body-2 (bodies world) :key #'name)
                        :frame-in-1 frame-in-1
                        :frame-in-2 frame-in-2)))
      (setf (enabled constraint) enabled)
      (loop for (lim-type . lim-value) in limits do
        (setf (limit constraint lim-type) lim-value))
      (setf (max-impulse constraint) max-impulse))))

(defmethod restore-state ((constraint hinge-constraint-state) (world bt-world))
  (let ((constraint (make-motor-constraint 'hinge-constraint world constraint)))
    (add-constraint world constraint)
    constraint))

(defmethod restore-state ((constraint slider-constraint-state) (world bt-world))
  (let ((constraint (make-motor-constraint 'slider-constraint world constraint)))
    (add-constraint world constraint)
    constraint))
