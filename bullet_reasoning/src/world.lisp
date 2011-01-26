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

(in-package :btr)

;;; Since we store and re-store the world references to rigid bodies,
;;; i.e. objects might get invalid, we need some sort of caching
;;; mechanism that allows us to unify object instances but update them
;;; when the world changes. For that, we create an object cache class
;;; that provides the actual rigid body as a slot and can handle
;;; invalidated objects by re-initializing based on the object name
;;; and the new world.

(defgeneric body (world cache))

(defclass object-cache ()
  ((world :initarg :world)
   (world-id :reader world-id)
   (name :initarg :name :reader name)
   (body :initarg :body)))

(defmethod initialize-instance :after ((cache object-cache) &key world)
  (when world
    (setf (slot-value cache 'world-id) (world-id world))))

(defmethod body ((world bt-world) (cache object-cache))
  (slot-value cache 'body))

(defmethod body :before ((world bt-world) (cache object-cache))
  (unless (and (slot-boundp cache 'body)
               (slot-boundp cache 'world)
               (eq world (slot-value cache 'world))
               (eq (world-id world) (world-id cache)))
    (setf (slot-value cache 'world) world)
    (setf (slot-value cache 'world-id) (world-id world))
    (setf (slot-value cache 'body)
          (find (name cache) (bodies world)
                :key #'name :test #'equal))))

(defun simulate (world secs &optional (dt 0.01))
  (multiple-value-bind (steps rest) (truncate secs dt)
    (when (> rest 0.0)
      (incf steps))
    (dotimes (i steps)
      (step-simulation world dt))))

(defun get-object (world name)
  (make-instance 'object-cache :world world :name name))

(defun find-objects (world &optional (pred (constantly t)))
  "Finds all objects that match the predicate"
  (remove-if-not
   pred (mapcar (lambda (body)
                  (make-instance 'object-cache
                                 :world world
                                 :name (name body)
                                 :body body))
                (bodies world))))

(defun contact-p (world obj-1 obj-2)
  (let ((body-1 (body world obj-1))
        (body-2 (body world obj-2)))
    (find-if (lambda (contact)
               (when (and
                      (> (array-dimension (contact-points contact) 0)
                         0)
                      (or (and (eq body-1 (body-1 contact))
                               (eq body-2 (body-2 contact)))
                          (and (eq body-2 (body-1 contact))
                               (eq body-1 (body-2 contact)))))
                 t))
             (contact-manifolds world))))

(defun find-all-contacts (world)
  (remove-if-not
   #'identity
   (mapcar (lambda (contact)
             (when (> (array-dimension (contact-points contact) 0)
                      0)
               (list (make-instance 'object-cache
                                    :world world
                                    :name (name (body-1 contact))
                                    :body (body-1 contact))
                     (make-instance 'object-cache
                                    :world world
                                    :name (name (body-2 contact))
                                    :body (body-2 contact)))))
           (contact-manifolds world))))

(defun find-objects-in-contact (world obj)
  (let ((body (body world obj)))
    (remove-if-not
     #'identity
     (mapcar (lambda (contact)
               (when (> (array-dimension (contact-points contact) 0)
                        0)
                 (cond ((eq body (body-1 contact))
                        (make-instance 'object-cache
                                       :world world
                                       :name (name (body-2 contact))
                                       :body (body-2 contact)))
                       ((eq body (body-2 contact))
                        (make-instance 'object-cache
                                       :world world
                                       :name (name (body-1 contact))
                                       :body (body-1 contact))))))
             (contact-manifolds world)))))

(defun poses-equal-p (pose-1 pose-2 dist-sigma ang-sigma)
  (and (< (cl-transforms:v-dist (cl-transforms:origin pose-1)
                                (cl-transforms:origin pose-2))
          dist-sigma)
       (< (cl-transforms:angle-between-quaternions
           (cl-transforms:orientation pose-1)
           (cl-transforms:orientation pose-2))
          ang-sigma)))

(defun stable-p (world obj)
  (when obj
    (let ((body (body world obj)))
      (not (eq (activation-state body)
               :active-tag)))))

(defun above-p (world obj-1 obj-2)
  "Returns T if `obj-1' is above `obj-2'"
  (let ((body-1 (body world obj-1))
        (body-2 (body world obj-2)))
    (> (cl-transforms:z (cl-transforms:origin (pose body-1)))
       (cl-transforms:z (cl-transforms:origin (pose body-2))))))

(defun find-objects-above (world obj)
  "Returns a list of all objects thate are above `obj'"
  (find-objects world (lambda (o) (above-p world o obj))))

(defun below-p (world obj-1 obj-2)
  "Returns T if `obj-1' is below `obj-2'"
  (let ((body-1 (body world obj-1))
        (body-2 (body world obj-2)))
    (< (cl-transforms:z (cl-transforms:origin (pose body-1)))
       (cl-transforms:z (cl-transforms:origin (pose body-2))))))

(defun find-objects-below (world obj)
  "Returns a list of all objects thate are below `obj'"
  (find-objects world (lambda (o) (below-p world o obj))))
