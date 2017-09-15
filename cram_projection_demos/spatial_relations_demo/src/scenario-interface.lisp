;;;
;;; Copyright (c) 2015, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(defparameter *demo-object-types* nil
  "An AList with object set keyword -> object types list.
E.g. '((:main . (plate fork)) (:other-set . (other-type))).")

(defparameter *demo-objects-initial-poses* nil
  "An AList with object type -> list of initial poses for objects of the type.
E.g. '((plate ((0 0 1) (0 0 0 1))
              ((1 0 1) (0 0 1 0)))
       (other-type ((1.2 3.4 5.3) (0 0 0 1))))
for 2 plates and 1 other-type.")

(defparameter *demo-objects* (make-hash-table :test #'eq)
  "All the spawned objects will go here. Hash table: name -> obj-instance.")

(defgeneric parameterize-demo (demo-name)
  (:documentation "Bind all the parameters such as *demo-object-types* etc."))

(defmethod spawn-object :around (name type &key pose color world)
  (declare (ignore type pose color world))
  (setf (gethash name *demo-objects*) (call-next-method)))

(defmethod kill-object :after (name)
  (remhash name *demo-objects*))

(defmethod kill-all-objects :after ()
  (clrhash *demo-objects*))

(defun spawn-demo (demo-name &key (set nil))
  "Function to spawn the demo objects.
`set' is, e.g., :main or :other-set, if not given spawns all objects."
  (kill-all-objects)
  (parameterize-demo demo-name)
  (clear-costmap-vis-object)
  (detach-all-objects (object-instance (robot-name)))
  (move-robot '((0 0 0) (0 0 1 0)))
  (park-robot)

  (let ((object-types
          (if set
              (cdr (assoc set *demo-object-types*))
              (apply #'append (mapcar #'cdr *demo-object-types*))))
        (resulting-object-names '()))
    (dolist (type object-types)
      (let ((poses-for-type (cdr (assoc type *demo-objects-initial-poses*))))
        (dotimes (i (length poses-for-type))
          (let ((name (new-symbol-with-id type i)))
            (format t "~a ~a ~%" name type)
            (push name resulting-object-names)
            (spawn-object name type :pose (nth i poses-for-type))))))
    (mapcar (alexandria:rcurry #'gethash *demo-objects*) resulting-object-names)))

(defun respawn-demo (demo-name &key (set nil))
  (setf *demo-object-types* nil)
  (spawn-demo demo-name :set set))

(defgeneric execute-demo (demo-name &key &allow-other-keys)
  (:documentation "A function to call after spawn-demo to execute the scenario.")
  (:method :before (demo-name &key (set nil))
    (respawn-demo demo-name :set set)))

(defun new-symbol-with-id (string number)
  (alexandria:format-symbol T "~a-~a" string number))

(defun new-symbol-from-strings (&rest strings)
  (alexandria:format-symbol T "~{~a~}" strings))

(declaim (inline new-symbol-with-id new-symbol-from-strings))

(defun demo-object-names ()
  (alexandria:hash-table-keys *demo-objects*))

(defun move-demo-objects-away ()
  (dolist (name (demo-object-names))
    (move-object name)))

(defun demo-object-instance (name)
  (gethash name *demo-objects*))
