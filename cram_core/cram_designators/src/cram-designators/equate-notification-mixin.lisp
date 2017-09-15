;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :cram-designators)

(defclass equate-notification-mixin ()
  ((equate-notification-callbacks
    :initform nil
    :documentation "List of notification callbacks that are executed
    whenever the designator is equated with another one.")))

(defgeneric register-equate-callback (designator callback)
  (:documentation "Registers the callback function object `callback'
  at `designator'. The callback will be executed whenever the
  designator is equated other designators. `callback' is a function
  object that takes one parameter, the designator the parameter
  `designator' has been equated to.")
  (:method ((designator equate-notification-mixin) callback)
    (declare (type function callback))
    (with-slots (equate-notification-callbacks) designator
      (pushnew callback equate-notification-callbacks))))

(defgeneric unregister-equate-callback (designator callback)
  (:documentation "Removes `callback' from the list of equate
  callbacks in `designator'.")
  (:method ((designator equate-notification-mixin) callback)
    (declare (type function callback))
    (with-slots (equate-notification-callbacks) designator
      (setf equate-notification-callbacks
            (remove callback equate-notification-callbacks)))))

(defgeneric execute-equate-callbacks (designator other)
  (:documentation "Executes all equation callbacks of
  `designator'. `other' specifies the designator this designator has
  been equated to.")
  (:method ((designator equate-notification-mixin) (other designator))
    (with-slots (equate-notification-callbacks) designator
      (dolist (callback equate-notification-callbacks)
        (funcall callback other))))
  ;; Note(moesenle): We need to specialize on type T here and not on
  ;; type DESIGNATOR because in the inheritance tree of the more
  ;; specific designator types (location, action, object), designator
  ;; and the mixins are on the same level. That means specializations
  ;; on mixins are only executed after specializations on DESIGNATOR.
  (:method ((designator t) (other t))
    nil))

(defmacro with-equate-callback ((designator callback) &body body)
  (with-gensyms (evaluated-designator evaluated-callback)
    `(let ((,evaluated-designator ,designator)
           (,evaluated-callback ,callback))
       (unwind-protect
            (progn
              (register-equate-callback ,evaluated-designator ,evaluated-callback)
              ,@body)
         (unregister-equate-callback ,evaluated-designator ,evaluated-callback)))))

(defun execute-all-equated-callbacks (designator other)
  (declare (type equate-notification-mixin designator)
           (type designator other))
  (labels ((iterate (designator)
             (when designator
               (execute-equate-callbacks designator other)
               (iterate (successor designator)))))
    (iterate (first-desig designator))))

(defmethod equate :after ((parent equate-notification-mixin) successor)
  (execute-all-equated-callbacks parent successor))
