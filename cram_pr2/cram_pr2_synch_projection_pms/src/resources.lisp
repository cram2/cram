;;; Copyright (c) 2011, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :projection-process-modules)

(defclass resources ()
  ((acquired-resources
    :initform nil :documentation "List of acquired resources.")
   (available-resources
    :initarg :resources
    :documentation "List of currently available, i.e. not acquired resources.")
   (resources
    :initarg :resources :reader resources
    :documentation "List of all resources, acquired and available ones.")
   (lock :initform (sb-thread:make-mutex))
   (condition :initform (sb-thread:make-waitqueue))
   (update-hooks :initform (make-hash-table :weakness :key))))

(defgeneric acquire-resource (resources &key resource wait test)
  (:documentation "Acquires `resource'. If `wait' is non-NIL, blocks
  until the resource becomes available. `test' is used to search for
  the resource. This method returns NIL if resource acquiration fails,
  T otherwise.")
  (:method ((resources resources) &key resource (wait t) (test #'eql))
    (with-slots (acquired-resources available-resources lock condition)
        resources
      (assert (find resource (slot-value resources 'resources) :test test))
      (prog1
          (sb-thread:with-mutex (lock)
            (loop do
              (let ((allocated-resource (find resource available-resources :test test)))
                (cond (allocated-resource
                       (push allocated-resource acquired-resources)
                       (setf available-resources
                             (remove resource available-resources :test test))
                       (sb-thread:condition-broadcast condition)
                       (return t))
                      (wait (sb-thread:condition-wait condition lock))
                      (t (return nil))))))
        (execute-update-hooks resources :resource resource :test test)))))

(defgeneric release-resource (resources &key resource test error-p)
  (:documentation "Releases a previously acquired resource. If
  `error-p' is NIL, does not throw an error if the resource hasn't
  been acquired before.")
  (:method ((resources resources) &key resource (test #'eql) (error-p t))
    (with-slots (acquired-resources available-resources lock condition)
        resources
      (assert (find resource (slot-value resources 'resources) :test test))
      (sb-thread:with-mutex (lock)
        (let ((acquired-resource (find resource acquired-resources :test test)))
          (when error-p
            (assert acquired-resource () "To release a resource it needs to be acquired."))
          (push acquired-resource available-resources)
          (setf acquired-resources (remove resource acquired-resources :test test))
          (sb-thread:condition-broadcast condition)))
      (execute-update-hooks resources :resource resource :test test))))

(defgeneric resource-available-fluent (resources &key resource test)
  (:documentation "Returns a fluent that is T if `resource' is
  available, otherwise NIL.")
  (:method ((resources resources) &key resource (test #'eql))
    (labels ((compute-value ()
               ;; Note(moesenle): we need to have the lock already.
               (with-slots (available-resources) resources
                 (not (null (find resource available-resources :test test)))))
             (fluent-update-function (fluent)
               (lambda ()
                 (setf (cpl:value fluent) (compute-value)))))
      (with-slots (update-hooks lock) resources
        (sb-thread:with-mutex (lock)
          (let ((fluent (cpl-impl:make-fluent :value (compute-value))))
            (setf (gethash fluent update-hooks)
                  (cons resource (fluent-update-function fluent)))
            fluent))))))

(defgeneric execute-update-hooks (resources &key resource test)
  (:method ((resources resources) &key resource (test #'eql))
    (with-slots (lock update-hooks) resources
      (sb-thread:with-mutex (lock)
        (maphash (lambda (key value)
                   (declare (ignore key))
                   (when (funcall test resource (car value))
                     (funcall (cdr value))))
                 update-hooks)))))

(defmacro with-resources ((resources resources-to-acquire &key test (wait t))
                          &body body)
  (alexandria:with-gensyms (test-symbol)
    (alexandria:once-only (resources resources-to-acquire wait)
      `(let (,@(when test `((,test-symbol ,test))))
         (unwind-protect
              (progn
                (dolist (resource ,resources-to-acquire)
                  (acquire-resource
                   ,resources :resource resource :wait ,wait ,@(when test `(:test ,test-symbol))))
                ,@body)
           (dolist (resource ,resources-to-acquire)
             (release-resource
              ,resources :resource resource :error-p nil ,@(when test `(:test ,test-symbol)))))))))
