;;;
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
;;;

(in-package :btr)

(defvar *disable-rete-integration* nil)

(defvar *perceived-object-mappings* (tg:make-weak-hash-table :weakness :key)
  "A weak hash-table that is used to map perceived-object instances to
  a corresponding object name used in the reasoning system.")

(defvar *perception-worker-thread* nil)
(defvar *perception-worker-input-queue* (make-instance 'physics-utils:event-queue))

(defun get-obj-name (perceived-object)
  (or (gethash perceived-object *perceived-object-mappings*)
      (setf (gethash perceived-object *perceived-object-mappings*)
            (gentemp "OBJ-" (find-package :btr)))))

(defun get-prolog-object-description (name desig perceived-object)
  (declare (ignore desig))
  (etypecase perceived-object
    (perception-pm:cop-perceived-object
       `(cop-object ,name ,(perception-pm:object-pose perceived-object)
                    :object-id ,(perception-pm:object-id perceived-object)))))

(defun maybe-create-perception-worker ()
  (flet ((perception-worker ()
           (loop do
             (handler-case
                 (let ((event (physics-utils:get-next-event *perception-worker-input-queue*)))
                   (cond ((eql event :exit)
                          (return nil))
                         (t (funcall event))))
               (error (e)
                 (warn 'simple-warning
                       :format-control "Worker thread throw an error: ~a"
                       :format-arguments (list e)))))))
    (unless (and *perception-worker-thread*
                 (sb-thread:thread-alive-p *perception-worker-thread*))
      (setf *perception-worker-thread*
            (sb-thread:make-thread #'perception-worker :name "PERCEPTION-WORKER")))))

(defun add-perceived-object (op &key ?desig ?perceived-object)
  (flet ((add ()
           (let* ((object-name (get-obj-name ?perceived-object))
                  (object-descr (get-prolog-object-description
                                 object-name ?desig ?perceived-object)))
             (force-ll
              (prolog `(and (bullet-world ?w)
                            (assert-object ?w ,@object-descr)))))))
    (unless *disable-rete-integration*
      (when (eq op :assert)
        (maybe-create-perception-worker)
        (physics-utils:post-event
         *perception-worker-input-queue*
         #'add)))))

(defun on-perception-started (input)
  (declare (ignore input))
  (maybe-create-perception-worker)
  ;; TODO: Don't be PR2 specific here
  (flet ((place-robot ()
           (with-vars-bound (?robot)
               (lazy-car
                (prolog `(and (bullet-world ?w)
                              (robot ?robot-name)
                              (%object ?w ?robot-name ?robot))))
             (when ?robot
               (set-robot-state-from-tf cram-roslisp-common:*tf* ?robot)))))
    (physics-utils:post-event *perception-worker-input-queue* #'place-robot)))

(defun on-pm-execute (op &key ?module ?input)
  (when (eql ?module :perception)
    (case op
      (:assert
         (roslisp:ros-info (perception reasoning) "Perception process module started")
         (on-perception-started ?input))
      (:retract
         (roslisp:ros-info (perception reasoning) "Perception process module finished. Synchronizing.")
         (physics-utils:wait-for-queue-empty *perception-worker-input-queue*)))))

(def-production on-object-perceived
  (perception-pm:object-perceived ?desig ?perceived-object))

(register-production-handler 'on-object-perceived 'add-perceived-object)
(register-production-handler 'cram-plan-knowledge:on-pm-execute 'on-pm-execute)
