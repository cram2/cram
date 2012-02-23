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

(defvar *perceived-object-mappings* (cons
                                     (tg:make-weak-hash-table :weakness :key-or-value)
                                     (tg:make-weak-hash-table :weakness :key-or-value))
  "Two weak hash-tables that map PERCEIVED-OBJECT instances to OBJECTS
  and the other way around.")

(defvar *perception-worker-thread* nil)
(defvar *perception-worker-input-queue* (make-instance 'physics-utils:event-queue))

(defvar *perception-visible-objects* (tg:make-weak-hash-table :weakness :key)
  "List of objects that are visible for a specific run of the
  perception process module. The table maps input designators to a
  list of visible object names.")

(defun get-obj-name (perceived-object)
  (let ((obj (gethash perceived-object (car *perceived-object-mappings*))))
    (if obj
        (name obj)
        (gentemp "OBJ-" (find-package :btr)))))

(defun get-perceived-object (obj)
  (declare (type object obj))
  (gethash obj (cdr *perceived-object-mappings*)))

(defun cache-object (obj perceived-obj)
  (declare (type object obj)
           (type perception-pm:perceived-object perceived-obj))
  (setf (gethash perceived-obj (car *perceived-object-mappings*))
        obj)
  (setf (gethash obj (cdr *perceived-object-mappings*))
        perceived-obj))

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
             (with-vars-bound (?obj)
                 (lazy-car
                  (prolog `(and (bullet-world ?w)
                                (assert (object ?w ,@object-descr))
                                (%object ?w ,object-name ?obj))))
               (cache-object ?obj ?perceived-object)))))
    (unless *disable-rete-integration*
      (when (eq op :assert)
        (maybe-create-perception-worker)
        (physics-utils:post-event
         *perception-worker-input-queue*
         #'add)))))

(defun on-perception-started (input)
  (maybe-create-perception-worker)
  (flet ((place-robot ()
           (with-vars-bound (?robot)
               (lazy-car
                (prolog `(and (bullet-world ?w)
                              (robot ?robot-name)
                              (%object ?w ?robot-name ?robot))))
             (when ?robot
               (set-robot-state-from-tf cram-roslisp-common:*tf* ?robot))))
         (init-visible-objects ()
           (with-vars-bound (?visible-objects)
               (lazy-car (prolog `(and (bullet-world ?w)
                                       (robot ?robot)
                                       (camera-frame ?camera)
                                       (link-pose ?w ?robot ?camera ?camera-pose)
                                       (setof ?obj (and (object-type ?obj household-object)
                                                        (visible ?w ?camera-pose ?obj))
                                              ?visible-objects))))
             (setf (gethash input *perception-visible-objects*)
                   (unless (is-var ?visible-objects)
                     ?visible-objects)))))
    (physics-utils:post-event
     *perception-worker-input-queue*
     (lambda ()
       (place-robot)
       (init-visible-objects)))))

(defun on-perception-finished (input)
  (maybe-create-perception-worker)
  (let ((potentially-visible (gethash input *perception-visible-objects*)))
    (flet ((remove-objects ()
             (force-ll (prolog `(and
                                 (bullet-world ?w)
                                 (or (and
                                      (member ?o ,potentially-visible)
                                      (findall ?other (and (contact ?w ?o ?other)
                                                           (object-type ?w ?other household-object))
                                               nil)
                                      (format "removing ~a~%" ?o)
                                      (retract (object ?o)))
                                     (and
                                      (object ?o-2)
                                      (not (member ?o-2 ,potentially-visible))
                                      (contact ?w ?o-1 ?o-2)
                                      (object-type ?w ?o-1 household-object)
                                      (object-type ?w ?o-2 household-object)
                                      ;; ?o is the old object and ?o-2
                                      ;; the new one. If we can
                                      ;; replace ?o by ?o-2 (e.g. when
                                      ;; ?o is a cluster, we replace
                                      ;; it. If the two descriptions
                                      ;; are equal, i.e. if we have
                                      ;; the same object twice, we
                                      ;; also replace
                                      ;; objects. Otherwise we get rid
                                      ;; of the new perception.
                                      (-> (object-replacable ?w ?o-1 ?o-2)
                                          (retract (object ?w ?o-1))
                                          (retract (object ?w ?o-2))))))))))
      (remhash input *perception-visible-objects*)
      (physics-utils:post-event *perception-worker-input-queue* #'remove-objects))))

(defun on-pm-execute (op &key ?module ?input)
  (when (eql ?module :perception)
    (case op
      (:assert
         (roslisp:ros-info (perception reasoning) "Perception process module started")
         (on-perception-started ?input))
      (:retract
         (roslisp:ros-info (perception reasoning) "Perception process module finished. Synchronizing.")
         (on-perception-finished ?input)
         (physics-utils:wait-for-queue-empty *perception-worker-input-queue*)))))

(def-production on-object-perceived
  (perception-pm:object-perceived ?desig ?perceived-object))

(register-production-handler 'on-object-perceived 'add-perceived-object)
(register-production-handler 'cram-plan-knowledge:on-pm-execute 'on-pm-execute)
