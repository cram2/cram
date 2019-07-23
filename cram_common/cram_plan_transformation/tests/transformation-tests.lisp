;;;
;;; Copyright (c) 2019, Arthur Niedzwiecki <aniedz@cs.uni-bremen.de>
;;;                     
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

(in-package :plt-tests)

(setf lisp-unit:*print-failures* t)
(setf lisp-unit:*print-errors* t)
(setf lisp-unit:*print-summary* t)
(setf btr-belief:*spawn-debug-window* t)

(defun move-to-origin ()
  "Navigating the robot will reset the rigid bodies of the objects,
such that they can fall down and collide with the supporting surface."
  (let* ((?pose
           (cl-transforms-stamped:make-pose-stamped 
            "map" 0.0
            (cl-transforms:make-3d-vector 0 0 0)
            (cl-tf:make-identity-rotation)))
         (?target-robot-location (desig:a location
                                          (pose ?pose))))
    (exe:perform (desig:an action
                           (type navigating)
                           (location ?target-robot-location)))))

(defun demo-successful ()
  "Check if bowl and cup are on the kitchen table."
  (btr:simulate btr:*current-bullet-world* 100)
  (prolog `(and (btr:bullet-world ?w)
                (btr:contact ?w :bowl-1 :kitchen "kitchen_island")
                (btr:contact ?w :cup-1 :kitchen "kitchen_island"))))

(defun execute-demo-for-both-hands (&optional (reset-tt nil) (retries 5))
  (when (>= retries 0)
    (when reset-tt
      (plt:reset-task-tree))
    (urdf-proj:with-projected-robot
      (cet:enable-fluent-tracing)
      (demo::demo-random nil '(:bowl :cup))
      (cet:disable-fluent-tracing)
      (move-to-origin))
    (or (demo-successful)
        (execute-demo-for-both-hands t (decf retries)))))


(defun demo-environment (&optional (reset-tt nil))
  (urdf-proj:with-projected-robot
      (demo-environment-rule)))


(define-test apply-rules-both-hands-test
  (unless (eq (roslisp:node-status) :RUNNING)
    (roslisp-utilities:startup-ros))
  (execute-demo-for-both-hands t)
  (let* ((executing-basic-demo-successful (demo-successful))
         (both-hands-rule-applied
           (when executing-basic-demo-successful (plt:apply-rules)))
         transformed-demo-results)
        (assert-true executing-basic-demo-successful)
        (assert-true both-hands-rule-applied)
        (when both-hands-rule-applied
          (btr-utils:kill-all-objects)
          (loop for try-count to 4
                until (demo-successful)
                do (execute-demo-for-both-hands)
                   (push (demo-successful) transformed-demo-results)))
        (assert-true (reduce (lambda (r1 r2) (or r1 r2)) transformed-demo-results))))
