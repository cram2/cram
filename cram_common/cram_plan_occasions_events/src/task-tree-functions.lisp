;;;
;;; Copyright (c) 2019, Arthur Niedzwiecki <niedzwiecki@uni-bremen.de>
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

(in-package :cpoe)

(defparameter *location-distance-threshold* 0.5)
(defun  get-location-distance-threshold ()
  "For prolog predicates."
  *location-distance-threshold*)

(defun location-desig-dist (desig-1 desig-2)
  (cl-tf:v-dist (cl-tf:origin (desig:desig-prop-value desig-1 :pose))
                (cl-tf:origin (desig:desig-prop-value desig-2 :pose))))

(defun subtasks-of-type (task type desig-class)
  "A very fast way to find all subtask from root `task' of a specfic `type'.
`desig-class' = action-designator -> breadth-first task-tree
              = motion-designator -> depth-first task-tree"
  (let* ((subtasks (case desig-class
                     (desig:action-designator (cpl:flatten-task-tree-broad task))
                     (desig:motion-designator (cpl:flatten-task-tree task))))
         (filter-predicate
           (lambda (node)
             (and (cpl:task-tree-node-code node)
                  (cpl:code-parameters
                   (cpl:task-tree-node-code node))
                  (cpl:code-sexp
                   (cpl:task-tree-node-code node))
                  (eq (car (cpl:code-sexp (cpl:task-tree-node-code node)))
                      'exe:perform)
                  (eq (desig:desig-prop-value
                       (car (cpl:code-parameters (cpl:task-tree-node-code node)))
                       :type)
                      type)
                  (eq (type-of
                       (car (cpl:code-parameters (cpl:task-tree-node-code node))))
                      desig-class)))))
    (remove-if-not filter-predicate subtasks)))
