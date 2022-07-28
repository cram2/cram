;;;
;;; Copyright (c) 2022, Arthur Niedzwiecki <niedzwiecki@uni-bremen.de>
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

(in-package :tt-export-tests)

;; Launchfile:
;; roslaunch cram_pr2_pick_place_demo sandbox.launch

(defun demo ()
  (setf btr:*debug-window* NIL)
  (unless (eq (roslisp:node-status) :RUNNING)
    (roslisp-utilities:startup-ros))
  (urdf-proj:with-projected-robot
    (demo::setting-demo '(:bowl))))

(defun node-tree->formatted-tree (node &optional (depth 2))
  "`depth' can be set to -1 to generate the whole tree."
  (when node
    (list (tt-export::format-node (tt-export::node->designator node))
          (unless (= depth 0)
            (mapcar (alexandria:rcurry #'node-tree->formatted-tree (- depth 1))
                    (tt-export::node-children node))))))

(define-test export-task-tree-from-setting-demo
  (demo)
  (let ((tree (tt-export::get-task-tree)))
    (assert-true tree)
    (assert-true (node-tree->formatted-tree tree))
    (let ((graph (export-task-tree :data tree :dry-run T)))
      (assert-true graph)
      (assert-true (slot-value graph 'cl-dot::nodes))
      (assert-true (slot-value graph 'cl-dot::edges))
      (assert-true (slot-value graph 'cl-dot::cluster-nodes))
      (assert-true (car (alexandria:hash-table-values
                         (slot-value graph 'cl-dot::cluster-nodes)))))))
