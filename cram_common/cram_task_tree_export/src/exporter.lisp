;;;
;;; Copyright (c) 2022, Arthur Niedzwiecki <aniedz@cs.uni-bremen.de>
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

(in-package :tt-export)

(defparameter *export-file-path* "/tmp/task_tree_export.dot")

(defmethod cl-dot:object-node ((node task-tree-node))
  (let ((node-label (format-node node)))
    (make-instance 'cl-dot:node
      :attributes `(:label (:left ,node-label) ; text-align left
                    :shape :box))))

(defmethod cl-dot:object-points-to ((node task-tree-node))
  (node-children node))

(defun export-task-tree (&key
                           (data (get-task-tree))
                           (path *export-file-path*)
                           dry-run)
  "Exports the task-tree to a dot file.
data - a task-tree-node
path - output filepath
dry-run - don't generate dot file, just the graph object "
  (unless data
    (error "No task tree built yet. Execute a demo first."))
  (let ((graph (cl-dot:generate-graph data)))
    (unless dry-run
      (let ((stream (open path
                          :direction :output
                          :if-exists :supersede
                          :if-does-not-exist :create)))
        (cl-dot:print-graph graph :stream stream)
        (close stream))
      (format T "File exported to ~a.~%~a~%~a~%~a" path
              "Install graphviz to convert from .dot file:"
              "dot -Tpdf <dot-in> -o <pdf-out>"
              "dot -Tsvg <dot-in> -o <svg-out>"))
    graph))


#+Examples-from-documantation(
;; Conses
(defmethod cl-dot:object-node ((object cons))
  (make-instance 'cl-dot:node
    :attributes `(:label "cell \\N"
                  :shape :box)))

(defmethod cl-dot:object-points-to ((object cons))
  (list (car object)
        (make-instance 'cl-dot:attributed
          :object (cdr object)
          ;; Make the CDR edges more important than the
          ;; CAR edges
          :attributes '(:weight 3))))

;; Symbols
(defmethod cl-dot:object-node ((object symbol))
  (format t "~a~%" object)
  (make-instance 'cl-dot:node
    :attributes `(:label ,object
                  :shape :hexagon
                  :style :filled
                  :color :black
                  :fillcolor "#ccccff")))
)
