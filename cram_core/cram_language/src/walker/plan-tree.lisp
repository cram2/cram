;;;
;;; Copyright (c) 2009, Nikolaus Demmel <demmeln@cs.tum.edu>
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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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

(in-package :walker)

(defstruct plan-tree-node
  (sexp nil)      ; The sexp of the _call_ to the plan as found in the walked
                  ; source code.
  (path nil)      ; Full path of the plan of that node.
  (parent nil)    ; NIL for the root of the tree.
  (children nil)) ; list of plan-tree-nodes.

;;; Define our own pretty print method, to avoid infinit recursion when
;;; *print-circle* is nil. It is similar to the general struct pretty printer
;;; but doesn't print the parent slot.
(defmethod print-object ((object plan-tree-node) stream) 
  (format stream "~<#(~;~W ~:@_:SEXP ~@_~W~:@_:PATH ~@_~W~:@_:CHILDREN ~@_~W~;)~:>"
          (list 'plan-tree-node
                (plan-tree-node-sexp object)
                (plan-tree-node-path object)
                (plan-tree-node-children object))))

(defun find-plan-node (plan-tree path)
  "Return the plan-tree-node specified by the path. Returns nil, if the path
   is not valid."
  (when plan-tree
    (if (null path)
        plan-tree
        (let ((last-path-elem (car (last path))))
          (dolist (x (plan-tree-node-children plan-tree) nil)
            (when (equal last-path-elem
                         (car (plan-tree-node-path x)))
              (return (find-plan-node x (butlast path)))))))))
