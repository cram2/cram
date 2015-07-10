;;;
;;; Copyright (c) 2009, Lorenz Moesenlechner <moesenle@cs.tum.edu>
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


(in-package :prolog)

(defclass alpha-memory-node (alpha-node)
  ((pattern :reader alpha-memory-node-pattern :initarg :pattern)
   (wme-memory :initform nil :reader wme-memory)
   (connections :reader connections :initform nil
                :documentation "List of connections input is called on
                                when this node is reached. This is the
                                point where beta nodes can connect
                                to.")))

(defmethod clear-facts ((node alpha-memory-node))
  (loop for wme in (wme-memory node)
        do (input node wme :retract)))

(defmethod gc-node ((node alpha-memory-node))
  (when (and (null (wme-memory node))
             (null (slot-value node 'connections)))
    (call-next-method)))

(defmethod input ((node alpha-memory-node) wme operation &key unmatched)
  (cond (unmatched
         (call-next-method))
        (t
         (let ((wme (typecase wme
                      (alpha-node wme)
                      (list node))))
           (case operation
             (:assert
              (push wme (slot-value node 'wme-memory)))
             (:retract
              (assert (wme-memory node) nil "wme-memory already empty. Cannot retract.")
              (pop-if! (curry #'eq wme) (slot-value node 'wme-memory))
              (gc-node node))
             (t
              (error "Operation `~a' unknown." operation)))
           (loop for connection in (slot-value node 'connections)
              do (input connection wme operation))
           wme))))
