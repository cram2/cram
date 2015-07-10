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

(defclass alpha-node ()
  ((parent :reader alpha-node-parent :initarg :parent)
   (children :initform (make-hash-table))
   (key :reader alpha-node-key :initarg :key)))

(defmethod clear-facts ((node alpha-node))
  (maphash (lambda (a b)
             (declare (ignore a))
             (clear-facts b))
           (slot-value node 'children))
  (clrhash (slot-value node 'children)))

(defmethod gc-node ((node alpha-node))
  (when (and (eql (hash-table-count (slot-value node 'children)) 0)
             (not (null (alpha-node-parent node))))
    (remhash (object-id (alpha-node-key node))
             (slot-value (alpha-node-parent node) 'children))
    (gc-node (alpha-node-parent node))))

;;; It might seem strange that we rebind 'wme' here and pass it to a
;;; wildcard node. But this assures that the callbacks receive a node
;;; that represents the pattern and uniquely identifies it even with
;;; the eq test.  If we would use the original wme represented by a
;;; list, we needed the 'equal' test that has runtime complexity
;;; linear in the size of the pattern. I guess that using the memory
;;; node of the pattern without any wildcards should be the most
;;; elegant solution and save most calculation resources.
(defmethod input ((node alpha-node) wme operation &key (unmatched wme))
  (assert unmatched)
  (assert (or (eq operation :assert)
              (eq operation :retract)))
  (destructuring-bind (key . unmatched) unmatched
    (assert (not (is-unnamed-var key)))
    (let* ((child (alpha-node-matching-child node wme key (not unmatched)))
           (wme (input child wme operation :unmatched unmatched))
           (wildcard-child (alpha-node-matching-child node wme '? (not unmatched))))
      (input wildcard-child wme operation :unmatched unmatched))))
