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

(defgeneric input (node wme operation &key &allow-other-keys)
  (:documentation "Processes a working memory entry. 'unmatched'
                   contains all unmatched data of the wme."))

(defgeneric propagate (beta-node token operation)
  (:documentation "Processes a token, coming from a
                   left-activation. Operation is either :assert
                   or :retract."))

(defgeneric gc-node (node)
  (:documentation "Removes nodes in the network that are no longer used.")
  (:method (node) nil))

(defgeneric object-id (obj)
  (:documentation "Returns an identifier that is used to identify the
  object. Object ids that are EQL indicate that objects are
  equal. This can be used to state that two tokens are equal although
  their references are not EQL.")
  (:method ((obj t))
    obj))

(defgeneric clear-facts (node)
  (:documentation "Retract all facts in order to clear the network."))

(defvar *prolog-facts* (make-hash-table :test 'eq))

(defmacro def-prolog-fact (name &rest pattern)
  `(setf (gethash ',name *prolog-facts*) ',pattern))

(defun find-prolog-fact (fact)
  (let ((pl-fact (gethash (car fact) *prolog-facts*)))
    (and pl-fact (eql (length fact) (length pl-fact)))))

(defmacro with-facts-asserted (fact-assertions &body body)
  "Executes body with facts asserted. `fact-assertions' is a list of
  facts that are to be asserted. After body finishes, the facts are
  retracted again."
  `(unwind-protect
        (progn
          ,@(loop for fact in fact-assertions
               collecting `(rete-assert ,fact))
          ,@body)
     ,@(loop for fact in fact-assertions
          collecting `(rete-retract ,fact))))
