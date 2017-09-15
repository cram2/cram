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

(defclass beta-join-node ()
  ((connections :reader connections :initform nil
                :documentation "The connections tokens are propagated
                               to (via the method 'propagate').")
   (right :reader beta-node-right :initarg :right
         :documentation "An alpha-memory this node receives input
                         from.")
   (left :reader beta-node-left :initarg :left
          :documentation "A beta node this node receives input
                          from.")
   (right-tokens :reader beta-node-right-tokens :initform nil :initarg :right-tokens
                 :documentation "The memory of right-activations.")
   (left-tokens :reader beta-node-left-tokens :initform nil :initarg :left-tokens
                :documentation "The memory of left-activations.")
   (matches :initform nil
            :documentation "List of newly created (matching)
                            tokens. Is used when WMEs are retracted.")
   (test :reader beta-node-test :initarg :test
         :initform (error "No test specified.")
         :documentation "Function with two parameters, a WME a
                         token. If the function returns a non-nil
                         value, the WME is a valid input for the node
                         and the corresponding token and a new token
                         is created that is passed to child
                         beta-nodes.")))

(defun make-beta-join-node (left-node right-node test-fn)
  "Registers a new beta node at an input beta-node (left-node) and an
   input alpha-node (right-node). The parameter test-fn must be a
   function object receiving two arguments, one WME and a token the
   WME is matched against. The return value is either a representation
   of the WME if the WMA and the token match or nil."
  (let ((new-node (make-instance 'beta-join-node
                    :left left-node
                    :right right-node
                    :test test-fn)))
    ;; register at parent beta memory
    (cond (left-node
           (pushnew new-node (slot-value left-node 'connections))
           (loop for token in (slot-value left-node 'matches)
              do (propagate new-node token :assert)))
          (t
           (setf (slot-value new-node 'left-tokens) (make-token nil))))
    ;; register at alpha-memory-node and read its memory.
    (pushnew new-node (slot-value right-node 'connections))
    (loop for token in (wme-memory right-node)
       do (input new-node token :assert))
    new-node))

(defmethod input ((node beta-join-node) wme operation &key &allow-other-keys)
  (flet ((do-assert ()
           (with-slots (connections left-tokens right-tokens matches test) node
             (push wme right-tokens)
             (let ((new-tokens (reduce (lambda (new-tokens token)
                                         (let ((result (make-token wme token)))
                                           (cond ((funcall test result)
                                                  (mapc (rcurry #'propagate result :assert)
                                                        connections)
                                                  (cons result new-tokens))
                                                 (t
                                                  new-tokens))))
                                       left-tokens
                                       :initial-value nil)))
               (setf matches (nconc new-tokens matches)))))
         (do-retract ()
           (with-slots (connections right-tokens matches) node
             (pop-if! (curry #'eq wme) right-tokens)
             (pop-if! (lambda (match)
                        (when (eq wme (token-wme match))
                          (mapc (rcurry #'propagate match :retract)
                                connections)
                          t))
                      matches))))
    (case operation
      (:assert (do-assert))
      (:retract (do-retract)))))

(defmethod propagate ((node beta-join-node) token operation)
  (flet ((do-assert ()
           (with-slots (connections left-tokens right-tokens matches test) node
             (push token left-tokens)
             (let ((new-tokens (reduce (lambda (new-tokens wme)
                                         (let ((result (make-token wme token)))
                                           (cond ((funcall test result)
                                                  (mapc (rcurry #'propagate result :assert)
                                                        connections)
                                                  (cons result new-tokens))
                                                 (t
                                                  new-tokens))))
                                       right-tokens
                                       :initial-value nil)))
               (setf matches (nconc new-tokens matches)))))
         (do-retract ()
           (with-slots (connections left-tokens matches) node
             (pop-if! (curry #'eq token) left-tokens)
             (pop-if! (lambda (match)
                        (when (eq token (token-parent match))
                          (mapc (rcurry #'propagate match :retract)
                                connections)
                          t))
                      matches))))
    (case operation
      (:assert (do-assert))
      (:retract (do-retract)))))

(defmethod gc-node ((node beta-join-node))
  (with-slots (connections left right) node
    (when (null connections)
      (when left
        (setf (slot-value left 'connections)
              (remove node (slot-value left 'connections)))
        (gc-node left))
      (when right
        (setf (slot-value right 'connections)
              (remove node (slot-value right 'connections)))
        (gc-node right)))))
