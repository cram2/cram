;;; Copyright (c) 2013, Georg Bartels <georg.bartels@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
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

(in-package :pr2-manipulation-process-module)

(defun get-knowrob-global-id-symbol (knowrob-namespace knowrob-symbol)
  "Queries knowrob for the global knowrob-id of 'knowrob-symbol' with 'knowrob-namespace', both represented as strings. The output is returned as a symbol."
  (declare (type string knowrob-namespace knowrob-symbol))
  (let* ((combined-name (concatenate 'string 
                                     knowrob-namespace ":" knowrob-symbol))
         (query-string (concatenate 'string
                                    "rdf_global_id(" combined-name ", C)"))
         
         (bindings (json-prolog:prolog-simple query-string)))
    (unless bindings
      (error
       'simple-error
       :format-control "No bindings returned when asking for global knowrob id. Namespace: ~a, symbol: ~a."
       :format-arguments '(knowrob-namespace knowrob-symbol)))
    (var-value '?c (lazy-car bindings))))

(defun knowrob-symbol->string (knowrob-symbol &optional (remove-quotes t))
  "Takes a 'knowrob-symbol' as typically returned when asking knowrob through json-prolog-client and returns the equivalent string. If remove-quotes is not NIL, the first and last character of the name of the symbol will be removed."
  (declare (type symbol knowrob-symbol))
  (let ((long-symbol-name (symbol-name knowrob-symbol)))
    (unless (> (length long-symbol-name) 1)
      (error
       'simple-error
       :format-control "Asked to remove quote symbols from a string with less than 2 symbols. String: ~a"
       :format-arguments '(long-symbol-name)))
    (if remove-quotes
        (subseq long-symbol-name 1 (- (length long-symbol-name) 1))
        long-symbol-name)))

(defun get-knowrob-motion-phases (namespace action-symbol)
  "Queries knowrob for motion phases of action 'action-symbol' situated in knowrob namespace 'namespace'. Returns the actual prolog bindings."
  (json-prolog:prolog 
   `("plan_subevents" ,(knowrob-symbol->string 
                        (get-knowrob-global-id-symbol
                         namespace
                         action-symbol))
                      ?p)))

(defun get-knowrob-motion-constraint-infos (motion-phase)
  (let ((phase
          (cond
            ((symbolp motion-phase) (knowrob-symbol->string motion-phase))
            ((stringp motion-phase) motion-phase)
            (t (error 'simple-error
                      :format-control "Input parameter 'motion-phase' was neither of type string nor a symbol."
                      :format-arguments '(motion-phase))))))
    (json-prolog:prolog
     `(and
       ("motion_constraint" ,phase ?c)
       ("constraint_properties" ?c ?type ?toolFeature ?worldFeature
                                ?weight ?lower ?upper ?minVel ?maxVel)))))