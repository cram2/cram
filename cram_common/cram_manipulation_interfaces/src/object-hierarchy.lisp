;;;
;;; Copyright (c) 2018, Christopher Pollok <cpollok@cs.uni-bremen.de>
;;;                     Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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

(in-package :cram-manipulation-interfaces)

(defun probe-sbcl (generic object-type)
  #+sbcl
  (let ((methods (sb-pcl:generic-function-methods generic)))
    (find object-type
          methods
          :key (lambda (x)
                 (car (sb-pcl:method-specializers x)))
          :test (lambda (x y)
                  (when (eql (type-of y) 'sb-mop:eql-specializer)
                    (eql
                     (sb-mop:eql-specializer-object y)
                     x)))))
  #-sbcl
  (error "Function PROBE-SBCL requires the SBCL compiler."))

(defun get-direct-supertypes (object-type)
  (mapcar (lambda (bindings)
            (cut:var-value '?super bindings))
          (cut:force-ll
           (prolog
            `(object-type-direct-subtype ?super ,object-type)))))

(defun find-most-specific-object-type-for-generic (generic object-type)
  "Find the most specific method of `generic' based on `object-type'."
  (if (probe-sbcl generic object-type)
      object-type
      (car (mapcar
            (alexandria:curry #'find-most-specific-object-type-for-generic generic)
            (get-direct-supertypes object-type)))))
