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

(defun get-direct-supertypes (object-type)
  (mapcar (lambda (bindings)
            (cut:var-value '?super bindings))
          (cut:force-ll
           (prolog
            `(object-type-direct-subtype ?super ,object-type)))))

(defun compute-applicable-methods-for-specific-type (generic object-type args)
  "Wrapper around `compute-applicable-methods' removing all results that don't
have a eql-specializer at the first position of their specializer list."
  (declare (type function generic)
           (type keyword object-type)
           (type (or list null) args))
  #+sbcl
  (remove-if-not
   (lambda (x) (eql (type-of x) 'sb-mop:eql-specializer))
   (compute-applicable-methods generic (cons object-type args))
   :key (lambda (x) (car (sb-pcl:method-specializers x))))
  #-sbcl
  (error "Function compute-applicable-methods-for-specific-type requires the SBCL compiler."))

;; TODO: Check each supertype, if there are multiple.
(defun find-most-specific-object-type-for-generic (generic object-type &rest args)
  "Find the most specific method of `generic' based on `object-type' while
making sure that the method specializers match with the `args'."
  (declare (type function generic)
           (type keyword object-type))
  (if (compute-applicable-methods-for-specific-type generic object-type args)
      object-type
      (car (mapcar
            (lambda (object-type)
              (apply #'find-most-specific-object-type-for-generic
                     generic
                     object-type
                     args))
            (get-direct-supertypes object-type)))))
