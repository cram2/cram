;;;
;;; Copyright (c) 2019, Christopher Pollok <cpollok@uni-bremen.de>
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

(in-package :man-int-tests)

(def-fact-group test-hierarchy (man-int:object-type-direct-subtype)
  (<- (man-int:object-type-direct-subtype :thing :specific-thing))
  (<- (man-int:object-type-direct-subtype :specific-thing :very-specific-thing))
  (<- (man-int:object-type-direct-subtype :thing :other-thing)))

(define-test find-most-specific-object-type-for-generic-same-type
  (defgeneric generic (object-type a)
    (:method ((object-type (eql :specific-thing)) a) nil))
  (assert-eql
   :specific-thing
   (man-int::find-most-specific-object-type-for-generic #'generic :specific-thing 1))
  (assert-nil
   (man-int::find-most-specific-object-type-for-generic #'generic :unrelated-thing 1))
  (fmakunbound 'generic))

(define-test find-most-specific-object-type-for-generic-parent-type
  (defgeneric generic (object-type a))
  (defmethod generic ((object-type (eql :thing)) a) nil)
  (assert-eql
   :thing
   (man-int::find-most-specific-object-type-for-generic #'generic :specific-thing 1))
  (assert-eql
   :thing
   (man-int::find-most-specific-object-type-for-generic #'generic :very-specific-thing 1))
  (defmethod generic ((object-type (eql :specific-thing)) a) nil)
  (assert-eql
   :specific-thing
   (man-int::find-most-specific-object-type-for-generic #'generic :specific-thing 1))
  (fmakunbound 'generic))

(define-test find-most-specific-object-type-for-generic-different-heritage
  (defgeneric generic (object-type a)
    (:method ((object-type (eql :thing)) a) nil)
    (:method ((object-type (eql :specific-thing)) a) nil))
  (assert-eql
   :specific-thing
   (man-int::find-most-specific-object-type-for-generic #'generic :specific-thing 1))
  (assert-eql
   :thing
   (man-int::find-most-specific-object-type-for-generic #'generic :other-thing 1))
  (fmakunbound 'generic))

(define-test find-most-specific-object-type-for-generic-no-additional-args
  (defgeneric generic (object-type)
    (:method ((object-type (eql :thing))) nil))
  (assert-eql
   :thing
   (man-int::find-most-specific-object-type-for-generic #'generic :thing))
  (fmakunbound 'generic))

(define-test find-most-specific-object-type-for-generic-specific-additional-args
  (defgeneric generic (object-type a)
    (:method ((object-type (eql :thing)) (a (eql 1))) nil))
  (assert-eql
   :thing
   (man-int::find-most-specific-object-type-for-generic #'generic :thing 1))
  (assert-nil
   (man-int::find-most-specific-object-type-for-generic #'generic :thing 2))
  (fmakunbound 'generic))
