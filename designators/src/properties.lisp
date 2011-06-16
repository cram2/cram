;;;
;;; Copyright (c) 2011, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;

(in-package :cl-user)

(defpackage cram-designator-properties
  (:use #:common-lisp)
  (:nicknames desig-props)
  (:export #:def-desig-package))

(defmacro desig-props:def-desig-package (name &body options)
  "Defines a package that uses some symbols in designator
properties. This macro is just a thin wrapper around DEFPACKAGE and
supports the same `options'. In addition, the
option (:DESIG-PROPERTIES [sym]*) can be specified to indicate that a
symbol `sym' should be used as a designator property. 

Example:

\(desig-props:def-desig-package foo
   (:use :cl)
   (:export bar)
   (:desig-properties baz xy)\)"
  (flet ((get-package-options (opts)
           (let ((package-options '(:nicknames :documentation :use :shadow :shadowing-import-from
                                    :import-from :export :intern :size)))
             (remove-if-not
              (lambda (opt) (member (car opt) package-options))
              opts))))
    (let ((prop-package (find-package :cram-designator-properties))
          (prop-syms (cdar (member :desig-properties options :key #'car))))
      `(eval-when (:compile-toplevel :load-toplevel :execute)
         ,(when prop-syms
            `(export ',(mapcar (lambda (sym) (intern (symbol-name sym) prop-package))
                               prop-syms)
                     ,prop-package))
         (defpackage ,name
           ,@(get-package-options options)
           ,@(when prop-syms
               `((:shadowing-import-from cram-designator-properties
                                         ,@prop-syms))))))))
