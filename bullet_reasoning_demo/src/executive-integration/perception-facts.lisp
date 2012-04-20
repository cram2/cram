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

(in-package :btr)

(defun has-cop-property (cop-object property)
  "Checks if `cop-object' contains a cop property `property'. Doesn't
  respect package names but compairs symbol names."
  (declare (type perception-pm:cop-perceived-object cop-object)
           (type symbol property))
  (find (symbol-name property) (perception-pm:object-properties cop-object)
        :key (compose #'symbol-name #'cadr)
        :test #'equal))

(defun cop-properties-equal (obj-1 obj-2)
  (declare (type perception-pm:cop-perceived-object obj-1 obj-2))
  (equal (perception-pm:object-properties obj-1)
         (perception-pm:object-properties obj-2)))

(def-fact-group perception-facts ()
  ;; We can replace clusters by other objects but not the other way
  ;; around.
  (<- (object-replacable ?w ?old ?new)
    (%object ?w ?old ?old-obj)
    (lisp-fun get-perceived-object ?old-obj ?old-po)
    (lisp-type ?old-po perception-pm:cop-perceived-object)
    (lisp-pred has-cop-property ?old-po cluster)
    (format "old is cluster ~%"))

  (<- (object-replacable ?w ?old ?new)
    (%object ?w ?old ?old-obj)
    (%object ?w ?new ?new-obj)
    (lisp-fun get-perceived-object ?old-obj ?old-po)
    (lisp-fun get-perceived-object ?new-obj ?new-po)    
    (lisp-type ?old-po perception-pm:cop-perceived-object)
    (lisp-type ?new-po perception-pm:cop-perceived-object)
    (lisp-pred cop-properties-equal ?old-po ?new-po)
    (format "equal ~%")))
