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


(in-package :desig)

(defmacro with-desig-props (props desig &body body)
  "Creates a new lexical environment with props bound as
  variables. The value of the variables corresponds to the value in
  the designator's properties or nil if the property is unbound."
  `(let ,(mapcar (lambda (prop)
                   ;; (check-desig-prop-package prop)
                   `(,prop (cadr (or (and (find ',prop (description ,desig) :key #'car)
                                          (check-desig-prop-package ',prop))
                                     (find ,(intern (string-upcase prop) "KEYWORD")
                                           (description ,desig) :key #'car)))))
          props)
     ,@body))

(defun desig-prop-value (desig prop-name)
  "Returns the first value matching the key `prop-name'."
  (when desig
    (check-desig-prop-package prop-name)
    (cadr (find prop-name (description desig) :key #'car))))

(defun desig-prop-values (designator property-name)
  "Returns the list of all values matching `property-name'."
  (when designator
    (check-desig-prop-package property-name)
    (mapcar #'cadr (remove-if-not (lambda (current-property-name)
                                    (eq current-property-name property-name))
                                  (description designator) :key #'car))))

(defun get-equal-designators (d)
  "Returns the set of all designators that have been equated to `d',
`d' excluded."
  (labels ((collect-eq-desigs (d fun &optional result)
             (if d
                 (collect-eq-desigs (funcall fun d) fun (cons d result))
                 result)))
    (append (collect-eq-desigs (parent d) #'parent)
            (collect-eq-desigs (successor d) #'successor))))
