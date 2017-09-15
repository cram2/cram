;;;
;;; Copyright (c) 2016, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :desig)

(defun variable-value-or-keyword (a-symbol)
  (if (symbolp a-symbol)
      (if (boundp a-symbol)
          (if (equal (symbol-value a-symbol) a-symbol)
              a-symbol                  ; it's a keyword
              (variable-value-or-keyword (symbol-value a-symbol))) ; it's a special var
          (if (eql (aref (string-upcase a-symbol) 0) #\?)
              a-symbol      ; it's a variable (starts with ?, e.g. ?x)
              (intern (string-upcase a-symbol) :keyword))) ; it's a simple symbol
      a-symbol)) ; it's not a symbol but a number or some other actual value

(defun parse-key-value-pairs (key-value-pairs)
  (labels ((parse (key-value-pair-list)
             (if (listp key-value-pair-list)
                 (cond (;; new designator construct
                        (and (symbolp (first key-value-pair-list))
                             (member (intern (string-upcase (first key-value-pair-list))
                                             :keyword)
                                     '(:a :an :all :some :the)))
                        (let ((quantifier (first key-value-pair-list))
                              (type-symbol (second key-value-pair-list))
                              (key-value-pairs-evaluated (cddr key-value-pair-list)))
                          `(,quantifier ,type-symbol ,@key-value-pairs-evaluated)))
                       ;; WHEN construct: (WHEN ?x (key ?x))
                       ((and (symbolp (first key-value-pair-list))
                             (eq (first key-value-pair-list) 'desig:when)
                             (eq (length key-value-pair-list) 3))
                        `(when ,(variable-value-or-keyword (second key-value-pair-list))
                           ,@(loop for key-value-pair in (cddr key-value-pair-list)
                                   collecting (parse key-value-pair))))
                       ;; standard key-value pair
                       (t
                        `(list ,@(loop for key-value-pair in key-value-pair-list
                                       collecting (parse key-value-pair)))))
                 (if (symbolp key-value-pair-list)
                     ;; we're at a leaf
                     (variable-value-or-keyword key-value-pair-list)
                     key-value-pair-list))))
    (parse key-value-pairs)))

(defmacro a (type &rest key-value-pairs-list)
  (let ((type-keyword (intern (string-upcase type) :keyword)))
    `(make-designator ,type-keyword
                      (remove NIL ,(parse-key-value-pairs key-value-pairs-list)))))

(defmacro an (&rest body)
  `(a ,@body))

(defmacro all (type &rest key-value-pairs-list)
  (let ((type-keyword (intern (string-right-trim '(#\S) (string-upcase type)) :keyword))
        (local-var-name (gensym "DESIG")))
    `(let ((,local-var-name
             (make-designator ,type-keyword
                              (remove NIL ,(parse-key-value-pairs key-value-pairs-list)))))
       (setf (slot-value ,local-var-name 'quantifier) :all)
       ,local-var-name)))

;; (defmacro some (&rest body)
;;   `(a ,@body))
;;
;; (defmacro the (&rest body)
;;   `(a ,@body))
;;
;; CAN'T BE USED, IT'S A COMMON LISP SPECIAL FORM


;; Example:
;;
;; (an action (to open)
;;            (location (the location (bla bla)
;;                                    (and (an object (another thing)
;;                                                    (kind bla))))))
