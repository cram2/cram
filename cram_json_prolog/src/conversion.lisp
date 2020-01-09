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

(in-package :json-prolog)

(defun prologify (s)
  (flet ((contains-lower-case-char (symbol)
           (and 
            (find-if (lambda (ch)
                       (let ((lch (char-downcase ch)))
                         (and (find lch "abcdefghijklmnopqrstuvwxyz")
                              (eq lch ch))))
                     (symbol-name symbol))
            t)))
    (if (contains-lower-case-char s)
        (string s)
        (string-downcase (substitute #\_ #\- (copy-seq (string s)))))))

(defun lispify (s)
  (string-upcase (string s)))

(defun replace-all (new old str)
  (with-output-to-string (out)
    (let ((pos (search old str)))
      (cond (pos
             (write-string (subseq str 0 pos) out)
             (write-string new out)
             (write-string (replace-all new old (subseq str (+ pos (length old)))) out))
            (t (write-string str out)))
      out)))

(defun escape-quotes (str)
  (replace-all "\\'" "'" str))

(defun unescape-string (str)
  (remove "\\" (copy-seq str)))

(defun jsonify-complex-type (exp &key prologify)
  (prolog->query-string (gensym (symbol-name (type-of exp))) :prologify prologify))

(defun prologify-complex-type (exp)
  (typecase exp
    (symbol (symbol-name exp))
    (t exp)))

(defun replace-complex-types (exp)
  (mapcar (lambda (e)
            (typecase e
              (list (replace-complex-types e))
              (string (prologify-complex-type e))
              (symbol (prologify-complex-type e))
              (t e)))
          exp))

(defun prolog->query-string (exp &key prologify)
  "Recursively walks exp and converts every lisp-expression
   into a simple prolog query representation."
  (when exp
    (typecase exp
      (string (format nil "'~a'" exp))
      (number (write-to-string exp))
      (symbol
         (cond ((is-var exp)
                (when (find #\- (symbol-name exp))
                  (error 'simple-error
                         :format-control
                         (concatenate 'string "Variable name `~a' invalid. "
                                      "For prolog, it must not contain `-' characters.")
                         :format-arguments (list exp)))
                (subseq (symbol-name exp) 1))
               (t (if prologify
                      (escape-quotes (prologify exp))
                      (escape-quotes (symbol-name exp))))))
      (list
       (cond ((eq (car exp) 'and)
              (format nil "~{~A~^, ~}" (mapcar 'prolog->query-string (cdr exp))))
             ((eq (car exp) 'or)
              (format nil "~{~A~^; ~}" (mapcar 'prolog->query-string (cdr exp))))
             ((eq (car exp) 'quote)
              (format nil "[~{~A~^, ~}]" (mapcar 'prolog->query-string (cadr exp))))
             ((eq (car exp) 'list)
              (format nil "[~{~A~^, ~}]" (mapcar 'prolog->query-string (cdr exp))))
             ((listp (car exp))
              (format nil "[~{~A~^, ~}]" (mapcar 'prolog->query-string exp)))
             (t
              (format nil "~a(~{~A~^, ~})" (car exp) (mapcar 'prolog->query-string (cdr exp)))))) 
      (t (jsonify-complex-type exp :prologify prologify)))))

(defun json->prolog (exp &key (lispify nil) (package *package*))
  "Converts a json encoded string into its lisp prolog
  representation."
  (flet ((map-operators (str)
           (string-case str
             (";" "OR")
             ("," "AND")
             (t str))))
    (when exp
      (typecase exp
        (string (let ((exp (map-operators (unescape-string exp))))
                  (if lispify
                      (intern (lispify exp) package)
                      (intern (concatenate 'string "'" exp "'")
                              package))))
        (number exp)
        (list (mapcar (alexandria:rcurry #'json->prolog :lispify lispify) exp))
        (hash-table
           (let ((list (gethash "list" exp))
                 (term (gethash "term" exp))
                 (var (gethash "variable" exp)))
             (cond (list (list 'quote (json->prolog list :lispify lispify)))
                   (term (json->prolog term :lispify lispify))
                   (var (intern (concatenate 'string "?" var) package)))))))))

(defun json-bdgs->prolog-bdgs (bdgs-str &key (lispify nil) (package *package*))
  (loop for var being the hash-keys in (yason:parse bdgs-str)
        using (hash-value bdg)
        collecting (cons (intern (concatenate 'string "?" var) package)
                         (json->prolog bdg :lispify lispify :package package))))





