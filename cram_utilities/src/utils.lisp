;;;
;;; Copyright (c) 2009, Lorenz Moesenlechner <moesenle@cs.tum.edu>,
;;;                     Nikolaus Demmel <demmeln@cs.tum.edu>
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


(in-package :cut)

(defun map-tree (fun tree)
  "Traverses a cons cell tree and applies fun to every car, aswell as
   every cdr that is not nil. fun is never called for cons cells as
   they are decended into."
  (cond ((null tree)
         nil)
        ((atom tree)
         (funcall fun tree))
        (t
         (cons (map-tree fun (car tree))
               (map-tree fun (cdr tree))))))

(defmacro pop-if! (pred lst)
  "Destructively modifies the sequence. The first item for which pred
  returns a non-nil value is removed."
  (with-gensyms (pred-sym)
    `(labels ((worker (pred lst)
                (when (cdr lst)
                  (if (funcall pred (cadr lst))
                      (setf (cdr lst) (cddr lst))
                      (worker pred (cdr lst))))))
       (let ((,pred-sym ,pred))
         (when ,lst
           (if (funcall ,pred-sym (car ,lst))
               (if (cdr ,lst)
                   (setf (car ,lst) (cadr ,lst)
                         (cdr ,lst) (cddr ,lst))
                   (setf ,lst nil))
               (worker ,pred-sym ,lst))))
       ,lst)))

(defun function-bound-feature (name package)
  "Generate a form suitable for testing with #+."
  (let* ((name (string name))
         (package (find-package (string package)))
         (symbol (and package (find-symbol name package))))
    (if (and package
             symbol
             (fboundp symbol))
        '(:and)
        '(:or))))

(defun flip (fun)
  "Returns a function object that flips the first two parameters and
   applies the rest of the parameters."
  (lambda (x y &rest r)
    (apply fun y x r)))

(defun ensure-unused-string (base-string used-p)
  "Appends increasing numbers to `base-string' until one is found where
  `used-p' returns NIL. Can be used to generate unique filenames."
  (if (funcall used-p base-string)
      (loop for n from 1
         for new-str = (concatenate 'string base-string (format nil "-~d" n))
         when (not (funcall used-p new-str)) do (return new-str))
      base-string))

(defun sanitize-filename (filename)
  "Conservatively replaces every character but alphanumerics, '-' and '_' by
   '_' to ensure the filename is valid in the filesystem (unix)."
  (substitute-if-not #\_ (lambda (char)
                           (let ((c (char-code char)))
                             (or (and (>= c (char-code #\a)) (<= c (char-code #\z)))
                                 (and (>= c (char-code #\A)) (<= c (char-code #\Z)))
                                 (and (>= c (char-code #\0)) (<= c (char-code #\9)))
                                 (= c (char-code #\-))
                                 (= c (char-code #\_)))))
                     filename))

(defun extremum (seq &key test (key #'identity))
  (assert test)
  (reduce (lambda (prev curr)
            (if (funcall test (funcall key curr) (funcall key prev))
                curr
                prev))
          seq))

(defun minimum (seq &key (test #'<) (key #'identity))
  (extremum seq :test test :key key))

(defun maximum (seq &key (test #'>) (key #'identity))
  (extremum seq :test test :key key))

(defun style-warn (datum &rest arguments)
  (apply #'sb-int:style-warn datum arguments))
