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

(defun minimum (seq &key (test #'<) (key #'identity))
  (extremum seq test :key key))

(defun maximum (seq &key (test #'>) (key #'identity))
  (extremum seq test :key key))

(defun style-warn (datum &rest arguments)
  (apply #'sb-int:style-warn datum arguments))

(defun compare (x y &key (test #'eql) (key #'identity))
  (funcall test (funcall key x) (funcall key y)))

(defun execute-string (string &key filename (cleanup t))
  "Compiles and executes the lisp code given by `string'. If
`filename' is set, uses it for compilation. If cleanup is non-NIL,
removes deletes the file and the compiled file after loading it."
  (declare (type string string)
           (type (or string pathname null) filename)
           (type (or null t) cleanup))
  (let ((filename (pathname
                   (concatenate
                    'string
                    (or filename
                        (sb-posix:mktemp "/tmp/execute-from-string-XXXXXX"))
                    ".lisp")))
        (compiled-file nil))
    (unwind-protect
         (progn
           (with-open-file (stream filename :direction :output)
             (write-string string stream))
           (setf compiled-file (compile-file filename))
           (load compiled-file))
      (when cleanup
        (delete-file filename)
        (when compiled-file
          (delete-file compiled-file))))))

;; NOTE(winkler): This macro is a default implementation for the
;; prediction-supporting `choose' macro. Once the prediction-package
;; gets loaded, this macro gets overwritten with added
;; functionality. The below implementation is here to support the
;; signature.
(defmacro choose (tag &key body parameters generators &allow-other-keys)
  `(:tag ,tag
     (let ((generated-param-hash-table (make-hash-table)))
       (labels ((generate-parameters ()
                  ,@(loop for (variables generator) in generators
                          collect
                          `(let ((generated-values ,generator))
                             ,@(loop for i from 0 below (length variables)
                                     as variable = (nth i variables)
                                     collect `(setf (gethash
                                                     ',variable
                                                     generated-param-hash-table)
                                                    (nth ,i generated-values)))))))
         (generate-parameters)
         (let* ,(mapcar (lambda (parameter)
                          `(,parameter (gethash
                                        ',parameter
                                        generated-param-hash-table)))
                 parameters)
           ,body)))))



(defun equalize-two-list-lengths (first-list second-list)
  "Returns two lists of equal length as VALUES.
To achieve equal length appends NILs at the end of the shorter of `first-list' and `second-list'."
  (let* ((first-length (length first-list))
         (second-length (length second-list))
         (max-length (max first-length second-length)))
    (values
     (if (> max-length first-length)
        (append first-list (make-list (- max-length first-length)))
        first-list)
     (if (> max-length second-length)
        (append second-list (make-list (- max-length second-length)))
        second-list))))

(defun equalize-lists-of-lists-lengths (first-list-of-lists second-list-of-lists)
  "Equalizes the length of lists inside of lists. E.g.:
 ((1) (2 3) (4)) and ((a b) (c)) becomes ((1 NIL) (2 3) (4)) and ((a b) (c NIL) NIL)."
  (let ((max-length (max (length first-list-of-lists)
                         (length second-list-of-lists)))
        first-result-l-of-ls second-result-l-of-ls)

   (loop for i from 0 to (1- max-length)
         do (let ((first-list (nth i first-list-of-lists))
                  (second-list (nth i second-list-of-lists)))
              (multiple-value-bind (first-equalized second-equalized)
                  (equalize-two-list-lengths first-list second-list)
                (setf first-result-l-of-ls
                      (append first-result-l-of-ls first-equalized)
                      second-result-l-of-ls
                      (append second-result-l-of-ls second-equalized)))))

   (values first-result-l-of-ls
           second-result-l-of-ls)))
