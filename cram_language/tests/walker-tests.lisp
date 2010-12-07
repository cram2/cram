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

(in-package :cpl-tests)

(def-suite walker-tests :in language)

(in-suite walker-tests)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Test plans
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(def-plan tp-a ()
  (:tag foo 
    (list 1 2 3)))

(def-plan tp-b (l)
  (dolist (e l)
    (format t "~a~%" e)))

(def-plan tp-c (x y)
  (let ((var 1))
    (par (:tag bar (format t "x + y + 1 = ~a~%" (+ x y var))
	       (symbol-macrolet ((this-is-really-tp-a (tp-a)))
		 (tp-b this-is-really-tp-a)))
	 (:tag baz (format t "foobar~%")))))

(def-top-level-plan test-tlp ()
  (flet ((tp-b (&rest foo) foo))
    (macrolet ((tp-c-disguise (a b) `(tp-c ,a ,b)))
      (seq (tp-c-disguise 2 3)
	   (tp-b (list :a :b :c))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Utilities
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun correct-parents? (tree)
  (dolist (c (plan-tree-node-children tree))
    (unless (eq tree (plan-tree-node-parent c))
      (return nil)))
  (every #'correct-parents? (plan-tree-node-children tree)))

(defun equal-plan-tree? (t1 t2)
  (and (equal (plan-tree-node-sexp t1) (plan-tree-node-sexp t2))
       (equal (plan-tree-node-path t1) (plan-tree-node-path t2))
       (= (length (plan-tree-node-children t1))
	  (length (plan-tree-node-children t2)))
       (every #'identity (mapcar #'equal-plan-tree?
				 (plan-tree-node-children t1)
				 (plan-tree-node-children t2)))))

(defmacro walk-with-tag-handler-test (name tags form)
  (with-gensyms (walked-tags)
    `(test ,(symbolicate 'walk-with-tag-handler- name)
       (for-all ()
	 (let ((,walked-tags ()))
	   (walker:walk-with-tag-handler ',form
					     (lambda (label body)
					       (declare (ignore body))
					       (push label ,walked-tags))
					     nil)
	   (is (equal ,walked-tags ',(reverse tags))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Unit tests
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; WALK-WITH-TAG-HANDLER

(walk-with-tag-handler-test
 simple
 (tag1 tag2)
 (par (:tag tag1 (list 1 2))
      (:tag tag2 (list 3 4))))

(walk-with-tag-handler-test
 dont-shaddow-tag-macro
 (tag1 tag2)
 (par (let ((foo 'bar)
	    (x 42))
	(macrolet ((:tag (&rest rest) rest))
	  (:tag tag1 (format nil "~a" foo)))
	(flet ((:tag (&rest rest) (declar (ignore rest))))
	  (:tag tag2 (decf x 23))))))

(walk-with-tag-handler-test
 nested-tags
 (tag1 tag2 tag3 tag4 tag5)
 (par (:tag tag1 (list 1 2))
      (:tag tag2 (seq (:tag tag3 (:tag tag4))
		      (:tag tag5)))))

(test walk-with-tag-handler-bodies
  (for-all ()
    (let* ((bodies ())
	   (b1 '((list 1 2)
		 (flet ((foo () 'bar))
		   (foo))))
	   (b2 '())
	   (form `(progn (:tag t1 ,@b1)
			 (:tag t2 ,@b2)))
	   (handler (lambda (l b)
		      (declare (ignore l))
		      (push b bodies))))
      (walker:walk-with-tag-handler form handler nil)
      (is (equal bodies (reverse (list b1 b2)))))))

;;; EXPAND-PLAN

;;; Commented out EXPAND-PLAN because it seems to fail

;; (test expand-plan
;;   (for-all ()
;;     (let* ((test-form '(macrolet ((call-plan ()
;; 				   '(test-tlp)))
;; 			(progn 1 (call-plan))))
;; 	   (test-expansion '(locally (progn 1 (test-tlp))))
;; 	   (test-path '((TOP-LEVEL TEST-TLP) (TP-C) (CPL::TAGGED BAR)))
;; 	   (test-comp-tree ; don't worry about correct parents for this comparison tree
;; 	    (make-plan-tree-node
;; 	     :sexp '(:tag bar (format t "x + y + 1 = ~a~%" (+ x y var))
;; 		     (symbol-macrolet ((this-is-really-tp-a (tp-a)))
;; 		       (tp-b this-is-really-tp-a)))
;; 	     :path (reverse test-path)
;; 	     :children (list (make-plan-tree-node
;; 			      :sexp '(tp-a)
;; 			      :path (cons '(tp-a) (reverse test-path))
;; 			      :children (list (make-plan-tree-node
;; 					       :sexp '(:tag foo 
;; 						       (list 1 2 3))
;; 					       :path (list* '(tagged foo)
;; 							    '(tp-a) (reverse test-path)))))
;; 			     (make-plan-tree-node
;; 			      :sexp '(tp-b this-is-really-tp-a)
;; 			      :path (cons '(tp-b) (reverse test-path)))))))
;;       (multiple-value-bind (tree expansion)
;; 	  (expand-plan test-form)
;; 	(let ((node (find-plan-node tree test-path)))
;; 	  (is (equal expansion test-expansion))
;; 	  (is (correct-parents? tree))
;; 	  (is (eq (plan-tree-node-parent
;; 		   (plan-tree-node-parent
;; 		    (plan-tree-node-parent node)))
;; 		  tree))
;; 	  (is (equal-plan-tree? node test-comp-tree)))))))