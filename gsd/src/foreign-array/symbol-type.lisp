;; Symbol-type declaration
;; Liam Healy 2009-12-20 22:27:54EST symbol-type.lisp
;; Time-stamp: <2010-06-19 18:46:52EDT symbol-type.lisp>
;;
;; Copyright 2009 Liam M. Healy
;; Distributed under the terms of the GNU General Public License
;;
;; This program is free software: you can redistribute it and/or modify
;; it under the terms of the GNU General Public License as published by
;; the Free Software Foundation, either version 3 of the License, or
;; (at your option) any later version.
;;
;; This program is distributed in the hope that it will be useful,
;; but WITHOUT ANY WARRANTY; without even the implied warranty of
;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;; GNU General Public License for more details.
;;
;; You should have received a copy of the GNU General Public License
;; along with this program.  If not, see <http://www.gnu.org/licenses/>.

;;; An "st" or symbol-type is a list (symbol type) where
;;; type could be (element-type array-dim).  These are examples of lists
;;; of sts: 
 ;; ((#:RET3500 SF-RESULT))
 ;; ((#:RET3501 (:DOUBLE (- NMAX NMIN)))) 
 ;; ((#:RET3502 (:DOUBLE (1+ KMAX))) (#:RET3503 (:DOUBLE (1+ KMAX)))
 ;;  (#:RET3504 (:DOUBLE (1+ KMAX))) (#:RET3505 (:DOUBLE (1+ KMAX)))
 ;;  (#:RET3506 :DOUBLE) (#:RET3507 :DOUBLE))

(in-package :grid)

(export
 '(make-st st-symbol st-type st-pointerp st-actual-type
   st-pointer-generic-pointer))

(defun make-st (symbol type)
  (list symbol type))

(defun st-symbol (decl)
  (first decl))

(defun st-type (decl)
  (second decl))

(defun st-pointerp (decl)
  "If this st represents a pointer, return the type of the object."
  (if
   (eq (st-type decl) :pointer)
   t				   ; return T for type if unknown type
   (if (and (listp (st-type decl))
	    (eq (first (st-type decl)) :pointer))
       (second (st-type decl)))))

(defun st-actual-type (decl)
  (or (st-pointerp decl) (st-type decl)))

(defun st-pointer-generic-pointer (decl)
  (if (st-pointerp decl)
      (make-st (st-symbol decl) :pointer)
      decl))


