;;; KiPla - Cognitive kitchen planner and coordinator
;;; Copyright (C) 2009 by Lorenz Moesenlechner <moesenle@cs.tum.edu>
;;;
;;; This program is free software; you can redistribute it and/or modify
;;; it under the terms of the GNU General Public License as published by
;;; the Free Software Foundation; either version 3 of the License, or
;;; (at your option) any later version.
;;;
;;; This program is distributed in the hope that it will be useful,
;;; but WITHOUT ANY WARRANTY; without even the implied warranty of
;;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;;; GNU General Public License for more details.
;;;
;;; You should have received a copy of the GNU General Public License
;;; along with this program.  If not, see <http://www.gnu.org/licenses/>.


(in-package :cl-user)

(defpackage :jlo
  (:documentation "Jlo interface")
  (:use #:common-lisp #:roslisp #:alexandria)
  (:export
   ;; jlo.lisp
   #:jlo #:id #:parent-id #:name #:partial-lo
   #:delay-updates #:with-updates
   #:make-jlo #:pose #:cov #:frame-query
   ;; gc.lisp
   #:empty-cache
   ;; utils.lisp
   #:indentity-jlo #:make-jlo-rpy #:inlier?
   #:euclidean-distance #:component-distance))
