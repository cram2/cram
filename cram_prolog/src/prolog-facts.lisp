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

(in-package :prolog)

(defmacro bin-pred-fact (pred)
  `(<- (,pred ?x ?y)
     (lisp-pred ,pred ?x ?y)))

(def-fact-group misc-utils ()
  (<- (== ?x ?x))
  ;; mind the symbols shadowed in cram/language
  (bin-pred-fact <)
  (bin-pred-fact >)
  (bin-pred-fact <=)
  (bin-pred-fact >=)
  
  (<- (equal ?x ?y)
    (ground (?x ?y))
    (lisp-pred equal ?x ?y))

  (<- (equal ?x ?y)
    (not (ground (?x ?y)))
    (== ?x ?y)))

(def-fact-group list-utils ()
  (<- (member ?x (?x . ?y)))
  (<- (member ?x (?y . ?z))
    (member ?x ?z))

  ;; for now only handle bound ?list
  (<- (length ?list ?length)
    (bound ?list)
    (lisp-fun length ?list ?length))

  (<- (append ?list-1 ?list-2 ?result)
    (bound ?list-1)
    (bound ?list-2)
    (lisp-fun append ?list-1 ?list-2 ?result)
    (cut))

  (<- (append nil ?list ?list))
  
  (<- (append (?elem . ?rest) ?list-2 (?elem . ?result))
    (append ?rest ?list-2 ?result))
  
  ;; for now only handle bound parameters
  (<- (sort ?list ?pred ?result)
    (sort ?list ?pred identity ?result))
  
  (<- (sort ?list ?pred ?key ?result)
    (bound ?list)
    (bound ?pred)
    (bound ?key)
    (lisp-fun copy-list ?list ?copy)
    (lisp-fun force-ll ?copy ?expanded-list)
    (lisp-fun sort ?expanded-list ?pred :key ?key ?result))

  (<- (reduce ?function ?list ?result)
    (bound ?list)
    (lisp-fun copy-list ?list ?copy)
    (lisp-fun force-ll ?copy ?expanded-list)
    (lisp-fun reduce ?function ?expanded-list ?result)))

(def-fact-group math-utils ()
  (<- (max ?list ?result)
    (reduce max ?list ?result)))

(def-fact-group string-utils ()
  (<- (string-concat ?s1 ?s2 ?s3)
    (bound ?s1) (bound ?s2)
    (lisp-fun concatenate string ?s1 ?s2 ?s3))
  (<- (string-concat ?s1 ?s2 ?s3)
    (or (not (bound ?s1)) (not (bound ?s2)))
    (error "Not implemented.")))

(def-fact-group debugging ()
  (<- (format . ?args)
    (lisp-fun symbol-function format ?format-fun)
    (lisp-fun apply ?format-fun t ?args ?_))
  (<- (warn)
    (warn ""))
  (<- (error)
    (error ""))
  (<- (warn ?datum . ?args)
    (lisp-fun symbol-function warn ?fun)
    (string-concat "[PROLOG WARNING] " ?datum ?string)
    (lisp-fun apply ?fun ?string ?args ?_))
  (<- (error ?datum . ?args)
    (lisp-fun symbol-function error ?fun)
    (string-concat "[PROLOG ERROR] " ?datum ?string)
    (lisp-fun apply ?fun ?string ?args ?_)))

(defun prolog-get-slot-value (obj slot)
  (slot-value obj slot))

(defun prolog-set-slot-value (obj slot new-value)
  (setf (slot-value obj slot) new-value))

(def-fact-group lisp-data-structs ()

  ;; Note: The difference between SLOT-VALUE and GET-SLOT-VALUE is, that the
  ;; former will SETF the slot, if ?VALUE is bound, while the latter will just
  ;; check if the slots value and ?VALUE unify.
  
  (<- (slot-value ?obj ?slot ?value)
    (bound ?obj)
    (bound ?slot)
    (not (bound ?value))
    (lisp-fun prolog-get-slot-value ?obj ?slot ?value))

  (<- (slot-value ?obj ?slot ?value)
    (bound ?obj)
    (bound ?slot)
    (bound ?value)
    (lisp-fun prolog-set-slot-value ?obj ?slot ?value ?_))
  
  (<- (get-slot-value ?obj ?slot ?value)
    (bound ?obj)
    (bound ?slot)
    (lisp-fun prolog-get-slot-value ?obj ?slot ?value))

  (<- (instance-of ?type ?obj)
    (bound ?type)
    (not (bound ?obj))
    (lisp-fun make-instance ?type ?obj))
  
  (<- (instance-of ?type ?obj)
    (lisp-type ?obj ?type))

  (<- (symbol-value ?sym ?val)
    (lisp-fun symbol-value ?sym ?val))

  (<- (set-symbol-value ?sym ?val)
    (lisp-fun set ?sym ?val ?_)))

(def-fact-group lisp-types ()
  (<- (lisp-type ?var ?type)
    (bound ?var)
    (not (bound ?type))
    (lisp-fun type-of ?var ?type))

  (<- (lisp-type ?var ?type)
    (ground (?var ?type))
    (lisp-pred prolog-type-p ?var ?type)))
