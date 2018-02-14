;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <demmeln@in.tum.de>
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

(in-package :cram-occasions-events)

(def-fact-group time ()

  (<- (bound-time ?t)
    (bound ?t)
    (lisp-pred typep ?t timestamp))

  ;; NOTE:
  ;; (throughout a b) -> including a, excluding b
  ;; (during a b) -> including a, excluding b

  (<- (duration-includes (throughout ?t1 ?t2) (at ?a))
    (bound-time ?t1)
    (bound-time ?t2)
    (bound-time ?a)
    (and (<= ?t1 ?a) (< ?a ?t2)))

  (<- (duration-includes (throughout ?t1 ?t2) (at ?a))
    (bound-time ?t1)
    (bound-time ?t2)
    (not (bound ?a))
    (== ?a ?t1))

   (<- (duration-includes (throughout ?t1 ?t2) (throughout ?a ?b))
    (bound-time ?t1)
    (bound-time ?t2)
    (bound-time ?a)
    (bound-time ?b)
    (and (<= ?t1 ?a) (< ?a ?b) (<= ?b ?t2)))

   (<- (duration-includes (throughout ?t1 ?t2) (during ?a ?b))
    (bound-time ?t1)
    (bound-time ?t2)
    (bound-time ?a)
    (bound-time ?b)
    (and (< ?t1 ?b) (< ?a ?b) (< ?a ?t2))))

;;; An attempt to a more general time handling follows. Turned out not
;;; so general and not at all elegant, maybe something emerges in the
;;; future...
#+nil
(
   (<- (foo ?x)
     (== ?x :foo))
  
   (<- (time-includes (throughout ?a ?b) ?t)
     (bound ?a) (bound ?b)
     (lisp-pred time-value-p ?a)
     (lisp-pred time-value-p ?b)
     (time-includes-throughout-helper ?a ?b ?t))
  
   (<- (time-includes-throughout-helper ?a ?b (throughout ?x ?y))
     (bound ?x) (bound ?y)
     (lisp-pred time-value-p ?x)
     (lisp-pred time-value-p ?y)
     (lisp-pred <= ?a ?x)
     (lisp-pred <= ?y ?b))

   (<- (time-includes-throughout-helper ?a ?b (throughout ?x ?y))
     (bound ?x) (not (bound ?y))
     (lisp-pred time-value-p ?x)
     (lisp-pred <= ?a ?x)
     (== ?b ?y))

   (<- (time-includes-throughout-helper ?a ?b (throughout ?x ?y))
     (not (bound ?x)) (bound ?y)
     (lisp-pred time-value-p ?y)
     (lisp-pred <= ?y ?b)
     (== ?a ?x))

   (<- (time-includes-throughout-helper ?a ?b (throughout ?x ?y))
     (not (bound ?x)) (not (bound ?y))
     (== ?b ?y)
     (== ?a ?x))

   (<- (time-includes-throughout-helper ?a ?b (during ?x ?y))
     (bound ?x) (bound ?y)
     (lisp-pred time-value-p ?x)
     (lisp-pred time-value-p ?y)
     (lisp-pred <= ?x ?b)
     (lisp-pred <= ?a ?y))

   (<- (time-includes-throughout-helper ?a ?b (during ?x ?y))
     (bound ?x) (not (bound ?y))
     (lisp-pred time-value-p ?x)
     (lisp-pred <= ?x ?b)
     (lisp-fun get-max-time ?y))

   (<- (time-includes-throughout-helper ?a ?b (during ?x ?y))
     (not (bound ?x)) (bound ?y)
     (lisp-pred time-value-p ?y)
     (lisp-pred <= ?a ?y)
     (lisp-fun get-min-time ?x))

   (<- (time-includes-throughout-helper ?a ?b (during ?x ?y))
     (not (bound ?x)) (not (bound ?y))
     (lisp-fun get-min-time ?x)
     (lisp-fun get-max-time ?y))

   (<- (time-includes-throughout-helper ?a ?b (at ?t))
     (bound ?t)
     (lisp-pred time-value-p ?t)
     (lisp-pred <= ?a ?t)
     (lisp-pred <= ?t ?b))

   (<- (time-includes-throughout-helper ?a ?b (at ?t))
     (not (bound ?t))
     (== ?t ?a)))
