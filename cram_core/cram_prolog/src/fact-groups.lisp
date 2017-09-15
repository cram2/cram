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


(in-package :prolog)

;;;
;;; ABOUT FACTS, FACT GROUPS, PREDICATES AND FUNCTORS 
;;;
;;; We try to stick with prolog terminology, but some things are also
;;; different. Interesting to the reader could be the following prolog
;;; dictionary: http://www.cse.unsw.edu.au/~billw/prologdict.html
;;;
;;; Fact definitions have the following layout:
;;;   (fact-head . list-of-clauses)
;;;
;;; The fact head is sometimes called the pattern and the list-of clauses also
;;; called the body of the fact.
;;;
;;; The fact head must be a list with a symbol (not a variable) in the
;;; car. This is called the functor.
;;;   ((functor . rest-of-pattern) . list-of-clauses)
;;;
;;; As of now we don't distinguish between predicates with the same functor,
;;; but different arity.
;;;
;;; Facts are defined with the (<- head . body) macro. For interactive
;;; development in order to be able to "recompile" an bunch of facts easily,
;;; facts can only be defined in fact-groups. When you reevaluate a fact group
;;; (e.g. C-c C-c in emacs+slime) it removes all previously defined facts for
;;; this fact group before adding the current set of facts.
;;;
;;; The totality of all facts for a functor are a predicate. For each
;;; predicate the facts usually may be defined only in one fact-group, since
;;; defining facts for the same predicate in multiple fact groups is usually a
;;; mistake and can cause subtle bugs.
;;;
;;; In order to allow certain predicates to still be defined accross fact
;;; groups, DEF-FACT-GROUP takes a list of functors that may be defined in
;;; this fact group even if they are already defined in another fact
;;; group. One particular use case for this is the HOLDS predicate, which is
;;; quite general. It would be impractical to define all HOLDS facts in one
;;; fact group only.
;;;
;;; A note on synchronization: At the moment we assume that facts are only
;;; defined at "compile-time" and thus do not synchronize the data structures
;;; used here. Access to fact definition and thus prolog queries are pure
;;; functions and need not be synchronized even if called by multiple threads.
;;;
;;; ON THE DATA STRUCTURES USED
;;;
;;; *FACT-GROUP-FACTS* is a hash table that keeps a list of (functor . facts)
;;; pairs with all prediactes defined in each fact group. This is used to
;;; remove a fact group together which all its fact definition and then build
;;; the complete list of remaining facts for each functor. Removing a fact
;;; group is needed to allow "recompilation" in interactive development.
;;;
;;; *PREDICATE-FACTS* also is a hash table that is indexed by the functors and
;;; keeps a list of all fact defintions as well as information about the fact
;;; groups the facts came from. The entries in this hash table are of the form
;;; (fact-group-names . list-of-facts), where fact-group-names is a list of
;;; fact groups that defined facts for this predicate and list-of-facts is the
;;; list of all facts from all fact groups for this predicate. If a prediacte
;;; is defined by only one fact group (which is the usual case), then
;;; fact-group-names simply is a list with one element.
;;;
;;; This representation means that every fact is present twice, once in
;;; *FACT-GROUP-FACTS* and once in *PREDICATE-FACTS*. This is intentional, as
;;; the representation is optimized for speed of reading access (the list of
;;; all facts for a functor is always the CDR of the the entry in
;;; *PREDICATE-FACTS*). Addition and removal of fact-groups is not time
;;; critical, as it happens at "compile time".
;;;

(defvar *fact-group-facts* (make-hash-table :test 'eq)
  "Hash table of a list of (functor . list-of-facts) pairs for each fact group.")

(defvar *predicate-facts* (make-hash-table :test 'eq)
  "Hash table of a list of (fact-groups . list-of-facts) pairs for each functor.")

(declaim (inline fact-head fact-clauses functor))

(defun fact-head (fact)
  (car fact))

(defun fact-clauses (fact)
  (cdr fact))

(defun functor (fact-head)
  (car fact-head))

(defun is-valid-functor (functor)
  (and functor
       (symbolp functor)
       (not (is-var functor))))

(defun build-fact-list (functor groups)
  "Concatenates all the fact lists for a prediacte and a list of fact groups."
  (loop for group in groups
     appending (cdr (assoc functor (gethash group *fact-group-facts*)))))

(defun remove-facts (functor fact-group)
  "Removes facts defined for a predicate, but only if they come from
  `fact-group'."
  (let ((groups (car (gethash functor *predicate-facts*))))
    (if (= 1 (length groups))
        (remhash functor *predicate-facts*)
        (let ((groups (remove fact-group groups)))
          (setf (gethash functor *predicate-facts*)
                (cons groups (build-fact-list functor groups)))))))

(defun remove-fact-group (name)
  "Removes the fact group and all defined facts."
  (loop for (functor . _) in (gethash name *fact-group-facts*)
     do (remove-facts functor name))
  (remhash name *fact-group-facts*))

(defun add-fact-group (name extendable-predicates list-of-facts)
  "Adds a or replace a fact-group. Signals a style warning if existing fact
   group is replaced. Signals an error if predicates defined in other fact
   groups are extended, except if the functors is listed in
   `extendable-predicates'."
  (when (nth-value 1 (gethash name *fact-group-facts*))
    (style-warn "Redefining prolog fact-group ~a." name)
    (remove-fact-group name))
  (unless (every #'is-valid-functor extendable-predicates)
    (error "Extendible predicates in the definition of fact group ~a
            contained an invalid functor ~a."
           name (find-if-not #'is-valid-functor extendable-predicates)))
  (loop for (head . _) in list-of-facts
     for functor = (functor head)
     for existing-def = (gethash functor *predicate-facts*)
     when (not (is-valid-functor functor))
     do (error "Functor ~a of fact head ~a is not valid." functor head)
     when (and existing-def (not (member functor extendable-predicates)))
     do (error "Functor ~a is being redefined in fact-group ~a.
                Other definitions were in fact-groups ~a."
               functor name (car existing-def)))
  (%add-fact-group name list-of-facts))

(defun %add-fact-group (name list-of-facts)
  "Internal helper with no error checking."
  (let* ((list-of-facts (mapcar (lambda (fact)
                                  (rename-vars fact :gensym-var-names nil))
                                list-of-facts))
         (list-of-functors (remove-duplicates (mapcar (lambda (fact)
                                                        (functor (fact-head fact)))
                                                      list-of-facts)))
         (list-of-preds (mapcar (lambda (func)
                                  (cons func
                                        (remove-if-not (lambda (fact)
                                                         (eq func (functor (fact-head fact))))
                                                       list-of-facts)))
                                list-of-functors)))
    (loop for (functor . new-facts) in list-of-preds
       for current-facts = (gethash functor *predicate-facts*)
       do (setf (gethash functor *predicate-facts*)
                (cons (append (car current-facts) (list name))  ; works even if
                      (append (cdr current-facts) new-facts)))) ; current-facts is NIL
    (setf (gethash name *fact-group-facts*)
          list-of-preds)))

(defun nconc-fact (list-of-facts fact-head fact-body)
  "Destructively adds a fact to a list-of-facts. The fact head and body are
   not modified."
  (nconc list-of-facts (list (cons fact-head fact-body))))

(defmacro <- (fact-head &body fact-code)
  (declare (ignore fact-head fact-code))
  `(error "Facts cannot be declared outside fact-groups."))

(defmacro def-fact-group (fact-group-name extendable-predicates &body facts)
  "Define a group of facts. Predicates already defined by other fact groups
   may only be extended if the corresponding functor ist listed in
   `extendable-prediactes'."
  (let ((list-of-facts (gensym "LIST-OF-FACTS-")))
    `(macrolet ((<- (fact-head &body fact-code)
                  `(setf ,',list-of-facts
                         (nconc-fact ,',list-of-facts ',fact-head ',fact-code))))
       (let ((,list-of-facts nil))
         ,@facts
         (add-fact-group ',fact-group-name ',extendable-predicates ,list-of-facts)))))

(defun get-predicate-facts (functor)
  "Returns the list of facts defined for a predicate. May be empty is the
   predicate is not defined."
  (cdr (gethash functor *predicate-facts*))) ; works even if functor is undefined
