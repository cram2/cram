;;;
;;; Copyright (c) 2009, Lars Kunze <kunzel@cs.tum.edu>
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

(in-package :liswip)

;; PL_unify_term() arguments
(defconstant PL-VARIABLE 1) ;; Variable (?-prefixed symbol)
(defconstant PL-ATOM 2)     ;; symbol
(defconstant PL-INTEGER 3)  ;; int
(defconstant PL-FLOAT 4)    ;; double 
(defconstant PL-STRING 5)   ;; const char 
(defconstant PL-TERM 6)
(defconstant PL-LIST 7)     ;; list

(defconstant FALSE 0)
(defconstant TRUE 1)

(defconstant PL-Q-NORMAL 2)
(defconstant PL-Q-NODEBUG 4)
(defconstant PL-Q-CATCH_EXCEPTION 8)

(define-foreign-library libpl
  (:darwin (:or "libpl.dylib" "/opt/local/lib/libpl.dylib"))
  (:unix (:or "libpl.so.5.6.58" "libpl.so"))
  (t (:default "libpl")))

(use-foreign-library libpl)

(defctype atom_t :unsigned-long)
(defctype term_t :unsigned-long)
(defctype qid_t :unsigned-long)
(defctype functor_t :unsigned-long)
(defctype predicate_t :pointer)
(defctype module_t :pointer)
(defctype fid_t :unsigned-long)
(defctype foreign_t :unsigned-long)
(defctype control_t :pointer)

(defcfun "PL_initialise" :int
  "Initialises the SWI-Prolog"
  (argc  :int)
  (argv  :pointer))

(defcfun "PL_is_initialised" :int
  (argc :pointer)
  (argv :pointer))

(defcfun "PL_cleanup" :int
  "This function performs the reverse of PL_initialise()"
   (status :int))
  
(defcfun "PL_halt" :int
  "Cleanup the Prolog environment using PL_cleanup() and calls exit() with the status argument"
  (status :int))

(defcfun "PL_new_module" module_t
  (name atom_t))

;; foreign context frames
(defcfun "PL_open_foreign_frame" fid_t)

(defcfun "PL_rewind_foreign_frame" :void
  (cid fid_t))

(defcfun "PL_close_foreign_frame" :void
  (cid fid_t))

(defcfun "PL_discard_foreign_frame" :void
  (cid fid_t))

;; finding predicates
(defcfun "PL_pred" predicate_t
  "Return a handle to a predicate for the specified name/arity in the given module"
  (functor functor_t)
  (module module_t))

(defcfun "PL_predicate" predicate_t
  "Return a handle to a predicate for the specified name/arity in the given module"
  (name :string)
  (arity :int)
  (module :string))

;; call-back
(defcfun "PL_open_query" qid_t
  (module module_t)
  (flags :int)
  (predicate predicate_t)
  (term term_t))

(defcfun "PL_next_solution" :int
  (qid qid_t))

(defcfun "PL_close_query" :void
  (qid qid_t))

(defcfun "PL_cut_query" :void
  (qid qid_t))

(defcfun "PL_exception" term_t
  (qid qid_t))

;; simplified
(defcfun "PL_call" :int
  (term term_t)
  (module module_t))

(defcfun "PL_call_predicate" :int
  (module module_t)
  (debug :int)
  (predicate predicate_t)
  (term term_t))

;;;; term-references
;; creating and destroying term-refs
(defcfun "PL_new_term_refs" term_t
  "Return n new term references. The first term-reference is returned. The others are t+1, t+2, etc."
  (n :int))

(defcfun "PL_new_term_ref" term_t
  "Return a fresh reference to a term")

;; constants
(defcfun "PL_new_atom" atom_t
  (chars :string))

(defcfun "PL_atom_chars" :string
  (atom atom_t))

(defcfun "PL_new_functor" functor_t
  (name atom_t)
  (arity :int))

(defcfun "PL_functor_arity" :int
  (f functor_t))

(defcfun "PL_functor_name" atom_t
  (f functor_t))

;; assign to term-references
(defcfun "PL_put_atom_chars" :void
  "Put an atom in the term-reference constructed from the 0-terminated string"
  (term  term_t)
  (chars :string))

(defcfun "PL_put_string_chars" :void
  (term term_t)
  (chars :string))

(defcfun "PL_put_float" :void
  (term  term_t)
  (val :double))

(defcfun "PL_put_integer" :void
  (term  term_t)
  (val :long))

(defcfun "PL_put_term" :void
  (t1 term_t)
  (t2 term_t))

(defcfun "PL_cons_functor" :void
  (t1 term_t)
  (f functor_t)
  (a1 term_t)
  (a2 term_t)
  (a3 term_t))

(defcfun "PL_cons_functor_v" :void
 (ht1 term_t)
 (f functor_t)
 (t2 term_t))

(defcfun "PL_cons_list" :void
  (list term_t)
  (first term_t)
  (rest term_t))

(defcfun "PL_put_nil" :void
  (term term_t))

;; verify types
(defcfun "PL_term_type" :int
  (term term_t))

(defcfun "PL_is_list" :int
  (term term_t))

(defcfun "PL_get_list" :int
  (lst term_t)
  (head term_t)
  (rest term_t))

;; get c-values from prolog terms
(defcfun "PL_get_integer" :int
  (term term_t)
  (val :pointer))

(defcfun "PL_get_float" :int
  (term term_t)
  (val :pointer))

(defcfun "PL_get_atom" :int
  (term term_t)
  (atom :pointer))

(defcfun "PL_get_atom_chars" :int
  (term term_t)
  (chars :pointer))

(defcfun "PL_get_arg" :int
  "If term1 is compound and index is between 1 and arity (including), assign term2 with a term-reference to the argument"
  (index :int)
  (term1 term_t)
  (term2 term_t))

(defcfun "PL_get_name_arity" :int
  (term term_t)
  (atom :pointer)
  (arity :pointer))

(defcfun "PL_get_chars" :int
  (term term_t)
  (s :pointer)
  (flags :unsigned-int))

(defcfun "PL_get_functor" :int
  (term term_t)
  (f :pointer))

(defcfun "PL_copy_term_ref" term_t
  (term term_t))

;; Foreign predicates
(defconstant PL_FA_NOTRACE 1)
(defconstant PL_FA_TRANSPARENT 2)
(defconstant PL_FA_NONDETERMINISTIC 4)
(defconstant PL_FA_VARARGS 8)

(defconstant PL-FIRST-CALL 0)
(defconstant PL-CUTTED 1)
(defconstant PL-REDO 2)

(defcfun "PL_register_foreign_in_module" :int
  (module :string)
  (name :string)
  (arity :int)
  (callback :pointer)
  (flags :int))

(defcfun "PL_register_foreign" :int
  (name :string)
  (arity :int)
  (callback :pointer)
  (flags :int))

(defcfun ("_PL_retry" pl-retry) foreign_t
  (context :long))

(defcfun ("_PL_retry_address" pl-retry-address) foreign_t
  (ctxaddr :pointer))

(defcfun "PL_foreign_control" :int
  (control control_t))

(defcfun "PL_foreign_context" :long
  (control control_t))

(defcfun "PL_foreign_context_address" :pointer
  (control control_t))

(defcfun "PL_unify_atom_chars" :int
  (trm term_t)
  (chars :string))

(defcfun "PL_unify_list_chars" :int
  (trm term_t)
  (chars :string))

(defcfun "PL_unify_string_chars" :int
  (trm term_t)
  (chars :string))

(defcfun "PL_unify_integer" :int
  (trm term_t)
  (n :long))

(defcfun "PL_unify_float" :int
  (trm term_t)
  (f :double))

(defcfun "PL_unify_list" :int
  (l term_t)
  (h term_t)
  (r term_t))

(defconstant PL-BLOB-MAGIC #x75293a01)
(defconstant PL-BLOB-UNIQUE 1)
(defconstant PL-BLOB-TEXT 2)
(defconstant PL-BLOB-NOCOPY 4)
(defconstant PL-BLOB-WCHAR 8)

(defcstruct pl_blob_t
  (magic :ulong)
  (flags :ulong)
  (name :string)
  (release :pointer)
  (compare :pointer)
  (write :pointer)
  (acquire :pointer))

(defcfun "PL_register_blob_type" :int
  (type :pointer))

(defcfun "PL_unregister_blob_type" :int
  (type :pointer))

(defcfun "PL_is_blob" :int
  (term term_t)
  (type :pointer))

(defcfun "PL_unify_blob" :int
  (term term_t)
  (blob :pointer)
  (len :uint)
  (type :pointer))

(defcfun "PL_put_blob" :int
  (term term_t)
  (blob :pointer)
  (len :uint)
  (type :pointer))

(defcfun "PL_get_blob" :int
  (term term_t)
  (blob :pointer)
  (len :pointer)
  (type :pointer))

(defcfun "PL_blob_data" :pointer
  (a atom_t)
  (len :uint)
  (type :pointer))

(defcfun "PL_register_atom" :void
  (atom atom_t))

(defcfun "PL_unregister_atom" :void
  (atom atom_t))

(defcfun "PL_thread_self" :int)

(defcfun "PL_thread_attach_engine" :int
  (attr :pointer))

(defcfun "PL_thread_destroy_engine" :int)
