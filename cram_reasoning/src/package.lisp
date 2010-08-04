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


(in-package :cl-user)

(defpackage :cram-reasoning
  (:use #:common-lisp #:cram-utilities)
  (:nicknames :crs)
  (:import-from #:alexandria
                #:curry #:rcurry #:compose #:with-gensyms)
  (:export #:lisp-fun
           #:lisp-pred
           #:bound
           #:ground
           #:member
           #:==
           #:format
           #:?_
           #:fail
           #:and
           #:or
           #:not
           #:<
           #:>
           #:<=
           #:>=
           #:unify
           #:unify-p
           #:prolog
           #:def-fact-group
           #:<-
           #:def-prolog-handler
           #:slot-value
           #:get-slot-value
           #:instance-of
           #:once
           #:findall
           #:forall
           #:filter-bindings
           #:query-var
           ;; Rete
           #:clear-alpha-network #:rete-assert #:rete-retract
           #:with-facts-asserted #:object-id
           #:rete-holds #:alpha-network-size
           #:def-production #:register-production
           #:clear-productions #:remove-production
           #:with-productions #:remove-production-handler
           #:register-production-handler
           #:with-production-handlers
           #:rete-proof))

