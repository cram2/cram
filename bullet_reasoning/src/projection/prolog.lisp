;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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

(in-package :btr)

(defvar *current-timeline* nil)

(defmacro with-timeline (timeline &body body)
  `(let ((*current-timeline* ,timeline))
     ,@body))

(def-prolog-handler holds-in-stored-world (bdgs ?world ?occasion)
  (let ((?world (var-value ?world bdgs))
        (?occasion (var-value ?occasion bdgs)))
    (assert (not (is-var ?world)) () "?world needs to be bound")
    (assert (not (is-var ?occasion)) () "?occasion needs to be bound")
    (when ?world
      (let ((*current-bullet-world* (restore-world-state
                                     ?world
                                     (make-instance 'bt-reasoning-world))))
        (prolog ?occasion bdgs)))))

(def-fact-group timeline-predicates (holds occurs)

  (<- (occurs ?timeline ?ev ?t)
    (get-slot-value ?timeline events ?events)
    (member ?event-instance ?events)
    (get-slot-value ?event-instance timestamp ?t)
    (get-slot-value ?event-instance event ?ev))
  
  (<- (holds ?timeline ?occ ?t)
    (not (bound ?timeline))
    (symbol-value *current-timeline* ?timeline)
    (holds ?timeline ?occ ?t))

  (<- (holds ?timeline ?occ (at ?t))
    (bound ?occ)
    (lisp-type ?timeline timeline)
    (timeline-world-at ?timeline ?t ?world)
    (holds-in-stored-world ?world ?occ))

  (<- (holds ?timeline ?occ (during ?t-1 ?t-2))
    (bound ?timeline)
    (bound ?occ)
    (timeline-world-at ?timeline ?t ?world)
    (lisp-pred >= ?t ?t-1)
    (lisp-pred < ?t ?t-2)
    (holds-in-stored-world ?world ?occ))

  (<- (holds ?timeline ?occ (throughout ?t-1 ?t-2))
    (bound ?timeline)
    (bound ?occ)    
    (every (and (timeline-world-at ?timeline ?t ?world)
                (lisp-pred >= ?t ?t-1)
                (lisp-pred < ?t ?t-2))
           (holds-in-stored-world ?world ?occ)))

  (<- (timeline-world-at ?timeline ?t ?world)
    (ground (?timeline ?t))
    (lisp-fun timeline-lookup ?timeline ?t ?world))

  (<- (timeline-world-at ?timeline ?t ?world)
    (bound ?timeline)
    (not (bound ?t))
    (occurs ?timeline ?_ ?t)
    (timeline-world-at ?timeline ?t ?world)))
