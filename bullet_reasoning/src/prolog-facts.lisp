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

(def-fact-group bullet-world-facts ()
  
  (<- (bullet-world ?world)
    (instance-of bt-reasoning-world ?world))

  (<- (assert-object ?world ?object-type ?name ?pose . ?args)
    (lisp-fun apply add-object
              ?world ?object-type
              ?name ?pose ?args
              ?_))

  (<- (retract-object ?world ?name)
    (lisp-fun remove-object ?world ?name))

  (<- (object ?world ?name ?obj)
    (bound ?name)
    (not (bound ?obj))
    (lisp-fun object ?world ?name ?obj))

  (<- (object ?world ?name ?obj)
    (not (bound ?name))
    (bound ?obj)
    (lisp-fun name ?obj ?name))

  (<- (object ?world ?name ?obj)
    (not (bound ?name))
    (not (bound ?obj))
    (lisp-fun find-objects ?world ?objs)
    (member ?obj ?objs)
    (lisp-fun name ?obj ?name))
  
  ;; Performs one simulation step in the world
  (<- (step ?world)
    (step ?world 0.01))

  ;; Performs one simulation step of length ?dt in the world
  (<- (step ?world ?dt)
    (lisp-fun step-simulation ?world ?dt))

  ;; Simulates the next ?t seconds
  (<- (simulate ?world ?t)
    (lisp-fun simulate ?world ?t ?_))

  ;; Simulates the next ?t seconds with a step of ?dt
  (<- (simulate ?world ?t ?dt)
    (lisp-fun simulate ?world ?t ?dt ?_))

  (<- (simulate-realtime ?world ?t)
    (lisp-fun simulate ?world ?t 0.01 :realtime ?_)))

(def-fact-group poses ()

  (<- (pose ?world ?obj ?pose)
    (lisp-fun body ?world ?obj ?body)
    (lisp-fun pose ?body ?pose))

  (<- (pose ?obj ?position ?orientation)
    (pose ?obj ?p)
    (position ?p ?position)
    (orientation ?p ?orientation))

  (<- (position ?pose (?x ?y ?z))
    (lisp-fun cl-transforms:origin ?pose ?p)
    (lisp-fun cl-transforms:x ?p ?x)
    (lisp-fun cl-transforms:y ?p ?y)
    (lisp-fun cl-transforms:z ?p ?z))

  (<- (orientation ?pose (?x ?y ?z ?w))
    (lisp-fun cl-transforms:orientation ?pose ?o)
    (lisp-fun cl-transforms:x ?o ?x)
    (lisp-fun cl-transforms:y ?o ?y)
    (lisp-fun cl-transforms:z ?o ?z)
    (lisp-fun cl-transforms:w ?o ?w))

  (<- (poses-equal ?pose-1 ?pose-2 (?dist-sigma ?ang-sigma))
    (lisp-pred poses-equal-p ?pose-1 ?pose-2 ?dist-sigma ?ang-sigma)))

(def-fact-group force-dynamic-states ()

  (<- (contact ?world ?obj-1 ?obj-2)
    (bound ?obj-1)
    (bound ?obj-2)
    (lisp-fun perform-collision-detection ?world ?_)
    (lisp-pred contact-p ?world ?obj-1 ?obj-2))

  (<- (contact ?world ?obj-1 ?obj-2)
    (bound ?obj-1)
    (not (bound ?obj-2))
    (lisp-fun perform-collision-detection ?world ?_)
    (lisp-fun find-objects-in-contact ?world ?obj-1 ?objs)
    (member ?obj-2 ?objs))

  (<- (contact ?world ?obj-1 ?obj-2)
    (not (bound ?obj-1))
    (not (bound ?obj-2))
    (lisp-fun perform-collision-detection ?world ?_)
    (lisp-fun find-all-contacts ?world ?contacts)
    (member (?obj-1 ?obj-2) ?contacts))

  (<- (stable ?world ?obj)
    (bound ?obj)
    (pose ?obj ?pose-1)
    (with-stored-world ?world
      (simulate ?world 2.5)
      (lisp-pred stable-p ?obj)
      (pose ?obj ?pose-2))
    (poses-equal ?pose-1 ?pose-2 (0.01 0.01)))

  (<- (supported-by ?world ?top ?bottom)
    (above ?top ?bottom)
    (contact ?world ?top ?bottom)
    (stable ?world ?top)))

(def-fact-group spatial-relations ()

  (<- (above ?obj-1 ?obj-2)
    (bound ?obj-1)
    (bound ?obj-2)
    (lisp-pred above-p ?obj-1 ?obj-2))

  (<- (above ?obj-1 ?obj-2)
    (not (bound ?obj-1))
    (bound ?obj-2)
    (lisp-fun find-objects-above ?obj-2 ?objs)
    (member ?obj-1 ?objs))

  (<- (above ?obj-1 ?obj-2)
    (bound ?obj-1)
    (not (bound ?obj-2))
    (lisp-fun find-objects-below ?obj-1 ?objs)
    (member ?obj-2 ?objs))

  (<- (below ?obj-1 ?obj-2)
    (bound ?obj-1 ?obj-2)
    (lisp-pred below-p ?obj-1 ?obj-2))

  (<- (below ?obj-1 ?obj-2)
    (not (bound ?obj-1))
    (bound ?obj-2)
    (lisp-fun find-objects-below ?obj-2 ?objs)
    (member ?obj-1 ?objs))

  (<- (below ?obj-1 ?obj-2)
    (bound ?obj-1)
    (not (bound ?obj-2))
    (lisp-fun find-objects-above ?obj-1 ?objs)
    (member ?obj-2 ?objs)))

(def-fact-group debug ()
  (<- (debug-window ?world)
    (lisp-fun add-debug-window ?world ?_)))
