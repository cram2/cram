;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :btr)

(def-fact-group reachability (object-grasp assert)
  (<- (valid-grasp ?world ?object ?grasp ?sides)
    (-> (not (object-grasp ?world ?object ?_ ?_))
        (and
         (grasp ?grasp)
         (side ?side)
         (== ?sides (?side)))
        (object-grasp ?world ?object ?grasp ?sides)))

  (<- (reachable ?w ?robot-name ?obj-name)
    (once (reachable ?w ?robot-name ?obj-name ?_)))

  (<- (reachable ?w ?robot-name ?obj-name ?side)
    (bullet-world ?w)
    (%object ?w ?robot-name ?robot)
    (lisp-type ?robot robot-object)
    (%object ?w ?obj-name ?obj)
    (with-stored-world ?w
      (once
       (robot-pre-grasp-joint-states ?pre-grasp-joint-states)
       (assert (joint-state ?w ?robot-name ?pre-grasp-joint-states))
       (valid-grasp ?w ?obj-name ?grasp ?sides)
       ;; Since forall doesn't fail if its cond fails, we verify its
       ;; validity first.
       (once (member ?side ?sides))
       (forall (member ?side ?sides)
               (lisp-pred object-reachable-p ?robot ?obj :side ?side :grasp ?grasp)))))

  (<- (point-reachable ?world ?robot-name ?point ?side)
    (bullet-world ?world)
    (side ?side)
    (%object ?world ?robot-name ?robot)
    (lisp-type ?robot robot-object)
    (with-stored-world ?world
      (once
       (robot-pre-grasp-joint-states ?pre-grasp-joint-states)
       (assert (joint-state ?w ?robot-name ?pre-grasp-joint-states))
       (grasp ?grasp)
       (lisp-pred point-reachable-p ?robot ?point :side ?side :grasp ?grasp))))

  (<- (pose-reachable ?world ?robot-name ?pose ?side)
    (bullet-world ?world)
    (side ?side)
    (%object ?world ?robot-name ?robot)
    (lisp-type ?robot robot-object)
    (with-stored-world ?world
      (once
       (robot-pre-grasp-joint-states ?pre-grasp-joint-states)
       (assert (joint-state ?w ?robot-name ?pre-grasp-joint-states))
       (lisp-pred pose-reachable-p ?robot ?pose :side ?side))))

  (<- (blocking ?w ?robot-name ?obj-name ?blocking-names)
    (blocking ?w ?robot-name ?obj-name ?_ ?blocking-names))

  (<- (blocking ?w ?robot-name ?obj-name ?side ?blocking-names)
    (bullet-world ?w)
    (%blocking ?w ?robot-name ?obj-name ?side ?blocking)
    (findall ?b (and (member ?b ?blocking))
             ?blocking-names))
  
  (<- (%blocking ?w ?robot-name ?obj-name ?side ?objs)
    (ground (?w ?robot-name))
    (%object ?w ?robot-name ?robot)
    (%object ?w ?obj-name ?obj)
    (with-stored-world ?w
      (setof
       ?o
       (and
        (valid-grasp ?w ?obj-name ?grasp ?sides)
        (member ?side ?sides)
        ;; We don't want to have the supporting object as a blocking
        ;; object.
        (-> (supported-by ?w ?obj-name ?supporting) (true) (true))
        ;; Generate all ik solutions
        (once
         (robot-pre-grasp-joint-states ?pre-grasp-joint-states)
         (assert (joint-state ?w ?robot-name ?pre-grasp-joint-states))
         (lisp-fun reach-object-ik ?robot ?obj :side ?side :grasp ?grasp ?ik-solutions)
         (member ?ik-solution ?ik-solutions))
        (%ik-solution-in-collision ?w ?robot ?ik-solution ?colliding-objects)
        (member ?o ?colliding-objects)
        (not (== ?o ?obj-name))
        (-> (bound ?supporting) (not (== ?o ?supporting)) (true)))
       ?objs)))

  (<- (assert (reach-ik-solution ?world ?robot ?pose ?side))
    (bullet-world ?world)
    (%object ?world ?robot ?robot-instance)
    (side ?side)
    (lisp-fun reach-pose-ik ?robot-instance ?pose :side ?side ?ik-solutions)
    (lisp-pred identity ?ik-solutions)
    (member ?ik-solution ?ik-solutions)
    (lisp-fun set-robot-state-from-joints ?ik-solution ?robot-instance ?_))
  
  (<- (%ik-solution-in-collision ?w ?robot ?ik-solution ?colliding-objects)
    (lisp-fun set-robot-state-from-joints ?ik-solution ?robot ?_)
    (%object ?w ?robot-name ?robot)
    (findall ?obj (contact ?w ?robot-name ?obj) ?colliding-objects)))
