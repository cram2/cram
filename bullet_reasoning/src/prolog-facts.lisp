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

(defvar *current-bullet-world* nil)

(defmacro with-current-bullet-world (world &body body)
  `(let ((*current-bullet-world* ,world))
     ,@body))

(def-fact-group bullet-world-facts (assert retract)

  (<- (clear-bullet-world)
    (lisp-fun set *current-bullet-world* nil ?_))
  
  (<- (clear-bullet-world ?world)
    (not (bound ?world))
    (instance-of bt-reasoning-world ?world))
  
  (<- (bullet-world ?world)
    (not (bound ?world))
    (symbol-value *current-bullet-world* ?current-world)
    (-> (lisp-pred identity ?current-world)
        (== ?world ?current-world)
        (and
         (instance-of bt-reasoning-world ?world)
         (set-symbol-value *current-bullet-world* ?world))))

  (<- (bullet-world ?world)
    (lisp-type ?world bt-reasoning-world))

  (<- (bullet-world ?world ?obj)
    ;; The world ?obj belongs to
    (get-slot-value ?obj world ?world))

  (<- (assert (object ?world ?object-type ?name ?pose . ?_))
    (object ?world ?name)
    (pose ?world ?p . ?pose)
    (assert (object-pose ?world ?name ?p)))
  
  (<- (assert (object ?world ?object-type ?name ?pose . ?args))
    (not (object ?world ?name))
    (lisp-fun apply add-object
              ?world ?object-type
              ?name ?pose ?args
              ?_))

  (<- (retract (object ?name))
    (retract (object ?_ ?name)))
  
  (<- (retract (object ?world ?name))
    (lisp-fun remove-object ?world ?name ?_))

  (<- (object ?name)
    (object ?_ ?name))
  
  (<- (object ?world ?name)
    (%object ?world ?name ?_))

  (<- (object-type ?name ?type)
    (object-type ?_ ?name ?type))
  
  (<- (object-type ?world ?name ?type)
    (bullet-world ?world)
    (%object ?world ?name ?obj)
    (lisp-type ?obj ?type))

  (<- (object-type ?world ?name ?type)
    (bullet-world ?world)
    (%object ?world ?name ?obj)
    (lisp-type ?obj household-object)
    (get-slot-value ?obj type ?type))

  (<- (household-object-type ?name ?type)
    (household-object-type ?_ ?name ?type))

  (<- (household-object-type ?world ?name ?type)
    (bullet-world ?world)
    (object ?world ?name)    
    (object-type ?world ?name household-object)
    (%object ?world ?name ?object-instance)
    (get-slot-value ?object-instance type ?type))
  
  (<- (%object ?world ?name ?obj)
    (bound ?name)
    (lisp-type ?name symbol)
    (not (bound ?obj))
    (bullet-world ?world)
    (lisp-fun object ?world ?name ?obj)
    (lisp-type ?obj object))

  (<- (%object ?world ?name ?obj)
    (bound ?obj)
    (bullet-world ?world ?obj)
    (lisp-type ?obj object)
    (lisp-fun name ?obj ?name))

  (<- (%object ?world ?name ?obj)
    (not (bound ?name))
    (not (bound ?obj))
    (bullet-world ?world)
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

(def-fact-group poses (assert)

  (<- (pose ?obj-name ?pose)
    (pose ?_ ?obj-name ?pose))
  
  (<- (pose ?w ?obj-name ?pose)
    (lisp-type ?obj-name symbol)
    (bullet-world ?w)
    (%object ?w ?obj-name ?obj)
    (%pose ?obj ?pose))
  
  (<- (%pose ?obj ?pose)
    (bound ?obj)
    (not (bound ?pose))
    (instance-of object ?obj)
    (lisp-fun pose ?obj ?pose))

  (<- (%pose ?obj ?pose)
    (bound ?obj)
    (bound ?pose)
    (%pose ?obj ?obj-pose)
    (poses-equal ?pose ?obj-pose 0.01 0.01))

  (<- (assert (object-pose ?obj-name ?pose))
    (assert (object-pose ?_ ?obj-name ?pose)))
  
  (<- (assert (object-pose ?world ?obj-name ?pose))
    (bound ?obj-name)
    (bound ?pose)
    (bullet-world ?world)
    (%object ?world ?obj-name ?obj)
    (lisp-fun set-object-pose ?obj ?pose ?_))

  (<- (pose ?obj-name ?position ?orientation)
    (pose ?_ ?obj-name ?position ?orientation))
  
  (<- (pose ?w ?obj-name ?position ?orientation)
    (bullet-world ?w)
    (lisp-type ?obj-name symbol)
    (%object ?w ?obj-name ?obj)
    (%pose ?obj ?position ?orientation))
  
  (<- (%pose ?obj ?position ?orientation)
    (bound ?obj)
    (instance-of object ?obj)
    (pose ?obj ?p)
    (position ?p ?position)
    (orientation ?p ?orientation))

  (<- (pose ?pose ?position ?orientation)
    (bound ?pose)
    (position ?pose ?position)
    (orientation ?pose ?orientation))

  (<- (pose ?pose (?x ?y ?z) (?ax ?ay ?az ?aw))
    (not (bound ?pose))
    (ground (?x ?y ?z))
    (ground (?ax ?ay ?az ?aw))
    (lisp-fun cl-transforms:make-3d-vector ?x ?y ?z ?o)
    (lisp-fun cl-transforms:make-quaternion ?ax ?ay ?az ?aw ?q)
    (lisp-fun cl-transforms:make-pose ?o ?q ?pose))
  
  (<- (pose ?pose ?position (?ax ?ay ?az ?aw))
    (not (bound ?pose))
    (lisp-type ?position cl-transforms:3d-vector)
    (ground (?ax ?ay ?az ?aw))
    (lisp-fun cl-transforms:make-quaternion ?ax ?ay ?az ?aw ?q)
    (lisp-fun cl-transforms:make-pose ?position ?q ?pose))

  (<- (pose ?pose (?x ?y ?z) ?orientation)
    (not (bound ?pose))
    (ground (?x ?y ?z))
    (lisp-type ?orientation cl-transforms:quaternion)
    (lisp-fun cl-transforms:make-3d-vector ?x ?y ?z ?o)
    (lisp-fun cl-transforms:make-pose ?position ?orientation ?pose))

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

(def-fact-group robot-model (assert retract)

  (<- (link ?robot-name ?link)
    (link ?_ ?robot-name ?link))

  (<- (link ?world ?robot-name ?link)
    (bullet-world ?world)
    (%object ?world ?robot-name ?robot)
    (lisp-fun link-names ?robot ?links)
    (member ?link ?links))
  
  (<- (link-pose ?robot-name ?name ?pose)
    (link-pose ?_ ?robot-name ?name ?pose))
  
  (<- (link-pose ?w ?robot-name ?name ?pose)
    (bullet-world ?w)
    (%object ?w ?robot-name ?robot)
    (%link-pose ?robot ?name ?pose))
  
  (<- (%link-pose ?robot ?name ?pose)
    (bound ?robot)
    (bound ?name)
    (lisp-type ?robot robot-object)
    (-> (bound ?pose)
        (and
         (lisp-fun link-pose ?robot ?name ?l-p)
         (poses-equal ?pose ?l-p 0.01 0.01))
        (and
         (lisp-fun link-pose ?robot ?name ?pose)
         (lisp-pred identity ?pose))))

  (<- (%link-pose ?robot ?name ?pose)
    (bound ?robot)
    (not (bound ?name))
    (lisp-type ?robot robot-object)
    (lisp-fun link-names ?robot ?names)
    (member ?name ?names)
    (%link-pose ?robot ?name ?pose))

  (<- (head-pointing-at ?robot-name ?pose)
    (head-pointing-at ?_ ?robot-name ?pose))
  
  (<- (head-pointing-at ?w ?robot-name ?pose)
    (robot ?robot-name)
    (robot-pan-tilt-links ?pan-link ?tilt-link)
    (robot-pan-tilt-joints ?pan-joint ?tilt-joint)
    (bullet-world ?w)
    (%object ?w ?robot-name ?robot)
    (lisp-fun calculate-pan-tilt
              ?robot ?pan-link ?tilt-link ?pose
              (?pan-pos ?tilt-pos))
    (lisp-fun set-joint-state ?robot ?pan-joint ?pan-pos ?_)
    (lisp-fun set-joint-state ?robot ?tilt-joint ?tilt-pos ?_))

  (<- (joint-state ?robot-name ?state)
    (joint-state ?_ ?robot-name ?state))

  (<- (joint-state ?world ?robot-name ?state)
    (bullet-world ?world)
    (%object ?world ?robot-name ?robot)
    (lisp-fun joint-state ?robot ?state))

  (<- (assert (joint-state ?robot-name ?joint-states))
    (assert (joint-state ?_ ?robot-name ?joint-states)))

  (<- (assert (joint-state ?world ?robot-name ?joint-states))
    (bullet-world ?world)
    (%object ?world ?robot-name ?robot)
    (lisp-fun set-robot-state-from-joints ?joint-states ?robot ?_))

  (<- (attached ?world ?robot ?link-name ?object)
    (bullet-world ?world)
    (%object ?world ?robot ?robot-instance)
    (lisp-fun attached-objects ?robot-instance ?attached-objects)
    (member (?object . ?links) ?attached-objects)
    (member ?link ?links))

  (<- (assert (attached ?world ?robot ?link-name ?object))
    (bullet-world ?world)
    (%object ?world ?robot ?robot-instance)
    (lisp-fun attach-object ?robot-instance ?object ?link-name ?_))

  (<- (retract (attached ?world ?robot ?object))
    (bullet-world ?world)
    (%object ?world ?robot ?robot-instance)
    (lisp-fun detach-object ?robot-instance ?object ?_))

  (<- (retract (attached ?world ?robot ?link-name ?object))
    (bullet-world ?world)
    (%object ?world ?robot ?robot-instance)
    (lisp-fun detach-object ?robot-instance ?object ?link-name ?_)))

(def-fact-group force-dynamic-states ()

  (<- (contact ?obj-1-name ?obj-2-name)
    (contact ?_ ?obj-1-name ?obj-2-name))
  
  (<- (contact ?world ?obj-1-name ?obj-2-name)
    (bound ?obj-1-name)
    (bound ?obj-2-name)
    (bullet-world ?world)
    (%object ?world ?obj-1-name ?obj-1)
    (%object ?world ?obj-2-name ?obj-2)    
    (lisp-fun perform-collision-detection ?world ?_)
    (lisp-pred contact-p ?world ?obj-1 ?obj-2))

  (<- (contact ?world ?obj-1-name ?obj-2-name)
    (bound ?obj-1-name)
    (not (bound ?obj-2-name))
    (bullet-world ?world)
    (%object ?world ?obj-1-name ?obj-1)
    (lisp-fun find-objects-in-contact ?world ?obj-1 ?objs)
    (member ?obj-2 ?objs)
    (%object ?world ?obj-2-name ?obj-2))

  (<- (contact ?world ?obj-1-name ?obj-2-name)
    (not (bound ?obj-1-name))
    (bound ?obj-2-name)
    (contact ?world ?obj-2-name ?obj-1-name))

  (<- (contact ?world ?obj-1-name ?obj-2-name)
    (not (bound ?obj-1-name))
    (not (bound ?obj-2-name))
    (bullet-world ?world)
    (lisp-fun find-all-contacts ?world ?contacts)
    (member (?obj-1 ?obj-2) ?contacts)
    (%object ?world ?obj-1-name ?obj-1)
    (%object ?world ?obj-2-name ?obj-2))

  (<- (contact ?world ?obj-1-name ?obj-2-name ?link)
    ;; Like CONTACT but also yields the name of a link that is in
    ;; contact with an object. This works only for multi-link objects
    ;; such as semantic maps or robots
    (contact ?world ?obj-1-name ?obj-2-name)
    (%object ?world ?obj-1-name ?obj-1)
    (%object ?world ?obj-2-name ?obj-2)
    (or (%link-contact ?obj-1 ?obj-2 ?link)
        (%link-contact ?obj-2 ?obj-1 ?link)))

  (<- (%link-contact ?robot-model ?obj ?link)
    (lisp-type ?robot-model robot-object)
    (lisp-fun link-contacts ?robot-model ?link-contacts)
    (member (?obj . ?link) ?link-contacts))

  (<- (stable ?obj-name)
    (lisp-type ?obj-name symbol)
    (stable ?_ ?obj-name))
  
  (<- (stable ?world ?obj-name)
    (bullet-world ?world)
    (%object ?world ?obj-name ?obj)
    (%pose ?obj ?pose-1)
    (with-stored-world ?world
      (simulate ?world 0.5)
      ;; checking for active-tag does not always work and requires
      ;; pretty long simlation times. Additionally some bodies are
      ;; just very unstable and never get deactivated, e.g. cylinders.
      ;;
      ;; (lisp-pred stable-p ?obj)
      (%pose ?obj ?pose-2))
    (poses-equal ?pose-1 ?pose-2 (0.01 0.03)))

  (<- (stable)
    (stable ?_))
  
  (<- (stable ?world)
    (bullet-world ?world)
    (forall (object ?world ?_ ?o)
            (stable ?world ?o)))

  (<- (supported-by ?top ?bottom)
    (supported-by ?_ ?top ?bottom))
  
  (<- (supported-by ?world ?top ?bottom)
    (bullet-world ?world)
    (above ?world ?top ?bottom)
    (contact ?world ?top ?bottom)
    (stable ?world ?top)))

(def-fact-group spatial-relations ()

  (<- (above ?obj-1-name ?obj-2-name)
    (above ?_ ?obj-1-name ?obj-2-name))
  
  (<- (above ?w ?obj-1-name ?obj-2-name)
    (bullet-world ?w)
    (-> (lisp-type ?obj-1-name symbol)
        (%object ?w ?obj-1-name ?obj-1)
        (true))
    (-> (lisp-type ?obj-2-name symbol)
        (%object ?w ?obj-2-name ?obj-2)
        (true))
    (%above ?w ?obj-1 ?obj-2)
    (%object ?w ?obj-1-name ?obj-1)
    (%object ?w ?obj-2-name ?obj-2))

  (<- (%above ?_ ?obj-1 ?obj-2)
    (bound ?obj-1)
    (bound ?obj-2)
    (lisp-type ?obj-1 object)
    (lisp-type ?obj-2 object)
    (lisp-pred above-p ?obj-1 ?obj-2))

  (<- (%above ?w ?obj-1 ?obj-2)
    (not (bound ?obj-1))
    (bound ?obj-2)
    (lisp-type ?obj-2 object)    
    (lisp-fun find-objects-above ?w ?obj-2 ?objs)
    (member ?obj-1 ?objs))

  (<- (%above ?w ?obj-1 ?obj-2)
    (bound ?obj-1)
    (not (bound ?obj-2))
    (lisp-type ?obj-1 object)
    (lisp-fun find-objects-below ?w ?obj-1 ?objs)
    (member ?obj-2 ?objs))

  (<- (%above ?w ?obj-1 ?obj-2)
    (not (bound ?obj-1))
    (not (bound ?obj-2))
    (%object ?w ?_ ?obj-1)
    (lisp-fun find-objects-below ?w ?obj-1 ?objs)
    (member ?obj-2 ?objs))  

  (<- (below ?obj-1-name ?obj-2-name)
    (below ?_ ?obj-1-name ?obj-2-name))
  
  (<- (below ?w ?obj-1-name ?obj-2-name)
    (bullet-world ?w)
    (-> (lisp-type ?obj-1-name symbol)
        (%object ?w ?obj-1-name ?obj-1)
        (true))
    (-> (lisp-type ?obj-2-name symbol)
        (%object ?w ?obj-2-name ?obj-2)
        (true))
    (%below ?w ?obj-1 ?obj-2)
    (%object ?w ?obj-1-name ?obj-1)
    (%object ?w ?obj-2-name ?obj-2))
  
  (<- (%below ?_ ?obj-1 ?obj-2)
    (bound ?obj-1)
    (bound ?obj-2)
    (lisp-type ?obj-1 object)
    (lisp-type ?obj-2 object)
    (lisp-pred below-p ?obj-1 ?obj-2))

  (<- (%below ?w ?obj-1 ?obj-2)
    (not (bound ?obj-1))
    (bound ?obj-2)
    (lisp-type ?obj-2 object)    
    (lisp-fun find-objects-below ?w ?obj-2 ?objs)
    (member ?obj-1 ?objs))

  (<- (%below ?w ?obj-1 ?obj-2)
    (bound ?obj-1)
    (not (bound ?obj-2))
    (lisp-type ?obj-1 object)    
    (lisp-fun find-objects-above ?w ?obj-1 ?objs)
    (member ?obj-2 ?objs)))

(def-fact-group visibility ()
  (<- (visible ?robot ?object)
    (visible ?_ ?object))

  (<- (visible ?world ?robot ?object)
    (robot ?robot)
    (camera-frame ?camera-frame)
    (link-pose ?robot ?camera-frame ?camera-pose)
    (visible-from ?world ?camera-pose ?object))
  
  (<- (visible-from ?camera-pose ?obj)
    (visible-from ?_ ?camera-pose ?obj))
  
  (<- (visible-from ?world ?camera-pose ?obj-name)
    (bound ?camera-pose)
    (bullet-world ?world)
    (%object ?world ?obj-name ?obj)
    (lisp-pred object-visible-p ?world ?camera-pose ?obj))

  (<- (occluding-objects ?world ?camera-pose ?obj-name ?occluding-names)
    (bound ?camera-pose)
    (bullet-world ?world)
    (%object ?world ?obj-name ?obj)
    (lisp-fun occluding-objects ?world ?camera-pose ?obj ?objs)
    (findall ?occ-name (and (member ?occ ?objs)
                            (%object ?world ?occ-name ?occ))
             ?occluding-names))

  (<- (occluding-object ?camera-pose ?obj ?occluding-obj)
    (occluding-object ?_ ?camera-pose ?obj ?occluding-obj))
  
  (<- (occluding-object ?world ?camera-pose ?obj ?occluding-obj)
    (occluding-objects ?world ?camera-pose ?obj ?objs)
    (member ?occluding-obj ?objs)))

(def-fact-group reachability (object-grasp)
  (<- (grasp :top))
  (<- (grasp :side))
  (<- (grasp :front))

  ;; This needs to be implemented for different object types.
  (<- (object-grasp ?object ?grasp)
    (fail))

  (<- (side :right))
  (<- (side :left))

  (<- (reachable ?robot-name ?obj-name)
    (reachable ?_ ?robot-name ?obj-name))

  (<- (reachable ?robot-name ?obj-name ?side)
    (reachable ?_ ?robot-name ?obj-name ?side))
  
  (<- (reachable ?w ?robot-name ?obj-name)
    (once (reachable ?w ?robot-name ?obj-name ?_)))

  (<- (reachable ?w ?robot-name ?obj-name ?side)
    (bullet-world ?w)
    (side ?side)
    (%object ?w ?robot-name ?robot)
    (lisp-type ?robot robot-object)
    (%object ?w ?obj-name ?obj)
    (with-stored-world ?w
      (once
       (robot-pre-grasp-joint-states ?pre-grasp-joint-states)
       (assert (joint-state ?w ?robot-name ?pre-grasp-joint-states))
       (object-grasp ?obj-name ?grasp)
       (lisp-pred object-reachable-p ?robot ?obj :side ?side :grasp ?grasp)
       (format "reachable~%"))))

  (<- (blocking ?robot-name ?obj-name ?blocking-names)
    (blocking ?_ ?robot-name ?obj-name ?blocking-names))

  (<- (blocking ?robot-name ?obj-name ?side ?blocking-names)
    (blocking ?_ ?robot-name ?obj-name ?side ?blocking-names))  
  
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
    (side ?side)
    (with-stored-world ?w
      (-> (setof
           ?o
           (and
            (object-grasp ?obj-name ?grasp)
            ;; We don't want to have the supporting object as a blocking
            ;; object.
            (-> (supported-by ?w ?obj-name ?supporting) (true) (true))
            ;; Generate all ik solutions
            (once
             (robot-pre-grasp-joint-states ?pre-grasp-joint-states)
             (assert (joint-state ?w ?robot-name ?pre-grasp-joint-states))
             (object-grasp ?obj-name ?grasp)
             (lisp-fun cl-transforms:make-identity-rotation ?identity-rotation)
             (lisp-fun reach-object-ik ?robot ?obj
                       :side ?side :grasp ?grasp :orientation-in-robot ?identity-rotation
                       ?ik-solutions)
             (lisp-pred identity ?ik-solutions))
            (member ?ik-solution ?ik-solutions)
            (%ik-solution-in-collision ?w ?robot ?ik-solution ?colliding-objects)
            (lisp-fun break ?_)
            (member ?o ?colliding-objects)
            (not (== ?o ?obj-name))
            (-> (bound ?supporting) (not (== ?o ?supporting)) (true)))
           ?objs)
          (true)
          (== ?objs ()))))
  
  (<- (%ik-solution-in-collision ?w ?robot ?ik-solution ?colliding-objects)
    (lisp-fun set-robot-state-from-joints ?ik-solution ?robot ?_)
    (%object ?w ?robot-name ?robot)
    (findall ?obj (contact ?w ?robot-name ?obj) ?colliding-objects)))

(def-fact-group debug ()
  (<- (debug-window ?world)
    (bullet-world ?world)
    (lisp-fun add-debug-window ?world ?_))

  (<- (debug-costmap ?costmap)
    (lisp-fun add-costmap-function-object ?costmap ?_))

  (<- (debug-costmap ?costmap ?z)
    (lisp-fun add-costmap-function-object ?costmap ?z ?_)))
