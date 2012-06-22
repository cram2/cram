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

(in-package :btr-desig)

(defvar *max-location-samples* 50)

(defmethod costmap-generator-name->score ((name (eql 'reachable-from-space)))
  5)

(defmethod costmap-generator-name->score ((name (eql 'reachable-from-weighted)))
  4)

(defmethod costmap-generator-name->score ((name (eql 'on-bounding-box)))
  5)

(def-fact-group bullet-reasoning-location-desig (desig-costmap
                                                 desig-loc
                                                 desig-location-prop)
  (<- (desig-costmap ?desig ?cm)
    (costmap ?cm)
    (desig-prop ?desig (reachable-from ?pose))
    (lisp-fun cl-transforms:origin ?pose ?point)
    (costmap-in-reach-distance ?distance)
    (costmap-add-function reachable-from-space
                          (make-range-cost-function ?point ?distance)
                          ?cm)
    (costmap-add-function reachable-from-weighted
                          (make-location-cost-function ?pose ?distance)
                          ?cm))

  (<- (desig-costmap ?designator ?costmap)
    (desig-prop ?designator (on ?object))
    (bullet-world ?world)
    (%object ?world ?object ?object-instance)
    (costmap ?costmap)
    (costmap-add-function
     on-bounding-box (make-object-bounding-box-costmap-generator ?object-instance)
     ?costmap)
    (costmap-add-cached-height-generator
     (make-object-bounding-box-height-generator ?object-instance)
     ?costmap))

  (<- (desig-location-prop ?desig ?loc)
    (loc-desig? ?desig)
    (or (desig-prop ?desig (obj ?o))
        (desig-prop ?desig (object ?o)))
    (btr:object ?_ ?o)
    (btr:pose ?_ ?o ?loc))

  (<- (desig-location-prop ?o ?loc)
    (btr:object ?_ ?o)
    (btr:pose ?_ ?o ?loc))

  (<- (desig-check-to-reach ?desig ?robot-pose)
    (bullet-world ?w)
    (robot ?robot)
    (assert (object-pose ?w ?robot ?robot-pose))
    (forall (contact ?w ?robot ?object)
            (attached ?w ?robot ?_ ?object))
    (forall (designator-reach-pose ?desig ?pose ?side)
            (or
             (and
              (lisp-type ?pose cl-transforms:pose)
              (pose-reachable ?w ?robot ?pose ?side))
             (and
              (lisp-type ?pose cl-transforms:3d-vector)
              (point-reachable ?w ?robot ?pose ?side)))))

  (<- (desig-check-to-see ?desig ?robot-pose)
    (bullet-world ?w)
    (robot ?robot)
    (camera-frame ?cam-frame)
    (desig-location-prop ?desig ?obj-pose)
    (assert (object-pose ?w ?robot ?robot-pose))
    (not (contact ?w ?robot ?_))
    (head-pointing-at ?w ?robot ?obj-pose)
    (desig-prop ?desig (obj ?obj))
    (-> (btr:object ?w ?obj)
        (visible ?w ?robot ?obj)
        (true)))

  (<- (desig-check-articulated-object-manipulation
       ?action-designator ?robot-pose)
    (bullet-world ?world)
    (robot ?robot)
    (desig-prop ?action-designator (to open))
    (desig-prop ?action-designator (handle ?handle))
    (lisp-fun newest-valid-designator ?handle ?current-handle)
    (-> (lisp-pred identity ?current-handle)
        (desig-prop ?current-handle (name ?handle-name))
        ;; Note(moesenle): This is not clean, it helps with debugging
        ;; though.
        (desig-prop ?handle (name ?handle-name)))
    (semantic-map ?world ?semantic-map)
    (with-stored-world ?world
      (btr:execute ?world (btr:open ?semantic-map ?handle-name))
      (not (contact ?world ?robot ?semantic-map))))

  (<- (location-valid
       ?desig ?pose
       (desig-check-articulated-object-manipulation ?action ?pose))
    (desig-prop ?desig (to execute))
    (desig-prop ?desig (action ?action))
    (desig-prop ?action (to open))
    (desig-prop ?action (handle ?_)))

  (<- (location-valid
       ?desig ?pose
       (desig-check-to-see ?desig ?pose))
    (desig-prop ?desig (to see))
    (desig-prop ?desig (obj ?obj)))

  (<- (location-valid
       ?desig ?pose
       (desig-check-to-reach ?desig ?pose))
    (reachability-designator ?desig))

  (<- (btr-desig-solution-valid ?desig ?solution)
    (bullet-world ?w)
    (findall ?check (location-valid ?desig ?solution ?check)
             ?checks)
    (with-stored-world ?w
      (forall (member ?check ?checks) (call ?check))))
  
  (<- (btr-desig-solutions ?desig ?points)
    (merged-desig-costmap ?desig ?cm)
    (debug-costmap ?cm 0.0)
    (costmap-samples ?cm ?solutions)
    (symbol-value *max-location-samples* ?max-samples)
    (take ?max-samples ?solutions ?n-solutions)
    (bagof ?point (and
                   (member ?point ?n-solutions)
                   (btr-desig-solution-valid ?desig ?point))
           ?points)))


