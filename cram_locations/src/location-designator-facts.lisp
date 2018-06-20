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

(in-package :cram-btr-visibility-costmap)

(defvar *max-location-samples* 50)

(defmethod costmap:costmap-generator-name->score ((name (eql 'visible))) 20)

(def-fact-group bullet-reasoning-location-desig (costmap:desig-costmap)

  (<- (visibility-costmap-metadata ?minimal-height ?maximal-height ?resolution ?size)
    (cram-robot-interfaces:robot ?robot)
    (cram-robot-interfaces:camera-minimal-height ?robot ?minimal-height)
    (cram-robot-interfaces:camera-maximal-height ?robot ?maximal-height)
    (costmap:costmap-resolution ?resolution)
    (costmap:visibility-costmap-size ?size))

  (<- (object-visibility-costmap ?designator ?costmap)
    (or (desig:desig-prop ?designator (:obj ?object))
        (desig:desig-prop ?designator (:object ?object)))
    (btr:bullet-world ?world)
    (btr-belief:object-designator-name ?object ?object-name)
    (cram-robot-interfaces:robot ?robot)
    (costmap:costmap ?costmap)
    (visibility-costmap-metadata ?minimal-height ?maximal-height ?resolution ?size)
    (costmap:costmap-add-function
     visible
     (make-object-visibility-costmap
      ?world ?object-name ?robot
      ?minimal-height ?maximal-height ?size ?resolution)
     ?costmap))

  (<- (unknown-object-visibility-costmap ?designator ?costmap)
    ;; object hasn't been perceived yet and
    ;; (OBJECT-VISIBILITY-COSTMAP ?des ?cm) failed
    (or (desig:desig-prop ?designator (:obj ?object))
        (desig:desig-prop ?designator (:object ?object)))
    (desig:desig-prop ?object (:at ?location))
    (btr:bullet-world ?world)
    (cram-robot-interfaces:robot ?robot)
    (costmap:costmap ?costmap)
    (visibility-costmap-metadata ?minimal-height ?maximal-height ?resolution ?size)
    (costmap:costmap-add-function
     visible
     (make-location-visibility-costmap
      ?world ?location ?robot
      ?minimal-height ?maximal-height ?size ?resolution)
     ?costmap))

  (<- (location-visibility-costmap ?designator ?costmap)
    (desig:desig-prop ?designator (:location ?location))
    (btr:bullet-world ?world)
    (cram-robot-interfaces:robot ?robot)
    (costmap:costmap ?costmap)
    (visibility-costmap-metadata ?minimal-height ?maximal-height ?resolution ?size)
    (costmap:costmap-add-function
     visible
     (make-location-visibility-costmap
      ?world ?location ?robot
      ?minimal-height ?maximal-height ?size ?resolution)
     ?costmap))

  (<- (costmap:desig-costmap ?designator ?costmap)
    (cram-robot-interfaces:visibility-designator ?designator)
    (once (or (object-visibility-costmap ?designator ?costmap)
              (unknown-object-visibility-costmap ?designator ?costmap)
              (location-visibility-costmap ?designator ?costmap))))

  (<- (desig-check-to-see ?desig ?robot-pose)
    (or (desig:desig-prop ?desig (:obj ?obj))
        (desig:desig-prop ?desig (:object ?obj)))
    (desig:desig-location-prop ?desig ?obj-pose)
    (btr:bullet-world ?w)
    (cram-robot-interfaces:robot ?robot)
    (assert (btr:object-pose ?w ?robot ?robot-pose))
    (btr:object-not-in-collision ?w ?robot)
    (cram-robot-interfaces:camera-frame ?robot ?cam-frame)
    (btr:head-pointing-at ?w ?robot ?obj-pose)
    (-> (btr:object ?w ?obj)
        (btr:visible ?w ?robot ?obj)
        (true)))

  (<- (location-valid ?desig ?pose (desig-check-to-see ?desig ?pose))
    (cram-robot-interfaces:visibility-designator ?desig)
    (or (desig:desig-prop ?desig (:obj ?obj))
        (desig:desig-prop ?desig (:object ?obj))))

  (<- (btr-desig-solution-valid ?desig ?solution)
    (btr-desig-solution-valid ?desig ?solution ?_))

  (<- (btr-desig-solution-valid ?desig ?solution ?checks)
    (btr:bullet-world ?w)
    (findall ?check (location-valid ?desig ?solution ?check)
             ?checks)
    (btr:with-copied-world ?w
      (forall (member ?check ?checks) (call ?check))))

  (<- (btr-desig-solutions ?desig ?points)
    (costmap:merged-desig-costmap ?desig ?cm)
    (btr:debug-costmap ?cm 0.0)
    (costmap:costmap-samples ?cm ?solutions)
    (symbol-value *max-location-samples* ?max-samples)
    (take ?max-samples ?solutions ?n-solutions)
    (bagof ?point (and
                   (member ?point ?n-solutions)
                   (btr-desig-solution-valid ?desig ?point))
           ?points))


  ;; EVERYTHING BELOW IS RELATED TO REACHABILITY COSTMAP AND IS DEPRECATED
  ;; (<- (desig-check-to-reach ?desig ?robot-pose)
  ;;   (bullet-world ?w)
  ;;   (robot ?robot)
  ;;   (assert (object-pose ?w ?robot ?robot-pose))
  ;;   (object-not-in-collision ?w ?robot)
  ;;   (forall (cram-robot-interfaces:designator-reach-pose ?desig ?robot-pose ?pose ?side)
  ;;           (or
  ;;            (and
  ;;             (lisp-type ?pose cl-transforms:pose)
  ;;             (pose-reachable ?w ?robot ?pose ?side))
  ;;            (and
  ;;             (lisp-type ?pose cl-transforms:3d-vector)
  ;;             (point-reachable ?w ?robot ?pose ?side)))))

  ;; (<- (desig-check-articulated-object-manipulation
  ;;      ?action-designator ?robot-pose)
  ;;   (bullet-world ?world)
  ;;   (robot ?robot)
  ;;   (desig-prop ?action-designator (:to :open))
  ;;   (desig-prop ?action-designator (:handle ?handle))
  ;;   (lisp-fun newest-effective-designator ?handle ?current-handle)
  ;;   (-> (lisp-pred identity ?current-handle)
  ;;       (desig-prop ?current-handle (:name ?handle-name))
  ;;       ;; Note(moesenle): This is not clean, it helps with debugging
  ;;       ;; though.
  ;;       (desig-prop ?handle (:name ?handle-name)))
  ;;   (semantic-map ?world ?semantic-map)
  ;;   (assert (object-pose ?world ?robot ?robot-pose))
  ;;   (with-copied-world ?world
  ;;     (btr:execute ?world (btr:open ?semantic-map ?handle-name))
  ;;     (not (contact ?world ?robot ?semantic-map))))

  ;; (<- (location-valid
  ;;      ?desig ?pose
  ;;      (desig-check-articulated-object-manipulation ?action ?pose))
  ;;   (desig-prop ?desig (:to :execute))
  ;;   (desig-prop ?desig (:action ?action))
  ;;   (desig-prop ?action (:to :open))
  ;;   (desig-prop ?action (:handle ?_)))

  ;; (<- (location-valid
  ;;      ?desig ?pose
  ;;      (desig-check-to-reach ?desig ?pose))
  ;;   (cram-robot-interfaces:reachability-designator ?desig))
)
