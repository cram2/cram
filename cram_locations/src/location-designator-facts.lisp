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
    (desig:desig-prop ?designator (:object ?object))
    (desig:desig-prop ?object (:name ?object-name))
    (btr:bullet-world ?world)
    ;; (btr-belief:object-designator-name ?object ?object-name)
    (cram-robot-interfaces:robot ?robot)
    (costmap:costmap ?costmap)
    (visibility-costmap-metadata ?minimal-height ?maximal-height ?resolution ?size)
    (costmap:costmap-add-function
     visible
     (make-object-visibility-costmap
      ?world ?object-name ?robot
      ?minimal-height ?maximal-height ?size ?resolution)
     ?costmap))

  ;; (<- (unknown-object-visibility-costmap ?designator ?costmap)
  ;;   ;; object hasn't been perceived yet and
  ;;   ;; (OBJECT-VISIBILITY-COSTMAP ?des ?cm) failed
  ;;   (desig:desig-prop ?designator (:object ?object))
  ;;   (desig:desig-prop ?object (:at ?location))
  ;;   (btr:bullet-world ?world)
  ;;   (cram-robot-interfaces:robot ?robot)
  ;;   (costmap:costmap ?costmap)
  ;;   (visibility-costmap-metadata ?minimal-height ?maximal-height ?resolution ?size)
  ;;   (costmap:costmap-add-function
  ;;    visible
  ;;    (make-location-visibility-costmap
  ;;     ?world ?location ?robot
  ;;     ?minimal-height ?maximal-height ?size ?resolution)
  ;;    ?costmap))

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
              ;; (unknown-object-visibility-costmap ?designator ?costmap)
              (location-visibility-costmap ?designator ?costmap))))

  (<- (desig-check-to-see ?desig ?robot-pose)
    ;; (desig:desig-prop ?desig (:object ?obj))
    ;; (desig:desig-location-prop ?desig ?obj-pose)
    (desig:desig-prop ?desig (:object ?some-object))
    (desig:current-designator ?some-object ?object)
    (lisp-fun man-int:get-object-pose-in-map ?object ?to-see-pose)
    (-> (lisp-pred identity ?to-see-pose)
        (and (btr:bullet-world ?w)
             (cram-robot-interfaces:robot ?robot)
             (assert (btr:object-pose ?w ?robot ?robot-pose))
             (btr:object-not-in-collision ?w ?robot)
             (cram-robot-interfaces:camera-frame ?robot ?cam-frame)
             (btr:head-pointing-at ?w ?robot ?to-see-pose)
             (desig:desig-prop ?object (:name ?object-name))
             (-> (btr:object ?w ?object-name)
                 (btr:visible ?w ?robot ?object-name)
                 (true)))
        (true)))

  (<- (location-valid ?desig ?pose (desig-check-to-see ?desig ?pose))
    (cram-robot-interfaces:visibility-designator ?desig)
    (or (desig:desig-prop ?desig (:object ?obj))
        (true)))

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
           ?points)))
