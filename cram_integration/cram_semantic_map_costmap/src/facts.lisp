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

(in-package :semantic-map-costmap)

(defmethod costmap-generator-name->score ((name (eql 'semantic-map-objects)))
  10)

(defmethod costmap-generator-name->score ((name (eql 'table-distribution)))
  9)

(defmethod costmap-generator-name->score ((name (eql 'semantic-map-free-space)))
  11)

(defmethod costmap-generator-name->score ((name (eql 'center-of-object)))
  12)

(def-fact-group semantic-map-costmap (desig-costmap
                                      desig-z-value)

  ;; relation-tag is either IN or ON at the moment
  (<- (semantic-map-desig-objects ?desig ?objects)
    (lisp-fun get-semantic-map ?semantic-map)
    (lisp-fun sem-map-desig:designator->semantic-map-objects
              ?desig ?semantic-map ?objects)
    (lisp-pred identity ?objects))

  (<- (semantic-map-objects ?objects)
    (lisp-fun get-semantic-map ?semantic-map)
    (lisp-fun sem-map-utils:semantic-map-parts ?semantic-map
              :recursive nil ?objects))

  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (:on ?_))
    (semantic-map-desig-objects ?desig ?objects)
    (costmap ?cm)
    (costmap-add-function semantic-map-objects
                          (make-semantic-map-costmap ?objects)
                          ?cm)
    (-> (not (desig-prop ?desig (:for ?_)))
        (costmap-add-cached-height-generator
         (make-semantic-map-height-function ?objects :on)
         ?cm)
        (true)))

  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (:in ?_))
    (semantic-map-desig-objects ?desig ?objects)
    (costmap ?cm)
    (costmap-add-function semantic-map-objects
                          (make-semantic-map-costmap ?objects)
                          ?cm)
    (-> (not (desig-prop ?desig (:for ?_)))
        (costmap-add-cached-height-generator
         (make-semantic-map-height-function ?objects :in)
         ?cm)
        (true)))

  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (:centered-with-padding ?padding))
    (costmap ?cm)
    (semantic-map-desig-objects ?desig ?objects)
    (costmap-add-function center-of-object
                          (make-semantic-object-center-costmap ?objects ?padding)
                          ?cm))

  (<- (desig-costmap ?desig ?cm)
    (or (cram-robot-interfaces:visibility-designator ?desig)
        (cram-robot-interfaces:reachability-designator ?desig))
    (costmap ?cm)
    (semantic-map-objects ?objects)
    (costmap-padding ?padding)
    (costmap-add-function semantic-map-free-space
                          (make-semantic-map-costmap
                           ?objects :invert t :padding ?padding)
                          ?cm)
    ;; Locations to see and to reach are on the floor, so we can use a
    ;; constant height of 0
    (costmap-add-cached-height-generator
     (make-constant-height-function 0.0)
     ?cm))

  ;; The desig-z-value and supporting-z-value predicates are sort of
  ;; an evil hack. At the moment they are used by grasping to infer
  ;; the height of the gripper above the supporting object which is
  ;; necessary to put down the object again. In the future, this
  ;; should be replaced by a more general approach
  (<- (desig-z-value ?desig ?point ?z)
    (loc-desig? ?desig)
    (desig-prop ?desig (:on ?_))
    (semantic-map-desig-objects ?desig ?objects)
    (member ?obj ?objects)
    (lisp-pred point-on-object ?obj ?point)
    (lisp-fun obj-z-value ?obj :on ?z))

  (<- (supporting-z-value ?point ?z)
    (semantic-map-objects ?objects)
    (member ?obj ?objects)
    (lisp-pred point-on-object ?obj ?point)
    (lisp-fun obj-z-value ?obj :on ?z)))
