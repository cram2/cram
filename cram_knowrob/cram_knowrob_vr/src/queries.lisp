;;;
;;; Copyright (c) 2018, Alina Hawkin <hawkin@cs.uni-bremen.de>
;;;                      Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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

(in-package :kvr)

;;;;;;;;;;;;;;;;;;; UTIL QUERIES USED IN INIT-EPISODE ;;;;;;;;;;;;;;;;;;;;;;;

(defun register-ros-package (package-name)
  (json-prolog:prolog-simple-1
   (concatenate 'string  "register_ros_package(" package-name ")") :mode 1))

(defun u-load-episodes (episodes-path)
  (json-prolog:prolog-simple-1
   (concatenate 'string "u_load_episodes('" episodes-path "')") :mode 1))

(defun owl-parse (semantic-map-path)
  (json-prolog:prolog-simple-1
   (concatenate 'string "owl_parse('" semantic-map-path "')") :mode 1))

(defun connect-to-db (db-name)
  (json-prolog:prolog-simple-1
   (concatenate 'string "connect_to_db('" db-name "')") :mode 1))

(defun map-marker-init ()
  (json-prolog:prolog-simple-1
   "sem_map_inst(MapInst),!,marker_update(object(MapInst))."))

;;;;;;;;;;;;;;;;;;; DATA EXTRACTING QUERIES ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun base-query-string (object-type)
  (concatenate
   'string
   "obj_type(ObjInst, knowrob:'" object-type "'),
    ep_inst(EpInst),
    u_occurs(EpInst, EventInst, Start, End),
    obj_type(EventInst, knowrob:'GraspingSomething'),
    rdf_has(EventInst, knowrob:'objectActedOn', ObjInst)"))

(defun query-grasping-event-by-object-type (object-type)
  (declare (type string object-type))
  "returns the event which was performed on the given object."
  (cut:lazy-mapcar
   (lambda (binding-set)
     (cut:var-value '|?EventInst| binding-set))
   (json-prolog:prolog-simple
    (concatenate
     'string
     (base-query-string object-type) ".")
    :package :kvr)))

(defun query-object-location-by-object-type (object-type start-or-end)
  (declare (type string object-type start-or-end))
  "returns the pose of an object at the start of the event performed on it.
`start-or-end' has to be either Start or End."
  (assert (or (equal start-or-end "Start") (equal start-or-end "End")))
  (cut:lazy-mapcar
   (lambda (binding-set)
     (cram-tf:flat-list->transform
      (cut:var-value '?pose binding-set)))
   (json-prolog:prolog-simple
    (concatenate
     'string
     (base-query-string object-type)",
      iri_xml_namespace(ObjInst, _, ObjShortName),
      actor_pose(EpInst, ObjShortName, " start-or-end ", POSE).")
    :package :kvr)))

(defun query-camera-location-by-object-type (object-type start-or-end)
  (declare (type string object-type start-or-end))
  "returns the pose of the camera (head) at the start of the event."
  (assert (or (equal start-or-end "Start") (equal start-or-end "End")))
  (cut:lazy-mapcar
   (lambda (binding-set)
     (cram-tf:flat-list->transform
      (cut:var-value '|?PoseCameraStart| binding-set)))
   (json-prolog:prolog-simple
    (concatenate
     'string
     (base-query-string object-type) ",
      obj_type(CameraInst, knowrob:'CharacterCamera'),
      iri_xml_namespace(CameraInst, _, CameraShortName),
      actor_pose(EpInst, CameraShortName, " start-or-end ", PoseCameraStart).")
    :package :kvr)))

(defun query-object-T-camera-by-object-type (object-type start-or-end)
  (declare (type string object-type start-or-end))
  "returns the transform object-T-camera at the start or end of grasping event.
`start-or-end' has to be either Start or End."
  (assert (or (equal start-or-end "Start") (equal start-or-end "End")))
  (cut:lazy-mapcar
   (lambda (binding-set)
     (let* ((map-T-obj
              (cram-tf:flat-list->transform
               (cut:var-value '|?ObjPose| binding-set)))
            (map-T-camera
              (cram-tf:flat-list->transform
               (cut:var-value '|?CameraPose| binding-set)))
            (obj-T-map
              (cl-transforms:transform-inv
               map-T-obj))
            (obj-T-cam
              (cl-transforms:transform*
               obj-T-map map-T-camera)))
       obj-T-cam))
   (json-prolog:prolog-simple
    (concatenate
     'string
     (base-query-string object-type)",
      iri_xml_namespace(ObjInst, _, ObjShortName),
      actor_pose(EpInst, ObjShortName, " start-or-end ", ObjPose),
      obj_type(CameraInst, knowrob:'CharacterCamera'),
      iri_xml_namespace(CameraInst, _, CameraShortName),
      actor_pose(EpInst, CameraShortName, " start-or-end ", CameraPose).")
    :package :kvr)))

(defun query-hand (object-type)
  "Returns which hand was used to interact with the object
in the currently loaded episode."
  (cut:lazy-mapcar
   (lambda (binding-set)
     (let ((hand-string
             (string
              (cut:var-value '|?HandTypeName| binding-set))))
       (if (search "Left" hand-string)
           :left
           (if (search "Right" hand-string)
               :right
               NIL))))
   (json-prolog:prolog-simple
    (concatenate
     'string
     (base-query-string object-type) ",
     performed_by(EventInst, HandInst),
     obj_type(HandInst, HandType),
     iri_xml_namespace(HandType, _, HandTypeName).")
    :package :kvr)))

(defun query-hand-location-by-object-type (object-type start-or-end)
  (declare (type string object-type start-or-end))
  "returns the pose of the hand at the start of the event performed by it."
  (assert (or (equal start-or-end "Start") (equal start-or-end "End")))
  (cut:lazy-mapcar
   (lambda (binding-set)
     (cram-tf:flat-list->transform
      (cut:var-value
       '|?PoseHandStart| binding-set)))
   (json-prolog:prolog-simple
    (concatenate
     'string
     (base-query-string object-type) ",
      performed_by(EventInst, HandInst),
      iri_xml_namespace(HandInst,_, HandInstShortName),
      actor_pose(EpInst, HandInstShortName, " start-or-end ", PoseHandStart).")
    :package :kvr)))

(defun query-contact-surface-name (object-type start-or-end)
  (declare (type string object-type start-or-end))
  "Returns the name of the object surface an object is picked up from."
  (assert (or (equal start-or-end "Start") (equal start-or-end "End")))
  (cut:lazy-mapcar
   (lambda (binding-set)
     (cut:var-value '|?SurfaceShortName| binding-set))
   ;; one could drop the object into the air,
   ;; such that the contact would only happen at (End + 1 second) or so
   ;; therefore, we calculate EndWithOffset to compare with that one
   ;; for readability we also create StartWithOffset
   ;; although it's the same as Start   (json-prolog:prolog-simple
   (json-prolog:prolog-simple
    (concatenate
     'string
     (base-query-string object-type) ",
      event_type(TouchingEventInst, knowrob_u:'TouchingSituation'),
      rdf_has(TouchingEventInst, knowrob_u:'inContact', ObjInst),
      rdf_has(TouchingEventInst, knowrob_u:'inContact', Surface),
      not(ObjInst==Surface),
      u_occurs(EpInst, TouchingEventInst, TouchStart, TouchEnd),
      StartWithOffset = Start,
      time_term(End, EndSeconds),
      EndWithOffset is EndSeconds + 1,
      time_between(" start-or-end "WithOffset, TouchStart, TouchEnd),
      iri_xml_namespace(Surface, _, SurfaceShortName).")
    :package :kvr)))

(defun query-contact-surface-transform (object-type start-or-end)
  (declare (type string object-type start-or-end))
  "Returns the name of the object surface an object is picked up from."
  (assert (or (equal start-or-end "Start") (equal start-or-end "End")))
  (cut:lazy-mapcar
   (lambda (binding-set)
     (cram-tf:flat-list->transform
      (cut:var-value '|?PoseSurface| binding-set)))
   (json-prolog:prolog-simple
    (concatenate
     'string
     (base-query-string object-type) ",
      event_type(TouchingEventInst, knowrob_u:'TouchingSituation'),
      rdf_has(TouchingEventInst, knowrob_u:'inContact', ObjInst),
      rdf_has(TouchingEventInst, knowrob_u:'inContact', Surface),
      not(ObjInst==Surface),
      u_occurs(EpInst, TouchingEventInst, TouchStart, TouchEnd),
      StartWithOffset = Start,
      time_term(End, EndSeconds),
      EndWithOffset is EndSeconds + 1,
      time_between(" start-or-end "WithOffset, TouchStart, TouchEnd),
      iri_xml_namespace(Surface, _, SurfaceShortName),
      actor_pose(EpInst, SurfaceShortName, Touch" start-or-end ", PoseSurface).")
    :package :kvr)))


(defun query-name-and-surface-T-object-by-object-type (object-type
                                                       start-or-end
                                                       &optional context)
  (declare (type string object-type start-or-end)
           (type (or null keyword) context))
  "Returns the name of the supporting surface an object is picked up from
and the transform surface-T-object as a lazy list of pairs:
 '((name-1 . surface-T-object-1) . rest-of-lazy-list)."
  (assert (or (equal start-or-end "Start") (equal start-or-end "End")))
  (cut:lazy-mapcar
   (lambda (binding-set)
     (let* ((surface-name
              (cut:var-value '|?SurfaceShortName| binding-set))
            (map-T-object
              (cram-tf:flat-list->transform
               (cut:var-value '|?ObjectPose| binding-set)))
            (map-T-surface
              (cram-tf:flat-list->transform
               (cut:var-value '|?SurfacePose| binding-set)))
            (surface-T-map
              (cl-transforms:transform-inv
               map-T-surface))
            (surface-T-object
              (cl-transforms:transform*
               surface-T-map map-T-object)))
       (cons surface-name surface-T-object)))
   (json-prolog:prolog-simple
    (concatenate
     'string
     (base-query-string object-type) ",
      event_type(TouchingEventInst, knowrob_u:'TouchingSituation'),
      rdf_has(TouchingEventInst, knowrob_u:'inContact', ObjInst),
      rdf_has(TouchingEventInst, knowrob_u:'inContact', Surface),
      not(ObjInst==Surface),
      u_occurs(EpInst, TouchingEventInst, TouchStart, TouchEnd),
      StartWithOffset = Start,
      time_term(End, EndSeconds),
      EndWithOffset is EndSeconds + 1,
      time_between(" start-or-end "WithOffset, TouchStart, TouchEnd),
      iri_xml_namespace(Surface, _, SurfaceShortName),
      actor_pose(EpInst, SurfaceShortName, Touch" start-or-end ", SurfacePose),
      iri_xml_namespace(ObjInst, _, ObjShortName),
      actor_pose(EpInst, ObjShortName, " start-or-end ", ObjectPose),
      obj_type(Surface, SurfaceType)"
      (case context
        ;; if we're setting a table, the source ("Start") should not be island,
        ;; and destination ("End") should be island
        (:table-setting
         (if (equal start-or-end "Start")
             ", not(owl_subclass_of(SurfaceType, knowrob:'IslandArea'))."
             ", owl_subclass_of(SurfaceType, knowrob:'IslandArea')."))
        ;; in case of table cleaning it's the other way around:
        ;; source "Start" should be island, destination "End" should not be island
        (:table-cleaning
         (if (equal start-or-end "Start")
             ", owl_subclass_of(SurfaceType, knowrob:'IslandArea')."
             ", not(owl_subclass_of(SurfaceType, knowrob:'IslandArea'))."))
        (t
         ".")))
    :package :kvr)))


(defun query-name-and-surface-T-camera-by-object-type (object-type
                                                       start-or-end
                                                       &optional context)
  (declare (type string object-type start-or-end)
           (type (or null keyword) context))
  "Returns the name of the supporting surface an object is picked up from
and the transform surface-T-camera as a lazy list of pairs:
 '((name-1 . surface-T-camera-1) . rest-of-lazy-list)."
  (assert (or (equal start-or-end "Start") (equal start-or-end "End")))
  (cut:lazy-mapcar
   (lambda (binding-set)
     (let* ((surface-name
              (cut:var-value '|?SurfaceShortName| binding-set))
            (map-T-camera
              (cram-tf:flat-list->transform
               (cut:var-value '|?CameraPose| binding-set)))
            (map-T-surface
              (cram-tf:flat-list->transform
               (cut:var-value '|?SurfacePose| binding-set)))
            (surface-T-map
              (cl-transforms:transform-inv
               map-T-surface))
            (surface-T-camera
              (cl-transforms:transform*
               surface-T-map map-T-camera)))
       (cons surface-name surface-T-camera)))
   (json-prolog:prolog-simple
    (concatenate
     'string
     (base-query-string object-type) ",
      event_type(TouchingEventInst, knowrob_u:'TouchingSituation'),
      rdf_has(TouchingEventInst, knowrob_u:'inContact', ObjInst),
      rdf_has(TouchingEventInst, knowrob_u:'inContact', Surface),
      not(ObjInst==Surface),
      u_occurs(EpInst, TouchingEventInst, TouchStart, TouchEnd),
      StartWithOffset = Start,
      time_term(End, EndSeconds),
      EndWithOffset is EndSeconds + 1,
      time_between(" start-or-end "WithOffset, TouchStart, TouchEnd),
      iri_xml_namespace(Surface, _, SurfaceShortName),
      actor_pose(EpInst, SurfaceShortName, Touch" start-or-end ", SurfacePose),
      obj_type(CameraInst, knowrob:'CharacterCamera'),
      iri_xml_namespace(CameraInst, _, CameraShortName),
      actor_pose(EpInst, CameraShortName, " start-or-end ", CameraPose),
      obj_type(Surface, SurfaceType)"
      (case context
        ;; if we're setting a table, the source ("Start") should not be island,
        ;; and destination ("End") should be island
        (:table-setting
         (if (equal start-or-end "Start")
             ", not(owl_subclass_of(SurfaceType, knowrob:'IslandArea'))."
             ", owl_subclass_of(SurfaceType, knowrob:'IslandArea')."))
        ;; in case of table cleaning it's the other way around:
        ;; source "Start" should be island, destination "End" should not be island
        (:table-cleaning
         (if (equal start-or-end "Start")
             ", owl_subclass_of(SurfaceType, knowrob:'IslandArea')."
             ", not(owl_subclass_of(SurfaceType, knowrob:'IslandArea'))."))
        (t
         ".")))
    :package :kvr)))



#+this-is-not-used
(defun query-table-location (object-type)
  (car
   (cut:lazy-mapcar
    (lambda (binding-set)
      (cram-tf:flat-list->transform
       (cut:var-value
        '|?PoseTable| binding-set)))
    (json-prolog:prolog-simple
     (concatenate
      'string
      (base-query-string object-type) ",
      obj_type(TableInst, knowrob:'IslandArea'),
      iri_xml_namespace(TableInst, _, TableShortName),
      actor_pose(EpInst, TableShortName, Start, PoseTable).")
     :package :kvr))))
