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
  (car
   (cut:lazy-mapcar
    (lambda (binding-set)
      (cut:var-value
       '|?EventInst| binding-set))
    (json-prolog:prolog-simple
     (concatenate
      'string
      (base-query-string object-type) ".")
     :package :kvr))))

(defun query-object-location-by-object-type (object-type start-or-end)
  (declare (type string object-type start-or-end))
  "returns the pose of an object at the start of the event performed on it.
`start-or-end' has to be either Start or End."
  (assert (or (equal start-or-end "Start") (equal start-or-end "End")))
  (car
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
     :package :kvr))))

(defun query-hand (object-type)
  "Returns which hand was used to interact with the object
in the currently loaded episode."
  (let* ((hand
           (car
            (cut:lazy-mapcar
             (lambda (binding-set)
               (cut:var-value
                '|?HandTypeName| binding-set))
             (json-prolog:prolog-simple
              (concatenate 'string
                           (base-query-string object-type) ",
                            performed_by(EventInst, HandInst),
                            obj_type(HandInst, HandType),
                            iri_xml_namespace(HandType, _, HandTypeName).")
              :package :kvr)))))
    (if (search "Left" (string hand))
        :left
        (if (search "Right" (string hand))
            :right
            NIL))))

(defun query-hand-location-by-object-type (object-type start-or-end)
  (declare (type string object-type start-or-end))
  "returns the pose of the hand at the start of the event performed by it."
  (assert (or (equal start-or-end "Start") (equal start-or-end "End")))
  (car
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
     :package :kvr))))

(defun query-camera-location-by-object-type (object-type start-or-end)
  (declare (type string object-type start-or-end))
  "returns the pose of the camera (head) at the start of the event."
  (assert (or (equal start-or-end "Start") (equal start-or-end "End")))
  (car
   (cut:lazy-mapcar
    (lambda (binding-set)
      (cram-tf:flat-list->transform
       (cut:var-value
        '|?PoseCameraStart| binding-set)))
    (json-prolog:prolog-simple
     (concatenate
      'string
      (base-query-string object-type) ",
      obj_type(CameraInst, knowrob:'CharacterCamera'),
      iri_xml_namespace(CameraInst, _, CameraShortName),
      actor_pose(EpInst, CameraShortName, " start-or-end ", PoseCameraStart).")
     :package :kvr))))

(defun query-contact-surface-pick-name (object-type)
  "Returns the name of the object surface an object is picked up from."
  (car
   (cut:lazy-mapcar
    (lambda (binding-set)
      (cut:var-value
       '|?SurfaceShortName| binding-set))
    (json-prolog:prolog-simple
     (concatenate
      'string
      (base-query-string object-type) ",
      event_type(TouchingEventInst, knowrob_u:'TouchingSituation'),
      rdf_has(TouchingEventInst, knowrob_u:'inContact', ObjInst),
      rdf_has(TouchingEventInst, knowrob_u:'inContact', Surface),
      iri_xml_namespace(Surface, _, SurfaceShortName).")
     :package :kvr))))

(defun query-contact-surface-pick-transform (object-type)
  "returns the pose of the surface an object is picked up from."
  (car
   (cut:lazy-mapcar
    (lambda (binding-set)
      (cram-tf:flat-list->transform
       (cut:var-value
        '|?PoseSurface| binding-set)))
    (json-prolog:prolog-simple
     (concatenate
      'string
      (base-query-string object-type) ",
      event_type(TouchingEventInst, knowrob_u:'TouchingSituation'),
      rdf_has(TouchingEventInst, knowrob_u:'inContact', ObjInst),
      rdf_has(TouchingEventInst, knowrob_u:'inContact', Surface),
      iri_xml_namespace(Surface, _, SurfaceShortName),
      u_occurs(EpInst, TouchingEventInst, Start, End),
      actor_pose(EpInst, SurfaceShortName, Start, PoseSurface).")
     :package :kvr))))

(defun query-contact-surface-place-name (object-type)
  "returns the name of the object surface an object is picked up from."
  (car
   (cut:lazy-mapcar
    (lambda (binding-set)
      (cut:var-value
       '|?SurfaceShortName| binding-set))
    (json-prolog:prolog-simple
     (concatenate
      'string
      (base-query-string object-type) ",
      iri_xml_namespace(ObjInst, _, ObjShortName),
      event_type(TouchingEventInst, knowrob_u:'TouchingSituation'),
      u_occurs(EpInst, TouchingEventInst, End, EndNew),
      rdf_has(TouchingEventInst, knowrob_u:'inContact', PlaceSurface),
      iri_xml_namespace(PlaceSurface, _, SurfaceShortName).")
     :package :kvr))))

(defun query-contact-surface-place-transform (object-type)
  "returns the pose of the surface an object is picked up from."
  (car
   (cut:lazy-mapcar
    (lambda (binding-set)
      (cram-tf:flat-list->transform
       (cut:var-value (intern "?PoseSurface") binding-set)))
    (json-prolog:prolog-simple
     (concatenate
      'string
      (base-query-string object-type) ",
      iri_xml_namespace(ObjInst, _, ObjShortName),
      event_type(TouchingEventInst, knowrob_u:'TouchingSituation'),
      u_occurs(EpInst, TouchingEventInst, End, EndNew),
      rdf_has(TouchingEventInst, knowrob_u:'inContact', PlaceSurface),
      iri_xml_namespace(PlaceSurface, _, SurfaceShortName),
      actor_pose(EpInst, SurfaceShortName, EndNew, PoseSurface).")))))


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
