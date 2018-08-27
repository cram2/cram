;;; NOTE: old functions which are not needed anymore but are currently kept for
;;; reference.
(in-package :kvr)


; TODO is there a more elegant solution for this? 
; A lazy list of all the data extracted from OpenEase for the current episode. Mainly poses.
(defvar *orig-poses-list* nil)

; A lazy list of all the data extracted from OpenEase for the current event. Mainly poses.
(defvar *poses-list* nil)


(defun get-grasp-something-poses ()
"Queries OpenEase for the episode data. This can take a while. Saves the
result into the *orig-poses-list* and the *poses-list* variables.
RESULT: List of timestamps, objects and their poses."
  (setq *orig-poses-list*
        (prolog-simple "ep_inst(EpInst),
          u_occurs(EpInst, EventInst, Start, End),
          event_type(EventInst, knowrob:'GraspingSomething'),
          rdf_has(EventInst, knowrob:'objectActedOn',ObjActedOnInst),
          performed_by(EventInst, HandInst),
          iri_xml_namespace(HandInst,_, HandInstShortName),
          obj_type(HandInst, HandType),

          iri_xml_namespace(HandType, _, HandTypeName),
          iri_xml_namespace(ObjActedOnInst, _, ObjShortName),

          obj_type(CameraInst, knowrob:'CharacterCamera'),
          iri_xml_namespace(CameraInst, _, CameraShortName),

          actor_pose(EpInst, ObjShortName, Start, PoseObjStart),
          actor_pose(EpInst, ObjShortName, End, PoseObjEnd),

          actor_pose(EpInst, CameraShortName, Start, PoseCameraStart),
          actor_pose(EpInst, CameraShortName, End, PoseCameraEnd),

          actor_pose(EpInst, HandInstShortName, Start, PoseHandStart),
          actor_pose(EpInst, HandInstShortName, End, PoseHandEnd)."))
  (setq *poses-list* (cut:lazy-car *orig-poses-list*)))


;; TODO use object type or something instead of just the count... Or a
;; combination of both? or with time stamps?
(defun get-next-obj-poses (count)
  "Extracts the data of a specific event within the current episode.
The event can be currently specifies with the count parameter. Also sets the
*poses-list* variable to the new event data.
COUNT: Starting from 0. Specifies which GraspingSomething event data is to be loaded.
RETURNS: A list containing that specific events data, saved in *poses-list* "
  (if (setq *poses-list*  (cut:lazy-elt *orig-poses-list* count))
      (progn
        (format t "Poses from event nr. ~D ." count)
        *poses-list*)
      (format t "No poses for event nr. ~D available. There are no more events for this query. Query result was NIL. " count)))



(defun get-all-episode-data-on-type (object-type)
  "gets all poses of necessary objects and human in on ecall.
Can be used to make sure the indivisual queries data is correct."
  ;;make sure obj is of correct type
  (setq object-type (object-type-filter object-type)) 
  (prolog-simple 
   (concatenate 'string
                "owl_has(Obj, rdf:type, knowrob:'" object-type "'),
                 rdf_has(EventInst, knowrob:'objectActedOn', Obj),
                 rdf_has(EventInst, knowrob:'objectActedOn', ObjActedOnInst),
                 u_occurs(EpInst, EventInst, Start, End),

                 performed_by(EventInst, HandInst),
                 iri_xml_namespace(HandInst,_, HandInstShortName),
                 obj_type(HandInst, HandType),

                 iri_xml_namespace(HandType, _, HandTypeName),
                 iri_xml_namespace(ObjActedOnInst, _, ObjShortName),

                 obj_type(CameraInst, knowrob:'CharacterCamera'),
                 iri_xml_namespace(CameraInst, _, CameraShortName),

                 actor_pose(EpInst, ObjShortName, Start, PoseObjStart),
                 actor_pose(EpInst, ObjShortName, End, PoseObjEnd),

                 actor_pose(EpInst, CameraShortName, Start, PoseCameraStart),
                 actor_pose(EpInst, CameraShortName, End, PoseCameraEnd),

                 actor_pose(EpInst, HandInstShortName, Start, PoseHandStart),
                 actor_pose(EpInst, HandInstShortName, End, PoseHandEnd).")))


