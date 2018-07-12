;;; Contains all the functions necessary to extract data from OpenEase into CRAM.
;;; NOTE: Just the data extraction is here. Its manipulation and adjustments are in the openase-to-bullet.lisp file
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



(defun make-poses (name &optional (poses-list *poses-list*))
  "Calls make-pose to convert the list of values from poses-list to a
cl-tf:transform, and then applies all the adjustments needed to convert
the pose from the OpenEase map into a proper pose for the bullet world.
NAME: The name of an item in the *poses-list* of which the pose is needed.
The time stamp is included into the name, meaning possible parameters could be: 
OPTIONAL POSES-LIST: A different poses-list can be given if needed. Otherwise
*poses-list* will be used as default.
RETURNS: A cl-tf:transform of the given object at the given timestamp, With all 
the adjustments for the differences between the OpenEase and the bullet world
already beeing made."
  (apply-bullet-transform
   (quaternion-w-flip
     (make-pose (cut:var-value (intern name) poses-list)))))


(defun make-pose (pose)
  "Makes a proper cl-tf:transform with a 3d-vector for translation and a
quaternion for rotation.
POSE: A list of position and rotation values all in one list, as returned by OpenEase.
RETURNS: A cl-tf:transform consisting of a 3d-vector and a quaternion."
  (cl-tf:make-transform
   (apply #'cl-tf:make-3d-vector (subseq pose 0 3))
   (apply #'cl-tf:make-quaternion (subseq pose 3 7))))
