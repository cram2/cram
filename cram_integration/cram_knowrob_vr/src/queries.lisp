(in-package :kvr)

;; NOTE move into utils?
(defun object-type-filter (object-type)
  "Maps the simple name of an object, e.g. cup to the one known in the database
for that object, e.g. CupEcoOrange."
  ;;do some filtering for exact object names
    (case object-type
      (muesli (setq object-type "KoellnMuesliKnusperHonigNuss"))
      (cup (setq object-type "CupEcoOrange"))
      (bowl (setq object-type "IkeaBowl"))
      (milk (setq object-type "MilramButtermilchErdbeere"))
      (fork (setq object-type "PlasticBlueFork"))
      (spoon (setq object-type "PlasticBlueSpoon"))
      (t (ros-warn nil "Unknown object type. Known types are: muesli, cup, bowl, milk, fork, spoon"))))


(defun get-event-by-object-type (object-type)
  "returns the event which was performed on the given object."
  (prolog-simple 
   (concatenate 'string
                "owl_has(Obj, rdf:type, knowrob:'" object-type "'),
                 rdf_has(EventInst, knowrob:'objectActedOn', Obj),
                 rdf_has(EventInst, knowrob:'objectActedOn', ObjActedOnInst),
                 u_occurs(EpInst, EventInst, Start, End).")))

(defun get-object-location-at-start-by-object-type (object-type)
  "returns the pose of an object at the start of the event performed on it."
  (prolog-simple 
   (concatenate 'string
                "owl_has(Obj, rdf:type, knowrob:'" object-type "'),
                 rdf_has(EventInst, knowrob:'objectActedOn', Obj),
                 rdf_has(EventInst, knowrob:'objectActedOn', ObjActedOnInst),
                 u_occurs(EpInst, EventInst, Start, End),
                 iri_xml_namespace(ObjActedOnInst, _, ObjShortName),
                 actor_pose(EpInst, ObjShortName, Start, PoseObjStart).")))

(defun get-object-location-at-end-by-object-type (object-type)
  "returns the pose of an object at the end of the event performed on it."
  (prolog-simple 
   (concatenate 'string
                "owl_has(Obj, rdf:type, knowrob:'" object-type "'),
                 rdf_has(EventInst, knowrob:'objectActedOn', Obj),
                 rdf_has(EventInst, knowrob:'objectActedOn', ObjActedOnInst),
                 u_occurs(EpInst, EventInst, Start, End),
                 iri_xml_namespace(ObjActedOnInst, _, ObjShortName),
                 actor_pose(EpInst, ObjShortName, End, PoseObjStart).")))

(defun get-hand-location-at-start-by-object-type (object-type)
  "returns the pose of the hand at the start of the event performed by it."
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
                 actor_pose(EpInst, HandInstShortName, Start, PoseHandStart).")))

(defun get-hand-location-at-end-by-object-type (object-type)
  "returns the pose of the hand at the end of the event performed by it."
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
                 actor_pose(EpInst, HandInstShortName, End, PoseHandEnd).")))


(defun get-camera-location-at-start-by-object-type (object-type)
  "returns the pose of the camera(head) at the start of the event."
  (prolog-simple 
   (concatenate 'string
                "owl_has(Obj, rdf:type, knowrob:'" object-type "'),
                 rdf_has(EventInst, knowrob:'objectActedOn', Obj),
                 rdf_has(EventInst, knowrob:'objectActedOn', ObjActedOnInst),
                 u_occurs(EpInst, EventInst, Start, End),
                 obj_type(CameraInst, knowrob:'CharacterCamera'),
                 iri_xml_namespace(CameraInst, _, CameraShortName),
                 actor_pose(EpInst, CameraShortName, Start, PoseCameraStart).")))

(defun get-camera-location-at-end-by-object-type (object-type)
  "returns the pose of the camera(head) at the end of the event."
  (prolog-simple 
   (concatenate 'string
                "owl_has(Obj, rdf:type, knowrob:'" object-type "'),
                 rdf_has(EventInst, knowrob:'objectActedOn', Obj),
                 rdf_has(EventInst, knowrob:'objectActedOn', ObjActedOnInst),
                 u_occurs(EpInst, EventInst, Start, End),
                 obj_type(CameraInst, knowrob:'CharacterCamera'),
                 iri_xml_namespace(CameraInst, _, CameraShortName),
                 actor_pose(EpInst, CameraShortName, End, PoseCameraEnd).")))



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

