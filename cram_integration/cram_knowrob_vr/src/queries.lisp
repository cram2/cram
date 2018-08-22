(in-package :kvr)

(defun object-type-filter (object-type)
  ;;do some filtering for exact object names
    (case object-type
      (muesli (setq object-type "KoellnMuesliKnusperHonigNuss"))
      (cup (setq object-type "CupEcoOrange"))
      (bowl (setq object-type "IkeaBowl"))
      (milk (setq object-type "MilramButtermilchErdbeere"))
      (fork (setq object-type "PlasticBlueFork"))
      (spoon (setq object-type "PlasticBlueSpoon"))
      (t (ros-warn nil "Unknown object type. Known types are: muesli, cup, bowl, milk, fork, spoon"))))


(defun get-event-on-knowrob-object-type (object-type)
  (prolog-simple 
   (concatenate 'string
                "owl_has(Obj, rdf:type, knowrob:'" object-type "'),
                 rdf_has(EventInst, knowrob:'objectActedOn', Obj),
                 rdf_has(EventInst, knowrob:'objectActedOn', ObjActedOn),
                 u_occurs(EpInst, EventInst, Start, End).")))

(defun get-all-episode-data-on-type (object-type)
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





















;; old queries
;;useful queries. need to be generalized

(defun get-episode-instance ()
  (prolog-simple "ep_inst(Ep_Inst)"))

;nop
(defun get-pose-of-actor ()
  (prolog-simple "actor_pose(EpInst, ObjShortName, Start, Pose)."))

(defun get-grasping-events ()
  (prolog-simple "event_type(EventInst, knowrob:'GraspSomething')."))

(defun get-obj-instance ()
  (prolog-simple "obj_type(CameraInst, knowrob:'CharacterCamera')."))


(defun get-hand-from-event ()
  (prolog-simple "performed_by(EventInst, HandType)."))

(defun get-object-acted-on ()
  (prolog-simple "rdf_has(EventInst, knowrob:'objectActedOn', ObjActedOnInst)."))

;; this might be unreal dependant!
(defun get-events-occured-in-episodes ()
  (prolog-simple "u_occurs(EpInst, EventInst, Start, End)."))

(defun split-pose ()
  (prolog-simple "u_split_pose(Pose, Pos, Quat)."))











