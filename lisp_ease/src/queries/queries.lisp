(in-package :le)

;;ct: clean table, st: set table
(defun ct-grasp-something-poses ()
  (print "Hand, head and object pose at beginning of grasp.")
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



(defun st-grasp-spmething-pose ()
  (prolog-simple "
   ep_inst(EpInst),
   u_occurs(EpInst, EventInst, Start, End),
   event_type(EventInst, knowrob:'GraspingSomething'), 
   rdf_has(EventInst, knowrob:'objectActedOn', ObjActedOnInst),
   u_marker_remove_all,
   performed_by(EventInst, HandInst),
   iri_xml_namespace(HandInst,_, HandInstShortName),
   obj_type(HandInst, HandType),
   iri_xml_namespace(HandType, _, HandTypeName),
   atom_concat('package://sim/unreal/', HandTypeName, _TempPath), atom_concat(_TempPath, '/', HandModelPath),
   view_bones_meshes(EpInst, HandInstShortName, Start, HandModelPath),
   iri_xml_namespace(ObjActedOnInst, _, ObjShortName),
   actor_pose(EpInst, ObjShortName, Start, Pose),
   u_split_pose(Pose, Pos, Quat),
   marker_pose(object(ObjActedOnInst), pose(Pos,Quat)),
   obj_type(CameraInst, knowrob:'CharacterCamera'),
   iri_xml_namespace(CameraInst, _, CameraShortName),
   actor_pose(EpInst, CameraShortName, Start, PoseC),
   u_split_pose(PoseC, PosC, QuatC),
   marker_pose(object(CameraInst), pose(PosC,QuatC))."))


(defun ct-release-object-with-trajectory ()
  (prolog-simple "ep_inst(EpInst),
   u_occurs(EpInst, EventInst, Start, End),
   event_type(EventInst, knowrob:'GraspingSomething'), 
   rdf_has(EventInst, knowrob:'objectActedOn', ObjActedOnInst),
   u_marker_remove_all,
   performed_by(EventInst, HandInst),
   iri_xml_namespace(HandInst,_, HandInstShortName),
   obj_type(HandInst, HandType),
   iri_xml_namespace(HandType, _, HandTypeName),
   atom_concat('package://sim/unreal/', HandTypeName, _TempPath), atom_concat(_TempPath, '/', HandModelPath),
   view_bones_meshes(EpInst, HandInstShortName, End, HandModelPath),
   iri_xml_namespace(ObjActedOnInst, _, ObjShortName),
   view_actor_traj(EpInst, ObjShortName, Start, End, 'point', 'green', 0.03, 0.01),
   actor_pose(EpInst, ObjShortName, End, Pose),
   u_split_pose(Pose, Pos, Quat),
   marker_pose(object(ObjActedOnInst), pose(Pos,Quat)),
   obj_type(CameraInst, knowrob:'CharacterCamera'),
   iri_xml_namespace(CameraInst, _, CameraShortName),
   actor_pose(EpInst, CameraShortName, End, PoseC),
   u_split_pose(PoseC, PosC, QuatC),
   marker_pose(object(CameraInst), pose(PosC,QuatC))."))


(defun st-release-object-with-trajectory ()
  (prolog-simple "ep_inst(EpInst),
   u_occurs(EpInst, EventInst, Start, End),
   event_type(EventInst, knowrob:'GraspingSomething'), 
   rdf_has(EventInst, knowrob:'objectActedOn', ObjActedOnInst),
   u_marker_remove_all,
   performed_by(EventInst, HandInst),
   iri_xml_namespace(HandInst,_, HandInstShortName),
   obj_type(HandInst, HandType),
   iri_xml_namespace(HandType, _, HandTypeName),
   atom_concat('package://sim/unreal/', HandTypeName, _TempPath), atom_concat(_TempPath, '/', HandModelPath),
   view_bones_meshes(EpInst, HandInstShortName, End, HandModelPath),
   iri_xml_namespace(ObjActedOnInst, _, ObjShortName),
   view_actor_traj(EpInst, ObjShortName, Start, End, 'point', 'green', 0.03, 0.01),
   actor_pose(EpInst, ObjShortName, End, Pose),
   u_split_pose(Pose, Pos, Quat),
   marker_pose(object(ObjActedOnInst), pose(Pos,Quat)),
   obj_type(CameraInst, knowrob:'CharacterCamera'),
   iri_xml_namespace(CameraInst, _, CameraShortName),
   actor_pose(EpInst, CameraShortName, End, PoseC),
   u_split_pose(PoseC, PosC, QuatC),
   marker_pose(object(CameraInst), pose(PosC,QuatC))."))


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

(defun get-all-events-in-episode-old ()
  (prolog-simple "ep_inst(EpInst), findall(EventInst, u_occurs(EpInst, EventInst, Start, End), EventList)."))
;;---------------------------------------------



(defun event-type-old (EventInst EventClassName)
  (prolog-simple (concatenate 'string "event_type("
            EventInst ", " "knowrob:'" EventClassName "')")))

;; ObjectActedOnName and EventInst swapped places. maybe important?
(defun rdf-has-old (EventInst ObjActedOnName ObjActedOnInst)
 (prolog-simple (concatenate 'string "rdf_has(" EventInst ", " ObjActedOnName ", " ObjActedOnInst ")")))


(defun iri-xml-namespace-old (A B)
  (prolog-simple (concatenate 'string "u_occurs(EpInst, EventInst, Start, End), rdf_has(EventInst, knowrob:'objectActedOn', ObjActedOnInst)," "iri_xml_namespace("
            A ", _, " B ")")))

(defun get-object-type-old (ObjInst ObjType)
  (prolog-simple (concatenate 'string "obj_type(" ObjInst ", " ObjType ")" )))


(defun test ()
  (prolog-simple "ep_inst(EpInst),
    u_occurs(EpInst, EventInst, Start, End),
    event_type(EventInst, knowrob:'GraspingSomething'),
    rdf_has(EventInst, knowrob:'objectActedOn',ObjActedOnInst),
    performed_by(EventInst, HandInst),
    iri_xml_namespace(HandInst,_, HandInstShortName),
    obj_type(HandInst, HandType),
    iri_xml_namespace(HandType, _, HandTypeName),
    iri_xml_namespace(ObjActedOnInst, _, ObjShortName),
    object_pose_at_time(ObjActedOnInst, Start, PoseAtStart),
    actor_pose(EpInst, HandInstShortName, Start, PoseHandStart),
    actor_pose(EpInst, HandInstShortName, End, PoseHandEnd).")
 )

(defun test2 ()
  (prolog-simple "ep_inst(EpInst),
    u_occurs(EpInst, EventInst, Start, End),
    event_type(EventInst, knowrob:'GraspingSomething'),
    rdf_has(EventInst, knowrob:'objectActedOn',ObjActedOnInst),
    performed_by(EventInst, EffectorInst),
    iri_xml_namespace(ObjActedOnInst, _, ObjShortName),
    actor_pose(EpInst, ObjShortName, End, Pose)."))

(defstruct object name type pose)


(defun get-event-old ()
  (ep-inst "EpInst")
  (u-occurs "EpInst" "EventInst" "Start" "End"))

;; but why doesn't it work the other way? T_T
(defun get-event-by-type-old (type)
  (cut:lazy-append
   (prolog-simple (concatenate 'string "ep_inst(EpInst),
    u_occurs(EpInst, EventInst, Start, End),
    event_type(EventInst, knowrob:'" type "')."))
    (prolog-simple (concatenate 'string "ep_inst(EpInst),
    u_occurs(EpInst, EventInst, Start, End),
    event_type(EventInst, knowrob_u:'" type "')."))))


(defun get-all-events-old (&optional (type NIL))
  (if type
      (cut:force-ll (get-event-by-type type)) 
      (cut:force-ll (get-event))))

(defun get-actor-effector-pose(EpInst EventInst EventClassName ObjActedOnInst ObjActedOnName ObjShortName EffectorInst EffectorType Start End Pose)
  (ep-inst EpInst)
  ;(u-occurs EpInst EventInst Start End)
  (event-type EventInst EventClassName)
  (rdf-has EventInst ObjActedOnName ObjActedOnInst)
  (performed-by EventInst EffectorInst)
  (get-object-type EffectorInst EffectorType)
  (iri-xml-namespace ObjActedOnInst ObjShortName)
  (prolog-simple (concatenate 'string "actor_pose(" EpInst ", " ObjShortName ", " Start ", " Pose ").")))


(defun get-plan-subevents (plan)
  (cut:force-ll (prolog-simple (concatenate 'string "plan_subevents(knowrob:'" plan "', Sub)."))))

(defun get-all-plan-subevents ()
  (cut:force-ll (prolog-simple (concatenate 'string "plan_subevents(Super, Sub)."))))


(defun get-necessary-capabilities-for-action (action-name)
  (cut:force-ll (prolog-simple (concatenate 'string "required_cap_for_action(knowrob:'" action-name "', Cap)."))))









