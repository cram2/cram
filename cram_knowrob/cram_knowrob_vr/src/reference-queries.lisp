(in-package :kvr)
;; NOTE old queries which extract the data from clean table and set table data
;; prerecorded data I started out with from Andrei.
;; These can be removed, or kept for further reference.
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
