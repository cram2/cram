(in-package :kvr)
;; TODO maybe remove this? They are not really necessary anymore?
;; --- queries important for initialization ---
(defun register-ros-package (package-name)
  (prolog-simple-1
   (concatenate 'string  "register_ros_package(" package-name ")") :mode 1))

(defun u-load-episodes (episodes-path)
  (prolog-simple-1
   (concatenate 'string "u_load_episodes('" episodes-path "')") :mode 1))

(defun owl-parse (semantic-map-path)
  (prolog-simple-1
   (concatenate 'string "owl_parse('" semantic-map-path "')") :mode 1))

(defun connect-to-db (db-name)
  (prolog-simple-1
   (concatenate 'string "connect_to_db('" db-name "')") :mode 1))

(defun sem-map-inst (map-inst &optional (stop "."))
  (prolog-simple-1
   (concatenate 'string "sem_map_inst(" map-inst ")" stop) :mode 1))

(defun marker-update (obj &optional)
  (prolog-simple-1
   (concatenate 'string "marker_update(" obj ")"):mode 1))

(defun object (instance)
  (prolog-simple-1
   (concatenate 'string "object(" instance ")") :mode 1))

(defun prolog-cut ()
  (prolog-simple-1
   (concatenate 'string "!") :mode 1))

;; this one is used for sure
(defun map-marker-init ()
  (prolog-simple "sem_map_inst(MapInst),!,marker_update(object(MapInst))." ))


;;--- other queries ---

(defun u-occurs (ep-inst event-inst start end)
  (prolog-simple-1
   (concatenate 'string "u_occurs(" ep-inst ", " event-inst ", " start "," end ")") :mode 1))

(defun event-type (event-inst event-type)
  (prolog-simple-1
   (concatenate 'string "event_type(" event-inst ", " event-type ")") :mode 1))

(defun rdf-has (event-inst obj-acted-on obj-acted-on-inst)
  (prolog-simple-1
   (concatenate 'string "rdf_has(" event-inst ", " obj-acted-on ", " obj-acted-on-inst ")") :mode 1))

(defun u-marker-remove-all ()
  (prolog-simple-1 "u_marker_remove_all"))

(defun performed-by (event-inst hand-inst)
  (prolog-simple-1
   (concatenate 'string "performed_by(" event-inst ", " hand-inst ")" ) :mode 1))

(defun iri-xml-namespace (obj obj-short)
  (prolog-simple-1
   (concatenate 'string "iri_xml_namespace(" obj ", _, " obj-short ")") :mode 1))

(defun obj-type (obj-inst obj-type)
  (prolog-simple-1
   (concatenate 'string "obj_type(" obj-inst ", " obj-type ")") :mode 1))

(defun atom-concat (a1 a2 a3)
  (prolog-simple-1
   (concatenate 'string "atom_concat(" a1 ", " a2 ", " a3 ")") :mode 1))

(defun view-bone-meshes (ep-inst obj-inst-short-name start model-path)
  (prolog-simple-1
   (concatenate 'string "view_bones_meshes(" ep-inst ", " obj-inst-short-name ", " start ", " model-path ")" ) :mode 1))

(defun actor-pose (ep-inst obj-short-name start pose)
  (prolog-simple-1
   (concatenate 'string "actor_pose(" ep-inst ", " obj-short-name ", " start ", " pose ")") :mode 1))

(defun u-split-pose (pose pos quat)
  (prolog-simple-1
   (concatenate 'string "u_split_pose(" pose ", " pos ", " quat ")") :mode 1))

(defun marker-pose (obj pose)
  (prolog-simple-1
   (concatenate 'string "marker_pose(" obj ", " pose ")") :mode 1))

(defun pose (pos quat)
  (prolog-simple-1
   (concatenate 'string "pose(" pos ", " quat ")") :mode 1))

(defun view-actor-traj (ep-inst obj-short-name start end point color size length)
  (prolog-simple-1
   (concatenate 'string "view_actor_traj(" ep-inst ", " obj-short-name ", " start ", " end ", " point ", " color  ", " size ", " length ")") :mode 1))

(defun object-pose-at-time (obj-inst start pose)
  (prolog-simple-1
   (concatenate 'string "oject_pose_at_time(" obj-inst ", " start ", " pose ")") :mode 1))

(defun findall (event-inst obj event-list)
  (prolog-simple-1
   (concatenate 'string "findall(" event-inst ", " obj ", " event-list ")") :mode 1))

(defun ep-inst (episode-inst)
  (string
   (cdaar
    (prolog-simple-1
     (concatenate 'string "ep_inst(" episode-inst ")") :mode 1))))

;; NOTE old queries which extract the data from clean table and set table data
;; prerecorded data I started out with from Andrei
;; These might actually neeed to be removed


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

;; NOTE These are and were experimental functions, which could have been used,
;; but weren't. Maybe worth looking into them deeper?

(defun get-plan-subevents (plan)
  (cut:force-ll (prolog-simple (concatenate 'string "plan_subevents(knowrob:'" plan "', Sub)."))))

(defun get-all-plan-subevents ()
  (cut:force-ll (prolog-simple (concatenate 'string "plan_subevents(Super, Sub)."))))


(defun get-necessary-capabilities-for-action (action-name)
  (cut:force-ll (prolog-simple (concatenate 'string "required_cap_for_action(knowrob:'" action-name "', Cap)."))))


;; TODO ???
(defstruct object name type pose)
