(in-package :kvr)

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











