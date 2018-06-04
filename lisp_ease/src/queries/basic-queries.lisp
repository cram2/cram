(in-package :le)

;; --- queries important for initialization ---
(defun register-ros-package (package-name)
  (prolog-simple-1 (concatenate 'string  "register_ros_package(" package-name ")") :mode 1))

(defun u-load-episodes (episodes-path)
  (prolog-simple-1 (concatenate 'string "u_load_episodes('" episodes-path "')") :mode 1))

(defun owl-parse (semantic-map-path)
  (prolog-simple-1 (concatenate 'string "owl_parse('" semantic-map-path "')") :mode 1))

(defun connect-to-db (db-name)
  (prolog-simple-1 (concatenate 'string "connect_to_db('" db-name "')") :mode 1))

(defun sem-map-inst (map-inst &optional (stop "."))
  (prolog-simple-1 (concatenate 'string "sem_map_inst(" map-inst ")" stop) :mode 1))

(defun marker-update (obj &optional)
  (prolog-simple-1 (concatenate 'string "marker_update(" obj ")"):mode 1))

(defun object (instance)
  (prolog-simple-1 (concatenate 'string "object(" instance ")") :mode 1))

(defun prolog-cut ()
  (prolog-simple-1 (concatenate 'string "!") :mode 1))

;; this one is used for sure
(defun map-marker-init ()
  (prolog-simple "sem_map_inst(MapInst),!,marker_update(object(MapInst))." ))


;;--- other queries ---

(defun u-occurs (ep-inst event-inst start end)
  (prolog-simple-1 (concatenate 'string "u_occurs(" ep-inst ", " event-inst ", " start "," end ")") :mode 1))

(defun event-type (event-inst event-type)
  (prolog-simple-1 (concatenate 'string "event_type(" event-inst ", " event-type ")") :mode 1))

(defun rdf-has (event-inst obj-acted-on obj-acted-on-inst)
  (prolog-simple-1 (concatenate 'string "rdf_has(" event-inst ", " obj-acted-on ", " obj-acted-on-inst ")") :mode 1))

(defun u-marker-remove-all ()
  (prolog-simple-1 "u_marker_remove_all"))

(defun performed-by (event-inst hand-inst)
  (prolog-simple-1 (concatenate 'string "performed_by(" event-inst ", " hand-inst ")" ) :mode 1))

(defun iri-xml-namespace (obj obj-short)
  (prolog-simple-1 (concatenate 'string "iri_xml_namespace(" obj ", _, " obj-short ")") :mode 1))

(defun obj-type (obj-inst obj-type)
  (prolog-simple-1 (concatenate 'string "obj_type(" obj-inst ", " obj-type ")") :mode 1))

(defun atom-concat (a1 a2 a3)
  (prolog-simple-1 (concatenate 'string "atom_concat(" a1 ", " a2 ", " a3 ")") :mode 1))

(defun view-bone-meshes (ep-inst obj-inst-short-name start model-path)
  (prolog-simple-1 (concatenate 'string "view_bones_meshes(" ep-inst ", " obj-inst-short-name ", " start ", " model-path ")" ) :mode 1))

(defun actor-pose (ep-inst obj-short-name start pose)
  (prolog-simple-1 (concatenate 'string "actor_pose(" ep-inst ", " obj-short-name ", " start ", " pose ")") :mode 1))

(defun u-split-pose (pose pos quat)
  (prolog-simple-1 (concatenate 'string "u_split_pose(" pose ", " pos ", " quat ")") :mode 1))

(defun marker-pose (obj pose)
  (prolog-simple-1 (concatenate 'string "marker_pose(" obj ", " pose ")") :mode 1))

(defun pose (pos quat)
  (prolog-simple-1 (concatenate 'string "pose(" pos ", " quat ")") :mode 1))

(defun view-actor-traj (ep-inst obj-short-name start end point color size length)
  (prolog-simple-1 (concatenate 'string "view_actor_traj(" ep-inst ", " obj-short-name ", " start ", " end ", " point ", " color  ", " size ", " length ")") :mode 1))

(defun object-pose-at-time (obj-inst start pose)
  (prolog-simple-1 (concatenate 'string "oject_pose_at_time(" obj-inst ", " start ", " pose ")") :mode 1))

(defun findall (event-inst obj event-list)
  (prolog-simple-1 (concatenate 'string "findall(" event-inst ", " obj ", " event-list ")") :mode 1))

(defun ep-inst (episode-inst)
  (string (cdaar (prolog-simple-1 (concatenate 'string "ep_inst(" episode-inst ")") :mode 1))))
