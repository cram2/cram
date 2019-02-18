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
(defun base-query (object-type)
  (concatenate 'string
               "owl_has(Obj, rdf:type, knowrob:'" object-type "'),
                rdf_has(EventInst, knowrob:'objectActedOn', Obj),
                rdf_has(EventInst, knowrob:'objectActedOn', ObjActedOnInst),
                u_occurs(EpInst, EventInst, Start, End)"))

(defun get-event-by-object-type (object-type)
  "returns the event which was performed on the given object."
 (car 
  (cut:lazy-mapcar
   (lambda (binding-set)
     (cut:var-value
      '|?EventInst| binding-set))
   (json-prolog:prolog-simple
    (concatenate 'string (base-query object-type) ".")
    :package :kvr))))

(defun get-object-location-at-start-by-object-type (object-type)
  "returns the pose of an object at the start of the event performed on it."
  (car
   (cut:lazy-mapcar
    (lambda (binding-set)
      (cram-tf:flat-list->transform
       (cut:var-value
        '|?PoseObjStart| binding-set)))
    (json-prolog:prolog-simple
     (concatenate 'string
                  (base-query object-type)",
                    iri_xml_namespace(ObjActedOnInst, _, ObjShortName),
                    actor_pose(EpInst, ObjShortName, Start, PoseObjStart).")
     :package :kvr))))

(defun get-object-location-at-end-by-object-type (object-type)
  "returns the pose of an object at the end of the event performed on it."
   (car
    (cut:lazy-mapcar
     (lambda (binding-set)     
       (cram-tf:flat-list->transform
        (cut:var-value
         '?pose binding-set)))
     (json-prolog:prolog-simple
      (concatenate 'string
                   "owl_has(Obj, rdf:type, knowrob:'" object-type "'),
                      rdf_has(EventInst, knowrob:'objectActedOn', Obj),
                      rdf_has(EventInst, knowrob:'objectActedOn', ObjActedOnInst),
                      u_occurs(EpInst, EventInst, Start, End),
                      iri_xml_namespace(ObjActedOnInst, _, ObjShortName),
                      actor_pose(EpInst, ObjShortName, End, POSE).")
      :package :kvr))))

(defun get-hand (object-type)
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
                           (base-query object-type) ",
                            performed_by(EventInst, HandInst),
                            iri_xml_namespace(HandInst,_, HandInstShortName),
                            obj_type(HandInst, HandType),
                            iri_xml_namespace(HandType, _, HandTypeName).")
              :package :kvr)))))
    (if (search "Left" (string hand))
        :left
        (if (search "Right" (string hand))
            :right
            NIL))))

(defun get-hand-location-at-start-by-object-type (object-type)
  "returns the pose of the hand at the start of the event performed by it."
  (car
   (cut:lazy-mapcar
    (lambda (binding-set)
      (cram-tf:flat-list->transform
       (cut:var-value
        '|?PoseHandStart| binding-set)))
    (json-prolog:prolog-simple
     (concatenate 'string
                  (base-query object-type) ",
                      performed_by(EventInst, HandInst),
                      iri_xml_namespace(HandInst,_, HandInstShortName),
                      obj_type(HandInst, HandType),
                      iri_xml_namespace(HandType, _, HandTypeName),
                      actor_pose(EpInst, HandInstShortName, Start, PoseHandStart).")
     :package :kvr))))

(defun get-hand-location-at-end-by-object-type (object-type)
  "returns the pose of the hand at the end of the event performed by it."
  (car
   (cut:lazy-mapcar
    (lambda (binding-set)
      (cram-tf:flat-list->transform
       (cut:var-value
        '|?PoseHandEnd| binding-set)))
    (json-prolog:prolog-simple
     (concatenate 'string
                  (base-query object-type) ",
                    performed_by(EventInst, HandInst),
                    iri_xml_namespace(HandInst,_, HandInstShortName),
                    obj_type(HandInst, HandType),
                    iri_xml_namespace(HandType, _, HandTypeName),
                    actor_pose(EpInst, HandInstShortName, End, PoseHandEnd).")
     :package :kvr))))

(defun get-camera-location-at-start-by-object-type (object-type)
  "returns the pose of the camera(head) at the start of the event."
  (car
   (cut:lazy-mapcar
    (lambda (binding-set)
      (cram-tf:flat-list->transform
       (cut:var-value
        '|?PoseCameraStart| binding-set)))
    (json-prolog:prolog-simple
     (concatenate 'string
                  (base-query object-type) ",
                    obj_type(CameraInst, knowrob:'CharacterCamera'),
                    iri_xml_namespace(CameraInst, _, CameraShortName),
                    actor_pose(EpInst, CameraShortName, Start, PoseCameraStart).")
     :package :kvr))))

(defun get-camera-location-at-end-by-object-type (object-type)
  "returns the pose of the camera(head) at the end of the event."
  (car
   (cut:lazy-mapcar
    (lambda (binding-set)
      (cram-tf:flat-list->transform
       (cut:var-value
        '|?PoseCameraEnd| binding-set)))
    (json-prolog:prolog-simple
     (concatenate 'string
                  (base-query object-type) ",
                    obj_type(CameraInst, knowrob:'CharacterCamera'),
                    iri_xml_namespace(CameraInst, _, CameraShortName),
                    actor_pose(EpInst, CameraShortName, End, PoseCameraEnd).")
     :package :kvr))))

(defun get-table-location ()
  (car
   (cut:lazy-mapcar
    (lambda (binding-set)
      (cram-tf:flat-list->transform
       (cut:var-value
        '|?PoseTable| binding-set)))
    (json-prolog:prolog-simple
     "ep_inst(EpInst),
                     u_occurs(EpInst, EventInst, Start, End),
                     obj_type(TableInst, knowrob:'IslandArea'),
                     iri_xml_namespace(TableInst, _, TableShortName),
                     actor_pose(EpInst, TableShortName, Start, PoseTable)."
     :package :kvr))))

(defun get-contact-surface-pick-name (object-type)
  "Returns the name of the object surface an object is picked up from."
  (car
   (cut:lazy-mapcar
    (lambda (binding-set)
      (cut:var-value
       '|?SurfaceShortName| binding-set))
    (json-prolog:prolog-simple
     (concatenate 'string
                  "ep_inst(EpInst),
                      owl_has(Obj, rdf:type, knowrob:'" object-type "'),
                      event_type(EventInst, knowrob_u:'TouchingSituation'),
                      rdf_has(EventInst, knowrob_u:'inContact', Obj),
                      rdf_has(EventInst, knowrob_u:'inContact', Surface),
                      iri_xml_namespace(Surface, _, SurfaceShortName).")
     :package :kvr))))

(defun get-contact-surface-pick-pose (object-type)
  "returns the pose of the surface an object is picked up from."
  (car
   (cut:lazy-mapcar
    (lambda (binding-set)
      (cram-tf:flat-list->transform
       (cut:var-value
        '|?PoseSurface| binding-set)))
    (json-prolog:prolog-simple
     (concatenate 'string
                  "ep_inst(EpInst),
                      owl_has(Obj, rdf:type, knowrob:'" object-type "'),
                      event_type(EventInst, knowrob_u:'TouchingSituation'),
                      rdf_has(EventInst, knowrob_u:'inContact', Obj),
                      rdf_has(EventInst, knowrob_u:'inContact', Surface),
                      iri_xml_namespace(Surface, _, SurfaceShortName),
                      u_occurs(EpInst, EventInst, Start, End),
                      actor_pose(EpInst, SurfaceShortName, Start, PoseSurface).")
     :package :kvr))))

(defun get-contact-surface-place-name (object-type)
  "returns the name of the object surface an object is picked up from."
  (car
   (cut:lazy-mapcar
    (lambda (binding-set)
      (cut:var-value
       '|?SurfaceShortName| binding-set))
    (json-prolog:prolog-simple
     (concatenate 'string
                  "ep_inst(EpInst),
                      owl_has(Obj, rdf:type, knowrob:'" object-type "'),
                      rdf_has(EventInst, knowrob:'objectActedOn', Obj),
                      rdf_has(EventInst, knowrob:'objectActedOn', ObjActedOnInst),
                      u_occurs(EpInst, EventInst, Start, End),
                      iri_xml_namespace(ObjActedOnInst, _, ObjShortName),
                      event_type(NewEvent, knowrob_u:'TouchingSituation'),
                      u_occurs(EpInst, NewEvent, End, EndNew),
                      rdf_has(NewEvent, knowrob_u:'inContact', PlaceSurface),
                      iri_xml_namespace(PlaceSurface, _, SurfaceShortName).")
     :package :kvr))))



(defun get-contact-surface-place-pose (object-type)
  "returns the pose of the surface an object is picked up from."
  (car
   (cut:lazy-mapcar
    (lambda (binding-set)
      (cram-tf:flat-list->transform
       (cut:var-value (intern "?PoseSurface") binding-set)))
    (json-prolog:prolog-simple
     (concatenate 'string
                  "ep_inst(EpInst),
                      owl_has(Obj, rdf:type, knowrob:'" object-type "'),
                      rdf_has(EventInst, knowrob:'objectActedOn', Obj),
                      rdf_has(EventInst, knowrob:'objectActedOn', ObjActedOnInst),
                      u_occurs(EpInst, EventInst, Start, End),
                      iri_xml_namespace(ObjActedOnInst, _, ObjShortName),
                      event_type(NewEvent, knowrob_u:'TouchingSituation'),
                      u_occurs(EpInst, NewEvent, End, EndNew),
                      rdf_has(NewEvent, knowrob_u:'inContact', PlaceSurface),
                      iri_xml_namespace(PlaceSurface, _, SurfaceShortName),
                      actor_pose(EpInst, SurfaceShortName, EndNew, PoseSurface).")))))
