(in-package :kvr)

(defun base-query (object-type)
  (concatenate 'string
                    "owl_has(Obj, rdf:type, knowrob:'" object-type "'),
                     rdf_has(EventInst, knowrob:'objectActedOn', Obj),
                     rdf_has(EventInst, knowrob:'objectActedOn', ObjActedOnInst),
                     u_occurs(EpInst, EventInst, Start, End)"))

(defun get-event-by-object-type (object-type)
  "returns the event which was performed on the given object."
  (cut:var-value
   (intern "?EventInst")
   (cut:lazy-car
    (prolog-simple 
     (concatenate 'string
                  (base-query object-type) ".")))))

(defun get-object-location-at-start-by-object-type (object-type)
  "returns the pose of an object at the start of the event performed on it."
  (make-pose
   (cut:var-value
    (intern "?PoseObjStart")
    (cut:lazy-car
     (prolog-simple 
      (concatenate 'string
                   (base-query object-type)",
                    iri_xml_namespace(ObjActedOnInst, _, ObjShortName),
                    actor_pose(EpInst, ObjShortName, Start, PoseObjStart)."))))))


(defun get-object-location-at-end-by-object-type (object-type)
  "returns the pose of an object at the end of the event performed on it."
  (make-pose
   (cut:var-value (intern "?PoseObjEnd")
                  (cut:lazy-car
                   (prolog-simple 
                    (concatenate 'string
                     "owl_has(Obj, rdf:type, knowrob:'" object-type "'),
                      rdf_has(EventInst, knowrob:'objectActedOn', Obj),
                      rdf_has(EventInst, knowrob:'objectActedOn', ObjActedOnInst),
                      u_occurs(EpInst, EventInst, Start, End),
                      iri_xml_namespace(ObjActedOnInst, _, ObjShortName),
                      actor_pose(EpInst, ObjShortName, End, PoseObjEnd)."))))))

;; returns the hand used in the curretnly loaded episode
(defun get-hand (object-type)
  "returns which hand was used to interact with the object"
  (let* ((hand
           (cut:var-value
            (intern "?HandTypeName")
            (cut:lazy-car
             (prolog-simple 
              (concatenate 'string
                           (base-query object-type) ",
                            performed_by(EventInst, HandInst),
                            iri_xml_namespace(HandInst,_, HandInstShortName),
                            obj_type(HandInst, HandType),
                            iri_xml_namespace(HandType, _, HandTypeName)."))))))
    (if (search "Left" (string hand))
        :left
        (if (search "Right" (string hand))
            :right
            NIL))))



(defun get-hand-location-at-start-by-object-type (object-type)
  "returns the pose of the hand at the start of the event performed by it."
  (make-pose
   (cut:var-value
    (intern "?PoseHandStart")
    (cut:lazy-car
     (prolog-simple 
      (concatenate 'string
                     (base-query object-type) ",
                      performed_by(EventInst, HandInst),
                      iri_xml_namespace(HandInst,_, HandInstShortName),
                      obj_type(HandInst, HandType),
                      iri_xml_namespace(HandType, _, HandTypeName),
                      actor_pose(EpInst, HandInstShortName, Start, PoseHandStart)."))))))

(defun get-hand-location-at-end-by-object-type (object-type)
  "returns the pose of the hand at the end of the event performed by it."
  (make-pose
   (cut:var-value
    (intern "?PoseHandEnd")
    (cut:lazy-car
     (prolog-simple 
      (concatenate 'string
                   (base-query object-type) ",
                    performed_by(EventInst, HandInst),
                    iri_xml_namespace(HandInst,_, HandInstShortName),
                    obj_type(HandInst, HandType),
                    iri_xml_namespace(HandType, _, HandTypeName),
                    actor_pose(EpInst, HandInstShortName, End, PoseHandEnd)."))))))


(defun get-camera-location-at-start-by-object-type (object-type)
  "returns the pose of the camera(head) at the start of the event."
  (make-pose
   (cut:var-value
    (intern "?PoseCameraStart")
    (cut:lazy-car
     (prolog-simple 
      (concatenate 'string
                   (base-query object-type) ",
                    obj_type(CameraInst, knowrob:'CharacterCamera'),
                    iri_xml_namespace(CameraInst, _, CameraShortName),
                    actor_pose(EpInst, CameraShortName, Start, PoseCameraStart)."))))))

(defun get-camera-location-at-end-by-object-type (object-type)
  "returns the pose of the camera(head) at the end of the event."
  (make-pose
   (cut:var-value
    (intern "?PoseCameraEnd")
    (cut:lazy-car
     (prolog-simple 
      (concatenate 'string
                   (base-query object-type) ",
                    obj_type(CameraInst, knowrob:'CharacterCamera'),
                    iri_xml_namespace(CameraInst, _, CameraShortName),
                    actor_pose(EpInst, CameraShortName, End, PoseCameraEnd)."))))))

(defun get-table-location ()
  (make-pose
   (cut:var-value
    (intern "?PoseTable")
    (cut:lazy-car
     (prolog-simple "ep_inst(EpInst),
                     u_occurs(EpInst, EventInst, Start, End),
                     obj_type(TableInst, knowrob:'IslandArea'),
                     iri_xml_namespace(TableInst, _, TableShortName),
                     actor_pose(EpInst, TableShortName, Start, PoseTable).")))))



(defun get-contact-surface-pick-name (object-type)
  "returns the name of the object surface an object is picked up from."
   (cut:var-value (intern "?SurfaceShortName")
                  (cut:lazy-car
                   (prolog-simple 
                    (concatenate 'string
                     "ep_inst(EpInst),
                      owl_has(Obj, rdf:type, knowrob:'" object-type "'),
                      event_type(EventInst, knowrob_u:'TouchingSituation'),
                      rdf_has(EventInst, knowrob_u:'inContact', Obj),
                      rdf_has(EventInst, knowrob_u:'inContact', Surface),
                      iri_xml_namespace(Surface, _, SurfaceShortName).")))))

(defun get-contact-surface-pick-pose (object-type)
  "returns the pose of the surface an object is picked up from."
  (make-pose
   (cut:var-value (intern "?PoseSurface")
                  (cut:lazy-car
                   (prolog-simple 
                    (concatenate 'string
                     "ep_inst(EpInst),
                      owl_has(Obj, rdf:type, knowrob:'" object-type "'),
                      event_type(EventInst, knowrob_u:'TouchingSituation'),
                      rdf_has(EventInst, knowrob_u:'inContact', Obj),
                      rdf_has(EventInst, knowrob_u:'inContact', Surface),
                      iri_xml_namespace(Surface, _, SurfaceShortName),
                      u_occurs(EpInst, EventInst, Start, End),
                      actor_pose(EpInst, SurfaceShortName, Start, PoseSurface)."))))))

(defun get-contact-surface-place-name (object-type)
  "returns the name of the object surface an object is picked up from."
   (cut:var-value (intern "?SurfaceShortName")
                  (cut:lazy-car
                   (prolog-simple 
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
                      iri_xml_namespace(PlaceSurface, _, SurfaceShortName).")))))



(defun get-contact-surface-place-pose (object-type)
  "returns the pose of the surface an object is picked up from."
  (make-pose
   (cut:var-value (intern "?PoseSurface")
                  (cut:lazy-car
                   (prolog-simple 
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
                      actor_pose(EpInst, SurfaceShortName, EndNew, PoseSurface)."))))))
