;;; Contains useful functions which help to find out the offset between the
;;; OpenEase (VR) and the bullet world semantic map.
;;; The scenario is, that an episode is recorded,
;;; in which the human places cereal objects on the edges
;;; of the kitchen island table, and milk objects on the edges of the sink
;;; table, so that one can differentiate, between the two tables, and can see
;;; how offset they are in the bullet world map. Then the transform can be
;;; adjusted and the objects can be moved, until they are on the edges of the
;;; tables in the bullet world as well.


;;; NOTE currently used map-offset transform between VR and OE:
;;; 3d-vector -2.65 -0.7 0.0 rotation around z axis value: pi


(in-package :kvr)
;; TODO: this doesn't quite work yet?? <<< check if it does
(defun get-all-food-drink-poses ()
  "Extracts all the positions of the cereals and milks and spawns the according objects. "
  (let* (poses-list)
    ; get a pose and object
    (setq poses-list
          (prolog-simple "ep_inst(EpInst),
                        u_occurs(EpInst, EventInst, Start, End),
                        owl_individual_of(ObjInst, knowrob:'FoodOrDrink'),
                        iri_xml_namespace(ObjInst, _, ObjShortName),
                        actor_pose(EpInst, ObjShortName, Start, PoseObj),
                        obj_type(ObjInst, ObjType)."))
    ; check of object is of type milk or something
    (if (some #'identity
              (mapcar
               #'(lambda ( str )
                   (search str
                           (string-downcase
                            (cut:var-value
                             (intern "?ObjShortName")
                             (cut:lazy-car poses-list)))))
                 ; list of items against which it's being checked 
               '("milch" "cereal" "blub")))
        (progn (list (cut:var-value
                      (intern "?ObjShortName")
                      (cut:lazy-car poses-list))
                     (make-poses "?PoseObj" (cut:lazy-car poses-list)))))))
