(in-package :kvr)

(defun move-object (transform obj)
  "Moves an object to the desired transform.
TRANSFORM: The transform describing the position to which an object should be moved.
OBJ: The object which is supposed to be moved.
usage example: (move-object (pose-lists-parser '|?PoseObjEnd|))"
  (let* ((pose (cl-transforms:transform->pose transform)))
    (prolog:prolog `(and (btr:bullet-world ?world)
                         (assert (btr:object-pose ?world ,obj ,pose))))))

(defun move-obj-with-offset (x-offset y-offset object-knowrob object-bullet)
  "Moves an object to a given x y offset from it's starting position. "
  (move-object
   (cl-transforms:make-transform
    (cl-transforms:make-3d-vector
     (+
      x-offset
      (cl-transforms:x
       (cl-transforms:translation
        (get-object-location-at-start-by-object-type object-knowrob))))
     (+
      y-offset
      (cl-transforms:y
       (cl-transforms:translation
        (get-object-location-at-start-by-object-type object-knowrob))))
     (cl-transforms:z
      (cl-transforms:translation
       (get-object-location-at-start-by-object-type object-knowrob))))
    (cl-transforms:rotation
     (get-object-location-at-start-by-object-type object-knowrob)))
   object-bullet))

(defun move-semantic-objects-to-start-pose ()
  (move-obj-with-offset
   *semantic-map-offset-x* *semantic-map-offset-y*
   "IkeaBowl" :edeka-red-bowl2)
  (move-obj-with-offset
   *semantic-map-offset-x* *semantic-map-offset-y*
   "KoellnMuesliKnusperHonigNuss" :koelln-muesli-knusper-honig-nuss2)
  (move-obj-with-offset
   *semantic-map-offset-x* *semantic-map-offset-y*
   "PlasticBlueFork" :fork-blue-plastic2)
  (move-obj-with-offset
   *semantic-map-offset-x* *semantic-map-offset-y*
   "CupEcoOrange" :cup-eco-orange2)
  (move-obj-with-offset
   *semantic-map-offset-x* *semantic-map-offset-y*
   "MilramButtermilchErdbeere" :weide-milch-small2))

(defun move-object-to-starting-pose (object)
  "Moves the object and robot to their respective locations at the beginning of
the episode. . "
  (move-object (pick-pose object)
               (object-type-filter-bullet object)))

(defun move-urdf-objects-to-start-pose ()
  "Spawns all objects of the current Episode at their places without any offsets."
  (move-object-to-starting-pose 'muesli)
  (move-object-to-starting-pose 'milk)
  (move-object-to-starting-pose 'cup)
  (move-object-to-starting-pose 'bowl)
  (move-object-to-starting-pose 'fork))

(defun move-object-to-placing-pose (object)
  "Moves the object and robot to their respective locations at the beginning of
the episode. . "
  (move-object (place-pose object)
               (object-type-filter-bullet object)))

(defun move-away-axes ()
  (let* ((transform (cl-transforms:make-transform
                     (cl-transforms:make-3d-vector 3 3 3)
                     (cl-transforms:make-identity-rotation))))
    (move-object transform :axes)
    (move-object transform :axes2)
    (move-object transform :axes3)))

(defun test-placement ()
  (move-object
   (cl-tf:make-transform
    (cl-tf:make-3d-vector
     1.35 0.6 0.915)
    (cl-tf:make-quaternion
     -3.180422326961139d-17 7.275958169294938d-10 1.0d0 -4.371138828673793d-8))
   :cup-eco-orange))

#+these-are-not-used-anymore
(
 (defun parse-str (str)
   "parses the output from knowrob to a proper string which prolog can use."
   (concatenate 'string "'"  (remove #\' (string str)) "'"))

 (defun transform-to-pose-stamped (transform)
   "Takes a transform and returnes a pose stamped without any changes."
   (cl-transforms-stamped:make-pose-stamped
    "map"
    0.0
    (cl-transforms:translation transform)
    (cl-transforms:rotation transform)))

 (defun place-offset-transform ()
 "Creates a transform which describes and offset for placing an object. "
  (let ()
      (cl-tf:make-transform
       (cl-tf:make-3d-vector 0.0 0.2 0.0)
       (cl-tf:make-identity-rotation))))

 )
