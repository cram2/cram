;;; Mapping from the object of the urdf kitchen to the semantic map.
;;; The semantic map kitchen is the one in OpenEase where the data is from.
;;; The URDF kitchen is the one the robot interacts with in the bullet world.
;;; The semantic map has a lot more objects then the URDF since all the cuttlery
;;; the robot interacts with is not present in the semantic map but spawned
;;; additionally.
(in-package :kvr)

(defparameter *semantic-map-offset-x* 0.0
  "Used in spawning semantic map and offsetting its objects")
(defparameter *semantic-map-offset-y* -3.0
  "Used in spawning semantic map and offsetting its objects")

(defparameter  *semantic-to-urdf*
  '((|''IslandArea_nhwy''| . :|KITCHEN.kitchen_island|)
    (|''SinkArea_pY2k''| . :|KITCHEN.sink_area|))
  "semantic map . urdf")

(defun match-kitchens (name)
  (cdr (assoc name *semantic-to-urdf*)))

(defun get-all-objects-urdf ()
  (btr:rigid-body-names
   (btr:object btr:*current-bullet-world* :kitchen)))

(defun get-all-objects-semantic ()
  (btr:rigid-body-names
   (btr:object btr:*current-bullet-world* :semantic-map-kitchen)))


(defun object-type-filter-prolog (object-type)
  "Maps the simple name of an object, e.g. cup to the one known in the database
for that object, e.g. CupEcoOrange."
  (case object-type
    (muesli "KoellnMuesliKnusperHonigNuss")
    (cup "CupEcoOrange")
    (bowl "IkeaBowl")
    (milk "MilramButtermilchErdbeere")
    (fork "PlasticBlueFork")
    (spoon "PlasticBlueSpoon")
    (t (ros-warn nil "Unknown object type. Known types are: muesli, cup, bowl, milk, fork, spoon"))))

(defun object-type-filter-bullet (object-type)
  "Maps the simple name of an object, e.g. cup to the one known in the database
for that object, e.g. CupEcoOrange."
  (case object-type
    (muesli :koelln-muesli-knusper-honig-nuss)
    (cup :cup-eco-orange)
    (bowl :edeka-red-bowl)
    (milk :weide-milch-small)
    (fork :fork-blue-plastic)
    (spoon :spoon-blue-plastic)
    (t (ros-warn nil "Unknown object type. Known types are: muesli, cup, bowl, milk, fork, spoon"))))

(defun object-type-fixer (object-type)
  "Takes care of the few cases where the name of the object within the recorded
VR data and the object within the bullet world, are different."
  (case object-type
    (:weide-milch-small :milram-buttermilch-erdbeere)
    (:edeka-red-bowl :ikea-bowl)
    (:fork-blue-plastic :plastic-blue-fork)
    (:spoon-blue-plastic :plastic-blue-spoon)
    (t object-type)))
