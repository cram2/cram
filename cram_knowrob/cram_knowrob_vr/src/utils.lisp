(in-package :kvr)

(defun object-type-filter-prolog (object-type)
  "Maps the simple name of an object, e.g. cup to the one known in the database
for that object, e.g. CupEcoOrange."
  ;;do some filtering for exact object names
    (case object-type
      (muesli (setq object-type "KoellnMuesliKnusperHonigNuss"))
      (cup (setq object-type "CupEcoOrange"))
      (bowl (setq object-type "IkeaBowl"))
      (milk (setq object-type "MilramButtermilchErdbeere"))
      (fork (setq object-type "PlasticBlueFork"))
      (spoon (setq object-type "PlasticBlueSpoon"))
      (t (ros-warn nil "Unknown object type. Known types are: muesli, cup, bowl, milk, fork, spoon"))))

(defun object-type-filter-bullet(object-type)
  "Maps the simple name of an object, e.g. cup to the one known in the database
for that object, e.g. CupEcoOrange."
  ;;do some filtering for exact object names
    (case object-type
      (muesli (setq object-type :koelln-muesli-knusper-honig-nuss))
      (cup (setq object-type :cup-eco-orange))
      (bowl (setq object-type :edeka-red-bowl))
      (milk (setq object-type :weide-milch-small))
      (fork (setq object-type :fork-blue-plastic))
      (spoon (setq object-type :spoon-blue-plastic))
      (t (ros-warn nil "Unknown object type. Known types are: muesli, cup, bowl, milk, fork, spoon"))))

(defun object-type-fixer(object-type)
  "Takes care of the few cases where the name of the object within the recorded
VR data and the object within the bullet world, are different."
  (case object-type
    (:weide-milch-small (setq object-type :milram-buttermilch-erdbeere))
    (:edeka-red-bowl (setq object-type :ikea-bowl))
    (:fork-blue-plastic (setq object-type :plastic-blue-fork))
    (:spoon-blue-plastic (setq object-type :plastic-blue-spoon))
    (t object-type)))


(defun move-object-to-starting-pose (object)
  "Moves the object and robot to their respective locations at the beginning of
the episode. . "
  (move-object (pick-pose
                (object-type-filter-prolog object))
               (object-type-filter-bullet object)))

(defun move-object-to-placing-pose (object)
  "Moves the object and robot to their respective locations at the beginning of
the episode. . "
  (move-object (place-pose
                (object-type-filter-prolog object))
               (object-type-filter-bullet object)))
 
(defun place-offset-transform ()
 "Creates a transform which describes and offset for placing an object. "
  (let ()
      (cl-tf:make-transform
       (cl-tf:make-3d-vector 0.0 0.2 0.0)
       (cl-tf:make-identity-rotation))))


(defun parse-str (str)
  "parses the output from knowrob to a proper string which prolog can use."
  (concatenate 'string "'"  (remove #\' (string str)) "'"))



(defun transform-to-pose-stamped (transform)
  "Takes a transform and returnes a pose stamped without any changes."
  (cl-tf:make-pose-stamped
   "map"
   0.0
   (cl-tf:translation transform)
   (cl-tf:rotation transform)))


(defun move-away-axes()
  (let* ((transform (cl-tf:make-transform
                     (cl-tf:make-3d-vector 3 3 3)
                     (cl-tf:make-identity-rotation))))
    (move-object transform :axes2)
    (move-object transform :axes3)))


(defun test-placement()
  (move-object 
      (cl-tf:make-transform
       (cl-tf:make-3d-vector 1.35 0.6 0.915)
       
       (cl-tf:make-quaternion
        -3.180422326961139d-17
        7.275958169294938d-10
        1.0d0
        -4.371138828673793d-8))
      :cup-eco-orange))
