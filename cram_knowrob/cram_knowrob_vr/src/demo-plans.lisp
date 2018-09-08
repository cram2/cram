(in-package :kvr)

(defun demo-all-pick-place ()
  "Picks and places all objects of an episode one by one. Meaning the robot will always hold on to just one object and finish placing it before going back to picking up another one. "

  (execute-pick-and-place :koelln-muesli-knusper-honig-nuss)
  (execute-pick-and-place :weide-milch-small)
  (execute-pick-and-place :cup-eco-orange)
  (execute-pick-and-place :edeka-red-bowl)
  (execute-pick-and-place :fork-blue-plastic))


(defun demo-all-obj ()
  "For the entire episode, first place the object at the location where it was
for the robot to pick up, and then pick it up and place it. "
  ;; muesli
  (move-object-to-starting-pose 'koelln-muesli-knusper-honig-nuss)
  (execute-pick-and-place :koelln-muesli-knusper-honig-nuss)
  
  ;; milk
  (move-object-to-starting-pose 'weide-milch-small)
  (execute-pick-and-place :weide-milch-small)
  
  ;; cup
  (move-object-to-starting-pose 'cup-eco-orange)
  (execute-pick-and-place :cup-eco-orange)

  ;; bowl
  (move-object-to-starting-pose 'edeka-red-bowl)
  (execute-pick-and-place :edeka-red-bowl)

  ;; fork
  (move-object-to-starting-pose 'fork-blue-plastic)
  (execute-pick-and-place :fork-blue-plastic))



(defun execution-adjustment-test (type)
  (pick-up-object (set-grasp-base-pose
                   (get-camera-location-at-start-by-object-type
                    (object-type-filter-prolog type)))
                  (set-grasp-look-pose
                   (get-object-location-at-start-by-object-type
                    (object-type-filter-prolog type)))
                  :cup-eco-orange
                  ))
