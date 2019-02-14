(in-package :kvr)

(defun demo-all-pick-place ()
  "Picks and places all objects of an episode one by one. Meaning the robot will always hold on to just one object and finish placing it before going back to picking up another one. "
  (move-urdf-objects-to-start-pose)

  (execute-pick-and-place 'muesli)
  (execute-pick-and-place 'milk)
  (execute-pick-and-place 'cup)
  (execute-pick-and-place 'bowl)
  (execute-pick-and-place 'fork))


(defun demo-all-obj ()
  ;; TODO This needs to be adapted to the newest changes!!!
  "For the entire episode, first place the object at the location where it was
for the robot to pick up, and then pick it up and place it. "

  ;; muesli
  (move-object-to-starting-pose 'koelln-muesli-knusper-honig-nuss)
  (execute-pick-and-place 'muesli)

  ;; milk
  (move-object-to-starting-pose 'weide-milch-small)
  (execute-pick-and-place 'milk)

  ;; cup
  (move-object-to-starting-pose 'cup-eco-orange)
  (execute-pick-and-place 'cup)

  ;; bowl
  (move-object-to-starting-pose 'edeka-red-bowl)
  (execute-pick-and-place 'bowl)

  ;; fork
  (move-object-to-starting-pose 'fork-blue-plastic)
  (execute-pick-and-place 'fork))
