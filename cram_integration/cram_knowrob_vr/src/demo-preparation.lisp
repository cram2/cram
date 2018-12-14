(in-package :kvr)

(defun demo-spawn-all-obj-in-place ()
  "Spawns all objects of the current Episode at their places without any offsets."
  (move-object-to-starting-pose 'muesli)
  (move-object-to-starting-pose 'milk)
  (move-object-to-starting-pose 'cup)
  (move-object-to-starting-pose 'bowl)
  (move-object-to-starting-pose 'fork))
