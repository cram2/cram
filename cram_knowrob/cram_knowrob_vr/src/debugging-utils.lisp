;;; Contains functions which are usefull for debugging any errors within the code

(in-package :kvr)

(defun reset-simulation ()
  "Resets the simulation and belief state. Re-spawns the objects at their
initial position."
  (cram-occasions-events:clear-belief)
  (init-items))
