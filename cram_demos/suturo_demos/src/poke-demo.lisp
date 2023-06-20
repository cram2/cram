
(in-package :su-demos)

;;@author Tim Rienits
(defun poke-demo ()
  "The poke-demo, where the HSR pokes a pringles can in the bulletworld simulation."
  
  (roslisp-utilities:startup-ros) ;; Start ROS first.
   (urdf-proj:with-simulated-robot ;; The macro for the simulation.
           (unwind-protect
                (progn
                  (terpri) ;; Line Break.
                  (format T "!PLAN WIRD GESTARTET!")
                  (terpri)
                  (spawn-pringles-on-table)
                  (sleep 3)
                  (move-in-front-of-pringles)
                  (sleep 3)
                  (pick-up-the-pringles)
                  (sleep 3)))))


(defun spawn-pringles-on-table ()
  "Spawn primitive cylinder as :pringles item."
    (btr:add-object btr:*current-bullet-world* :cylinder-item 'cylinder-1
                    '((1.6 -1.4 0.787) (0 0 1 1))
                    :mass 0.2 :size (cl-transforms:make-3d-vector 0.03 0.03 0.08)
                    :item-type :pringles))

(defun move-in-front-of-pringles ()
  "Move in front of les pringles."
  (let* ((vector  (cl-tf2::make-3d-vector 1.0d0 -1.4d0 0d0))
         (rotation (cl-tf2::make-quaternion 0.0 0.0 0.0 1.0)))

    ;; This function moves the HSR to the coordinates we just defined.
    (move-hsr (cl-tf2::make-pose-stamped "map" 0 vector rotation))
    ))

(defun pick-up-the-pringles ()
  "Detect and poke les pringles."

  ;; Detect the pringles can.
  (let*((?pringles-desig
                    (desig:an object
                              (type :pringles)))
       (?perceived-object-desig
                    (exe:perform (desig:an action
                                           (type detecting)
                                           (object ?pringles-desig)))))
    
    ;; Extract the Pose from the Object Designator we just got.
     (roslisp:with-fields 
                 ((?pose
                   (cram-designators::pose cram-designators:data))) 
                 ?perceived-object-desig

       ;;A little workaround: We first define a list we use the Designator on.
       ;; We then just call the poking Action Designator with our pose.
       (let ((?superpose (list ?pose)))
        (mapc
         (lambda (?temppose)
           (let ((?poses `(,?temppose)))
             (exe:perform
              (desig:an action
                  (type poking)
                  (left-poses ?poses)
                  (collision-mode :allow-all)))))
         ?superpose)))))
