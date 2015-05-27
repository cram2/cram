(in-package :reas-inf)

(defun spawn-object (name type pose-stamped-msg color-msg bounding-box &key (world btr:*current-bullet-world*))
  "Spawns an object with the name `name', the type `type', the color `color-msg' (`std_msgs/ColorRGBA')
at `pose-stamped-msg' (`geometry_msgs/PoseStamped') in the bullet world `world.
If there is already an object `name' in `world', nothing happens."
  (out-info "spawn-object()")
  (if (object-exists name)
      (progn
        (out-error "Object ~a already exists!" name)
        nil)
      (let ((type-symbol (get-object-type type))
            (name-symbol (make-keyword name)))
        (out-info "type-symbol: ~a" type-symbol)
        (out-info "name-symbol: ~a" name-symbol)
        (let* ((pose-stamped (cl-tf2:from-msg pose-stamped-msg))
               (pose-stamped-fixed (pose-stamped->pose-stamped-fixed pose-stamped))
               (color (color-msg-to-list color-msg))
               (box (cl-bullet:make-bounding-box :center pose-stamped-fixed
                                                 :dimensions bounding-box)))
          (out-info "pose: ~a" pose-stamped-fixed)
          (spatial-relations-demo::spawn-object name-symbol
                                                type-symbol
                                                :pose pose-stamped-fixed
                                                :color color
                                                :world world)
          (approximate-object-to-bounding-box name box world))
        (object-exists name))))

(defun update-camera-pose (pose &optional (world btr:*current-bullet-world*))
  (out-debug "update-camera-pose()")
  (out-debug "pose: ~a" pose)
  (not (null
        (let* ((color '(0 1 0))
               (camera-name *camera*)
               (view-name *camera-view*)
               (radius 0.05)
               (view-r 0.049)
               (view-h 0.1)
               (view-pose (cl-tf:transform
                           (cl-tf:pose->transform pose)
                           (cl-tf:make-pose
                            (cl-tf:make-3d-vector 0 0 view-h)
                            (cl-tf:make-quaternion 0 0 0 1)))))
          (if (prolog `(object ,world ,camera-name))
              (progn
                (out-debug "Camera exists.")
                (prolog `(assert (object-pose ,world ,camera-name ,pose)))
                (prolog `(assert (object-pose ,world ,view-name ,view-pose))))
              (progn
                (out-debug "Camera does not exist.")
                (prolog `(assert (object ,world sphere ,camera-name ,pose :mass 0.0 :color ,color :radius ,radius)))
                (prolog `(assert (object ,world cone ,view-name ,view-pose :mass 0.0 :color ,color :radius ,view-r :height ,view-h)))))))))

(defun remove-camera (&optional (world btr:*current-bullet-world*))
  (out-debug "remove-camera()")
  (prolog `(retract (object ,world ,*camera*)))
  (prolog `(retract (object ,world ,*camera-view*))))

(defun move-robot (pose &key (world btr:*current-bullet-world*) (robot-name 'cram-pr2-knowledge::pr2))
  `(assert (object-pose ,world ,robot-name ,pose)))

(defun move-object (name pose-stamped-msg bounding-box &key (world btr:*current-bullet-world*))
  "Moves the object with the name `name' to `pose-stamped-msg' (`geometry_msgs/PoseStamped')
in the bullet world `world.
If there is no object `name' in `world', nothing happens."
  (out-debug "move-object()")
  (let* ((pose-stamped (cl-tf2:from-msg pose-stamped-msg))
         (pose-stamped-fixed (pose-stamped->pose-stamped-fixed pose-stamped)))
    (move-object-by-fixed-pose name pose-stamped-fixed :world world :bounding-box bounding-box)))

(defun move-object-by-fixed-pose (name pose &key (world btr:*current-bullet-world*) bounding-box)
  (out-info "move-object-by-fixed-pose")
  (if (not (object-exists name world))
      (progn
        (out-error "Trying to move non existent object ~a!" name)
        nil)
      (let* ((name-symbol (make-keyword name))
             (box (cl-bullet:make-bounding-box :center pose
                                               :dimensions bounding-box))
             (new-pose (if bounding-box
                           (get-approximated-bounding-box-to-bounding-box-pose
                            (get-object-bounding-box name)
                            box)
                           pose)))
        (unless new-pose
          (setf new-pose pose))
        (out-info "pose: ~a" new-pose)
        (prolog `(assert (object-pose ,world ,name-symbol ,new-pose)))
        (let ((moved-pose (get-object-pose name :world world)))
          (out-info "moved-pose: ~a" moved-pose)
          (poses-equal-p new-pose moved-pose 0.01 10.00)))))

(defun remove-obj (name &optional (world btr:*current-bullet-world*))
  "Removes the object `name' from the bullet world `world'."
  (out-info "remove-obj()")
  (if (not (object-exists name world))
      (progn
        (out-error "Trying to remove non existent object ~a" name)
        nil)
      (let ((name-symbol (make-keyword name)))
        (prolog `(retract (object ,world ,name-symbol)))
        (not (object-exists name world)))))

(defun remove-all-objects (&optional (world btr:*current-bullet-world*))
  "Removes all objects from the bullet world `world'."
  (out-info "Removing all objects.")
  (prolog `(and (household-object-type ,world ?obj ?type) (retract (object ,world ?obj)) (cram-reasoning:fail)))
  (not (exists-any-object)))

(defun simulate-world (duration &key (world btr:*current-bullet-world*) copy)
  "Simulates the bullet world `world' for duration `duration'.
If `copy' is `t', the simulation is being executed on a copy of `world'.
Returns the simulated world."
  (out-info "Simulating world ~a. duration: ~a, copy: ~a" world duration copy)
  (let ((use-world (if copy
                       (copy-world world)
                       world)))
    (prolog `(simulate ,use-world ,duration))
    (out-info "Returning world ~a" use-world)
    use-world))

(defun copy-world (&optional (world btr:*current-bullet-world*))
  "Returns a copy of the bullet world `world'."
  (out-info "Copying world.")
  (cdr (car (car (prolog `(copied-world ,world ?world))))))
