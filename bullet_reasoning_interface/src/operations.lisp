(in-package :reas-inf)

(defun spawn-object (name type pose-stamped-msg color-msg &key (world btr:*current-bullet-world*))
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
               (color (color-msg-to-list color-msg)))
          (spatial-relations-demo::spawn-object name-symbol
                                                type-symbol
                                                :pose pose-stamped-fixed
                                                :color color
                                                :world world))
        (object-exists name))))

(defun move-object (name pose-stamped-msg &key (world btr:*current-bullet-world*))
  "Moves the object with the name `name' to `pose-stamped-msg' (`geometry_msgs/PoseStamped')
in the bullet world `world.
If there is no object `name' in `world', nothing happens."
  (if (not (object-exists name world))
      (progn
        (out-error "Trying to move non existent object ~a!" name)
        nil)
        (let* ((name-symbol (make-keyword name))
               (pose (cl-tf2:from-msg pose-stamped-msg))
               (pose-stamped (cl-tf2:from-msg pose-stamped-msg))
               (pose-stamped-fixed (pose-stamped->pose-stamped-fixed pose-stamped)))
          (prolog `(assert (object-pose ,world ,name-symbol ,pose-stamped-fixed)))
          (poses-equal-p pose (get-object-pose name :world world) 0.01 0.01))))

(defun remove-obj (name &optional (world btr:*current-bullet-world*))
  "Removes the object `name' from the bullet world `world'."
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
