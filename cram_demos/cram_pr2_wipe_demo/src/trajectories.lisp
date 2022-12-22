(in-package :pr2-wipe)

;;Function that differentiates if a surface is vertical or horizontal if called with the grasp type :scrubbing. If called with any other keyword the function just returns the keyword.
(defun differentiate-surface-types (grasp surface) 
  (let* ((s (car surface))
    (name (desig:desig-prop-value s :name)))
    (if (equal grasp :scrubbing)
      (progn
        ;;If the z dimension of the object is larger than the x dimension return :vertical.
        (if (> (cl-transforms:z
              (cl-bullet::bounding-box-dimensions
               (btr::aabb
                (btr:object
                 btr:*current-bullet-world* name))))
               (cl-transforms:x
                (cl-bullet::bounding-box-dimensions
                 (btr::aabb
                  (btr:object
                   btr:*current-bullet-world* name))))) 
            (progn :vertical)
            (progn :horizontal)))
      (progn grasp))))

;;Calculates the wiping poses for the given surface.
(defun get-wiping-poses (grasp initial-pose surface)
  ;;Sets all parameters that are necessary for the surface especially N and the dimesnions of the surface.
  (let* ((current-pose (first initial-pose)))
    (let* ((return-poses '())
          (n (+ (floor
              (* (cl-transforms:y
                 (cl-bullet::bounding-box-dimensions
                  (btr::aabb
                   (btr:object btr:*current-bullet-world*
                               (desig:desig-prop-value
                                surface :name)))))
                 20))
                1))
           (x-dimensions-object (cl-transforms:x
                                 (cl-bullet::bounding-box-dimensions
                                  (btr::aabb
                                   (btr:object
                                    btr:*current-bullet-world*
                                               (desig:desig-prop-value surface :name))))))
           (y-dimensions-object (cl-transforms:y
                                 (cl-bullet::bounding-box-dimensions
                                  (btr::aabb
                                   (btr:object
                                    btr:*current-bullet-world*
                                               (desig:desig-prop-value surface :name))))))
           (z-dimensions-object (cl-transforms:z
                                 (cl-bullet::bounding-box-dimensions
                                  (btr::aabb
                                   (btr:object
                                    btr:*current-bullet-world*
                                               (desig:desig-prop-value surface :name)))))))

    
      ;;Assigns the x-, y- and z-distance that are necessary for the helper functions.
      (let* ((x-distance x-dimensions-object )
             (y-distance (/ y-dimensions-object n))
             (z-distance z-dimensions-object))

        ;;Calls the helper functions calculate-pose-down and calculate-pose-up alternatively but N times in total. Returns the function with a list of wiping poses.
        (dotimes (x n)
          (if (equalp (mod x 2) 0)
              (progn
                (setf current-pose
                      (calculate-pose-down current-pose x-distance y-distance z-distance grasp))
                (push current-pose return-poses)
                )
              (progn
                (setf current-pose
                      (calculate-pose-up current-pose x-distance y-distance z-distance grasp))
                (push current-pose return-poses)))))
      (reverse return-poses))))


;;Helper function that copies a pose and modifies it to return a pose that is below the copied one. Takes into account if a surface is horizontal or vertical.
(defun calculate-pose-down (pose x-distance y-distance z-distance grasp)
  (case grasp
      ((or :horizontal :spreading)
       (cram-tf::copy-pose-stamped
        pose
        :origin
        (let ((vector (cl-transforms:origin pose)))
          (cl-transforms:copy-3d-vector
           vector
           :x  (- (cl-transforms:x vector) x-distance) 
           
           :y  (- (cl-transforms:y vector) y-distance)
          
           :z  (cl-transforms:z vector)))
        :orientation
        (cl-transforms:orientation pose)))

      (:vertical   
       (cram-tf::copy-pose-stamped
        pose
        :origin
        (let ((vector (cl-transforms:origin pose)))
          (cl-transforms:copy-3d-vector
           vector
           :x  (cl-transforms:x vector)  
           
           :y  (- (cl-transforms:y vector) y-distance)
          
           :z  (- (cl-transforms:z vector) z-distance)))
        :orientation
        (cl-transforms:orientation pose)))))


;;Helper function that copies a pose and modifies it to return a pose that is above the copied one. Takes into account if a surface is horizontal or vertical.
(defun calculate-pose-up (pose x-distance y-distance z-distance grasp)
  (case grasp
    ((or :horizontal :spreading)
     (cram-tf::copy-pose-stamped
      pose
      :origin
      (let ((vector (cl-transforms:origin pose)))
        (cl-transforms:copy-3d-vector
         vector
         :x  (+ (cl-transforms:x vector) x-distance) 
         
         :y  (- (cl-transforms:y vector) y-distance)
          
         :z (cl-transforms:z vector)))
      :orientation
      (cl-transforms:orientation pose)))

    (:vertical
     (cram-tf::copy-pose-stamped
      pose
      :origin
      (let ((vector (cl-transforms:origin pose)))
        (cl-transforms:copy-3d-vector
         vector
         :x  (cl-transforms:x vector) 
         
         :y  (- (cl-transforms:y vector) y-distance)
          
         :z  (+ (cl-transforms:z vector) z-distance)))
      :orientation
      (cl-transforms:orientation pose)))))


;;Main function of trajectories.lisp that gets the initial pose and calls get-wiping-poses to get a list of wiping poses. The lists of poses are made into traj-segments.
(defmethod get-trajectory  ((action-type (eql :wiping))
                                                         arm
                                                         grasp
                                                         location
                                                         surface
                                                         &key)
  
  (let* ((surface
           (car surface))
         (object-name
           (desig:desig-prop-value surface :name))
         (object-type
           (desig:desig-prop-value surface :type))
         (bTo
           (man-int:get-object-transform surface))
          
         (bTb-offset
           (get-object-type-robot-frame-wipe-approach-transform-generic surface object-type arm grasp))
         
         (offset
            (man-int:get-object-type-to-gripper-transform
             object-type object-name arm grasp))
         
         (initial-pose
           (print (cl-tf:copy-pose-stamped 
            (print (man-int:calculate-gripper-pose-in-base
              (print (cram-tf:apply-transform
               (print (cram-tf:copy-transform-stamped 
                bTb-offset
                :rotation (cl-tf:make-identity-rotation)))
               bTo))
              arm offset))
            )))
         
         (wiping-poses (get-wiping-poses grasp (list initial-pose) surface)))

     
    (mapcar (lambda (label poses-in-base)
              (man-int:make-traj-segment
               :label label
               :poses (mapcar 
                       (lambda (pose-in-base) 
                         (let ((mTb (cram-tf:pose->transform-stamped
                                     "map"
                                     "base_footprint"
                                     0.0
                                     (btr:pose (btr:get-robot-object))))
                               (bTg
                                 (cram-tf:pose-stamped->transform-stamped
                                  pose-in-base
                                  (cl-tf:child-frame-id bTo))))
                           
                           (cl-tf:ensure-pose-stamped
                            (cram-tf:apply-transform mTb bTg))))
                       poses-in-base)))
            (list :initial
              :wiping)
            (list (list initial-pose)
                  wiping-poses))))
