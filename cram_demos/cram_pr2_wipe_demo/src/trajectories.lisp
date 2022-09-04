(in-package :pr2-wipe)


(defun get-wiping-poses (grasp initial-pose surface &optional (angle (cram-math:degrees->radians 100)))
  (print "get-wiping-poses called")
  (defparameter *current-pose* (first initial-pose))
  (print surface)


  (let* ((return-poses '())
         (pose (first initial-pose))
         (pose-x (cl-transforms:x (cl-transforms:origin pose)))
         (pose-y (cl-transforms:y (cl-transforms:origin pose)))
         (pose-z (cl-transforms:z (cl-transforms:origin pose)))
         (pose-orientation (cl-transforms:orientation pose))
         (n (+ (floor
              (* (cl-transforms:y
                 (cl-bullet::bounding-box-dimensions
                  (btr::aabb  (btr:object btr:*current-bullet-world* (desig:desig-prop-value surface :name))))) 20)) 1))
         (x-dimensions-object (cl-transforms:x
                               (cl-bullet::bounding-box-dimensions
                                (btr::aabb  (btr:object btr:*current-bullet-world*  (desig:desig-prop-value surface :name))))))
         (y-dimensions-object (cl-transforms:y
                               (cl-bullet::bounding-box-dimensions
                                (btr::aabb  (btr:object btr:*current-bullet-world* (desig:desig-prop-value surface :name))))))
          (z-dimensions-object (cl-transforms:z
                               (cl-bullet::bounding-box-dimensions
                                (btr::aabb  (btr:object btr:*current-bullet-world* (desig:desig-prop-value surface :name)))))))

   
       (let* ((x-offset x-dimensions-object )
              (y-offset (/ y-dimensions-object n))
              (z-offset z-dimensions-object))

          (dotimes (x n)
            (if (equalp (mod x 2) 0)
                (progn
                  (setf *current-pose* (calculate-pose-down *current-pose* x-offset y-offset z-offset grasp))
                  (push *current-pose* return-poses)
                  )
                (progn
                  (setf *current-pose* (calculate-pose-up *current-pose* x-offset y-offset z-offset grasp))
                  (push *current-pose* return-poses)
                  ))))
        (reverse return-poses)
     ))


(defun calculate-pose-down (pose offset-x offset-y offset-z grasp)

  (case grasp
      (:countertop
       (cram-tf::copy-pose-stamped
        pose
        :origin
        (let ((vector (cl-transforms:origin pose)))
          (cl-transforms:copy-3d-vector
           vector
           :x  (- (cl-transforms:x vector) offset-x) 
           
           :y  (- (cl-transforms:y vector) offset-y)
          
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
           
           :y  (- (cl-transforms:y vector) offset-y)
          
           :z  (- (cl-transforms:z vector) offset-z)))
        :orientation
        (cl-transforms:orientation pose)))
  
  (:knife
       (cram-tf::copy-pose-stamped
        pose
        :origin
        (let ((vector (cl-transforms:origin pose)))
          (cl-transforms:copy-3d-vector
           vector
           :x  (- (cl-transforms:x vector) offset-x) 
           
           :y  (- (cl-transforms:y vector) offset-y)
          
           :z (cl-transforms:z vector)))
        :orientation
        (cl-transforms:orientation pose))

   )))



(defun calculate-pose-up (pose offset-x offset-y offset-z grasp)

  (case grasp
    (:countertop
     (cram-tf::copy-pose-stamped
      pose
      :origin
      (let ((vector (cl-transforms:origin pose)))
        (cl-transforms:copy-3d-vector
         vector
         :x  (+ (cl-transforms:x vector) offset-x) 
         
         :y  (- (cl-transforms:y vector) offset-y)
          
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
         
         :y  (- (cl-transforms:y vector) offset-y)
          
         :z  (+ (cl-transforms:z vector) offset-z)))
      :orientation
      (cl-transforms:orientation pose)))

   (:knife
       (cram-tf::copy-pose-stamped
        pose
        :origin
        (let ((vector (cl-transforms:origin pose)))
          (cl-transforms:copy-3d-vector
           vector
           :x  (+ (cl-transforms:x vector) offset-x) 
           
           :y  (- (cl-transforms:y vector) offset-y)
          
           :z (cl-transforms:z vector)))
        :orientation
        (cl-transforms:orientation pose))

    )))

  


  
  ;; (list
  ;;  (cl-tf:make-pose-stamped
  ;;   "map" 1
  ;;   (cl-tf:make-3d-vector
  ;;    (cl-pose-stamped:x
  ;;     (cl-pose-stamped:translation pose))
  ;;    (cl-transforms-stamped:y
  ;;     (cl-transforms-stamped:translation pose))
  ;;    (cl-transforms-stamped:z
  ;;     (cl-transforms-stamped:translation pose)))
  ;;   (cl-transforms-stamped:rotation pose)
  ;;    )))
                  
                  



  

                                
                                
          


            

(defmethod man-int:get-action-trajectory :heuristics 20 ((action-type (eql :wiping))
                                                         arm
                                                         grasp
                                                         location
                                                         objects-acted-on
                                                         &key )
  (print "Get action trajectory called")
  (let* ((object
           (car objects-acted-on))
         (object-name
           (desig:desig-prop-value object :name))
         (object-type
           (desig:desig-prop-value object :type))
         (bTo
           (man-int:get-object-transform object))  
         (bTb-offset
           (get-object-type-robot-frame-wipe-approach-transform-generic
             object object-type arm grasp))
         (oTg-std
           (cram-tf:copy-transform-stamped
            (man-int:get-object-type-to-gripper-transform
             object-type object-name arm grasp)
            :rotation (cl-tf:make-identity-rotation)))
         
         (approach-pose
           (cl-tf:copy-pose-stamped 
            (man-int:calculate-gripper-pose-in-base
              (cram-tf:apply-transform
               (cram-tf:copy-transform-stamped 
                bTb-offset
                :rotation (cl-tf:make-identity-rotation))
               bTo)
              arm oTg-std)
            :orientation 
            (cl-tf:rotation bTb-offset)))
         
         (wiping-poses
           (get-wiping-poses grasp (list approach-pose) object)))
    (print "in wiping poses")
    (mapcar (lambda (label poses-in-base)
              (man-int:make-traj-segment
               :label label
               :poses (mapcar 
                       (lambda (pose-in-base)
                         (let ((mTb (cram-tf:pose->transform-stamped
                                     cram-tf:*fixed-frame*
                                     cram-tf:*robot-base-frame*
                                     0.0
                                     (btr:pose (btr:get-robot-object))))
                               (bTg-std
                                 (cram-tf:pose-stamped->transform-stamped
                                  pose-in-base
                                  (cl-tf:child-frame-id bTo))))
                           (cl-tf:ensure-pose-stamped
                            (cram-tf:apply-transform mTb bTg-std))))
                       poses-in-base)))
            '(:approach
              :wiping)
            `((,approach-pose)
              ,wiping-poses))))
