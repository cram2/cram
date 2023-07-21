;;;
;;; Copyright (c) 2023, Tina Van <van@uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :cram-manipulation-interfaces)

(defmethod man-int:get-action-trajectory :heuristics 20 ((action-type (eql :mixing))
                                                         arm
                                                         grasp
                                                         location
                                                         objects-acted-on
                                                         &key
                                                           context
                                                           reso
                                                           rounds
                                                           tool-object-type
                                                           )
  
  (print "entered mixing")
  (let* ((object
           (car objects-acted-on))
         (object-name
           (desig:desig-prop-value object :name))
         (object-type
           (desig:desig-prop-value object :type))
         (bTo
           (man-int:get-object-transform object)         
           )

         ;; The first part of the btb-offset transform encodes the
         ;; translation difference between the gripper and the
         ;; object. gripper to goal approach position - this depends on object type height and tool object height whihc needs to calced. This depends mostly on the defined coordinate frame of the object and how objects should be rotated so the tool is correctly inserted.
         (bTb-offset
           (get-object-type-robot-frame-mix-approach-transform-destructuring
            object-type object-name tool-object-type))
         
                                        ;function plus stuff matrix of container rim to tool lenght from grip to end

                                        ;depending on object usually set on 12 o clock of the container opening
         (bTb-liftoffset
           (get-object-type-robot-frame-mix-retract-transform-destructuring
            object-type arm grasp tool-object-type))
         ;; Since the grippers orientation should not depend on the
         ;; orientation of the object it is omitted here.

         (oTg-std
           (cram-tf:copy-transform-stamped
            (man-int:get-object-type-to-gripper-transform tool-object-type object-name arm grasp)
            :translation
            (let* ((transform-translation (cl-transforms:translation  (man-int:get-object-type-to-gripper-transform tool-object-type object-name arm grasp) )))
              (cl-transforms:copy-3d-vector
               transform-translation
               :z (let ((x-transform-translation
                          (cl-transforms:x transform-translation))) 
                    (if (plusp x-transform-translation)
                        x-transform-translation (* x-transform-translation -1)
                        ))
               :x  (cl-transforms:y transform-translation)
               :y (cl-transforms:x transform-translation)))))

         (container-arm (if (eql arm :right) :left :right))
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
         
         (mix-poses (rounds-amount rounds
                                   (adjust-circle-poses-context approach-pose reso context object-type tool-object-type)))
         
         (start-mix-poses (rec-spiral-poses object-type approach-pose reso tool-object-type))
         
                                        ;spiral inwards
         (end-mix-poses (reverse-spiral-poses object-type approach-pose reso tool-object-type))
                                        ;retract
         (retract-pose
           (cl-tf:copy-pose-stamped 
            (man-int:calculate-gripper-pose-in-base
             (cram-tf:apply-transform
              (cram-tf:copy-transform-stamped 
               bTb-liftoffset
               :rotation (cl-tf:make-identity-rotation))
              bTo)
             arm oTg-std)
            :orientation 
            (cl-tf:rotation bTb-liftoffset)))
         
	 )
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
            '(
	      :approach
	      :start-mix
	      :mid-mix
              :end-mix
              :retract
	      )
	    `(
              (,approach-pose)
	      ,start-mix-poses
	      ,mix-poses
              ,end-mix-poses
              (,retract-pose)
	      ))))	     

(defgeneric get-object-type-robot-frame-mix-grip-retract-transform (object-type arm grasp)
  (:documentation "Returns a transform stamped")
  (:method (object-type arm grasp)
    (man-int::call-with-specific-type #'get-object-type-robot-frame-mix-grip-retract-transform
                                      object-type arm grasp)))

(defmethod get-object-type-robot-frame-mix-grip-retract-transform :around (object-type arm grasp)
  (destructuring-bind
      ((x y z) (ax ay az aw))
      (call-next-method)
    (cl-tf:transform->transform-stamped
     cram-tf:*robot-base-frame*
     cram-tf:*robot-base-frame*
     0.0
     (cl-tf:pose->transform
      (cl-transforms:make-pose
       (cl-transforms:make-3d-vector x y z)
       (cl-transforms:make-quaternion ax ay az aw))))))

(defgeneric get-object-type-robot-frame-mix-retract-transform (object-type arm grasp)
  (:documentation "Returns a transform stamped")
  (:method (object-type arm grasp)
    (man-int::call-with-specific-type #'get-object-type-robot-frame-mix-retract-transform
                                      object-type arm grasp)))



(defgeneric get-object-type-robot-frame-mix-grip-approach-transform (object-type arm grasp)
  (:documentation "Returns a transform stamped")
  (:method (object-type arm grasp)
    (man-int::call-with-specific-type #'get-object-type-robot-frame-mix-grip-approach-transform
                                      object-type arm grasp)))

(defmethod get-object-type-robot-frame-mix-grip-approach-transform :around (object-type arm grasp)
  (destructuring-bind
      ((x y z) (ax ay az aw))
      (call-next-method)
    (cl-tf:transform->transform-stamped
     cram-tf:*robot-base-frame*
     cram-tf:*robot-base-frame*
     0.0
     (cl-tf:pose->transform
      (cl-transforms:make-pose
       (cl-transforms:make-3d-vector x y z)
       (cl-transforms:make-quaternion ax ay az aw))))))

(defgeneric get-object-type-robot-frame-mix-approach-transform (object-type)
  (:documentation "Returns a transform stamped")
  (:method (object-type)
    (man-int::call-with-specific-type #'get-object-type-robot-frame-mix-approach-transform
                                      object-type)))

(defun get-object-type-robot-frame-mix-approach-transform-destructuring (object-type object-name tool-object-type)
  (let* ((container (get-object-type-robot-frame-mix-rim-bottom-transform object-type))
         (tool (get-object-type-robot-frame-mix-tool-transform tool-object-type))
         (height 0)
         (newpose '()))

      ;height - opening and bottom radius need to be wider than tool
    (if (or (>= (or (caddar tool)(cadar tool)) (cadar container)))
        (error "tool is too wide to fit into container")
        )
    (setf height (+ (caddar container)(caar tool)))
    (setf newpose (cons (append (list 0 0 height)) (last container)))
    
    (destructuring-bind
        ((x y z) (ax ay az aw))
        newpose
      (cl-tf:transform->transform-stamped
       cram-tf:*robot-base-frame*
       cram-tf:*robot-base-frame*
       0.0
       (cl-tf:pose->transform
        (cl-transforms:make-pose
         (cl-transforms:make-3d-vector x y z)
         (cl-transforms:make-quaternion ax ay az aw)))))))

(defun get-object-type-robot-frame-mix-retract-transform-destructuring (object-type arm grasp tool-object-type)
  (let* ((container (get-object-type-robot-frame-mix-rim-bottom-transform object-type))
         (tool (get-object-type-robot-frame-mix-tool-transform tool-object-type))
         (height 0)
         (newpose '()))

    (setf height (+ (caar container)(caar tool)))
    (setf newpose (cons (append (list 0 0 height)) (last container)))    
    (destructuring-bind
        ((x y z) (ax ay az aw))
        newpose
      (cl-tf:transform->transform-stamped
       cram-tf:*robot-base-frame*
       cram-tf:*robot-base-frame*
       0.0
       (cl-tf:pose->transform
        (cl-transforms:make-pose
         (cl-transforms:make-3d-vector x y z)
         (cl-transforms:make-quaternion ax ay az aw)))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                                        ;gets upstairs, drawing functions below
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun adjust-circle-poses-context (pose reso context object-type tool-object-type)
  (if (eq context :mix-circle)
      (adjust-circle-poses pose reso 1 object-type tool-object-type)
      (if (eq context :mix-eclipse)
          (adjust-circle-poses pose reso 0.03 object-type tool-object-type)
          (if (eq context :mix-orbit)
              (orbital-poses pose reso object-type tool-object-type )))))

;;TOPrim = T Bottomrim = nil ; which means that default will always take bottomrim
(defun calc-radius(container-object-type tool-object-type)
  (let* ((container 0)
         (toolw 0)
         (toold 0))
    
    (setf container (nth 1 (car (get-object-type-robot-frame-mix-rim-bottom-transform container-object-type))))   
    (setf toolw (nth 1 (car (get-object-type-robot-frame-mix-tool-transform tool-object-type))))
    (setf toold (nth 2 (car (get-object-type-robot-frame-mix-tool-transform tool-object-type))))
    (if (> toolw toold)
        (- container toolw)
        (- container toold))))

;;Toolhalvingtoggle is T when radius size halving is needed or nil if no radius sizing halving is needed.
(defun adjust-circle-poses(pose reso newerate container-object-type tool-object-type &rest toolhalvingtoggle)

  (let* (
         (erate 1);== circle;  stirring == (erate 0.03) elipsiness
         (angle 0)
         (x 1)
         (defaultreso 12)
         (rim (calc-radius container-object-type tool-object-type)) )

    (setf erate newerate)
    (if (not (not toolhalvingtoggle))
        (setf rim (/ rim 2)))

    (if (>= reso 2) (setf defaultreso reso))
    (setf angle (/(* 2  pi) defaultreso))

                                        ;defaultreso is the holder for whatever ?reso was decided on or real default reso- look above
    (loop while (<= x defaultreso)
          do (setf x   (+ x  1))
             
          collect  (change-v pose :x-offset (* erate (* rim (cos (* x angle))))
                                  :y-offset (* rim (sin (* x angle))))))) 

(defun rec-spiral-poses(object-type pose reso tool-object-type)
  (let
      ( (k 0.4) ;0.3 <-'spiralness'
        (defaultreso 12)
        (rim 0)
        (angle 0)  
        (x 1)
        )

    (setf rim (calc-radius object-type tool-object-type))
    (setf angle (/(* 2  pi) defaultreso))
    (loop while (<= x defaultreso)
	  do (setf x   (+ x  1))
          collect(change-v pose :x-offset (*(* (/ rim defaultreso)
                                               (exp (* k (* x angle))) (cos (* x angle))))
                                :y-offset (*(* (/ rim defaultreso)
                                               (exp (* k(* x  angle))) (sin (* x  angle))))))))

(defun reverse-spiral-poses(object-type pose reso tool-object-type)
  (let 
      ( (k 0.4) ;0.3 <-'spiralness'
        (defaultreso 12)
        (rim 0)
                                        ;for spiral only top rim needed.
        (angle 0)  
        (x 0)
        (start-pose nil) )
    
    (setf rim (calc-radius object-type tool-object-type))
    
    (setf angle (/(* 2  pi) defaultreso))
    (setf x defaultreso)
    (loop while (>= x 0)
	  do (setf x (- x 1))
          collect(change-v pose :x-offset (*(* (/ rim defaultreso)
                                               (exp (* k (* x  angle))) (cos (* x  angle))))
                                :y-offset (*(* (/ rim defaultreso)
                                               (exp (* k(* x   angle))) (sin (* x   angle))))))))

                                        ;fixed reso value 9 for smaller circle as details not needed
                                        ;additional command to adjust-circle-poses 1 for circle form; 2 for halved tool-object-width circle size
(defun orbital-poses(pose reso container-object-type tool-object-type)
  (let ((containerrim 0)
	(erate 1);<- circle;  stirring == (erate 0.03)
	(angle 0)
	(x 1)
        (defaultreso 12)
        (currentpose-smaller-radius-pose) ;ever changing small circle center poses
        (smallcircleposes '());list of one small circle at a time
        (orbitalposes '()))

    (setf containerrim (calc-radius container-object-type tool-object-type))

    (if (>= reso 2) (setf defaultreso reso))
    (setf angle (/(* 2  pi) defaultreso))
                                        ;defaultreso is this case is jsut the holder for whatever ?reso was decided on or real default reso- look above
    (loop while (<= x defaultreso)
          do (setf x   (+ x  1))
             (setf currentpose-smaller-radius-pose (cl-tf:copy-pose-stamped
                                                    (change-v pose :x-offset (* erate (* (/ containerrim 2) (cos (* x angle))))
                                                                   :y-offset (* (/ containerrim 2) (sin (* x angle))))))
             (setf smallcircleposes (adjust-circle-poses currentpose-smaller-radius-pose 9 1 container-object-type tool-object-type 2))
          append smallcircleposes)))

(defun get-start-mix-poses(reso pose)
  (print "spiral calculation")  
  (let*
      ((positions (list pose))
       (part (/ 360 reso)) 
       (angle (/(* 2 pi) part))
       (currentpose pose))

    (loop for x from 1 to part
          do
             (cons positions

                   (change-v currentpose :x-offset (* (exp (* part angle))(cos (* part angle)))
                                         :y-offset (* (exp (* part angle)) (sin (* part angle)))
                                         )))))


                                        ;repetition of the mid-mix motion
(defun rounds-amount (round one-pose)
  (let ((rounds round)
        (pose one-pose))

    (if (not round)
        (setf rounds 1)
        (setf rounds round)
        )
    (loop for x from 1 to rounds
          append pose)))

(defun change-v (pose &key (x-offset 0.0) (y-offset 0.0) (z-offset 0.0))
  (print "change-v")
  (cram-tf::copy-pose-stamped
   pose 
   :origin
   (let ((transform-translation (cl-transforms:origin pose)))
     (cl-transforms:copy-3d-vector
      transform-translation
      :x (let ((x-transform-translation (cl-transforms:x transform-translation)))
           (+ x-transform-translation x-offset))
      :y (let ((y-transform-translation (cl-transforms:y transform-translation)))
           (+ y-transform-translation y-offset))
      :z (let ((z-transform-translation (cl-transforms:z transform-translation)))
           (+ z-transform-translation z-offset))))
   :orientation
   (cl-transforms:orientation pose)))
