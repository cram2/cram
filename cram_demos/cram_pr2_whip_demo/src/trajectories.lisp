;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;                     Christopher Pollok <cpollok@cs.uni-bremen.de>
;;;                     Vanessa Hassouna <hassouna@uni-bremen.de>
;;;                     Thomas Lipps <tlipps@uni-bremen.de>
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


;;get pouring trajectory workes like picking-up it will get the 
;;object-type-to-gripper-tilt-approch-transform und makes a traj-segment out of it
;;here we have only the approach pose, followed by that is the titing pose (above)
;;TODO: change name and put into designator the correct key
(defmethod man-int:get-action-trajectory :heuristics 20 ((action-type (eql :mixing))
                                                         arm
                                                         grasp
                                                         location
                                                         objects-acted-on
                                                         &key
                                                         context
                                                         reso
                                                         tool-object-type  
                                                           )
                                                         
  (print "entered mixing")
  (print tool-object-type)
  ;;TODO DONT CHANGE THIS SAME +++++++++++
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
         ;; object. The static defined orientation of bTb-offset
         ;; describes how the gripper should be orientated to approach
         ;; the object in which something should be poured into. This
         ;; depends mostly on the defined coordinate frame of the
         ;; object and how objects should be rotated to pour something
         ;; out of them.
         (bTb-offset
	   ;;TODO: call correct function
           (get-object-type-robot-frame-mix-approach-transform
            object-type arm grasp))
           ;depending on object usually set on 12 o clock of the container opening
         (bTb-liftoffset
           (get-object-type-robot-frame-mix-retract-transform
            object-type arm grasp))
         ;; Since the grippers orientation should not depend on the
         ;; orientation of the object it is omitted here.
         (oTg-std
           (cram-tf:copy-transform-stamped
            (man-int:get-object-type-to-gripper-transform
             object-type object-name arm grasp)
            :rotation (cl-tf:make-identity-rotation)))
	 ;; (other-arm (cond ((equal arm :left) :right)(t :left)))
	 ;; (grip-container-pose
	 ;;   (cl-tf:copy-pose-stamped 
         ;;    (man-int:calculate-gripper-pose-in-base
         ;;      (cram-tf:apply-transform
         ;;       (cram-tf:copy-transform-stamped 
         ;;        bTb-offset
         ;;        :rotation (cl-tf:make-identity-rotation))
         ;;       bTo)
         ;;      other-arm oTg-std)
         ;;    :orientation 
         ;;    (cl-tf:rotation bTb-offset)))
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
         ;top+ bottom, append
         (mix-poses (adjust-circle-poses-context approach-pose reso context object-type tool-object-type))
					;(mix-poses  (circle-poses approach-pose))
	(start-mix-poses (rec-spiral-poses object-type approach-pose reso))
                                        ; (spiral-poses approach-pose))
         ;spiral inwards
         (end-mix-poses (reverse-spiral-poses object-type approach-pose reso))
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
	 ;;TODO: here come all your new poses calculated from the approach pose
	 ;;wrote new functions that changes height and stuff but as metioned in the
	 ;;comments below its hardcoded should be aabb box stuff calculating
	 
         ;; ;;approach-pose was not a list yet
         ;; (flip-tilt-poses
         ;;   (get-flip-tilt-poses grasp (list approach-pose)
	 ;; 			(cram-math:degrees->radians 15)
	 ;; 			-0.085))
	 
         ;; ;;flip-tilt-poses is a list already
	 ;; ;;the 0.15 value should depent on the object acted on.. but for now its k
	 ;; (push-foward-poses
	 ;;   (get-flip-tilt-poses grasp
	 ;; 			(get-push-foward-poses grasp flip-tilt-poses 0.12)
	 ;; 			(cram-math:degrees->radians 6.5) -0.03))

	 ;; (lift-pancake-poses
	 ;;   (get-flip-tilt-poses grasp push-foward-poses
	 ;; 			(cram-math:degrees->radians 0) 0.1))

	 ;; (flip-pancake-poses
	 ;;   (get-flip-tilt-poses :left
         ;;                        (get-push-foward-poses :left 
	 ;; 			lift-pancake-poses 0.05)
	 ;; 			(cram-math:degrees->radians -90) 0.2))

	   
	 )
    (print "pose is generated now the traj-segments are calculated")
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
            '(;:grip-container
	      :approach
	      :start-mix
	      :mid-mix
              :end-mix
              :retract
	      )
	    `(;(,grip-container-pose)
	      (,approach-pose)
	      ,start-mix-poses
	      ,mix-poses
              ,end-mix-poses
              (,retract-pose)
	      ))))	     



;; =========  is in household defined normaly ==========
(defmethod get-object-type-robot-frame-mix-grip-approach-transform
    ((object-type (eql :big-bowl))
     (arm (eql :left))
     (grasp (eql :top)))
   '((0 -0.12  0.161)(1 0 0 0)))

;;TODO: change name and numbers this name should be the same as the function belows
(defmethod get-object-type-robot-frame-mix-approach-transform
    ((object-type (eql :big-bowl))
      (arm (eql :right))
     (grasp (eql :top)))
  '((0.02 -0.12 0.161)(1 0 0 0)))

(defmethod get-object-type-robot-frame-mix-grip-retract-transform
    ((object-type (eql :big-bowl))
     (arm (eql :left))
     (grasp (eql :top)))
   '((0.02 -0.12  0.2)(1 0 0 0)))

(defmethod get-object-type-robot-frame-mix-retract-transform
     ((object-type (eql :big-bowl))
      (arm (eql :right))
     (grasp (eql :top)))
  '((0.02 -0.12 0.28)(1 0 0 0)))   

;;the z should be  defined by:
;;object in hand where?
;;how long is object from where gripper is
;;y is height

;should be defined in household later--cos 12 is too close to rim


;decided to use the z axis as radius measure (important for mix center point calculation)
;for top rim look at mix-approach-transform
(defmethod get-object-type-robot-frame-mix-rim-grip-deep-approach-transform
    ((object-type (eql :big-bowl))
        (arm (eql :right))
     (grasp (eql :top)))
  '((0.0 -0.12 0.15)(1 0 0 0)))

;;height depending on tool...center to bottom measurment
;;-bowl bottom can be looked up in rim-bottom-transform ^
;decided y is height/lenght of object from center grip
 (defmethod get-object-type-robot-frame-mix-tool-grip-bottom-transform
 ((object-type (eql :whisk))
      (arm (eql :right))
     (grasp (eql :top)))
  '((0.02 -0.12 0.06)(1 0 0 0)))

                                        ;decided to use the z axis as radius measure (important for mix center point calculation)
;decided y to be height og bowl and ground of bowl - need to calc with whisk dimensions to get robot gripper position in object tf before calc on higher level
(defmethod get-object-type-robot-frame-mix-rim-bottom-transform
   ((object-type (eql :big-bowl)))
   '((0.0 -0.12 0.06)(1 0 0 0)))

(defmethod get-object-type-robot-frame-mix-rim-top-transform
   ((object-type (eql :big-bowl)))
  '((0.0 -0.09 0.11)(1 0 0 0))) ;0.11

(defmethod get-object-type-robot-frame-mix-tool-transform
  ((object-type (eql :whisk)))
  '((0.0 0.145 0.015)(1 0 0 0))) ;why did I thought that 0.3 would be a good x ?? asking past tina . it gotta be 0.14 says my  19.02. brain

;; =========  is in trajectory defined normaly ==========
(defgeneric get-object-type-robot-frame-mix-rim-grip-deep-approach-transform (object-type arm grasp)
  (:documentation "Returns a transform stamped")
  (:method (object-type arm grasp)
    (man-int::call-with-specific-type #'get-object-type-robot-frame-mix-rim-grip-deep-approach-transform
                             object-type arm grasp)))

(defmethod get-object-type-robot-frame-mix-rim-grip-deep-approach-transform  (object-type arm grasp)
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

(defgeneric get-object-type-robot-frame-mix-tool-grip-bottom-transform ( object-type arm grasp)
  (:documentation "Returns a transform stamped")
  (:method (object-type arm grasp)
    (man-int::call-with-specific-type #'get-object-type-robot-frame-mix-tool-grip-bottom-transform
                             object-type arm grasp)))

(defmethod get-object-type-robot-frame-mix-tool-grip-bottom-transform (object-type arm grasp)
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

(defmethod get-object-type-robot-frame-mix-retract-transform :around (object-type arm grasp)
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
;;---retract^

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

(defgeneric get-object-type-robot-frame-mix-approach-transform (object-type arm grasp)
  (:documentation "Returns a transform stamped")
  (:method (object-type arm grasp)
    (man-int::call-with-specific-type #'get-object-type-robot-frame-mix-approach-transform
                             object-type arm grasp)))

(defmethod get-object-type-robot-frame-mix-approach-transform :around (object-type arm grasp)
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

(defun adjust-circle-poses-context (pose reso context object-type tool-object-type)
  (if (eq context :mix)
      (adjust-circle-poses pose reso 1 object-type tool-object-type
                           )
      (if (eq context :mix-eclipse)
           (adjust-circle-poses pose reso 0.03 object-type tool-object-type
                               )
           (if (eq
                context :mix-orbit)
             (orbital-poses pose reso object-type tool-object-type
                             )))
      ))

(defun calc-radius(container-object-type tool-object-type)
  (let ((container 0)
        (tool 0))
    (setf container (nth 2 (car (get-object-type-robot-frame-mix-rim-top-transform container-object-type))))
    (setf tool (nth 2 (car (get-object-type-robot-frame-mix-tool-transform tool-object-type))))
    
(- container tool)
                                        ;returns a rim number
  ))

(defun adjust-circle-poses(pose reso newerate object-type tool-object-type)
  (print tool-object-type)
  (let ((containerrim 0); <- currently - big-bowl hard gecoded, gotta adjust top and bottom rim
	(erate 1);== circle;  stirring == (erate 0.03) elipsiness
	(angle 0)
	(x 1)
        (defaultreso 12)
	)

    (setf erate newerate)
    (setf containerrim (nth 2 (car (get-object-type-robot-frame-mix-rim-bottom-transform object-type))))
    (print containerrim)
    ;; (if (eq context :mix-orbit)
    ;;    (orbital-poses pose reso)
    ;;     )
     (if (>= reso 2) (setf defaultreso reso))
     (setf angle (/(* 2  pi) defaultreso))
                                        ;reso))
    ;(and (setf angle (/(* 2 pi) ?reso)) (setf defaultreso ?reso))
 ;   )
    
 ;defaultreso is this case is jsut the holder for whatever ?reso was decided on or real default reso- look above
   
        (loop while (<= x defaultreso)
              do (setf x   (+ x  1))
            
    collect  (change-v pose :x-offset (* erate (* containerrim (cos (* x angle))))
			    :y-offset (* containerrim (sin (* x angle))))
    ))) 

(defun rec-spiral-poses(object-type pose reso)
  (let
   ( (k 0.4) ;0.3 <-'spiralness'
     (defaultreso 12)
     (rim 0.06); needs to be pulled out from household - same goes for top and bottom diffrence.
     ;for spiral only top rim needed.
     (angle 0)  
     (x 1)
     )
    
    (setf rim (nth 2 (car (get-object-type-robot-frame-mix-rim-bottom-transform object-type))))
                                        ; adjustment cos of get-object...-rim-bottom changes of structure (nth 2 (car (get-object-type-robot-frame-mix-rim-bottom-transform object-type))))
 (setf angle (/(* 2  pi) defaultreso))
    (loop while (<= x defaultreso)
	  do (setf x   (+ x  1))
	     collect(change-v pose :x-offset (*(* (/ rim defaultreso) (exp (* k (* x angle))) (cos (* x angle))))
				   :y-offset (*(* (/ rim defaultreso) (exp (* k(* x  angle))) (sin (* x  angle)))))
	  )))

(defun reverse-spiral-poses(object-type pose reso)
  (let
   ( (k 0.4) ;0.3 <-'spiralness'
     (defaultreso 11)
     (rim 0.06); needs to be pulled out from household - same goes for top and bottom diffrence.
     ;for spiral only top rim needed.
     (angle 0)  
     (x 0)
     (start-pose nil)
     )
    
    (setf rim ;(get-object-type-robot-frame-mix-rim-bottom-transform object-type))
          (nth 2 (car (get-object-type-robot-frame-mix-rim-bottom-transform object-type))))
   
    (setf angle (/(* 2  pi) defaultreso))
    (setf x defaultreso)
    (loop while (>= x 0)
	  do (setf x (- x 1))
	     collect(change-v pose :x-offset (*(* (/ rim defaultreso) (exp (* k (* x  angle))) (cos (* x  angle))))
				   :y-offset (*(* (/ rim defaultreso) (exp (* k(* x   angle))) (sin (* x   angle)))))
	  ))
  )

 ;orbital cos my dream gave me inspiration..
(defun orbital-poses(pose reso object-type tool-object-type)
   (let ((containerrim 0); <- currently - big-bowl hard gecoded, gotta adjust top and bottom rim
	(erate 1);<- circle;  stirring == (erate 0.03)
	(angle 0)
	(x 1)
        (defaultreso 12)
        (smallercircle 0.04) ;radius 3 cm circle for now
         (currentpose-smaller-radius-pose) ;ever changing small circle center
         (smallcircleposes '());list of one small circle at a time
         (orbitalposes '())
	)
   (if (>= reso 2) (setf defaultreso reso))
    (setf angle (/(* 2  pi) defaultreso));reso))
      
 ;defaultreso is this case is jsut the holder for whatever ?reso was decided on or real default reso- look above
(loop while (<= x defaultreso)
      do (setf x   (+ x  1))
         (setf currentpose-smaller-radius-pose (cl-tf:copy-pose-stamped (change-v pose :x-offset (* erate (* containerrim (cos (* x angle))))
                                                                                  :y-offset (* containerrim (sin (* x angle))))))
         (setf smallcircleposes (adjust-circle-poses currentpose-smaller-radius-pose  9 1 object-type tool-object-type))
             ; or better: do (decf row)
      
        ;; (cons smallcircleposes (change-v pose :x-offset (* erate (* containerrim (cos (* x angle))))
         ;;                          :y-offset (* containerrim (sin (* x angle)))))
        ; (collect currentpose-smaller-radius)
        append smallcircleposes)
                                        ;calc origin pose to current pose distance
                  ;(adjust-circle-poses currentpose-smaller-radius 9 1)
                                        ;x-offset below
      ))
                  
                  

                                        ;full small circle
                  
       ;going for fixed reso 9 cos small circle doesn't need to be super clean(?) 
                 ; (adjust-circle-poses (setf currentpose-smaller-radius (change-v pose :x-offset (* erate (* smallercircle (cos (* x angle))))
                  ;                                                                     :y-offset (* smallercircle  (sin (* x angle)))))
                   ;                   9 1))
		  
      
 ; cl-transforms:copy-3d-vector
; for small circle: (adjust-circle-poses pose reso :mix-circle/:mix)
  

(defun get-start-mix-poses(reso pose);grasp start-mix-poses &optional) - for pose z axis is needed to be in object coordinationsys
(print "spiral calculation")  
  ;; iteration through psi
  (let
    ((positions (list pose))
    (part (/ 360 12)) ;make sure its int
     (angle (/(* 2 pi)(/ 360 12))) ; 12 is replacment for reso for now
     (currentpose pose)
	      )

    ()

  (loop for x from 1 to part
	; resolution of circle; angle from 0 to (*2 pi) ;phi in radians
	do
	   (cons positions

		 (change-v currentpose :x-offset (* (exp (* part angle))(cos (* part angle)))
                                       :y-offset (* (exp (* part angle)) (sin (* part angle)))
		 ) 
	;	'((* (exp (* part angle))(cos (* part angle)))    ;x = r(angle) cos angle -> r(angle)= euler^angle
	;	(* (exp (* part angle) (sin (* part angle))))	 ;y= r(angle) sin angle
	;	0)
    
;(print "translating spiral poses to otb")

    ))))


;; (defun get-circle-poses(reso)
;;   (print "whisking circle calculated")
;;     (let
;;     ((positions get-object-type-robot-frame-mix-approach-transform)
;;     (part (/ 360 reso)) ;make sure it's int
;;      (angle (/(* 2 pi)(/ 360 reso)))
;;      (currentpose get-object-type-robot-frame-mix-approach-transform)
;;      )
      
;;   (loop for x from 1 to part
;; 	; resolution of circle; angle from 0 to (*2 pi) ;phi in radians
;; 	do
;; 	   (* (cos (* angle part)) (currentpose )); = x
;; 	   (* (sin (* angle part)) r) ; =y
;; 	   (cons positions
		 
;; 	;	(change-v (currentpose :x-offset () :y-offset ())) ;<- ==r
;; ;r = get 
;; 					;pi 2*r ; (x-h)² +(y-v)² = r² while h,v center(h = horizontal, v = vertical) of circle and r radius
;;   ;x= r * cos +x; y= r*sin +y 
;;   ))))

(defvar test)

(defun change-v (pose &key (x-offset 0.0) (y-offset 0.0) (z-offset 0.0))
  (print "change-v")
  (setf test pose)
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


(defun get-push-foward-poses (grasp poses &optional (push 0))
  (print "push-calculated")

  (mapcar (lambda (?pose)
	    (case grasp
		(:front (change-v ?pose :x-offset (+ push)))
		(:left (change-v ?pose :y-offset (+ push)))
		(:right (change-v ?pose :y-offset (- push)))
		(:top (change-v ?pose :x-offset (- push)))
		(t (error "front, left, right, top"))))
          poses))
