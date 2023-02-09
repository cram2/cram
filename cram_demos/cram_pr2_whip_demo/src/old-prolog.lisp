(in-package :demo)

(defun my-plan-function (&key
             &allow-other-keys)
  (print "My action designator is executable"))


(def-fact-group my-actions (desig:action-grounding)
  (<- (desig:action-grounding ?action-designator (my-plan-function ?resolved-action-designator))
    (spec:property ?action-designator (:type :my-action-designator))

    (desig:designator :action ((:type :my-action-designator))
                      ?resolved-action-designator)))

(defun whisk (&key
	       ((:object ?object-designator))
	       ((:object-type ?object-type))
	       ((:object-name ?object-name))
	       ((:arms ?arms))
	       ((:grasp ?grasp))
	       ((:context ?context))
	       ((:reso ?reso))
	       ;;((:effort ?effort))
	       ;;((:gripper-opening ?gripper-opening))
	      ; ((:left-grip-container-poses ?left-grip-container-poses))
	      ; ((:right-grip-container-poses ?right-grip-container-poses))
	       ((:left-approach-poses ?left-approach-poses))
	       ((:right-approach-poses ?right-approach-poses))
	       ((:left-start-mix-poses ?left-start-mix-poses))
	       ((:right-start-mix-poses ?right-start-mix-poses))
	       ((:left-mid-mix-poses ?left-mid-mix-poses))
               ((:right-mid-mix-poses ?right-mid-mix-poses))
               ((:left-end-mix-poses ?left-end-mix-poses))
               ((:right-end-mix-poses ?right-end-mix-poses))
               ((:left-retract-poses ?left-retract-poses))
	       ((:right-retract-poses ?right-retract-poses))
             &allow-other-keys)

  ;;just some prints cause who does not like them
  (format t "My action designator is executable; ~%
             flipping: object-type ~a object-name ~a arm: ~a grasp: ~a ~%"
	  ?object-type ?object-name ?arms ?grasp)


;  (roslisp:ros-info (cut-pour pour) "Approaching")

  ;grip bowl
    ;; (exe:perform
    ;;  (desig:an action
    ;;            (type approaching)
    ;;            (left-poses ?left-grip-container-poses)
    ;;            (right-poses ?right-grip-container-poses)
    ;;            ;;(desig:when ?collision-mode
    ;; 	       ;;(collision-mode ?collision-mode))))
    ;; 	       ))
    ;; (cpl:sleep 2)
  
  ;tool to center of container
    (exe:perform
     (desig:an action
               (type approaching)
               (left-poses ?left-approach-poses)
               (right-poses ?right-approach-poses)
               ;;(desig:when ?collision-mode
	       ;;(collision-mode ?collision-mode))))
	       ))
    (cpl:sleep 2)

   ;mix
 
    (exe:perform
     (desig:an action
               (type blending)
               (left-poses ?left-start-mix-poses)
               (right-poses ?right-start-mix-poses)
               ;;(desig:when ?collision-mode
  	       (collision-mode :allow-all)))
    (cpl:sleep 2)

  (if  (eq ?context :mix)
    (exe:perform
     (desig:an action
               (type blending)
               (left-poses ?left-mid-mix-poses)
               (right-poses ?right-mid-mix-poses)
               ;;(desig:when ?collision-mode
    	       (collision-mode :allow-all))))
  (cpl:sleep 2)

    (exe:perform
     (desig:an action
               (type blending)
               (left-poses ?left-end-mix-poses)
               (right-poses ?right-end-mix-poses)
               ;;(desig:when ?collision-mode
    	       (collision-mode :allow-all)))
  (cpl:sleep 2)

    (format t "my poses left: ~a ~%
             my poses right: ~a ~%" ?left-retract-poses
	     ?right-retract-poses)
    (exe:perform
     (desig:an action
               (type approaching)
               (left-poses ?left-retract-poses)
               (right-poses ?right-retract-poses)
               ;;(desig:when ?collision-mode
    	       (collision-mode :allow-all)))
  (cpl:sleep 2)

  )

 ;; (def-fact-group pancake-actions (desig:action-grounding)
;;   (<- (desig:action-grounding ?action-designator (whisk ?resolved-action-designator))
;;     (spec:property ?action-designator (:type :whisking))
;; ;;TODO: THIS STAYS THE SAME DONT CHANGE !!!! ++++++++++++++++++++++++++++++++
;;     ;; ==============   extract info from ?action-designator   ==============
;;     (spec:property ?action-designator (:object ?object-designator))
;;     (desig:current-designator ?object-designator ?current-object-desig)
;;     (spec:property ?current-object-desig (:type ?object-type))
;;     (spec:property ?current-object-desig (:name ?object-name))
;;     (spec:property ?action-designator (:context ?context))
       
    
;;     ;; ==============   extract arms or set arms               ==============
;;     (man-int:arms-for-object-type ?object-type ?arms-for-object)
;;     (-> (equal ?arms-for-object nil)
;;         (-> (spec:property ?action-designator (:arm ?arm))
;; 	    (true)
;; 	    (format "Please set a specific arm ~%")))


;;     ;; ==============  infer missing information like ?grasp   ==============
;;     (lisp-fun man-int:get-object-transform ?current-object-desig ?object-transform)
;;     (lisp-fun man-int:calculate-object-faces ?object-transform (?facing-robot-face ?bottom-face))
;;     (-> (man-int:object-rotationally-symmetric ?object-type)
;;         (equal ?rotationally-symmetric t)
;;         (equal ?rotationally-symmetric nil))
;;     (-> (spec:property ?action-designator (:grasp ?grasp))
;;         (true)
;;         (and (member ?arm ?arms)
;;              (lisp-fun man-int:get-action-grasps ?object-type ?arm ?object-transform ?grasps)
;;              (member ?grasp ?grasps)))
;;     ;;(lisp-fun man-int:get-action-gripping-effort ?object-type ?effort)
;;     ;;(lisp-fun man-int:get-action-gripper-opening ?object-type ?gripper-opening)

;; ;temporary ..stay 
;;     (->( equal ?reso nil)
;;     (true))
;;     ;;======infer reso AHHHHHHH?! =====
;;   ;  (format "printout: ~a" (type-of ?action-designator))
;;   ;         (get-reso-from-list ?action-designator) ; (cdr (assoc ,:reso ,?action-designator :reso #'equal)))
;;   ;  (-> (spec:property ?action-designator (:reso ?reso))
;;    ;     (true)
;;     ;    (?reso is (get-action-reso ?reso))
;;      ;   )
;; ;   (defmacro getassoc (key alist)
;; ;  `(cdr (assoc ,key ,alist :test #'equal)))

;; ;(defun get-object-transform (object-designator)
;;  ; (car (getassoc :transform (desig:desig-prop-value object-designator :pose))))

;;        ;(and  (format "reso was nil and set to: ~a" ?reso)
;;         ;    (get-action-reso :reso)
;;          ;   ))
;;           ;  (format "reso is prolly not nil ~a" ?reso)
;;        ; )
;;       ;  (lisp-fun get-action-reso ?reso))
;;          ;   (format "reso is:  ~a" ?reso))
;;       ;  (true))
;;       ; (equal ?reso 12)) ;default reso
    
;; 	     ;TODO reso changes (and (lisp-fun get-default-reso ?reso)
	    
 
;;   ;      (true)
;;        ;)   
    
;;     ;;TODO: THIS STAYS THE SAME DONT CHANGE TILL HERE !!!! ++++++++++++

    
;;     ;; ==============  calculate trajectory with given grasp  ==============
;;     (equal ?objects (?current-object-desig))
;;     (-> (member :left ?arm)
;; 	;;TODO change name here nad start with one pose only
;; 	(and (lisp-fun man-int:get-action-trajectory :mixing
;; 		       :left ?grasp T ?objects
;; 		       ?left-trajectory) ;?reso)
;; 	    ; (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :grip-container
;; 		;       ?right-grip-container-poses)
;; 	     (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :approach
;; 		       ?left-approach-poses)
;; 	     (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :start-mix
;; 	      	       ?left-start-mix-poses)
;; 	     (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :mid-mix
;; 	      	       ?left-mid-mix-poses)
;;              (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :end-mix
;;                        ?left-end-mix-poses)
;;              (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :retract
;;                        ?left-retract-poses)

;; 	     (lisp-fun get-arm-return :left ?left)
;; 	     (format "arm: ~a ~% ?left-trajectory: ~a" ?left ?left-trajectory))
;;         ;;care u have to define the else here otherwise it cannot be resolved
;; 	(and (equal ?left NIL)
;; 	     (equal ?left-trajectory NIL)
;; 	     (equal ?left-approach-poses NIL)
;; 	     (equal ?left-start-mix-poses NIL)
;; 	     (equal ?left-mid-mix-poses NIL)
;;              (equal ?left-end-mix-poses NIL)
;; 	     (equal ?left-retract-poses NIL)
;; 	     ))
    

;;     (-> (member :right ?arm)
;; 	;;TODO change name here nad start with one pose only
;; 	 (and (lisp-fun man-int:get-action-trajectory :mixing
;; 			:right ?grasp T ?objects
;; 			?right-trajectory) ;?reso)
;; 	    ;  (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :grip-container
;; 		;	?left-grip-container-poses)
;; 	      (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :approach
;; 			?right-approach-poses)
;; 	       (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :start-mix
;; 	       		?right-start-mix-poses)
;; 	      (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :mid-mix
;; 	      		?right-mid-mix-poses)
;;                (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :end-mix
;;                          ?right-end-mix-poses)
;;                 (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :retract
;;                          ?right-retract-poses)

  
;; 	      (lisp-fun get-arm-return :right ?right)
;;      	     (format "arm: ~a ~% ?right-trajectory: ~a" ?right ?right-trajectory))
;; 	;;care u have to define the else here otherwise it cannot be resolved
;; 	 (and (equal ?right NIL)
;; 	      (equal ?right-trajectory NIL)
;; 	      (equal ?right-approach-poses NIL)
;; 	      (equal ?right-start-mix-poses NIL)
;; 	      (equal ?right-mid-mix-poses NIL)
;;               (equal ?right-end-mix-poses NIL)
;; 	      (equal ?right-retract-poses NIL)
;; 	      ))

;;     ;; ==============   put together resolving designator      ==============
;;     ;;TODO change name
;;     (desig:designator :action ((:type :whisking)
;; 			       (:object ?object-designator)
;; 			       (:object-type ?object-type)
;; 			       (:object-name ?object-name)
;; 			       (:arms ?arm)
;; 		               (:grasp ?grasp)
;; 			       (:context ?context)
;; 			       (:reso ?reso)
;; 			       ;;(:effort ?effort)
;; 			       ;;(:gripper-opening ?gripper-opening)
;; 			      ; (:left-grip-container-poses ?left-grip-container-poses)
;; 			      ; (:right-grip-container-poses ?right-grip-container-poses)
;; 		               (:left-approach-poses ?left-approach-poses)
;; 			       (:right-approach-poses ?right-approach-poses)
;; 			       (:left-start-mix-poses ?left-start-mix-poses)
;; 			       (:right-start-mix-poses ?right-start-mix-poses)
;;                                (:left-mid-mix-poses ?left-mid-mix-poses)
;;                                (:right-mid-mix-poses ?right-mid-mix-poses)
;;                                (:left-end-mix-poses ?left-end-mix-poses)
;;                                (:right-end-mix-poses ?right-end-mix-poses)
;;                                (:left-retract-poses ?left-retract-poses)
;; 	                       (:right-retract-poses ?right-retract-poses)
;; 			       )
;;                       ?resolved-action-designator)))

;;thats just for fun
(defun get-arm-return (?arm)
  ?arm)

 (defun get-action-reso (?reso)
(if (eq ?reso nil) 12 ?reso))

(defun get-reso-from-list (alist)
 (cdr (assoc :reso alist)))

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
                                                        ; reso  
                                                           )
                                                         
  (print "entered mixing")
  
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
         (mix-poses (adjust-circle-poses approach-pose :reso))
					;(mix-poses  (circle-poses approach-pose))
	(start-mix-poses (rec-spiral-poses object-type approach-pose :reso))
                                        ; (spiral-poses approach-pose))
         ;spiral inwards
         (end-mix-poses (reverse-spiral-poses object-type approach-pose :reso))
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
   '((0 -0.12  0.161)(1 0 0 0))) ;x -0.12

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

;;the z should be defined by:
;;object in hand where?
;;how long is object from where gripper is
;;yeh
;;TODO: at least write it like it would make sense like that
;;if its really in the hand who cares

;should be defined in household later--cos 12 is too close to rim
(defmethod get-object-type-robot-frame-mix-rim-bottom-transform
   ((object-type (eql :big-bowl)))
  '((0.0 -0.12 0.06)(1 0 0 0)))

(defmethod get-object-type-robot-frame-mix-rim-top-transform
   ((object-type (eql :big-bowl)))
  '((0.0 -0.9 0.11)(1 0 0 0)))

;; =========  is in trajectory defined normaly ==========

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

(defun adjust-circle-poses(pose reso)
  (let ((containerrim 0.06); <- currently - big-bowl hard gecoded, gotta adjust top and bottom rim
	(erate 1);<- circle;  stirring == (erate 0.03)
	(angle 0)
	(x 1)
        (defaultreso 12)
	)
;(if (eql reso nil)
    (setf angle (/(* 2  pi) defaultreso));reso))
    ;(and (setf angle (/(* 2 pi) ?reso)) (setf defaultreso ?reso))
 ;   )
    
    
 ;defaultreso is this case is jsut the holder for whatever ?reso was decided on or real default reso- look above
(loop while (<= x defaultreso)
  do (setf x   (+ x  1))           ; or better: do (decf row)
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

   ; (print "~a" reso)
    (setf rim (nth 2 (car (get-object-type-robot-frame-mix-rim-bottom-transform object-type))))
   
    
 (setf angle (/(* 2  pi) defaultreso))
    (loop while (<= x defaultreso)
	  do (setf x   (+ x  1))
	     collect(change-v pose :x-offset (*(* (/ rim defaultreso) (exp (* k (* x angle))) (cos (* x angle))))
				   :y-offset (*(* (/ rim defaultreso) (exp (* k(* x  angle))) (sin (* x  angle)))))
	  )))

(defun reverse-spiral-poses(object-type pose reso)
  (let
   ( (k 0.4) ;0.3 <-'spiralness'
     (defaultreso 12)
     (rim 0.06); needs to be pulled out from household - same goes for top and bottom diffrence.
     ;for spiral only top rim needed.
     (angle 0)  
     (x 0)
     (start-pose nil)
     )
    
   ; (print "~a" reso)
    (setf rim (nth 2 (car (get-object-type-robot-frame-mix-rim-bottom-transform object-type))))
   
    (setf angle (/(* 2  pi) defaultreso))
    (setf x defaultreso)
    (loop while (>= x 0)
	  do (setf x (- x 1))
	     collect(change-v pose :x-offset (*(* (/ rim defaultreso) (exp (* k (* x  angle))) (cos (* x  angle))))
				   :y-offset (*(* (/ rim defaultreso) (exp (* k(* x   angle))) (sin (* x   angle)))))
	  ))
  )

(defun get-start-rim-pose
(cram-tf::copy-pose-stamped)
  )

;; (defun spiral-poses(pose)
;;   (print "spiral")
;;   (let
;;    ( (k 0.4) ;0.3
;;      (a 12)
;;      (rim 0.06)
;;     )  
;;   (list
;;    ;spiral made iwth r = 0.06; phi = angle ,  r= a*e^(k* phi); a = amount of segments
;;    ;k = gradient of one segment to the other
;;    		 ;; (change-v pose :x-offset (*(* 0.06 (exp 0))(cos 0))
;;                  ;;                :y-offset (*(* 0.06 (exp 0)) (sin 0)))
;;    		    (change-v pose :x-offset (*(* (/ rim a) (exp (* k (/ pi 6)))) (cos (/ pi 6)))
;; 				    :y-offset (*(* (/ rim a) (exp (* k(/ pi 6)))) (sin (/ pi 6))))
;;    		    (change-v pose :x-offset (*(* (/ rim a) (exp (* k (/ pi 3)))) (cos (/ pi 3)))
;; 				    :y-offset (*(* (/ rim a) (exp (* k(/ pi 3)))) (sin (/ pi 3))))
;;    		    (change-v pose :x-offset (*(* (/ rim a) (exp (* k (/ pi 2)))) (cos (/ pi 2)))
;; 				    :y-offset (*(* (/ rim a) (exp (* k(/ pi 2)))) (sin (/ pi 2))))
;;    		    (change-v pose :x-offset (*(* (/ rim a) (exp (* k(* 2(/ pi 3))))) (cos (* 2(/ pi 3))))
;; 				    :y-offset (*(* (/ rim a) (exp (* k(* 2(/ pi 3))))) (sin (* 2(/ pi 3)))))
;;    		    (change-v pose :x-offset (*(* (/ rim a) (exp (* k (* 5(/ pi 6))))) (cos  (* 5(/ pi 6))))
;; 				    :y-offset (*(* (/ rim a) (exp (* k (* 5(/ pi 6))))) (sin  (* 5(/ pi 6)))))

;;   		    (change-v pose :x-offset (*(* (/ rim a) (exp (* k pi))) (cos pi))
;; 				    :y-offset (*(* (/ rim a) (exp (* k pi))) (sin pi)))

;;    		    (change-v pose :x-offset (*(* (/ rim a) (exp (* k (* 7(/ pi 6))))) (cos  (* 7(/ pi 6))))
;; 				    :y-offset (*(* (/ rim a) (exp (* k (* 7(/ pi 6))))) (sin  (* 7(/ pi 6)))))		    
;;    		    (change-v pose :x-offset (*(* (/ rim a) (exp (* k (* 4(/ pi 3))))) (cos  (* 4(/ pi 3))))
;; 				    :y-offset (*(* (/ rim a) (exp (* k (* 4(/ pi 3))))) (sin  (* 4(/ pi 3)))))
;;    		    (change-v pose :x-offset (*(* (/ rim a) (exp (* k (* 3(/ pi 2))))) (cos  (* 3(/ pi 2))))
;; 				    :y-offset (*(* (/ rim a) (exp (* k (* 3(/ pi 2))))) (sin  (* 3(/ pi 2)))))
;;    		    (change-v pose :x-offset (*(* (/ rim a) (exp (* k (* 5(/ pi 3))))) (cos  (* 5(/ pi 3))))
;; 				    :y-offset (*(* (/ rim a) (exp (* k (* 5(/ pi 3))))) (sin  (* 5(/ pi 3)))))
;;    		    (change-v pose :x-offset (*(* (/ rim a) (exp (* k (* 11(/ pi 6))))) (cos  (* 11(/ pi 6))))
;; 				    :y-offset (*(* (/ rim a) (exp (* k (* 11(/ pi 6))))) (sin  (* 11(/ pi 6)))))

;;    		    (change-v pose :x-offset (*(* (/ rim a) (exp (* k (* 2 pi)))) (cos  (* 2 pi)))
;; 				    :y-offset (*(* (/ rim a) (exp (* k (* 2 pi)))) (sin  (* 2 pi))))

;; 	     ;ausgangs posi -> should be in mid-mix
;; 		    ;; (change-v pose :x-offset (* 0.06 (cos 0))
;;        		    ;;                :y-offset (* 0.06 (sin 0)))
;;  )))

(defun get-start-mix-poses(reso pose);grasp start-mix-poses &optional) - for pose z axis is needed to be in object coordinationsys
(print "spiral calculation")  
  ;; iteration through psi
  (let
    ((positions (list pose))
    (part (/ 360 12)) ;make sure it's int
     (angle (/(* 2 pi)(/ 360 12))) ; 12 is replacment for reso for now
     (currentpose pose)
	      )


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

;; =========  is in trajectory defined normly ==========
(defun get-flip-tilt-poses (grasp approach-poses &optional (angle (cram-math:degrees->radians 15))
						   (height 0))
  (print "flip-tilt-poses-calculated")

  (mapcar (lambda (?approach-pose)
	    (let ((?pose (change-v ?approach-pose :z-offset height)))
	      ;;depending on the grasp the angle to tilt is different
	      (case grasp
		(:front (rotate-once-pose-flip ?pose (- angle) :y))
		(:left (rotate-once-pose-flip ?pose (- angle) :x))
		(:right (rotate-once-pose-flip ?pose (+ angle) :x))
		(:top (rotate-once-pose-flip ?pose (+ angle) :y))
		(t (error "front, left, right, top")))))
          approach-poses))

;;helper function for tilting
;;rotate the pose around the axis in an angle
(defun rotate-once-pose-flip (pose angle axis)
  (print "rotate once")
  (cl-transforms-stamped:copy-pose-stamped
   pose
   :orientation (let ((pose-orientation (cl-transforms:orientation pose)))
                  (cl-tf:normalize
                   (cl-transforms:q*
                    (cl-transforms:axis-angle->quaternion
                     (case axis
                       (:x (cl-transforms:make-3d-vector 1 0 0))
                       (:y (cl-transforms:make-3d-vector 0 1 0))
                       (:z (cl-transforms:make-3d-vector 0 0 1))
                       (t (error "in ROTATE-ONCE-POSE forgot to specify axis properly: ~a" axis)))
                     angle)
                    pose-orientation)))))
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
