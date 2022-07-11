;;;
;;; Copyright (c) 2022, Vanessa Hassouna <hassouna@cs.uni-bremen.de>
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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors
;;;       may be used to endorse or promote products derived from this software
;;;       without specific prior written permission.
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

(in-package :demo)


;; (<- (desig:action-grounding ?action-designator (pour ?resolved-action-designator))
;;     (spec:property ?action-designator (:type :pouring))
;;     ;; extract info from ?action-designator
;;     (spec:property ?action-designator (:object ?object-designator))
;;     (desig:current-designator ?object-designator ?current-object-desig)
;;     (spec:property ?current-object-desig (:type ?object-type))
;;     (spec:property ?current-object-desig (:name ?object-name))
;;      (-> (spec:property ?action-designator (:arms ?arms))
;;         (true)
;;         (and (man-int:robot-free-hand ?_ ?arm)
;;              (equal ?arms (?arm))))
;;      (lisp-fun man-int:get-object-transform ?current-object-desig ?object-transform)
   
;;     ;; infer missing information like ?grasp type, gripping ?maximum-effort, manipulation poses
;;     (lisp-fun man-int:calculate-object-faces ?object-transform (?facing-robot-face ?bottom-face))
;;     (-> (man-int:object-rotationally-symmetric ?object-type)
;;         (equal ?rotationally-symmetric t)
;;         (equal ?rotationally-symmetric nil))
;;     (-> (spec:property ?action-designator (:grasp ?grasp))
;;         (true)
;;         (and (member ?arm ?arms)
;;              (lisp-fun man-int:get-action-grasps ?object-type ?arm ?object-transform ?grasps)
;;              (member ?grasp ?grasps)))
;;     (lisp-fun man-int:get-action-gripping-effort ?object-type ?effort)
;;     (lisp-fun man-int:get-action-gripper-opening ?object-type ?gripper-opening)

;;     ;; calculate trajectory
;;     (equal ?objects (?current-object-desig))
;;     (-> (member :left ?arms)
;;         (and (lisp-fun man-int:get-action-trajectory :pouring :left ?grasp T ?objects 
;;                        ?left-pouring-pose)
;;              (lisp-fun man-int:get-traj-poses-by-label ?left-pouring-pose :approach
;;                        ?left-approach-poses)
;;              (lisp-fun man-int:get-traj-poses-by-label ?left-pouring-pose :tilting
;;                        ?left-tilt-poses))
             
;;         (and (equal ?left-approach-poses NIL)
;;              (equal ?left-tilt-poses NIL)))

;;      (-> (member :right ?arms)
;;         (and (lisp-fun man-int:get-action-trajectory :pouring :right ?grasp T ?objects 
;;                        ?right-pouring-pose)
;;              (lisp-fun man-int:get-traj-poses-by-label ?right-pouring-pose :approach
;;                        ?right-approach-poses)
;;              (lisp-fun man-int:get-traj-poses-by-label ?right-pouring-pose :tilting
;;                        ?right-tilt-poses))
             
;;         (and (equal ?right-approach-poses NIL)
;;              (equal ?right-tilt-poses NIL)))

;;      (-> (desig:desig-prop ?action-designator (:collision-mode ?collision-mode))
;;         (true)
;;         (equal ?collision-mode nil))

;;      ;; put together resulting action designator
;;     (desig:designator :action ((:type :pouring)
;;                                (:object ?current-object-desig)
;;                                (:object-type ?object-type)
;;                                (:object-name  ?object-name)
;;                                (:arms ?arms)
;;                                (:grasp ?grasp)
;;                                (:left-approach-poses ?left-approach-poses)
;;                                (:right-approach-poses ?right-approach-poses)
;;                                (:left-tilt-poses ?left-tilt-poses)
;;                                (:right-tilt-poses ?right-tilt-poses)
;;                                (:collision-mode ?collision-mode))
;;                       ?resolved-action-designator))
    



;; (defun translate-pose-in-base (bTg &key (x-offset 0.0) (y-offset 0.0) (z-offset 0.0))
;;   (cram-tf:translate-transform-stamped bTg
;;                                        :x-offset x-offset
;;                                        :y-offset y-offset
;;                                        :z-offset z-offset))


;; (defun get-tilting-poses (grasp approach-poses &optional (angle (cram-math:degrees->radians 100)))
;;   (mapcar (lambda (?approach-pose)
;;             ;;depending on the grasp the angle to tilt is different
;;             (case grasp
;;               (:front (rotate-once-pose ?approach-pose (+ angle) :y))
;;               (:top-front (rotate-once-pose ?approach-pose (+ angle) :y))
;;               (:left-side (rotate-once-pose ?approach-pose (+ angle) :x))
;;               (:top-left (rotate-once-pose ?approach-pose (+ angle) :x))
;;               (:right-side (rotate-once-pose ?approach-pose (- angle) :x))
;;               (:top-right (rotate-once-pose ?approach-pose (- angle) :x))
;;               (:back (rotate-once-pose ?approach-pose (- angle) :y))
;;               (:top (rotate-once-pose ?approach-pose (- angle) :y))
;;               (t (error "can only pour from :side, back or :front :top :top-side"))))
;;           approach-poses))

;; ;;helper function for tilting
;; ;;rotate the pose around the axis in an angle
;; (defun rotate-once-pose (pose angle axis)
;;   (cl-transforms-stamped:copy-pose-stamped
;;    pose
;;    :orientation (let ((pose-orientation (cl-transforms:orientation pose)))
;;                   (cl-tf:normalize
;;                    (cl-transforms:q*
;;                     (cl-transforms:axis-angle->quaternion
;;                      (case axis
;;                        (:x (cl-transforms:make-3d-vector 1 0 0))
;;                        (:y (cl-transforms:make-3d-vector 0 1 0))
;;                        (:z (cl-transforms:make-3d-vector 0 0 1))
;;                        (t (error "in ROTATE-ONCE-POSE forgot to specify axis properly: ~a" axis)))
;;                      angle)
;;                     pose-orientation)))))

;; ;;get pouring trajectory workes like picking-up it will get the 
;; ;;object-type-to-gripper-tilt-approch-transform und makes a traj-segment out of it
;; ;;here we have only the approach pose, followed by that is the titing pose (above)
;; (defmethod man-int:get-action-trajectory :heuristics 20 ((action-type (eql :pouring))
;;                                                          arm
;;                                                          grasp
;;                                                          location
;;                                                          objects-acted-on
;;                                                          &key )
;;   (let* ((object
;;            (car objects-acted-on))
;;          (object-name
;;            (desig:desig-prop-value object :name))
;;          (object-type
;;            (desig:desig-prop-value object :type))
;;          (bTo
;;            (man-int:get-object-transform object))
;;          ;; The first part of the btb-offset transform encodes the
;;          ;; translation difference between the gripper and the
;;          ;; object. The static defined orientation of bTb-offset
;;          ;; describes how the gripper should be orientated to approach
;;          ;; the object in which something should be poured into. This
;;          ;; depends mostly on the defined coordinate frame of the
;;          ;; object and how objects should be rotated to pour something
;;          ;; out of them.

;; 	 ;;on the poured-into
;; 	;;  (defmethod man-int:get-object-type-robot-frame-tilt-approach-transform 
;;   ;;   ((object-type (eql :bowl))
;;   ;;    arm
;;   ;;    (grasp (eql :top-left)))
;;   ;; '((0.0 0.2 0.24)(0 0 -0.707 0.707)))

;;          (bTb-offset
;;            (man-int::get-object-type-robot-frame-tilt-approach-transform
;;             object-type arm grasp))
;;          ;; Since the grippers orientation should not depend on the
;;          ;; orientation of the object it is omitted here.

;; 	 ;;on the poured-into o
;;   ;; 	 (man-int:def-object-type-to-gripper-transforms :bowl '(:left :right) :top
;;   ;; :grasp-translation `(,(- *bowl-grasp-x-offset*) 0.0d0 ,*bowl-grasp-z-offset*)
;;   ;; :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
;;   ;; :pregrasp-offsets `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
;;   ;; :2nd-pregrasp-offsets `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
;;   ;; :lift-translation `(0.0 0.0 ,*bowl-pregrasp-z-offset*)
;;   ;; :2nd-lift-translation `(0.0 0.0 ,*bowl-pregrasp-z-offset*))
;;          (oTg-std
;;            (cram-tf:copy-transform-stamped
;;             (man-int:get-object-type-to-gripper-transform
;;              object-type object-name arm grasp)
;;             :rotation (cl-tf:make-identity-rotation)))

;; 	 ;;btb basicly umgerechnet zu gripper pose in base gespeichert als approach-
;; 	 ;'die rotation wird aber nicht umgerechnt die bleibt die selbe und wird
;; 	 ;;aus btb-offset genommen
;;          (approach-pose
;;            (cl-tf:copy-pose-stamped 
;;             (man-int:calculate-gripper-pose-in-base
;;               (cram-tf:apply-transform
;;                (cram-tf:copy-transform-stamped 
;;                 bTb-offset
;;                 :rotation (cl-tf:make-identity-rotation))
;;                bTo)
;;               arm oTg-std)
;;             :orientation 
;;             (cl-tf:rotation bTb-offset)))

;; 	 ;;uses xy rotation so the gripper is tilted (still act on the object-pour
;; 	 ;;into
;;          (tilting-poses
;;            (get-tilting-poses grasp (list approach-pose))))

;;     ;;bis hier hin waren alle operationen noch im frame vom object/gripper

    
;;     ;;ueber approach-pose und tiling poses wird gefiltert
;;     ;;es wird fuer beides ein make-traj-segment erstellt,
;;     ;;labe = approach/tilting
;;     ;;poes: ueber jede pose in approach/tiling wird iteriert
;;     ;;die uebergebene pose wird mit robtoer pose transformiert
;;     ;;so dass die pose dann in frame von dem roboter am ende ist
;;     (mapcar (lambda (label poses-in-base)
;;               (man-int:make-traj-segment
;;                :label label
;;                :poses (mapcar 
;;                        (lambda (pose-in-base)
;;                          (let ((mTb (cram-tf:pose->transform-stamped
;;                                      cram-tf:*fixed-frame*
;;                                      cram-tf:*robot-base-frame*
;;                                      0.0
;;                                      (btr:pose (btr:get-robot-object))))
;;                                (bTg-std
;;                                  (cram-tf:pose-stamped->transform-stamped
;;                                   pose-in-base
;;                                   (cl-tf:child-frame-id bTo))))
;;                            (cl-tf:ensure-pose-stamped
;;                             (cram-tf:apply-transform mTb bTg-std))))
;;                        poses-in-base)))
;;             '(:approach
;;               :tilting)
;;             `((,approach-pose)
;;               ,tilting-poses))))



;; (defun pour (&key
;;                ((:object ?object-designator))
;;                ((:object-name  ?object-name))
;;                ((:object-type ?object-type))
;;                ((:arms ?arms))
;;                ((:grasp ?grasp))
;;                ((:left-approach-poses ?left-approach-poses))
;;                ((:right-approach-poses ?right-approach-poses))
;;                ((:left-tilt-poses ?left-tilt-poses))
;;                ((:right-tilt-poses ?right-tilt-poses))
;;                ((:collision-mode ?collision-mode))
;;              &allow-other-keys)
;;   "Object already in hand, approach 2nd object, tilt 100degree, tilt back"
  
;;   (roslisp:ros-info (cut-pour pour) "Approaching")
;;   (cpl:with-failure-handling
;;       ((common-fail:manipulation-low-level-failure (e)
;;          (roslisp:ros-warn (cut-and-pour-plans pour)
;;                            "Manipulation messed up: ~a~%Ignoring."
;;                            e)
;;          ;; (return)
;;          ))
;;     (exe:perform
;;      (desig:an action
;;                (type approaching)
;;                (left-poses ?left-approach-poses)
;;                (right-poses ?right-approach-poses)
;;                (desig:when ?collision-mode
;;                  (collision-mode ?collision-mode))))
;;     (cpl:sleep 2)
    
;;     (roslisp:ros-info (cut-pour pour) "Tilting")
;;     (cpl:with-failure-handling
;;         ((common-fail:manipulation-low-level-failure (e)
;;            (roslisp:ros-warn (cut-and-pour-plans pour)
;;                              "Manipulation messed up: ~a~%Ignoring."
;;                              e)))
;;       (exe:perform
;;        (desig:an action
;;                  (type tilting)
;;                  (left-poses ?left-tilt-poses)
;;                  (right-poses ?right-tilt-poses)
;;                  (desig:when ?collision-mode
;;                    (collision-mode ?collision-mode)))))

;;     (cpl:sleep 4)
    
;;     (cpl:with-failure-handling
;;         ((common-fail:manipulation-low-level-failure (e)
;;            (roslisp:ros-warn (cut-and-pour-plans pour)
;;                              "Manipulation messed up: ~a~%Ignoring."
;;                              e)))
;;       (exe:perform
;;        (desig:an action
;;                  (type approaching)
;;                  (left-poses ?left-approach-poses)
;;                  (right-poses ?right-approach-poses)
;;                  (desig:when ?collision-mode
;;                    (collision-mode ?collision-mode)))))))
