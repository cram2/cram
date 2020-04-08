;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;                     Christopher Pollok <cpollok@cs.uni-bremen.de>
;;;                     Vanessa Hassouna <hassouna@uni-bremen.de>
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

(defstruct traj-segment
  (label nil :type keyword)
  ;; (collision-mode :allow-all :type keyword)
  (poses nil :type list))

(defun make-empty-trajectory (labels)
  (mapcar (lambda (x)
            (make-traj-segment :label x :poses nil))
          labels))

(defun get-traj-poses-by-label (trajectory label)
  (traj-segment-poses
   (find label
         trajectory
         :key #'traj-segment-label)))


(defun calculate-gripper-pose-in-base (base-to-object-transform arm
                                       object-to-standard-gripper-transform)
  "Returns bPg, given bTo, the arm and oTg': bPg = bTo * oTg' * g'Tg.
`arm' is :left or :right."
  (let* ((gripper-tool-frame
           (ecase arm
             (:left cram-tf:*robot-left-tool-frame*)
             (:right cram-tf:*robot-right-tool-frame*)))
         (standard-to-particular-gripper-transform ; g'Tg
           (cl-transforms-stamped:transform->transform-stamped
            gripper-tool-frame
            gripper-tool-frame
            0.0
            (cut:var-value
             '?transform
             (car (prolog:prolog
                   `(and (cram-robot-interfaces:robot ?robot)
                         (cram-robot-interfaces:standard-to-particular-gripper-transform
                          ?robot ?transform)))))))
         (base-to-standard-gripper-transform      ; bTg'
           (cram-tf:multiply-transform-stampeds
            cram-tf:*robot-base-frame* gripper-tool-frame
            base-to-object-transform              ; bTo
            object-to-standard-gripper-transform  ; oTg'
            :result-as-pose-or-transform :transform)))
    (cram-tf:multiply-transform-stampeds ; bTg' * g'Tg = bTg
     cram-tf:*robot-base-frame* gripper-tool-frame
     base-to-standard-gripper-transform      ; bTg'
     standard-to-particular-gripper-transform ; g'Tg
     :result-as-pose-or-transform :pose)))

(defun calculate-gripper-pose-in-map (base-to-object-transform arm
                                      object-to-standard-gripper-transform)
  (let ((gripper-tool-frame
          (ecase arm
            (:left cram-tf:*robot-left-tool-frame*)
            (:right cram-tf:*robot-right-tool-frame*))))
    (cram-tf:multiply-transform-stampeds
     cram-tf:*fixed-frame* gripper-tool-frame
     (cram-tf:pose-stamped->transform-stamped
      (cram-tf:robot-current-pose) cram-tf:*robot-base-frame*)
     (cram-tf:pose-stamped->transform-stamped
      (calculate-gripper-pose-in-base base-to-object-transform arm
                                      object-to-standard-gripper-transform)
      gripper-tool-frame)
     :result-as-pose-or-transform :pose)))



;;;;;;;;;;;;;;;; Everything below is for pick/place & cut/pour only ;;;;;;;;;;;;;;;;;;

(defvar *known-grasp-types* nil
  "A list of symbols representing all known grasp types")

(defgeneric get-object-type-to-gripper-transform (object-type object-name arm grasp)
  (:documentation "Returns a pose stamped.
Gripper is defined by a convention where Z is pointing towards the object.")
  (:method (object-type object-name arm grasp)
    (call-with-specific-type #'get-object-type-to-gripper-transform
                             object-type object-name arm grasp)))


(defgeneric get-object-type-to-gripper-pregrasp-transform (object-type object-name
                                                           arm grasp grasp-transform)
  (:documentation "Returns a transform stamped")
  (:method (object-type object-name arm grasp grasp-transform)
    (call-with-specific-type #'get-object-type-to-gripper-pregrasp-transform
                             object-type object-name arm grasp grasp-transform)))


(defgeneric get-object-type-fixed-frame-slice-up-transform (object-type arm grasp)
  (:documentation "Returns a transform stamped"))

(defmethod get-object-type-fixed-frame-slice-up-transform :around (object-type arm grasp)
    (destructuring-bind
      ((x y z) (ax ay az aw))
      (call-next-method)
    (cl-tf:transform->transform-stamped
     cram-tf:*fixed-frame*
     cram-tf:*fixed-frame*
     0.0
     (cl-tf:pose->transform
      (cl-transforms:make-pose
       (cl-transforms:make-3d-vector x y z)
       (cl-transforms:make-quaternion ax ay az aw))))))


(defgeneric get-object-type-fixed-frame-slice-down-transform (object-type arm grasp)
  (:documentation "Returns a transform stamped"))

(defmethod get-object-type-fixed-frame-slice-down-transform :around (object-type arm grasp)
  (destructuring-bind
      ((x y z) (ax ay az aw))
      (call-next-method)
    (cl-tf:transform->transform-stamped
     cram-tf:*fixed-frame*
     cram-tf:*fixed-frame*
     0.0
     (cl-tf:pose->transform
      (cl-transforms:make-pose
       (cl-transforms:make-3d-vector x y z)
       (cl-transforms:make-quaternion ax ay az aw))))))


(defgeneric get-object-type-fixed-frame-tilt-approach-transform (object-type arm grasp)
  (:documentation "Returns a transform stamped"))

(defmethod get-object-type-fixed-frame-tilt-approach-transform :around (object-type arm grasp)
  (destructuring-bind
      ((x y z) (ax ay az aw))
      (call-next-method)
    (cl-tf:transform->transform-stamped
     cram-tf:*fixed-frame*
     cram-tf:*fixed-frame*
     0.0
     (cl-tf:pose->transform
      (cl-transforms:make-pose
       (cl-transforms:make-3d-vector x y z)
       (cl-transforms:make-quaternion ax ay az aw))))))


(defgeneric get-object-type-to-gripper-2nd-pregrasp-transform (object-type object-name
                                                               arm grasp grasp-transform)
  (:documentation "Returns a transform stamped. Default value is NIL.")
  (:method (object-type object-name arm grasp grasp-transform)
    (call-with-specific-type #'get-object-type-to-gripper-2nd-pregrasp-transform
                             object-type object-name arm grasp grasp-transform)))

(defgeneric get-object-type-to-gripper-lift-transform (object-type object-name
                                                       arm grasp grasp-transform)
  (:documentation "Returns a transform stamped")
  (:method (object-type object-name arm grasp grasp-transform)
    (call-with-specific-type #'get-object-type-to-gripper-lift-transform
                             object-type object-name arm grasp grasp-transform)))

(defgeneric get-object-type-to-gripper-2nd-lift-transform (object-type object-name
                                                           arm grasp grasp-transform)
  (:documentation "Returns a transform stamped")
  (:method (object-type object-name arm grasp grasp-transform)
    (call-with-specific-type #'get-object-type-to-gripper-2nd-lift-transform
                             object-type object-name arm grasp grasp-transform)))


(defmacro def-object-type-to-gripper-transforms (object-type arm grasp-type
                                                 &key
                                                   (grasp-translation ''(0.0 0.0 0.0))
                                                   (grasp-rot-matrix ''((1.0 0.0 0.0)
                                                                        (0.0 1.0 0.0)
                                                                        (0.0 0.0 1.0)))
                                                   (pregrasp-offsets ''(0.0 0.0 0.0))
                                                   (2nd-pregrasp-offsets ''(0.0 0.0 0.0))
                                                   (lift-offsets ''(0.0 0.0 0.0))
                                                   (2nd-lift-offsets ''(0.0 0.0 0.0)))
  `(let ((evaled-object-type ,object-type)
         (evaled-arm ,arm)
         (evaled-grasp-type ,grasp-type)
         (evaled-grasp-translation ,grasp-translation)
         (evaled-grasp-rot-matrix ,grasp-rot-matrix)
         (evaled-pregrasp-offsets ,pregrasp-offsets)
         (evaled-2nd-pregrasp-offsets ,2nd-pregrasp-offsets)
         (evaled-lift-offsets ,lift-offsets)
         (evaled-2nd-lift-offsets ,2nd-lift-offsets))
     (let ((object-list
             (if (listp evaled-object-type)
                 evaled-object-type
                 (list evaled-object-type)))
           (arm-list
             (if (listp evaled-arm)
                 evaled-arm
                 (list evaled-arm))))
       (mapcar (lambda (object)
                 (mapcar (lambda (arm)
                           (let ((transform
                                   (cl-transforms-stamped:make-transform-stamped
                                    (roslisp-utilities:rosify-underscores-lisp-name object)
                                    (roslisp-utilities:rosify-underscores-lisp-name arm)
                                    0.0
                                    (cl-transforms:make-3d-vector
                                     (first evaled-grasp-translation)
                                     (second evaled-grasp-translation)
                                     (third evaled-grasp-translation))
                                    (cl-transforms:matrix->quaternion
                                     (make-array '(3 3)
                                                 :initial-contents evaled-grasp-rot-matrix)))))

                             (pushnew evaled-grasp-type *known-grasp-types*)

  (defmethod get-object-type-to-gripper-transform ((object-type (eql object))
                                                   object-name
                                                   (arm (eql arm))
                                                   (grasp (eql evaled-grasp-type)))
    (let ((grasp-transform transform))
      (if grasp-transform
          (cram-tf:copy-transform-stamped
           grasp-transform
           :frame-id (roslisp-utilities:rosify-underscores-lisp-name object-name)
           :child-frame-id (ecase arm
                             (:left cram-tf:*robot-left-tool-frame*)
                             (:right cram-tf:*robot-right-tool-frame*)))
          (error "Grasp transform not defined for object type ~a with arm ~a and grasp ~a~%"
                 object-type arm grasp))))

  (defmethod get-object-type-to-gripper-pregrasp-transform ((object-type (eql object))
                                                            object-name
                                                            (arm (eql arm))
                                                            (grasp (eql evaled-grasp-type))
                                                            grasp-transform)
    (let ((pregrasp-offsets evaled-pregrasp-offsets))
      (if pregrasp-offsets
          (destructuring-bind (x y z) pregrasp-offsets
            (cram-tf:translate-transform-stamped
             grasp-transform
             :x-offset x :y-offset y :z-offset z))
          (error "Pregrasp transform not defined for object type ~a with arm ~a and grasp ~a~%"
                 object-type arm grasp))))
                             
  (defmethod get-object-type-to-gripper-2nd-pregrasp-transform ((object-type (eql object))
                                                                object-name
                                                                (arm (eql arm))
                                                                (grasp (eql evaled-grasp-type))
                                                                grasp-transform)
    (let ((offsets evaled-2nd-pregrasp-offsets))
      (if offsets
          (destructuring-bind (x y z) offsets
            (cram-tf:translate-transform-stamped
             grasp-transform
             :x-offset x :y-offset y :z-offset z))
          (error "2nd pregrasp transform not defined for object type ~a with arm ~a and grasp ~a~%"
                 object-type arm grasp))))

  (defmethod get-object-type-to-gripper-lift-transform ((object-type (eql object))
                                                        object-name
                                                        (arm (eql arm))
                                                        (grasp (eql evaled-grasp-type))
                                                        grasp-transform)
    (let ((offsets evaled-lift-offsets))
      (if offsets
          (destructuring-bind (x y z) offsets
            (cram-tf:translate-transform-stamped
             grasp-transform
             :x-offset x :y-offset y :z-offset z))
          (error "Lift transform not defined for object type ~a with arm ~a and grasp ~a~%"
                 object-type arm grasp))))

  (defmethod get-object-type-to-gripper-2nd-lift-transform ((object-type (eql object))
                                                            object-name
                                                            (arm (eql arm))
                                                            (grasp (eql evaled-grasp-type))
                                                            grasp-transform)
    (let ((offsets evaled-2nd-lift-offsets))
      (if offsets
          (destructuring-bind (x y z) offsets
            (cram-tf:translate-transform-stamped
             grasp-transform
             :x-offset x :y-offset y :z-offset z))
          (error "2nd lift transform not defined for object type ~a with arm ~a and grasp ~a~%"
                 object-type arm grasp))))

                             ))
                         arm-list))
               object-list))))



(defmethod get-action-trajectory :heuristics 20 ((action-type (eql :picking-up))
                                                 arm
                                                 grasp
                                                 objects-acted-on
                                                 &key)
  (let* ((object
           (car objects-acted-on))
         (object-name
           (desig:desig-prop-value object :name))
         (object-type
           (desig:desig-prop-value object :type))
         (bTo
           (man-int:get-object-transform object))
         (oTg-std
           (man-int:get-object-type-to-gripper-transform
            object-type object-name arm grasp)))

    (mapcar (lambda (label transforms)
              (make-traj-segment
               :label label
               :poses (mapcar (alexandria:curry #'calculate-gripper-pose-in-map bTo arm)
                              transforms)))
            '(:reaching
              :grasping
              :lifting)
            `((,(man-int:get-object-type-to-gripper-pregrasp-transform
                 object-type object-name arm grasp oTg-std)
               ,(man-int:get-object-type-to-gripper-2nd-pregrasp-transform
                 object-type object-name arm grasp oTg-std))
              (,oTg-std)
              (,(man-int:get-object-type-to-gripper-lift-transform
                 object-type object-name arm grasp oTg-std)
               ,(man-int:get-object-type-to-gripper-2nd-lift-transform
                 object-type object-name arm grasp oTg-std))))))

(defmethod get-action-trajectory :heuristics 20 ((action-type (eql :placing))
                                                 arm
                                                 grasp
                                                 objects-acted-on
                                                 &key target-object-transform-in-base)
  (let* ((object
           (car objects-acted-on))
         (object-name
           (desig:desig-prop-value object :name))
         (object-type
           (desig:desig-prop-value object :type))
         (oTg-std
           (get-object-type-to-gripper-transform
            object-type object-name arm grasp)))

    (mapcar (lambda (label transforms)
              (make-traj-segment
               :label label
               :poses (mapcar (alexandria:curry #'calculate-gripper-pose-in-map
                                                target-object-transform-in-base arm)
                              transforms)))
            '(:reaching
              :putting
              :retracting)
            `((,(man-int:get-object-type-to-gripper-2nd-lift-transform
                 object-type object-name arm grasp oTg-std)
               ,(man-int:get-object-type-to-gripper-lift-transform
                 object-type object-name arm grasp oTg-std))
              (,oTg-std)
              (,(man-int:get-object-type-to-gripper-2nd-pregrasp-transform
                 object-type object-name arm grasp oTg-std)
               ,(man-int:get-object-type-to-gripper-pregrasp-transform
                 object-type object-name arm grasp oTg-std))))))


(defun get-tilting-poses (grasp approach-poses &optional (angle (cram-math:degrees->radians 100)))
  (mapcar (lambda (?approach-pose)
            ;;depending on the grasp the angle to tilt is different
            (case grasp
              (:front (rotate-once-pose ?approach-pose (- angle) :y))
              (:left-side (rotate-once-pose ?approach-pose (+ angle) :x))
              (:right-side (rotate-once-pose ?approach-pose (- angle) :x))
              (:back (rotate-once-pose ?approach-pose (+ angle) :y))
              (t (error "can only pour from :side, back or :front"))))
          approach-poses))

;;helper function for tilting
;;rotate the pose around the axis in an angle
(defun rotate-once-pose (pose angle axis)
  (cl-transforms-stamped:copy-pose-stamped
   pose
   :orientation (let ((pose-orientation (cl-transforms:orientation pose)))
                  (cl-transforms:q*
                   (cl-transforms:axis-angle->quaternion
                    (case axis
                      (:x (cl-transforms:make-3d-vector 1 0 0))
                      (:y (cl-transforms:make-3d-vector 0 1 0))
                      (:z (cl-transforms:make-3d-vector 0 0 1))
                      (t (error "in ROTATE-ONCE-POSE forgot to specify axis properly: ~a" axis)))
                    angle)
                   pose-orientation))))

;;get pouring trajectory workes like picking-up it will get the 
;;object-type-to-gripper-tilt-approch-transform und makes a traj-segment out of it
;;here we have only the approach pose, followed by that is the titing pose (above)
(defmethod get-action-trajectory :heuristics 20 ((action-type (eql :pouring))
                                                 arm
                                                 grasp
                                                 objects-acted-on
                                                 &key )
  (let* ((object
           (car objects-acted-on))
         (object-name
           (desig:desig-prop-value object :name))
         (object-type
           (desig:desig-prop-value object :type))
         (bTo
           (man-int:get-object-transform object))
         (oTg-std
           (man-int:get-object-type-to-gripper-transform
            object-type object-name arm grasp))
         (approach-pose
           (let* ((tmp-pose-stmp (funcall
                                  (alexandria:curry #'calculate-gripper-pose-in-map bTo arm)
                                  oTg-std))
                  (transform (cram-tf:apply-transform
                              (man-int::get-object-type-fixed-frame-tilt-approach-transform
                               object-type arm grasp)
                              (cl-tf:make-transform-stamped
                               cram-tf:*fixed-frame*
                               cram-tf:*fixed-frame*
                               0.0
                               (cl-tf:origin tmp-pose-stmp)
                               (cl-tf:orientation tmp-pose-stmp)))))
             (cl-tf:make-pose-stamped 
               cram-tf:*fixed-frame*
               0.0
               (cl-tf:translation transform)
               (cl-tf:rotation transform))))
         (tilting-poses
           (get-tilting-poses grasp (list approach-pose))))
    (mapcar (lambda (label poses)
              (make-traj-segment
               :label label
               :poses poses))
            '(:approach
              :tilting)
            `((,approach-pose)
              ,tilting-poses))))


;;;;;;;;;;;;;;;;;;; OBJECT TO OTHER OBJECT TRANSFORMS ;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar *known-attachment-types* nil
  "A list of symbols representing all known attachment types")

(defgeneric get-object-type-in-other-object-transform (object-type object-name
                                                       other-object-type other-object-name
                                                       attachment))

(defun get-object-placement-transform (object-name object-type
                                       other-object-name other-object-type other-object-transform
                                       attachment-type)
  "Returns a transform in robot base frame where the object named `object-name' should go"
  (let* ((base-frame
           cram-tf:*robot-base-frame*)
         (object-frame
           (roslisp-utilities:rosify-underscores-lisp-name object-name))
         (base-to-object-transform  ; bTo = bToo * ooTo
           (cram-tf:multiply-transform-stampeds
            base-frame
            object-frame
            other-object-transform
            (get-object-type-in-other-object-transform ; ooTo
             object-type object-name other-object-type other-object-name attachment-type))))
    base-to-object-transform))



(defmacro def-object-type-in-other-object-transform (object-type other-object-type attachment-type
                                                     &key
                                                       (attachment-translation ''(0.0 0.0 0.0))
                                                       (attachment-rot-matrix ''((1.0 0.0 0.0)
                                                                                 (0.0 1.0 0.0)
                                                                                 (0.0 0.0 1.0))))
  `(let ((evaled-object-type ,object-type)
         (evaled-other-object-type ,other-object-type)
         (evaled-attachment-type ,attachment-type)
         (evaled-attachment-translation ,attachment-translation)
         (evaled-attachment-rot-matrix ,attachment-rot-matrix))
     (let ((object-list
             (if (listp evaled-object-type)
                 evaled-object-type
                 (list evaled-object-type)))
           (other-object-list
             (if (listp evaled-other-object-type)
                 evaled-other-object-type
                 (list evaled-other-object-type))))
       (mapcar (lambda (object)
                 (mapcar (lambda (other-object)
                           (let ((transform
                                   (cl-transforms-stamped:make-transform-stamped
                                    (roslisp-utilities:rosify-underscores-lisp-name other-object)
                                    (roslisp-utilities:rosify-underscores-lisp-name object)
                                    0.0
                                    (cl-transforms:make-3d-vector
                                     (first evaled-attachment-translation)
                                     (second evaled-attachment-translation)
                                     (third evaled-attachment-translation))
                                    (cl-transforms:matrix->quaternion
                                     (make-array '(3 3)
                                                 :initial-contents
                                                 evaled-attachment-rot-matrix)))))

                             (pushnew evaled-attachment-type *known-attachment-types*)

   (defmethod get-object-type-in-other-object-transform ((object-type (eql object))
                                                         object-name
                                                         (other-object-type (eql other-object))
                                                         other-object-name
                                                         (attachment (eql evaled-attachment-type)))
         (let ((attachment-transform transform))
           (if attachment-transform
               (cram-tf:copy-transform-stamped
                attachment-transform
                :frame-id (roslisp-utilities:rosify-underscores-lisp-name other-object-name)
                :child-frame-id (roslisp-utilities:rosify-underscores-lisp-name object-name))
               (error "Attachment transform not defined for ~a with ~a attached with ~a~%"
                      object other-object attachment))))

                             ))
                         other-object-list))
               object-list))))



;;;;;;;;;;;;;;;;;;; CAMERA TO OBJECT TRANSFORMS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *default-look-z-offset* 0.2 "in meters")
(defparameter *default-look-x-offset* 0.15 "in meters")

(defun get-object-look-from-pose (object-transform)
  (declare (type cl-transforms-stamped:transform-stamped object-transform))
  "Returns a pose stamped representing bTg -- transfrom from base to gripper.
Used for generating a pose for a hand-mounted camera.
Take object-transform, ensure it's from base frame  -- bTo.
Take an initial pose of gripper in base bTg and set its X and Y to object X and Y.
Set Z to offset object Z."

  (unless (equal (cl-transforms-stamped:frame-id object-transform)
                 cram-tf:*robot-base-frame*)
    (error "In grasp calculations the OBJECT-TRANSFORM did not have ~
correct parent frame: ~a and ~a"
           (cl-transforms-stamped:frame-id object-transform)
           cram-tf:*robot-base-frame*))

  (let* ((gripper-initial-pose
           (cl-transforms-stamped:make-pose-stamped
            cram-tf:*robot-base-frame*
            0.0
            (cl-transforms:make-3d-vector 0 0 0)
            (cl-transforms:matrix->quaternion
             #2A((-1 0 0)
                 (0 1 0)
                 (0 0 -1)))))
         (object-x-in-base (cl-transforms:x (cl-transforms:translation object-transform)))
         (object-y-in-base (cl-transforms:y (cl-transforms:translation object-transform)))
         (object-z-in-base (cl-transforms:z (cl-transforms:translation object-transform)))
         (offset-object-x (- object-x-in-base *default-look-x-offset*))
         (offset-object-z (+ object-z-in-base *default-look-z-offset*)))
    (cl-transforms-stamped:copy-pose-stamped
     gripper-initial-pose
     :origin (cl-transforms:make-3d-vector offset-object-x object-y-in-base offset-object-z))))
