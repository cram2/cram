;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;                     Christopher Pollok <cpollok@cs.uni-bremen.de>
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



;;;;;;;;;;;;;;;; Everything below is for pick and place only ;;;;;;;;;;;;;;;;;;

(defvar *known-grasp-types* nil
  "A list of symbols representing all known grasp types")

(defgeneric get-object-type-to-gripper-transform (object-type object-name arm grasp)
  (:documentation "Returns a pose stamped.
Gripper is defined by a convention where Z is pointing towards the object.")
  (:method (object-type object-name arm grasp)
    (call-with-specific-type #'get-object-type-to-gripper-transform
                             object-type object-name arm grasp)))

(defgeneric get-object-type-to-gripper-pregrasp-transforms (object-type object-name
                                                            arm grasp grasp-transform)
  (:documentation "Returns a list of transform stampeds")
  (:method (object-type object-name arm grasp grasp-transform)
    (call-with-specific-type #'get-object-type-to-gripper-pregrasp-transforms
                             object-type object-name arm grasp grasp-transform)))

(defgeneric get-object-type-wrt-base-frame-lift-transforms (object-type arm grasp)
  (:documentation "Returns a list of transform stampeds bTb representing the
lift and 2nd-lift offset of given `object-type' with `arm' and `grasp'
in `cram-tf:*robot-base-frame*'. Therefore, instantiated methods will
specify how much an object with given `object-type' will be lifted
up in meters after grasping it, where the offset is defined w.r.t. base frame.")
  (:method (object-type arm grasp)
    (call-with-specific-type #'get-object-type-wrt-base-frame-lift-transforms
                             object-type arm grasp)))



(defmacro def-object-type-to-gripper-transforms (object-type arm grasp-type
                                                 &key
                                                   (grasp-translation
                                                    ''(0.0 0.0 0.0))
                                                   (grasp-rot-matrix
                                                    ''((1.0 0.0 0.0)
                                                       (0.0 1.0 0.0)
                                                       (0.0 0.0 1.0)))
                                                   (pregrasp-offsets
                                                    ''(0.0 0.0 0.0))
                                                   (2nd-pregrasp-offsets
                                                    ''(0.0 0.0 0.0))
                                                   (lift-translation
                                                    ''(0.0 0.0 0.0))
                                                   (lift-rotation
                                                    ''(0.0 0.0 0.0 1.0))
                                                   (2nd-lift-translation
                                                    ''(0.0 0.0 0.0))
                                                   (2nd-lift-rotation
                                                    ''(0.0 0.0 0.0 1.0)))
  "`grasp-translation' is the translation part of oTg-std,
   `grasp-rot-matrix' is the rotation part, represented as a rotation matrix 2x2 list of lists,
   `pregrasp-offsets' is a list of 3 values, which is the offset of the gripper in object frame,
   `2nd-pregrasp-offsets' is the same as pregrasp offsets, for picking it goes after pregrasp,
   `lift-translation' and `lift-rotation' are the transform of the gripper in robot base frame,
   `2nd-lift-translation' and `2nd-lift-rotation' are similar, they go after lift in picking."
  `(let ((evaled-object-type ,object-type)
         (evaled-arm ,arm)
         (evaled-grasp-type ,grasp-type)
         (evaled-grasp-translation ,grasp-translation)
         (evaled-grasp-rot-matrix ,grasp-rot-matrix)
         (evaled-pregrasp-offsets ,pregrasp-offsets)
         (evaled-2nd-pregrasp-offsets ,2nd-pregrasp-offsets)
         (evaled-lift-translation ,lift-translation)
         (evaled-lift-rotation ,lift-rotation)
         (evaled-2nd-lift-translation ,2nd-lift-translation)
         (evaled-2nd-lift-rotation ,2nd-lift-rotation))
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
                                                 :initial-contents evaled-grasp-rot-matrix))))
                                 (lift-transform
                                   (cl-transforms-stamped:make-transform
                                    (cl-transforms:make-3d-vector
                                     (first evaled-lift-translation)
                                     (second evaled-lift-translation)
                                     (third evaled-lift-translation))
                                    (cl-transforms:make-quaternion
                                     (first evaled-lift-rotation)
                                     (second evaled-lift-rotation)
                                     (third evaled-lift-rotation)
                                     (fourth evaled-lift-rotation))))
                                 (2nd-lift-transform
                                   (cl-transforms-stamped:make-transform
                                    (cl-transforms:make-3d-vector
                                     (first evaled-2nd-lift-translation)
                                     (second evaled-2nd-lift-translation)
                                     (third evaled-2nd-lift-translation))
                                    (cl-transforms:make-quaternion
                                     (first evaled-2nd-lift-rotation)
                                     (second evaled-2nd-lift-rotation)
                                     (third evaled-2nd-lift-rotation)
                                     (fourth evaled-2nd-lift-rotation)))))

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

  (defmethod get-object-type-to-gripper-pregrasp-transforms ((object-type (eql object))
                                                             object-name
                                                             (arm (eql arm))
                                                             (grasp (eql evaled-grasp-type))
                                                             grasp-transform)
    (let ((pregrasp-offsets evaled-pregrasp-offsets)
          (2nd-pregrasp-offsets evaled-2nd-pregrasp-offsets))
      (if (and pregrasp-offsets 2nd-pregrasp-offsets)
          (list
           (destructuring-bind (x y z) pregrasp-offsets
             (cram-tf:translate-transform-stamped
              grasp-transform
              :x-offset x :y-offset y :z-offset z))
           (destructuring-bind (x y z) 2nd-pregrasp-offsets
             (cram-tf:translate-transform-stamped
              grasp-transform
              :x-offset x :y-offset y :z-offset z)))
          (error "Pregrasp transforms not defined for object type ~a with arm ~a and grasp ~a~%"
                 object-type arm grasp))))

  (defmethod get-object-type-wrt-base-frame-lift-transforms ((object-type (eql object))
                                                             (arm (eql arm))
                                                             (grasp (eql evaled-grasp-type)))
    (let ((this-lift-transform lift-transform)
          (this-2nd-lift-transform 2nd-lift-transform))
      (if (and this-lift-transform this-2nd-lift-transform)
          (list
           (cl-transforms-stamped:transform->transform-stamped
            cram-tf:*robot-base-frame* cram-tf:*robot-base-frame* 0.0
            this-lift-transform)
           (cl-transforms-stamped:transform->transform-stamped
            cram-tf:*robot-base-frame* cram-tf:*robot-base-frame* 0.0
            this-2nd-lift-transform))
          (error "Lift transforms w.r.t. base frame not defined ~
                  for object type ~a with arm ~a and grasp ~a~%"
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
         (oTg-std
           (get-object-type-to-gripper-transform
            object-type object-name arm grasp))
         (bTo
           (man-int:get-object-transform object))
         (oTb
           (cram-tf:transform-stamped-inv bTo))
         (bTb-lift
           (first (get-object-type-wrt-base-frame-lift-transforms
                   object-type arm grasp)))
         (bTb-2ndlift
           (second (get-object-type-wrt-base-frame-lift-transforms
                    object-type arm grasp)))
         (oTg-lift
           (reduce #'cram-tf:apply-transform
                   `(,oTb ,bTb-lift ,bTo ,oTg-std)
                   :from-end T))
         (oTg-2ndlift
           (reduce #'cram-tf:apply-transform
                   `(,oTb ,bTb-2ndlift ,bTo ,oTg-std)
                   :from-end T))
         (oTg-pregrasp
           (first (get-object-type-to-gripper-pregrasp-transforms
                   object-type object-name arm grasp oTg-std)))
         (oTg-2ndpregrasp
           (second (get-object-type-to-gripper-pregrasp-transforms
                    object-type object-name arm grasp oTg-std))))
    
    (mapcar (lambda (label transforms)
              (make-traj-segment
               :label label
               :poses (mapcar (alexandria:curry #'calculate-gripper-pose-in-map bTo arm)
                              transforms)))
            '(:reaching
              :grasping
              :lifting)
            `((,oTg-pregrasp ,oTg-2ndpregrasp)
              (,oTg-std)
              (,oTg-lift ,oTg-2ndlift)))))

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
            object-type object-name arm grasp))
         (bTo
           target-object-transform-in-base)
         (oTb
           (cram-tf:transform-stamped-inv bTo))
         (bTb-lift
           (first (get-object-type-wrt-base-frame-lift-transforms
                   object-type arm grasp)))
         (bTb-2ndlift
           (second (get-object-type-wrt-base-frame-lift-transforms
                    object-type arm grasp)))
         (oTg-lift
           (reduce #'cram-tf:apply-transform
                   `(,oTb ,bTb-lift ,bTo ,oTg-std)
                   :from-end T))
         (oTg-2ndlift
           (reduce #'cram-tf:apply-transform
                   `(,oTb ,bTb-2ndlift ,bTo ,oTg-std)
                   :from-end T))
         (oTg-pregrasp
           (first (get-object-type-to-gripper-pregrasp-transforms
                   object-type object-name arm grasp oTg-std)))
         (oTg-2ndpregrasp
           (second (get-object-type-to-gripper-pregrasp-transforms
                    object-type object-name arm grasp oTg-std))))

    (mapcar (lambda (label transforms)
              (make-traj-segment
               :label label
               :poses (mapcar (alexandria:curry #'calculate-gripper-pose-in-map
                                                target-object-transform-in-base arm)
                              transforms)))
            '(:reaching
              :putting
              :retracting)
            `((,oTg-2ndlift ,otg-lift)
              (,oTg-std)
              (,oTg-2ndpregrasp ,oTg-pregrasp)))))



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
