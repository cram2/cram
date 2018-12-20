;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

;;;;;;;;;;;;;;;;;;; OBJECT TO GRIPPER TRANSFORMS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar *known-grasp-types* nil
  "A list of symbols representing all known grasp types")

(defstruct traj-segment
  (label nil :type keyword)
  ;;(collision-mode :allow-all :type keyword)
  (poses nil :type list))

(defun make-empty-trajectory (labels)
  (mapcar
   (lambda (x)
     (make-traj-segment :label x :poses nil))
   labels))

(defun get-traj-poses-by-label (trajectory label)
  (traj-segment-poses
   (find label
         trajectory
         :key #'traj-segment-label)))

(defgeneric get-action-trajectory (action-type arm grasp objects-acted-on
                                   &key &allow-other-keys)
  (:documentation "Returns a list of traj-segments.
`action-type' describes for which type of action the trajectory will be.
`arm' a single keyword eg. :left
`grasp' describes which type of grasp should be used.
`objects-acted-on' designators describing the objects used by the action."))

(defgeneric get-object-type-gripping-effort (object-type)
  (:documentation "Returns effort in Nm, e.g. 50."))

(defgeneric get-object-type-gripper-opening (object-type)
  (:documentation "How wide to open the gripper before grasping, in m."))

(defun make-object-to-standard-gripper->base-to-particular-gripper-transformer
    (object-transform arm)
  "Make a function that transforms oTg' -> bTg; Assuming g' is standard gripper."
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
                          ?robot ?transform))))))))
  (lambda (object-to-standard-gripper)
    (when object-to-standard-gripper
      (let ((base-to-standard-gripper-transform
              (cram-tf:multiply-transform-stampeds
               cram-tf:*robot-base-frame* gripper-tool-frame
               object-transform          ; bTo
               object-to-standard-gripper ; oTg'
               :result-as-pose-or-transform :transform))) ; bTo * oTg' = bTg'
        (cram-tf:multiply-transform-stampeds ; bTg' * g'Tg = bTg
         cram-tf:*robot-base-frame* gripper-tool-frame
         base-to-standard-gripper-transform      ; bTg'
         standard-to-particular-gripper-transform ; g'Tg
         :result-as-pose-or-transform :pose))))))

;;;;;;;;;;;;;;;; Everything below is for pick and place (only) ;;;;;;;;;;;;;;;;;;

(defgeneric get-object-type-to-gripper-transform (object-type object-name arm grasp)
  (:documentation "Returns a pose stamped.
Gripper is defined by a convention where Z is pointing towards the object."))

(defgeneric get-object-type-to-gripper-pregrasp-transform (object-type object-name
                                                           arm grasp grasp-transform)
  (:documentation "Returns a transform stamped"))

(defgeneric get-object-type-to-gripper-2nd-pregrasp-transform (object-type object-name
                                                               arm grasp grasp-transform)
  (:documentation "Returns a transform stamped. Default value is NIL."))

(defgeneric get-object-type-to-gripper-lift-transform (object-type object-name
                                                       arm grasp grasp-transform)
  (:documentation "Returns a transform stamped"))

(defgeneric get-object-type-to-gripper-2nd-lift-transform (object-type object-name
                                                           arm grasp grasp-transform)
  (:documentation "Returns a transform stamped"))


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
          (error "Second pregrasp transform not defined for object type ~a with arm ~a and grasp ~a~%"
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
          (error "Second lift transform not defined for object type ~a with arm ~a and grasp ~a~%"
                 object-type arm grasp))))

                             ))
                         arm-list))
               object-list))))


;; (defgeneric get-object-grasping-poses (object-name object-type arm grasp object-transform)
;;   (:documentation "Returns a list of (pregrasp-pose 2nd-pregrasp-pose grasp-pose lift-pose)")
;;   (:method (object-name object-type arm grasp object-transform)
;;     (declare (type symbol object-name object-type arm grasp)
;;              (type cl-transforms-stamped:transform-stamped object-transform))

;;     ;; First correct the object transform such that rotationally-symmetric objects
;;     ;; would not be grasped in an awkward way with weird orientations
;;     (when (prolog `(object-rotationally-symmetric ,object-type))
;;       (setf object-transform
;;             (cram-tf:copy-transform-stamped
;;              object-transform
;;              :rotation (cl-transforms:make-identity-rotation))))

;;     (let* ((gripper-tool-frame
;;              (ecase arm
;;                (:left cram-tf:*robot-left-tool-frame*)
;;                (:right cram-tf:*robot-right-tool-frame*)))
;;            (object-to-standard-gripper-transform ; oTg'
;;              (get-object-type-to-gripper-transform
;;               (find-most-specific-object-type-for-generic
;;                #'get-object-type-to-gripper-transform object-type)
;;               object-name arm grasp))
;;            (object-to-standard-gripper-pregrasp-transform ; oTg'
;;              (get-object-type-to-gripper-pregrasp-transform
;;               (find-most-specific-object-type-for-generic
;;                #'get-object-type-to-gripper-transform object-type)
;;               object-name arm grasp
;;               object-to-standard-gripper-transform))
;;            (object-to-standard-gripper-2nd-pregrasp-transform ; oTg'
;;              (get-object-type-to-gripper-2nd-pregrasp-transform
;;               (find-most-specific-object-type-for-generic
;;                #'get-object-type-to-gripper-transform object-type)
;;               object-name arm grasp
;;               object-to-standard-gripper-transform))
;;            (object-to-standard-gripper-lift-transform ; oTg'
;;              (get-object-type-to-gripper-lift-transform
;;               (find-most-specific-object-type-for-generic
;;                #'get-object-type-to-gripper-transform object-type)
;;               object-name arm grasp
;;               object-to-standard-gripper-transform))
;;            (object-to-standard-gripper-2nd-lift-transform ; oTg'
;;              (get-object-type-to-gripper-2nd-lift-transform
;;               (find-most-specific-object-type-for-generic
;;                #'get-object-type-to-gripper-transform object-type)
;;               object-name arm grasp
;;               object-to-standard-gripper-transform))
;;            (standard-to-particular-gripper-transform ; g'Tg
;;              (cl-transforms-stamped:transform->transform-stamped
;;               gripper-tool-frame
;;               gripper-tool-frame
;;               0.0
;;               (cut:var-value
;;                '?transform
;;                (car (prolog:prolog
;;                      `(and (cram-robot-interfaces:robot ?robot)
;;                            (cram-robot-interfaces:standard-to-particular-gripper-transform
;;                             ?robot ?transform))))))))
;;       (when (and object-to-standard-gripper-transform standard-to-particular-gripper-transform)

;;         (flet ((object-to-standard-gripper->base-to-particular-gripper (object-to-standard-gripper)
;;                  (when object-to-standard-gripper
;;                    (let ((base-to-standard-gripper-transform
;;                            (cram-tf:multiply-transform-stampeds
;;                             cram-tf:*robot-base-frame* gripper-tool-frame
;;                             object-transform          ; bTo
;;                             object-to-standard-gripper ; oTg'
;;                             :result-as-pose-or-transform :transform))) ; bTo * oTg' = bTg'
;;                      (cram-tf:multiply-transform-stampeds ; bTg' * g'Tg = bTg
;;                       cram-tf:*robot-base-frame* gripper-tool-frame
;;                       base-to-standard-gripper-transform      ; bTg'
;;                       standard-to-particular-gripper-transform ; g'Tg
;;                       :result-as-pose-or-transform :pose)))))
          
;;           (mapcar #'object-to-standard-gripper->base-to-particular-gripper
;;                   (list object-to-standard-gripper-pregrasp-transform
;;                         object-to-standard-gripper-2nd-pregrasp-transform
;;                         object-to-standard-gripper-transform
;;                         object-to-standard-gripper-lift-transform
;;                         object-to-standard-gripper-2nd-lift-transform)))))))

;;;;;;;;;;;;;;;;;;;;;;; Pick and Place specific stuff ends here. ;;;;;;;;;;;;;;;;;;;;;

(def-fact-group object-knowledge (object-rotationally-symmetric orientation-matters ;; object-type-grasp
                                                                )

  (<- (object-rotationally-symmetric ?object-type)
    (fail))

  ;; The predicate ORIENTATION-MATTERS holds for all objects where the
  ;; orientation really matters when putting down the object. E.g. for
  ;; knives, forks, etc, the orientation is important while for plates
  ;; the orientation doesn't matter at all.
  (<- (orientation-matters ?object-type-symbol)
    (fail))

  ;; (<- (object-type-grasp ?object-type ?grasp)
  ;;   (fail))
  )

#+sbcl
;; TODO(cpo): I think this needs work. This only works when obejct-type
;;            is the first specializer of the method.
;; Which might be okay, because the interface dictates in what order the specializers are...
(defun probe-sbcl (generic object-type)
  (let ((gfms (sb-pcl:generic-function-methods generic)))
    (find object-type gfms
          :key (lambda (x)
                 (car (sb-pcl:method-specializers x)))
          :test (lambda (x y)
                  (when (eql (type-of y) 'sb-mop:eql-specializer)
                    (eql
                     (sb-mop:eql-specializer-object y)
                     x))))))
#-sbcl
(error "This version of cram requires the use of the SBCL compiler.")

(defun get-direct-supertypes (object-type)
  (mapcar
   (lambda (x)
     (cut:with-vars-bound (?super) x ?super))
   (cut:force-ll
    (prolog `(object-type-direct-subtype ?super ,object-type)))))

(defun find-most-specific-object-type-for-generic (generic object-type)
  "Find the most specific method of `generic' based on `object-type'."
  (if (probe-sbcl generic object-type)
      object-type
      (car (mapcar
            (alexandria:curry #'find-most-specific-object-type-for-generic generic)
            (get-direct-supertypes object-type)))))


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
