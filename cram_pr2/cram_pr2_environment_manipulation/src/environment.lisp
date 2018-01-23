(in-package :pr2-em)

(defun get-urdf-link-pose (name)
  (btr:pose (btr:rigid-body (btr:object btr:*current-bullet-world* :kitchen) (btr::make-rigid-body-name "KITCHEN" name :pr2-em))))

(defun get-container-link (container-name)
  (gethash container-name (cl-urdf:links (btr:urdf (btr:object btr:*current-bullet-world* :kitchen)))))

(defun get-container-joint-type (container-name)
  (find-container-joint-type-under-joint (cl-urdf:from-joint (get-container-link container-name))))

(defun find-container-joint-type-under-joint (joint)
  "Return the first joint type different from :FIXED under the given JOINT."
  (if (eq :FIXED (cl-urdf:joint-type joint))
      (find-container-joint-type-under-joint (car (cl-urdf:to-joints
                                                   (cl-urdf:child joint))))
      (cl-urdf:joint-type joint)))

(defun get-handle-link (container-name)
  (when (symbolp container-name)
    (setf container-name (string-downcase container-name)))
  (find-handle-under-link
   (get-container-link container-name)))

(defun find-handle-under-link (link)
  (if (search "handle" (cl-urdf:name link))
      link
      (find-handle-under-link (cl-urdf:child
                     (car (cl-urdf:to-joints link))))))

(defun get-joint-position (joint)
  (gethash (cl-urdf:name joint)
           (btr:joint-states (btr:object btr:*current-bullet-world* :kitchen))))

(defun get-connecting-joint (part)
  "Returns the connecting (moveable) joint of `part', which can be either
  a link or a joint of the kitchen URDF."
  (when part
    (if (typep part 'cl-urdf:joint)
        (or
         (when (not (eql (cl-urdf:joint-type part) :FIXED))
           part)
         (get-connecting-joint (cl-urdf:parent part)))
        (when (typep part 'cl-urdf:link)
          (get-connecting-joint (cl-urdf:from-joint part))))))

(defun get-manipulated-pose (link-name joint-position &key relative)
  "Returns the pose of a link based on its connection joint position
  `joint-position'. If `relative' is T, the actual value is calculated
  by `joint-position' * <joint maximal value>. this method returns two
  values, the new pose of the object and the joint that was changed."
  (let ((link (get-container-link link-name)))
    (when (typep link 'cl-urdf:link)
      (let ((joint (get-connecting-joint link)))
        (when joint
          (values
           (case (cl-urdf:joint-type joint)
             (:PRISMATIC
              (cl-tf:transform->pose
               (cl-tf:transform*
                (cl-tf:pose->transform (get-urdf-link-pose link-name))
                (cl-tf:make-transform
                 (cl-tf:v*
                  (cl-urdf:axis joint)
                  (-
                   (if relative
                       (* joint-position
                          (cl-urdf:upper (cl-urdf:limits joint)))
                       joint-position)
                   (get-joint-position joint)))
                 (cl-tf:make-identity-rotation)))))
             (:REVOLUTE (error 'simple-error :format-control "Manipulation of revolute joints not implemented.")))
           joint))))))
