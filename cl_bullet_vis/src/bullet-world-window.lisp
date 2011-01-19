
(in-package :bt-vis)

(defclass bullet-world-window (glut:window gl-context)
  ((world :accessor world :initarg :world
          :initform (error 'simple-error :format-control "world argument required"))
   (motion-mode :initform nil :reader motion-mode)
   (pointer-pos :initform nil :reader pointer-pos))
  (:default-initargs :width 640 :height 480 :title "glut-teapot.lisp"
    :mode '(:double :rgba :depth)))

(defmethod glut:display-window :before ((w bullet-world-window))
  (gl:clear-color 0 0 0 0)
  (gl:cull-face :back)
  (gl:depth-func :lequal)
  (gl:shade-model :smooth)
  ;; (gl:light-model :light-model-local-viewer 1)
  ;; (gl:light-model :light-model-ambient #(0.5 0.5 0.5 1))
  (gl:enable :light0 :lighting :cull-face :depth-test :color-material :blend)
  (%gl:blend-func :src-alpha :one-minus-src-alpha)
  (gl:hint :perspective-correction-hint :nicest))

(defmethod glut:display ((window bullet-world-window))
  (gl:load-identity)
  ;; Frst we rotate our coordinate system such that x and y are along
  ;; the ground plane and z is up:
  ;;  z
  ;;  |      x
  ;;  |   /- 
  ;;  | /-
  ;;  +-------y
  ;;
  (gl:enable :light0 :lighting :cull-face :depth-test :color-material :blend)
  (gl:rotate 90 1 0 0)
  (gl:rotate -90 0 0 1)
  (gl:rotate 180 1 0 0)
  (gl:clear :color-buffer :depth-buffer)
  (set-camera (camera-transform window))
  (gl:light :light0 :position (vector
                               (cl-transforms:x (light-position window))
                               (cl-transforms:y (light-position window))
                               (cl-transforms:z (light-position window))
                               0))
  (gl:light :light0 :ambient #(0 0 0 1))
  (gl:light :light0 :diffuse #(1 1 1 1))
  (gl:light :light0 :specular #(1 1 1 1))
  (gl:with-pushed-matrix
    (draw-world window (world window)))
  (glut:swap-buffers)
  (gl:flush))

(defmethod glut:reshape ((window bullet-world-window) width height)
  (gl:viewport 0 0 width height)
  (gl:matrix-mode :projection)
  (gl:load-identity)
  (glu:perspective 50 (/ width height) 0.5 100)
  (gl:matrix-mode :modelview)
  (gl:load-identity))

(defmethod glut:keyboard ((window bullet-world-window) key x y)
  (declare (ignore x y))
  (when (eql key #\Esc)
    (glut:destroy-current-window)))

(defmethod glut:mouse ((window bullet-world-window) button state x y)
  (with-slots (motion-mode pointer-pos camera-transform) window
    (case state
      (:up (setf motion-mode nil))
      (:down (unless motion-mode
               (setf pointer-pos `(,x ,y))
               (case button
                 (:left-button (setf motion-mode :rotate))
                 (:right-button (setf motion-mode :translate))
                 (:wheel-up (setf camera-transform (cl-transforms:transform*
                                                    camera-transform
                                                    (cl-transforms:make-transform
                                                     (cl-transforms:make-3d-vector 0.3 0 0)
                                                     (cl-transforms:make-quaternion 0 0 0 1)))))
                 (:wheel-down (setf camera-transform (cl-transforms:transform*
                                                      camera-transform
                                                      (cl-transforms:make-transform
                                                       (cl-transforms:make-3d-vector -0.3 0 0)
                                                       (cl-transforms:make-quaternion 0 0 0 1))))))))))
  (glut:post-redisplay))

(defmethod glut:motion ((window bullet-world-window) x y)
  (with-slots (motion-mode camera-transform pointer-pos) window
    (unless pointer-pos
      (return-from glut:motion nil))
    (destructuring-bind (old-x old-y) pointer-pos
      (let ((dx (/ (- x old-x) (glut:width window)))
            (dy (/ (- y old-y) (glut:height window)))
            (y-axis-in-world (cl-transforms:rotate (cl-transforms:rotation camera-transform)
                                                   (cl-transforms:make-3d-vector 0 1 0))))
        (setf pointer-pos `(,x ,y))
        (case motion-mode
          (:rotate
             (setf camera-transform (cl-transforms:transform* (cl-transforms:make-transform
                                                               (cl-transforms:make-3d-vector 0 0 0)
                                                               (cl-transforms:axis-angle->quaternion
                                                                (cl-transforms:make-3d-vector 0 0 1)
                                                                (* dx pi)))
                                                              (cl-transforms:make-transform
                                                               (cl-transforms:make-3d-vector 0 0 0)
                                                               (cl-transforms:axis-angle->quaternion
                                                                y-axis-in-world
                                                                (* dy pi)))                                                              
                                                              camera-transform)))
          (:translate
             (setf camera-transform (cl-transforms:transform* camera-transform
                                                              (cl-transforms:make-transform
                                                               (cl-transforms:make-3d-vector 0 (* dx 10) (* dy 10))
                                                               (cl-transforms:make-quaternion 0 0 0 1)))))))))
  (glut:post-redisplay))

(defun set-camera (camera-transform)
  (gl:mult-matrix (transform->gl-matrix
                   (cl-transforms:transform-inv camera-transform))))

;; (defun bullet-world-window ()
;;   (glut:display-window (make-instance 'bullet-world-window
;;                                       :camera-transform (cl-transforms:make-transform
;;                                                          (cl-transforms:make-3d-vector -5 0 3)
;;                                                          (cl-transforms:axis-angle->quaternion
;;                                                           (cl-transforms:make-3d-vector 0 1 0)
;;                                                           (/ pi 8))))))
