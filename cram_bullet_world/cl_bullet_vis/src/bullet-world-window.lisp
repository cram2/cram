
(in-package :bt-vis)

(defvar *bullet-window-loop-rate* 100
  "The update rate in Hz of the event loop.")

(defvar *background-color* (list (/ 206 255) (/ 210 255) (/ 237 255) 0))

(glut:init)

(defclass bullet-world-window (glut:window bullet-world-gl-context event-queue)
  ((frame-rate :initform 25 :initarg :frame-rate :reader frame-rate
               :documentation "The desired frame rate in Hz. The
               system tries to redisplay the window at this rate.")
   (motion-mode :initform nil :reader motion-mode)
   (camera-center-distance :reader camera-center-distance
                           :initarg :camera-center-distance
                           :initform 3.0)
   (pointer-pos :initform nil :reader pointer-pos)
   (closed :initform nil :reader closed)
   (redrawing :initform nil :accessor redrawing)
   (hidden :initform nil :reader hidden :initarg :hidden)
   (display-callbacks :initform nil :accessor display-callbacks))
  (:default-initargs :width 640 :height 480 :title "bullet visualization"
                     :mode '(:double :rgba :depth :alpha)))

(defgeneric close-window (window)
  (:method ((w bullet-world-window))
    (post-event w '(:close))))

(defgeneric show-window (window)
  (:method ((window bullet-world-window))
    (post-event
     window
     `(:display :callback ,(lambda ()
                             (glut:show-window)
                             (setf (slot-value window 'hidden) nil))))))

(defgeneric hide-window (window)
  (:method ((window bullet-world-window))
    (post-event
     window
     `(:display :callback ,(lambda ()
                             (glut:hide-window)
                             (setf (slot-value window 'hidden) t))))))

(defgeneric process-event (window type &key))

(defmethod glut:display-window :around ((w bullet-world-window))
  (unwind-protect
       (call-next-method)
    (setf (slot-value w 'closed) t)))

(defmethod glut:idle ((w bullet-world-window))
  (loop for ev = (get-next-event w (/ *bullet-window-loop-rate*))
        while ev do (apply #'process-event w ev)))

(defmethod glut:display-window :before ((w bullet-world-window))
  (when (hidden w)
    (glut:hide-window))
  (with-rendering-lock
    (gl:cull-face :back)
    (gl:depth-func :lequal)
    (gl:shade-model :smooth)
    (gl:blend-func :src-alpha :one-minus-src-alpha)
    (gl:hint :perspective-correction-hint :nicest)))

(defmethod glut:display-window :after ((w bullet-world-window))
  (when (frame-rate w)
    (glut:enable-tick w (truncate (* (/ (frame-rate w)) 1000)))))

(defvar *draw-ui* nil)
(defvar *draw-textured-number* nil)
(defvar *text-to-draw* "")

(defun draw-ui (window)
  (when *draw-ui*
    (gl:disable :lighting)
    (gl:with-pushed-matrix
      (let ((text-point (cl-transforms:transform-point
                        (camera-transform window)
                        (cl-transforms:make-3d-vector 3.0 0 1.3))))
        (gl:translate (cl-transforms:x text-point)
                      (cl-transforms:y text-point)
                      (cl-transforms:z text-point)))
      (gl:color 1 0 0 1.0)
      (%gl:raster-pos-3f 0.0 0.0 0.0)
      (glut:bitmap-string glut:+bitmap-times-roman-24+ *text-to-draw*)
      (glut:bitmap-string glut:+bitmap-times-roman-24+ "Red")
      (when *draw-textured-number*
        ;; (print (gl:get* :current-raster-position))
        ;; (print (gl:get* :current-raster-position-valid))
        (let ((text-pose (cl-transforms:transform-pose
                          (camera-transform window)
                          (cl-transforms:make-pose
                           (cl-transforms:make-3d-vector 3.0 0 1.3)
                           (cl-transforms:make-identity-rotation)))))
          ;; (gl:translate (cl-transforms:x (cl-transforms:origin text-pose))
          ;;               (cl-transforms:y (cl-transforms:origin text-pose))
          ;;               (cl-transforms:z (cl-transforms:origin text-pose)))
          (gl:translate 0.5 0 0)
          (multiple-value-bind (axis angle)
              (cl-transforms:quaternion->axis-angle (cl-transforms:orientation text-pose))
            (gl:rotate (cma:radians->degrees angle)
                       (cl-transforms:x axis) (cl-transforms:y axis) (cl-transforms:z axis))
            (gl:rotate 90 0 1 0)
            (gl:rotate -90 0 0 1)))
        (gl:enable :lighting)
        (gl:color 1 0 0 1.0)
        (gl:scale 0.1 0.1 0.1)
        ;; (glut:solid-cube 2.0)
        (gl:with-pushed-attrib (:texture-bit :enable-bit)
          (multiple-value-bind (texture-handle new?)
              (cl-bullet-vis:get-texture-handle window (gensym))
            (gl:bind-texture :texture-2d texture-handle)
            ;; (print new?)
            (when new?
              (gl:tex-env :texture-env :texture-env-mode :modulate)
              (gl:tex-parameter :texture-2d :texture-min-filter :linear-mipmap-linear)
              (gl:tex-parameter :texture-2d :texture-mag-filter :linear)
              (gl:tex-parameter :texture-2d :texture-wrap-s :repeat)
              (gl:tex-parameter :texture-2d :texture-wrap-t :repeat)
              (glu:build-2d-mipmaps
               :texture-2d 4 16 16 :rgba :float
               (texture-str->bitmap
                (concatenate
                 'string
                 "xxxxxxxxxxxxxxxx"
                 "x              x"
                 "x      xx      x"
                 "x     x x      x"
                 "x    x  x      x"
                 "x   x   x      x"
                 "x       x      x"
                 "x       x      x"
                 "x       x      x"
                 "x       x      x"
                 "x       x      x"
                 "x       x      x"
                 "x    xxxxxxx   x"
                 "x              x"
                 "x              x"
                 "xxxxxxxxxxxxxxxx")
                #\o #\Space #\x '(1 1 1 1)))))
          (gl:enable :texture-2d)
          (gl:disable :cull-face)
          (gl:disable :lighting)
          (gl:with-primitive :quads
            (gl:normal 0 0 1)
            (gl:tex-coord 0.0 0.0)
            (gl:vertex -0.5 -0.5)
            (gl:tex-coord 1.0 0.0)
            (gl:vertex 0.5 -0.5)
            (gl:tex-coord 1.0 1.0)
            (gl:vertex 0.5 0.5)
            (gl:tex-coord 0.0 1.0)
            (gl:vertex -0.5 0.5)))))
    (gl:with-pushed-matrix
      (let ((text-point (cl-transforms:transform-point
                        (camera-transform window)
                        (cl-transforms:make-3d-vector 3.0 -0.2 1.3))))
        (gl:translate (cl-transforms:x text-point)
                      (cl-transforms:y text-point)
                      (cl-transforms:z text-point)))
      (gl:color 0 1 0 1.0)
      (%gl:raster-pos-3f 0.0 0.0 0.0)
      (glut:bitmap-string glut:+bitmap-times-roman-24+ "Green"))
    (gl:enable :lighting)))

(defmethod glut:display ((window bullet-world-window))
  (with-rendering-lock
    (gl:matrix-mode :projection)
    (gl:load-identity)
    (glu:perspective 50 (/ (glut:width window) (glut:height window)) 0.1 1000)
    (gl:matrix-mode :modelview)
    (init-camera)
    (gl:enable :light0 :lighting :cull-face :depth-test :color-material :blend :rescale-normal)
    (apply #'gl:clear-color *background-color*)
    (gl:clear :color-buffer :depth-buffer)
    (set-camera (camera-transform window))
    (gl:light :light0 :position (vector
                                 (cl-transforms:x (light-position window))
                                 (cl-transforms:y (light-position window))
                                 (cl-transforms:z (light-position window))
                                 0))
    (gl:light-model :light-model-ambient #(0.5 0.5 0.5 1.0))
    (gl:light :light0 :diffuse #(0.8 0.8 0.8 1))
    (gl:light :light0 :specular #(0.8 0.8 0.8 1))
    (gl:color 0.8 0.8 0.8 1.0)
    (gl:with-pushed-matrix
      (let ((transparent-objects (remove-if-not #'gl-object-transparent (gl-objects window)))
            (opaque-objects (remove-if #'gl-object-transparent (gl-objects window))))
        (dolist (obj opaque-objects)
          (draw window obj))
        (%gl:depth-mask nil)
        (dolist (obj transparent-objects)
          (draw window obj))
        (%gl:depth-mask t)))
    ;; Draw UI texts
    (draw-ui window)
    ;; When we are moving around, draw a little yellow disk similar to
    ;; that one RVIZ draws.
    (when (or (eq (motion-mode window) :rotate)
              (eq (motion-mode window) :translate))
      (gl:with-pushed-matrix
        (let ((disk-pos (cl-transforms:transform-point
                         (camera-transform window)
                         (cl-transforms:make-3d-vector (camera-center-distance window) 0 0))))
          (gl:translate (cl-transforms:x disk-pos)
                        (cl-transforms:y disk-pos)
                        (cl-transforms:z disk-pos)))
        (gl:color 0.8 0.8 0.0 1.0)
        (gl:scale 1 1 0.1)
        (glut:solid-sphere 0.1 50 50)))
    (let ((callbacks (display-callbacks window)))
      (setf (display-callbacks window) nil)
      (map 'nil #'funcall callbacks))
    (setf (redrawing window) nil)
    (gl:flush)
    (glut:swap-buffers)
    (gc-gl-context window)))

(defmethod glut:reshape ((window bullet-world-window) width height)
  (with-slots (glut:width glut:height) window
    (setf glut:width width)
    (setf glut:height height)
    (gl:viewport 0 0 width height)))

(defmethod glut:keyboard ((window bullet-world-window) key x y)
  (declare (ignore x y))
  (when (eql key #\Esc)
    (close-window window)))

(defmethod glut:mouse ((window bullet-world-window) button state x y)
  (with-slots (motion-mode pointer-pos camera-transform camera-center-distance) window
    (case state
      (:up (setf motion-mode nil))
      (:down (unless motion-mode
               (setf pointer-pos `(,x ,y))
               (case button
                 (:left-button (setf motion-mode :rotate))
                 (:right-button (setf motion-mode :translate))
                 (:wheel-up
                    (setf camera-transform (cl-transforms:transform*
                                            camera-transform
                                            (cl-transforms:make-transform
                                             (cl-transforms:make-3d-vector (* 0.10 camera-center-distance) 0 0)
                                             (cl-transforms:make-quaternion 0 0 0 1))))
                    (setf camera-center-distance (* camera-center-distance 0.90)))
                 (:wheel-down
                    (setf camera-transform (cl-transforms:transform*
                                            camera-transform
                                            (cl-transforms:make-transform
                                             (cl-transforms:make-3d-vector (* -0.10 camera-center-distance) 0 0)
                                             (cl-transforms:make-quaternion 0 0 0 1))))
                    (setf camera-center-distance (* camera-center-distance 1.10))))))))
  (glut:post-redisplay))

(defmethod glut:motion ((window bullet-world-window) x y)
  (with-slots (motion-mode camera-transform camera-center-distance pointer-pos) window
    (unless pointer-pos
      (return-from glut:motion nil))
    (destructuring-bind (old-x old-y) pointer-pos
      (let ((dx (/ (- x old-x) (glut:width window)))
            (dy (/ (- y old-y) (glut:height window)))
            (y-axis-in-world (cl-transforms:rotate (cl-transforms:rotation camera-transform)
                                                   (cl-transforms:make-3d-vector 0 1 0)))
            (camera-center (cl-transforms:transform-point
                            camera-transform
                            (cl-transforms:make-3d-vector
                             camera-center-distance 0 0))))
        (setf pointer-pos `(,x ,y))
        (case motion-mode
          (:rotate
             (setf camera-transform (cl-transforms:transform*
                                     (cl-transforms:make-transform
                                      camera-center (cl-transforms:make-quaternion 0 0 0 1))
                                     (cl-transforms:make-transform
                                      (cl-transforms:make-3d-vector 0 0 0)
                                      (cl-transforms:axis-angle->quaternion
                                       (cl-transforms:make-3d-vector 0 0 1)
                                       (* dx pi)))
                                     (cl-transforms:make-transform
                                      (cl-transforms:make-3d-vector 0 0 0)
                                      (cl-transforms:axis-angle->quaternion
                                       y-axis-in-world
                                       (* dy pi)))
                                     (cl-transforms:transform-inv
                                      (cl-transforms:make-transform
                                       camera-center (cl-transforms:make-quaternion 0 0 0 1)))
                                     camera-transform)))
          (:translate
             (setf camera-transform (cl-transforms:transform* camera-transform
                                                              (cl-transforms:make-transform
                                                               (cl-transforms:make-3d-vector
                                                                0
                                                                (float (* dx 10))
                                                                (float (* dy 10)))
                                                               (cl-transforms:make-quaternion 0 0 0 1)))))))))
  (glut:post-redisplay))

(defmethod glut:tick ((w bullet-world-window))
  (unless (closed w)
    (glut:post-redisplay)))

(defmethod process-event ((w bullet-world-window) (type (eql :close)) &key)
  (setf (slot-value w 'closed) t)
  (glut:destroy-current-window))

(defmethod process-event ((w bullet-world-window) (type (eql :redisplay)) &key)
  (unless (or (closed w) (redrawing w))
    (setf (redrawing w) t)
    (glut:post-redisplay)))

(defmethod process-event ((w bullet-world-window) (type (eql :display)) &key callback)
  (push callback (display-callbacks w))
  (glut:display w))

(defmethod run-in-gl-context ((window bullet-world-window) function)
  (let ((lock (sb-thread:make-mutex))
        (condition (sb-thread:make-waitqueue))
        (result nil)
        (done nil))
    (post-event
     window
     `(:display
       :callback ,(lambda ()
                    (unwind-protect
                         (handler-case
                             (setf result (list :ok (funcall function)))
                           (error (e)
                             (setf result (list :error e)))
                           (warning (w)
                             (setf result (list :warning w))))
                      (sb-thread:with-mutex (lock)
                        (setf done t)
                        (sb-thread:condition-broadcast condition))))))
    (sb-thread:with-mutex (lock)
      (loop until done
            do (handler-case
                   (sb-ext:with-timeout 0.01
                     (sb-thread:condition-wait condition lock))
                 (sb-ext:timeout ()
                   nil))
            finally (destructuring-bind (status value) result
                      (return
                        (ecase status
                          (:ok value)
                          (:error (error value))
                          (:warning (warn value)))))))))

(defun init-camera ()
  "Sets the camera such that x points forward, y to the left and z
upwards. This matches ROS' coordinates best."
  (gl:load-identity)
  (gl:rotate 90 1 0 0)
  (gl:rotate -90 0 0 1)
  (gl:rotate 180 1 0 0))

(defun set-camera (camera-transform)
  (gl:mult-matrix (transform->gl-matrix
                   (cl-transforms:transform-inv camera-transform))))

