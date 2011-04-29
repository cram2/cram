
(in-package :bt-vis)

(defvar *bullet-window-loop-rate* 100
  "The update rate in Hz of the event loop.")

(defclass bullet-world-window (glut:window gl-context event-queue)
  ((world :accessor world :initarg :world
          :initform (error 'simple-error :format-control "world argument required"))
   (frame-rate :initform 25 :initarg :frame-rate :reader frame-rate
               :documentation "The desired frame rate in Hz. The
               system tries to redisplay the window at this rate.")
   (motion-mode :initform nil :reader motion-mode)
   (camera-center-distance :reader camera-center-distance
                           :initarg :camera-center-distance
                           :initform 3.0)
   (pointer-pos :initform nil :reader pointer-pos)
   (closed :initform nil :reader closed)
   (redrawing :initform nil :accessor redrawing)
   (display-callbacks :initform nil :accessor display-callbacks))
  (:default-initargs :width 640 :height 480 :title "bullet visualization"
    :mode '(:double :rgba :depth)))

(defgeneric close-window (window)
  (:method ((w bullet-world-window))
    (post-event w '(:close))))

(defgeneric process-event (window type &key))

;; We want to implement our own main loop that supports different
;; events, not only OpenGL events.
(defmethod glut:display-window :around ((w bullet-world-window))
  (let ((glut:*run-main-loop-after-display* nil))
    (call-next-method)
    (loop until (closed w) do
      (loop for ev = (get-next-event w (/ *bullet-window-loop-rate*))
            while ev do (apply #'process-event w ev))
      (glut:main-loop-event))
    ;; Clean up. Process all pending opengl events
    (glut:main-loop-event)))

(defmethod glut:display-window :before ((w bullet-world-window))
  (gl:clear-color 0 0 0 0)
  (gl:cull-face :back)
  (gl:depth-func :lequal)
  (gl:shade-model :smooth)
  (gl:blend-func :src-alpha :one-minus-src-alpha)
  (gl:hint :perspective-correction-hint :nicest))

(defmethod glut:display-window :after ((w bullet-world-window))
  (glut:disable-event w :idle)
  (when (frame-rate w)
    (glut:enable-tick w (truncate (* (/ (frame-rate w)) 1000)))))

(defmethod glut:display ((window bullet-world-window))
  (gl:matrix-mode :projection)
  (gl:load-identity)
  (glu:perspective 50 (/ (glut:width window) (glut:height window)) 0.1 100)
  (gl:matrix-mode :modelview)
  (init-camera)
  (gl:enable :light0 :lighting :cull-face :depth-test :color-material :blend :rescale-normal)
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
    (with-world-locked (world window)
      (draw window (world window))))
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
  (gc-gl-context window))

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
  (glut:post-redisplay))

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

(defmacro with-bullet-window-context (window &body body)
  (let ((lock (gensym "LOCK-"))
        (condition (gensym "CONDITION-"))
        (result (gensym "RESULT-"))
        (done (gensym "DONE-")))
    `(let ((,lock (sb-thread:make-mutex))
           (,condition (sb-thread:make-waitqueue))
           (,result nil)
           (,done nil))
       (post-event
        ,window
        `(:display
          :callback ,(lambda ()
                       (unwind-protect
                            (handler-case
                                (setf ,result (list :ok (progn ,@body)))
                              (error (e)
                                (setf ,result (list :error e)))
                              (warning (w)
                                (setf ,result (list :warning w))))
                         (sb-thread:with-mutex (,lock)
                           (setf ,done t)
                           (sb-thread:condition-broadcast ,condition))))))
       (sb-thread:with-mutex (,lock)
         (loop until ,done do
           (handler-case
               (sb-ext:with-timeout 0.01
                 (sb-thread:condition-wait ,condition ,lock))
             (sb-ext:timeout ()
               nil))
               finally (destructuring-bind (status value)
                           ,result
                         (return
                           (ecase status
                             (:ok value)
                             (:error (error value))
                             (:warning (warn value))))))))))

