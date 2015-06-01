(in-package :reas-inf)

(defvar *start-time*)

(defmacro out-info (&rest args)
  `(ros-info (bullet-reasoning-interface) ,@args))

(defmacro out-error (&rest args)
  `(ros-error (bullet-reasoning-interface) ,@args))

(defmacro out-debug (&rest args)
  `(ros-debug (bullet-reasoning-interface) ,@args))
