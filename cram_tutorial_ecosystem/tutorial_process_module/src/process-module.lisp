(in-package :tutorial-process-module)

(defvar *known-objects* nil
  "List of known objects used in the scope of this tutorial.")
(defvar *knowledge-base* nil
  "List of known objects, including extra information not acquirable
through only perception.")

(defun add-object (desc-perc desc-know)
  (push (make-designator
         'cram-designators:object
         desc-perc)
        *known-objects*)
  (push (make-designator
         'cram-designators:object
         (append desc-perc desc-know))
        *knowledge-base*))

(defun init-tutorial-process-module ()
  "Initialized the tutorial process module."
  (setf *known-objects* nil)
  (setf *knowledge-base* nil)
  (add-object
   `((desig-props:type desig-props:box)
     (desig-props:color desig-props:blue))
   `((desig-props:name "Blue box")
     (desig-props:category desig-props:container)
     (desig-props:owner "Markus")))
  (add-object
   `((desig-props:type desig-props:box)
     (desig-props:color desig-props:blue))
   `((desig-props:name "Blue box")
     (desig-props:category desig-props:container)
     (desig-props:owner "Daniela")))
  (add-object
   `((desig-props:type desig-props:box)
     (desig-props:color desig-props:red))
   `((desig-props:name "Red box")
     (desig-props:category desig-props:cornflakes)))
  (add-object
   `((desig-props:type desig-props:box)
     (desig-props:color desig-props:white))
   `((desig-props:name "White box")
     (desig-props:category desig-props:container)))
  (add-object
   `((desig-props:type desig-props:bowl)
     (desig-props:color desig-props:green))
   `((desig-props:name "Green bowl")))
  (add-object
   `((desig-props:type desig-props:bowl)
     (desig-props:color desig-props:red))
   `((desig-props:name "Red bowl")))
  (add-object
   `()
   `((desig-props:type desig-props:cutlery)
     (desig-props:name "Some cutlery")))
  (add-object
   `((desig-props:color desig-props:white))
   `((desig-props:name "A white object"))))

(register-ros-init-function init-tutorial-process-module)

(defun generate-random-location ()
  (make-designator
   'cram-designators:location
   `((desig-props:pose ,(tf:make-pose
                         (tf:make-3d-vector (/ (random 100) 100.0)
                                            (/ (random 100) 100.0)
                                            (/ (random 100) 100.0))
                         (tf:make-identity-rotation))))))

(defgeneric call-action (action &rest params))

(defmethod call-action ((action-sym t) &rest params)
  (roslisp:ros-info
   (tutorial process-module)
   "Unimplemented operation `~a' with parameters ~a. Doing nothing."
   action-sym params)
  (sleep 0.5))

(defmethod call-action :around (action-sym &rest params)
  (roslisp:ros-info (tutorial process-module)
                    "Executing action ~a ~a."
                    action-sym params)
  (prog1 (call-next-method)
    (roslisp:ros-info (tutorial process-module)
                      "Action done.")))

(def-process-module tutorial-process-module (desig)
  (apply #'call-action (reference desig)))
