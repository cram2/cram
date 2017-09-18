(in-package :cram-tutorial-executive)

(defmacro with-process-modules (&body body)
  `(cpm:with-process-modules-running
       (tutorial-process-module:tutorial-process-module)
     ,@body))

(defun tutorial-plan ()
  (top-level
    (with-process-modules
      (with-designators
          ((obj-desig (object `((desig-props:type desig-props:box))))
           (perceive-action
            (action `((desig-props:to desig-props:perceive)
                      (desig-props:obj ,obj-desig)))))
        (perform perceive-action)))))

(def-top-level-cram-function perceive-objects ()
  (with-process-modules
    (with-designators
        ((obj-1 (object `((color red))))
         (act-perceive (action
                        `((desig-props:to
                           desig-props:perceive)
                          (desig-props:obj ,obj-1)))))
      (perform act-perceive))))

(def-top-level-cram-function perceive-and-ground-objects ()
  (with-process-modules
    (with-designators
        ((obj-1 (object `((color red))))
         (act-perceive (action
                        `((desig-props:to
                           desig-props:perceive)
                          (desig-props:obj ,obj-1)))))
      (let ((objects (perform act-perceive)))
        (loop for object in objects
              collect
              (with-designators
                  ((act-ground
                    (action
                     `((desig-props:to
                        desig-props:ground)
                       (desig-props:obj ,object)))))
                (perform act-ground)))))))

(def-top-level-cram-function move-perceived-object ()
  (with-process-modules
    (with-designators
        ((obj-1 (object `((color red) (type box))))
         (act-perceive (action
                        `((desig-props:to
                           desig-props:perceive)
                          (desig-props:obj ,obj-1)))))
      (let ((object (first (perform act-perceive))))
        (when object
          (with-designators
               ((act-move
                 (action
                  `((desig-props:to desig-props:move)
                    (desig-props:obj ,object)
                    (desig-props:loc
                     ,(tut-pm:generate-random-location))))))
            (perform act-move)))))))

(def-top-level-cram-function perceive-failure ()
  (with-process-modules
    (with-designators
        ((obj-1 (object `((color red)))))
      (with-failure-handling
          ((cram-tutorial-planlib:ambiguous-perception (f)
             (declare (ignore f))
             (roslisp:ros-error
              (perceive-failure plan)
              "Found more than one object that match the given description. This is a failure in this context as we were asking for `the' object.")))
        (cram-tutorial-planlib::perceive-object :the obj-1)))))
