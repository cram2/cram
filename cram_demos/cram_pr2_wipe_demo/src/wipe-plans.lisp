(in-package :pr2-wipe)
 
(defun wipe (&key
                ((:surface ?surface-designator))
                ((:surface-name  ?surface-name))
                ((:arm ?arm))
                ((:left-wipe-poses ?left-wipe-poses))
                ((:right-wipe-poses ?right-wipe-poses))
                ((:collision-mode ?collision-mode))
             &allow-other-keys)

  ;;repeat approach designator until all poses have been performed
  (cond ((equal ?left-wipe-poses nil)

         (loop for x in ?right-wipe-poses
               do
                        (roslisp:ros-info (wiping) "wiping")
                        (cpl:with-failure-handling
                            ((common-fail:manipulation-low-level-failure (e)
                               (roslisp:ros-warn (wipe-plans wipe)
                                                 "Manipulation messed up: ~a~%Ignoring."
                                                 e)))
                          (exe:perform
                           (desig:an action
                                     (type approaching)
                                     (right-poses x)
                                     (desig:when ?collision-mode
                                       (collision-mode ?collision-mode)))))))


        ((equal ?right-wipe-poses nil)
               (loop for x in ?left-wipe-poses

                     do
                        (roslisp:ros-info (wiping) "wiping")
                        (cpl:with-failure-handling
                            ((common-fail:manipulation-low-level-failure (e)
                               (roslisp:ros-warn (wipe-plans wipe)
                                                 "Manipulation messed up: ~a~%Ignoring."
                                                 e)))
                          (exe:perform
                           (desig:an action
                                     (type approaching)
                                     (left-poses x)
                                     (desig:when ?collision-mode
                                       (collision-mode ?collision-mode)))))))))
