(in-package :cram-integration-tests)

(defparameter *motions-executed* 0)

(defmethod exe:generic-perform :after ((designator motion-designator))
  (incf *motions-executed*))

(defun reset-motions-counter ()
  (setf *motions-executed* 0))

(defun executed-motions? ()
  (> *motions-executed* 0))

(defun initialize ()
  (sb-ext:gc :full t)
  (setf btr-belief:*spawn-debug-window* nil)

  ;;(when ccl::*is-logging-enabled*
  ;;    (setf ccl::*is-client-connected* nil)
  ;;    (ccl::connect-to-cloud-logger)
  ;;    (ccl::reset-logged-owl))

  ;; (setf proj-reasoning::*projection-checks-enabled* t)

  (btr:detach-all-objects (btr:get-robot-object))
  (btr:detach-all-objects (btr:get-environment-object))
  (btr-utils:kill-all-objects)
  ;; (setf (btr:joint-state (btr:get-environment-object)
  ;;                        "sink_area_left_upper_drawer_main_joint")
  ;;       0.0
  ;;       (btr:joint-state (btr:get-environment-object)
  ;;                        "sink_area_left_middle_drawer_main_joint")
  ;;       0.0
  ;;       (btr:joint-state (btr:get-environment-object)
  ;;                        "iai_fridge_door_joint")
  ;;       0.0
  ;;       (btr:joint-state (btr:get-environment-object)
  ;;                        "oven_area_area_right_drawer_main_joint")
  ;;       0.0
  ;;       (btr:joint-state (btr:get-environment-object)
  ;;                        "sink_area_trash_drawer_main_joint")
  ;;       0)
  (btr-belief::publish-environment-joint-state
   (btr:joint-states (btr:get-environment-object)))

  (setf desig::*designators* (tg:make-weak-hash-table :weakness :key))

  (coe:clear-belief)

  (btr:clear-costmap-vis-object)

  ;; (setf cram-robot-pose-guassian-costmap::*orientation-samples* 3)

  (reset-motions-counter)
  )

(defun get-pose ()
  (let ((map-T-surface (cl-transforms:pose->transform
                        (btr:link-pose (btr:get-environment-object)
                                       "sink_area_surface"))))
    (let* ((surface-T-object
             (cl-transforms:pose->transform
              (cram-tf:list->pose '((0.2 -0.15 0.1) (0 0 0 1)))))
           (map-T-object
             (cl-transforms:transform* map-T-surface surface-T-object)))
      (cl-transforms:transform->pose map-T-object))))

(defun spawn-object-on-sink-counter ()
  (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
  (btr-utils:kill-all-objects)
  (btr:detach-all-objects (btr:get-robot-object))
  (btr:detach-all-objects (btr:get-environment-object))

  (let ((spawn-pose (get-pose)))
    (btr-utils:spawn-object :test-object :breakfast-cereal)
    (btr-utils:move-object :test-object spawn-pose)))

(defun move-robot-in-position ()
  (btr-utils:move-robot '((.6 .6 0) (0 0 0 1))))

(define-test navigation-goal
  (initialize)
  (let* ((?pose (cl-tf:make-pose-stamped "map" 0.0
                                         (cl-tf:make-3d-vector 0.7 0.7 0)
                                         (cl-tf:make-identity-rotation)))
         (?goal `(cpoe:robot-loc ,(a location (pose ?pose))))
         (executed-motions-initially? nil))
    (urdf-proj:with-simulated-robot
      (perform
       (an action
           (type going)
           (target (a location (pose ?pose)))
           (goal ?goal)))
      (setf executed-motions-initially? (executed-motions?))
      (reset-motions-counter)
      (perform
       (an action
           (type going)
           (target (a location (pose ?pose)))
           (goal ?goal))))
    (lisp-unit:assert-true executed-motions-initially?))
  (lisp-unit:assert-false (executed-motions?)))

(define-test arms-positioned-goal
  (initialize)
  (urdf-proj:with-simulated-robot
    (let ((?goal `(cpoe:arms-positioned :park :park)))
      (perform
       (an action
           (type positioning-arm)
           (left-configuration park)
           (right-configuration park)
           (goal ?goal)))))
  (lisp-unit:assert-false (executed-motions?)))

(define-test torso-at-goal
  (initialize)
  (let ((executed-motions-initially? nil))
    (urdf-proj:with-simulated-robot
      (let ((?goal `(cpoe:torso-at :lower-limit)))
        (perform
         (an action
             (type moving-torso)
             (joint-angle :lower-limit)
             (goal ?goal)))
        (setf executed-motions-initially? (executed-motions?))
        (reset-motions-counter)
        (perform
         (an action
             (type moving-torso)
             (joint-angle :lower-limit)
             (goal ?goal)))))
    (lisp-unit:assert-true executed-motions-initially?))
  (lisp-unit:assert-false (executed-motions?)))

(define-test look-at-goal
  (initialize)
  (let* ((?pose (cl-tf:make-pose-stamped "map" 0.0
                                         (cl-tf:make-3d-vector 0.7 0.7 1)
                                         (cl-tf:make-identity-rotation)))
         (?look-pos (a location
                      (pose ?pose)))
         (?goal `(cpoe:looking-at ,?look-pos))
         (action (an action
                     (type looking)
                     (target ?look-pos)
                     (goal ?goal)))
         (executed-motions-initially? nil))
    (urdf-proj:with-simulated-robot
      (perform action)
      (setf executed-motions-initially? (executed-motions?))
      (reset-motions-counter)
      (perform action))
    (lisp-unit:assert-true executed-motions-initially?))
  (lisp-unit:assert-false (executed-motions?)))

(define-test container-state-open-goal
  (initialize)
  (let* ((?container (an object
                         (type drawer)
                         (urdf-name sink-area-left-upper-drawer-main)
                         (part-of environment)))
         (?goal `(cpoe:container-state ,?container :open))
         (action (an action
                     (type opening)
                     (object ?container)
                     (arm left)
                     (goal ?goal)))
         (executed-motions-initially? nil))
    (urdf-proj:with-simulated-robot
      (btr-utils:move-robot '((.5 .4 0) (0 0 0 1)))
      (perform action)
      (setf executed-motions-initially? (executed-motions?))
      (reset-motions-counter)
      (perform action))
    (lisp-unit:assert-true executed-motions-initially?))
  (lisp-unit:assert-false (executed-motions?)))

(define-test container-state-close-goal
  (initialize)
  (let* ((?container (an object
                         (type drawer)
                         (urdf-name sink-area-left-upper-drawer-main)
                         (part-of environment)))
         (?goal `(cpoe:container-state ,?container :closed))
         (action (an action
                     (type closing)
                     (object ?container)
                     (arm left)
                     (goal ?goal)))
         (executed-motions-initially? nil))
    (urdf-proj:with-simulated-robot
      (btr:set-robot-state-from-joints
       `((,"sink_area_left_upper_drawer_main_joint" ,0.4))
       (btr:object btr:*current-bullet-world* :environment))
      (btr-utils:move-robot '((.5 .4 0) (0 0 0 1)))
      (perform action)
      (setf executed-motions-initially? (executed-motions?))
      (reset-motions-counter)
      (perform action))
    (lisp-unit:assert-true executed-motions-initially?))
  (lisp-unit:assert-false (executed-motions?)))

(define-test ees-at
  (initialize)
  (urdf-proj:with-simulated-robot
    (let* ((tool-transform
             (cl-tf:lookup-transform cram-tf:*transformer* "map" cram-tf:*robot-right-tool-frame*))
           (tool-pos (cl-tf:translation tool-transform))
           (tool-rot (cl-tf:rotation tool-transform))
           (tool-pose (cl-tf:make-pose-stamped "map" 0.0 tool-pos tool-rot))
           (?tool-pose-list (list tool-pose)))
      (let* ((?goal `(cpoe:ees-at nil ,tool-pose))
             (action (an action
                         (type reaching)
                         (right-poses ?tool-pose-list)
                         (goal ?goal))))
        (perform action))))
  (lisp-unit:assert-false (executed-motions?)))

(define-test picking-up-goal
  (initialize)
  
  (spawn-object-on-sink-counter)
  (let ((executed-motions-initially? nil))
    (urdf-proj:with-simulated-robot
      (move-robot-in-position)
      (let ((?look-pose (cl-tf:make-pose-stamped "map" 0
                                                 (cl-tf:origin (get-pose))
                                                 (cl-tf:orientation (get-pose)))))
        (perform (an action
                     (type looking)
                     (target (a location
                                (pose ?look-pose))))))
      (let* ((?object
               (perform
                (an action
                    (type perceiving)
                    (object
                     (an object
                         (type breakfast-cereal))))))
             ;; We copy the object here because the original gets changed,
             ;; in a way that referencing does not work anymore.
             (?object-copy (desig:copy-designator ?object))
             (?goal `(cpoe:object-picked ,?object)))
        (perform
         (an action
             (type picking-up)
             (object ?object)
             (goal ?goal)))
        (setf executed-motions-initially? (executed-motions?))
        (reset-motions-counter)
        (perform
         (an action
             (type picking-up)
             (object ?object-copy)
             (goal ?goal)))))
    (lisp-unit:assert-true executed-motions-initially?))
  (lisp-unit:assert-false (executed-motions?)))
