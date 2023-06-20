(in-package :su-real)

;;All of the following is Designator/PM stuff.
(defmacro with-hsr-process-modules (&body body)
  "Receives a body of lisp code `body'. Runs the code contained in `body' with all the necessary process modules"
  `(cram-process-modules:with-process-modules-running
       (hsr-navigation
        suturo-pm
        giskard::giskard-pm
        common-desig:wait-pm
        rk:robokudo-perception-pm
        joints:joint-state-pm
        )
     (cpl-impl::named-top-level (:name :top-level),@body)))


;;Process module itself
(cram-process-modules:def-process-module hsr-navigation (motion-designator)
  "Receives motion-designator `motion-designator'. Calls the process module HSR-NAVIGATION with the appropriate designator."
  (destructuring-bind (command argument &rest args)
      (desig:reference motion-designator)
    (declare (ignore args))
    (ecase command
      (cram-common-designators:move-base
       (su-demos::call-nav-action-ps argument)))));;change package in the future

(cpm:def-process-module suturo-pm (motion-designator)
  (destructuring-bind (command argument-1 &rest rest-args)
      (desig:reference motion-designator)
    (ecase command
      (cram-common-designators:open-gripper
       (su-real:open-gripper
        :effort argument-1))
      (cram-common-designators:close-gripper
       (su-real:close-gripper
        :effort argument-1)))))

;; Suturo
(prolog:def-fact-group suturo-fact (cpm:matching-process-module
                                   cpm:available-process-module)

  (prolog:<- (cpm:matching-process-module ?motion-designator su-real::suturo-pm)
    (or (desig:desig-prop ?motion-designator (:type :gripper-motion))))

  (prolog:<- (cpm:available-process-module su-real::suturo-pm)
    (prolog:not (cpm:projection-running ?_))))

;;Denotes the PM as avaivailable
(cram-prolog:def-fact-group available-hsr-process-modules (cpm:available-process-module
                                                           cpm:matching-process-module)

  (cram-prolog:<- (cpm:available-process-module hsr-navigation))
  
  (cram-prolog:<- (cpm:matching-process-module ?desig  hsr-navigation)
    (desig:desig-prop ?desig (:type :going))))


;;Designator inference rules
(cram-prolog:def-fact-group hsr-motion-designators (desig:motion-grounding)

  (cram-prolog:<- (desig:motion-grounding ?designator (going ?pose))
    (desig:desig-prop ?designator (:type :going))
    (desig:desig-prop ?designator (:target ?pose)))

  (cram-prolog:<- (desig:motion-grounding ?designator (going goal-pose))
    (desig:desig-prop ?designator (:type :going))
    (desig:desig-prop ?designator (:x ?x))
    (desig:desig-prop ?designator (:y ?y))
    (desig:desig-prop ?designator (:angle ?angle))))

;; ------
(prolog:def-fact-group giskard-pm (cpm:matching-process-module
                                   cpm:available-process-module)

  (prolog:<- (cpm:matching-process-module ?motion-designator giskard:giskard-pm)
    (or (desig:desig-prop ?motion-designator (:type :moving-tcp))
        (desig:desig-prop ?motion-designator (:type :reaching))
        (desig:desig-prop ?motion-designator (:type :lifting))
        (desig:desig-prop ?motion-designator (:type :retracting))
        (desig:desig-prop ?motion-designator (:type :aligning-height))
        (desig:desig-prop ?motion-designator (:type :placing))
        (desig:desig-prop ?motion-designator (:type :placing-neatly))
        (desig:desig-prop ?motion-designator (:type :tilting))
        (desig:desig-prop ?motion-designator (:type :moving-gripper))
        (desig:desig-prop ?motion-designator (:type :moving-gripper-joint))
        (desig:desig-prop ?motion-designator (:type :moving-arm-joints))
        (desig:desig-prop ?motion-designator (:type :pulling))
        (desig:desig-prop ?motion-designator (:type :pushing))
        (desig:desig-prop ?motion-designator (:type :picking-up))
       ;; (desig:desig-prop ?motion-designator (:type :going))
        (desig:desig-prop ?motion-designator (:type :moving-torso))
        (desig:desig-prop ?motion-designator (:type :moving-custom))
        (desig:desig-prop ?motion-designator (:type :looking))
        (desig:desig-prop ?motion-designator (:type :closing-gripper))
        (desig:desig-prop ?motion-designator (:type :opening-gripper))))

  (prolog:<- (cpm:available-process-module giskard:giskard-pm)
    (prolog:not (cpm:projection-running ?_))))

(prolog:def-fact-group joint-state-pm-facts (cpm:matching-process-module
                                             cpm:available-process-module)

  (prolog:<- (cpm:matching-process-module ?motion-designator joint-state-pm)
    (or (desig:desig-prop ?motion-designator (:type :monitoring-joint-state))))

  (prolog:<- (cpm:available-process-module joint-state-pm)
    (prolog:not (cpm:projection-running ?_))))

