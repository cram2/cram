(in-package :su-demos)

;; aktuell wird die kitchen in der bulletworld nicht mit reingeladen, da ich den teil aus dem code welcher im kommentar am ende der seite ist rausgeloescht habe.
;; zum fix wahrscheinlich am besten in die setup.lisp von einer anderen demo schauen und kopieren

(defun setup-bullet-world ()
  (setf btr:*current-bullet-world* (make-instance 'btr:bt-reasoning-world))

  (let* ((robot
           (or rob-int:*robot-urdf*
               (setf rob-int:*robot-urdf*
                     (get-urdf-hsrb rob-int:*robot-description-parameter*)))))

    (assert
     (cut:force-ll
      (prolog
       `(and
         (btr:bullet-world ?w)
         ,@(when btr-belief:*spawn-debug-window*
             '((btr:debug-window ?w)))
         (btr:assert ?w (btr:object :static-plane :floor ((0 0 0) (0 0 0 1))
                                                  :normal (0 0 1) :constant 0
                                                  :collision-mask (:default-filter)))
         (-> (rob-int:robot ?robot)
             (and (btr:assert ?w (btr:object :urdf ?robot ((0 0 0) (0 0 0 1))
                                             :urdf ,robot))
                  (rob-int:robot-joint-states ?robot :arm :left :park
                                              ?left-joint-states)
                  (assert (btr:joint-state ?world ?robot ?left-joint-states))
                  (rob-int:robot-torso-link-joint ?robot ?_ ?torso-joint)
                  (rob-int:joint-lower-limit ?robot ?torso-joint ?lower-limit)
                  (rob-int:joint-upper-limit ?robot ?torso-joint ?upper-limit)
                  (lisp-fun cma::average ?lower-limit ?upper-limit
                            ?average-joint-value)
                  (assert (btr:joint-state ?world ?robot
                                           ((?torso-joint ?average-joint-value)))))
             (warn "ROBOT was not defined. Have you loaded a robot package?")))))))


  (let ((robot-object (btr:get-robot-object)))
    (if robot-object
        (btr:set-robot-state-from-tf cram-tf:*transformer* robot-object)
        (warn "ROBOT was not defined. Have you loaded a robot package?"))))





;; roslaunch sutuuro_demos ....launch
;; Add all necessary init functions for suturo here, starting with the startup function
;; Not ready for proper usage yet
(defun init-projection ()
  ;; (setf cram-tf:*transformer* (make-instance 'cl-tf2:buffer-client))
  ;;(setf cram-tf:*tf-default-timeout* 0.5) ; projection tf is very fast

  ;;(coe:clear-belief)


  (def-fact-group costmap-metadata (costmap:costmap-size
                                    costmap:costmap-origin
                                    costmap:costmap-resolution
                                    costmap:orientation-samples
                                    costmap:orientation-sample-step)
    (<- (costmap:costmap-size 12 12))
    (<- (costmap:costmap-origin -6 -6))
    (<- (costmap:costmap-resolution 0.04))
    (<- (costmap:orientation-samples 2))
      (<- (costmap:orientation-sample-step 0.3)))
  
  (btr-belief:setup-world-database)

  ;;(setup-bullet-world)

  (setf cram-tf:*tf-default-timeout* 2.0)
  (setf cram-tf:*tf-broadcasting-enabled* t)
  (setf cram-tf:*transformer* (make-instance 'cl-tf2:buffer-client))
  
  (setf prolog:*break-on-lisp-errors* t)

  (btr:clear-costmap-vis-object)
  ;(btr:add-objects-to-mesh-list "cram_hsrb_pick_demo"))


(roslisp-utilities:register-ros-init-function init-projection)




;; (defun setup-bullet-world ()
;;   (setf btr:*current-bullet-world* (make-instance 'btr:bt-reasoning-world))

;;   (let* ((robot
;;            (or rob-int:*robot-urdf*
;;                (setf rob-int:*robot-urdf*
;;                      (get-urdf-hsrb rob-int:*robot-description-parameter*))))
;;          (kitchen
;;            (or btr-belief:*kitchen-urdf*
;;                (let ((kitchen-urdf-string
;;                        (roslisp:get-param btr-belief:*kitchen-parameter* nil)))
;;                  (when kitchen-urdf-string
;;                    (setf btr-belief:*kitchen-urdf*
;;                          (cl-urdf:parse-urdf kitchen-urdf-string)))))))

;;     (assert
;;      (cut:force-ll
;;       (prolog
;;        `(and
;;          (btr:bullet-world ?w)
;;          ,@(when btr-belief:*spawn-debug-window*
;;              '((btr:debug-window ?w)))
;;          (btr:assert ?w (btr:object :static-plane :floor ((0 0 0) (0 0 0 1))
;;                                                   :normal (0 0 1) :constant 0
;;                                                   :collision-mask (:default-filter)))
;;          (-> (man-int:environment-name ?environment-name)
;;              (btr:assert ?w (btr:object :urdf ?environment-name
;;                                         ((0 0 0) (0 0 0 1))
;;                                         :collision-group :static-filter
;;                                         :collision-mask (:default-filter
;;                                                          :character-filter)
;;                                         ,@(when kitchen
;;                                             `(:urdf ,kitchen))
;;                                         :compound T))
;;              (warn "MAN-INT:ENVIRONMENT-NAME was not defined. ~
;;                            Have you loaded an environment knowledge package?"))
;;          (-> (rob-int:robot ?robot)
;;              (and (btr:assert ?w (btr:object :urdf ?robot ((0 0 0) (0 0 0 1))
;;                                              :urdf ,robot))
;;                   (rob-int:robot-joint-states ?robot :arm :left :park
;;                                               ?left-joint-states)
;;                   (assert (btr:joint-state ?world ?robot ?left-joint-states))
;;                   (rob-int:robot-torso-link-joint ?robot ?_ ?torso-joint)
;;                   (rob-int:joint-lower-limit ?robot ?torso-joint ?lower-limit)
;;                   (rob-int:joint-upper-limit ?robot ?torso-joint ?upper-limit)
;;                   (lisp-fun btr-belief::average ?lower-limit ?upper-limit
;;                             ?average-joint-value)
;;                   (assert (btr:joint-state ?world ?robot
;;                                            ((?torso-joint ?average-joint-value)))))
;;              (warn "ROBOT was not defined. Have you loaded a robot package?")))))))


;;   (let ((robot-object (btr:get-robot-object)))
;;     (if robot-object
;;         (btr:set-robot-state-from-tf cram-tf:*transformer* robot-object)
;;         (warn "ROBOT was not defined. Have you loaded a robot package?"))))
