(in-package :btr-belief-tests)

(defparameter *kitchen-changed* t)

(defun init-env ()
  "Resets the world. Only resets kitchen if it changed."
  (when *kitchen-changed*
    (let ((kitchen-urdf 
            (cl-urdf:parse-urdf 
             (roslisp:get-param "kitchen_description"))))
      (prolog:prolog
       `(and (btr:bullet-world ?world)
             (assert (btr:object ?world
                                 :urdf
                                 :kitchen ((0 0 0) (0 0 0 1))
                                 :urdf ,kitchen-urdf
                                 :collision-group :static-filter
                                 :collision-mask (:default-filter
                                                  :character-filter)
                                 :compound T)))))
    (setf *kitchen-changed* nil))
  (btr:prolog-?w
    `(btr:item-type ?w ?obj ?type)
    `(btr:retract (btr:object ?w ?obj)))
  (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo"))


(define-test sink-counter-stable-position-test
  (let* ((obj-name 'bowl-1)
         (spawn-pose (cl-transforms-stamped:make-pose-stamped
                      "map" 0.0
                      (cl-transforms:make-3d-vector 1.4 0.8 0.87)
                      (cl-transforms:make-identity-rotation))))
    (init-env)
    (prolog:prolog '(and (btr:bullet-world ?world)
                     (assert (btr:object ?world :mesh bowl-1 ((1.4 0.8 0.87) (0 0 0 1))
                              :mass 0.2 :color (1 0 0) :mesh :bowl))))
    (btr:simulate btr:*current-bullet-world* 100)
    (assert-true (< (cl-transforms:v-dist
                     (cl-transforms:origin spawn-pose)
                     (cl-transforms:origin (btr:pose (btr:object btr:*current-bullet-world* obj-name))))
                    0.1))))
    

(define-test sink-counter-unstable-position-test
  (let* ((obj-name 'bowl-1)
         (spawn-pose (cl-transforms-stamped:make-pose-stamped
                      "map" 0.0
                      (cl-transforms:make-3d-vector 1.4 0.8 1.87)
                      (cl-transforms:make-identity-rotation))))
    (init-env)
    (prolog:prolog '(and (btr:bullet-world ?world)
                     (assert (btr:object ?world :mesh bowl-1 ((1.4 0.8 1.87) (0 0 0 1))
                              :mass 0.2 :color (1 0 0) :mesh :bowl))))
    (btr:simulate btr:*current-bullet-world* 100)
    (assert-true (> (cl-transforms:v-dist
                     (cl-transforms:origin spawn-pose)
                     (cl-transforms:origin (btr:pose (btr:object btr:*current-bullet-world* obj-name))))
                    0.1))))
    
(define-test sink-counter-oven-drawer-corner-unstable-with-recovery-test
  (setf btr-belief::*perception-instability-threshold* 0.09)
  (unwind-protect 
       (let* ((obj-name 'bowl-1)
              (spawn-pose (cl-transforms-stamped:make-pose-stamped
                           "map" 0.0
                           (cl-transforms:make-3d-vector 1.53 1.34 0.849)
                           (cl-transforms:make-identity-rotation))))
         (init-env)
         (prolog:prolog '(and (btr:bullet-world ?world)
                          (assert (btr:object ?world :mesh bowl-1 ((1.53 1.34 0.849) (0 0 0 1))
                                   :mass 0.2 :color (1 0 0) :mesh :bowl))))
         (btr:simulate btr:*current-bullet-world* 100)
         (assert-true (> (cl-transforms:v-dist
                          (cl-transforms:origin spawn-pose)
                          (cl-transforms:origin (btr:pose (btr:object
                                                           btr:*current-bullet-world*
                                                           obj-name))))
                         btr-belief::*perception-instability-threshold*))
         (btr-belief::check-and-correct-perception-instability obj-name spawn-pose)
         (assert-true (< (cl-transforms:v-dist
                          (cl-transforms:origin spawn-pose)
                          (cl-transforms:origin (btr:pose (btr:object
                                                           btr:*current-bullet-world*
                                                           obj-name))))
                         btr-belief::*perception-instability-threshold*)))
    (setf btr-belief::*perception-instability-threshold* 0.1)))


(define-test bowl-on-edge-falling-stabilize-test
  "Spawns bowl on the edge of the sink area surface, about to fall.
Tests if the bowl is stable after correction."
  (let* ((obj-name 'bowl-1)
         (coords '(1.23 0.8 0.9))
         (spawn-pose (cl-tf:make-pose-stamped
                      "map" 0.0
                      (apply 'cl-tf:make-3d-vector coords)
                      (cl-tf:make-identity-rotation)))
         (bowl-desig (desig:an object (type :bowl))))
    ;; Prepare the object designator to be fired in the perceive event.
    (setf (slot-value bowl-desig 'desig:data) 
          (make-instance 'desig:object-designator-data
                         :object-identifier obj-name
                         :pose spawn-pose))
    (init-env)
    (prolog:prolog `(and (btr:bullet-world ?world)
                         (assert (btr:object ?world :mesh ,obj-name (,coords (0 0 0 1))
                                             :mass 0.2 :color (1 0 0) :mesh :bowl))))
    ;; Verify that object is unstable.
    (let ((world-copy (cl-bullet:copy-world btr:*current-bullet-world*)))
      (btr:simulate world-copy 20)
      (assert-false (btr:stable-p (btr:object world-copy obj-name))))
    ;; Fire event
    (cram-occasions-events:on-event
     (make-instance 'cram-plan-occasions-events:object-perceived-event
                    :object-designator bowl-desig
                    :perception-source :whatever))
    ;; Verify Object is stable.
    (btr:simulate btr:*current-bullet-world* 20)
    (assert-true (btr:stable-p (btr:object btr:*current-bullet-world* obj-name)))))

#+This-test-was-supposed-to-check-if-a-spoon-doesnt-fall-through-a-drawer-but-it-cant-be-recreated
((define-test spoon-falling-through-drawer-test
  (setf btr-belief::*perception-instability-threshold* 0.09)
  (unwind-protect 
       (let* ((obj-name 'spoon-1)
              (coords '(1.45 0.87 0.72))
              (spawn-pose (cl-transforms-stamped:make-pose-stamped
                           "map" 0.0
                           (apply 'cl-tf:make-3d-vector coords)
                           (cl-tf:make-identity-rotation)))
              (obj-desig (desig:an object (type :spoon))))
         (init-env)
         (setf (btr:joint-state (btr:object btr:*current-bullet-world* :kitchen)
                                "sink_area_left_upper_drawer_main_joint") 0.4)
         (setf *kitchen-changed* t)
         
         (setf (slot-value obj-desig 'desig:data) 
               (make-instance 'desig:object-designator-data
                              :object-identifier obj-name
                              :pose spawn-pose))
         
         (prolog:prolog `(and (btr:bullet-world ?world)
                          (assert (btr:object ?world :mesh ,obj-name (,coords (0 0 0 1))
                                   :mass 0.2 :color (1 0 0) :mesh :spoon))))

         (let ((world-copy (cl-bullet:copy-world btr:*current-bullet-world*)))
           (btr:simulate world-copy 10)
           (assert-false (btr:stable-p (btr:object world-copy obj-name))))
         
         (cram-occasions-events:on-event
          (make-instance 'cram-plan-occasions-events:object-perceived-event
                         :object-designator obj-desig
                         :perception-source :whatever))
         (btr:simulate btr:*current-bullet-world* 10)
         (assert-true (btr:stable-p (btr:object btr:*current-bullet-world* obj-name))))
    
    (setf btr-belief::*perception-instability-threshold* 0.1))))

;; This test has some bug that needs to be fixed. Commenting until it is checked out
;; (define-test kitchen-island-counted-edge-unstable-with-recovery-test
;;   (let* ((obj-name :bowl-1)
;;          (spawn-pose (cl-transforms-stamped:make-pose-stamped
;;                       "map" 0.0
;;                       (cl-transforms:make-3d-vector -0.6607 1.2022 0.822)
;;                       (cl-transforms:make-identity-rotation))))
;;          (let ((kitchen-urdf 
;;                  (cl-urdf:parse-urdf 
;;                   (roslisp:get-param "kitchen_description"))))
;;            (prolog:prolog
;;             `(and (btr:bullet-world ?world)
;;                   (assert (btr:object ?world
;;                                       :urdf
;;                                       :kitchen ((0 0 0) (0 0 0 1))
;;                                       :urdf ,kitchen-urdf
;;                                       :collision-group :static-filter
;;                                       :collision-mask (:default-filter
;;                                                        :character-filter)
;;                                       :compound T)))))
;;     (btr:prolog-?w
;;       `(btr:item-type ?w ?obj ?type)
;;       `(btr:retract (btr:object ?w ?obj)))
;;     (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
;;     (prolog:prolog '(and (btr:bullet-world ?world)
;;                      (assert (btr:object ?world :mesh :bowl-1 ((-0.6607 1.2022 0.822) (0 0 0 1))
;;                               :mass 0.2 :color (1 0 0) :mesh :bowl))))
;;     (btr:simulate btr:*current-bullet-world* 100)
;;     (assert-true (> (cl-transforms:v-dist
;;                      (cl-transforms:origin spawn-pose)
;;                      (cl-transforms:origin (btr:pose (btr:object
;;                                                       btr:*current-bullet-world*
;;                                                       obj-name))))
;;                     btr-belief::*perception-instability-threshold*))
;;     (btr-belief::check-and-correct-perception-instability :bowl-1 spawn-pose)
    ;; (assert-true (< (cl-transforms:v-dist
    ;;                  (cl-transforms:origin spawn-pose)
    ;;                  (cl-transforms:origin (btr:pose (btr:object
    ;;                                                   btr:*current-bullet-world*
    ;;                                                   obj-name))))
    ;;                 btr-belief::*perception-instability-threshold*))))
