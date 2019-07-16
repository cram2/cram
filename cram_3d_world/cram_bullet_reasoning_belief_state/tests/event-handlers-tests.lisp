(in-package :btr-belief-tests)

(define-test sink-counter-stable-position-test
  (let* ((obj-name "bowl-1")
         (obj-type :bowl)
         (spawn-pose (cl-transforms-stamped:make-pose-stamped
                      "map" 0.0
                      (cl-transforms:make-3d-vector 1.4 0.8 0.87)
                      (cl-transforms:make-identity-rotation)))
         (spawned-object))
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
    (btr-utils:kill-all-objects)
    (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
    (setf spawned-object
          (btr-utils:spawn-object obj-name obj-type :pose spawn-pose))
    (btr:simulate btr:*current-bullet-world* 100)
    (assert-true (< (cl-transforms:v-dist
                     (cl-transforms:origin spawn-pose)
                     (cl-transforms:origin (btr:pose spawned-object)))
                    0.1))))
    

(define-test sink-counter-unstable-position-test
  (let* ((obj-name "bowl-1")
         (obj-type :bowl)
         (spawn-pose (cl-transforms-stamped:make-pose-stamped
                      "map" 0.0
                      (cl-transforms:make-3d-vector 1.4 0.8 1.87)
                      (cl-transforms:make-identity-rotation)))
         (spawned-object))
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
    (btr-utils:kill-all-objects)
    (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
    (setf spawned-object
          (btr-utils:spawn-object obj-name obj-type :pose spawn-pose))
    (btr:simulate btr:*current-bullet-world* 100)
    (assert-true (> (cl-transforms:v-dist
                     (cl-transforms:origin spawn-pose)
                     (cl-transforms:origin (btr:pose spawned-object)))
                    0.1))))
    

(define-test sink-counter-oven-drawer-corner-unstable-position-with-recovery-test
  (setf btr-belief::*perception-instability-threshold* 0.09)
  (unwind-protect 
       (let* ((obj-name :bowl-1)
              (obj-type :bowl)
              (spawn-pose (cl-transforms-stamped:make-pose-stamped
                           "map" 0.0
                           (cl-transforms:make-3d-vector 1.53 1.34 0.849)
                           (cl-transforms:make-identity-rotation)))
              (spawned-object))
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
         (btr-utils:kill-all-objects)
         (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
         (setf spawned-object
               (btr-utils:spawn-object obj-name obj-type :pose spawn-pose))
         (btr:simulate btr:*current-bullet-world* 100)
         (assert-true (> (cl-transforms:v-dist
                          (cl-transforms:origin spawn-pose)
                          (cl-transforms:origin (btr:pose spawned-object)))
                         btr-belief::*perception-instability-threshold*))
         (btr-belief::check-and-correct-perception-instability :bowl-1 spawn-pose)
         (assert-true (< (cl-transforms:v-dist
                          (cl-transforms:origin spawn-pose)
                          (cl-transforms:origin (btr:pose spawned-object)))
                         btr-belief::*perception-instability-threshold*)))
    (setf btr-belief::*perception-instability-threshold* 0.1)))


(define-test kitchen-island-counted-edge-unstable-with-recovery-test
  (let* ((obj-name :bowl-1)
         (obj-type :bowl)
         (spawn-pose (cl-transforms-stamped:make-pose-stamped
                      "map" 0.0
                      (cl-transforms:make-3d-vector -0.6607 1.2022 0.89)
                      (cl-transforms:make-identity-rotation)))
         (spawned-object))
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
    (btr-utils:kill-all-objects)
    (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
    (setf spawned-object
          (btr-utils:spawn-object obj-name obj-type :pose spawn-pose))
    (btr:simulate btr:*current-bullet-world* 100)
    (assert-true (> (cl-transforms:v-dist
                     (cl-transforms:origin spawn-pose)
                     (cl-transforms:origin (btr:pose spawned-object)))
                    btr-belief::*perception-instability-threshold*))
    (btr-belief::check-and-correct-perception-instability :bowl-1 spawn-pose)
    (assert-true (< (cl-transforms:v-dist
                     (cl-transforms:origin spawn-pose)
                     (cl-transforms:origin (btr:pose spawned-object)))
                    btr-belief::*perception-instability-threshold*))))
