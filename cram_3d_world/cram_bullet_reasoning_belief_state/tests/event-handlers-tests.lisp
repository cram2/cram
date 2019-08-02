(in-package :btr-belief-tests)

(define-test sink-counter-stable-position-test
  (let* ((obj-name 'bowl-1)
         (spawn-pose (cl-transforms-stamped:make-pose-stamped
                      "map" 0.0
                      (cl-transforms:make-3d-vector 1.4 0.8 0.87)
                      (cl-transforms:make-identity-rotation))))
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
    (btr:prolog-?w
      `(btr:item-type ?w ?obj ?type)
      `(btr:retract (btr:object ?w ?obj)))
    (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
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
    (btr:prolog-?w
      `(btr:item-type ?w ?obj ?type)
      `(btr:retract (btr:object ?w ?obj)))
    (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
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
         (btr:prolog-?w
           `(btr:item-type ?w ?obj ?type)
           `(btr:retract (btr:object ?w ?obj)))
         (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
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
