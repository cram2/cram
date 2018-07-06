(in-package :le)

(defun test-full-execution ()
  "executes the pick and place task for the entire episode, using both hands of the robot. Maaning: two objects are being carried at once. "
  ;; pick up two
  (get-next-obj-poses 0)
  (pick-up-object (set-grasp-base-pose (make-poses "?PoseCameraStart"))
                  (set-grasp-look-pose (make-poses "?PoseObjStart"))
                  :koelln-muesli-knusper-honig-nuss
                  nil)
  
(get-next-obj-poses 1)
  (pick-up-object (set-grasp-base-pose (make-poses "?PoseCameraStart"))
                  (set-grasp-look-pose (make-poses "?PoseObjStart"))
                  :weide-milch-small
                  t)

;; place two
(get-next-obj-poses 0)
(place-object (set-grasp-base-pose (make-poses "?PoseCameraEnd"))
                (set-grasp-look-pose (place-pose-btr-island))
                (set-grasp-look-pose (place-pose-btr-island))
                ?obj-desig1
                nil)

(get-next-obj-poses 1)
(place-object (set-grasp-base-pose (make-poses "?PoseCameraEnd"))
                (set-grasp-look-pose (place-pose-btr-island))
                (set-grasp-look-pose (place-pose-btr-island))
                ?obj-desig2
                t)

;------
;; pick up two
  (get-next-obj-poses 2)
  (pick-up-object (set-grasp-base-pose (make-poses "?PoseCameraStart"))
                  (set-grasp-look-pose (make-poses "?PoseObjStart"))
                  :cup-eco-orange
                  nil)
  
(get-next-obj-poses 3)
  (pick-up-object (set-grasp-base-pose (make-poses "?PoseCameraStart"))
                  (set-grasp-look-pose (make-poses "?PoseObjStart"))
                  :edeka-red-bowl
                  t)

;; place two
(get-next-obj-poses 2)
(place-object (set-grasp-base-pose (make-poses "?PoseCameraEnd"))
                (set-grasp-look-pose (place-pose-btr-island))
                (set-grasp-look-pose (place-pose-btr-island))
                ?obj-desig1
                nil)

(get-next-obj-poses 3)
(place-object (set-grasp-base-pose (make-poses "?PoseCameraEnd"))
                (set-grasp-look-pose (place-pose-btr-island))
                (set-grasp-look-pose (place-pose-btr-island))
                ?obj-desig2
                t)

;;----
(get-next-obj-poses 4)
  (pick-up-object (set-grasp-base-pose (make-poses "?PoseCameraStart"))
                  (set-grasp-look-pose (make-poses "?PoseObjStart"))
                  :fork-blue-plastic
                  nil)

;; place two
(get-next-obj-poses 4)
(place-object (set-grasp-base-pose (make-poses "?PoseCameraEnd"))
                (set-grasp-look-pose (place-pose-btr-island))
                (set-grasp-look-pose (place-pose-btr-island))
                ?obj-desig1
                nil))

;-------
(defun demo-all-pick-place ()
  "Picks and places all objects of an episode one by one. Meaning the robot will always hold on to just one object and finish placing it before going back to picking up another one. "
  (get-next-obj-poses 0)
  (execute-pick-and-place :koelln-muesli-knusper-honig-nuss)
  (get-next-obj-poses 1)
  
  (execute-pick-and-place :weide-milch-small)
  (get-next-obj-poses 2)
  (execute-pick-and-place :cup-eco-orange)
  (get-next-obj-poses 3)
  (execute-pick-and-place :edeka-red-bowl)
  (get-next-obj-poses 4)
  (execute-pick-and-place :fork-blue-plastic))


(defun demo-all-obj ()
  "For the entire episode, first place the object at the location where it was for the robot to pick up, and then pick it up and place it. "
  (get-grasp-something-poses)
  ;; muesli
  (alternative-demo 'koelln-muesli-knusper-honig-nuss)
  (execute-pick-and-place :koelln-muesli-knusper-honig-nuss)

  ;; milk
  (get-next-obj-poses 1)
  (alternative-demo 'weide-milch-small)
  (execute-pick-and-place :weide-milch-small)

  ;; cup
  (get-next-obj-poses 2)
  (alternative-demo 'cup-eco-orange)
  (execute-pick-and-place :cup-eco-orange)

  ;; bowl
  (get-next-obj-poses 3)
  (alternative-demo 'edeka-red-bowl)
  (execute-pick-and-place :edeka-red-bowl)

  ;; fork
  (get-next-obj-poses 4)
  (alternative-demo 'fork-blue-plastic)
  (execute-pick-and-place :fork-blue-plastic)
  )
