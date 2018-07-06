(in-package :le)


(defun spawn-with-quaternion ()
  "spawns all the objects within the episode at their locations, also moves the robot already in position to pick up the last object. In this function th eobjects are being spawned wth the already fixed quaternion."
  (get-next-obj-poses 0)
  (move-object  (make-poses-with-quaternion "?PoseObjStart") 'koelln-muesli-knusper-honig-nuss)
  (get-next-obj-poses 1)
  (move-object  (make-poses-with-quaternion "?PoseObjStart") 'weide-milch-small)
  (get-next-obj-poses 2)
  (move-object  (make-poses-with-quaternion "?PoseObjStart") 'cup-eco-orange)
  (get-next-obj-poses 3)
  (move-object  (make-poses-with-quaternion "?PoseObjStart") 'edeka-red-bowl)
  (get-next-obj-poses 4)
  (move-object  (make-poses-with-quaternion "?PoseObjStart") 'fork-blue-plastic)

  (move-to-object (set-grasp-base-pose (make-poses-with-quaternion "?PoseCameraStart")) (set-grasp-look-pose (make-poses-with-quaternion "?PoseObjStart"))))


(defun spawn-without-transform ()
  "Spawns objects without the applying the transform which fixes the offset between the Virtual Reality and the bullet world."
  (get-next-obj-poses 0)
  (move-object  (make-poses-without-transform "?PoseObjStart") 'koelln-muesli-knusper-honig-nuss)
  (get-next-obj-poses 1)
  (move-object  (make-poses-without-transform "?PoseObjStart") 'weide-milch-small)
  (get-next-obj-poses 2)
  (move-object  (make-poses-without-transform "?PoseObjStart") 'cup-eco-orange)
  (get-next-obj-poses 3)
  (move-object  (make-poses-without-transform "?PoseObjStart") 'edeka-red-bowl)
  (get-next-obj-poses 4)
  (move-object  (make-poses-without-transform "?PoseObjStart") 'fork-blue-plastic)

  (move-to-object (set-grasp-base-pose (make-poses-without-transform "?PoseCameraStart")) (set-grasp-look-pose (make-poses-without-transform "?PoseObjStart"))))
 

(defun demo-spawn-obj-in-place-offset ()
  "Spawns all object of the Episode with a given offset."
  (get-grasp-something-poses)
  (get-next-obj-poses 0)
  ;; muesli
  (move-object
   (add-pos-offset-to-transform
    (make-poses "?PoseObjStart") 0.05 0.0 0.0) 'koelln-muesli-knusper-honig-nuss)
  
  ;; milk
  (get-next-obj-poses 1)
  (move-object
   (add-pos-offset-to-transform
    (make-poses "?PoseObjStart") 0.05 0.0 0.0) 'weide-milch-small)

  ;; cup
  (get-next-obj-poses 2)
  (move-object
   (cl-tf:make-transform
    (cl-tf:translation
     (add-pos-offset-to-transform
      (make-poses "?PoseObjStart") 0.05 0.0 0.0))
     (cl-tf:make-identity-rotation))
   'cup-eco-orange)

  ;; bowl
  (get-next-obj-poses 3)
  (move-object
   (add-pos-offset-to-transform
    (make-poses "?PoseObjStart") 0.05 0.0 0.0) 'edeka-red-bowl)

  ;; fork
  (get-next-obj-poses 4)
  (move-object
   (cl-tf:make-transform
    (cl-tf:translation
     (add-pos-offset-to-transform
      (make-poses "?PoseObjStart") 0.05 0.0 0.0))
    (cl-tf:make-quaternion  0 0 1 1)) 'fork-blue-plastic))


(defun demo-spawn-obj-in-place ()
  "Spawns all objects of the current Episode at their places without any offsets."
  (get-grasp-something-poses)
  ;; muesli
  (alternative-demo 'koelln-muesli-knusper-honig-nuss)
  
  ;; milk
  (get-next-obj-poses 1)
  (alternative-demo 'weide-milch-small)

  ;; cup
  (get-next-obj-poses 2)
  (alternative-demo 'cup-eco-orange)

  ;; bowl
  (get-next-obj-poses 3)
  (alternative-demo 'edeka-red-bowl)

  ;; fork
  (get-next-obj-poses 4)
  (alternative-demo 'fork-blue-plastic))

(defun spawn-all-own-obj ()
  "Spawns all the objects with which the robot can interact in the world."
  (add-bowl)
  (add-cup)
  (add-muesli)
  (add-fork)
  (add-milk))
