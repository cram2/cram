(in-package :le)


(defun spawn-with-quaternion ()
  "spawns all the objects within the episode at their locations, also moves the robot already in position to pick up the last object. In this function th eobjects are being spawned wth the already fixed quaternion."
  (get-next-obj-poses 0)
  (move-object  (make-poses-with-quaternion "?PoseObjStart") 'ba-muesli)
  (get-next-obj-poses 1)
  (move-object  (make-poses-with-quaternion "?PoseObjStart") 'ba-milk)
  (get-next-obj-poses 2)
  (move-object  (make-poses-with-quaternion "?PoseObjStart") 'ba-cup)
  (get-next-obj-poses 3)
  (move-object  (make-poses-with-quaternion "?PoseObjStart") 'ba-bowl)
  (get-next-obj-poses 4)
  (move-object  (make-poses-with-quaternion "?PoseObjStart") 'ba-fork)

  (move-to-object (set-grasp-base-pose (make-poses-with-quaternion "?PoseCameraStart")) (set-grasp-look-pose (make-poses-with-quaternion "?PoseObjStart"))))


(defun spawn-without-transform ()
  "Spawns objects without the applying the transform which fixes the offset between the Virtual Reality and the bullet world."
  (get-next-obj-poses 0)
  (move-object  (make-poses-without-transform "?PoseObjStart") 'ba-muesli)
  (get-next-obj-poses 1)
  (move-object  (make-poses-without-transform "?PoseObjStart") 'ba-milk)
  (get-next-obj-poses 2)
  (move-object  (make-poses-without-transform "?PoseObjStart") 'ba-cup)
  (get-next-obj-poses 3)
  (move-object  (make-poses-without-transform "?PoseObjStart") 'ba-bowl)
  (get-next-obj-poses 4)
  (move-object  (make-poses-without-transform "?PoseObjStart") 'ba-fork)

  (move-to-object (set-grasp-base-pose (make-poses-without-transform "?PoseCameraStart")) (set-grasp-look-pose (make-poses-without-transform "?PoseObjStart"))))
 

(defun demo-spawn-obj-in-place-offset ()
  "Spawns all object of the Episode with a given offset."
  (get-grasp-something-poses)
  (get-next-obj-poses 0)
  ;; muesli
  (move-object
   (add-pos-offset-to-transform
    (make-poses "?PoseObjStart") 0.05 0.0 0.0) 'ba-muesli)
  
  ;; milk
  (get-next-obj-poses 1)
  (move-object
   (add-pos-offset-to-transform
    (make-poses "?PoseObjStart") 0.05 0.0 0.0) 'ba-milk)

  ;; cup
  (get-next-obj-poses 2)
  (move-object
   (cl-tf:make-transform
    (cl-tf:translation
     (add-pos-offset-to-transform
      (make-poses "?PoseObjStart") 0.05 0.0 0.0))
     (cl-tf:make-identity-rotation))
   'ba-cup)

  ;; bowl
  (get-next-obj-poses 3)
  (move-object
   (add-pos-offset-to-transform
    (make-poses "?PoseObjStart") 0.05 0.0 0.0) 'ba-bowl)

  ;; fork
  (get-next-obj-poses 4)
  (move-object
   (cl-tf:make-transform
    (cl-tf:translation
     (add-pos-offset-to-transform
      (make-poses "?PoseObjStart") 0.05 0.0 0.0))
    (cl-tf:make-quaternion  0 0 1 1)) 'ba-fork))


(defun demo-spawn-obj-in-place ()
  "Spawns all objects of the current Episode at their places without any offsets."
  (get-grasp-something-poses)
  ;; muesli
  (alternative-demo 'ba-muesli)
  
  ;; milk
  (get-next-obj-poses 1)
  (alternative-demo 'ba-milk)

  ;; cup
  (get-next-obj-poses 2)
  (alternative-demo 'ba-cup)

  ;; bowl
  (get-next-obj-poses 3)
  (alternative-demo 'ba-bowl)

  ;; fork
  (get-next-obj-poses 4)
  (alternative-demo 'ba-fork))

(defun spawn-all-own-obj ()
  "Spawns all the objects with which the robot can interact in the world."
  (add-bowl)
  (add-cup)
  (add-muesli)
  (add-fork)
  (add-milk))
