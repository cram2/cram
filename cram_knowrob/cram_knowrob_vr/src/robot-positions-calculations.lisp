(in-package :kvr)

(defparameter *human-feet-offset* 0.0 ; was 0.3 for Andrei's Data -0.05 last
  "a human feet offset. Since the human feet are a lot smaller then the robots
base, this offset can be added in order to prevent the robot from crashing
into the bases of tables.
in short: moves the robot away from the given pose to avoid collisions.")

(defun remove-z (pose)
  "Remove z coordinate in order to move the robot to the x y coordinates
within the map.
`pose': the given pose from which the z coordinate of the translation will be deleted.
RETURNS: a new pose with the z coordinate within the translation set to 0.0"
  (let* ((translation (cl-transforms:translation pose)))
    (cl-transforms:make-transform
     (cl-transforms:make-3d-vector (cl-transforms:x translation)
                                   (cl-transforms:y translation)
                                   0)
     (cl-tf:rotation pose))))

(defun set-grasp-base-pose (transform)
  "Calculates the transform of where the robot should stand in order to interact
with an object. Based on data from Virtual Reality. This function removes the z
component of the Camera pose of the human and also fixes the quaternion so that
the robot won't tilt into the pane of the floor.
TRANSFORM: The transform of the Camera (head position of the human)  which is
given by the Virtual Reality.
RETURNS: A pose stamped, with removed z components and fixed rotations so that
the robot can be placed at this position.
EXAMPLE USAGE: (make-poses \"?PoseCameraStart\")"
  (let* ((pose ; make the transform a viable robot position
           (cl-transforms:transform->pose (remove-z transform)))
         (quaternion
           (cl-transforms:orientation pose))
         (x ; transform into a matrix, take 3rd column X
           (aref (cl-transforms:quaternion->matrix quaternion) 0 2))
         (y ; transform into a matrix, take 3rd column Y
           (aref (cl-transforms:quaternion->matrix quaternion) 1 2))
         (new-quaternion
           (cl-transforms:axis-angle->quaternion
            (cl-transforms:make-3d-vector 0 0 1)
            (atan y x))))
    (cl-transforms-stamped:make-pose-stamped
     "map"
     0.0
     (cl-transforms:make-3d-vector
      (if (plusp (cl-transforms:z new-quaternion))
          (+ (cl-transforms:x (cl-tf:origin pose)) *human-feet-offset*)
          (- (cl-transforms:x (cl-tf:origin pose)) *human-feet-offset*))
      (cl-transforms:y (cl-tf:origin pose))
      0)
     new-quaternion)))

(defun set-grasp-look-pose (transform)
  "Transforms the given transform of the position of an object, into a pose
stamped, at which the robot will look for an object.
TRANSFORM: The given transform of the objects assumed/derrived location.
RETURNS: A pose stamped of where the robot should look for an object.
EXAMPLE USAGE: (make-poses \"?PoseObjStart\")"
  (let* ((?grasp-look-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-tf:origin (cl-tf:transform->pose transform))
            (cl-tf:orientation (cl-tf:transform->pose transform)))))
    ?grasp-look-pose))

(defun set-place-pose (transform)
  "Takes the given transform of the placing the object pose, sets the
quaternion to an identity one and transforms the transform into a pose stamped.
TRANSFORM: The given transform at which the human placed an object.
RETURNS: A pose stamped, with an identity quaternion.
EXAMPLE USAGE WITH: (make-poses \"?PoseObjEnd\")"
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:origin (cl-transforms:transform->pose transform))
   (cl-transforms:make-identity-rotation)))

(defun place-pose-btr-island (type)
  "Calculates the placing pose for an object, relative to the bullet world
kitchen island. This is needed, since the OpenEase kitchen island and the
bullet world kitchen island, are slightly offset to one another, and the offset
fixing the general semantic map offset, is not enough to fix it.
Therefore relative poses are being calculated.
RETURNS: A cl-tf:transform representing the pose and orientation, at which the
robot in the bullet world should place the object currently in hand."
  ;;FIXME A wrong pose is being calculated. I don't know why yet.
  (let* ((table-pose-oe (get-table-location))
         (table-pose-bullet ; get pose of Table in map frame
           (cl-transforms:pose->transform
            (btr:pose
             (if (btr:rigid-body
                  (btr:object btr:*current-bullet-world* :kitchen)
                  '|IslandArea_nhwy|)
                 nil
                 (btr:rigid-body
                  (btr:object btr:*current-bullet-world* :kitchen)
                  ':|KITCHEN.kitchen_island|)))))
         (place-pose ; calculate place pose relative to bullet table
           (cl-tf:transform*
            table-pose-bullet
            (cl-tf:transform-inv table-pose-oe)
            (get-object-location-at-end-by-object-type type))))
    place-pose))


(defun place-pose (type)
  "Clauclates the placing transform of the object
relative to the surface it was placed on.
Transform is returned in the urdf map frame.
umap-T-uobj = umap-T-usurface * inv(ssurface-T-ssmap) * ssmap-T-sobj" 
  (let* ((prolog-type
           (object-type-filter-prolog type))
         (table-pose-oe
           (get-contact-surface-place-pose prolog-type))
         (table-pose-bullet ; get pose of Table in map frame
           (cl-tf:pose->transform
            (btr:pose
             (btr:rigid-body
              (btr:object btr:*current-bullet-world* :kitchen)
              (match-kitchens
               (get-contact-surface-place-name prolog-type))))))
         (place-pose ; calculate place pose relative to bullet table
           (cl-tf:transform*
            table-pose-bullet
            (cl-tf:transform-inv table-pose-oe)
            (get-object-location-at-end-by-object-type prolog-type))))
    place-pose))


(defun pick-pose (type)
 "Calculates the picking up transform of the object relative to the surface it was
picked up from. Transform is returned in the urdf map frame.
umap-T-uobj = umap-T-usurface * inv(ssurface-T-ssmap) * ssmap-T-sobj."
  (let* ((prolog-type
           (object-type-filter-prolog type))
         (surface-pose-oe
           (get-contact-surface-pick-pose prolog-type))
         (surface-pose-bullet ; get pose of Table in map frame
           (cl-transforms:pose->transform
            (btr:pose
             (btr:rigid-body
              (btr:object btr:*current-bullet-world* :kitchen)
              (match-kitchens
               (get-contact-surface-pick-name prolog-type))))))
         (pick-pose ; calculate pick pose relative to bullet table
           (cl-transforms:transform*
            surface-pose-bullet
            (cl-tf:transform-inv surface-pose-oe)
            (get-object-location-at-start-by-object-type prolog-type))))
    pick-pose))


(defun urobot-T-uobj (type)
  "Calculates the transform between the urdf robot T urdf object.
urobot-T-uobj = inv(umap-T-robot) * umap-T-obj"
  (let* ((umap-T-robot
           (cl-transforms:pose->transform
            (btr:pose (btr:get-robot-object))))
         (umap-T-obj
           (cl-transforms:pose->transform
            (btr:pose
             (btr:object btr:*current-bullet-world*
                         (object-type-filter-bullet type)))))
         (robot-T-obj
           (cl-tf:transform*
            (cl-tf:transform-inv umap-T-robot)
            umap-T-obj)))
    robot-T-obj))

(defun ucamera-T-usurface (type &optional (time :start))
  "calculates the transfrom between urdf actor/camera/robot and the urdf surface.
ucamera-T-usurface = inv(smap-T-scamera) * smap-T-ssurface * inv(umap-T-ssurface)"
  (let* ((prolog-type (object-type-filter-prolog type))
         (smap-T-scamera
           (if (eql time :start)
               (get-camera-location-at-start-by-object-type prolog-type)
               (get-camera-location-at-end-by-object-type prolog-type)))
         (smap-T-ssurface
           (if (eql time :start)
               (get-contact-surface-pick-pose prolog-type)
               (get-contact-surface-place-pose prolog-type)))
         (umap-T-usurface (cl-tf:pose->transform
           (btr:pose
            (btr:rigid-body
             (btr:object btr:*current-bullet-world* :kitchen)
             (match-kitchens
              (if (eql time :start)
                  (get-contact-surface-pick-name prolog-type)
                  (get-contact-surface-place-name prolog-type)))))))
         (ucamera-T-usurface
           (cl-transforms:transform-inv
            (cl-transforms:transform*
             (cl-transforms:transform-inv smap-T-scamera)
             smap-T-ssurface
             (cl-transforms:transform-inv umap-T-usurface)))))
    ucamera-T-usurface))

(defun umap-T-human (type &optional (time :start))
  "Calculates the transform urdfmap T human actor from semantic map
umap-T-human = umap-T-uobj * inv(smap-T-sobj) * smap-T-scamera"
  (let* ((prolog-type (object-type-filter-prolog type))
         (umap-T-uobj
           (cl-tf:pose->transform
            (btr:pose
             (btr:object btr:*current-bullet-world*
                         (object-type-filter-bullet type)))))
         (smap-T-sobj
           ;; differentiante between picking up and placing via time variable
           ;; it should be set to :start or :end
           (if (eql time :start)
               (get-object-location-at-start-by-object-type prolog-type)
               (get-object-location-at-end-by-object-type prolog-type)))
         (smap-T-scamera
           (if (eql time :start)
               (get-camera-location-at-start-by-object-type prolog-type)
               (get-camera-location-at-end-by-object-type prolog-type)))
         (umap-T-human
           (cl-transforms:transform*
            umap-T-uobj
            (cl-transforms:transform-inv smap-T-sobj)
            smap-T-scamera)))
    umap-T-human))

(defun umap-T-robot (type)
  "urdf map T to the robot.
umap-T-robot = umap-T-robot * urobot-T-uobj"
  (cl-tf:transform*
   (cl-tf:pose->transform
    (btr:pose (btr:get-robot-object)))
   (urobot-T-uobj type)))

;; experiments
;; use the laser scanner instead of base footprint directly
;; to avoid colliding with table
(defun umap-T-laser-T-human (type &optional (time :start))
  "Calculates the transform urdfmap T human actor from semantic map
umap-T-human = umap-T-uobj * inv(smap-T-sobj) * smap-T-scamera"
  (let* ((umap-T-laser
           (cl-tf:pose->transform
            (btr:link-pose (btr:get-robot-object) "base_laser_link")))
         (umap-T-base
           (cl-tf:pose->transform
            (btr:link-pose (btr:get-robot-object) "base_footprint")))
         (umap-T-laser-human
           (cl-tf:transform*
            umap-T-base
            (cl-tf:transform-inv umap-T-laser)
            (umap-T-human type time))))
    umap-T-laser-human))

(defun umap-T-laser-T-place (type &optional (time :end))
  (let* ((umap-T-laser
           (cl-tf:pose->transform
            (btr:link-pose (btr:get-robot-object) "base_laser_link")))
         (umap-T-base
           (cl-tf:pose->transform
            (btr:link-pose (btr:get-robot-object) "base_footprint")))
         (umap-T-laser-surface
           (cl-tf:transform*
            umap-T-base
            (cl-tf:transform-inv umap-T-laser)
            (ucamera-T-usurface type time))))
    umap-T-laser-surface))
