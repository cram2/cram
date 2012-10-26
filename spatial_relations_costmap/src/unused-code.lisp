(in-package :spatial-relations-costmap)

;;; prolog.lisp

(defmacro costmap-generator-names->scores (name-score-pairs)
  `(progn
     ,@(loop for (name score) in name-score-pairs collecting
             `(defmethod costmap-generator-name->score ((name (eql ,name))) ,score))))

(def-fact-group symbols ()
  (<- (intern ?string ?symbol)
    (bound ?string)
    (lisp-fun intern ?string ?symbol))

  (<- (make-symbol ?string-1 ?string-2 ?symbol)
    (bound ?string-1)
    (bound ?string-2)
    (lisp-fun string-concat ?string-1 "-" ?string-2 ?symbol-str)
    (intern ?symbol-str ?symbol))

  ;; (make-symbol ?gen-name-prefix "FIELD" ?field-generator)
  )

(def-fact-group spatial-relations-costmap (desig-costmap)
  ;; uses make-axis-boundary-cost-function to resolve the designator
  (<- (left-of-axis-costmap ?designator ?reference-pose ?costmap)
    (lisp-fun get-y-of-pose ?reference-pose ?y-of-pose)
    (costmap ?costmap)
    (costmap-add-function
     left-of-axis
     (make-axis-boundary-cost-function :Y ?y-of-pose >)
     ?costmap))

  (<- (potential-field-costmap ?designator ?object ?gen-name-prefix ?costmap)
    ;; padding will go away as soon as the occupancy map is done
    ;; (it should include paddings)
    (lisp-fun cl-transforms:origin ?reference-pose ?point)
    (cm-padding-in-meters ?padding-size)
    (instance-of padding-generator ?padding-generator-id)
    (costmap-add-function
     ?padding-generator-id
     (make-range-cost-function ?point ?padding-size :invert t)
     ?costmap)
    ;; This will be added back when near and far is implemented.
    (cm-size-in-meters ?cm-size)
    (instance-of gaussian-generator ?guassian-generator-id)
    (costmap-add-function
     ?gaussian-generator-id
     (make-location-cost-function ?reference-pose ?cm-size)
     ?costmap)
    ;; height generator that assigns the z coordinate of the ref. object to the costmap
    ;;(lisp-fun cl-transforms:origin ?reference-pose ?point)
    (semantic-map-costmap:supporting-z-value ?point ?z)
    (costmap-add-height-generator
     (semantic-map-costmap::make-constant-height-function ?z)
     ?costmap))
  
  ;; ;; left-of for tf poses using gaussian cost-function
  ;; (<- (desig-costmap ?designator ?costmap)
  ;;   (desig-prop ?designator (left-of ?pose))
  ;;   (lisp-type ?pose cl-transforms:pose)
  ;;   (left-of-costmap ?designator ?pose cm-padding-in-meters ?costmap))

  ;; ;; left-of for tf poses using axis-boundary cost-function
  ;; (<- (desig-costmap ?designator ?costmap)
  ;;   (desig-prop ?designator (left-of ?pose))
  ;;   (lisp-type ?pose cl-transforms:pose)
  ;;   (left-of-axis-costmap ?designator ?pose ?costmap))

  ;; ;; left-of for tf poses using potential-field cost-function
  ;; ;; this is all screwed up now
  ;; (<- (desig-costmap ?designator ?costmap)
  ;;   (desig-prop ?designator (left-of ?pose))
  ;;   (lisp-type ?pose cl-transforms:pose)
  ;;   (potential-field-costmap ?designator ?object left-of ?costmap))

  (<- (get-axis-and-predicate ?obj-pose ?supp-obj-pose ?supp-obj-dims
                              ?relation ?axis ?pred)
    (lisp-fun get-closest-edge ?obj-pose ?supp-obj-pose ?supp-obj-dims ?edge)
    (format "the edge = ~a~%" ?edge)
    (lisp-fun get-info-for-relation ?edge ?relation ?info)
    (lisp-fun assoc :axis ?info ?axis-list)
    (lisp-fun cdr ?axis-list ?axis)
    (lisp-fun assoc :pred ?info ?pred-list)
    (lisp-fun cdr ?pred-list ?pred)
    (format "axis = ~a~%pred = ~a~%" ?axis ?pred))



  ;; right-of for bullet objects using potential field cost-function
  (<- (desig-costmap ?designator ?costmap)
    (desig-prop ?designator (right-of ?object))
    (btr:bullet-world ?world)
    (btr:object ?world ?object)
    (-> (or
         (desig-prop ?designator (in-front-of ?object))
         (desig-prop ?designator (behind ?object)))
        (costmap-low-threshold ?threshold)
        (costmap-high-threshold ?threshold))
    (potential-field-costmap ?designator ?object right-of ?threshold ?costmap))
  )
  

(def-fact-group location-desig-utils ()
  (<- (make-pr2-pose ?pose)
    (lisp-fun make-new-pr2-pose ?pose)))


;;; cost-functions.lisp

(defun getBottomZ (obj)
  (let ((aabb (aabb obj)))
    (- (cl-transforms:z (cl-bullet:bounding-box-center aabb))
       (/ (cl-transforms:z (cl-bullet:bounding-box-dimensions aabb)) 2))))

(defun make-new-pr2-pose ()
    (cl-transforms:make-pose
     (cl-transforms:make-3d-vector -1.85d0 2.5d0 0.0d0)
     (cl-transforms:make-identity-rotation)))

(defvar *relations*
  '((:front . (("LEFT-OF" . ((:axis . :Y) (:pred . >)))
               ("RIGHT-OF" . ((:axis . :Y) (:pred . <)))
               ("BEHIND" . ((:axis . :X) (:pred . >)))
               ("IN-FRONT-OF" . ((:axis . :X) (:pred . <)))))
    (:back . (("LEFT-OF" . ((:axis . :Y) (:pred . <)))
              ("RIGHT-OF" . ((:axis . :Y) (:pred . >)))
              ("BEHIND" . ((:axis . :X) (:pred . <)))
              ("IN-FRONT-OF" . ((:axis . :X) (:pred . >)))))
    (:right . (("LEFT-OF" . ((:axis . :X) (:pred . <)))
               ("RIGHT-OF" . ((:axis . :X) (:pred . >)))
               ("BEHIND" . ((:axis . :Y) (:pred . >)))
               ("IN-FRONT-OF" . ((:axis . :Y) (:pred . <)))))
    (:left . (("LEFT-OF" . ((:axis . :X) (:pred . >)))
              ("RIGHT-OF" . ((:axis . :X) (:pred . <)))
              ("BEHIND" . ((:axis . :Y) (:pred . <)))
              ("IN-FRONT-OF" . ((:axis . :Y) (:pred . >)))))))

(defun get-info-for-relation (edge relation)
  (cdr (assoc relation (cdr (assoc edge *relations*)) :test #'equalp)))


;; (with-designators
;;     ((pot-location (location '((on counter) (name kitchen-island))))
;;      (pot (object `((type pot) (at ,pot-location))))
;;      (left-of-pot (location `((left-of ,pot)))))
;;   (perceive-object 'a pot)
;;   (reference left-of-pot))
;; (make-designator 'location `((left-of ,(make-designator 'object `((type pot) (at ,(make-designator 'location `((pose ,(...)))))))))
;; (make-designator 'location `((left-of pot-1)))
;; (force-ll (prolog `(household-object-type ?_ pot ?type)))


;;; designator-integration.lisp

;(register-location-generator 5 generate-left-of)
;(register-location-validation-function 5 validate-left-of)

(defun generate-left-of (designator)
  (let ((left-of-property (desig-prop-value designator 'left-of)))
    (when (typep left-of-property 'cl-transforms:pose)
      (list
       (cl-transforms:transform-pose
        (cl-transforms:reference-transform
         left-of-property)
        (cl-transforms:make-pose
         (cl-transforms:make-3d-vector 0.0 0.05 0.0)
         (cl-transforms:make-identity-rotation)))))))

(defun validate-left-of (designator pose)
  (let ((left-of-property (desig-prop-value designator 'left-of)))
    (format t "validate pose ~a~%" pose)
    (if (not left-of-property)
        t
        t)))




;; built-test-world.lisp

(force-ll (prolog `(and
                    ,@(loop for i from 1 to num-of-sets-on-table collect
                            `(assert (object ?w cutlery ,(new-symbol "KNIFE-" i)
                                             ((2 0 0) (0 0 0 1)) :color (0.5 0 0)
                                             :mass 0.2 :cutlery-type knife)))

                    ,@(loop for i from 1 to 7 collect
                            `(make-on-counter-top-desig ,(new-symbol "?DES" i)))

                    (assign-object-pos pot-1 ?des1)
                    (assign-object-pos bowl-1 ?des2)
                    (assign-object-pos mondamin-1 ?des3)
                    (assign-object-pos mug-1 ?des4)
                    (assign-object-pos plate-1 ?des5) 
                    (assign-object-pos fork-1 ?des6)
                    (assign-object-pos knife-1 ?des7)
                    (assign-pos-on-table-near-world-right-edge plate-1)
                    (assign-pos-on-table plate-1)

                    (assert (object-pose ?w plate-1 ,pose-on-table-1))
                    (assert (object-pose ?w plate-3 ,pose-on-table-2)))))

(setf pr2 (var-value '?pr2 (lazy-car (prolog `(and (robot ?robot) (%object ?w ?robot ?pr2)) *bdgs*))))
(setf sem-map (var-value '?sem-map (lazy-car (prolog `(%object ?w sem-map ?sem-map) *bdgs*))))


(defun staple-plates-on-counter ()
  (prolog `(assert (object-pose ?_ plate-1 ((0.9 1 0.86d0) (0 0 0 1)))))
  (prolog `(assert (object-pose ?_ plate-2 ((0.9 1 0.889d0) (0 0 0 1)))))
  (prolog `(assert (object-pose ?_ plate-3 ((0.9 1 0.91d0) (0 0 0 1))))))

(defun set-obj-pose (obj-name pose)
  (let ((obj (gethash obj-name (slot-value *current-bullet-world* 'objects))))
    (btr::set-object-pose obj pose)))


(defun pose-on-table ()
  (let ((x (- (+ 1.74d0 (random 0.4d0))))
        (y (+ 0.95 (random 1.3d0)))
        (z 1.0d0))
    (cl-transforms:make-pose
     (cl-transforms:make-3d-vector x y z)
     (cl-transforms:make-identity-rotation))))

(defun pose-on-table-near-world-right-edge ()
  (let ((x (- (+ 1.74d0 (random 0.4d0))))
        (y 0.9d0)
        (z 1.0d0))
    (cl-transforms:make-pose
     (cl-transforms:make-3d-vector x y z)
     (cl-transforms:make-identity-rotation))))

(def-fact-group build-test-world ()
  (<- (assign-pos-on-table ?obj-name)
    (once
     (bound ?obj-name)
     (bullet-world ?w)
     (lisp-fun pose-on-table ?pose)
     (assert (object-pose ?w ?obj-name ?pose))))

  (<- (assign-pos-on-table-near-world-right-edge ?obj-name)
    (once
     (bound ?obj-name)
     (bullet-world ?w)
     (lisp-fun pose-on-table-near-world-right-edge ?pose)
     (assert (object-pose ?w ?obj-name ?pose)))))



(defun put-n-sets-on-table (&optional (n 1))
  (loop for i from 1 to n do
    (put-nth-set-on-counter :n i)
    (let ((plate (put-plate-from-counter-on-table)))
      (mapcar (alexandria:rcurry #'put-object-from-counter-near-plate plate)
              '(btr::fork btr::knife btr::mug)))))

(defun put-nth-set-on-counter (&key (n 1) (plate t) (mug t) (fork t) (knife t))
   (and plate
        (prolog `(assert (object-pose ?_ ,(new-symbol-with-id "PLATE-" n) ((0.9 1 0.8594702033075609d0) (0 0 0 1))))))
   (and mug
        (prolog `(assert (object-pose ?_ ,(new-symbol-with-id "MUG-" n) ((0.8 1.4 0.9119799601336841d0) (0 0 0 1))))))
   (and fork
        (prolog `(assert (object-pose ?_ ,(new-symbol-with-id "FORK-" n) ((1 0.8 0.8569999588202436d0) (0 0 0 1))))))
   (and knife
        (prolog `(assert (object-pose ?_ ,(new-symbol-with-id "KNIFE-" n) ((1 0.7 0.8569999867081037d0) (0 0 0 1)))))))