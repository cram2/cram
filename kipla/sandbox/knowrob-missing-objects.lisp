
(in-package :kipla)

(defun init-comp-missing-obj ()
  (ros-info :kipla "Waiting for knowrob service.")
  (cond ((wait-for-service "/json_prolog/query" 1)
         (ros-info :kipla "Knowrob service found.")      
         (json-prolog:prolog-1 '(register-ros-package "comp_missingobj"))
         (json-prolog:prolog-1 '(visualisation-canvas ?a)))
        (t
         (ros-warn :kipla "Could not find knowrob service."))))

(register-ros-init-function init-comp-missing-obj)

(defun get-missing-obj-desigs ()
  (mapcar (lambda (owl-type)
            (make-designator 'object `((type cluster) (owl-type ,(register-owl-type owl-type)))))
          (let ((bdg (car (json-prolog:prolog-1
                           `("comp_missingObjectTypes" "'http://ias.cs.tum.edu/kb/knowrob.owl#KitchenTable0'"
                                                       ?perceivedo ?missingo ?types)))))
            (prog1 (var-value '?types bdg)
              (ros-info :kipla "Visualizing perceived objects ~a" (var-value '?perceivedo bdg))
              (ros-info :kipla "Visualizing missing objects ~a" (var-value '?missingo bdg))
              (json-prolog:prolog-1 `(visualize-perceived-objects (list ,@(var-value '?perceivedo bdg))))
              (json-prolog:prolog-1 `(visualize-missing-objects (list ,@(var-value '?missingo bdg))))))))

(def-plan missing-obj-explore ()
  (achieve `(arm-parked :both))
  (unwind-protect
       (with-designators ((table (location `((on table))))
                          (counter (location `((on counter)))))
         (set-param "/update_table_memory" 1)
         (wait-for-shoulder-scan)
         (achieve `(loc Robot ,(make-designator 'location `((to see) (location ,counter)))))
         (loop repeat 3 do (wait-for-shoulder-scan))
         (achieve `(loc Robot ,(make-designator 'location `((to see) (location ,table)))))
         (loop repeat 3 do (wait-for-shoulder-scan)))
    (set-param "/update_table_memory" 0)))

(def-top-level-plan complete-missing-objs ()
  (pursue
    (run-process-modules)
    (seq
      (sleep 0.5)
      (missing-obj-explore)
      (let ((missing-objects (get-missing-obj-desigs)))
        (let ((obj (car missing-objects)))
          (assert obj)
          (with-designators ((put-down-loc (location `((on table) (for ,obj)))))
            (achieve `(loc ,obj ,put-down-loc))))))))
