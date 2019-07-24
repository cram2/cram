(in-package :btr-tests)

;; Help functions to convert objects to lists, so these can be compared with equals
(defun orientation->list (orient)
  (list (cl-transforms:w orient)
        (cl-transforms:x orient)
        (cl-transforms:y orient)
        (cl-transforms:z orient)))

(defun vector->list (vector)
  (list (cl-transforms:x vector)
        (cl-transforms:y vector)
        (cl-transforms:z vector)))

(defun pose->list (pose)
  (list (vector->list (cl-transforms:origin pose))
        (orientation->list (cl-transforms:orientation pose))))

(defun pose-equal (other-pose pose)
  (equal (pose->list pose)
         (pose->list other-pose)))


(define-test create-static-collision-information-works
  ;; Tests if the collision information is properly saved and if it has an effect on
  ;; the object by simulating the bullet world
  (btr-utils:spawn-object 'o1 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (let* ((pose-o1 (btr:pose (btr:object btr:*current-bullet-world* 'o1))))
    
    (btr::create-static-collision-information (btr:object btr:*current-bullet-world* 'o1))
    (lisp-unit:assert-equal :CF-STATIC-OBJECT (car
                                               (cl-bullet:collision-flags
                                                (first
                                                 (btr:rigid-bodies (btr:object btr:*current-bullet-world* 'o1))))))
    (btr:simulate btr:*current-bullet-world* 1)
    (let ((new-pose-o1 (btr:pose (btr:object btr:*current-bullet-world* 'o1))))
      
      (lisp-unit:assert-true (pose-equal pose-o1 new-pose-o1))))

  ;; recreate begin state for next test case
  (btr:remove-object btr:*current-bullet-world* 'o1))


(define-test reset-collision-information-works
  ;; Tests if the collision information is properly reseted and if it has an effect on
  ;; the object by simulating the bullet world
  (btr-utils:spawn-object 'o1 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'o2 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))

  (let* ((pose-o1 (btr:pose (btr:object btr:*current-bullet-world* 'o1))))
    (btr:attach-object 'o1 'o2)
    (btr::reset-collision-information (btr:object btr:*current-bullet-world* 'o1)
                                      (list (btr::make-collision-information :rigid-body-name 'o1 :flags NIL)))
    (lisp-unit:assert-equal NIL (car
                                 (cl-bullet:collision-flags
                                  (first
                                   (btr:rigid-bodies (btr:object btr:*current-bullet-world* 'o1))))))
    (btr:simulate btr:*current-bullet-world* 1)
    (let ((new-pose-o1 (btr:pose (btr:object btr:*current-bullet-world* 'o1))))
      
      (lisp-unit:assert-false (pose-equal pose-o1 new-pose-o1))))
    
  ;; recreate begin state for next test case
  (btr:remove-object btr:*current-bullet-world* 'o1)
  (btr:remove-object btr:*current-bullet-world* 'o2))

(define-test reset-collision-information-fails
  ;; Tests if nothing happens if nil is passed
  (btr-utils:spawn-object 'o1 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (lisp-unit:assert-false (btr::reset-collision-information (btr:object btr:*current-bullet-world* 'o1) nil))
  (btr:remove-object btr:*current-bullet-world* 'o1))

(define-test attach-object-called-with-item-symbols
  ;; Tests if the attach-object function gets properly called if it was called with the object item names
  (btr-utils:spawn-object 'oo1 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'oo2 :mug :pose 
                           '((-1 0.0 0.92)(0 0 0 1)))

  (btr:attach-object 'oo1 'oo2)
  (lisp-unit:assert-equal 'oo1 (car (assoc 'oo1 (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo2)))))
  (lisp-unit:assert-equal 'oo2 (car (assoc 'oo2 (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo1)))))

  (btr:remove-object btr:*current-bullet-world* 'oo1)
  (btr:remove-object btr:*current-bullet-world* 'oo2))

(define-test attach-object-called-with-more-items-connecting-to-one-item-in-one-call
  ;; Attaches three objects and 'o1 is connected with both objects in one call
  (btr-utils:spawn-object 'o1 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'o2 :mug :pose 
                          '((-1 0.75 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'o3 :mug :pose 
                          '((-1 0.75 0.92)(0 0 0 1)))

  ;;connect objects o2 <-> o1 <-> o3
  (btr:attach-object (list 'o2 'o3) 'o1)
  
  ;;these are connected o2 <-> o1 <-> o3
  (lisp-unit:assert-equal 'o2 (car (assoc 'o2 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o1)))))
  (lisp-unit:assert-equal 'o3 (car (assoc 'o3 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o1)))))
  (lisp-unit:assert-equal 'o1 (car (assoc 'o1 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o2)))))
  (lisp-unit:assert-equal 'o1 (car (assoc 'o1 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o3)))))
  ;;these are not connected to each other: o2 o3
  (lisp-unit:assert-false (equal 'o2 (car (assoc 'o2 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o3))))))
  (lisp-unit:assert-false (equal 'o3 (car (assoc 'o3 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o2))))))

  ;; recreate begin state for next test case
  (btr:remove-object btr:*current-bullet-world* 'o1)
  (btr:remove-object btr:*current-bullet-world* 'o2)
  (btr:remove-object btr:*current-bullet-world* 'o3))

(define-test attach-object-called-with-item-symbol-and-nil-as-parameter
  ;; Tests if the attach-object function does nothing if called with nil and an valid item object
  (btr-utils:spawn-object 'oo1 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))

  (lisp-unit:assert-false (btr:attach-object 'oo1 nil))
  (lisp-unit:assert-number-equal 0 (list-length (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo1))))
  (lisp-unit:assert-false (btr:attach-object nil 'oo1))
  (lisp-unit:assert-number-equal 0 (list-length (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo1))))
  (lisp-unit:assert-false (btr:attach-object nil nil))
  
  (btr:remove-object btr:*current-bullet-world* 'oo1))

(define-test detach-object-called-with-item-symbols
  ;; Tests if the detach-object function gets properly called if it was called with the object item names
  (btr-utils:spawn-object 'oo1 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'oo2 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))

  (btr:attach-object 'oo1 'oo2)
  (lisp-unit:assert-equal 'oo1 (car (assoc 'oo1 (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo2)))))
  (lisp-unit:assert-equal 'oo2 (car (assoc 'oo2 (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo1)))))

  (btr:detach-object 'oo1 'oo2)
  (lisp-unit:assert-false (equal 'oo2 (car (assoc 'oo2 (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo1))))))
  (lisp-unit:assert-false (equal 'oo1 (car (assoc 'oo1 (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo2))))))

  (btr:remove-object btr:*current-bullet-world* 'oo1)
  (btr:remove-object btr:*current-bullet-world* 'oo2))


(define-test detach-object-called-with-item-symbol-and-nil-as-parameter
  ;; Tests if the detach-object function does nothing if called with nil and an valid item object
  (btr-utils:spawn-object 'oo1 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'oo2 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))

  (btr:attach-object 'oo1 'oo2)
  (lisp-unit:assert-equal 'oo1 (car (assoc 'oo1 (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo2)))))
  (lisp-unit:assert-equal 'oo2 (car (assoc 'oo2 (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo1)))))

  (lisp-unit:assert-false (btr:detach-object nil 'oo1))
  (lisp-unit:assert-equal 'oo1 (car (assoc 'oo1 (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo2)))))
  (lisp-unit:assert-equal 'oo2 (car (assoc 'oo2 (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo1)))))

  (lisp-unit:assert-false (btr:detach-object 'oo1 nil))
  (lisp-unit:assert-equal 'oo1 (car (assoc 'oo1 (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo2)))))
  (lisp-unit:assert-equal 'oo2 (car (assoc 'oo2 (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo1)))))

  (lisp-unit:assert-false (btr:detach-object nil nil))

  (btr:remove-object btr:*current-bullet-world* 'oo1)
  (btr:remove-object btr:*current-bullet-world* 'oo2))

  
