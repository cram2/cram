(in-package :cram-pr2-pick-place-demo-tests)

(defun round-to (number precision &optional (what #'round))
    (let ((div (expt 10 precision)))
      (/ (funcall what (* number div)) div)))

(defun round-poses (poses &key (precision 3))
   (mapcar (lambda (n) (mapcar (lambda (m) (float (round-to m precision)))  (second n))) poses))

(defun move-surface (&key (x 0) (y 0) (z 0))
  (let* ((old-pose (btr:link-pose (btr:object btr:*current-bullet-world* :kitchen) "sink_area_surface"))
         (new-origin (list (+ (cl-transforms:x (cl-transforms:origin old-pose)) x)
                           (+ (cl-transforms:y (cl-transforms:origin old-pose)) y)
                           (+ (cl-transforms:z (cl-transforms:origin old-pose)) z)))
                        
         (new-pose (cl-transforms:make-pose
                    (cl-transforms:make-3d-vector (first new-origin)
                                                  (second new-origin)
                                                  (third new-origin))
                    (cl-transforms:orientation old-pose))))
    (setf (btr:link-pose (btr:object btr:*current-bullet-world* :kitchen) "sink_area_surface")
                         new-pose)))
  

(define-test make-poses-relative-default
  (let* ((goal '((1.335 0.665 0.94) ;; breakfast-cereal 
                (1.335 0.865 0.94) ;; cup 
                (1.355 1.065 0.94) ;; bowl 
                (1.385 0.915 0.79) ;; spoon 
                (1.465 0.865 0.94))) ;; milk
         (poses (round-poses (cram-pr2-pick-place-demo::make-poses-relative))))
    (lisp-unit:assert-equal goal poses)))
    

(define-test make-poses-relative-x-move
  (let ((goal '((2.335 0.665 0.94) ;; breakfast-cereal 
                (2.335 0.865 0.94) ;; cup 
                (2.355 1.065 0.94) ;; bowl 
                (2.385 0.915 0.79) ;; spoon 
                 (2.465 0.865 0.94))));; milk
    (move-surface :x 1)
    (let ((poses (round-poses (cram-pr2-pick-place-demo::make-poses-relative))))
      (move-surface :x -1)
      (lisp-unit:assert-equal goal poses))))

(define-test make-poses-relative-y-move
  (let ((goal '((1.335 1.665 0.94) ;; breakfast-cereal 
                (1.335 1.865 0.94) ;; cup 
                (1.355 2.065 0.94) ;; bowl 
                (1.385 1.915 0.79) ;; spoon 
                (1.465 1.865 0.94))));; milk
    (move-surface :y 1)
    (let ((poses (round-poses (cram-pr2-pick-place-demo::make-poses-relative))))
      (move-surface :y -1)
      (lisp-unit:assert-equal goal poses))))

(define-test make-poses-relative-z-move
  (let ((goal '((1.335 0.665 1.94) ;; breakfast-cereal 
                (1.335 0.865 1.94) ;; cup 
                (1.355 1.065 1.94) ;; bowl 
                (1.385 0.915 1.79) ;; spoon 
                (2.465 0.865 1.94))));; milk
    (move-surface :z 1)
    (let ((poses (round-poses (cram-pr2-pick-place-demo::make-poses-relative))))
      (move-surface :z -1)
      (lisp-unit:assert-equal goal poses))))
