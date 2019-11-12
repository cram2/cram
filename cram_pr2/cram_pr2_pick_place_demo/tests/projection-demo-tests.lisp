(in-package :cram-pr2-pick-place-demo-tests)

(defparameter *relative-poses*
  '("sink_area_surface"
    ((:breakfast-cereal . ((0.2 -0.15 0.1) (0 0 0 1)))
     (:cup . ((0.2 -0.35 0.1) (0 0 0 1)))
     (:bowl . ((0.18 -0.55 0.1) (0 0 0 1)))
     (:spoon . ((0.15 -0.4 -0.05) (0 0 0 1)))
     (:milk . ((0.07 -0.35 0.1) (0 0 0 1))))))

(defun round-to (number precision &optional (what #'round))
  (let ((div (expt 10 precision)))
    (/ (funcall what (* number div)) div)))

(defun round-poses (type-pose-pairs &key (precision 3))
  (mapcar (lambda (type-pose-pair)
            (with-slots (cl-transforms:x cl-transforms:y cl-transforms:z)
                (cl-transforms:origin (cdr type-pose-pair))
              `(,(float (round-to cl-transforms:x precision))
                ,(float (round-to cl-transforms:y precision))
                ,(float (round-to cl-transforms:z precision)))))
          type-pose-pairs))

(defun move-surface (&key (x 0) (y 0) (z 0) (alpha 0.0))
  (let* ((old-pose (btr:link-pose (btr:get-environment-object) "sink_area_surface"))
         (rotated-pose (cram-tf:rotate-pose old-pose :z alpha))
         (new-pose (cram-tf:translate-pose
                    rotated-pose
                    :x-offset x :y-offset y :z-offset z)))
    (setf (btr:link-pose (btr:get-environment-object) "sink_area_surface")
          new-pose)))


(define-test make-poses-relative-default
  (let* ((goal '((1.335 0.44 0.94) ;; breakfast-cereal
                 (1.335 0.64 0.94) ;; cup
                 (1.355 0.84 0.94) ;; bowl
                 (1.385 0.69 0.79) ;; spoon
                 (1.465 0.64 0.94))) ;; milk
         (poses (round-poses
                 (cram-pr2-pick-place-demo::make-poses-relative *relative-poses*))))
    (lisp-unit:assert-equal goal poses)))


(define-test make-poses-relative-x-move
  (let ((goal '((2.335 0.44 0.94) ;; breakfast-cereal
                (2.335 0.64 0.94) ;; cup
                (2.355 0.84 0.94) ;; bowl
                (2.385 0.69 0.79) ;; spoon
                (2.465 0.64 0.94)))) ;; milk
    (move-surface :x 1)
    (unwind-protect
         (let ((poses (round-poses
                       (cram-pr2-pick-place-demo::make-poses-relative *relative-poses*))))
           (lisp-unit:assert-equal goal poses))
      (move-surface :x -1))))

(define-test make-poses-relative-y-move
  (let ((goal '((1.335 1.44 0.94) ;; breakfast-cereal
                (1.335 1.64 0.94) ;; cup
                (1.355 1.84 0.94) ;; bowl
                (1.385 1.69 0.79) ;; spoon
                (1.465 1.64 0.94)))) ;; milk
    (move-surface :y 1)
    (unwind-protect
         (let ((poses (round-poses
                       (cram-pr2-pick-place-demo::make-poses-relative *relative-poses*))))
           (lisp-unit:assert-equal goal poses))
      (move-surface :y -1))))

(define-test make-poses-relative-z-move
  (let ((goal '((1.335 0.44 1.94) ;; breakfast-cereal
                (1.335 0.64 1.94) ;; cup
                (1.355 0.84 1.94) ;; bowl
                (1.385 0.69 1.79) ;; spoon
                (1.465 0.64 1.94)))) ;; milk
    (move-surface :z 1)
    (unwind-protect
         (let ((poses (round-poses
                       (cram-pr2-pick-place-demo::make-poses-relative *relative-poses*))))
           (lisp-unit:assert-equal goal poses))
      (move-surface :z -1))))


(define-test make-poses-relative-alpha-move
  (let ((goal '((1.735 0.14 0.94) ;; breakfast-cereal
                (1.735 -0.06 0.94) ;; cup
                (1.715 -0.26 0.94) ;; bowl
                (1.685 -0.11 0.79) ;; spoon
                (1.605 -0.06 0.94)))) ;; milk
    (move-surface :alpha pi)
    (unwind-protect
         (let ((poses (round-poses
                       (cram-pr2-pick-place-demo::make-poses-relative *relative-poses*))))
           (lisp-unit:assert-equal goal poses))
      (move-surface :alpha (- pi)))))
