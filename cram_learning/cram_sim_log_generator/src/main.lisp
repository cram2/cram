(in-package :cslg)

(defun main (num-experiments)
  (roslisp-utilities:startup-ros)
  (setf ccl::*is-logging-enabled* t)
  (ccl::connect-to-cloud-logger)
  (loop for x from 1 to num-experiments
        do (let ((experiment-id (format nil "~x" (random (expt 16 8)))))
             (format t "Starting experiment ~a~%" experiment-id)
             (pr2-proj:with-simulated-robot (demo::demo-random))
             (ccl::export-log-to-owl (concatenate 'string experiment-id ".owl"))
             (format t "Done with experiment ~a~%" experiment-id))))
