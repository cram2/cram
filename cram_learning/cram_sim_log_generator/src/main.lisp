(in-package :cslg)

(defun main (num-experiments)
  (setf cram-bullet-reasoning-belief-state:*spawn-debug-window* nil)
  (roslisp-utilities:startup-ros)
  (setf ccl::*is-logging-enabled* t)
  (ccl::connect-to-cloud-logger)
  (setq roslisp::*debug-stream* nil)
  (loop for x from 1 to num-experiments
        do (let ((experiment-id (format nil "~x" (random (expt 16 8)))))
             (let ((experiment-save-path
                     (concatenate 'string
                                  "~/projection-experiments/" experiment-id "/")))
               (ensure-directories-exist experiment-save-path)
               (setq ccl::*prolog-query-save-path* experiment-save-path)
               (format t "Starting experiment ~a~%" experiment-id)
               (pr2-proj:with-simulated-robot (demo::demo-random))
               (ccl::export-log-to-owl (concatenate 'string experiment-id ".owl"))
               (format t "Done with experiment ~a~%" experiment-id)
               (asdf-utils:run-program (concatenate 'string "docker cp seba:/home/ros/user_data/" experiment-id ".owl " experiment-save-path))
               (ccl::merge-all-csv-files)))))
