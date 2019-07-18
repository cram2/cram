(in-package :cslg)

(defun main (num-experiments)
  (setf cram-bullet-reasoning-belief-state:*spawn-debug-window* nil)
  (setf cram-tf:*tf-broadcasting-enabled* t)
  (roslisp-utilities:startup-ros :name "cram" :anonymous nil)
  (setf ccl::*is-logging-enabled* t)
  (setf ccl::*host* "'https://localhost'")
  (setf ccl::*cert-path* "'/home/koralewski/Desktop/localhost.pem'")
  (setf ccl::*api-key* "'K103jdr40Rp8UX4egmRf42VbdB1b5PW7qYOOVvTDAoiNG6lcQoaDHONf5KaFcefs'")
  (ccl::connect-to-cloud-logger)
  ;;(setq roslisp::*debug-stream* nil)
  (loop for x from 1 to num-experiments
        do (let ((experiment-id (format nil "~d" (truncate (* 1000000 (cram-utilities:current-timestamp))))))
             (let ((experiment-save-path
                     (concatenate 'string
                                  "~/projection-experiments/" experiment-id "/")))
               (ensure-directories-exist experiment-save-path)
               (setq ccl::*prolog-query-save-path* experiment-save-path)
               (format t "Starting experiment ~a~%" experiment-id)
               (asdf-utils:run-program (concatenate 'string "rosrun mongodb_log mongodb_log -c " experiment-id " &"))
               (unwind-protect
;;                    (urdf-proj:with-simulated-robot (demo::demo-random nil '(:bowl :spoon)))
;;                    (urdf-proj:with-simulated-robot (demo::demo-random))
                    ;;(demo::generate-training-data nil '(:cup))
                    (urdf-proj:with-simulated-robot (demo::evaluation-there-and-back-again))
                      
                 (ccl::export-log-to-owl (concatenate 'string experiment-id ".owl"))
                 (format t "Done with experiment ~a~%" experiment-id)
                 (asdf-utils:run-program (concatenate 'string "docker cp seba:/home/ros/user_data/" experiment-id ".owl " experiment-save-path))
                 (unwind-protect (asdf-utils:run-program "killall -r 'mongodb_log'"))
                 (unwind-protect (asdf-utils:run-program "killall -r 'mongod'"))
                 (ccl::reset-logged-owl))))))
