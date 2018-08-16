(in-package :cslg)

(defun generate-neem (&optional objects-to-fetch-deliever)
  (setf cram-tf:*tf-broadcasting-enabled* t)
  (roslisp-utilities:startup-ros :name "cram" :anonymous nil)
  (setf ccl::*is-logging-enabled* t)
  (setf ccl::*host* "'https://localhost'")
  (setf ccl::*cert-path* "'/home/koralewski/Desktop/localhost.pem'")
  (setf ccl::*api-key* "'K103jdr40Rp8UX4egmRf42VbdB1b5PW7qYOOVvTDAoiNG6lcQoaDHONf5KaFcefs'")
  (ccl::connect-to-cloud-logger)

  (let ((experiment-id (format nil "~d" (truncate (* 1000000 (cram-utilities:current-timestamp))))))
    (format t "Starting experiment ~a~%" experiment-id)
    
    (unwind-protect
         (if objects-to-fetch-deliever
             (pr2-proj:with-simulated-robot (demo::demo-random nil '(:bowl :spoon)))
             (pr2-proj:with-simulated-robot (demo::demo-random)))
                 (ccl::export-log-to-owl (concatenate 'string experiment-id ".owl"))
                 (format t "Done with experiment ~a~%" experiment-id)
                 (ccl::reset-logged-owl))))
