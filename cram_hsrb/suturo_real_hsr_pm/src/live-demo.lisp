(in-package :su-real)

;; author Luca Krohm
(defun demo (demo)
  "Receives keyword 'demo' and executes the corresponding demo within the correct environment "

  (defvar *plan* nil)
  (case demo
    ;; saves the plan corresponding to the keyworld to be executed later
    (:clean (setf *plan* (list #'su-demos::clean-the-table-demo2)))
    (:groceries (setf *plan* (list #'su-demos::storing-groceries-demo2)))
    (:breakfast (setf *plan* (list #'su-demos::serve-breakfast-demo)))
    (:all (setf *plan* (list #'su-demos::storing-groceries-demo2
                                #'su-demos::serve-breakfast-demo
                                #'su-demos::clean-the-table-demo2)))
    (otherwise (roslisp:ros-error (run-demo)
                                  "Demo ~a is not a valid demo!"
                                  demo)))
  ;; starts the plan within the correct environment
   (with-hsr-process-modules
    (unwind-protect
         (mapc #'funcall *plan*))))
