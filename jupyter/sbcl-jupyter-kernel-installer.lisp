;; When the dependencies for common-lisp-jupyter are prepared,
;; load this file to install the jupyter kernel with
;; /usr/bin/sbcl --dynamic-space-size 4096 --load sbcl-kernel-installer.lisp

(load (parse-namestring (concatenate 'string (sb-ext:posix-getenv "ROS_ROOT") "lisp/scripts/roslisp-sbcl-init")))
(asdf:load-system :common-lisp-jupyter)
(defmethod jupyter:command-line :around ((instance jupyter/common-lisp::user-installer))
  (let* ((original-command-line (call-next-method))
         (additional-entries `("--dynamic-space-size"
                               "4096"
                               ,jupyter/common-lisp::+eval-flag+
                               "(load (parse-namestring (concatenate 'string (sb-ext:posix-getenv \"ROS_ROOT\") \"lisp/scripts/roslisp-sbcl-init\")))")))
    (concatenate 'list
                 (subseq original-command-line 0 1)
                 additional-entries
                 (subseq original-command-line 1))))
(cl-jupyter:install)
(quit)
