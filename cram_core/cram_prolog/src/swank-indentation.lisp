
(in-package :prolog)

(eval-when (:load-toplevel)
  (when (and (find-package "SWANK")
             (boundp (intern "*APPLICATION-HINTS-TABLES*"
                             (find-package "SWANK"))))
    (push (alexandria:alist-hash-table
           '((with-production . let)
             (with-production-handlers . flet)))
          (symbol-value (intern "*APPLICATION-HINTS-TABLES*"
                                (find-package "SWANK"))))))
