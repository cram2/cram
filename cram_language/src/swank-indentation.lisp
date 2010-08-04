
(in-package :cpl-impl)

(eval-when (:load-toplevel)
  (when (and (find-package "SWANK")
             (boundp (intern "*APPLICATION-HINTS-TABLES*"
                             (find-package "SWANK"))))
    (push (alist-hash-table
           '((with-failure-handling . flet)))
          (symbol-value (intern "*APPLICATION-HINTS-TABLES*"
                                (find-package "SWANK"))))))
