
(in-package :prolog)

(defun rete-prove (expr &optional bdgs)
  "Returns a list of bindings that satisfy the conjunction of tokens
  in the sense of the rete algorithm. This function is a one-shot
  request to the alpha-network and should be used where productions
  that are triggered several times and do online-processings are not
  necessary."
  ;; What when it does not hold?
  (labels ((add-bdgs (bdgs prev-bdgs &optional (result prev-bdgs))
             (cond (bdgs
                    (let ((new-bdgs (add-bdg (car (lazy-car bdgs))
                                             (cdr (lazy-car bdgs))
                                             prev-bdgs)))
                      (when new-bdgs
                        (add-bdgs (lazy-cdr bdgs) prev-bdgs new-bdgs))))
                   (t
                    (list result)))))
    (if (car expr)
        (lazy-mapcan (lambda (bdg)
                       (rete-prove (cdr expr) bdg))
                     (lazy-mapcan (rcurry #'add-bdgs bdgs)
                                  (rete-holds (substitute-vars (car expr) bdgs))))
        (list bdgs))))
