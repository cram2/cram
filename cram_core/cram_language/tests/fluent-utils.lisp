(in-package :cpl-tests)


(defun end-of-time ()
  "The Omega. Use (WAIT-FOR (END-OF-TIME)) to wait infinitively."
  (make-fluent :name "END-OF-TIME" :value nil))

(defun fluent (value)
  "Very short form to be used when writing test cases."
  (make-fluent :name "ANONYMOUS" :value value))

