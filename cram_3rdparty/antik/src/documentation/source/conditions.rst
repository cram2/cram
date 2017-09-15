Conditions
==========

Conditions helpful for arithmetic calculations are provided. Complex number are sometimes undesirable; ``making-complex-number`` is a condition that will be signalled when a computation will produce a complex number. The macro ``handling-complex-number`` allows various restarts which can be selected, depending on the desired action. For example::

  (handling-complex-number return-zero
    (/ (sin x)
       (* pi (sqrt (+ (expt (cos y) 2)
		      (expt (* (cos z) (sin y)) 2)
		      (- (expt (cos x) 2)))))))

will return zero if from the body form if any calculation inside returns a complex number.

.. cl:package:: antik
.. FAIL cl:condition:: coerce-undefined
.. FAIL cl:condition:: coerce-nil
.. FAIL cl:condition:: making-complex-number
.. cl:macro:: handling-complex-number 
.. cl:macro:: arithmetic-errors-return-nan
