Statistics
==========

Given a function ``#'make-orbit`` which takes several keyword arguments ``:semimajor-axis``, ``:eccentricity`` and ``:inclination``, the is example of ``low-discrepancy-sample`` will make five orbits with values in the specified ranges::

   (low-discrepancy-sample
      5
      'make-orbit
      '(:semimajor-axis #_7000_km #_9000_km) 
      '(:eccentricity 0.0)
      '(:inclination #_95_deg #_105_deg))

.. cl:package:: antik
.. cl:function:: low-discrepancy-sample 
.. cl:function:: apply-to-arguments 
.. cl:function:: list-no-key
