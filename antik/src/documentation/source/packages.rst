.. _systems-packages:

Systems and Packages
====================

There are several ASDF systems_ that can be loaded,

- ``antik``
- ``science-data``
- ``physical-dimension``
- ``foreign-array``
- ``antik-base``

Except for ``antik``, each of these also loads the one listed after it; ``antik`` loads ``physical-dimension``. It also loads GSLL_. Most users should load these systems using Quicklisp_::

     (ql:quickload :antik)

There are two Common Lisp packages_ defined by Antik, ``antik`` and ``grid``.  Some functions defined by Antik are generalizations of functions defined by Common Lisp, so the name is chosen the same, but in one of the two new packages. Shadowing_ is used to make sure that reference to the symbol will use the Antik definition and not the CL definition.  

There is one additional package defined for the user's convience, ``antik-user``, which uses_ and properly shadows all symbols but has no definitions initially. The function :cl:function:`make-user-package` will make a new package with these features.

.. cl:package:: antik
.. cl:function:: make-user-package

.. _GSLL: https://common-lisp.net/project/gsll/
.. _quicklisp: http://www.quicklisp.org/
.. _systems: https://common-lisp.net/project/asdf/
.. _packages: http://www.lispworks.com/documentation/HyperSpec/Body/11_aa.htm
.. _shadowing: http://www.lispworks.com/documentation/HyperSpec/Body/f_shadow.htm#shadow,shadow
.. _uses: http://www.lispworks.com/documentation/HyperSpec/Body/f_use_pk.htm#use-package,use
