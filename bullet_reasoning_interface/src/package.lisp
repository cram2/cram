(desig-props:def-desig-package bullet-reasoning-interface
  (:nicknames :reas-inf)
  (:use
   #:cram-designators
   #:roslisp
   #:btr
   #:cpl)
   (:shadowing-import-from #:btr object pose object-pose width height)
   (:import-from #:cram-reasoning prolog force-ll)
  (:export)
  (:desig-properties
 ;;;object properties
;;   #:mondamin
;;   #:fruit-orange
;;   #:fruit-apple
;;   #:bowl
;;   #:nesquik
;;   #:sugar
   ))
