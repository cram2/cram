(defpackage :suturo-real-hsr-pm
  (:nicknames :su-real)
  ;;(:use #:common-lisp :roslisp :cpl)
  (:use #:common-lisp #:cram-prolog #:cram-designators #:cram-executive #:cram-giskard)
  (:export
   #:with-real-hsr-pm
   #:wait-for-startsignal
   ;; Make sure that exported functions are always working and up to date
   ;; Functions that are not exported may be WIP

   #:demo

   ;; designator plan functions
   #:pick-up
   #:place
   #:open-door
   #:open-gripper
   #:close-gripper

   #:su-pour
   
))
