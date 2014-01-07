(in-package :cl-user)

(desig-props:def-desig-package cram-beginner-tutorial
  (:nicknames :tut)
  (:use #:cpl #:roslisp #:cl-transforms #:cram-reasoning #:cram-designators #:cram-plan-library
        #:cram-plan-failures
        #:cram-plan-knowledge

)
(:shadowing-import-from :cpl #:fail #:< #:> #:+ #:- #:* #:/ #:= #:eq #:eql #:not )
(:desig-properties
;;;action properties
    #:action
    #:shape 
    #:radius 
    #:triangle 
    #:square
    #:pentagon  
    #:hexagon
;;;location Properties
    #:location
    #:navigation
    #:turtle-position
    #:goal
    #:vpos
    #:hpos
    #:left
    #:right
    #:top
    #:bottom
    #:center

)

)
