(in-package :cl-user)

(desig-props:def-desig-package cram-beginner-tutorial
  (:nicknames :tut)
  (:use #:cpl
        #:roslisp
        #:cl-transforms
        #:cram-designators)

(:desig-properties
;;;action properties
    #:shape
    #:radius
    #:triangle
    #:square
    #:pentagon
    #:hexagon
;;;location Properties
    #:vpos
    #:hpos
    #:left
    #:right
    #:top
    #:bottom
    #:center))
