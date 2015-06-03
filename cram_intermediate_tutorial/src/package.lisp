(in-package :cl-user)

(desig-props:def-desig-package cram-intermediate-tutorial
  (:nicknames :tuti)
  (:use #:cpl
        #:roslisp
        #:cl-transforms
        #:cram-designators)
  (:import-from #:cram-reasoning #:<-)

  (:export
   init-cram-moveit-tutorial
   *cube-mesh*
   *pose-mid*
   *pose-cube*
   *pose-right*
   *pose-right-msg*
   *start-robot-pose*
   *planned-trajectory*)

  (:desig-properties
   ;; action properties
   #:shape
   #:navigation
   #:goal
   #:radius
   #:triangle
   #:square
   #:pentagon
   #:hexagon
   ;; location Properties
   #:vpos
   #:hpos
   #:left
   #:right
   #:top
   #:bottom
   #:center))
