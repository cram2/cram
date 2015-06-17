;;;
;;; Copyright (c) 2015, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :cl-user)

(desig-props:def-desig-package bullet-reasoning-utilities
  (:use #:common-lisp #:bullet-reasoning #:cram-reasoning #:cram-designators
        #:cram-utilities)
  (:shadowing-import-from #:btr object object-pose)
  (:export
   ;; misc.lisp
   clear-costmap-viz
   ;; object-database.lisp
   scenario-objects-init-pose scenario-objects-default-color scenario-object-color
   scenario-object-shape scenario-object-extra-attributes
   ;; objects.lisp
   spawn-object kill-object kill-all-objects move-object move-object-onto
   object-instance object-pose object-exists household-object-exists
   assign-object-pos assign-object-pos-on
   ;; plan-library.lisp
   find-object-on-counter
   ;; robot.lisp
   robot-name move-robot move-robot-away
   ;; setup.lisp
   semantic-map-object-name init start-ros-and-bullet)
  (:desig-properties #:on #:name #:type #:at
                     #:plate #:fork #:knife #:mug #:pot #:bowl #:mondamin
                     #:spatula #:pancake-maker #:orange #:apple #:sugar-box
                     #:cereal))
