;;;
;;; Copyright (c) 2023, Tina Van <van@uni-bremen.de>
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

(in-package :demo)

(defun intermix (&key
	       ((:object ?object-designator))
	       ((:object-type ?object-type))
	       ((:object-name ?object-name))
	       ((:arms ?arms))
	       ((:grasp ?grasp))
                ((:context ?context))
                ((:rounds ?rounds))
                ((:reso ?reso))
                ((:source ?source-designator))
                ((:source-type ?source-type))
 
               ((:left-approach-poses ?left-approach-poses))
	       ((:right-approach-poses ?right-approach-poses))
	       ((:left-start-mix-poses ?left-start-mix-poses))
	       ((:right-start-mix-poses ?right-start-mix-poses))
	       ((:left-mid-mix-poses ?left-mid-mix-poses))
               ((:right-mid-mix-poses ?right-mid-mix-poses))
               ((:left-end-mix-poses ?left-end-mix-poses))
               ((:right-end-mix-poses ?right-end-mix-poses))
               ((:left-retract-poses ?left-retract-poses))
	       ((:right-retract-poses ?right-retract-poses))
             &allow-other-keys)

  (format t "mixing action designator is executable; ~%")

  ;tool to center of container
    (exe:perform
     (desig:an action
               (type approaching)
               (left-poses ?left-approach-poses)
               (right-poses ?right-approach-poses)
	       ))
    (cpl:sleep 2)

   ;mix
 
    (exe:perform
     (desig:an action
               (type blending)
               (left-poses ?left-start-mix-poses)
               (right-poses ?right-start-mix-poses)
  	       (collision-mode :allow-all)))
  (cpl:sleep 2)
  
       (exe:perform
     (desig:an action
               (type blending)
               (left-poses ?left-mid-mix-poses)
               (right-poses ?right-mid-mix-poses)
    	       (collision-mode :allow-all)))
  (cpl:sleep 2)

    (exe:perform
     (desig:an action
               (type blending)
               (left-poses ?left-end-mix-poses)
               (right-poses ?right-end-mix-poses)
    	       (collision-mode :allow-all)))
  (cpl:sleep 2)

  
   ;retract
    (exe:perform
     (desig:an action
               (type approaching)
               (left-poses ?left-retract-poses)
               (right-poses ?right-retract-poses)
    	      ))
  (cpl:sleep 2)

  )
