;;;
;;; Copyright (c) 2022, Tina Van <van@uni-bremen.de>
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

;:WIP
(defun whip (&key
	       ((:object ?current-object-desig))
	       ((:object-type ?object-type))
	       ((:object-name  ?object-name))
	       ((:arms ?arms))
	       ((:effort ?effort))
	       ((:grasp ?grasp))
	       ((:left-whip-approach-poses ?left-whip-approach-poses))
	       ((:right-whip-approach-poses ?right-whip-approach-poses))
	       ((:collision-mode ?collision-mode))
	     &allow-other-keys)
        
  (roslisp:ros-info (cut-pour pour) "Approaching")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (cut-and-pour-plans pour)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         ;; (return)
         ))
    (exe:perform
     (desig:an action
               (type approaching)
               (left-poses ?left-whip-approach-poses)
               (right-poses ?right-whip-approach-poses)
               (desig:when ?collision-mode
                 (collision-mode ?collision-mode)))))
  )

