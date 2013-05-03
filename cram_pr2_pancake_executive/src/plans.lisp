;;; Copyright (c) 2013, Georg Bartels <georg.bartels@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
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

(in-package :pr2-pancake-ex)

(def-goal (achieve (object-flipped ?obj ?tool-left ?tool-right))
  (ros-info (achieve plan-lib) "(achieve (object-flipped))")
  (with-designators 
      ((flipping-trajectory
        (action `((type trajectory)
                  (to flip)
                  (obj-acted-on ,?obj)
                  (obj-acted-with ,?tool-left)
                  (obj-acted-with ,?tool-right))))
       (head-trajectory
        (action `((type trajectory)
                  (to see)
                  (obj ,?obj)))))
    ;; TODO(Georg): find out whether we need to use 'monitor-action'
    (with-failure-handling
        ((object-not-found (f)
           (declare (ignore f))
           (retry)))
      (ros-info (achieve plan-lib) "Pointing head at object.")
      (perform head-trajectory)
      (ros-info (achieve plan-lib) "Perceiving object.")
      (perceive-object 'currently-visible ?obj)
      (let ((perceived-desigs (perceive-object 'currently-visible ?obj)))
        (desig:equate (lazy-car perceived-desigs)
                            ?obj)
        (ros-info (achieve plan-lib) "Performing flip action.")
        (perform flipping-trajectory)))))