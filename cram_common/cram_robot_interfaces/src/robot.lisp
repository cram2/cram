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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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

(in-package :cram-robot-interfaces)

(def-fact-group robot (robot
                       robot-base-frame robot-odom-frame
                       robot-torso-link-joint
                       robot-joint-states robot-pose)
  (<- (robot ?robot-name)
    (fail))

  (<- (robot-base-frame ?robot-name ?base-frame)
    (fail))

  (<- (robot-odom-frame ?robot-name ?odom-frame)
    (fail))

  (<- (robot-torso-link-joint ?robot-name ?torso-link ?torso-joint)
    (fail))

  (<- (robot-joint-states ?robot-name ?joints-group ?left-or-right-or-which
                          ?configuration-name ?joint-states)
    (fail))

  (<- (robot-pose ?robot-name ?pose-for-which-joint-group ?left-or-right-or-which
                  ?pose-name ?pose)
    (fail)))


(def-fact-group utils (arms arms-that-are-not-neck)
  (<- (arms ?robot-name ?arms)
    (once (or (setof ?arm (rob-int:arm ?robot ?arm) ?arms)
              (equal ?arms NIL))))

  (<- (arms-that-are-not-neck ?robot-name ?arms)
    (once (or (setof ?arm (and (rob-int:arm ?robot ?arm)
                               (not (rob-int:neck ?robot ?arm)))
                     ?arms)
              (equal ?arms NIL)))))


(defun current-robot-symbol ()
  (let ((robot-symbol
          (cut:var-value '?r (car (prolog:prolog '(robot ?r))))))
    (if (cut:is-var robot-symbol)
        NIL
        robot-symbol)))

(defun current-robot-package ()
  (symbol-package (current-robot-symbol)))

(defun current-robot-name ()
  (symbol-name (current-robot-symbol)))
