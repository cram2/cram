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

(def-fact-group ptu (camera-frame camera-minimal-height camera-maximal-height
                                  robot-pan-tilt-links robot-pan-tilt-joints
                                  robot-neck-parking-joint-states
                                  robot-neck-looking-joint-states)
  ;; Unifies ?frame with the name of the camera frame present on the ?robot
  (<- (camera-frame ?robot ?frame)
    (fail))

  ;; ?min-height is possible minimal distance (in meters) of the camera from
  ;; the 'ground'.
  ;; E.g. for PR2 it's the Z pose when the torso is maximally down.
  ;; For a quadrotor it will be close to 0, as it can stand on the ground
  (<- (camera-minimal-height ?robot ?min-height)
    (fail))
  ;; ?max-height is the opposite of ?min-height, the maximal possible
  ;; distance from the ground. For quadrotors it will be the maximal
  ;; distance it can fly off the ground...
  (<- (camera-maximal-height ?robot ?max-height)
    (fail))

  (<- (robot-pan-tilt-links ?robot ?pan-link ?tilt-link)
    (fail))
  (<- (robot-pan-tilt-joints ?robot ?pan-joint ?tilt-joint)
    (fail))

  (<- (robot-neck-parking-joint-states ?robot ?max-height)
    (fail))
  (<- (robot-neck-looking-joint-states ?robot ?max-height)
    (fail)))
