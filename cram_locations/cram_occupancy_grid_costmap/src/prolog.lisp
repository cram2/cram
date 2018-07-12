;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :occupancy-grid-costmap)

(defmethod costmap-generator-name->score ((name (eql 'static-occupied)))
  15)

(defmethod costmap-generator-name->score ((name (eql 'free-space)))
  15)

(def-fact-group occupancy-grid-costmap (desig-costmap)
  
  (<- (drivable-location-costmap ?cm ?padding)
    (costmap ?cm)
    (-> (symbol-value *current-map* ?map)
        (and
         (inverted-occupancy-grid ?map ?free-space)
         (occupancy-grid ?map ?static-occupied (padding ?padding))
         (costmap-add-function free-space (make-occupancy-grid-cost-function ?free-space) ?cm)
         (costmap-add-function static-occupied
                               (make-occupancy-grid-cost-function ?static-occupied :invert t)
                               ?cm))
        (true)))

  (<- (desig-costmap ?desig ?cm)
    (cram-robot-interfaces:visibility-designator ?desig)
    (costmap ?cm)
    (costmap-padding ?padding)
    (drivable-location-costmap ?cm ?padding))

  (<- (desig-costmap ?desig ?cm)
    (cram-robot-interfaces:reachability-designator ?desig)
    (costmap ?cm)
    (costmap-manipulation-padding ?padding)
    (drivable-location-costmap ?cm ?padding)))
