;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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
;;;

(in-package :location-costmap)

(def-fact-group location-costmap-utils ()
  (<- (global-fluent-value ?name ?value)
    (symbol-value ?name ?fl)
    (lisp-fun cpl:value ?fl ?value)
    (lisp-pred identity ?value)))

(def-fact-group location-costmap ()
  
  (<- (occupancy-grid ?msg ?grid)
    (occupancy-grid ?msg ?grid (padding nil)))

  (<- (inverted-occupancy-grid ?msg ?grid)
    (inverted-occupancy-grid ?msg ?grid (padding nil)))
  
  (<- (occupancy-grid ?msg ?grid (padding ?p))
    (not (bound ?grid))
    (bound ?msg)
    (lisp-type ?msg nav_msgs-msg:occupancygrid)
    (lisp-fun occupancy-grid-msg->occupancy-grid ?msg :padding ?p ?grid))

  (<- (inverted-occupancy-grid ?msg ?grid (padding ?p))
    (not (bound ?grid))
    (bound ?msg)
    (lisp-type ?msg nav_msgs-msg:occupancygrid)
    (lisp-fun occupancy-grid-msg->occupancy-grid ?msg :padding ?p :invert t ?grid))

  (<- (occupancy-grid ?msg ?grid (padding ?p))
    (not (bound ?grid))
    (bound ?msg)
    (lisp-type ?msg nav_msgs-msg:gridcells)
    (lisp-fun grid-cells-msg->occupancy-grid ?msg ?p ?grid))

  (<- (inverted-occupancy-grid ?msg ?grid (padding ?p))
    (not (bound ?grid))
    (bound ?msg)
    (lisp-type ?msg nav_msgs-msg:gridcells)
    (lisp-fun grid-cells-msg->occupancy-grid ?msg ?p ?tmp-grid)
    (lisp-fun invert-occupancy-grid ?tmp-grid ?grid)))

(def-fact-group location-costmap-desigs (desig-loc)
  (<- (desig-loc ?desig (costmap ?cm))
    (loc-desig? ?desig)
    (bagof ?c (desig-costmap ?desig ?c) ?costmaps)
    (lisp-fun merge-costmaps ?costmaps ?cm)))
