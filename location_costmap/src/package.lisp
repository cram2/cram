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

(desig:register-designator-properties #:to #:see #:reach)

(defpackage location-costmap
    (:use :cl :crs :cut :cram-roslisp-common :alexandria
          :roslisp :designators-ros #:cram-designators)
  (:export #:make-gauss-cost-function
           #:make-location-cost-function
           #:make-range-cost-function
           #:make-occupancy-grid-cost-function
           #:make-axis-boundary-cost-function
           #:make-padded-costmap-cost-function
           #:global-fluent-value
           #:occupancy-grid
           #:inverted-occupancy-grid
           #:costmap-metadata
           #:costmap-size #:costmap-origin #:costmap-resolution
           #:costmap
           #:costmap-add-function
           #:costmap-add-generator
           #:costmap-add-heightmap
           #:costmap-add-heightmap-generator
           #:costmap-generator-name->score
           #:desig-costmap
           #:merged-desig-costmap
           #:height-map
           #:height-map-lookup
           #:height-map-set
           #:make-padding-mask
           #:point-in-padding-mask-p
           #:costmap-padding
           #:padding
           #:occupancy-grid-put-mask
           #:grid-cells-msg->occupancy-grid
           #:grid-cells-msg->height-map
           #:occupancy-grid-msg->occupancy-grid
           #:occupancy-grid-msg-metadata
           #:occupancy-grid-mean
           #:occupancy-grid-covariance
           #:location-costmap
           #:get-cost-map
           #:get-map-value
           #:invalid-probability-distribution
           #:register-cost-function
           #:generate-point
           #:gen-costmap-sample
           #:costmap-samples
           #:generators
           #:cost-functions
           #:merge-costmaps
           #:location-costmap->collision-map
           #:occupancy-grid->grid-cells-msg
           #:publish-location-costmap
           #:publish-point
           #:publish-pose
           #:costmap-location-proxy
           #:*z-padding*
           #:2d-cov #:points-mean
           #:points-cov))
