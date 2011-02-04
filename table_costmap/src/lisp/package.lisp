
(in-package :cl-user)

(defpackage table-costmap
    (:use :cl :roslisp :cram-roslisp-common
          :crs :location-costmap :map-annotation
          :cram-designators #:designators-ros)
  (:export #:*map-fl*
           #:*table-grid-cells-fl*
           #:*table-height-map-fl*
           #:on #:name #:to #:see #:reach
           #:table #:obj #:location #:counter
           #:in #:reach
           #:costmap-padding
           #:costmap-manipulation-padding
           #:costmap-in-reach-padding
           #:drivable-location-costmap
           #:table-properties
           #:*initial-stddev*
           #:*accept-threshold*
           #:cluster-tables
           #:get-table-cluster
           #:init-table-clusters
           #:table-cluster
           #:cov #:mean
           #:*initial-stddev*
           #:*accept-threshold*)
  (:import-from :cpl #:make-fluent #:value))
