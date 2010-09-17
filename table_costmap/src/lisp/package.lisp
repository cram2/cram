
(in-package :cl-user)

(defpackage table-costmap
    (:use :cl :roslisp :cram-roslisp-common
          :crs :location-costmap :map-annotation
          :cram-designators #:designators-ros)
  (:export #:*map-fl*
           #:*table-grid-cells-fl*
           #:*table-height-map-fl*
           #:on #:name #:to #:see #:reach
           #:table #:obj #:location
           #:costmap-padding
           #:costmap-manipulation-padding
           #:drivable-location-costmap)
  (:import-from :cpl #:make-fluent #:value))
