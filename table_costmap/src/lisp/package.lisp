
(in-package :cl-user)

(defpackage table-costmap
    (:use :cl :roslisp :cram-roslisp-common
          :crs :location-costmap :map-annotation
          :cram-designators)
  (:export #:*map-fl*
           #:*table-grid-cells-fl*
           #:*table-height-map-fl*
           #:on #:name #:to #:see #:reach
           #:table)
  (:import-from :cpl #:make-fluent #:value))
