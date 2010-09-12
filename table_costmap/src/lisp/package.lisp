
(in-package :cl-user)

(defpackage table-costmap
    (:use :cl :roslisp :cram-roslisp-common
          :crs :location-costmap :map-annotation)
  (:export #:*map-fl*
           #:*table-grid-cells-fl*
           #:*table-height-map-fl*)
  (:import-from :cpl #:make-fluent #:value))
