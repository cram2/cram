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

(defsystem cram-location-costmap
  :depends-on (alexandria
               cram-prolog
               cram-language
               cram-math
               cram-utilities
               roslisp-utilities
               cl-transforms
               cl-transforms-stamped
               nav_msgs-msg
               visualization_msgs-msg
               trivial-garbage
               cram-tf ; for visualizing markers in the fixed frame
               )
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "occupancy-grid" :depends-on ("package"))
             (:file "padding-mask" :depends-on ("package"))
             (:file "cost-functions"
              :depends-on ("package" "occupancy-grid" "padding-mask"))
             (:file "costmap-generators" :depends-on ("package" "occupancy-grid"))
             (:file "location-costmap"
              :depends-on ("package" "occupancy-grid" "costmap-generators"))
             (:file "2d-value-map" :depends-on ("package" "occupancy-grid"))
             (:file "location-prolog-handlers"
              :depends-on ("package" "location-costmap" "2d-value-map"))
             (:file "designator-integration" :depends-on ("package" "location-costmap")) 
             (:file "facts" :depends-on ("package" "cost-functions"))
             (:file "cost-function-utils" :depends-on ("package"))
             (:file "ros-grid-cells" :depends-on ("package" "2d-value-map"))
             (:file "ros-occupancy-grid" :depends-on ("package"))
             (:file "visualization"
              :depends-on ("package" "occupancy-grid" "location-costmap"))))))
