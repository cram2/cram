;;; KiPla - Cognitive kitchen planner and coordinator
;;; Copyright (C) 2010 by Lorenz Moesenlechner <moesenle@cs.tum.edu>
;;;
;;; This program is free software; you can redistribute it and/or modify
;;; it under the terms of the GNU General Public License as published by
;;; the Free Software Foundation; either version 3 of the License, or
;;; (at your option) any later version.
;;;
;;; This program is distributed in the hope that it will be useful,
;;; but WITHOUT ANY WARRANTY; without even the implied warranty of
;;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;;; GNU General Public License for more details.
;;;
;;; You should have received a copy of the GNU General Public License
;;; along with this program.  If not, see <http://www.gnu.org/licenses/>.

(defsystem table-costmap
  :depends-on ("cram-reasoning"
               "cram-language"
               "cram-roslisp-common"
               "location-costmap"
               "map-annotation"
               "nav_msgs-msg")
  :components
  ((:file "package")
   (:file "table-clusters" :depends-on ("package"))
   (:file "cost-functions" :depends-on ("package" "table-clusters"))
   (:file "ros-handlers" :depends-on ("package" "table-clusters"))
   (:file "facts" :depends-on ("package" "ros-handlers" "cost-functions"))))

