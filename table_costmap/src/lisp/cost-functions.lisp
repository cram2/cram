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

(in-package :table-costmap)

(defun make-table-cost-function (name &optional (std-dev 1.0))
  (let* ((mean (get-annotated-point name))
         (mean-vec (make-array 2 :initial-contents `(,(cl-transforms:x mean) ,(cl-transforms:y mean))))
         (scaled-cov (make-array '(2 2) :initial-contents `((,std-dev 0) (0 ,std-dev)))))
    (make-gauss-cost-function mean-vec scaled-cov)))
