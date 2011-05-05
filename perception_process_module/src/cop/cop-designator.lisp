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

(in-package :perception-pm)

(defstruct cop-desig-query-info
  object-classes
  object-ids
  poses
  (matches 20) ;Number of objects to match maximally
  )

(defstruct cop-desig-location-info
  poses)

(defstruct cop-desig-info
  designator
  query
  location)

(defvar *cop-desig-ignore-props* '(at)
  "Designator properties that are not used by cop.")

(defun cop-ignore-property-p (prop)
  (let ((prop (etypecase prop
                (symbol prop)
                (list (car prop)))))
    (and (member prop *cop-desig-ignore-props*)
         t)))

(defun ensure-desig-info (info desig)
  (or info (make-cop-desig-info
              :query (make-cop-desig-query-info)
              :location (make-cop-desig-location-info)
              :designator desig)))

(defmethod resolve-designator ((desig object-designator) (role (eql 'cop)))
  (let ((desig-info (ensure-desig-info result desig)))
    (setf (cop-desig-query-info-object-classes (cop-desig-info-query desig-info))
          (nconc (mapcar (compose #'rosify-lisp-name #'cadr)
                         (remove-if #'cop-ignore-property-p (description desig)))
                  (cop-desig-query-info-object-classes (cop-desig-info-query desig-info))))
    desig-info))

;; object_classes: Cluster, IceTea, Mug, red, black, Jug
;; (register-object-desig-resolver type :cop (result desig)
;;   (with-desig-props (type) desig
;;     (let ((desig-info (ensure-desig-info result desig)))
;;       (when type
;;         (push (ccase type
;;                 (mug "Mug")
;;                 (icetea "IceTea")
;;                 (cluster "Cluster")
;;                 (jug "Jug")
;;                 (placemat "PlaceMat")
;;                 (coke "Coke"))
;;               (cop-desig-query-info-object-classes (cop-desig-info-query desig-info))))
;;       desig-info)))

;; (register-object-desig-resolver color :cop (result desig)
;;   (with-desig-props (color) desig
;;     (let ((desig-info (ensure-desig-info result desig)))
;;       (when color
;;         (push (ccase color
;;                 (black "black")
;;                 (red "red")
;;                 (white "white")
;;                 (blue "blue")
;;                 (green "green"))
;;               (cop-desig-query-info-object-classes (cop-desig-info-query desig-info))))
;;       desig-info)))

;; (register-object-desig-resolver at :cop (result desig)
;;   ;; (at <loc-desig>). loc-desig must be a resolved location
;;   ;; designator, i.e. it must return a valid pose (as lo id)
;;   (with-desig-props (at) desig
;;     (let ((desig-info (ensure-desig-info result desig)))
;;       (when at
;;         (setf (cop-desig-location-info-poses (cop-desig-info-location desig-info))
;;               (loop with loc = at
;;                  while loc                   
;;                  collecting (reference loc)
;;                  do (setf loc (next-solution loc)))))
;;       desig-info)))

