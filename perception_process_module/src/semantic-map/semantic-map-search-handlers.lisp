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

(defclass semantic-map-object ()
  ((pose :initarg :pose :reader object-pose)
   (desig :initarg :desig :accessor object-desig :initform nil)
   (timestamp :initarg :timestamp :reader object-timestamp)
   (properties :initarg :properties :reader object-properties)
   (probability :initform 0.0 :accessor perceived-object-probability)
   (position-idx :initarg :position-idx
                 :reader position-idx)
   (height-idx :initarg :height-idx
               :reader height-idx)))

(defun link-name->pose (link-name)
  (let ((tf (tf:lookup-transform *tf* :target-frame "/map" :source-frame link-name)))
    (tf:make-pose-stamped (tf:frame-id tf) 0
                          (cl-transforms:translation tf)
                          (cl-transforms:rotation tf))))

(def-fact-group semantic-map-obj-desigs ()
  (<- (link-pose ?link-name ?pose)
    (bound ?link-name)
    (lisp-fun link-name->pose ?link-name ?pose))
  
  (<- (sem-map-obj-position left-of-sink "/ias_kitchen/drawer_sink_col1_center_link" 4))
  (<- (sem-map-obj-position fridge "/ias_kitchen/handle_drawer_fridge_top_link" 8))
  (<- (height low 0))
  (<- (height middle 1))
  (<- (height top 2))
  (<- (height fridge 3))
  
  (<- (semantic-map-obj ?desig ?descr)
    (desig-prop ?desig (type fridge))
    (instance-of semantic-map-object ?descr)
    (sem-map-obj-position fridge ?frame ?p-id)
    (height fridge ?h-id)
    (link-pose ?frame ?pose)
    (slot-value ?descr pose ?pose)
    (slot-value ?descr position-idx ?p-id)
    (slot-value ?descr height-idx ?h-id)
    (lisp-fun cut:current-timestamp ?timestamp)
    (lisp-fun description ?desig ?props)
    (slot-value ?descr timestamp ?timestamp)
    (slot-value ?descr properties ?props))

  (<- (semantic-map-obj ?desig ?descr)
    (desig-prop ?desig (type drawer))
    (desig-prop ?desig (sem-map-obj-position ?p))
    (desig-prop ?desig (height ?h))
    (sem-map-obj-position ?p ?frame ?p-id)
    (height ?h ?h-id)
    (link-pose ?frame ?pose)
    (instance-of semantic-map-object ?descr)
    (slot-value ?descr pose ?pose)    
    (slot-value ?descr position-idx ?p-id)
    (slot-value ?descr height-idx ?h-id)
    (lisp-fun cut:current-timestamp ?timestamp)
    (lisp-fun description ?desig ?props)
    (slot-value ?descr timestamp ?timestamp)
    (slot-value ?descr properties ?props)))

(defmethod make-new-desig-description ((old-desig object-designator)
                                       (po semantic-map-object))
  (let ((obj-loc-desig (make-designator 'location `((pose ,(object-pose po))))))
    (cons `(at ,obj-loc-desig)
          (remove 'at (description old-desig) :key #'car))))

(defun assert-semantic-map-desig (desig)
  (with-vars-bound (?obj)
      (lazy-car (prolog `(semantic-map-obj ,desig ?obj)))
    (unless (is-var ?obj)
      (assert-perceived-object ?obj (description desig))
      ?obj)))

(defmethod object-search-function ((type (cl:eql 'fridge)) desig &optional perceived-object)
  (declare (ignore perceived-object))  
  (assert-semantic-map-desig desig))

(defmethod object-search-function ((type (cl:eql 'drawer)) desig &optional perceived-object)
  (declare (ignore perceived-object))
  (assert-semantic-map-desig desig))
