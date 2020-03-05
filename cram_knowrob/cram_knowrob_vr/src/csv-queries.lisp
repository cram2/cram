;;;
;;; Copyright (c) 2019, Thomas Lipps <tlipps@uni-bremen.de>
;;;
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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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

(in-package :kvr)

(defun get-umap-T-uobj (sample-for-object-type)
  "Returns the transformation of given sample from map to obj for
a given sample `sample-for-object-type'."
  (when sample-for-object-type
    (let* ((surface-name
             (first sample-for-object-type))
           (ssurface-T-sobject
             (second sample-for-object-type))
           (rigid-body 
             (btr:rigid-body
              (btr:get-environment-object)
              (match-kitchens surface-name)))
           (umap-T-usurface
             (when rigid-body
               (cl-transforms:pose->transform
                (btr:pose rigid-body)))))
      (when (and umap-T-usurface
                 ssurface-T-sobject)
        (cl-transforms:transform*
         umap-T-usurface ssurface-T-sobject)))))
       
(defun umap-T-uobj->uobj-position (umap-T-uobj)
  "Returns the position of object with given transformation `umap-to-uobj'"
  (when umap-T-uobj
    (cl-transforms:origin
     (cl-transforms-stamped:make-pose-stamped
      cram-tf:*fixed-frame*
      0.0
      (cl-transforms:translation umap-T-uobj)
      (cl-transforms:rotation umap-T-uobj)))))

(defun samples-for-object-type (type)
  "Returns all valid samples recorded in the VR Kitchen with given
  object type `type'"
  (let* ((sample-for-object-type-ll
           (kvr::query-for-csv-export
            type
            "End"
            )))
      
      (remove-if-not (alexandria:curry #'every #'identity)
       (cut:force-ll
       (cut:lazy-mapcar
        (lambda (sample-for-object-type)
          (let* ((ep-inst-and-obj-short-and-inst-name
                   (last sample-for-object-type 3))
                 ;; the episode name from sample-for-object-type
                 (ep-inst-name (car ep-inst-and-obj-short-and-inst-name))
                 ;; the object short instance name from sample-for-object-type
                 (obj-short-name (car (cdr ep-inst-and-obj-short-and-inst-name)))
                 ;; the object instance name from sample-for-object-type
                 (obj-inst-name (car (cdr (cdr ep-inst-and-obj-short-and-inst-name))))
                 ;; the symbolic storage location from sample-for-object-type
                 (object-from
                   (cut:lazy-car (query-contact-surface-name type "Start" ep-inst-name)))
                 ;; The Start sample with given object type, episode name
                 ;; and object instance name
                 (start-sample-for-object-instance (kvr::filter-query-for-csv-export
                                                    type
                                                    "Start"
                                                    nil
                                                    :obj-short-name
                                                    obj-short-name
                                                    :ep-inst 
                                                    ep-inst-name
                                                    ;; :obj-inst 
                                                    ;; obj-inst-name
                                                    ))
                 ;; The subsymbolic destination and storage transformations
                 ;; from map to object from sample-for-object-type
                 (umap-T-uobj-dest (get-umap-T-uobj
                                    sample-for-object-type))
                 (umap-T-uobj-storage (get-umap-T-uobj
                                       (cut:lazy-car start-sample-for-object-instance)))
                 ;; The subsymbolic destination and storage location
                 ;; from sample-for-object-type
                 (obj-dest-position (umap-T-uobj->uobj-position
                                     umap-T-uobj-dest))
                 (obj-storage-position (umap-T-uobj->uobj-position
                                        umap-T-uobj-storage))
                 (obj-dest-orientation (fourth 
                                        sample-for-object-type))
                 (obj-storage-orientation (fourth
                                           (cut:lazy-car start-sample-for-object-instance))))
            (roslisp:ros-info (kvr export) "Got sample for ~A" obj-short-name)
            (list
             type
             object-from
             (if obj-storage-position
                 (cl-transforms:x obj-storage-position)
                 nil)
             (if obj-storage-position
                 (cl-transforms:y obj-storage-position)
                 nil)
             obj-storage-orientation
             (nth 0 sample-for-object-type) ;; symbolic destination location
             (if obj-dest-position
                 (cl-transforms:x obj-dest-position)
                 nil)
             (if obj-dest-position
                 (cl-transforms:y obj-dest-position)
                 nil)
             obj-dest-orientation
             (third sample-for-object-type) ;; arm
             )))
        sample-for-object-type-ll)))))

;; TODO Maybe save query-for-csv-exports in hash-table with keys being
;; the object-type and cache these
;; Hash-table cache should be deleteable with an init-cache
(defun filter-query-for-csv-export (object-type start-or-end
                                    &optional context 
                                    &key obj-short-name
                                      ep-inst obj-inst)
  "Gets the output from `query-for-csv-export' with given
`object-type', `start-or-end' and `context' and then filters this
output with given `obj-short-name', `ep-inst' and
`obj-inst'. `obj-type', `obj-short-name', `ep-inst' and `obj-inst'
have to come from the semantic map representing semantically the VR
Kitchen in OWL."

  (let ((samples (cut:force-ll
                  (query-for-csv-export object-type
                                        start-or-end
                                        context))))
    (if  (and (or obj-short-name obj-inst)
              ep-inst)
         (remove-if-not (lambda (sample)
                          (let ((ep-inst-obj-short-name-obj-inst-list (last
                                                                       sample
                                                                       3)))
                            (and (or 
                                  (search obj-inst (third ep-inst-obj-short-name-obj-inst-list))
                                  (search obj-short-name (second ep-inst-obj-short-name-obj-inst-list)))
                                 (search ep-inst (first ep-inst-obj-short-name-obj-inst-list)))))
                        samples)
         samples)))

(defun query-for-csv-export (object-type start-or-end &optional context)
  (declare (type string object-type start-or-end)
           (type (or null keyword) context))
  "Returns the OWL type of the supporting surface name an object with
   type `object-type' is picked up from or placed. Moreover, it
   returns the transform surface-T-object, the used robot arm, the
   z angle of the object, the episode name and the short and long name
   of the object instance. All these values are returned in the
   described order in a lazy list."
  (assert (or (equal start-or-end "Start") (equal start-or-end "End")))
  (cut:lazy-mapcar
   (lambda (binding-set)
     (let* ((surface-name
              (cut:var-value '|?SurfaceTypeShortName| binding-set))
            (map-T-object
              (cram-tf:flat-list->transform
               (cut:var-value '|?ObjectPose| binding-set)))
            (map-T-surface
              (cram-tf:flat-list->transform
               (cut:var-value '|?SurfacePose| binding-set)))
            (surface-T-map
              (cl-transforms:transform-inv
               map-T-surface))
            (surface-T-object
              (cl-transforms:transform*
               surface-T-map map-T-object))
            (map-T-camera
              (cram-tf:flat-list->transform
               (cut:var-value '|?CameraPose| binding-set)))
            (obj-T-map
              (cl-transforms:transform-inv
               map-T-object))
            (obj-angle
              (let ((angle (car
                            (last ;; get z rotation
                             (cl-tf:quaternion->euler
                              (cl-transforms:rotation map-T-object))))))
              ;; (multiple-value-bind (vector angle)
              ;;     (cl-tf:quaternion->axis-angle
              ;;      (cl-tf:normalize (cl-transforms:rotation
              ;;                        map-T-object)))
                angle))
            (hand-string
              (string
               (cut:var-value '|?HandTypeName| binding-set)))
            (EpInst-name
              (first (remove-if-not (alexandria:curry #'search "UnrealExperiment")
                                    (split-sequence:split-sequence-if
                                     (lambda (c)
                                       (or (equal c #\')
                                           (equal c #\#)))
                                     (string (cut:var-value '|?EpInst|
                                                            binding-set))))))
            (ObjInst-name
              (string (cut:var-value '|?ObjInst|
                                     binding-set)))
            (ObjShort-name
              (first (remove-if-not (lambda (e)
                                      (and (search "_" e)
                                           (not (search "http" e))))
                                    (split-sequence:split-sequence-if
                                     (lambda (c)
                                       (or (equal c #\')
                                           (equal c #\#)))
                                     ObjInst-name)))))
       (if (search "Left" hand-string)
           (setf hand-string :left)
           (if (search "Right" hand-string)
               (setf hand-string :right)
               (setf hand-string NIL)))

       (list surface-name surface-T-object hand-string obj-angle
             EpInst-name ObjShort-name ObjInst-name)))



   (json-prolog:prolog-simple
    (concatenate
     'string
     (base-query-string object-type) ",
      performed_by(EventInst, HandInst),
      obj_type(HandInst, HandType),
      iri_xml_namespace(HandType, _, HandTypeName),
      event_type(TouchingEventInst, knowrob_u:'TouchingSituation'),
      rdf_has(TouchingEventInst, knowrob_u:'inContact', ObjInst),
      rdf_has(TouchingEventInst, knowrob_u:'inContact', Surface),
      not(ObjInst==Surface),
      u_occurs(EpInst, TouchingEventInst, TouchStart, TouchEnd),
      StartWithOffset = Start,
      time_term(End, EndSeconds),
      EndWithOffset is EndSeconds + 1,
      time_between(" start-or-end "WithOffset, TouchStart, TouchEnd),
      iri_xml_namespace(Surface, _, SurfaceShortName),
      actor_pose(EpInst, SurfaceShortName, Touch" start-or-end ", SurfacePose),
      iri_xml_namespace(ObjInst, _, ObjShortName),
      actor_pose(EpInst, ObjShortName, " start-or-end ", ObjectPose),
      obj_type(Surface, SurfaceType),
      iri_xml_namespace(SurfaceType, _, SurfaceTypeShortName),
      iri_xml_namespace(ObjInst, _, ObjShortName),
      actor_pose(EpInst, ObjShortName, " start-or-end ", ObjPose),
      obj_type(CameraInst, knowrob:'CharacterCamera'),
      iri_xml_namespace(CameraInst, _, CameraShortName),
      actor_pose(EpInst, CameraShortName, " start-or-end ", CameraPose)"  
      (case context
        ;; if we're setting a table, the source ("Start") should not be island,
        ;; and destination ("End") should be island
        (:table-setting
         (if (equal start-or-end "Start")
             ", not(owl_subclass_of(SurfaceType, knowrob:'IslandArea'))."
             ", owl_subclass_of(SurfaceType, knowrob:'IslandArea')."))
        ;; in case of table cleaning it's the other way around:
        ;; source "Start" should be island, destination "End" should not be island
        (:table-cleaning
         (if (equal start-or-end "Start")
             ", owl_subclass_of(SurfaceType, knowrob:'IslandArea')."
             ", not(owl_subclass_of(SurfaceType, knowrob:'IslandArea'))."))
        (t
         ".")))
    :package :kvr)))
     
