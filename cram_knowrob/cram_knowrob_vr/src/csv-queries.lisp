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

(defun samples-for-object-type (type)
  (let* ((sample-for-object-type-ll
           (kvr::query-for-csv-export
            (object-type-filter-prolog type)
            "End"
            :table-setting))
         (epinst-name
           (first (last (car sample-for-object-type-ll))))
         (object-from
           (query-contact-surface-name type "Start" epinst-name)))
    (cut:force-ll
     (cut:lazy-mapcar
      (lambda (sample-for-object-type)
        (let* ((surface-name
                 (first sample-for-object-type))
               (ssurface-T-sobject
                 (second sample-for-object-type))
               (umap-T-usurface
                 (cl-transforms:pose->transform
                  (btr:pose
                   (btr:rigid-body
                    (btr:get-environment-object)
                    (match-kitchens surface-name)))))
               (umap-T-uobj
                 (cl-transforms:transform*
                  umap-T-usurface ssurface-T-sobject))
               (obj-position
                 (cl-transforms:origin
                  (cl-transforms-stamped:make-pose-stamped
                   cram-tf:*fixed-frame*
                   0.0
                   (cl-transforms:translation umap-T-uobj)
                   (cl-transforms:rotation umap-T-uobj)))))
          (list
           type
           (car object-from)
           (nth 0 sample-for-object-type)
           (cl-transforms:x obj-position)
           (cl-transforms:y obj-position)
           (third sample-for-object-type)
           (nth 3 sample-for-object-type))))
      sample-for-object-type-ll))))

(defun query-for-csv-export (object-type start-or-end &optional context)
  (declare (type string object-type start-or-end)
           (type (or null keyword) context))
  "Returns the OWL type of the supporting surface an object is picked up from or
placed, the transform surface-T-object and the used hand of the robot as a lazy list
of pairs: '((name-1 . surface-T-object-1 . hand-1) . rest-of-lazy-list)."
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
            (obj-T-cam-zeroed-x-y-orient
              (let* ((obj-T-cam (cl-transforms:transform* obj-T-map map-T-camera))
                     (obj-T-cam-q (cl-tf:rotation obj-T-cam)))
                (cl-tf:make-transform
                 (cl-tf:translation obj-T-cam)
                 (cl-tf:make-quaternion 0
                                        0
                                        (cl-tf:z obj-T-cam-q)
                                        (cl-tf:w obj-T-cam-q)))))
            (angle-between-obj-and-cam
              (* 180
                 (/
                  (cl-tf:angle-between-quaternions
                   (cl-transforms:rotation (cl-transforms:transform-inv obj-T-cam-zeroed-x-y-orient))
                   (cl-tf:make-identity-rotation)) pi)))
            (discreted-angle-betw-obj-and-cam
              (if (or
                   (< 45 angle-between-obj-and-cam 135)
                   (< 225 angle-between-obj-and-cam 315))
                  "horizontal"
                  "vertical"))
            (hand-string
              (string
               (cut:var-value '|?HandTypeName| binding-set)))
            (EpInst-name
              (first (remove-if-not (alexandria:curry #'search "UnrealExperiment")
                                    (split-sequence:split-sequence-if
                                     (lambda (c)
                                       (or (equal c #\')
                                           (equal c #\#)))
                                     (string (cut:var-value '|?EpInst| binding-set)))))))
       (if (search "Left" hand-string)
           (setf hand-string :left)
           (if (search "Right" hand-string)
               (setf hand-string :right)
               (setf hand-string NIL)))
       (list surface-name surface-T-object hand-string discreted-angle-betw-obj-and-cam EpInst-name)))
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
