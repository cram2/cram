;;;
;;; Copyright (c) 2018, Alina Hawkin <hawkin@cs.uni-bremen.de>
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

;;;;;;;;;;;;;;;;; utils ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun move-object (transform obj)
  "Moves an object to the desired transform.
TRANSFORM: The transform describing the position to which an object should be moved.
OBJ: The object which is supposed to be moved.
usage example: (move-object (pose-lists-parser '|?PoseObjEnd|))"
  (let* ((pose (cl-transforms:transform->pose transform)))
    (prolog:prolog `(and (btr:bullet-world ?world)
                         (assert (btr:object-pose ?world ,obj ,pose))))))

;;;;;;;;;;;;;;;;;;;; semantic initial spawning ;;;;;;;;;;;;;;

(defun move-obj-with-offset (x-offset y-offset object-knowrob object-bullet)
  "Moves an object to a given x y offset from it's starting position. "
  (let ((object-pose
          (car (query-object-location-by-object-type object-knowrob "Start"))))
    (move-object
     (cl-transforms:make-transform
      (cl-transforms:make-3d-vector
       (+ x-offset (cl-transforms:x (cl-transforms:translation object-pose)))
       (+ y-offset (cl-transforms:y (cl-transforms:translation object-pose)))
       (cl-transforms:z (cl-transforms:translation object-pose)))
      (cl-transforms:rotation object-pose))
     object-bullet)))

(defun move-semantic-objects-to-start-pose ()
  (move-obj-with-offset
   *semantic-map-offset-x* *semantic-map-offset-y*
   "BowlLarge" :edeka-red-bowl2)
  (move-obj-with-offset
   *semantic-map-offset-x* *semantic-map-offset-y*
   "KoellnMuesliKnusperHonigNuss" :koelln-muesli-knusper-honig-nuss2)
  (move-obj-with-offset
   *semantic-map-offset-x* *semantic-map-offset-y*
   "PlasticBlueFork" :fork-blue-plastic2)
  (move-obj-with-offset
   *semantic-map-offset-x* *semantic-map-offset-y*
   "CupEcoOrange" :cup-eco-orange2)
  (move-obj-with-offset
   *semantic-map-offset-x* *semantic-map-offset-y*
   "MilramButtermilchErdbeere" :weide-milch-small2))

;;;;;;;;;;;;;;;;;;; urdf initial spawning ;;;;;;;;;;;;;;;;;

(defun move-object-to-starting-pose (object)
  "Moves the object and robot to their respective locations at the beginning of
the episode. . "
  (move-object (cl-transforms:pose->transform
                (cram-tf:translate-pose
                 (car (umap-P-uobj-through-surface-ll object "Start"))
                 :x-offset -0.1))
               (object-type-filter-bullet object)))

(defun move-urdf-objects-to-start-pose ()
  "Spawns all objects of the current Episode at their places without any offsets."
  (move-object-to-starting-pose 'muesli)
  (move-object-to-starting-pose 'milk)
  (move-object-to-starting-pose 'cup)
  (move-object-to-starting-pose 'bowl)
  (move-object-to-starting-pose 'fork))

(defun move-object-to-placing-pose (object)
  "Moves the object and robot to their respective locations at the beginning of
the episode. . "
  (move-object (cl-transforms:pose->transform
                (car (umap-P-uobj-through-surface-ll object "End")))
               (object-type-filter-bullet object)))

;;;;;;;;;;;;; other utils ;;;;;;;;;;;;;;;;;;;;

(defun move-away-axes ()
  (let* ((transform (cl-transforms:make-transform
                     (cl-transforms:make-3d-vector 3 3 3)
                     (cl-transforms:make-identity-rotation))))
    (move-object transform :axes)
    (move-object transform :axes2)
    (move-object transform :axes3)))

(defun test-placement ()
  (move-object
   (cl-tf:make-transform
    (cl-tf:make-3d-vector
     1.35 0.6 0.915)
    (cl-tf:make-quaternion
     -3.180422326961139d-17 7.275958169294938d-10 1.0d0 -4.371138828673793d-8))
   :cup-eco-orange))

#+these-are-not-used-anymore
(
 (defun parse-str (str)
   "parses the output from knowrob to a proper string which prolog can use."
   (concatenate 'string "'"  (remove #\' (string str)) "'"))

 (defun transform-to-pose-stamped (transform)
   "Takes a transform and returnes a pose stamped without any changes."
   (cl-transforms-stamped:make-pose-stamped
    "map"
    0.0
    (cl-transforms:translation transform)
    (cl-transforms:rotation transform)))

 (defun place-offset-transform ()
 "Creates a transform which describes and offset for placing an object. "
  (let ()
      (cl-tf:make-transform
       (cl-tf:make-3d-vector 0.0 0.2 0.0)
       (cl-tf:make-identity-rotation))))

 )
