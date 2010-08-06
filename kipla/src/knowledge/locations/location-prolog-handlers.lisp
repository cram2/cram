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

(in-package :kipla-reasoning)

(defun costmap-metadata ()
  (with-vars-bound (?width ?height ?resolution ?origin-x ?origin-y)
      (lazy-car (prolog `(and (costmap-size ?width ?height)
                              (costmap-origin ?origin-x ?origin-y)
                              (costmap-resolution ?resolution))))
    (list :width ?width :height ?height :resolution ?resolution
          :origin-x ?origin-x :origin-y ?origin-y)))

(def-prolog-handler cost-function (bdgs ?cm ?score ?generator-fun &rest args)
  (flet ((var-value-strict (var bdgs)
           (let ((val (var-value var bdgs)))
             (unless (is-var val)
               val))))
    
    (assert (is-var ?cm) ()
            "?CM is an output parameter and therefore must be a variable.")
    (let ((cm (or (var-value-strict ?cm bdgs)
                  (apply #'make-instance 'location-costmap (costmap-metadata))))
          (fun (cond ((is-var ?generator-fun)
                      (symbol-function (var-value-strict ?generator-fun bdgs)))
                     ((and ?generator-fun (symbolp ?generator-fun))
                      (symbol-function ?generator-fun))))
          (score (if (is-var ?score)
                     (var-value-strict ?score bdgs)
                     ?score)))
      (assert fun () "Generator function must be specified.")
      (check-type score integer)
      (register-cost-function cm (apply fun (sublis bdgs args)) score)
      (let ((out-var (var-value ?cm bdgs)))
        (list
         (if (is-var out-var)
             (add-bdg out-var cm bdgs)
             bdgs))))))
