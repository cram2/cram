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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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

(in-package :btr)

(defclass semantic-map-object (robot-object sem-map-utils:semantic-map) ())

(defmethod (setf joint-state) :before (new-value (sem-map semantic-map-object) name)
  (attach-contacting-objects sem-map :test (lambda (obj link-name)
                                             (declare (ignore link-name))
                                             (typep obj 'household-object))))

(defmethod (setf link-pose) :before (new-value (sem-map semantic-map-object) name)
  (attach-contacting-objects sem-map :test (lambda (obj link-name)
                                             (declare (ignore link-name))
                                             (typep obj 'household-object))))

(defmethod copy-object ((obj semantic-map-object) (world bt-reasoning-world))
  (with-slots (pose parts) obj
    (change-class (call-next-method) 'semantic-map-object :parts parts)))

(defmethod add-object ((world bt-world) (type (eql 'semantic-map)) name pose &key urdf)
  (let ((sem-map (sem-map-utils:get-semantic-map)))
    (change-class (add-object world 'urdf name pose :urdf urdf) 'semantic-map-object
                  :parts (slot-value sem-map 'sem-map-utils::parts))))
