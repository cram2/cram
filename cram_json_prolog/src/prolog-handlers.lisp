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

(in-package :json-prolog)

(defun rename-prolog-vars (vars)
  "Renames variables and creates valid names for prolog. Returns an
alist of the form ((new-var . old-var) ...)"
  (mapcar (lambda (var)
            ;; We need to re-intern the symbol to get it in a sane package
            (cons var (intern (symbol-name (gen-var (substitute #\_ #\- (symbol-name var)))))))
          vars))

(prolog:def-prolog-handler json-prolog (bdgs form &rest key-args &key prologify lispify package)
  (declare (ignore prologify lispify package))
  (when (wait-for-prolog-service 0.5)
    (let* ((form (substitute-vars form bdgs))
           (vars (remove-if #'is-unnamed-var (vars-in form)))
           (var-mappings (rename-prolog-vars vars))
           ;; force-ll to make sure the json query is finished immediately
           (result (force-ll (apply #'prolog (sublis var-mappings form)
                                    key-args))))
      (lazy-mapcar (lambda (binding)
                     (block nil
                       (reduce (lambda (bdgs var)
                                 (let ((val (cdr (assoc var var-mappings))))
                                   (or
                                    (add-bdg var (var-value val binding)
                                             bdgs)
                                    (return nil))))
                               vars :initial-value bdgs)))
                   result))))
