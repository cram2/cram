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

(in-package :cffi-ros-utils)

(defun ros-library-paths (pkg-name)
  "Returns the list of library paths as exported by the ROS package
  with name `pkg-name' in its cpp tag. This function parses the lflags
  attribute and collects all library paths specified by -L."
  (mapcar (lambda (seq)
            (subseq seq 2))
          (delete-if-not
           (lambda (seq)
             (equal (subseq seq 0 2) "-L"))
           (split-sequence:split-sequence
            #\Space
            (car (ros-load:rospack "export" "--lang=cpp" "--attrib=lflags" pkg-name))
            :remove-empty-subseqs t))))

(defmacro define-foreign-ros-library (alias lib-file &optional (ros-package ros-load:*current-ros-package*))
  `(progn
     (define-foreign-library ,alias
       (:unix ,lib-file))

     (eval-when (:compile-toplevel :load-toplevel :execute)
       (dolist (path (ros-library-paths ,ros-package))
         (pushnew (concatenate 'string path "/")
                  *foreign-library-directories*
                  :test #'equal)))))
