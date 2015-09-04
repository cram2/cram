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
  (apply #'append (mapcar (lambda (seq)
            (split-sequence:split-sequence #\: (subseq seq 2) :remove-empty-subseqs t))
          (delete-if-not
           (lambda (seq)
             (equal (subseq seq 0 2) "-L"))
           (split-sequence:split-sequence
             #\Space
             (car (ros-load:rospack "export" "--lang=cpp" "--attrib=lflags" pkg-name))
             :remove-empty-subseqs t)))))

(defun ros-include-paths (pkg-name)
  "Returns the list of include paths as exported by the ROS package
  `pkg-name'"
  (split-sequence:split-sequence
   #\Space
   (car (ros-load:rospack "cflags-only-I" pkg-name))
   :remove-empty-subseqs t))

(defclass ros-grovel-file (cffi-grovel:grovel-file)
  ((ros-package :initarg :ros-package)))

(defmethod reinitialize-instance :after ((c ros-grovel-file)
                                         &key (ros-package ros-load:*current-ros-package*))
  (setf (cffi-grovel::cc-flags-of c)
        (append (mapcar (lambda (path)
                          (concatenate 'string "-I" path))
                        (ros-include-paths ros-package))
                (cffi-grovel::cc-flags-of c))))

(defmacro define-foreign-ros-library (alias lib-file &optional (ros-package ros-load:*current-ros-package*))
  (flet ((find-ros-library (file ros-package)
           (let ((file-path (pathname file)))
             (dolist (lib-path (ros-library-paths ros-package) nil)
               (let ((complete-file-path (merge-pathnames
                                          file-path
                                          (make-pathname :directory lib-path))))
                 (format t "~a~%" complete-file-path)
                 (when (probe-file complete-file-path)
                   (return-from find-ros-library complete-file-path)))))))
    `(define-foreign-library ,alias
       (:unix ,(namestring
                (or (find-ros-library lib-file ros-package)
                    (error
                     'simple-error
                     :format-control "Unable to find ros library `~a' in dependencies of package ~a"
                     :format-arguments (list lib-file ros-package))))))))
