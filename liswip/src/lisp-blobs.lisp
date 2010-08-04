;;;
;;; Copyright (c) 2009, Lorenz Moesenlechner <moesenle@cs.tum.edu>
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

(in-package :liswip)

(defvar *lisp-blob-data-pool* (make-data-pool))

(defvar *lisp-blob-type* nil)

(defun init-lisp-blobs ()
  (setf *lisp-blob-type* (foreign-alloc 'pl_blob_t))
  (setf (foreign-slot-value *lisp-blob-type* 'pl_blob_t 'magic) PL-BLOB-MAGIC
        (foreign-slot-value *lisp-blob-type* 'pl_blob_t 'flags) PL-BLOB-UNIQUE
        (foreign-slot-value *lisp-blob-type* 'pl_blob_t 'name) "LISP_OBJECT"
        
        (foreign-slot-value *lisp-blob-type* 'pl_blob_t 'release)
        (callback release-lisp-blob)

        (foreign-slot-value *lisp-blob-type* 'pl_blob_t 'compare)
        (null-pointer)

        (foreign-slot-value *lisp-blob-type* 'pl_blob_t 'write)
        (null-pointer)

        (foreign-slot-value *lisp-blob-type* 'pl_blob_t 'acquire)
        (null-pointer))
  (pl-register-blob-type *lisp-blob-type*))

(defun cleanup-lisp-blobs ()
  (assert (eql (pl-is-initialised (null-pointer) (null-pointer))
               FALSE))
  (assert *lisp-blob-type*)
  (pl-unregister-blob-type *lisp-blob-type*)
  (foreign-free *lisp-blob-type*)
  (setf *lisp-blob-type* nil))

(defcallback release-lisp-blob :int ((atom atom_t))
  (with-foreign-objects ((len :pointer)
                         (type :pointer))
    (let ((data (pl-blob-data atom len type)))
      (assert (equal (foreign-slot-value (mem-ref type :pointer) 'pl_blob_t 'name)
                     "LISP_OBJECT"))
      (delete-pool-value *lisp-blob-data-pool* (mem-ref data :uint))))
  TRUE)

(defun put-lisp-object (term obj)
  (let ((id (new-pool-value *lisp-blob-data-pool* obj)))
    (with-foreign-object (id-ptr :uint)
      (setf (mem-ref id-ptr :uint) id)
      (pl-put-blob term id-ptr (foreign-type-size :uint) *lisp-blob-type*))))

(defun unify-lisp-object (term obj)
  (let ((id (new-pool-value *lisp-blob-data-pool* obj)))
    (with-foreign-object (id-ptr :uint)
      (setf (mem-ref id-ptr :uint) id)
      (pl-unify-blob term id-ptr (foreign-type-size :uint) *lisp-blob-type*))))

(defun get-lisp-object (term &key (gc t))
  "When gc is non NIL, replaces the object in the datapool by a weak
reference to it and registers a finalizer that removes the object from
the data pool on garbage collection."

  (flet ((blob-pool-finalizer (id)
           (lambda ()
             (delete-pool-value *lisp-blob-data-pool* id))))
    (with-foreign-objects ((blob :pointer)
                           (len :pointer)
                           (type :pointer))
      (let ((result (pl-get-blob term blob len type)))
        (assert (eql result TRUE) () "Term seems not to contain a blob.")
        (assert (equal (foreign-slot-value (mem-ref type :pointer) 'pl_blob_t 'name)
                       "LISP_OBJECT"))
        (let* ((value-id (mem-ref (mem-ref blob :pointer) :uint))
               (value (pool-value *lisp-blob-data-pool* value-id)))
          (cond ((tg:weak-pointer-p value)
                 (assert (tg:weak-pointer-value value) ()
                         "Trying to access garbage collected
                          value. This should not happen and indicates
                          a bug!")
                 (tg:weak-pointer-value value))
                (gc
                 (setf (pool-value *lisp-blob-data-pool* value-id) (tg:make-weak-pointer value))
                 (tg:finalize value (blob-pool-finalizer value-id))
                 value)
                (t
                 value)))))))

(defun lisp-object? (term)
  (with-foreign-object (type :pointer)
    (let ((result (pl-is-blob term type)))
      (and (eql result TRUE)
           (equal (foreign-slot-value (mem-ref type :pointer) 'pl_blob_t 'name)
                  "LISP_OBJECT")))))
