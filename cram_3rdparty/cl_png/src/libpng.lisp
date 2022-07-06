(in-package #:png)

#+cffi-features:darwin
(push #p"/usr/X11/lib/" cffi:*foreign-library-directories*)

#+cffi-features:darwin
(push #p"/opt/local/lib/" cffi:*foreign-library-directories*)

(define-foreign-library libpng
  (:darwin "libpng.dylib")
  (t (:default "libpng")))

(use-foreign-library libpng)

(defparameter +png-libpng-ver-string+ (get-png-libpng-ver-string))

;;; Foreign function definitions.

(defcfun "png_access_version_number" :uint32)

(defcfun "png_create_read_struct" :pointer
  (user-png-ver :string)
  (error-ptr :pointer)
  (error-fn :pointer)
  (warn-fn :pointer))

(defcfun "png_destroy_read_struct" :void
  (png-ptr-ptr :pointer)
  (info-ptr-ptr :pointer)
  (end-info-ptr-ptr :pointer))

(defcfun "png_create_write_struct" :pointer
  (user-png-ver :string)
  (error-ptr :pointer)
  (error-fn :pointer)
  (warn-fn :pointer))

(defcfun "png_destroy_write_struct" :void
  (png-ptr-ptr :pointer)
  (info-ptr-ptr :pointer))

(defcfun "png_create_info_struct" :pointer
  (png-ptr :pointer))

(defcfun "png_destroy_info_struct" :void
  (png-ptr :pointer)
  (info-ptr-ptr :pointer))

(defcfun "png_init_io" :void
  (png-ptr :pointer)
  (file :pointer))

(defcfun "png_set_read_fn" :void
  (png-ptr :pointer)
  (io-ptr :pointer)
  (read-data-fn :pointer))

(defcfun "png_set_write_fn" :void
  (png-ptr :pointer)
  (io-ptr :pointer)
  (write-data-fn :pointer)
  (output-flush-fn :pointer))

(defcfun "png_get_io_ptr" :pointer
  (png-ptr :pointer))

(defcfun "png_read_info" :void
  (png-ptr :pointer)
  (info-ptr :pointer))

(defcfun "png_read_png" :void
  (png-ptr :pointer)
  (info-ptr :pointer)
  (png-transforms :int)
  (params :pointer))

(defcfun "png_get_IHDR" :uint32
  (png-ptr :pointer)
  (info-ptr :pointer)
  (width-uint32-ptr :pointer)
  (height-uint32-ptr :pointer)
  (bit-depth-int-ptr :pointer)
  (color-type-int-ptr :pointer)
  (interlace-type-int-ptr :pointer)
  (compression-type-int-ptr :pointer)
  (filter-type-int-ptr :pointer))

(defcfun "png_set_IHDR" :void
  (png-ptr :pointer)
  (info-ptr :pointer)
  (width :uint32)
  (height :uint32)
  (bit-depth :int)
  (color-type :int)
  (interlace-type :int)
  (compression-type :int)
  (filter-type :int))

(defcfun "png_set_bgr" :void
  (png-ptr :pointer))

(defcfun "png_set_palette_to_rgb" :void
  (png-ptr :pointer))

(defcfun "png_set_expand_gray_1_2_4_to_8" :void
  (png-ptr :pointer))

(defcfun "png_set_expand" :void
  (png-ptr :pointer))

(defcfun "png_get_valid" :uint32
  (png-ptr :pointer)
  (info-ptr :pointer)
  (flag :uint32))

(defcfun "png_set_tRNS_to_alpha" :void
  (png-ptr :pointer))

(defcfun "png_set_strip_16" :void
  (png-ptr :pointer))

(defcfun "png_set_strip_alpha" :void
  (png-ptr :pointer))

(defcfun "png_set_swap" :void
  (png-ptr :pointer))

(defcfun "png_get_rows" :pointer
  (png-ptr :pointer)
  (info-ptr :pointer))

(defcfun "png_set_rows" :void
  (png-ptr :pointer)
  (info-ptr :pointer)
  (row-pointers :pointer))

(defcfun "png_read_image" :void
  (png-ptr :pointer)
  (row-pointers :pointer))

(defcfun "png_write_png" :void
  (png-ptr :pointer)
  (info-ptr :pointer)
  (transforms :int)
  (params :pointer))

(defcfun "memcpy" :pointer
  (dest :pointer)
  (source :pointer)
  (n size))


;;; Input/output.

(defvar *stream*)

(defvar *buffer*)

(defun ensure-buffer-sufficient (needed)
  (when (< (length *buffer*) needed)
    (let ((new-length (length *buffer*)))
      (loop while (< new-length needed)
	 do (setf new-length (* 2 new-length)))
      (setf *buffer* (make-shareable-byte-vector new-length)))))

(defcallback user-read-data :void ((png-ptr :pointer) (data :pointer)
				   (length png-size))
  (declare (ignore png-ptr))
  (ensure-buffer-sufficient length)
  (let ((bytes-read (read-sequence *buffer* *stream* :start 0 :end length)))
    (unless (= bytes-read length)
      (error "Expected to read ~D bytes, but only read ~D." length
	     bytes-read)))
  (with-pointer-to-vector-data (buffer-ptr *buffer*)
    (memcpy data buffer-ptr length)))

(defcallback user-write-data :void ((png-ptr :pointer) (data :pointer)
				    (length png-size))
  (declare (ignore png-ptr))
  (ensure-buffer-sufficient length)
  (with-pointer-to-vector-data (buffer-ptr *buffer*)
    (memcpy buffer-ptr data length))
  (write-sequence *buffer* *stream* :start 0 :end length))

(defcallback user-flush-data :void ((png-ptr :pointer))
  (declare (ignore png-ptr)))


;;; Error handling.

(defcallback error-fn :void ((png-structp :pointer) (message :string))
  (declare (ignore png-structp))
  (error message))

(defcallback warn-fn :void ((png-structp :pointer) (message :string))
  (declare (ignore png-structp))
  (error message))


;;; Encode and decode PNG files.

(defmacro with-png-struct ((var &key (direction :input)) &body body)
  (let ((pointer (gensym "POINTER")))
    `(let ((,var (,(ecase direction
			  (:input 'png-create-read-struct)
			  (:output 'png-create-write-struct))
		   +png-libpng-ver-string+ (null-pointer)
		   (callback error-fn) (callback warn-fn)))
	   (*buffer* (make-shareable-byte-vector 1024)))
       (when (null-pointer-p ,var)
	 (error "Failed to allocate PNG write struct."))
       (unwind-protect (progn ,@body)
	 (with-foreign-pointer (,pointer (foreign-type-size :pointer))
	   (setf (mem-ref ,pointer :pointer) ,var)
	   ,(ecase direction
		   (:input `(png-destroy-read-struct ,pointer (null-pointer)
						     (null-pointer)))
		   (:output `(png-destroy-write-struct ,pointer
						       (null-pointer)))))))))

(defmacro with-png-info-struct ((var png-struct initform) &body body)
  (let ((pointer (gensym "POINTER")))
    `(let ((,var ,initform))
       (when (null-pointer-p ,var)
	 (error "Failed to allocate PNG info struct."))
       (unwind-protect (progn ,@body)
	 (with-foreign-pointer (,pointer (foreign-type-size :pointer))
	   (setf (mem-ref ,pointer :pointer) ,var)
	   (png-destroy-info-struct ,png-struct ,pointer))))))

(defun get-ihdr (png-ptr info-ptr)
  (with-foreign-pointer (width (foreign-type-size :uint32))
    (with-foreign-pointer (height (foreign-type-size :uint32))
      (with-foreign-pointer (bit-depth (foreign-type-size :int))
	(with-foreign-pointer (color-type (foreign-type-size :int))
	  (png-get-ihdr png-ptr info-ptr width height bit-depth
			color-type (null-pointer) (null-pointer)
			(null-pointer))
	  (values (mem-ref width :uint32) (mem-ref height :uint32)
		  (mem-ref bit-depth :int) (mem-ref color-type :int)))))))

(defun bytes-per-pixel (image)
  (ecase (image-bit-depth image)
    (16 2)
    (8 1)))

(defmacro with-row-pointers ((rows-ptr image)
			     &body body)
  (let ((row-pointers (gensym "ROW-POINTERS"))
	(raw-data (gensym "RAW-DATA"))
	(i (gensym "I")))
    `(let ((,row-pointers (make-shareable-byte-vector
			   (* (image-height ,image)
			      (foreign-type-size :pointer)))))
       (with-pointer-to-vector-data (,rows-ptr ,row-pointers)
	 (with-pointer-to-array-data (,raw-data ,image)
	   (dotimes (,i (image-height ,image))
	     (setf (mem-aref ,rows-ptr :pointer ,i)
		   (inc-pointer ,raw-data (* ,i (image-width ,image)
					     (image-channels ,image)
					     (bytes-per-pixel ,image)))))
	   ,@body)))))

(defun grayp (color-type)
  (zerop (logand color-type (lognot +png-color-mask-alpha+))))

(defun decode (input &key swapbgr preserve-alpha)
  "Reads an image in PNG format from input and returns an array of
type IMAGE.  If the bit depth of the PNG file is less than or equal to
8, an 8-BIT-IMAGE will be returned; otherwise, a 16-BIT-IMAGE will be
returned.

Applications that would like to receive images of consistent bit
depth (rather than 8 or 16 depending on the PNG file) can apply the
function 8-BIT-IMAGE or the function 16-BIT-IMAGE to the result of
DECODE.

Bit depths less than 8 will be converted to 8 bits when read, and bit
depths between 8 and 16 bits will be converted to 16 bits.  As an
example, 2-bit PNG files contain only the pixel values 0, 1, 2, and 3.
These will be converted to 0, 85, 170, and 255, respectively, in order
to fill the dynamic range of the 8-bit image that is returned.

Swaps blue and red if SWAPBGR set.

Strips alpha channel unless PRESERVE-ALPHA is set.

Signals an error if reading the image fails."
  (with-png-struct (png-ptr :direction :input)
    (with-png-info-struct (info-ptr png-ptr (png-create-info-struct png-ptr))
      (with-png-info-struct (end-ptr png-ptr (png-create-info-struct png-ptr))
	(let ((*stream* input))
	  (png-set-read-fn png-ptr (null-pointer) (callback user-read-data))
	  (png-read-info png-ptr info-ptr)
	  (multiple-value-bind (width height bit-depth color-type)
	      (get-ihdr png-ptr info-ptr)
	    (when (= color-type +png-color-type-palette+)
	      (png-set-palette-to-rgb png-ptr))
	    (when (grayp color-type)
	      ;; png-set-expand-gray-1-2-4-to-8 did nothing on CCL
	      ;; DarwinPPC, but png-set-expand seems to work.
	      (png-set-expand png-ptr))
	    #+little-endian
	    (when (= bit-depth 16)
	      (png-set-swap png-ptr))
	    (unless (or preserve-alpha
			(zerop (logand color-type +png-color-mask-alpha+)))
	      (png-set-strip-alpha png-ptr))
        (when swapBGR
          (png-set-bgr png-ptr))
	(let* ((alphas (if (and preserve-alpha
				(plusp (logand color-type
					       +png-color-mask-alpha+)))
			   1 0))
	       (image (make-image height width
				  (+ (if (grayp color-type) 1 3) alphas)
				  (if (= 16 bit-depth) 16 8))))
	  (with-row-pointers (row-pointers image)
	    (png-set-rows png-ptr info-ptr row-pointers)
	    (png-read-image png-ptr row-pointers))
	  image)))))))

(defun decode-file (pathname &key swapbgr)
  (with-open-file (input pathname :element-type '(unsigned-byte 8))
    (decode input :swapbgr swapbgr)))

(defun encode (image output &key swapbgr)
  "Writes IMAGE in PNG format to OUTPUT.  The current version always
writes an 8-bit PNG file if image is an 8-BIT-IMAGE and a 16-bit PNG
file if image is an 16-BIT-IMAGE.  Future versions may write PNG files
of lower bit depths than IMAGE when the least significant bits may be
trimmed without loss of precision.

Signals an error if writing the image fails."
  (check-type image (or grayscale-image rgb-image))
  (with-png-struct (png-ptr :direction :output)
    (with-png-info-struct (info-ptr png-ptr (png-create-info-struct png-ptr))
      (let ((*stream* output))
	(png-set-write-fn png-ptr (null-pointer) (callback user-write-data)
			  (callback user-flush-data))
	(png-set-ihdr png-ptr info-ptr (image-width image) (image-height image)
		      (image-bit-depth image)
                      (ecase (image-channels image)
                        (1 +png-color-type-gray+)
                        (2 +png-color-type-gray-alpha+)
                        (3 +png-color-type-rgb+)
                        (4 +png-color-type-rgb-alpha+))
		      +png-interlace-none+ +png-compression-type-default+
		      +png-filter-type-default+)
	(when swapBGR
	  (png-set-bgr png-ptr))
	(with-row-pointers (row-pointers image)
	  (png-set-rows png-ptr info-ptr row-pointers)
	  (png-write-png png-ptr info-ptr
			 #+little-endian +png-transform-swap-endian+
			 #-little-endian +png-transform-identity+
			 (null-pointer))))))
  t)

(defun encode-file (image pathname &key swapbgr)
  (with-open-file (output pathname :element-type '(unsigned-byte 8)
			  :direction :output :if-exists :supersede)
    (encode image output :swapbgr swapbgr)))
