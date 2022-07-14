(defpackage #:bmp
  (:use #:common-lisp #:image)
  (:export #:decode #:encode))

(in-package #:bmp)

(defun read-u2le (in)
  "Reads 2 unsigned BYTES in little-endian from IN stream without
testing for end of file."
  (let ((u 0))
    (setf (ldb (byte 8 0) u) (read-byte in))
    (setf (ldb (byte 8 8) u) (read-byte in))
    u))

(defun read-u4le (in)
  "Reads 4 unsigned BYTES in little-endian from IN stream without
testing for end of file."
  (let ((u 0))
    (setf (ldb (byte 8  0) u) (read-byte in))
    (setf (ldb (byte 8  8) u) (read-byte in))
    (setf (ldb (byte 8 16) u) (read-byte in))
    (setf (ldb (byte 8 24) u) (read-byte in))
    u))

(defun write-u2le (out u)
  "Writes 2 unsigned BYTES in little-endian to OUT stream."
  (write-byte (ldb (byte 8  0) u) out)
  (write-byte (ldb (byte 8  8) u) out))

(defun write-u4le (out u)
  "Writes 4 unsigned BYTES in little-endian to OUT stream."
  (write-byte (ldb (byte 8  0) u) out)
  (write-byte (ldb (byte 8  8) u) out)
  (write-byte (ldb (byte 8 16) u) out)
  (write-byte (ldb (byte 8 24) u) out))

(define-condition unhandled-compression ()
  ((mode :initarg :mode :reader unhandled-compression-mode))
  (:report (lambda (condition stream)
             (format stream "Unhandled BMP bitmap compression mode ~a~%"
                     (unhandled-compression-mode condition)))))

(define-condition unhandled-bitcount ()
  ((bitcount :initarg :bitcount :reader unhandled-bitcount-bitcount))
  (:report (lambda (condition stream)
             (format stream "Unhandled BMP bitcount ~a~%"
                     (unhandled-bitcount-bitcount condition)))))

(defun decode (input &key bgr flip strip-alpha)
  "Reads an image in BMP format from input and returns an array of
type IMAGE.  The bit depth of the returned IMAGE will be either 8 or
16.

Setting FLIP will flip the image vertically.
Setting STRIP-ALPHA will strip an alpha channel, if present. 

Setting BGR will leave the channels ordered as read from the BMP
stream. That is, in BGR or BGRA order, rather than swapping them to
RGB or RGBA order normal to IMAGE).

The current version will decode 24-bit RGB and 32-bit ARGB BMP
streams. Compressed formats are not currently supported and will
signal an error.

Signals an error if reading the image fails."
  ;; Check for "BM" signature at beginning
  (unless (= #x4d42 (read-u2le input))
    (error "~s Not a BMP bitmap image stream" input))
  ;; Read rest of header
  (let* ((file-sz (read-u4le input))
         (reserve (read-u4le input))
         (raster-data-offset (read-u4le input))
         (sz (read-u4le input))
         (cols (read-u4le input))
         (rows (read-u4le input))
         (planes (read-u2le input))
         (bitcount (read-u2le input))
         (compression (read-u4le input))
         (image-sz  (read-u4le input))
         (ppm  (list (read-u4le input) (read-u4le input)))
         (colors (list (read-u4le input) (read-u4le input)))
         (current-offset 54))
    (declare (ignorable reserve sz planes ppm colors))
    ;; Check if we can decode this stream
    (unless (= compression 0)
      (error 'unhandled-compression :mode compression))
    (unless (member bitcount '(8 24 32))
      (error 'unhandled-bitcount :bitcount bitcount))
    ;; Note: the image-sz is only non-zero when compression is being
    ;; used.  The image size should be calculated from the filesize
    ;; and the raster-data-offset, both of which must be accurate.
    (setf image-sz (- file-sz raster-data-offset))
    (let* ((channels (floor image-sz (* rows cols)))
           (bit-depth (/ bitcount channels))
           (stripping-alpha (and strip-alpha (= channels 4)))
           (chan-loop-lim (if stripping-alpha (1- channels) channels))
           (image (make-image rows cols
                              (if stripping-alpha 3 channels)
                              (if (= 16 bit-depth) 16 8)))
           (npad (mod (* channels cols) 4))
           (bgridx (make-array channels)))

      ;; Set up BGR channel swap index array
      (dotimes (c channels) (setf (aref bgridx c) c))
      (when (and (not bgr) (>= channels 3))
        (setf (aref bgridx 0) 2)
        (setf (aref bgridx 2) 0))

      ;; Fix npad 
      (when (< 0 npad) (setf npad (- 4 npad)))

      ;; Change some values to reflect stripping of alpha
      (when stripping-alpha
        (setf bitcount 24
              image-sz (* 24/32 image-sz))
        (setf file-sz (+ image-sz raster-data-offset)))
#|
      (format t "~&       dims: ~ax~a~%" rows cols)
      (format t "     bitcount: ~a~%" bitcount)
      (format t "  compression: ~a~%" compression)
      (format t "    file size: ~a  image size:~a~%" file-sz image-sz)
      (format t "raster-offset: ~x~%" raster-data-offset)
      (format t "         npad:~a~%" npad)
      (format t "       bgridx:~s~%" bgridx)
|#
      ;; Read palette if there is one
      (when (> raster-data-offset current-offset)
        (let ((table-size (- raster-data-offset current-offset)))
          ;; (format t "   table-size: ~d~%" table-size)
          (dotimes (r table-size)
            (read-byte input))))

      ;; Read pixel data      
      (dotimes (row (image-height image))
        (dotimes (col (image-width image))
          (dotimes (chan chan-loop-lim)
            (setf (aref image
                        (if flip (- rows row 1) row)
                        col
                        (aref bgridx chan)) (read-byte input)))
          (when stripping-alpha
            (read-byte input)))
        (dotimes (c npad)
          (read-byte input)))
      image)))


(defun decode-file (pathname &key flip bgr)
  "Reads file PATHNAME, decodes as BMP file and returns IMAGE."
  (with-open-file (input pathname :element-type '(unsigned-byte 8))
    (bmp:decode input :flip flip :bgr bgr)))


(defun encode (image output &key flip (xppi 72) (yppi 72)
	       xppm yppm (reserve 0) strip-alpha bgr)
  "Writes IMAGE in BMP format to OUTPUT.

Flips image vertically if FLIP set.  XPPI and YPPI specify pixels per
inch Strips alpha channel if STRIP-ALPHA is set (and there is one) - in
other words, encodes ARGB as RGB.

The current version only encodes RGB and ARGB BMP files (24 and 32 bit
pixels, respectively). Paletted versions signal an error.

Signals an error if writing the image fails."
  ;; Notes:
  ;;      reserve: 4-bytes that can be set by caller arbitrarily.
  ;;      
  ;;  compression: 0= BI_RGB (none: most common)
  ;;               1= BI_RLE8 palettized (8-bits/pixel)
  ;;               2= BI_RLE4 palettized (4-bits/pixel)
  ;;               3= BI_BITFIELDS (16,32-bits/pixel bitmaps)
  ;;               4= BI_JPEG (not supported)
  ;;               5= BI_PNG (not supported)
  ;;
  ;;    xppm,yppm: horizontal,vertical resolution in pixels/meter
  ;;
  ;;  colors_used: 0 defaults to 2**n
  ;;  colors_important is generally ignored
  ;;  
  (check-type image (or rgb-image grayscale-image))
  (let ((raster-data-offset (if (/= 1 (image-channels image)) 54 (+ 54 1024)))
        (sz                 40)
        (planes             1)
        (compression        0)
        (imagesize          0)
        (colors-used        0)
        (colors-important   0)
        (stripping-alpha (and strip-alpha (= 4 (image-channels image)))))
    ;; Convert the resolution if necessary
    (unless (numberp xppm)
      (setf xppm (floor xppi 0.0254)))
    (unless (numberp yppm)
      (setf yppm (floor yppi 0.0254)))
    ;; Rows of pixels are padded out to 4-byte boundaries, so we have
    ;; to calculate the number of pad bytes
    (let* ((channels    (if stripping-alpha 3 (image-channels image)))
           (bitcount    (* channels (image-bit-depth image)))
           (bytes/pixel (/ bitcount 8))
           (bytes/row   (* bytes/pixel (image-width image) channels))
           (npad        (mod bytes/row 4))
           (nbytes      (* (image-height image)
                           (+ npad (* channels
                                      (image-width image)))))
           (bgridx      (make-array channels))
           (filesize    (+ nbytes raster-data-offset)))
      ;; Set up BGR channel swap index array
      (dotimes (c channels) (setf (aref bgridx c) c))
      (when (and (not bgr) (>= channels 3))
        (setf (aref bgridx 0) 2)
        (setf (aref bgridx 2) 0))
      ;; Fix npad 
      (when (and (= channels 1) (< 0 npad))
        (setf npad (- 4 npad)))
      ;; Write "BM" signature
      (write-u2le output 19778)
      ;; Write primary header
      (write-u4le output filesize)
      (write-u4le output reserve)
      (write-u4le output raster-data-offset)
      ;; Write DIB header
      (write-u4le output sz)
      (write-u4le output (image-width image))
      (write-u4le output (image-height image))
      (write-u2le output planes)
      (write-u2le output bitcount)
      (write-u4le output compression)
      (write-u4le output imagesize)
      (write-u4le output xppm)
      (write-u4le output yppm)
      (write-u4le output colors-used)
      (write-u4le output colors-important)
      ;; Write color table if required
      (cond ((member bitcount '(8)) ; bogus color table for grayscale images
             (dotimes (n 256)
               (dotimes (c 3)
                 (write-byte n output))
               (write-byte 0 output)))
            ((member bitcount '(24 32))) ; don't need one
            (t (error 'unhandled-bitcount :bitcount bitcount)))
      ;; Write raster data
      (dotimes (row (image-height image))
        (dotimes (col (image-width image))
          (dotimes (chan channels)
            (write-byte (aref image
                              (if flip (- (image-height image) row 1) row)
                              col
                              (aref bgridx chan))
                        output)))
        (dotimes (c npad)
          (write-byte 0 output))))))

(defun encode-file (image pathname &key flip strip-alpha bgr)
  "Encodes IMAGE as BMP and writes to PATHNAME."
  (with-open-file (output pathname :element-type '(unsigned-byte 8)
			  :direction :output :if-exists :supersede)
    (bmp:encode image output :flip flip :strip-alpha strip-alpha :bgr bgr)))


