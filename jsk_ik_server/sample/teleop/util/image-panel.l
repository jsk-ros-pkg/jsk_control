

(defun set-pixel
  (image x y color) ;; color is BGR vector
  (let* ( (col (concatenate cons color))
	  (depth (if (find-method image :depth)
		     (/ (send image :depth) 8)
		   1))
	  (x (* x depth)) )
    (labels ((itter
	      (off col-list)
	      (unless (null col-list)
		(progn
		  (send image :set-pixel (+ x off) y (round (car col-list)))
		  (itter (+ 1 off) (cdr col-list))))))
      (itter 0 col))))

(defun get-pixel
 (image x y)
 (let ( (depth (if (find-method image :depth)
		   (/ (send image :depth) 8)
		 1))
	(pixel (send image :pixel x y)) )
   (labels ( (itter (cnt)
		    (if (minusp cnt)
			nil
		      (cons (logand (ash pixel (* cnt -8)) #xff)
			    (itter (- cnt 1))))) )
     (itter (- depth 1)))))

(defun map-image
  (image func)
  (dotimes (x (send image :width))
    (dotimes (y (send image :height))
      (funcall func image x y))))

(defun array-image24
  (array width height)
  (let ( (ret (instance img::color-image24 :init width height)) )
    (dotimes (x width)
      (dotimes (y height)
	(set-pixel ret x y (nth x (nth y array)))))
    ret))

(defun func-image24
  (func width height)
  (let ( (ret (instance img::color-image24 :init width height)) )
    (dotimes (x width)
      (dotimes (y height)
	(set-pixel ret x y (funcall func x y))))
    ret))

(defun mono-fatten
  (image)
  (let ( (buf (send image :copy)) )
    (dotimes (x (- (send image :width) 2))
      (dotimes (y (- (send image :height) 2))
	(dotimes (dx 3)
	  (dotimes (dy 3)
	    (if (< (apply #'+ (get-pixel image (+ x dx) (+ y dy)))
		   (/ 255 2))
		(set-pixel buf (+ x 1) (+ y 1)
			   (map float-vector
				#'(lambda (a) 0) (get-pixel image x y))))))))
    buf))

(defun resize-image
  (image width height)
  (let ( (ret (send (send image :copy) :init width height))
					;(instance img::color-image
					;:init width height))
	 (w-step (/ (send image :width) (* 1.0 width)))
	 (h-step (/ (send image :height) (* 1.0 height))) )
    (map-image ret
	       #'(lambda (ret x y)
		   (set-pixel
		    ret x y
		    (get-pixel
		     image
		     (floor (* x w-step))
		     (floor (* y h-step))))))
    ret))

(defun reverse-pixel
  (image)
  (let ( (ret (instance img::color-image24
			:init
			(send image :width)
			(send image :height))) )
    (map-image ret
	       #'(lambda (ret x y)
		   (set-pixel
		    ret x y
;		    (map cons
;			 #'(lambda (num) (logand (lognot num) #xff))
		    (reverse (get-pixel image x y)))))
    ret))

(defclass image-panel
  :super x::panel
  :slots (name
	  image-buf
	  image
	  res)
  )

(defmethod image-panel
  (:create
   (&key
    (height 50)
    (width 50)
    (name "graphPanel")
    (image nil)
    (resolution nil)
    &rest args)
   (send-super* :create :height height :width width args)
   (setq res resolution)
   (if image
       (send self :image image))
   (send self :name name))
  (:name
   (&optional tit)
   (if tit
       (progn
	 (send self :title tit)
	 (setq name tit))
     name))
  (:image-push nil (push image image-buf))
  (:image-pop nil (setq image (pop image-buf)))
  (:image
   (&optional i)
   (if i
       (if (stringp i)
	   (send self :image-file i)
	 (setq image i))
     image))
  (:adjust-size
   nil
   (send self :resize (send image :width) (send image :height)))
  (:image-file
   (image-path)
   (send self :image (img::read-image-file image-path)))
  (:set-pixel
   (x y col)
   (set-pixel image x y col))
   ;; (x y color) ;; color is BGR vector
   ;; (let* ( (col (concatenate cons color))
   ;; 	   (depth (/ (send self :depth) 8))
   ;; 	   (x (* x depth)) )
   ;;   (labels ((itter
   ;; 	       (off col-list)
   ;; 	       (unless (null col-list)
   ;; 		 (progn
   ;; 		   (send image :set-pixel (+ x off) y (round (car col-list)))
   ;; 		   (itter (+ 1 off) (cdr col-list))))))
   ;;     (itter 0 col))))
  (:pixel (x y) (send image :pixel x y))
  (:get-pixel
   (x y)
   (get-pixel image x y))
   ;; (let ( (depth (/ (send self :depth) 8))
  ;; 	  (pixel (send image :pixel x y)) )
   ;;   (labels ( (itter (cnt)
  ;; 		      (if (minusp cnt)
  ;; 			  nil
   ;; 			(cons (logand (ash pixel (* cnt -8)) #xff)
   ;; 			      (itter (- cnt 1))))) )
   ;;     (itter (- depth 1)))))
  (:reverse-pixel
   (x y)
   (send self :set-pixel x y
	 (map cons #'(lambda (num) (logand (lognot num) #xff))
	      (send self :get-pixel x y))))
  (:noize-image
   (n/s)
   (map-image image
	      #'(lambda (image x y)
		  (if (< (random 1.0) n/s)
		      (send self :reverse-pixel x y)))))
  (:white-image
   (&optional (width (send self :width)) (height (send self :height)))
   (setq image (instance img::color-image24 :init width height)))
  ;; monochromized image cannot use set-pixel ???
  (:mono-image
   (&optional (i image))
   (if (stringp i)
       (progn
	 (send self :image-file i)
	 (send self :mono-image))
     (setq image (send i :monochromize))))
  (:resolution
   (&optional r)
   (if r (setq res r) res))
  (:resize-image
   (width height)
   (setq image (resize-image image width height)))
  (:image-depth
   nil
   (if (find-method image :depth)
       (/ (send image :depth) 8)
     1))
  (:repaint
   (&optional (i image))
   (if res
       (let* ( (small
		(resize-image (send i :to24)
			      (round (* (car res) (send image :width)))
			      (round (* (cadr res) (send image :height)))))
	       (big (resize-image small
				  (send image :width) (send image :height))) )
	 (send-super :putimage big))
     (send-super :putimage (send image :to24)))
   (send self :flush))
  )


(defun pickview-output
  (file-name)
  (img::write-image-file
   file-name
   (send *pickview* :viewer :viewsurface :getglimage)))

#|

(setq p (instance image-panel :create))
(send p :mono-image "image/mikan.jpg")
(send p :adjust-size)
(send p :repaint)


#|

(setq p (instance imagePanel :create))
(send p :image-file "image/mikan.png")
(send p :adjust-size)
(send p :repaint)


#|

(setq a (instance imagePanel :create))
;(setq b (img::read-png-file "./image/a.png"))
(setq b (img::read-image-file "./image/a.png"))
(send a :resize (send b :width) (send b :height))
(send a :putimage b)
(send a :flush)
