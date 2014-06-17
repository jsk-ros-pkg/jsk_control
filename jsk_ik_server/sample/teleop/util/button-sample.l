
(require "image-panel.l")

(defun click-callback
  (obj &rest args)
  ;; overwrite here
  (pprint (send obj :slots))
  )

;; CAUSION!!: use :putimage method
(defun draw-image-on-panel
  (image panel
   &key
   (x 3)
   (y 3)
   (skip-func #'(lambda (col) (eq #xFF000000 (logand #FF000000 col))))
   col
   &allow-other-keys)
  ;; (send panel
  ;; 	:putimage
  ;; 	image
  ;; 	:dst-x x :dst-y y)
  (dotimes (dx (send image :width))
    (dotimes (dy (send image :height))
      (setq col (send image :pixel dx dy))
      (unless
  	  (funcall skip-func col)
  	(send panel :draw-point
  	      (integer-vector (+ x dx) (+ y dy))
  	      (instance x::gcontext
  			:create
  			:foreground col)))))
  t)

;; create window to set joint-angle
(defclass button-sample
  :super x::panel
  :slots (button-list
	  name-list
	  image-list
	  )
  )

(defmethod button-sample
  (:create
   (&key
    ((:name-list nl) (list "BACK" "PLAY" "NEXT"))
    ((:image-list il) (list nil nil nil))
    (w 50) (h 50)
    (width (+ 20 (* w (length nl))))
    (height (+ 10 h))
    (window-name "button-sample")
    &rest
    args)
   (send-super* :create
		:title window-name
		:width width :height height
		:font x::font-courb12
		:background #xFFEEEE
		args)
   (setq name-list nl)
   (setq image-list il)
   (setq button-list
	 (mapcar #'(lambda (name)
		     (send-super :create-item x::button-item
				 name
				 self
				 :pre-button-callback
				 :width w :height h
				 :background #xEEEEFF))
		 name-list))
   (mapcar
    #'(lambda (bt im)
	(send bt :put :image im))
    button-list image-list)
   self)
  (:pre-button-callback
   (obj &rest args) ;; bug?
   (apply #'click-callback
	  (cons obj args))
   t)
  (:nomethod
   (&rest args)
   (let (sym val)
     (cond
      ((keywordp (car args))
       (setq sym (read-from-string (send (car args) :pname)))
       (setq val (assoc sym (send self :slots)))))
     (cond
      ((or (null sym) (null val)) nil)
      ((> (length args) 1)
       (eval (list 'setq sym '(cadr args))))
      (t (cdr val)))))
  )

(defmethod x::button-item
  (:draw-label
   (&optional
    (state :up) (color x::bg-color) (border 2) (offset)
    (image (send self :get :image)))
   (send self :3d-fill-rectangle  0 0 x::width x::height border
	 x::light-edge-color x::dark-edge-color x::bg-color
	 x::topleft-edge-polygon state)
   (unless offset
     (setq offset
	   (cdr (assoc state '((:up . 0) (:down . 2) (:flat . 1))))))
   (unless offset (setq offset 0))
   (cond
    (image
     (send self :putimage
	   image
	   :dst-x offset :dst-y offset)))
   (send self :string
	 (+ ;4
	  (/ x::width 2)
	  (- (/ (aref x::labeldots 2) 2))
	  offset)			;x
	 (+ ;; (aref labeldots 0) 2 
	  (/ x::height 2) (aref x::labeldots 0)
	  (- (/ (+ (aref x::labeldots 0) (aref x::labeldots 1)) 2))
	  offset)	;y
	 x::label))
  (send self :flush))

#|

(setq a
      (instance button-sample :create
		:image-list
		(mapcar
		 #'(lambda (path)
		     (reverse-pixel
		      (resize-image
		       (img::read-image-file (format nil "image/~A" path))
		       48 48)))
		 (list "prev.png" "play.png" "next.png"))))
(send a :flush)
