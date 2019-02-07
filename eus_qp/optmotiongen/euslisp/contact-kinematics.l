(load "./util.l")


;;;;;;;;;;;;;;;;;;;;;;;;;
;; 2d-planar-contact
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass 2d-planar-contact
  :super cascaded-link
  :slots ((_contact-coords "$T_{\mathit{cnt}}$")
          (_contact-pre-coords "$T_{\mathit{cnt\mathchar`-pre}}$")
          )
  :documentation
  "
二次元平面上の長方形領域での接触座標を表す仮想の関節・リンク構造．
  "
  )

(defmethod 2d-planar-contact
  (:init
   (&key
    (name "contact")
    (contact-pre-offset 100) ;; [mm]
    )
   "
Initialize instance
   "
   (send-super :init :name name)
   (let (vl-list-root vj-list-root)
     ;; 1. setup link
     (setq vl-list-root
           (mapcar #'(lambda (i)
                       (instance bodyset-link :init (make-cascoords) :bodies (list (make-cube 1e-3 1e-3 1e-3))
                                 :name (read-from-string (format nil ":~a-virtual-~a" name i))
                                 :weight 0.0
                                 ))
                   (range 4)))
     (dotimes (i 3)
       (send (elt vl-list-root i) :assoc (elt vl-list-root (1+ i))))
     (send self :assoc (car vl-list-root))
     (setq _contact-coords (make-cascoords :parent (car (last vl-list-root))))
     (setq _contact-pre-coords (make-cascoords :pos (float-vector 0 0 contact-pre-offset) :parent (car (last vl-list-root))))
     (setq links vl-list-root)
     ;; 2. setup joint
     (setq vj-list-root
           (mapcar #'(lambda (joint-class
                              joint-axis
                              joint-name
                              parent-link
                              child-link
                              )
                       (instance joint-class :init
                                 :name joint-name
                                 :parent-link parent-link :child-link child-link
                                 :axis joint-axis :min -1e10 :max 1e10
                                 :max-joint-torque 0.0
                                 ))
                   (list linear-joint linear-joint rotational-joint)
                   (list :x :y :z)
                   (list (read-from-string (format nil ":~a-root-virtual-x" (send self :name)))
                         (read-from-string (format nil ":~a-root-virtual-y" (send self :name)))
                         (read-from-string (format nil ":~a-root-virtual-yaw" (send self :name))))
                   (butlast vl-list-root)
                   (cdr vl-list-root)
                   ))
     (setq joint-list vj-list-root)
     (send self :init-ending)
     self
     ))
  (:contact-coords
   (&rest args)
   "
return $T_{\mathit{cnt}} := \{ \bm{p}_{\mathit{cnt}}, \bm{R}_{\mathit{cnt}} \}$
   "
   (forward-message-to _contact-coords args)
   )
  (:contact-pre-coords
   (&rest args)
   "
return $T_{\mathit{cnt\mathchar`-pre}} := \{ \bm{p}_{\mathit{cnt\mathchar`-pre}}, \bm{R}_{\mathit{cnt\mathchar`-pre}} \}$
   "
   (forward-message-to _contact-pre-coords args)
   )
  (:set-from-face
   (&key
    (face)
    (margin 150.0) ;; [mm]
    )
   "
set coords and min/max joint angle from face.
   "
   (let* ((x-edge (elt (send face :edges) 0))
          (x-length (max 0.0 (- (send x-edge :length) (* 2 margin))))
          (x-vec (send x-edge :direction))
          (y-edge (elt (send face :edges) 1))
          (y-length (max 0.0 (- (send y-edge :length) (* 2 margin))))
          (y-vec (send y-edge :direction))
          (z-vec (send face :normal))
          (rot (transpose (matrix x-vec y-vec z-vec)))
          (pos (elt (send face :centroid) 1))
          )
     (send self :newcoords (make-coords :pos pos :rot rot))
     (send (elt joint-list 0) :max-angle (* 0.5 x-length))
     (send (elt joint-list 0) :min-angle (* -0.5 x-length))
     (send (elt joint-list 1) :max-angle (* 0.5 y-length))
     (send (elt joint-list 1) :min-angle (* -0.5 y-length))
     ))
  )
