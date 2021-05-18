;;copied from optmotiongen

(defun concatenate-matrix-column
    (&rest args)
  "
Concatenate matrix in column direction.
Override to support zero row matrix.
  "
  (let (m ml (size-of-column (array-dimension (car args) 1)))
    (dolist (mat args)
      (unless (= (array-dimension mat 1) size-of-column)
        (error ";; concatenate-matrix-column matrix size error (size=~A)~%" (mapcar #'(lambda (x) (array-dimension x 1)) args)))
      (setq m (if mat (array-dimension mat 0) 0))
      (dotimes (i m)
        (push (matrix-row mat i) ml)))
    (cond (ml (apply #'matrix (reverse ml)))
          (t (make-matrix 0 size-of-column)))
    ))

(defun concatenate-matrix-row
    (&rest args)
  "
Concatenate matrix in row direction.
Override to support column row matrix.
  "
  (let (m ml (size-of-column (array-dimension (car args) 0)))
    (dolist (mat args)
      (unless (= (array-dimension mat 0) size-of-column)
        (error ";; concatenate-matrix-row matrix size error (size=~A)~%" (mapcar #'(lambda (x) (array-dimension x 0)) args)))
      (setq m (if mat (array-dimension mat 1) 0))
      (dotimes (i m)
        (push (matrix-column mat i) ml)))
    (cond (ml (transpose (apply #'matrix (reverse ml))))
          (t (make-matrix size-of-column 0)))
    ))

(defun concatenate-matrix-diagonal
  (&rest args)
  "
Concatenate matrix in diagonal.
Override to support zero row matrix.
  "
  (when (every #'(lambda (mm) (= (array-dimension mm 0) 0)) args)
    (return-from concatenate-matrix-diagonal
                 (make-matrix 0 (apply #'+ (mapcar #'(lambda (mm) (array-dimension mm 1)) args)))))
  (let (mat m ll ml vl)
    (dolist (mm args)
      (push (if mm (array-dimension mm 1) 0) ll))
    (setq ll (reverse ll))
    (dotimes (i (length args))
      (setq mat (nth i args))
      (setq m (if mat (array-dimension mat 0) 0))
      (dotimes (j m)
        (setq vl nil)
        (dotimes (k (length ll))
          (if (= i k) (push (matrix-row mat j) vl)
            (push (make-array (nth k ll)
                              :element-type float-vector
                              :initial-element 0) vl)))
        (push (apply #'concatenate
                     (cons float-vector (reverse vl))) ml)))
    (when ml (apply #'matrix (reverse ml)))
    ))

(defun block-matrix
  (m &optional (i 0) (j 0) (ii nil) (jj nil))
  ;; i, j : start row and column idx
  ;; ii, jj : row and column length of block matrix
  (unless ii (setq ii (- (array-dimension m 0) i)))
  (unless jj (setq jj (- (array-dimension m 1) j)))
  (let ((ret (make-matrix ii jj)))
    (dotimes (_i ii)
      (dotimes (_j jj)
        (setf (aref ret _i _j) (aref m (+ _i i) (+ _j j)))))
    ret))

(defun block-vec
  ;; i : start idx
  ;; ii : length of block vector
  (m &optional (i 0) (ii nil))
  (unless ii (setq ii (- (length m) i)))
  (let ((ret (instantiate float-vector ii)))
    (dotimes (_i ii)
      (setf (aref ret _i) (aref m (+ _i i))))
    ret))

(defun copy-block-matrix
  (m bm &optional (i 0) (j 0))
  (dotimes (_i (length (matrix-column bm 0)))
    (dotimes (_j (length (matrix-row bm 0)))
      (setf (aref m (+ _i i) (+ _j j)) (aref bm _i _j))))
  m)

(defun copy-block-vec
  (m bm &optional (i 0))
  (dotimes (_i (length bm))
    (setf (aref m (+ _i i)) (aref bm _i)))
  m)

(defun range (n)
  (cond ((<= n 0)
         (return-from range nil))
        ((= n 1)
         (list 0))
        (t
         (append (range (- n 1)) (list (- n 1))))))

;; copy end

(defun fill-block-matrix
  (m num &optional (i 0) (j 0) (row 0) (col 0))
  (dotimes (_i row)
    (dotimes (_j col)
      (setf (aref m (+ _i i) (+ _j j)) num)))
  m)

(defun fill-block-vec
  (m num &optional (i 0) (len 0))
  (dotimes (_i len)
    (setf (aref m (+ _i i)) num))
  m)

(defun copy-block-colvector
  (m bm &optional (i 0) (j 0))
  (dotimes (_i (length bm))
    (setf (aref m (+ _i i) j) (elt bm _i)))
  m)

(defun copy-block-rowvector
  (m bm &optional (i 0) (j 0))
  (dotimes (_j (length bm))
    (setf (aref m i (+ _j j)) (elt bm _j)))
  m)

(defun get-link-list
    (to)
  (cond
    ((derivedp to bodyset-link)
     (append (get-link-list (send to :parent-link)) (list to)))
    ((derivedp to cascaded-coords)
     (get-link-list (send to :parent)))
    (t
     nil)))

(defmethod coordinates
  (:inverse-axis (axis)
   (transform (transpose (send self :worldrot))
              (case axis
                (:x #f(1 0 0))
                (:y #f(0 1 0))
                (:z #f(0 0 1))))
   )
  (:difference-position
   (coords &key (translation-axis t) (world :local))
   "return difference in position of given coords, translation-axis can take (:x, :y, :z, :xy, :yz, :zx)."
   (let ((dif-pos
          (cond 
            ((equal world :local) (send self :inverse-transform-vector (send coords :worldpos)))
            ((equal world :world) (v- (send coords :worldpos) (send self :worldpos)))
            (t (transform (transpose (send world :worldrot)) (v- (send coords :worldpos) (send self :worldpos)))))))
    (case
     translation-axis
      ((:x :xx) (setf (elt dif-pos 0) 0))
      ((:y :yy) (setf (elt dif-pos 1) 0))
      ((:z :zz) (setf (elt dif-pos 2) 0))
      ((:xy :yx) (setf (elt dif-pos 0) 0) (setf (elt dif-pos 1) 0))
      ((:yz :zy) (setf (elt dif-pos 1) 0) (setf (elt dif-pos 2) 0))
      ((:zx :xz) (setf (elt dif-pos 2) 0) (setf (elt dif-pos 0) 0)))
    dif-pos))
  (:difference-rotation
   (coords &key (rotation-axis t) (world :local))
   "return difference in rotation of given coords, rotation-axis can take (:x, :y, :z, :xx, :yy, :zz, :xm, :ym, :zm)"
   (labels
    ((need-mirror-for-nearest-axis
      (coords0 coords1 axis)
      (let* ((a0 (send coords0 :axis axis))
             (a1 (send coords1 :axis axis))
             (a1m (v- a1))
             (dr1 (scale (acos (v. a0 a1)) (normalize-vector (v* a0 a1))))
             (dr1m (scale (acos (v. a0 a1m)) (normalize-vector (v* a0 a1m)))))
        (< (norm dr1) (norm dr1m)))))
     (let (dif-rotmatrix dif-rot a0 a1)
       (cond
         ((equal world :local)
          (case
           rotation-axis
            ((:x :y :z)
             (setq a0 (send self :axis rotation-axis)
                   a1 (send coords :axis rotation-axis))
             (setq dif-rot
                   (transform (transpose (send self :worldrot))
                              (scale (acos (v. a0 a1)) (normalize-vector (v* a0 a1))))))
            ((:xx :yy :zz)
             (let ((axis (case rotation-axis (:xx :x) (:yy :y) (:zz :z))) a0 a2)
               (setq a0 (send self :axis axis))
               (setq a2 (send coords :axis axis))
               (unless (need-mirror-for-nearest-axis self coords axis) (setq a2 (v- a2)))
               (setq dif-rot (transform (transpose (send self :worldrot))
                                        (scale (acos (v. a0 a2)) (normalize-vector (v* a0 a2)))))))
            ((:xm :ym :zm)
             (let ((rot (send coords :worldrot)))
               (unless (need-mirror-for-nearest-axis self coords (case rotation-axis (:xm :y) (:ym :z) (:zm :x)))
                 (setq rot (rotate-matrix rot pi (case rotation-axis (:xm :x) (:ym :y) (:zm :z)))))
               (setq dif-rotmatrix (m* (transpose (send self :worldrot)) rot))
               (setq dif-rot (user::matrix-log dif-rotmatrix))
               ))
            (nil
             (setq dif-rot (float-vector 0 0 0)))
            (t
                                  (setq dif-rotmatrix (m* (transpose (send self :worldrot)) (send coords :worldrot)))
                                  (setq dif-rot (user::matrix-log dif-rotmatrix))
                                  )))
         ((equal world :world)
          (case
           rotation-axis
            ((:x :y :z)
             (setq a0 (send self :inverse-axis rotation-axis)
                   a1 (send coords :inverse-axis rotation-axis))
             (setq dif-rot
                   (send self :rotate-vector
                         (scale (- (acos (v. a0 a1))) (normalize-vector (v* a0 a1))))))
            ((:xx :yy :zz)
             (error "not supported rotation-axis ~A" rotation-axis))
            ((:xm :ym :zm)
             (error "not supported rotation-axis ~A" rotation-axis))
            (nil
             (setq dif-rot (float-vector 0 0 0)))
            (t
             (setq dif-rotmatrix (m* (transpose (send self :worldrot)) (send coords :worldrot)))
             (setq dif-rot (send self :rotate-vector (user::matrix-log dif-rotmatrix)))
             )))
         (t
          (case
           rotation-axis
            ((:x :y :z)
             (setq a0 (transform (transpose (send self :worldrot)) (send world :axis rotation-axis))
                   a1 (transform (transpose (send coords :worldrot)) (send world :axis rotation-axis)))
             (setq dif-rot
                   (transform (transpose (send world :worldrot))
                              (send self :rotate-vector
                                    (scale (- (acos (v. a0 a1))) (normalize-vector (v* a0 a1)))))))
            ((:xx :yy :zz)
             (error "not supported rotation-axis ~A" rotation-axis))
            ((:xm :ym :zm)
             (error "not supported rotation-axis ~A" rotation-axis))
            (nil
             (setq dif-rot (float-vector 0 0 0)))
            (t
             (setq dif-rotmatrix (m* (transpose (send self :worldrot)) (send coords :worldrot)))
             (setq dif-rot (transform (transpose (send world :worldrot)) (send self :rotate-vector (user::matrix-log dif-rotmatrix))))
             ))))
       dif-rot)))
  )


(defmethod robot-model
  (:make-sole-polygon-ex
   (name) ;;name of link
   (unless (send self :link name) (return-from :make-sole-polygon-ex nil))
   (send (send self :link name) :worldcoords)
   (let* ((target-nm (read-from-string (format nil "~A-sole-body" name)))
          (vs (remove-duplicates
                 (flatten (send-all (flatten (send (send self :link name) :bodies)) :vertices))
                 :test #'(lambda (x y) (eps-v= x y *epsilon*))))
          (min-vs (find-extream vs #'(lambda (x) (elt x 2)) #'<))
          (b (make-prism (quickhull
                          (remove-if
                           #'(lambda (p)
                               (< 5.0 (- (elt p 2) (elt min-vs 2)))) vs)) ;; 5.0 is margin
                         #F(0 0 1)
                         :name target-nm
                         ))
          )
     (send (send self :link name) :assoc b)
     (send self :put target-nm b)
     (send b :worldcoords)
     (let ((f (find-if #'(lambda (x) (memq :bottom (send x :id))) (send b :faces))))
       (send f :name name)
       f))))
