;;copied from optmotiongen

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


;; copy end
