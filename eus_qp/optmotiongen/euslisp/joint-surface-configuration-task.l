(load "package://eus_qp/optmotiongen/euslisp/instant-configuration-task.l")
(load "package://eus_qp/optmotiongen/euslisp/compound-configuration-task.l")
(load "./surface-configuration-task.l")

(unless (boundp 'compound-configuration-task) ;; dummy class definition for document
  (defclass compound-configuration-task :super object))


;;;;;;;;;;;;;;;;;;;;;;;;;
;; joint-surface-configuration-task
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass joint-surface-configuration-task
  :super compound-configuration-task
  :slots (;; config-task
          (_instant-config-task "instant-configuration-task instance")
          (_surface-kin-contact-list "list of (surface-config-task, kin-attention-name, contact-attention-name)")
          (_instant-config-distance-scale "scale for distance calculation of instant-configuration-task")
          )
  :documentation
  "
多関節機構の先に付いたボディ表面上の着目点を扱うためのコンフィギュレーションとタスク関数のクラス．

コンフィギュレーション$\bm{q}$の取得・更新，タスク関数$\bm{e}(\bm{q})$の取得，タスク関数のヤコビ行列$\frac{\partial \bm{e}(\bm{q})}{\partial \bm{q}}$の取得，コンフィギュレーションの等式・不等式制約$\bm{A}, \bm{b}, \bm{C}, \bm{d}$の取得のためのメソッドが定義されている．
  "
  )

(defmethod joint-surface-configuration-task
  (:init
   (&key
    (instant-config-task)
    (surface-kin-contact-list)
    (instant-config-distance-scale 1.0)
    )
   "
Initialize instance
   "
   (setq _instant-config-task instant-config-task)
   (setq _surface-kin-contact-list surface-kin-contact-list)
   (setq _instant-config-distance-scale instant-config-distance-scale)
   (send-super
    :init
    :config-task-list
    (append (list _instant-config-task) (mapcar #'(lambda (x) (elt x 0)) _surface-kin-contact-list)))
   (send self :_check-kin-rotation-type)
   )
  (:_check-kin-rotation-type
   (&aux
    (kin-rotation-type-list
     (send _instant-config-task :get-val '_kin-rotation-type-list))
    )
   (dolist (surface-kin-contact _surface-kin-contact-list)
     (let* ((kin-name (elt surface-kin-contact 1))
            (attention-or-target
             (cond ((and (>= (length surface-kin-contact) 4)
                         (equal (elt surface-kin-contact 3) :target))
                    :target)
                   (t
                    :attention))
             )
            (kin-idx
             (position-if
              #'(lambda (c) (equal (send c :name) kin-name))
              (cond ((equal attention-or-target :attention)
                     (send _instant-config-task :kin-attention-coords-list))
                    (t
                     (send _instant-config-task :kin-target-coords-list))))
             )
            )
       (unless (equal (elt kin-rotation-type-list kin-idx) :normal)
         (error "kin-rotation-type of body surface contact should be :normal (kin-name=~a)~%" kin-name))
       ))
   )
  (:instant-config-task
   ()
   "
return instant-configuration-task instance.
   "
   _instant-config-task
   )
  (:surface-kin-contact-list
   ()
   "
return surface-kin-contact-list, which is list of (surface-config-task, kin-attention-name, contact-attention-name).
   "
   _surface-kin-contact-list
   )
  (:set-config
   (config-new
    &key
    (relative? nil)
    &aux
    (surface-config-task)
    (config-idx)
    )
   "
Set $\bm{q}$.
   "
   ;; order of 1,2,3 is important
   (setq config-idx (send _instant-config-task :dim-config))
   (dolist (surface-kin-contact _surface-kin-contact-list)
     (setq surface-config-task (elt surface-kin-contact 0))
     ;; 1. update configuration of surface-config-task
     (send surface-config-task :set-config
           (subseq config-new config-idx (+ config-idx 2))
           :relative? relative?
           )
     (setq config-idx (+ config-idx 2))
     ;; 2. update the kinematics and contact coords of instant-configuration-task
     (send self :_update-coords :surface-kin-contact surface-kin-contact)
     )
   ;; 3. update configuration of instant-config-task (joint-angle, etc)
   (send _instant-config-task :set-config
         (subseq config-new 0 (send _instant-config-task :dim-config))
         :relative? relative?
         )
   )
  (:_update-coords
   (&key
    (surface-kin-contact)
    (surface-config-task (elt surface-kin-contact 0))
    (kin-name (elt surface-kin-contact 1))
    (contact-name (elt surface-kin-contact 2))
    (attention-or-target
     (cond ((and (>= (length surface-kin-contact) 4)
                 (equal (elt surface-kin-contact 3) :target))
            :target)
           (t
            :attention))
     )
    (kin-coords-method
     (cond ((equal attention-or-target :attention)
            :kin-attention-coords)
           (t
            :kin-target-coords))
     )
    (contact-coords-method
     (cond ((equal attention-or-target :attention)
            :contact-attention-coords)
           (t
            :contact-target-coords))
     )
    &aux
    (surface-coords
     (send surface-config-task :surface-coords))
    (orig-kin-coords
     (if kin-name
         (send _instant-config-task kin-coords-method :name kin-name)))
    (orig-contact-coords
     (if contact-name
         (send _instant-config-task contact-coords-method :name contact-name)))
    (new-kin-coords
     (if kin-name
         (make-cascoords
          :name kin-name
          :coords surface-coords
          :parent (send orig-kin-coords :parent))))
    (new-contact-coords
     (if contact-name
         (make-cascoords
          :name contact-name
          :coords surface-coords
          :parent (send orig-contact-coords :parent))))
    )
   (when kin-name
     (when (send orig-kin-coords :get :generated-in-joint-surface-configuration-task)
       (send (send orig-kin-coords :parent) :dissoc orig-kin-coords))
     (send new-kin-coords :put :generated-in-joint-surface-configuration-task t)
     (send _instant-config-task kin-coords-method
           :name kin-name
           :coords new-kin-coords
           ))
   (when contact-name
     (when (send orig-contact-coords :get :generated-in-joint-surface-configuration-task)
       (send (send orig-contact-coords :parent) :dissoc orig-contact-coords))
     (send new-contact-coords :put :generated-in-joint-surface-configuration-task t)
     (send _instant-config-task contact-coords-method
           :name contact-name
           :coords new-contact-coords
           ))
   )
  )

(defmethod joint-surface-configuration-task
  (:task-value
   (&key (update? t))
   "
return $\bm{e}(\bm{q})$
   "
   (when update?
     ;; need to update task-value of surface-config-task
     (mapcar #'(lambda (x) (send (elt x 0) :task-value :update? update?)) _surface-kin-contact-list))
   (send _instant-config-task :task-value :update? update?)
   )
  (:task-jacobian
   (&aux
    (instant-jacobi
     (send _instant-config-task :task-jacobian))
    (kin-rotation-type-list
     (send _instant-config-task :get-val '_kin-rotation-type-list))
    (config-idx)
    )
   "
return $\frac{\partial \bm{e}}{\partial \bm{q}}$
   "
   ;; 1. left side is jacobian of instant-config-task
   (cond (_task-jacobi
          (fill (array-entity _task-jacobi) 0)
          (copy-block-matrix _task-jacobi instant-jacobi 0 0)
          )
         (t
          (setq _task-jacobi
                (concatenate-matrix-row
                 instant-jacobi
                 (make-matrix
                  (length (matrix-column instant-jacobi 0))
                  (* 2 (length _surface-kin-contact-list)))
                 ))
          ))
   ;; 2. right side is jacobian of surface-config-task
   (setq config-idx (send _instant-config-task :dim-config))
   (dolist (surface-kin-contact _surface-kin-contact-list)
     (let* ((surface-config-task (elt surface-kin-contact 0))
            (kin-name (elt surface-kin-contact 1))
            (attention-or-target
             (cond ((and (>= (length surface-kin-contact) 4)
                         (equal (elt surface-kin-contact 3) :target))
                    :target)
                   (t
                    :attention))
             )
            (kin-idx
             (position-if
              #'(lambda (c) (equal (send c :name) kin-name))
              (cond ((equal attention-or-target :attention)
                     (send _instant-config-task :kin-attention-coords-list))
                    (t
                     (send _instant-config-task :kin-target-coords-list))))
             )
            (task-idx
             (apply
              #'+
              (mapcar
               #'(lambda (rot-type) (if (equal rot-type :normal) 4 6))
               (subseq kin-rotation-type-list 0 kin-idx)
               )))
            )
       (copy-block-matrix
        _task-jacobi
        (scale-matrix
         (if (equal attention-or-target :attention) 1 -1)
         (send surface-config-task :task-jacobian))
        task-idx
        config-idx
        )
       (setq config-idx (+ config-idx 2))
       ))
   _task-jacobi
   )
  )

(defmethod joint-surface-configuration-task
  (:_config-vector-for-distance
   ()
   (apply
    #'concatenate
    float-vector
    (append
     (list (scale _instant-config-distance-scale (send _instant-config-task :config-vector)))
     (send-all (mapcar #'(lambda (x) (elt x 0)) _surface-kin-contact-list) :surface-pos)
     ))
   )
  (:distance-vector
   (config-task)
   "
return distance $\bm{d}$, which is used for sqp-msc (sqp with multiple solution candidates).
   "
   (v- (send self :_config-vector-for-distance) (send config-task :_config-vector-for-distance))
   )
  (:distance-jacobian
   ()
   "
return distance jacobian $\bm{J}_{d}$, which is used for sqp-msc (sqp with multiple solution candidates).
   "
   (apply
    #'concatenate-matrix-diagonal
    (append
     (list (scale-matrix _instant-config-distance-scale (unit-matrix (send _instant-config-task :dim-config))))
     (send-all (mapcar #'(lambda (x) (elt x 0)) _surface-kin-contact-list) :distance-jacobian)
     ))
   )
  )
