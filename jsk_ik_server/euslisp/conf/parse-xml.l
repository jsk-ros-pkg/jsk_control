
(defun event-once
  (&key
   (target "    <group name=\"left_arm\">")
   (list-target (coerce target cons))
   (mode :out)
   (depth 0)
   buf
   )
  (cond
   ((null list-target)
    (list (cons :rest list-target)
	  (cons :depth depth)
	  (cons :mode mode)))
   (t
    (case mode
	  (:skip
	   (cond
	    ((substringp "-->" target)
	     (list (cons :rest nil)
		   (cons :depth depth)
		   (cons :mode :out)))
	    (t
	     (list (cons :rest nil)
		   (cons :depth depth)
		   (cons :mode :skip)))))
	  (:out
	   (case (car list-target)
		 (#\<
		  (list (cons :rest (cdr list-target))
			(cons :depth depth)
			(cons :mode :start)))
		 (t
		  (event-once :target target
			      :list-target (cdr list-target)
			      :depth depth
			      :mode mode))))
	  (:start
	   (cond
	    ((substringp "?xml" target)
	     (list (cons :rest nil)
		   (cons :depth depth)
		   (cons :mode :out)))
	    ((and (substringp "<!--" target)
		  (substringp "-->" target))
	     (list (cons :rest nil)
		   (cons :depth depth)
		   (cons :mode :out)))
	    ((substringp "<!--" target)
	     (list (cons :rest nil)
		   (cons :depth depth)
		   (cons :mode :skip)))
	    (t
	     (case (car list-target)
		   (#\ 
		    (list
		     (cons :name (coerce (reverse buf) string))
		     (cons :rest (cdr list-target))
		     (cons :depth depth)
		     (cons :mode :in)))
		   (#\/
		    (list
		     (cons :name (coerce (reverse buf) string))
		     (cons :rest (cdr list-target))
		     (cons :depth (- depth 1))
		     (cons :mode :out)))
		   (#\>
		    (list
		     (cons :name (coerce (reverse buf) string))
		     (cons :rest (cdr list-target))
		     (cons :depth (+ 1 depth))
		     (cons :mode :out)))
		   (t
		    (event-once :target target
				:list-target (cdr list-target)
				:depth depth
				:mode mode
				:buf (cons (car list-target) buf)))))))
	  (:in
	   (case (car list-target)
		 (#\=
		  (list
		   (cons :key (coerce (reverse buf) string))
		   (cons :rest (cdr list-target))
		   (cons :depth depth)
		   (cons :mode mode)))
		 (#\ 
		  (list
		   (cons :value (coerce (reverse buf) string))
		   (cons :rest (cdr list-target))
		   (cons :depth depth)
		   (cons :mode mode)))
		 (#\/
		  (list
		   (cons :value (coerce (reverse buf) string))
		   (cons :rest (cdr list-target))
		   (cons :depth depth)
		   (cons :mode :out)))
		 (#\>
		  (list
		   (cons :value (coerce (reverse buf) string))
		   (cons :rest (cdr list-target))
		   (cons :depth (+ 1 depth))
		   (cons :mode :out)))
		 (t
		  (event-once :target target
			      :list-target (cdr list-target)
			      :depth depth
			      :mode mode
			      :buf (cons (car list-target) buf)
			      ))
		 ))))))

(defun event-until-null-string
  (&key
   (target "    <group name=\"left_arm\">")
   (list-target (coerce target cons))
   (mode :out)
   (depth 0)
   (event
    (event-once :target target
		:list-target list-target
		:mode mode
		:depth depth))
   buf
   (debug? nil)
   )
  (cond
   (debug?
    (format t "[~A] ~A~%   ~A~%" mode target event)
    (read-line)))
  (cond
   ((null (cdr (assoc :rest event))) (list event))
   ;;(list (remove-if #'(lambda (k) (eq :rest (car k))) event)))
   (t
    (cons
     event
     (event-until-null-string
      :target target
      :list-target (cdr (assoc :rest event))
      :depth (cdr (assoc :depth event))
      :mode (cdr (assoc :mode event)))))))

(defun event-from-file
  (&key
   (path "HRP2JSKNT.srdf")
   (str (open path :direction :input))
   (mode :out)
   (depth 0)
   (target (read-line str nil))
   buf)
  (cond
   ((null target) (close str) nil)
   (t
    (append
     (setq
      buf
      (event-until-null-string
       :target target
       :depth depth
       :mode mode))
     (event-from-file
      :path path
      :str str
      :mode (cdr (assoc :mode (car (last buf))))
      :depth (cdr (assoc :depth (car (last buf)))))))))

(defun find-element
  (&key
   (xml-data (event-from-file))
   (name "disable_collisions")
   (mode :skip)
   (key-list)
   (value-list)
   )
  (cond
   ((null xml-data) nil)
   ((and (eq mode :match)
	 (eq :out (cdr (assoc :mode (car xml-data)))))
    (cons
     (list (cons :key-list key-list)
	   (cons :value-list value-list))
     (find-element
      :xml-data (cdr xml-data)
      :name name
      :mode :skip)))
   ((eq mode :match)
    (find-element
     :xml-data (cdr xml-data)
     :name name
     :mode :match
     :key-list (flatten (cons (cdr (assoc :key (car xml-data))) key-list))
     :value-list (flatten (cons (cdr (assoc :value (car xml-data))) value-list))))
   ((and (cdr (assoc :name (car xml-data)))
	 (substringp name (cdr (assoc :name (car xml-data)))))
    (find-element :xml-data (cdr xml-data)
		  :name name
		  :mode :match))
   (t
    (find-element :xml-data (cdr xml-data) :name name :mode :skip))))

(defun parse-xml
  (&key
   (path "HRP2JSKNT.srdf")
   (raw-xml-data (event-from-file :path path))
   (now (car raw-xml-data))
   (depth -1)
   (mode :out)
   (name "start")
   key-list
   value-list
   (debug? nil)
   )
  (cond
   ((or (null raw-xml-data)
	(> depth (cdr (assoc :depth now))))
    (if debug? (progn (format t "[:out(~A)] ~A~%" depth now) (read-line)))
    nil)
   ((and (< depth (cdr (assoc :depth now)))
	 (or name key-list value-list
	     (cdr (assoc :name now))
	     (cdr (assoc :key now))
	     (cdr (assoc :value now))))
    (if debug? (progn (format t "[:start(~A)] ~A~%" depth now) (read-line)))
    (cons
     (list
      (cons :name (or name (cdr (assoc :name now))))
      (cons :key-list (flatten (cons (cdr (assoc :key now)) key-list)))
      (cons :value-list (flatten (cons (cdr (assoc :value now)) value-list)))
      (cons :depth (cdr (assoc :depth now)))
      (cons :members
	    (parse-xml
	     :path path :raw-xml-data (cdr raw-xml-data)
	     :depth (cdr (assoc :depth now)) :name nil :debug? debug?)))
     (parse-xml
      :path path
      :raw-xml-data
      (member-if #'(lambda (d) (eq (cdr (assoc :depth d)) depth))
		 raw-xml-data)
      :depth depth
      :name nil :debug? debug?)
     ))
   ((< depth (cdr (assoc :depth now)))
    (if debug? (progn (format t "[:start2(~A)] ~A~%" depth now) (read-line)))
    (append
     (list (cons :depth (cdr (assoc :depth now)))
	   (cons :members
		 (list
		  (parse-xml
		   :path path :raw-xml-data raw-xml-data
		   :depth (cdr (assoc :depth now))
		   :name nil :debug? debug?))))
     (parse-xml
      :path path
      :raw-xml-data
      (member-if #'(lambda (d) (eq (cdr (assoc :depth d)) depth))
		 raw-xml-data)
      :depth (cdr (assoc :depth now))
      :name nil :debug? debug?))
    )
   ((and (eq (cdr (assoc :mode now)) :out)  name)
    (if debug? (progn (format t "[:out2(~A)] ~A~%" depth now) (read-line)))
    (cons
     (list (cons :name name)
	   (cons :key-list (reverse key-list))
	   (cons :value-list (reverse value-list))
	   (cons :depth (cdr (assoc :depth now))))
     (parse-xml :path path :raw-xml-data (cdr raw-xml-data)
		:depth (cdr (assoc :depth now))
		:mode :out :name nil :debug? debug?)))
   (t
    (if debug? (progn (format t "[:in(~A)] ~A~%" depth now) (read-line)))
    (parse-xml
     :path path :raw-xml-data (cdr raw-xml-data)
     :depth (cdr (assoc :depth now))
     :mode (cdr (assoc :mode now))
     :name (or name (cdr (assoc :name now)))
     :key-list (flatten (cons (cdr (assoc :key now)) key-list))
     :value-list (flatten (cons (cdr (assoc :value now)) value-list))
     :debug? debug?))
   ))

(defun urdf2list
  (in out
      &key
      (debug? nil)
      (bind-symbol '*urdf*))
  (let ((list (parse-xml :path in :debug? debug?))
	(str (open out :direction :output)))
    (format str "(setq ~A~%'" bind-symbol)
    (pprint list str)
    (format str ")")
    (close str)
    out))
