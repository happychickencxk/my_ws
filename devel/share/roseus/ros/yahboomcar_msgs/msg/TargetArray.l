;; Auto-generated. Do not edit!


(when (boundp 'yahboomcar_msgs::TargetArray)
  (if (not (find-package "YAHBOOMCAR_MSGS"))
    (make-package "YAHBOOMCAR_MSGS"))
  (shadow 'TargetArray (find-package "YAHBOOMCAR_MSGS")))
(unless (find-package "YAHBOOMCAR_MSGS::TARGETARRAY")
  (make-package "YAHBOOMCAR_MSGS::TARGETARRAY"))

(in-package "ROS")
;;//! \htmlinclude TargetArray.msg.html


(defclass yahboomcar_msgs::TargetArray
  :super ros::object
  :slots (_data ))

(defmethod yahboomcar_msgs::TargetArray
  (:init
   (&key
    ((:data __data) (let (r) (dotimes (i 0) (push (instance yahboomcar_msgs::Target :init) r)) r))
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&rest __data)
   (if (keywordp (car __data))
       (send* _data __data)
     (progn
       (if __data (setq _data (car __data)))
       _data)))
  (:serialization-length
   ()
   (+
    ;; yahboomcar_msgs/Target[] _data
    (apply #'+ (send-all _data :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; yahboomcar_msgs/Target[] _data
     (write-long (length _data) s)
     (dolist (elem _data)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; yahboomcar_msgs/Target[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (let (r) (dotimes (i n) (push (instance yahboomcar_msgs::Target :init) r)) r))
     (dolist (elem- _data)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get yahboomcar_msgs::TargetArray :md5sum-) "d6166ed47e2fc687ee6aed6f7960f37b")
(setf (get yahboomcar_msgs::TargetArray :datatype-) "yahboomcar_msgs/TargetArray")
(setf (get yahboomcar_msgs::TargetArray :definition-)
      "yahboomcar_msgs/Target[] data

================================================================================
MSG: yahboomcar_msgs/Target
string frame_id
time stamp
float32 scores
float32 ptx
float32 pty
float32 distw
float32 disth
float32 centerx
float32 centery
float32 distance_x
float32 distance_y
float32 distance
")



(provide :yahboomcar_msgs/TargetArray "d6166ed47e2fc687ee6aed6f7960f37b")


