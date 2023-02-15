;; Auto-generated. Do not edit!


(when (boundp 'mr_msgs::GripperState)
  (if (not (find-package "MR_MSGS"))
    (make-package "MR_MSGS"))
  (shadow 'GripperState (find-package "MR_MSGS")))
(unless (find-package "MR_MSGS::GRIPPERSTATE")
  (make-package "MR_MSGS::GRIPPERSTATE"))

(in-package "ROS")
;;//! \htmlinclude GripperState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*OPEN*" (find-package "MR_MSGS::GRIPPERSTATE"))
(shadow '*OPEN* (find-package "MR_MSGS::GRIPPERSTATE"))
(defconstant mr_msgs::GripperState::*OPEN* 1)
(intern "*CLOSE*" (find-package "MR_MSGS::GRIPPERSTATE"))
(shadow '*CLOSE* (find-package "MR_MSGS::GRIPPERSTATE"))
(defconstant mr_msgs::GripperState::*CLOSE* 0)
(defclass mr_msgs::GripperState
  :super ros::object
  :slots (_header _names _positionsL _positionsR _torqueL _torqueR ))

(defmethod mr_msgs::GripperState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:names __names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:positionsL __positionsL) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:positionsR __positionsR) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:torqueL __torqueL) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:torqueR __torqueR) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _names __names)
   (setq _positionsL __positionsL)
   (setq _positionsR __positionsR)
   (setq _torqueL __torqueL)
   (setq _torqueR __torqueR)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:names
   (&optional __names)
   (if __names (setq _names __names)) _names)
  (:positionsL
   (&optional __positionsL)
   (if __positionsL (setq _positionsL __positionsL)) _positionsL)
  (:positionsR
   (&optional __positionsR)
   (if __positionsR (setq _positionsR __positionsR)) _positionsR)
  (:torqueL
   (&optional __torqueL)
   (if __torqueL (setq _torqueL __torqueL)) _torqueL)
  (:torqueR
   (&optional __torqueR)
   (if __torqueR (setq _torqueR __torqueR)) _torqueR)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string[] _names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _names)) 4
    ;; float64[] _positionsL
    (* 8    (length _positionsL)) 4
    ;; float64[] _positionsR
    (* 8    (length _positionsR)) 4
    ;; float64[] _torqueL
    (* 8    (length _torqueL)) 4
    ;; float64[] _torqueR
    (* 8    (length _torqueR)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string[] _names
     (write-long (length _names) s)
     (dolist (elem _names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float64[] _positionsL
     (write-long (length _positionsL) s)
     (dotimes (i (length _positionsL))
       (sys::poke (elt _positionsL i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _positionsR
     (write-long (length _positionsR) s)
     (dotimes (i (length _positionsR))
       (sys::poke (elt _positionsR i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _torqueL
     (write-long (length _torqueL) s)
     (dotimes (i (length _torqueL))
       (sys::poke (elt _torqueL i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _torqueR
     (write-long (length _torqueR) s)
     (dotimes (i (length _torqueR))
       (sys::poke (elt _torqueR i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string[] _names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float64[] _positionsL
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _positionsL (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _positionsL i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _positionsR
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _positionsR (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _positionsR i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _torqueL
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _torqueL (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _torqueL i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _torqueR
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _torqueR (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _torqueR i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get mr_msgs::GripperState :md5sum-) "3e09f13f34e6efdfa5ea653e97d62200")
(setf (get mr_msgs::GripperState :datatype-) "mr_msgs/GripperState")
(setf (get mr_msgs::GripperState :definition-)
      "std_msgs/Header header
string[]  names
float64[] positionsL
float64[] positionsR
float64[] torqueL
float64[] torqueR

int32 OPEN=1
int32 CLOSE=0

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :mr_msgs/GripperState "3e09f13f34e6efdfa5ea653e97d62200")


