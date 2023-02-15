; Auto-generated. Do not edit!


(cl:in-package csv_msgs-msg)


;//! \htmlinclude csvfile.msg.html

(cl:defclass <csvfile> (roslisp-msg-protocol:ros-message)
  ((Time
    :reader Time
    :initarg :Time
    :type cl:float
    :initform 0.0)
   (RightC7Shoulder
    :reader RightC7Shoulder
    :initarg :RightC7Shoulder
    :type cl:float
    :initform 0.0)
   (RightShoulder
    :reader RightShoulder
    :initarg :RightShoulder
    :type cl:float
    :initform 0.0)
   (RightElbow
    :reader RightElbow
    :initarg :RightElbow
    :type cl:float
    :initform 0.0)
   (RightWrist
    :reader RightWrist
    :initarg :RightWrist
    :type cl:float
    :initform 0.0))
)

(cl:defclass csvfile (<csvfile>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <csvfile>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'csvfile)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name csv_msgs-msg:<csvfile> is deprecated: use csv_msgs-msg:csvfile instead.")))

(cl:ensure-generic-function 'Time-val :lambda-list '(m))
(cl:defmethod Time-val ((m <csvfile>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader csv_msgs-msg:Time-val is deprecated.  Use csv_msgs-msg:Time instead.")
  (Time m))

(cl:ensure-generic-function 'RightC7Shoulder-val :lambda-list '(m))
(cl:defmethod RightC7Shoulder-val ((m <csvfile>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader csv_msgs-msg:RightC7Shoulder-val is deprecated.  Use csv_msgs-msg:RightC7Shoulder instead.")
  (RightC7Shoulder m))

(cl:ensure-generic-function 'RightShoulder-val :lambda-list '(m))
(cl:defmethod RightShoulder-val ((m <csvfile>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader csv_msgs-msg:RightShoulder-val is deprecated.  Use csv_msgs-msg:RightShoulder instead.")
  (RightShoulder m))

(cl:ensure-generic-function 'RightElbow-val :lambda-list '(m))
(cl:defmethod RightElbow-val ((m <csvfile>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader csv_msgs-msg:RightElbow-val is deprecated.  Use csv_msgs-msg:RightElbow instead.")
  (RightElbow m))

(cl:ensure-generic-function 'RightWrist-val :lambda-list '(m))
(cl:defmethod RightWrist-val ((m <csvfile>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader csv_msgs-msg:RightWrist-val is deprecated.  Use csv_msgs-msg:RightWrist instead.")
  (RightWrist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <csvfile>) ostream)
  "Serializes a message object of type '<csvfile>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'RightC7Shoulder))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'RightShoulder))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'RightElbow))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'RightWrist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <csvfile>) istream)
  "Deserializes a message object of type '<csvfile>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Time) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'RightC7Shoulder) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'RightShoulder) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'RightElbow) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'RightWrist) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<csvfile>)))
  "Returns string type for a message object of type '<csvfile>"
  "csv_msgs/csvfile")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'csvfile)))
  "Returns string type for a message object of type 'csvfile"
  "csv_msgs/csvfile")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<csvfile>)))
  "Returns md5sum for a message object of type '<csvfile>"
  "34dbd2050eabe306f510a061084b9163")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'csvfile)))
  "Returns md5sum for a message object of type 'csvfile"
  "34dbd2050eabe306f510a061084b9163")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<csvfile>)))
  "Returns full string definition for message of type '<csvfile>"
  (cl:format cl:nil "float32 Time~%float32 RightC7Shoulder~%float32 RightShoulder~%float32 RightElbow~%float32 RightWrist~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'csvfile)))
  "Returns full string definition for message of type 'csvfile"
  (cl:format cl:nil "float32 Time~%float32 RightC7Shoulder~%float32 RightShoulder~%float32 RightElbow~%float32 RightWrist~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <csvfile>))
  (cl:+ 0
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <csvfile>))
  "Converts a ROS message object to a list"
  (cl:list 'csvfile
    (cl:cons ':Time (Time msg))
    (cl:cons ':RightC7Shoulder (RightC7Shoulder msg))
    (cl:cons ':RightShoulder (RightShoulder msg))
    (cl:cons ':RightElbow (RightElbow msg))
    (cl:cons ':RightWrist (RightWrist msg))
))
