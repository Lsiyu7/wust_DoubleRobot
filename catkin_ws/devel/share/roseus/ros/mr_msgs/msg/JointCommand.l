;; Auto-generated. Do not edit!


(when (boundp 'mr_msgs::JointCommand)
  (if (not (find-package "MR_MSGS"))
    (make-package "MR_MSGS"))
  (shadow 'JointCommand (find-package "MR_MSGS")))
(unless (find-package "MR_MSGS::JOINTCOMMAND")
  (make-package "MR_MSGS::JOINTCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude JointCommand.msg.html


(intern "*CURRENT_CMD*" (find-package "MR_MSGS::JOINTCOMMAND"))
(shadow '*CURRENT_CMD* (find-package "MR_MSGS::JOINTCOMMAND"))
(defconstant mr_msgs::JointCommand::*CURRENT_CMD* 1)
(intern "*SPEED_CMD*" (find-package "MR_MSGS::JOINTCOMMAND"))
(shadow '*SPEED_CMD* (find-package "MR_MSGS::JOINTCOMMAND"))
(defconstant mr_msgs::JointCommand::*SPEED_CMD* 2)
(intern "*POSITION_CMD*" (find-package "MR_MSGS::JOINTCOMMAND"))
(shadow '*POSITION_CMD* (find-package "MR_MSGS::JOINTCOMMAND"))
(defconstant mr_msgs::JointCommand::*POSITION_CMD* 3)
(intern "*TRAJECTORY_CMD*" (find-package "MR_MSGS::JOINTCOMMAND"))
(shadow '*TRAJECTORY_CMD* (find-package "MR_MSGS::JOINTCOMMAND"))
(defconstant mr_msgs::JointCommand::*TRAJECTORY_CMD* 3)
(defclass mr_msgs::JointCommand
  :super ros::object
  :slots (_command _names _cmdPos _cmdspd _cmdCurr ))

(defmethod mr_msgs::JointCommand
  (:init
   (&key
    ((:command __command) 0)
    ((:names __names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:cmdPos __cmdPos) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:cmdspd __cmdspd) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:cmdCurr __cmdCurr) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _command (round __command))
   (setq _names __names)
   (setq _cmdPos __cmdPos)
   (setq _cmdspd __cmdspd)
   (setq _cmdCurr __cmdCurr)
   self)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:names
   (&optional __names)
   (if __names (setq _names __names)) _names)
  (:cmdPos
   (&optional __cmdPos)
   (if __cmdPos (setq _cmdPos __cmdPos)) _cmdPos)
  (:cmdspd
   (&optional __cmdspd)
   (if __cmdspd (setq _cmdspd __cmdspd)) _cmdspd)
  (:cmdCurr
   (&optional __cmdCurr)
   (if __cmdCurr (setq _cmdCurr __cmdCurr)) _cmdCurr)
  (:serialization-length
   ()
   (+
    ;; int32 _command
    4
    ;; string[] _names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _names)) 4
    ;; float64[] _cmdPos
    (* 8    (length _cmdPos)) 4
    ;; float64[] _cmdspd
    (* 8    (length _cmdspd)) 4
    ;; float64[] _cmdCurr
    (* 8    (length _cmdCurr)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _command
       (write-long _command s)
     ;; string[] _names
     (write-long (length _names) s)
     (dolist (elem _names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float64[] _cmdPos
     (write-long (length _cmdPos) s)
     (dotimes (i (length _cmdPos))
       (sys::poke (elt _cmdPos i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _cmdspd
     (write-long (length _cmdspd) s)
     (dotimes (i (length _cmdspd))
       (sys::poke (elt _cmdspd i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _cmdCurr
     (write-long (length _cmdCurr) s)
     (dotimes (i (length _cmdCurr))
       (sys::poke (elt _cmdCurr i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _command
     (setq _command (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string[] _names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float64[] _cmdPos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cmdPos (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _cmdPos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _cmdspd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cmdspd (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _cmdspd i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _cmdCurr
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cmdCurr (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _cmdCurr i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get mr_msgs::JointCommand :md5sum-) "316fe6f888e87acbb2cf55d67ad8f983")
(setf (get mr_msgs::JointCommand :datatype-) "mr_msgs/JointCommand")
(setf (get mr_msgs::JointCommand :definition-)
      "int32 command
string[]  names
float64[] cmdPos
float64[] cmdspd
float64[] cmdCurr

int32 CURRENT_CMD=1
int32 SPEED_CMD=2
int32 POSITION_CMD=3
int32 TRAJECTORY_CMD=3

")



(provide :mr_msgs/JointCommand "316fe6f888e87acbb2cf55d67ad8f983")


