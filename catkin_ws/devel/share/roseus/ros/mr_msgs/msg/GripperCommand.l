;; Auto-generated. Do not edit!


(when (boundp 'mr_msgs::GripperCommand)
  (if (not (find-package "MR_MSGS"))
    (make-package "MR_MSGS"))
  (shadow 'GripperCommand (find-package "MR_MSGS")))
(unless (find-package "MR_MSGS::GRIPPERCOMMAND")
  (make-package "MR_MSGS::GRIPPERCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude GripperCommand.msg.html


(defclass mr_msgs::GripperCommand
  :super ros::object
  :slots (_name _positionL _positionR ))

(defmethod mr_msgs::GripperCommand
  (:init
   (&key
    ((:name __name) "")
    ((:positionL __positionL) 0.0)
    ((:positionR __positionR) 0.0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _positionL (float __positionL))
   (setq _positionR (float __positionR))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:positionL
   (&optional __positionL)
   (if __positionL (setq _positionL __positionL)) _positionL)
  (:positionR
   (&optional __positionR)
   (if __positionR (setq _positionR __positionR)) _positionR)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; float64 _positionL
    8
    ;; float64 _positionR
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; float64 _positionL
       (sys::poke _positionL (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _positionR
       (sys::poke _positionR (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _positionL
     (setq _positionL (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _positionR
     (setq _positionR (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get mr_msgs::GripperCommand :md5sum-) "bacced25655daea064ca2c4b7a402fd6")
(setf (get mr_msgs::GripperCommand :datatype-) "mr_msgs/GripperCommand")
(setf (get mr_msgs::GripperCommand :definition-)
      "string  name
float64 positionL
float64 positionR

")



(provide :mr_msgs/GripperCommand "bacced25655daea064ca2c4b7a402fd6")


