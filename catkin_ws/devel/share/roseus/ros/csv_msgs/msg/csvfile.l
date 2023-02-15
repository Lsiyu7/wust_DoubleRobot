;; Auto-generated. Do not edit!


(when (boundp 'csv_msgs::csvfile)
  (if (not (find-package "CSV_MSGS"))
    (make-package "CSV_MSGS"))
  (shadow 'csvfile (find-package "CSV_MSGS")))
(unless (find-package "CSV_MSGS::CSVFILE")
  (make-package "CSV_MSGS::CSVFILE"))

(in-package "ROS")
;;//! \htmlinclude csvfile.msg.html


(defclass csv_msgs::csvfile
  :super ros::object
  :slots (_Time _RightC7Shoulder _RightShoulder _RightElbow _RightWrist ))

(defmethod csv_msgs::csvfile
  (:init
   (&key
    ((:Time __Time) 0.0)
    ((:RightC7Shoulder __RightC7Shoulder) 0.0)
    ((:RightShoulder __RightShoulder) 0.0)
    ((:RightElbow __RightElbow) 0.0)
    ((:RightWrist __RightWrist) 0.0)
    )
   (send-super :init)
   (setq _Time (float __Time))
   (setq _RightC7Shoulder (float __RightC7Shoulder))
   (setq _RightShoulder (float __RightShoulder))
   (setq _RightElbow (float __RightElbow))
   (setq _RightWrist (float __RightWrist))
   self)
  (:Time
   (&optional __Time)
   (if __Time (setq _Time __Time)) _Time)
  (:RightC7Shoulder
   (&optional __RightC7Shoulder)
   (if __RightC7Shoulder (setq _RightC7Shoulder __RightC7Shoulder)) _RightC7Shoulder)
  (:RightShoulder
   (&optional __RightShoulder)
   (if __RightShoulder (setq _RightShoulder __RightShoulder)) _RightShoulder)
  (:RightElbow
   (&optional __RightElbow)
   (if __RightElbow (setq _RightElbow __RightElbow)) _RightElbow)
  (:RightWrist
   (&optional __RightWrist)
   (if __RightWrist (setq _RightWrist __RightWrist)) _RightWrist)
  (:serialization-length
   ()
   (+
    ;; float32 _Time
    4
    ;; float32 _RightC7Shoulder
    4
    ;; float32 _RightShoulder
    4
    ;; float32 _RightElbow
    4
    ;; float32 _RightWrist
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _Time
       (sys::poke _Time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _RightC7Shoulder
       (sys::poke _RightC7Shoulder (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _RightShoulder
       (sys::poke _RightShoulder (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _RightElbow
       (sys::poke _RightElbow (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _RightWrist
       (sys::poke _RightWrist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _Time
     (setq _Time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _RightC7Shoulder
     (setq _RightC7Shoulder (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _RightShoulder
     (setq _RightShoulder (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _RightElbow
     (setq _RightElbow (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _RightWrist
     (setq _RightWrist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get csv_msgs::csvfile :md5sum-) "34dbd2050eabe306f510a061084b9163")
(setf (get csv_msgs::csvfile :datatype-) "csv_msgs/csvfile")
(setf (get csv_msgs::csvfile :definition-)
      "float32 Time
float32 RightC7Shoulder
float32 RightShoulder
float32 RightElbow
float32 RightWrist


")



(provide :csv_msgs/csvfile "34dbd2050eabe306f510a061084b9163")


