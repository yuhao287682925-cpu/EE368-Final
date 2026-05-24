;; Auto-generated. Do not edit!


(when (boundp 'block_drawing_msgs::ContactState)
  (if (not (find-package "BLOCK_DRAWING_MSGS"))
    (make-package "BLOCK_DRAWING_MSGS"))
  (shadow 'ContactState (find-package "BLOCK_DRAWING_MSGS")))
(unless (find-package "BLOCK_DRAWING_MSGS::CONTACTSTATE")
  (make-package "BLOCK_DRAWING_MSGS::CONTACTSTATE"))

(in-package "ROS")
;;//! \htmlinclude ContactState.msg.html


(defclass block_drawing_msgs::ContactState
  :super ros::object
  :slots (_normal_force _torque_x _torque_y _torque_z _in_contact ))

(defmethod block_drawing_msgs::ContactState
  (:init
   (&key
    ((:normal_force __normal_force) 0.0)
    ((:torque_x __torque_x) 0.0)
    ((:torque_y __torque_y) 0.0)
    ((:torque_z __torque_z) 0.0)
    ((:in_contact __in_contact) nil)
    )
   (send-super :init)
   (setq _normal_force (float __normal_force))
   (setq _torque_x (float __torque_x))
   (setq _torque_y (float __torque_y))
   (setq _torque_z (float __torque_z))
   (setq _in_contact __in_contact)
   self)
  (:normal_force
   (&optional __normal_force)
   (if __normal_force (setq _normal_force __normal_force)) _normal_force)
  (:torque_x
   (&optional __torque_x)
   (if __torque_x (setq _torque_x __torque_x)) _torque_x)
  (:torque_y
   (&optional __torque_y)
   (if __torque_y (setq _torque_y __torque_y)) _torque_y)
  (:torque_z
   (&optional __torque_z)
   (if __torque_z (setq _torque_z __torque_z)) _torque_z)
  (:in_contact
   (&optional (__in_contact :null))
   (if (not (eq __in_contact :null)) (setq _in_contact __in_contact)) _in_contact)
  (:serialization-length
   ()
   (+
    ;; float64 _normal_force
    8
    ;; float64 _torque_x
    8
    ;; float64 _torque_y
    8
    ;; float64 _torque_z
    8
    ;; bool _in_contact
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _normal_force
       (sys::poke _normal_force (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _torque_x
       (sys::poke _torque_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _torque_y
       (sys::poke _torque_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _torque_z
       (sys::poke _torque_z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _in_contact
       (if _in_contact (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _normal_force
     (setq _normal_force (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _torque_x
     (setq _torque_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _torque_y
     (setq _torque_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _torque_z
     (setq _torque_z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _in_contact
     (setq _in_contact (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get block_drawing_msgs::ContactState :md5sum-) "d94143f44062f49290d19adbd3e1fb22")
(setf (get block_drawing_msgs::ContactState :datatype-) "block_drawing_msgs/ContactState")
(setf (get block_drawing_msgs::ContactState :definition-)
      "# ContactState.msg
# 接触状态: 笔尖与物块表面的接触信息

float64 normal_force        # 法向力估计值 [N]
float64 torque_x            # 末端力矩 X [Nm]
float64 torque_y            # 末端力矩 Y [Nm]
float64 torque_z            # 末端力矩 Z [Nm]
bool    in_contact          # 是否判定为接触中

")



(provide :block_drawing_msgs/ContactState "d94143f44062f49290d19adbd3e1fb22")


