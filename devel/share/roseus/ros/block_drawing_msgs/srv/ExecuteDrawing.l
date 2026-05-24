;; Auto-generated. Do not edit!


(when (boundp 'block_drawing_msgs::ExecuteDrawing)
  (if (not (find-package "BLOCK_DRAWING_MSGS"))
    (make-package "BLOCK_DRAWING_MSGS"))
  (shadow 'ExecuteDrawing (find-package "BLOCK_DRAWING_MSGS")))
(unless (find-package "BLOCK_DRAWING_MSGS::EXECUTEDRAWING")
  (make-package "BLOCK_DRAWING_MSGS::EXECUTEDRAWING"))
(unless (find-package "BLOCK_DRAWING_MSGS::EXECUTEDRAWINGREQUEST")
  (make-package "BLOCK_DRAWING_MSGS::EXECUTEDRAWINGREQUEST"))
(unless (find-package "BLOCK_DRAWING_MSGS::EXECUTEDRAWINGRESPONSE")
  (make-package "BLOCK_DRAWING_MSGS::EXECUTEDRAWINGRESPONSE"))

(in-package "ROS")





(defclass block_drawing_msgs::ExecuteDrawingRequest
  :super ros::object
  :slots (_start ))

(defmethod block_drawing_msgs::ExecuteDrawingRequest
  (:init
   (&key
    ((:start __start) nil)
    )
   (send-super :init)
   (setq _start __start)
   self)
  (:start
   (&optional (__start :null))
   (if (not (eq __start :null)) (setq _start __start)) _start)
  (:serialization-length
   ()
   (+
    ;; bool _start
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _start
       (if _start (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _start
     (setq _start (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass block_drawing_msgs::ExecuteDrawingResponse
  :super ros::object
  :slots (_success _message ))

(defmethod block_drawing_msgs::ExecuteDrawingResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass block_drawing_msgs::ExecuteDrawing
  :super ros::object
  :slots ())

(setf (get block_drawing_msgs::ExecuteDrawing :md5sum-) "570b7d04f9d3b17893f17c4fdcf5ca06")
(setf (get block_drawing_msgs::ExecuteDrawing :datatype-) "block_drawing_msgs/ExecuteDrawing")
(setf (get block_drawing_msgs::ExecuteDrawing :request) block_drawing_msgs::ExecuteDrawingRequest)
(setf (get block_drawing_msgs::ExecuteDrawing :response) block_drawing_msgs::ExecuteDrawingResponse)

(defmethod block_drawing_msgs::ExecuteDrawingRequest
  (:response () (instance block_drawing_msgs::ExecuteDrawingResponse :init)))

(setf (get block_drawing_msgs::ExecuteDrawingRequest :md5sum-) "570b7d04f9d3b17893f17c4fdcf5ca06")
(setf (get block_drawing_msgs::ExecuteDrawingRequest :datatype-) "block_drawing_msgs/ExecuteDrawingRequest")
(setf (get block_drawing_msgs::ExecuteDrawingRequest :definition-)
      "# ExecuteDrawing.srv
# 模块B服务: 启动绘画执行 (使用 Action 模式时为简单触发)

bool start    # true=启动, false=取消
---
bool success
string message

")

(setf (get block_drawing_msgs::ExecuteDrawingResponse :md5sum-) "570b7d04f9d3b17893f17c4fdcf5ca06")
(setf (get block_drawing_msgs::ExecuteDrawingResponse :datatype-) "block_drawing_msgs/ExecuteDrawingResponse")
(setf (get block_drawing_msgs::ExecuteDrawingResponse :definition-)
      "# ExecuteDrawing.srv
# 模块B服务: 启动绘画执行 (使用 Action 模式时为简单触发)

bool start    # true=启动, false=取消
---
bool success
string message

")



(provide :block_drawing_msgs/ExecuteDrawing "570b7d04f9d3b17893f17c4fdcf5ca06")


