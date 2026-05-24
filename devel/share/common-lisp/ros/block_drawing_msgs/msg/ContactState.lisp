; Auto-generated. Do not edit!


(cl:in-package block_drawing_msgs-msg)


;//! \htmlinclude ContactState.msg.html

(cl:defclass <ContactState> (roslisp-msg-protocol:ros-message)
  ((normal_force
    :reader normal_force
    :initarg :normal_force
    :type cl:float
    :initform 0.0)
   (torque_x
    :reader torque_x
    :initarg :torque_x
    :type cl:float
    :initform 0.0)
   (torque_y
    :reader torque_y
    :initarg :torque_y
    :type cl:float
    :initform 0.0)
   (torque_z
    :reader torque_z
    :initarg :torque_z
    :type cl:float
    :initform 0.0)
   (in_contact
    :reader in_contact
    :initarg :in_contact
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ContactState (<ContactState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ContactState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ContactState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name block_drawing_msgs-msg:<ContactState> is deprecated: use block_drawing_msgs-msg:ContactState instead.")))

(cl:ensure-generic-function 'normal_force-val :lambda-list '(m))
(cl:defmethod normal_force-val ((m <ContactState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-msg:normal_force-val is deprecated.  Use block_drawing_msgs-msg:normal_force instead.")
  (normal_force m))

(cl:ensure-generic-function 'torque_x-val :lambda-list '(m))
(cl:defmethod torque_x-val ((m <ContactState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-msg:torque_x-val is deprecated.  Use block_drawing_msgs-msg:torque_x instead.")
  (torque_x m))

(cl:ensure-generic-function 'torque_y-val :lambda-list '(m))
(cl:defmethod torque_y-val ((m <ContactState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-msg:torque_y-val is deprecated.  Use block_drawing_msgs-msg:torque_y instead.")
  (torque_y m))

(cl:ensure-generic-function 'torque_z-val :lambda-list '(m))
(cl:defmethod torque_z-val ((m <ContactState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-msg:torque_z-val is deprecated.  Use block_drawing_msgs-msg:torque_z instead.")
  (torque_z m))

(cl:ensure-generic-function 'in_contact-val :lambda-list '(m))
(cl:defmethod in_contact-val ((m <ContactState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader block_drawing_msgs-msg:in_contact-val is deprecated.  Use block_drawing_msgs-msg:in_contact instead.")
  (in_contact m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ContactState>) ostream)
  "Serializes a message object of type '<ContactState>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'normal_force))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'torque_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'torque_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'torque_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'in_contact) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ContactState>) istream)
  "Deserializes a message object of type '<ContactState>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'normal_force) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'torque_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'torque_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'torque_z) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'in_contact) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ContactState>)))
  "Returns string type for a message object of type '<ContactState>"
  "block_drawing_msgs/ContactState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ContactState)))
  "Returns string type for a message object of type 'ContactState"
  "block_drawing_msgs/ContactState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ContactState>)))
  "Returns md5sum for a message object of type '<ContactState>"
  "d94143f44062f49290d19adbd3e1fb22")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ContactState)))
  "Returns md5sum for a message object of type 'ContactState"
  "d94143f44062f49290d19adbd3e1fb22")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ContactState>)))
  "Returns full string definition for message of type '<ContactState>"
  (cl:format cl:nil "# ContactState.msg~%# 接触状态: 笔尖与物块表面的接触信息~%~%float64 normal_force        # 法向力估计值 [N]~%float64 torque_x            # 末端力矩 X [Nm]~%float64 torque_y            # 末端力矩 Y [Nm]~%float64 torque_z            # 末端力矩 Z [Nm]~%bool    in_contact          # 是否判定为接触中~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ContactState)))
  "Returns full string definition for message of type 'ContactState"
  (cl:format cl:nil "# ContactState.msg~%# 接触状态: 笔尖与物块表面的接触信息~%~%float64 normal_force        # 法向力估计值 [N]~%float64 torque_x            # 末端力矩 X [Nm]~%float64 torque_y            # 末端力矩 Y [Nm]~%float64 torque_z            # 末端力矩 Z [Nm]~%bool    in_contact          # 是否判定为接触中~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ContactState>))
  (cl:+ 0
     8
     8
     8
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ContactState>))
  "Converts a ROS message object to a list"
  (cl:list 'ContactState
    (cl:cons ':normal_force (normal_force msg))
    (cl:cons ':torque_x (torque_x msg))
    (cl:cons ':torque_y (torque_y msg))
    (cl:cons ':torque_z (torque_z msg))
    (cl:cons ':in_contact (in_contact msg))
))
