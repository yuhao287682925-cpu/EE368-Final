// Auto-generated. Do not edit!

// (in-package block_drawing_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetBlockPoseRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.block_pose = null;
      this.L = null;
      this.W = null;
      this.H = null;
      this.face_offset_u = null;
      this.face_offset_v = null;
      this.center_face = null;
      this.center_u_mm = null;
      this.center_v_mm = null;
    }
    else {
      if (initObj.hasOwnProperty('block_pose')) {
        this.block_pose = initObj.block_pose
      }
      else {
        this.block_pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('L')) {
        this.L = initObj.L
      }
      else {
        this.L = 0.0;
      }
      if (initObj.hasOwnProperty('W')) {
        this.W = initObj.W
      }
      else {
        this.W = 0.0;
      }
      if (initObj.hasOwnProperty('H')) {
        this.H = initObj.H
      }
      else {
        this.H = 0.0;
      }
      if (initObj.hasOwnProperty('face_offset_u')) {
        this.face_offset_u = initObj.face_offset_u
      }
      else {
        this.face_offset_u = new Array(5).fill(0);
      }
      if (initObj.hasOwnProperty('face_offset_v')) {
        this.face_offset_v = initObj.face_offset_v
      }
      else {
        this.face_offset_v = new Array(5).fill(0);
      }
      if (initObj.hasOwnProperty('center_face')) {
        this.center_face = initObj.center_face
      }
      else {
        this.center_face = 0;
      }
      if (initObj.hasOwnProperty('center_u_mm')) {
        this.center_u_mm = initObj.center_u_mm
      }
      else {
        this.center_u_mm = 0.0;
      }
      if (initObj.hasOwnProperty('center_v_mm')) {
        this.center_v_mm = initObj.center_v_mm
      }
      else {
        this.center_v_mm = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetBlockPoseRequest
    // Serialize message field [block_pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.block_pose, buffer, bufferOffset);
    // Serialize message field [L]
    bufferOffset = _serializer.float64(obj.L, buffer, bufferOffset);
    // Serialize message field [W]
    bufferOffset = _serializer.float64(obj.W, buffer, bufferOffset);
    // Serialize message field [H]
    bufferOffset = _serializer.float64(obj.H, buffer, bufferOffset);
    // Check that the constant length array field [face_offset_u] has the right length
    if (obj.face_offset_u.length !== 5) {
      throw new Error('Unable to serialize array field face_offset_u - length must be 5')
    }
    // Serialize message field [face_offset_u]
    bufferOffset = _arraySerializer.float64(obj.face_offset_u, buffer, bufferOffset, 5);
    // Check that the constant length array field [face_offset_v] has the right length
    if (obj.face_offset_v.length !== 5) {
      throw new Error('Unable to serialize array field face_offset_v - length must be 5')
    }
    // Serialize message field [face_offset_v]
    bufferOffset = _arraySerializer.float64(obj.face_offset_v, buffer, bufferOffset, 5);
    // Serialize message field [center_face]
    bufferOffset = _serializer.int32(obj.center_face, buffer, bufferOffset);
    // Serialize message field [center_u_mm]
    bufferOffset = _serializer.float64(obj.center_u_mm, buffer, bufferOffset);
    // Serialize message field [center_v_mm]
    bufferOffset = _serializer.float64(obj.center_v_mm, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetBlockPoseRequest
    let len;
    let data = new SetBlockPoseRequest(null);
    // Deserialize message field [block_pose]
    data.block_pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [L]
    data.L = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [W]
    data.W = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [H]
    data.H = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [face_offset_u]
    data.face_offset_u = _arrayDeserializer.float64(buffer, bufferOffset, 5)
    // Deserialize message field [face_offset_v]
    data.face_offset_v = _arrayDeserializer.float64(buffer, bufferOffset, 5)
    // Deserialize message field [center_face]
    data.center_face = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [center_u_mm]
    data.center_u_mm = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [center_v_mm]
    data.center_v_mm = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 180;
  }

  static datatype() {
    // Returns string type for a service object
    return 'block_drawing_msgs/SetBlockPoseRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0fd2dcc60a20d920ebc1d9f65ead0ab0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # SetBlockPose.srv
    # 设置物块在机械臂基坐标系下的位姿、尺寸、以及各面的图案投影偏移
    
    geometry_msgs/Pose   block_pose    # T_block_base (物块底面中心→基座)
    float64 L                          # 物块长 [m]
    float64 W                          # 物块宽 [m]
    float64 H                          # 物块高 [m]
    
    # 各面图案投影偏移 [mm], 相对于面中心
    # face_offset_u[i]: 沿面内 u 轴偏移, face_offset_v[i]: 沿面内 v 轴偏移
    # i=0顶面, 1前面, 2右面, 3后面, 4左面
    # 默认全0 = 图案中心对齐面中心
    float64[5] face_offset_u          # 每面 u 向偏移 [mm] (独立面模式)
    float64[5] face_offset_v          # 每面 v 向偏移 [mm] (独立面模式)
    
    # 跨面连续模式: 图案中心在块表面上的锚点
    # center_face = -1 表示使用独立面模式 (每个面独立画)
    # center_face = 0~4 时使用连续模式, 图案从此面此点出发, 超界自动跨邻面
    int32    center_face              # -1=独立面模式, 0~4=连续模式起始面
    float64  center_u_mm              # 图案中心 u 坐标 [mm]
    float64  center_v_mm              # 图案中心 v 坐标 [mm]
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetBlockPoseRequest(null);
    if (msg.block_pose !== undefined) {
      resolved.block_pose = geometry_msgs.msg.Pose.Resolve(msg.block_pose)
    }
    else {
      resolved.block_pose = new geometry_msgs.msg.Pose()
    }

    if (msg.L !== undefined) {
      resolved.L = msg.L;
    }
    else {
      resolved.L = 0.0
    }

    if (msg.W !== undefined) {
      resolved.W = msg.W;
    }
    else {
      resolved.W = 0.0
    }

    if (msg.H !== undefined) {
      resolved.H = msg.H;
    }
    else {
      resolved.H = 0.0
    }

    if (msg.face_offset_u !== undefined) {
      resolved.face_offset_u = msg.face_offset_u;
    }
    else {
      resolved.face_offset_u = new Array(5).fill(0)
    }

    if (msg.face_offset_v !== undefined) {
      resolved.face_offset_v = msg.face_offset_v;
    }
    else {
      resolved.face_offset_v = new Array(5).fill(0)
    }

    if (msg.center_face !== undefined) {
      resolved.center_face = msg.center_face;
    }
    else {
      resolved.center_face = 0
    }

    if (msg.center_u_mm !== undefined) {
      resolved.center_u_mm = msg.center_u_mm;
    }
    else {
      resolved.center_u_mm = 0.0
    }

    if (msg.center_v_mm !== undefined) {
      resolved.center_v_mm = msg.center_v_mm;
    }
    else {
      resolved.center_v_mm = 0.0
    }

    return resolved;
    }
};

class SetBlockPoseResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetBlockPoseResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetBlockPoseResponse
    let len;
    let data = new SetBlockPoseResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'block_drawing_msgs/SetBlockPoseResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '937c9679a518e3a18d831e57125ea522';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    string message
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetBlockPoseResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: SetBlockPoseRequest,
  Response: SetBlockPoseResponse,
  md5sum() { return '39ae2210f4fabcad1e458e4361dd4a03'; },
  datatype() { return 'block_drawing_msgs/SetBlockPose'; }
};
