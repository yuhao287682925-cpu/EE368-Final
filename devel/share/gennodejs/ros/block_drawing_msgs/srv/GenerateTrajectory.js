// Auto-generated. Do not edit!

// (in-package block_drawing_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let SurfaceTrajectory = require('../msg/SurfaceTrajectory.js');

//-----------------------------------------------------------

class GenerateTrajectoryRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.svg_file = null;
      this.target_width_mm = null;
      this.target_height_mm = null;
      this.faces = null;
      this.test_pattern = null;
    }
    else {
      if (initObj.hasOwnProperty('svg_file')) {
        this.svg_file = initObj.svg_file
      }
      else {
        this.svg_file = '';
      }
      if (initObj.hasOwnProperty('target_width_mm')) {
        this.target_width_mm = initObj.target_width_mm
      }
      else {
        this.target_width_mm = 0.0;
      }
      if (initObj.hasOwnProperty('target_height_mm')) {
        this.target_height_mm = initObj.target_height_mm
      }
      else {
        this.target_height_mm = 0.0;
      }
      if (initObj.hasOwnProperty('faces')) {
        this.faces = initObj.faces
      }
      else {
        this.faces = [];
      }
      if (initObj.hasOwnProperty('test_pattern')) {
        this.test_pattern = initObj.test_pattern
      }
      else {
        this.test_pattern = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GenerateTrajectoryRequest
    // Serialize message field [svg_file]
    bufferOffset = _serializer.string(obj.svg_file, buffer, bufferOffset);
    // Serialize message field [target_width_mm]
    bufferOffset = _serializer.float64(obj.target_width_mm, buffer, bufferOffset);
    // Serialize message field [target_height_mm]
    bufferOffset = _serializer.float64(obj.target_height_mm, buffer, bufferOffset);
    // Serialize message field [faces]
    bufferOffset = _arraySerializer.int32(obj.faces, buffer, bufferOffset, null);
    // Serialize message field [test_pattern]
    bufferOffset = _serializer.string(obj.test_pattern, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GenerateTrajectoryRequest
    let len;
    let data = new GenerateTrajectoryRequest(null);
    // Deserialize message field [svg_file]
    data.svg_file = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [target_width_mm]
    data.target_width_mm = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [target_height_mm]
    data.target_height_mm = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [faces]
    data.faces = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [test_pattern]
    data.test_pattern = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.svg_file);
    length += 4 * object.faces.length;
    length += _getByteLength(object.test_pattern);
    return length + 28;
  }

  static datatype() {
    // Returns string type for a service object
    return 'block_drawing_msgs/GenerateTrajectoryRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '101569b0caf27e51af004362761e0107';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # GenerateTrajectory.srv
    # 模块A服务: SVG/图案 → 轨迹序列
    
    string   svg_file                          # SVG文件路径 (空=使用内置测试图案)
    float64  target_width_mm                   # 目标图案宽度 [mm]
    float64  target_height_mm                  # 目标图案高度 [mm]
    int32[]  faces                             # 要画的面的列表 (0~4)
    string   test_pattern                      # 内置测试图案类型: "square" / "circle" / "star" (svg_file为空时生效)
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GenerateTrajectoryRequest(null);
    if (msg.svg_file !== undefined) {
      resolved.svg_file = msg.svg_file;
    }
    else {
      resolved.svg_file = ''
    }

    if (msg.target_width_mm !== undefined) {
      resolved.target_width_mm = msg.target_width_mm;
    }
    else {
      resolved.target_width_mm = 0.0
    }

    if (msg.target_height_mm !== undefined) {
      resolved.target_height_mm = msg.target_height_mm;
    }
    else {
      resolved.target_height_mm = 0.0
    }

    if (msg.faces !== undefined) {
      resolved.faces = msg.faces;
    }
    else {
      resolved.faces = []
    }

    if (msg.test_pattern !== undefined) {
      resolved.test_pattern = msg.test_pattern;
    }
    else {
      resolved.test_pattern = ''
    }

    return resolved;
    }
};

class GenerateTrajectoryResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.trajectories = null;
      this.success = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('trajectories')) {
        this.trajectories = initObj.trajectories
      }
      else {
        this.trajectories = [];
      }
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
    // Serializes a message object of type GenerateTrajectoryResponse
    // Serialize message field [trajectories]
    // Serialize the length for message field [trajectories]
    bufferOffset = _serializer.uint32(obj.trajectories.length, buffer, bufferOffset);
    obj.trajectories.forEach((val) => {
      bufferOffset = SurfaceTrajectory.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GenerateTrajectoryResponse
    let len;
    let data = new GenerateTrajectoryResponse(null);
    // Deserialize message field [trajectories]
    // Deserialize array length for message field [trajectories]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.trajectories = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.trajectories[i] = SurfaceTrajectory.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.trajectories.forEach((val) => {
      length += SurfaceTrajectory.getMessageSize(val);
    });
    length += _getByteLength(object.message);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'block_drawing_msgs/GenerateTrajectoryResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '42f26797fa281018517161f56d37f514';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    block_drawing_msgs/SurfaceTrajectory[] trajectories  # 每个面对应一条轨迹
    bool     success
    string   message
    
    
    ================================================================================
    MSG: block_drawing_msgs/SurfaceTrajectory
    # SurfaceTrajectory.msg
    # 单面轨迹: 包含该面所有 waypoints
    
    int32   face_id                    # 0=顶面, 1=前面, 2=右面, 3=后面, 4=左面
    geometry_msgs/Pose[] waypoints     # 末端期望位姿 (含法兰补偿)
    float64[] arc_lengths              # 累计弧长, 用于速度规划 [m]
    
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
    const resolved = new GenerateTrajectoryResponse(null);
    if (msg.trajectories !== undefined) {
      resolved.trajectories = new Array(msg.trajectories.length);
      for (let i = 0; i < resolved.trajectories.length; ++i) {
        resolved.trajectories[i] = SurfaceTrajectory.Resolve(msg.trajectories[i]);
      }
    }
    else {
      resolved.trajectories = []
    }

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
  Request: GenerateTrajectoryRequest,
  Response: GenerateTrajectoryResponse,
  md5sum() { return '8fbb008b13b5fc9766400a28fbedf619'; },
  datatype() { return 'block_drawing_msgs/GenerateTrajectory'; }
};
