// Auto-generated. Do not edit!

// (in-package block_drawing_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class SurfaceTrajectory {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.face_id = null;
      this.waypoints = null;
      this.arc_lengths = null;
    }
    else {
      if (initObj.hasOwnProperty('face_id')) {
        this.face_id = initObj.face_id
      }
      else {
        this.face_id = 0;
      }
      if (initObj.hasOwnProperty('waypoints')) {
        this.waypoints = initObj.waypoints
      }
      else {
        this.waypoints = [];
      }
      if (initObj.hasOwnProperty('arc_lengths')) {
        this.arc_lengths = initObj.arc_lengths
      }
      else {
        this.arc_lengths = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SurfaceTrajectory
    // Serialize message field [face_id]
    bufferOffset = _serializer.int32(obj.face_id, buffer, bufferOffset);
    // Serialize message field [waypoints]
    // Serialize the length for message field [waypoints]
    bufferOffset = _serializer.uint32(obj.waypoints.length, buffer, bufferOffset);
    obj.waypoints.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Pose.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [arc_lengths]
    bufferOffset = _arraySerializer.float64(obj.arc_lengths, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SurfaceTrajectory
    let len;
    let data = new SurfaceTrajectory(null);
    // Deserialize message field [face_id]
    data.face_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [waypoints]
    // Deserialize array length for message field [waypoints]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.waypoints = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.waypoints[i] = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [arc_lengths]
    data.arc_lengths = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 56 * object.waypoints.length;
    length += 8 * object.arc_lengths.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'block_drawing_msgs/SurfaceTrajectory';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5b572666885f29b8eb6f14e36c9ed7c7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new SurfaceTrajectory(null);
    if (msg.face_id !== undefined) {
      resolved.face_id = msg.face_id;
    }
    else {
      resolved.face_id = 0
    }

    if (msg.waypoints !== undefined) {
      resolved.waypoints = new Array(msg.waypoints.length);
      for (let i = 0; i < resolved.waypoints.length; ++i) {
        resolved.waypoints[i] = geometry_msgs.msg.Pose.Resolve(msg.waypoints[i]);
      }
    }
    else {
      resolved.waypoints = []
    }

    if (msg.arc_lengths !== undefined) {
      resolved.arc_lengths = msg.arc_lengths;
    }
    else {
      resolved.arc_lengths = []
    }

    return resolved;
    }
};

module.exports = SurfaceTrajectory;
