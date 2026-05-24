// Auto-generated. Do not edit!

// (in-package block_drawing_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ContactState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.normal_force = null;
      this.torque_x = null;
      this.torque_y = null;
      this.torque_z = null;
      this.in_contact = null;
    }
    else {
      if (initObj.hasOwnProperty('normal_force')) {
        this.normal_force = initObj.normal_force
      }
      else {
        this.normal_force = 0.0;
      }
      if (initObj.hasOwnProperty('torque_x')) {
        this.torque_x = initObj.torque_x
      }
      else {
        this.torque_x = 0.0;
      }
      if (initObj.hasOwnProperty('torque_y')) {
        this.torque_y = initObj.torque_y
      }
      else {
        this.torque_y = 0.0;
      }
      if (initObj.hasOwnProperty('torque_z')) {
        this.torque_z = initObj.torque_z
      }
      else {
        this.torque_z = 0.0;
      }
      if (initObj.hasOwnProperty('in_contact')) {
        this.in_contact = initObj.in_contact
      }
      else {
        this.in_contact = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ContactState
    // Serialize message field [normal_force]
    bufferOffset = _serializer.float64(obj.normal_force, buffer, bufferOffset);
    // Serialize message field [torque_x]
    bufferOffset = _serializer.float64(obj.torque_x, buffer, bufferOffset);
    // Serialize message field [torque_y]
    bufferOffset = _serializer.float64(obj.torque_y, buffer, bufferOffset);
    // Serialize message field [torque_z]
    bufferOffset = _serializer.float64(obj.torque_z, buffer, bufferOffset);
    // Serialize message field [in_contact]
    bufferOffset = _serializer.bool(obj.in_contact, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ContactState
    let len;
    let data = new ContactState(null);
    // Deserialize message field [normal_force]
    data.normal_force = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [torque_x]
    data.torque_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [torque_y]
    data.torque_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [torque_z]
    data.torque_z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [in_contact]
    data.in_contact = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 33;
  }

  static datatype() {
    // Returns string type for a message object
    return 'block_drawing_msgs/ContactState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd94143f44062f49290d19adbd3e1fb22';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ContactState.msg
    # 接触状态: 笔尖与物块表面的接触信息
    
    float64 normal_force        # 法向力估计值 [N]
    float64 torque_x            # 末端力矩 X [Nm]
    float64 torque_y            # 末端力矩 Y [Nm]
    float64 torque_z            # 末端力矩 Z [Nm]
    bool    in_contact          # 是否判定为接触中
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ContactState(null);
    if (msg.normal_force !== undefined) {
      resolved.normal_force = msg.normal_force;
    }
    else {
      resolved.normal_force = 0.0
    }

    if (msg.torque_x !== undefined) {
      resolved.torque_x = msg.torque_x;
    }
    else {
      resolved.torque_x = 0.0
    }

    if (msg.torque_y !== undefined) {
      resolved.torque_y = msg.torque_y;
    }
    else {
      resolved.torque_y = 0.0
    }

    if (msg.torque_z !== undefined) {
      resolved.torque_z = msg.torque_z;
    }
    else {
      resolved.torque_z = 0.0
    }

    if (msg.in_contact !== undefined) {
      resolved.in_contact = msg.in_contact;
    }
    else {
      resolved.in_contact = false
    }

    return resolved;
    }
};

module.exports = ContactState;
