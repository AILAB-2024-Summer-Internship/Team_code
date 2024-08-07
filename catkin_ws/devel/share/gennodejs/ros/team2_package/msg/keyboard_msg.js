// Auto-generated. Do not edit!

// (in-package team2_package.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class keyboard_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.fb_speed = null;
      this.yaw = null;
      this.gear = null;
      this.brake = null;
    }
    else {
      if (initObj.hasOwnProperty('fb_speed')) {
        this.fb_speed = initObj.fb_speed
      }
      else {
        this.fb_speed = 0.0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('gear')) {
        this.gear = initObj.gear
      }
      else {
        this.gear = 0;
      }
      if (initObj.hasOwnProperty('brake')) {
        this.brake = initObj.brake
      }
      else {
        this.brake = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type keyboard_msg
    // Serialize message field [fb_speed]
    bufferOffset = _serializer.float32(obj.fb_speed, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float32(obj.yaw, buffer, bufferOffset);
    // Serialize message field [gear]
    bufferOffset = _serializer.int32(obj.gear, buffer, bufferOffset);
    // Serialize message field [brake]
    bufferOffset = _serializer.bool(obj.brake, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type keyboard_msg
    let len;
    let data = new keyboard_msg(null);
    // Deserialize message field [fb_speed]
    data.fb_speed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gear]
    data.gear = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [brake]
    data.brake = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'team2_package/keyboard_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bd4b2bffdbfc91ff88466d3ab2a24fe4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 fb_speed
    float32 yaw
    int32 gear
    bool brake
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new keyboard_msg(null);
    if (msg.fb_speed !== undefined) {
      resolved.fb_speed = msg.fb_speed;
    }
    else {
      resolved.fb_speed = 0.0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.gear !== undefined) {
      resolved.gear = msg.gear;
    }
    else {
      resolved.gear = 0
    }

    if (msg.brake !== undefined) {
      resolved.brake = msg.brake;
    }
    else {
      resolved.brake = false
    }

    return resolved;
    }
};

module.exports = keyboard_msg;
