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

class tracked_object {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.center_x = null;
      this.center_y = null;
      this.v_x = null;
      this.v_y = null;
      this.size_x = null;
      this.size_y = null;
    }
    else {
      if (initObj.hasOwnProperty('center_x')) {
        this.center_x = initObj.center_x
      }
      else {
        this.center_x = 0.0;
      }
      if (initObj.hasOwnProperty('center_y')) {
        this.center_y = initObj.center_y
      }
      else {
        this.center_y = 0.0;
      }
      if (initObj.hasOwnProperty('v_x')) {
        this.v_x = initObj.v_x
      }
      else {
        this.v_x = 0.0;
      }
      if (initObj.hasOwnProperty('v_y')) {
        this.v_y = initObj.v_y
      }
      else {
        this.v_y = 0.0;
      }
      if (initObj.hasOwnProperty('size_x')) {
        this.size_x = initObj.size_x
      }
      else {
        this.size_x = 0.0;
      }
      if (initObj.hasOwnProperty('size_y')) {
        this.size_y = initObj.size_y
      }
      else {
        this.size_y = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type tracked_object
    // Serialize message field [center_x]
    bufferOffset = _serializer.float64(obj.center_x, buffer, bufferOffset);
    // Serialize message field [center_y]
    bufferOffset = _serializer.float64(obj.center_y, buffer, bufferOffset);
    // Serialize message field [v_x]
    bufferOffset = _serializer.float64(obj.v_x, buffer, bufferOffset);
    // Serialize message field [v_y]
    bufferOffset = _serializer.float64(obj.v_y, buffer, bufferOffset);
    // Serialize message field [size_x]
    bufferOffset = _serializer.float64(obj.size_x, buffer, bufferOffset);
    // Serialize message field [size_y]
    bufferOffset = _serializer.float64(obj.size_y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type tracked_object
    let len;
    let data = new tracked_object(null);
    // Deserialize message field [center_x]
    data.center_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [center_y]
    data.center_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [v_x]
    data.v_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [v_y]
    data.v_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [size_x]
    data.size_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [size_y]
    data.size_y = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'team2_package/tracked_object';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ee6d8746eebf377eed126cb7859c31c3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 center_x
    float64 center_y
    float64 v_x
    float64 v_y
    float64 size_x
    float64 size_y
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new tracked_object(null);
    if (msg.center_x !== undefined) {
      resolved.center_x = msg.center_x;
    }
    else {
      resolved.center_x = 0.0
    }

    if (msg.center_y !== undefined) {
      resolved.center_y = msg.center_y;
    }
    else {
      resolved.center_y = 0.0
    }

    if (msg.v_x !== undefined) {
      resolved.v_x = msg.v_x;
    }
    else {
      resolved.v_x = 0.0
    }

    if (msg.v_y !== undefined) {
      resolved.v_y = msg.v_y;
    }
    else {
      resolved.v_y = 0.0
    }

    if (msg.size_x !== undefined) {
      resolved.size_x = msg.size_x;
    }
    else {
      resolved.size_x = 0.0
    }

    if (msg.size_y !== undefined) {
      resolved.size_y = msg.size_y;
    }
    else {
      resolved.size_y = 0.0
    }

    return resolved;
    }
};

module.exports = tracked_object;
