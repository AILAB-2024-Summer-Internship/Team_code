// Auto-generated. Do not edit!

// (in-package team2_package.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class globalwaypoints {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.road_options = null;
      this.x = null;
      this.y = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('road_options')) {
        this.road_options = initObj.road_options
      }
      else {
        this.road_options = [];
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = [];
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type globalwaypoints
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [road_options]
    bufferOffset = _arraySerializer.int8(obj.road_options, buffer, bufferOffset, null);
    // Serialize message field [x]
    bufferOffset = _arraySerializer.float64(obj.x, buffer, bufferOffset, null);
    // Serialize message field [y]
    bufferOffset = _arraySerializer.float64(obj.y, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type globalwaypoints
    let len;
    let data = new globalwaypoints(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [road_options]
    data.road_options = _arrayDeserializer.int8(buffer, bufferOffset, null)
    // Deserialize message field [x]
    data.x = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [y]
    data.y = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.road_options.length;
    length += 8 * object.x.length;
    length += 8 * object.y.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'team2_package/globalwaypoints';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f169d64c8dec16596456b30665e5fbde';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    int8[] road_options
    float64[] x
    float64[] y
    
    int8 VOID=-1
    int8 LEFT=1
    int8 RIGHT=2
    int8 STRAIGHT=3
    int8 LANEFOLLOW=4
    int8 CHANGELANELEFT=5
    int8 CHANGELANERIGHT=6
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new globalwaypoints(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.road_options !== undefined) {
      resolved.road_options = msg.road_options;
    }
    else {
      resolved.road_options = []
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = []
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = []
    }

    return resolved;
    }
};

// Constants for message
globalwaypoints.Constants = {
  VOID: -1,
  LEFT: 1,
  RIGHT: 2,
  STRAIGHT: 3,
  LANEFOLLOW: 4,
  CHANGELANELEFT: 5,
  CHANGELANERIGHT: 6,
}

module.exports = globalwaypoints;
