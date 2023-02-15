// Auto-generated. Do not edit!

// (in-package mr_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class GripperState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.names = null;
      this.positionsL = null;
      this.positionsR = null;
      this.torqueL = null;
      this.torqueR = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('names')) {
        this.names = initObj.names
      }
      else {
        this.names = [];
      }
      if (initObj.hasOwnProperty('positionsL')) {
        this.positionsL = initObj.positionsL
      }
      else {
        this.positionsL = [];
      }
      if (initObj.hasOwnProperty('positionsR')) {
        this.positionsR = initObj.positionsR
      }
      else {
        this.positionsR = [];
      }
      if (initObj.hasOwnProperty('torqueL')) {
        this.torqueL = initObj.torqueL
      }
      else {
        this.torqueL = [];
      }
      if (initObj.hasOwnProperty('torqueR')) {
        this.torqueR = initObj.torqueR
      }
      else {
        this.torqueR = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GripperState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [names]
    bufferOffset = _arraySerializer.string(obj.names, buffer, bufferOffset, null);
    // Serialize message field [positionsL]
    bufferOffset = _arraySerializer.float64(obj.positionsL, buffer, bufferOffset, null);
    // Serialize message field [positionsR]
    bufferOffset = _arraySerializer.float64(obj.positionsR, buffer, bufferOffset, null);
    // Serialize message field [torqueL]
    bufferOffset = _arraySerializer.float64(obj.torqueL, buffer, bufferOffset, null);
    // Serialize message field [torqueR]
    bufferOffset = _arraySerializer.float64(obj.torqueR, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GripperState
    let len;
    let data = new GripperState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [names]
    data.names = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [positionsL]
    data.positionsL = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [positionsR]
    data.positionsR = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [torqueL]
    data.torqueL = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [torqueR]
    data.torqueR = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.names.forEach((val) => {
      length += 4 + val.length;
    });
    length += 8 * object.positionsL.length;
    length += 8 * object.positionsR.length;
    length += 8 * object.torqueL.length;
    length += 8 * object.torqueR.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mr_msgs/GripperState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3e09f13f34e6efdfa5ea653e97d62200';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    string[]  names
    float64[] positionsL
    float64[] positionsR
    float64[] torqueL
    float64[] torqueR
    
    int32 OPEN=1
    int32 CLOSE=0
    
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
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GripperState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.names !== undefined) {
      resolved.names = msg.names;
    }
    else {
      resolved.names = []
    }

    if (msg.positionsL !== undefined) {
      resolved.positionsL = msg.positionsL;
    }
    else {
      resolved.positionsL = []
    }

    if (msg.positionsR !== undefined) {
      resolved.positionsR = msg.positionsR;
    }
    else {
      resolved.positionsR = []
    }

    if (msg.torqueL !== undefined) {
      resolved.torqueL = msg.torqueL;
    }
    else {
      resolved.torqueL = []
    }

    if (msg.torqueR !== undefined) {
      resolved.torqueR = msg.torqueR;
    }
    else {
      resolved.torqueR = []
    }

    return resolved;
    }
};

// Constants for message
GripperState.Constants = {
  OPEN: 1,
  CLOSE: 0,
}

module.exports = GripperState;
