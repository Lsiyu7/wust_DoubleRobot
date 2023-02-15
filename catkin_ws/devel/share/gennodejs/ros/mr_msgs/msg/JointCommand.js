// Auto-generated. Do not edit!

// (in-package mr_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class JointCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.command = null;
      this.names = null;
      this.cmdPos = null;
      this.cmdspd = null;
      this.cmdCurr = null;
    }
    else {
      if (initObj.hasOwnProperty('command')) {
        this.command = initObj.command
      }
      else {
        this.command = 0;
      }
      if (initObj.hasOwnProperty('names')) {
        this.names = initObj.names
      }
      else {
        this.names = [];
      }
      if (initObj.hasOwnProperty('cmdPos')) {
        this.cmdPos = initObj.cmdPos
      }
      else {
        this.cmdPos = [];
      }
      if (initObj.hasOwnProperty('cmdspd')) {
        this.cmdspd = initObj.cmdspd
      }
      else {
        this.cmdspd = [];
      }
      if (initObj.hasOwnProperty('cmdCurr')) {
        this.cmdCurr = initObj.cmdCurr
      }
      else {
        this.cmdCurr = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointCommand
    // Serialize message field [command]
    bufferOffset = _serializer.int32(obj.command, buffer, bufferOffset);
    // Serialize message field [names]
    bufferOffset = _arraySerializer.string(obj.names, buffer, bufferOffset, null);
    // Serialize message field [cmdPos]
    bufferOffset = _arraySerializer.float64(obj.cmdPos, buffer, bufferOffset, null);
    // Serialize message field [cmdspd]
    bufferOffset = _arraySerializer.float64(obj.cmdspd, buffer, bufferOffset, null);
    // Serialize message field [cmdCurr]
    bufferOffset = _arraySerializer.float64(obj.cmdCurr, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointCommand
    let len;
    let data = new JointCommand(null);
    // Deserialize message field [command]
    data.command = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [names]
    data.names = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [cmdPos]
    data.cmdPos = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [cmdspd]
    data.cmdspd = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [cmdCurr]
    data.cmdCurr = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.names.forEach((val) => {
      length += 4 + val.length;
    });
    length += 8 * object.cmdPos.length;
    length += 8 * object.cmdspd.length;
    length += 8 * object.cmdCurr.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mr_msgs/JointCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '316fe6f888e87acbb2cf55d67ad8f983';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 command
    string[]  names
    float64[] cmdPos
    float64[] cmdspd
    float64[] cmdCurr
    
    int32 CURRENT_CMD=1
    int32 SPEED_CMD=2
    int32 POSITION_CMD=3
    int32 TRAJECTORY_CMD=3
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JointCommand(null);
    if (msg.command !== undefined) {
      resolved.command = msg.command;
    }
    else {
      resolved.command = 0
    }

    if (msg.names !== undefined) {
      resolved.names = msg.names;
    }
    else {
      resolved.names = []
    }

    if (msg.cmdPos !== undefined) {
      resolved.cmdPos = msg.cmdPos;
    }
    else {
      resolved.cmdPos = []
    }

    if (msg.cmdspd !== undefined) {
      resolved.cmdspd = msg.cmdspd;
    }
    else {
      resolved.cmdspd = []
    }

    if (msg.cmdCurr !== undefined) {
      resolved.cmdCurr = msg.cmdCurr;
    }
    else {
      resolved.cmdCurr = []
    }

    return resolved;
    }
};

// Constants for message
JointCommand.Constants = {
  CURRENT_CMD: 1,
  SPEED_CMD: 2,
  POSITION_CMD: 3,
  TRAJECTORY_CMD: 3,
}

module.exports = JointCommand;
