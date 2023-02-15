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

class GripperCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.positionL = null;
      this.positionR = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('positionL')) {
        this.positionL = initObj.positionL
      }
      else {
        this.positionL = 0.0;
      }
      if (initObj.hasOwnProperty('positionR')) {
        this.positionR = initObj.positionR
      }
      else {
        this.positionR = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GripperCommand
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [positionL]
    bufferOffset = _serializer.float64(obj.positionL, buffer, bufferOffset);
    // Serialize message field [positionR]
    bufferOffset = _serializer.float64(obj.positionR, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GripperCommand
    let len;
    let data = new GripperCommand(null);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [positionL]
    data.positionL = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [positionR]
    data.positionR = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.name.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mr_msgs/GripperCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bacced25655daea064ca2c4b7a402fd6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string  name
    float64 positionL
    float64 positionR
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GripperCommand(null);
    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.positionL !== undefined) {
      resolved.positionL = msg.positionL;
    }
    else {
      resolved.positionL = 0.0
    }

    if (msg.positionR !== undefined) {
      resolved.positionR = msg.positionR;
    }
    else {
      resolved.positionR = 0.0
    }

    return resolved;
    }
};

module.exports = GripperCommand;
