// Auto-generated. Do not edit!

// (in-package csv_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class csvfile {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Time = null;
      this.RightC7Shoulder = null;
      this.RightShoulder = null;
      this.RightElbow = null;
      this.RightWrist = null;
    }
    else {
      if (initObj.hasOwnProperty('Time')) {
        this.Time = initObj.Time
      }
      else {
        this.Time = 0.0;
      }
      if (initObj.hasOwnProperty('RightC7Shoulder')) {
        this.RightC7Shoulder = initObj.RightC7Shoulder
      }
      else {
        this.RightC7Shoulder = 0.0;
      }
      if (initObj.hasOwnProperty('RightShoulder')) {
        this.RightShoulder = initObj.RightShoulder
      }
      else {
        this.RightShoulder = 0.0;
      }
      if (initObj.hasOwnProperty('RightElbow')) {
        this.RightElbow = initObj.RightElbow
      }
      else {
        this.RightElbow = 0.0;
      }
      if (initObj.hasOwnProperty('RightWrist')) {
        this.RightWrist = initObj.RightWrist
      }
      else {
        this.RightWrist = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type csvfile
    // Serialize message field [Time]
    bufferOffset = _serializer.float32(obj.Time, buffer, bufferOffset);
    // Serialize message field [RightC7Shoulder]
    bufferOffset = _serializer.float32(obj.RightC7Shoulder, buffer, bufferOffset);
    // Serialize message field [RightShoulder]
    bufferOffset = _serializer.float32(obj.RightShoulder, buffer, bufferOffset);
    // Serialize message field [RightElbow]
    bufferOffset = _serializer.float32(obj.RightElbow, buffer, bufferOffset);
    // Serialize message field [RightWrist]
    bufferOffset = _serializer.float32(obj.RightWrist, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type csvfile
    let len;
    let data = new csvfile(null);
    // Deserialize message field [Time]
    data.Time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [RightC7Shoulder]
    data.RightC7Shoulder = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [RightShoulder]
    data.RightShoulder = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [RightElbow]
    data.RightElbow = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [RightWrist]
    data.RightWrist = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'csv_msgs/csvfile';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '34dbd2050eabe306f510a061084b9163';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 Time
    float32 RightC7Shoulder
    float32 RightShoulder
    float32 RightElbow
    float32 RightWrist
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new csvfile(null);
    if (msg.Time !== undefined) {
      resolved.Time = msg.Time;
    }
    else {
      resolved.Time = 0.0
    }

    if (msg.RightC7Shoulder !== undefined) {
      resolved.RightC7Shoulder = msg.RightC7Shoulder;
    }
    else {
      resolved.RightC7Shoulder = 0.0
    }

    if (msg.RightShoulder !== undefined) {
      resolved.RightShoulder = msg.RightShoulder;
    }
    else {
      resolved.RightShoulder = 0.0
    }

    if (msg.RightElbow !== undefined) {
      resolved.RightElbow = msg.RightElbow;
    }
    else {
      resolved.RightElbow = 0.0
    }

    if (msg.RightWrist !== undefined) {
      resolved.RightWrist = msg.RightWrist;
    }
    else {
      resolved.RightWrist = 0.0
    }

    return resolved;
    }
};

module.exports = csvfile;
