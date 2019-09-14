// Auto-generated. Do not edit!

// (in-package camultiplex.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class TTest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.TString = null;
      this.value = null;
    }
    else {
      if (initObj.hasOwnProperty('TString')) {
        this.TString = initObj.TString
      }
      else {
        this.TString = '';
      }
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TTest
    // Serialize message field [TString]
    bufferOffset = _serializer.string(obj.TString, buffer, bufferOffset);
    // Serialize message field [value]
    bufferOffset = _serializer.uint32(obj.value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TTest
    let len;
    let data = new TTest(null);
    // Deserialize message field [TString]
    data.TString = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [value]
    data.value = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.TString.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'camultiplex/TTest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6dd10cf3b8877d89431c09fd24eede0b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string TString
    uint32 value
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TTest(null);
    if (msg.TString !== undefined) {
      resolved.TString = msg.TString;
    }
    else {
      resolved.TString = ''
    }

    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = 0
    }

    return resolved;
    }
}

module.exports = TTest;
