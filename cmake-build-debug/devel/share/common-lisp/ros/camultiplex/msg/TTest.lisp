; Auto-generated. Do not edit!


(cl:in-package camultiplex-msg)


;//! \htmlinclude TTest.msg.html

(cl:defclass <TTest> (roslisp-msg-protocol:ros-message)
  ((TString
    :reader TString
    :initarg :TString
    :type cl:string
    :initform "")
   (value
    :reader value
    :initarg :value
    :type cl:integer
    :initform 0))
)

(cl:defclass TTest (<TTest>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TTest>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TTest)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camultiplex-msg:<TTest> is deprecated: use camultiplex-msg:TTest instead.")))

(cl:ensure-generic-function 'TString-val :lambda-list '(m))
(cl:defmethod TString-val ((m <TTest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camultiplex-msg:TString-val is deprecated.  Use camultiplex-msg:TString instead.")
  (TString m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <TTest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camultiplex-msg:value-val is deprecated.  Use camultiplex-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TTest>) ostream)
  "Serializes a message object of type '<TTest>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'TString))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'TString))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'value)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'value)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'value)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TTest>) istream)
  "Deserializes a message object of type '<TTest>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'TString) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'TString) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'value)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TTest>)))
  "Returns string type for a message object of type '<TTest>"
  "camultiplex/TTest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TTest)))
  "Returns string type for a message object of type 'TTest"
  "camultiplex/TTest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TTest>)))
  "Returns md5sum for a message object of type '<TTest>"
  "6dd10cf3b8877d89431c09fd24eede0b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TTest)))
  "Returns md5sum for a message object of type 'TTest"
  "6dd10cf3b8877d89431c09fd24eede0b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TTest>)))
  "Returns full string definition for message of type '<TTest>"
  (cl:format cl:nil "string TString~%uint32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TTest)))
  "Returns full string definition for message of type 'TTest"
  (cl:format cl:nil "string TString~%uint32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TTest>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'TString))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TTest>))
  "Converts a ROS message object to a list"
  (cl:list 'TTest
    (cl:cons ':TString (TString msg))
    (cl:cons ':value (value msg))
))
