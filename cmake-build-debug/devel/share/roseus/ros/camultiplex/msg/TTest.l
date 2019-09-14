;; Auto-generated. Do not edit!


(when (boundp 'camultiplex::TTest)
  (if (not (find-package "CAMULTIPLEX"))
    (make-package "CAMULTIPLEX"))
  (shadow 'TTest (find-package "CAMULTIPLEX")))
(unless (find-package "CAMULTIPLEX::TTEST")
  (make-package "CAMULTIPLEX::TTEST"))

(in-package "ROS")
;;//! \htmlinclude TTest.msg.html


(defclass camultiplex::TTest
  :super ros::object
  :slots (_TString _value ))

(defmethod camultiplex::TTest
  (:init
   (&key
    ((:TString __TString) "")
    ((:value __value) 0)
    )
   (send-super :init)
   (setq _TString (string __TString))
   (setq _value (round __value))
   self)
  (:TString
   (&optional __TString)
   (if __TString (setq _TString __TString)) _TString)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; string _TString
    4 (length _TString)
    ;; uint32 _value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _TString
       (write-long (length _TString) s) (princ _TString s)
     ;; uint32 _value
       (write-long _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _TString
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _TString (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _value
     (setq _value (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get camultiplex::TTest :md5sum-) "6dd10cf3b8877d89431c09fd24eede0b")
(setf (get camultiplex::TTest :datatype-) "camultiplex/TTest")
(setf (get camultiplex::TTest :definition-)
      "string TString
uint32 value

")



(provide :camultiplex/TTest "6dd10cf3b8877d89431c09fd24eede0b")


