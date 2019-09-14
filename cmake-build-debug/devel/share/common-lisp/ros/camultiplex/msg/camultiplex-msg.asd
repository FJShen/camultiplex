
(cl:in-package :asdf)

(defsystem "camultiplex-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "TTest" :depends-on ("_package_TTest"))
    (:file "_package_TTest" :depends-on ("_package"))
  ))