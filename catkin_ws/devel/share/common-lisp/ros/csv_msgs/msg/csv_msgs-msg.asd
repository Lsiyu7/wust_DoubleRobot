
(cl:in-package :asdf)

(defsystem "csv_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "csvfile" :depends-on ("_package_csvfile"))
    (:file "_package_csvfile" :depends-on ("_package"))
  ))