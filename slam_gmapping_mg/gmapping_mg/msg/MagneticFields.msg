 #modified from MagneticField.msg

 Header header                           # timestamp is the time the
                                         # field was measured
                                         # frame_id is the location and orientation
                                         # of the field measurement

 geometry_msgs/Vector3[9] magnetic_field # x, y, and z components of the
                                         # field vector in Tesla
                                         # If your sensor does not output 3 axes,
                                         # put NaNs in the components not reported.
