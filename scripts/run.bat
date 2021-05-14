@ECHO OFF
call c:\opt\ros\foxy\setup.bat
call install\setup.bat
ros2 run virtual_camera virtual_camera
