@ECHO OFF
call c:\opt\ros\foxy\setup.bat
call install\setup.bat
ros2 launch virtual_camera showimageraw.launch.py
