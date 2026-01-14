# image_crop

A node for cropping image streams.
Useful for removing black borders, for instance on DIYed ultrawide cameras.
Or for displaying a specific part of an image, as one can dynamically change the size and location of the crop.

## examples

This node is configured over parameters.  
It will only subscribe to the imcoming image if it sees a subscriber on it's outgoing topic.  
It's almost always preferrable to use a compressed transport.  
```
# The crop is made from a starting Point (x)
# from which a rectangle with the crop height & width is drawn.
#         ┌────────────────┐
#         │                │
#         │  ┌──────┐      │
#         │  │      │height│
# x-height╌╌╌x──────┘      │
#         │  ╎ width       │
#         └──╎─────────────┘
#         y-height       

# set the incoming image transport
ros2 run image_crop image_crop --ros-args -r image:=/camera/image_raw -p image_transport:="compressed"

# set the crop starting point
ros2 run image_crop image_crop --ros-args -r image:=/camera/image_raw -p image_transport:=compressed -p crop_start_x:=500 -p crop_start_y:=100

# set the crop size
ros2 run image_crop image_crop --ros-args -r image:=/camera/image_raw -p image_transport:=compressed -p target_height:=42 -p target_width:=1337
```
