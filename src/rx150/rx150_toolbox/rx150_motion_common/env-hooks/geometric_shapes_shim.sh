# Compatibility shim for the stale ~/ws_moveit overlay, whose MoveIt binaries
# still require libgeometric_shapes.so.2.3.2 while ROS Humble now ships 2.3.4.
_geometric_shapes_shim_dir="$HOME/.local/lib/ros_shim"
_geometric_shapes_shim_lib="$_geometric_shapes_shim_dir/libgeometric_shapes.so.2.3.2"

if [ -e "$_geometric_shapes_shim_lib" ]; then
  ament_prepend_unique_value LD_LIBRARY_PATH "$_geometric_shapes_shim_dir"
fi

unset _geometric_shapes_shim_lib
unset _geometric_shapes_shim_dir
