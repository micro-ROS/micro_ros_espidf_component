# Extra ROS2 packages for microros
Place extra ROS2 packages here, they will get copied into the ROS workspace at build time.

The ESP-IDF has trouble calling clean on `make` based components, so changes to these packages may not get pulled into source on a rebuild.

If changes are not getting merged, try `make -f libmicroros.mk clean` within the microros component directory to force a rebuild of just this component.
