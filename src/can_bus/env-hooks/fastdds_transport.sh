# Keep explicit middleware choices; the workspace default avoids SHM crash-recovery stalls.
case "${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" in
  rmw_fastrtps_cpp|rmw_fastrtps_dynamic_cpp)
    if [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ] && [ -z "${FASTDDS_DEFAULT_PROFILES_FILE:-}" ]; then
      export FASTRTPS_DEFAULT_PROFILES_FILE="${COLCON_CURRENT_PREFIX:-${AMENT_CURRENT_PREFIX:?Missing package prefix}}/share/can_bus/config/fastdds_udp.xml"
    fi
    ;;
esac
