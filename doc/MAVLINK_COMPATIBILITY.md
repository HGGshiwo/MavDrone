# MAVLink Compatibility Matrix

This document lists MAVLink messages and commands currently implemented by MavDrone, and where behavior differs from a default PX4-style implementation.

Scope:
- Based on current implementation in the Android app code under `app/src/main/java/nl/wur/mavdrone/communication`.
- Focused on server-side MAVLink behavior (what MavDrone accepts, emits, and translates).

## Protocol profile and major differences

1. Autopilot identity
- Heartbeat autopilot is reported as `MAV_AUTOPILOT_GENERIC_MISSION_FULL` (not `MAV_AUTOPILOT_PX4`).
- Custom mode values emulate PX4-style main/sub-modes where possible.

2. Mission protocol flavor
- Mission transfer is implemented with `MISSION_ITEM_INT` / `MISSION_REQUEST_INT`.
- Integer mission protocol is the primary/expected path.

3. Command handling model
- Most actionable commands are handled through `COMMAND_LONG`.
- `COMMAND_CANCEL` is implemented for selected flight commands (takeoff/land/RTL).

4. Camera and gimbal model
- DJI camera/gimbal capabilities are translated to MAVLink camera/gimbal manager APIs.
- Some fields/behaviors are approximated where DJI APIs do not map 1:1.

5. Geofence behavior
- Geofence mission items are supported and interpreted by MavDrone.
- Breach handling behavior is implementation-specific (RTL/land/guided-like recovery logic).

## Commands implemented

| MAVLink command | Scope | Difference vs default MAVLink/PX4 behavior |
|---|---|---|
| `MAV_CMD_REQUEST_MESSAGE` | Core/component base | Only messages registered by each component can be requested. |
| `MAV_CMD_SET_MESSAGE_INTERVAL` | Core/component base | Interval control applies only to implemented senders. |
| `MAV_CMD_GET_MESSAGE_INTERVAL` | Core/component base | Reports interval only for implemented senders. |
| `MAV_CMD_DO_SET_MODE` | Flight controller/autopilot | PX4-like custom mode handling is partial and mapped to DJI-supported flows. |
| `MAV_CMD_COMPONENT_ARM_DISARM` | Flight controller/autopilot | Arm/disarm behavior is app-state and DJI-gated, not PX4 motor arming internals. |
| `MAV_CMD_NAV_TAKEOFF` | Flight controller/autopilot | Requires DJI-compatible preconditions (for example stabilized/armed state checks). |
| `MAV_CMD_NAV_LAND` | Flight controller/autopilot | Can be direct DJI autoland or converted land mission depending on parameters. |
| `MAV_CMD_DO_LAND_START` | Flight controller/autopilot | Treated together with land flow, mapped to DJI landing behavior. |
| `MAV_CMD_NAV_RETURN_TO_LAUNCH` | Flight controller/autopilot | Mapped to DJI go-home action semantics. |
| `MAV_CMD_DO_SET_HOME` | Flight controller/autopilot | Uses DJI homepoint APIs; behavior follows DJI constraints. |
| `MAV_CMD_GET_HOME_POSITION` | Flight controller/autopilot | Returns mapped DJI homepoint state. |
| `MAV_CMD_OVERRIDE_GOTO` | Flight controller/autopilot | Implemented through mission pause/resume or temporary generated goto mission. |
| `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES` | Flight controller/autopilot | Capabilities are curated for implemented features, not full PX4 coverage. |
| `MAV_CMD_REQUEST_FLIGHT_INFORMATION` | Flight controller/autopilot | Timing fields are app-tracked and DJI-derived. |
| `MAV_CMD_LOGGING_START` | Flight controller/autopilot | ACK path implemented; does not imply PX4 logger parity. |
| `MAV_CMD_LOGGING_STOP` | Flight controller/autopilot | ACK path implemented; does not imply PX4 logger parity. |
| `MAV_CMD_MISSION_START` | Mission control | Start mapped to DJI waypoint mission manager execution. |
| `MAV_CMD_DO_PAUSE_CONTINUE` | Mission control | Pause/resume mapped to DJI mission execution states. |
| `MAV_CMD_DO_SET_MISSION_CURRENT` | Mission control | Current index is mapped through MAVLink-to-DJI waypoint index conversion. |
| `MAV_CMD_DO_FENCE_ENABLE` | Mission control/geofence | Enables/disables app-side geofence engine rather than PX4-native fence subsystem. |
| `MAV_CMD_VIDEO_START_STREAMING` | Camera | Uses DJI live stream manager and RTSP setup, not generic MAVLink camera backend. |
| `MAV_CMD_VIDEO_STOP_STREAMING` | Camera | Stops/updates DJI live stream manager state. |
| `MAV_CMD_SET_CAMERA_MODE` | Camera | MAVLink camera mode mapped to supported DJI camera modes subset. |
| `MAV_CMD_RESET_CAMERA_SETTINGS` | Camera | Mapped to DJI reset capability where available. |
| `MAV_CMD_IMAGE_START_CAPTURE` | Camera | Single/interval capture mapped to DJI shoot-photo flows and constraints. |
| `MAV_CMD_IMAGE_STOP_CAPTURE` | Camera | Stops DJI interval capture flow where active. |
| `MAV_CMD_VIDEO_START_CAPTURE` | Camera | Mapped to DJI start-record flow. |
| `MAV_CMD_VIDEO_STOP_CAPTURE` | Camera | Mapped to DJI stop-record flow. |
| `MAV_CMD_STORAGE_FORMAT` | Camera | Mapped to DJI storage format paths with implementation-specific handling. |
| `MAV_CMD_REQUEST_CAMERA_INFORMATION` | Camera | Returns DJI-derived camera metadata/capabilities. |
| `MAV_CMD_REQUEST_CAMERA_SETTINGS` | Camera | Returns mapped camera mode/settings subset. |
| `MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS` | Camera | Returns synthesized capture status from DJI state. |
| `MAV_CMD_REQUEST_STORAGE_INFORMATION` | Camera | Returns DJI/onboard storage model mapping. |
| `MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION` | Camera | Returns DJI RTSP stream info mapping. |
| `MAV_CMD_REQUEST_VIDEO_STREAM_STATUS` | Camera | Returns DJI live-stream status mapping. |
| `MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE` | Gimbal | Ownership/control model is app-managed per configured controller IDs. |
| `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW` | Gimbal | Mapped to DJI gimbal angle control with MAVLink flag translation constraints. |
| `MAV_CMD_DO_MOUNT_CONTROL` | Gimbal | Supported with MAVLink-targeting style mapping to DJI gimbal actions. |

### Other implemented MAVLink control messages (non-command)

| MAVLink message | Scope | Difference vs default MAVLink/PX4 behavior |
|---|---|---|
| `GIMBAL_MANAGER_SET_PITCHYAW` | Gimbal | Handled via app-managed controller ownership checks and DJI control mapping. |
| `GIMBAL_MANAGER_SET_ATTITUDE` | Gimbal | Quaternion-based setpoint mapped to DJI angle control with yaw-frame conversion. |
| `FILE_TRANSFER_PROTOCOL` | File transfer | Implemented in app with session-based MAVLink FTP behavior over supported file/media backends. |

## Mission item commands implemented

### Waypoint mission (`MAV_MISSION_TYPE_MISSION`)

| Mission item | How MavDrone implements it | Difference vs default MAVLink/PX4 behavior |
|---|---|---|
| `MAV_CMD_NAV_TAKEOFF` | Used as takeoff reference for DJI wayline mission generation. | Yaw (`param4`) is not supported in mission upload validation and may be ignored in conversion logic. |
| `MAV_CMD_NAV_WAYPOINT` | Converted to DJI waypoints (`WaylineExecuteWaypoint`). | Only global relative altitude waypoint frames are accepted for upload. |
| `MAV_CMD_NAV_DELAY` | Converted to DJI hover action. | Time-based delay only; absolute-time variant (`param1 = -1`) is rejected. |
| `MAV_CMD_NAV_LAND` | Converted to DJI final waypoint + mission finish action `AUTO_LAND`. | Must be the last mission item when used in waypoint missions. |
| `MAV_CMD_NAV_RETURN_TO_LAUNCH` | Converted to mission finish action `GO_HOME`. | Must be the last mission item when used in waypoint missions. |
| `MAV_CMD_DO_CHANGE_SPEED` | Sets global or local waypoint speed in DJI mission model. | `param2 = -2` is used to reset to global speed; behavior follows converter rules rather than full PX4 semantics. |
| `MAV_CMD_CONDITION_YAW` | Converted to DJI rotate-yaw action. | Direction is derived from `param3` (clockwise/counter-clockwise); rate/advanced yaw semantics are not fully modeled. |
| `MAV_CMD_CONDITION_DELAY` | Converted to DJI hover action at current waypoint stage. | Implemented as action-level delay; not a full scheduler/condition engine. |
| `MAV_CMD_IMAGE_START_CAPTURE` | Converted to either single-photo action or timed/multiple capture action. | Counted burst semantics are limited; continuous/timed behavior is represented via DJI action groups. |
| `MAV_CMD_IMAGE_STOP_CAPTURE` | Stops a running image-capture action group. | Requires matching prior start in mission validation; unmatched stop may be rejected/ignored depending on direction of conversion. |
| `MAV_CMD_VIDEO_START_CAPTURE` | Converted to DJI start-record action. | Treated as long-running recording state until stop action or mission end handling. |
| `MAV_CMD_VIDEO_STOP_CAPTURE` | Converted to DJI stop-record action. | Requires prior start in mission validation; unmatched stop may be rejected/ignored depending on conversion path. |
| `MAV_CMD_DO_SET_CAM_TRIGG_DIST` | Mapped to repeated photo action with distance trigger. | Trigger-distance is represented via DJI multiple-distance action grouping, not native MAVLink trigger engine. |
| `MAV_CMD_SET_CAMERA_FOCUS` | Converted to DJI set-focus-type action. | Mapping is approximate; some focus modes are coerced and auto-focus may be emulated with area-focus helper actions. |
| `MAV_CMD_SET_CAMERA_ZOOM` | Converted to DJI zoom action using focal length. | Implemented as focal-length zoom mapping; non-focal zoom semantics are not fully preserved. |
| `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW` | Converted to DJI gimbal rotate action with MAVLink gimbal flags mapping. | Exactly one yaw reference mode is required (earth-frame xor vehicle-frame) during mission validation. |
| `MAV_CMD_DO_MOUNT_CONTROL` | Converted to DJI gimbal rotate action. | Treated as MAVLink-targeting style gimbal command with simplified mapping to DJI rotate behavior. |

Mission-level conversion notes:
- DJI requires at least two waypoints. If only one MAVLink waypoint is provided and current aircraft GPS is available, MavDrone inserts an extra waypoint (typically current position) to satisfy DJI constraints.
- Action groups are normalized to DJI wayline actions; some MAVLink mission semantics are reconstructed when exporting back to MAVLink.

### Geofence mission (`MAV_MISSION_TYPE_FENCE`)

| Mission item | How MavDrone implements it | Difference vs default MAVLink behavior |
|---|---|---|
| `MAV_CMD_NAV_FENCE_RETURN_POINT` | Stored as geofence return point used by breach handling logic. | Enforcement behavior depends on MavDrone fence action settings (guided-like return/RTL/land), not autopilot-native geofence internals. |
| `MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION` | Converted to inclusion circle geometry. | Evaluated in MavDrone geofence checker; not delegated to PX4 geofence engine. |
| `MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION` | Converted to exclusion circle geometry. | Same as above; evaluated by app-side geofence logic. |
| `MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION` | Polygon assembled from vertices and validated as complete ring. | Polygon completion and validation is strict in converter logic. |
| `MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION` | Exclusion polygon assembled similarly. | Same strict app-side validation and execution model. |

## Messages emitted by MavDrone

### Common/base

| Message | Source | Difference vs default MAVLink protocol |
|---|---|---|
| `HEARTBEAT` | All active MAVLink components | `autopilot` reports `MAV_AUTOPILOT_GENERIC_MISSION_FULL` instead of a PX4-specific autopilot id. |
| `PROTOCOL_VERSION` | Component base layer | No notable protocol-level deviation. |
| `SYSTEM_TIME` | Component base layer | No notable protocol-level deviation. |
| `MESSAGE_INTERVAL` | Component base layer | Availability is limited to messages registered by each component implementation. |
| `PARAM_VALUE` | Component parameter system | Parameter surface is implementation-defined (DJI-backed set), not PX4 parameter set completeness. |
| `PARAM_EXT_VALUE` | Component parameter system | Extended parameter support is implementation-defined per component. |
| `COMMAND_ACK` | Command handlers | ACK result choices can reflect DJI/backend constraints rather than PX4/autopilot-internal causes. |
| `MISSION_ACK` | Mission handlers | Mission ACKs include converter/validation constraints specific to DJI wayline translation. |

### Flight/vehicle state

| Message | Source | Difference vs default MAVLink protocol |
|---|---|---|
| `SYS_STATUS` | Autopilot component | Values are derived from DJI state and health mapping, not PX4 estimator/controller internals. |
| `EXTENDED_SYS_STATE` | Autopilot component | Landed/VTOL semantics are mapped from DJI states and may not match PX4 state machine transitions exactly. |
| `GPS_RAW_INT` | Autopilot component | GPS reporting is mapped from DJI keys; fix-level semantics can differ from PX4 GPS pipeline details. |
| `ATTITUDE` | Autopilot component | Attitude is DJI-derived; frame/sign conventions are mapped for MAVLink compatibility. |
| `GLOBAL_POSITION_INT` | Autopilot component | Position/velocity/heading are translated from DJI representations. |
| `ALTITUDE` | Autopilot component | Relative altitude mapping is DJI-source dependent. |
| `HOME_POSITION` | Autopilot component | Home position semantics follow DJI homepoint handling. |
| `AUTOPILOT_VERSION` | Autopilot component | Capability flags are curated for supported features and may differ from PX4 capability breadth. |
| `FLIGHT_INFORMATION` | Autopilot component | Arming/takeoff timing fields are produced from app-side tracked timing values. |
| `COMPONENT_INFORMATION` | Autopilot component | Metadata endpoints are generated by app-side component info generators. |
| `COMPONENT_METADATA` | Autopilot component | Metadata content/coverage is implementation-specific. |

### Mission state and transfer

| Message | Source | Difference vs default MAVLink protocol |
|---|---|---|
| `MISSION_CURRENT` | Mission manager/waypoint manager | State/progress is reconstructed from DJI wayline execution state and mavlink-to-DJI index mapping. |
| `MISSION_ITEM_REACHED` | Waypoint manager | Emission is based on mapped action/waypoint progression, not native PX4 mission item executor callbacks. |
| `MISSION_COUNT` | Mission transfer logic | Mission transfer follows supported mission types and converter constraints (not full generic mission-item universe). |
| `MISSION_REQUEST_INT` | Mission transfer logic | INT mission protocol path is the primary supported transfer mode. |
| `MISSION_ITEM_INT` | Mission transfer logic | Accepted item set is restricted to converter-supported commands/frames/params. |
| `MISSION_ACK` | Mission transfer/validation logic | Errors may reflect DJI conversion constraints (frame/sequence/param checks) beyond baseline MAVLink parser checks. |
| `FENCE_STATUS` | Geofence manager | Fence status is app-side geofence engine output, not PX4 geofence subsystem output. |

### Payload, I/O, and perception

| Message | Source | Difference vs default MAVLink protocol |
|---|---|---|
| `CAMERA_INFORMATION` | Camera/onboard camera components | Capability/model fields are DJI-derived and may omit fields expected by specific MAVLink camera implementations. |
| `CAMERA_SETTINGS` | Camera/onboard camera components | Mode/settings are mapped subset, not full PX4 camera stack parity. |
| `CAMERA_CAPTURE_STATUS` | Camera component | Capture state is synthesized from DJI capture/session state. |
| `CAMERA_IMAGE_CAPTURED` | Camera component | Generated from DJI media events; timing/metadata fidelity depends on DJI media callbacks. |
| `VIDEO_STREAM_INFORMATION` | Camera/onboard camera components | Stream info reflects DJI RTSP/live-stream manager setup, not generic camera backend assumptions. |
| `VIDEO_STREAM_STATUS` | Camera/onboard camera components | Status values are mapped from DJI livestream status API. |
| `STORAGE_INFORMATION` | Camera/onboard camera components | Storage model comes from DJI storage APIs (or onboard cache path for onboard camera mode). |
| `GIMBAL_MANAGER_INFORMATION` | Gimbal component | Capability and limit fields are mapped from DJI gimbal model and static/product-specific data. |
| `GIMBAL_MANAGER_STATUS` | Gimbal component | Control ownership/state follows app-managed gimbal manager control model. |
| `GIMBAL_DEVICE_INFORMATION` | Gimbal component | Some device metadata/limits may be static or product-profile based. |
| `GIMBAL_DEVICE_ATTITUDE_STATUS` | Gimbal component | Attitude/quaternion is translated from DJI attitude and yaw-reference handling. |
| `BATTERY_STATUS` | Battery components | Battery fields are aggregated/mapped from DJI battery keys (including multi-battery behavior). |
| `SMART_BATTERY_INFO` | Battery components | Smart battery fields come from known battery profiles plus DJI key values. |
| `RC_CHANNELS` | Remote controller integration | Channel values are scaled from DJI controller ranges to MAVLink RC range conventions. |
| `OBSTACLE_DISTANCE` | Radar/perception integration | Obstacle fan is resampled/collapsed to MAVLink message constraints from DJI obstacle array size. |

## Important differences from default PX4/common expectations

1. PX4 compatibility is partial
- PX4-like custom modes are translated only for supported flows (not all PX4 mode combinations are implemented).

2. Mission constraints are stricter than generic MAVLink
- Waypoint frame support is restricted (global-relative-altitude frames for waypoint items).
- Some sequencing and parameter combinations are enforced and can return mission errors.
- `MAV_CMD_NAV_LAND` and `MAV_CMD_NAV_RETURN_TO_LAUNCH` are expected as terminal mission actions when used in missions.

3. Action translation is not always 1:1
- Some camera/gimbal/focus semantics are approximated because DJI APIs expose different primitives.

4. Arm/disarm semantics
- Arm/disarm is handled as MavDrone control-state gating for mission/takeoff logic and does not map perfectly to PX4 motor state semantics.

5. Stream/camera topology
- Video stream exposure and camera indexing are adapted to available DJI live stream sources and may differ from multi-stream assumptions in some GCS tools.
