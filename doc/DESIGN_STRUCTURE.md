# MavDrone structure

This document describes the code structure and runtime design of MavDrone, with a focus on the Android MAVLink server implementation.

## Architecture drawings

### 1. Runtime sequence (high level)

```mermaid
sequenceDiagram
    participant GCS as QGC/MAVSDK/ROS2
    participant UI as MainActivity/Widgets
    participant MM as MavManager
    participant MS as MavService
    participant MC as MavConnection
    participant MCS as MavComponents

    GCS->>MC: Connect + MAVLink traffic
    MC->>MCS: Route inbound messages
    MCS-->>MC: Outbound telemetry/ACKs
    MC-->>GCS: Send MAVLink packets
    MC-->>MS: Connection status updates
    MS-->>MM: Service communication events
    MM-->>UI: Status/UI updates
    UI->>MM: Start server
    MM->>MS: Start foreground service
    MS->>MC: Start MAVLink TCP server thread
    MC->>MCS: Create MAVLink components
```

### 2. Mission conversion pipeline

```mermaid
flowchart TD
    A[MISSION_COUNT / MISSION_ITEM_INT] --> B[MissionManager]
    B --> C[WaypointMission or GeofenceMission]
    C --> D[Validation\nallowed cmds/frames/params]
    D --> E[DJIMavlinkConversion]
    E --> F[DJI Wayline / Geofence objects]
    F --> G[WaypointManager / GeofenceManager]
    G --> H[DJI execution + mission state callbacks]
    H --> I[MISSION_CURRENT / MISSION_ITEM_REACHED / MISSION_ACK]
```

### 3. Full class hierarchy (MAVLink server)

```mermaid
classDiagram
    class BaseComponent
    class MavBaseComponent
    class MavManager
    class MavService
    class MavConnection
    class MavComponents
    class Drone
    class Camera
    class Gimbal
    class Battery
    class MissionManager
    class WaypointManager
    class GeofenceManager
    class CameraSettings
    class CameraStream
    class GimbalControl
    class DroneSettings

    BaseComponent --> MavBaseComponent
    BaseComponent --> MissionManager
    BaseComponent --> WaypointManager
    BaseComponent --> GeofenceManager

    MavBaseComponent --> Drone
    MavBaseComponent --> Camera
    MavBaseComponent --> Gimbal
    MavBaseComponent --> Battery

    MavManager --> MavService
    MavService --> MavConnection
    MavConnection --> MavComponents
    MavComponents <|-- Drone
    MavComponents <|-- Camera
    MavComponents <|-- Gimbal
    MavComponents <|-- Battery

    Drone --> MissionManager
    Drone --> DroneSettings
    MissionManager --> WaypointManager
    MissionManager --> GeofenceManager

    Camera --> CameraSettings
    Camera --> CameraStream

    Gimbal --> GimbalControl
```