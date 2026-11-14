# IFC system diagram

## 1. Delay estimation

`stream_telemetry` has two roles: it first publishes `stream_info` (frame time, size, reset) as soon as a frame arrives, and then `stream_telemetry` (that `stream_info` plus synchronized telemetry, horizon and features) once all data of the frame have arrived.

```mermaid
flowchart TB
    classDef input fill:#6aa84f,stroke:#274e13,color:#fff
    classDef manager fill:#e69138,stroke:#783f04,color:#fff
    classDef telemetry fill:#3d85c6,stroke:#073763,color:#fff

    camera([Camera stream]):::input
    autopilot([Autopilot telemetry]):::input

    horizon([Horizon detection])
    tracker([Feature tracker])
    delay([Delay estimator])
    manager([Calibration manager]):::manager
    synchronizer([Telemetry synchronizer])
    stream([Stream telemetry]):::telemetry

    camera -->|video| horizon
    camera -->|video| tracker
    camera -->|video| stream
    autopilot -->|vehicle_attitude<br/>vehicle_angular_velocity| delay
    autopilot -->|vehicle_attitude<br/>vehicle_local_position<br/>vehicle_global_position| synchronizer
    horizon -->|horizon| tracker
    horizon -->|horizon| delay
    tracker -->|feature_matches| delay
    delay -->|delay| manager
    manager -.->|calibration_results| synchronizer
    synchronizer -.->|autopilot_telemetry| tracker
    synchronizer -->|autopilot_telemetry| stream
    horizon -->|horizon| stream
    tracker -->|feature_matches| stream
    stream -->|stream_info| synchronizer
    stream -->|stream_info| manager
```

## 2. Camera orientation and intrinsics

```mermaid
flowchart TB
    classDef input fill:#6aa84f,stroke:#274e13,color:#fff
    classDef manager fill:#e69138,stroke:#783f04,color:#fff
    classDef telemetry fill:#3d85c6,stroke:#073763,color:#fff

    manager([Calibration manager]):::manager
    orientation(["Camera estimator"])
    stream([Stream telemetry]):::telemetry

    orientation -->|camera_orientation<br/>camera_intrinsics| manager
    stream -->|stream_telemetry| orientation
```

## 3. Distortion

```mermaid
flowchart TB
    classDef input fill:#6aa84f,stroke:#274e13,color:#fff
    classDef manager fill:#e69138,stroke:#783f04,color:#fff
    classDef telemetry fill:#3d85c6,stroke:#073763,color:#fff

    subgraph sources [" "]
        direction LR
        camera([Camera stream]):::input
        stream([Stream telemetry]):::telemetry
    end
    style sources fill:none,stroke:none

    manager([Calibration manager]):::manager
    sfm([Structure from Motion])
    terrain([Terrain model])

    camera -->|video| sfm
    manager -.->|calibration_results| sfm
    sfm -->|distortion<br/>camera_orientation_correction<br/>camera_intrinsics_correction| manager
    stream -->|stream_telemetry| sfm
    stream -->|stream_telemetry| terrain
    terrain -->|terrain| sfm
```
