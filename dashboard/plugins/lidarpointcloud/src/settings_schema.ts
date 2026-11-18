export type SettingKind = 'b' | 'i' | 'f' | 't' | 'p' | 's';
export interface SettingField {
  key: string; label: string; kind: SettingKind; options?: string[]; readOnly?: boolean;
}
export interface SettingGroup { title: string; fields: SettingField[] }

export const settingGroups: Record<string, SettingGroup> = {
  "logging": {
    "title": "Capture Logging",
    "fields": [
      {
        "key": "RawLoggingEnabled",
        "label": "RAW recording",
        "kind": "b"
      },
      {
        "key": "PointCloudLoggingEnabled",
        "label": "Point-cloud recording",
        "kind": "b"
      }
    ]
  },
  "lidar": {
    "title": "LiDAR",
    "fields": [
      {
        "key": "Lidar",
        "label": "LiDAR model",
        "kind": "s",
        "options": [
          "unitree-l2",
          "quanergy-m8",
          "realsense-l515"
        ]
      },
      {
        "key": "ReconnectIntervalSeconds",
        "label": "Reconnect interval (s)",
        "kind": "i"
      },
      {
        "key": "SensorIP",
        "label": "Sensor IP",
        "kind": "t"
      },
      {
        "key": "SensorPort",
        "label": "Sensor port",
        "kind": "i"
      },
      {
        "key": "ConnectionMode",
        "label": "Unitree transport",
        "kind": "s",
        "options": [
          "auto",
          "serial",
          "udp"
        ]
      },
      {
        "key": "LocalIP",
        "label": "Local IP",
        "kind": "t"
      },
      {
        "key": "LocalPort",
        "label": "Local UDP port",
        "kind": "i"
      },
      {
        "key": "SerialPort",
        "label": "Serial port",
        "kind": "p"
      },
      {
        "key": "BaudRate",
        "label": "Baud rate",
        "kind": "i"
      },
      {
        "key": "InitializeOnConnect",
        "label": "Initialize on connect",
        "kind": "b"
      },
      {
        "key": "WorkMode",
        "label": "Unitree work mode",
        "kind": "i"
      },
      {
        "key": "UsbSerial",
        "label": "USB serial",
        "kind": "t"
      },
      {
        "key": "DepthWidth",
        "label": "Depth width",
        "kind": "i"
      },
      {
        "key": "DepthHeight",
        "label": "Depth height",
        "kind": "i"
      },
      {
        "key": "DepthFps",
        "label": "Depth FPS",
        "kind": "i"
      }
    ]
  },
  "camera": {
    "title": "Camera",
    "fields": [
      {
        "key": "Camera",
        "label": "Camera source",
        "kind": "s",
        "options": [
          "None",
          "RealSense"
        ],
        "readOnly": true
      }
    ]
  },
  "lio_calibration": {
    "title": "Point-LIO · IMU and Calibration",
    "fields": [
      {
        "key": "LioRobustImuEnabled",
        "label": "Robust IMU",
        "kind": "b"
      },
      {
        "key": "LioStationaryUpdateEnabled",
        "label": "Stationary update",
        "kind": "b"
      },
      {
        "key": "LioLidarToImuTranslationX",
        "label": "LiDAR to IMU X (m)",
        "kind": "f"
      },
      {
        "key": "LioLidarToImuTranslationY",
        "label": "LiDAR to IMU Y (m)",
        "kind": "f"
      },
      {
        "key": "LioLidarToImuTranslationZ",
        "label": "LiDAR to IMU Z (m)",
        "kind": "f"
      },
      {
        "key": "LioLidarToImuRollDeg",
        "label": "LiDAR to IMU Roll (°)",
        "kind": "f"
      },
      {
        "key": "LioLidarToImuPitchDeg",
        "label": "LiDAR to IMU Pitch (°)",
        "kind": "f"
      },
      {
        "key": "LioLidarToImuYawDeg",
        "label": "LiDAR to IMU Yaw (°)",
        "kind": "f"
      }
    ]
  },
  "lio_scan": {
    "title": "Point-LIO · Scans and Local Map",
    "fields": [
      {
        "key": "LioScanVoxelSizeMeters",
        "label": "Scan voxel (m)",
        "kind": "f"
      },
      {
        "key": "LioMapVoxelSizeMeters",
        "label": "Map voxel (m)",
        "kind": "f"
      },
      {
        "key": "LioLocalMapRadiusMeters",
        "label": "Local map radius (m)",
        "kind": "f"
      },
      {
        "key": "LioLocalMapMaxPoints",
        "label": "Local map point limit",
        "kind": "i"
      },
      {
        "key": "LioScanMinPoints",
        "label": "Minimum scan points",
        "kind": "i"
      },
      {
        "key": "LioScanMaxPoints",
        "label": "Maximum scan points",
        "kind": "i"
      },
      {
        "key": "LioInitializationSamples",
        "label": "Initialization samples",
        "kind": "i"
      }
    ]
  },
  "lio_noise": {
    "title": "Point-LIO · Covariances",
    "fields": [
      {
        "key": "LioImuAccelerationCovariance",
        "label": "Acceleration covariance",
        "kind": "f"
      },
      {
        "key": "LioImuGyroscopeCovariance",
        "label": "Gyroscope covariance",
        "kind": "f"
      },
      {
        "key": "LioImuAccelerationBiasCovariance",
        "label": "Acceleration bias covariance",
        "kind": "f"
      },
      {
        "key": "LioImuGyroscopeBiasCovariance",
        "label": "Gyroscope bias covariance",
        "kind": "f"
      },
      {
        "key": "LioVelocityProcessCovariance",
        "label": "Velocity process covariance",
        "kind": "f"
      },
      {
        "key": "LioAccelerationProcessCovariance",
        "label": "Acceleration process covariance",
        "kind": "f"
      },
      {
        "key": "LioAngularVelocityProcessCovariance",
        "label": "Angular velocity process covariance",
        "kind": "f"
      },
      {
        "key": "LioLidarMeasurementCovariance",
        "label": "LiDAR measurement covariance",
        "kind": "f"
      },
      {
        "key": "LioImuAccelerationNorm",
        "label": "IMU acceleration norm",
        "kind": "f"
      },
      {
        "key": "LioImuAccelerationSaturation",
        "label": "Acceleration saturation",
        "kind": "f"
      },
      {
        "key": "LioImuGyroscopeSaturation",
        "label": "Gyroscope saturation",
        "kind": "f"
      }
    ]
  },
  "lio_quality": {
    "title": "Point-LIO · Matching and Timing",
    "fields": [
      {
        "key": "LioMinRangeMeters",
        "label": "Minimum matching range (m)",
        "kind": "f"
      },
      {
        "key": "LioPlaneThresholdMeters",
        "label": "Plane threshold (m)",
        "kind": "f"
      },
      {
        "key": "LioMaxImuGapMilliseconds",
        "label": "Maximum IMU gap (ms)",
        "kind": "f"
      },
      {
        "key": "LioMinMatchRatio",
        "label": "Minimum match ratio",
        "kind": "f"
      },
      {
        "key": "LioMaxResidualMeters",
        "label": "Maximum residual (m)",
        "kind": "f"
      },
      {
        "key": "LioMaxTranslationStepMeters",
        "label": "Maximum translation step (m)",
        "kind": "f"
      },
      {
        "key": "LioMaxRotationStepDeg",
        "label": "Maximum rotation step (°)",
        "kind": "f"
      },
      {
        "key": "LioSynchronizationTimeoutMilliseconds",
        "label": "Synchronization timeout (ms)",
        "kind": "i"
      }
    ]
  },
  "radar": {
    "title": "Radar",
    "fields": [
      {
        "key": "Radar",
        "label": "Radar source",
        "kind": "s",
        "options": [
          "None"
        ],
        "readOnly": true
      }
    ]
  },
  "telemetry": {
    "title": "Grafana and System Monitor",
    "fields": [
      {
        "key": "GrafanaPublishIntervalMilliseconds",
        "label": "Grafana publish interval (ms)",
        "kind": "i"
      },
      {
        "key": "GrafanaRetryIntervalSeconds",
        "label": "Grafana retry interval (s)",
        "kind": "i"
      },
      {
        "key": "SystemMonitorIntervalMilliseconds",
        "label": "Monitor interval (ms)",
        "kind": "i"
      }
    ]
  },
  "room_map": {
    "title": "Room Map · Build and Cache",
    "fields": [
      {
        "key": "RoomMapEnabled",
        "label": "Room Map enabled",
        "kind": "b"
      },
      {
        "key": "RoomMapVoxelSizeMeters",
        "label": "Voxel (m)",
        "kind": "f"
      },
      {
        "key": "RoomMapCacheMaxVoxels",
        "label": "Cache voxel limit",
        "kind": "i"
      },
      {
        "key": "RoomMapCacheMaxTiles",
        "label": "Cache tile limit",
        "kind": "i"
      },
      {
        "key": "RoomMapTileSizeMeters",
        "label": "Tile (m)",
        "kind": "f"
      },
      {
        "key": "RoomMapLodPointsPerNode",
        "label": "LOD points per node",
        "kind": "i"
      },
      {
        "key": "RoomMapMinObservations",
        "label": "Minimum observations",
        "kind": "i"
      },
      {
        "key": "RoomMapObservationIntervalMilliseconds",
        "label": "Observation interval (ms)",
        "kind": "i"
      },
      {
        "key": "RoomMapFreezeAfterSeconds",
        "label": "Freeze after (s, 0 = continuous)",
        "kind": "i"
      }
    ]
  },
  "room_map_io": {
    "title": "Room Map · Source, Raycast and Streaming",
    "fields": [
      {
        "key": "RoomMapPublishIntervalMilliseconds",
        "label": "Checkpoint interval (ms)",
        "kind": "i"
      },
      {
        "key": "RoomMapLoadExisting",
        "label": "Load PCD reference",
        "kind": "b"
      },
      {
        "key": "RoomMapFile",
        "label": "PCD path on edge",
        "kind": "t"
      },
      {
        "key": "RoomMapFreeSpaceMinObservations",
        "label": "Free space observation minimum",
        "kind": "i"
      },
      {
        "key": "RoomMapRaycastMaxRaysPerWindow",
        "label": "Ray limit per window",
        "kind": "i"
      },
      {
        "key": "RoomMapRaycastMaxRangeMeters",
        "label": "Maximum ray range (m)",
        "kind": "f"
      },
      {
        "key": "RoomMapRaycastSurfaceMarginMeters",
        "label": "Ray surface margin (m)",
        "kind": "f"
      },
      {
        "key": "RoomMapWebSocketBindAddress",
        "label": "Room Map bind address",
        "kind": "t",
        "readOnly": true
      },
      {
        "key": "RoomMapWebSocketPort",
        "label": "Room Map port",
        "kind": "i",
        "readOnly": true
      },
      {
        "key": "RoomMapWebSocketMaxPoints",
        "label": "Room Map streamed points",
        "kind": "i"
      },
      {
        "key": "RoomMapWebSocketMaxClients",
        "label": "Map stream clients per server",
        "kind": "i"
      },
      {
        "key": "RoomMapWebSocketRetryIntervalSeconds",
        "label": "Map stream retry interval (s)",
        "kind": "i"
      }
    ]
  },
  "gaussian": {
    "title": "Gaussian Map",
    "fields": [
      {
        "key": "GaussianMapEnabled",
        "label": "Gaussian Map enabled",
        "kind": "b"
      },
      {
        "key": "GaussianMapWebSocketBindAddress",
        "label": "Gaussian bind address",
        "kind": "t",
        "readOnly": true
      },
      {
        "key": "GaussianMapWebSocketPort",
        "label": "Gaussian port",
        "kind": "i",
        "readOnly": true
      },
      {
        "key": "GaussianMapWebSocketMaxPoints",
        "label": "Gaussian streamed points",
        "kind": "i"
      }
    ]
  },
  "foreground": {
    "title": "Foreground",
    "fields": [
      {
        "key": "ForegroundEnabled",
        "label": "Foreground enabled",
        "kind": "b"
      },
      {
        "key": "ForegroundBackgroundDistanceMeters",
        "label": "Background tolerance (m)",
        "kind": "f"
      },
      {
        "key": "ForegroundOutlierFilterEnabled",
        "label": "Outlier filter enabled",
        "kind": "b"
      },
      {
        "key": "ForegroundOutlierRadiusMeters",
        "label": "Near radius (m)",
        "kind": "f"
      },
      {
        "key": "ForegroundOutlierStartDistanceMeters",
        "label": "Radius growth starts (m)",
        "kind": "f"
      },
      {
        "key": "ForegroundOutlierEndDistanceMeters",
        "label": "Radius growth ends (m)",
        "kind": "f"
      },
      {
        "key": "ForegroundOutlierMaxRadiusMeters",
        "label": "Maximum radius (m)",
        "kind": "f"
      },
      {
        "key": "ForegroundOutlierMinNeighbors",
        "label": "Minimum neighbors",
        "kind": "i"
      },
      {
        "key": "ForegroundMaxPoints",
        "label": "Maximum sampled points",
        "kind": "i"
      },
      {
        "key": "ForegroundPublishIntervalMilliseconds",
        "label": "Processing and publish interval (ms)",
        "kind": "i"
      },
      {
        "key": "ForegroundWebSocketBindAddress",
        "label": "Foreground bind address",
        "kind": "t",
        "readOnly": true
      },
      {
        "key": "ForegroundWebSocketPort",
        "label": "Foreground port",
        "kind": "i",
        "readOnly": true
      }
    ]
  }
};
