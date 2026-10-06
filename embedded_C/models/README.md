# Topic model boundaries

Models describe data only. IMU and mapping models do not include LiDAR envelopes
or LiDAR point layouts; they compile independently of device/transport code.

| Header | Type | Topic |
| --- | --- | --- |
| `imu/imu_message.hpp` | `ImuFrame`, `ImuMessage` | `lidar/imu` |
| `mapping/localization.hpp` | `LocalizationStatus` | `localization/status` |
| `mapping/ground_status.hpp` | `GroundStatus`, `GroundStatusMessage` | `processing/ground_status` |
| `mapping/room_map.hpp` | `RoomMapPoint`, `RoomMapFrame`, `RoomMapMessage` | `mapping/room_map` |
| `mapping/room_map.hpp` | `RoomMapViewMessage`, `RoomMapStatus` | `mapping/room_map_view`, `mapping/room_map_status` |

`ImuMessage` and `GroundStatusMessage` are independent structures, not aliases
of `LidarMessage<T>`. Their `source_id` identifies the producer. Every active
IMU, integrated or external, publishes the same `ImuMessage` type to `lidar/imu`.
The existing Grafana wire label `lidar_id` is retained at the formatter boundary
so the dashboard does not need reimporting solely for this refactor.

Room-map geometry owns XYZ and intensity only. The mapper explicitly copies
these fields from accepted world points into persistent cells. Per-point ray
origins, rings, return IDs and acquisition timestamps stay on the measured scan,
where free-space traversal still needs them; they are not map topic fields.
Map storage and LOD readers use `RoomMapPoint`/`RoomMapFrame` throughout, and the
map WebSocket encoder accepts `RoomMapMessage` directly. PCD and LPC1 formats are
unchanged. Tile/LOD records retain their previous width; obsolete sensor-only
slots are reserved and ignored when reading older pages.

## Clock contract

Synchronization and propagation use the Windows/Linux host's monotonic
`std::chrono::steady_clock`. `received_monotonic_ns` is captured at transport
receipt, and `measurement_timestamp_ns` aligns the sample to that host clock
before queuing. Original device timestamps are retained for relative intervals
and reset/order checks, never as an independent absolute synchronization epoch.
Do not timestamp a queued measurement with the time a worker happens to process it.

Calendar timestamps use the host `system_clock` for Grafana/display metadata
and filenames only. They may change with NTP or manual clock adjustments and
must not be used for LIO integration. Room-map observation windows and active
build time continue to use host monotonic elapsed time.
