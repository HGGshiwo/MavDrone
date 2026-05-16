from __future__ import annotations

import os
from asyncio import CancelledError, create_task, gather, sleep
from dataclasses import dataclass
from logging import INFO, basicConfig, getLogger
from typing import TYPE_CHECKING

import numpy as np
from mavsdk import System
from mavsdk.gimbal import ControlMode, GimbalError, GimbalMode, SendMode
from mavsdk.mission_raw import MissionRawError
from mavsdk.telemetry import LandedState

from mavdrone.exceptions import (
    DroneNotInAirException,
    NoMissionException,
    NotArmedException,
    NotConnectedException,
)
from mavdrone.mission import create_mission
from mavdrone.status import DroneStatus

if TYPE_CHECKING:
    from asyncio import Future
    from types import TracebackType
    from typing import Callable

    from mavsdk.mission_raw import MissionItem
    from numpy.typing import NDArray

log = getLogger(__name__)


@dataclass
class MavSDKServerConnection:
    address: str
    port: int


class Drone:
    """Class representing a drone."""

    def __init__(
        self,
        address: str = "udp://:14540",
        timeout: float = 8.0,
        start_attempts: int = 3,
        upload_attempts: int = 3,
        mavsdk_connection: MavSDKServerConnection | None = None,
        log_level: int = INFO,
    ) -> None:
        """
        Initialize a Drone instance.

        Args:
            address: Address of the drone's system (default: "udp://:14540").
            timeout: Timeout for MAVLink communication with the drone (default: 8.0 seconds).
            start_attempts: Number of attempts to start the mission (default: 3).
            upload_attempts: Number of attempts to upload the mission (default: 3).
            custom_server_connection: Override the server connection.
            log_level: Logging level (default: logging.INFO).
        """
        # Initialise logging
        basicConfig(level=log_level)

        if mavsdk_connection is not None:
            self._system = System(
                mavsdk_server_address=mavsdk_connection.address,
                port=mavsdk_connection.port,
            )
        elif os.name == "nt":
            self._system = System(mavsdk_server_address="localhost", port=50051)
        else:
            self._system = System()

        self.status = DroneStatus()

        self._address = address
        self._timeout = timeout
        self._start_attempts = start_attempts
        self._upload_attempts = upload_attempts

        self.tasks: list[Future] = []

    @property
    def connected(self) -> bool:
        """Check if the drone is connected."""
        return self.status.armed and self.status.gps_fixed

    @property
    def armed(self) -> bool:
        """Check if the drone is armed."""
        return self.status.armed

    @property
    def mission_items(self) -> list[MissionItem]:
        return self.status.mission_items

    async def __aenter__(self) -> Drone:
        await self.connect()
        return self

    async def __aexit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: TracebackType,
    ) -> None:
        await self.disconnect()

    def set_mission(self, mission_items: list[MissionItem]) -> None:
        self.status.mission_items = mission_items

    async def connect(self, wait_time: float = 5.0) -> None:
        """Connect to the drone."""
        log.info(f"Trying to connect to {self._address}")

        await self._system.connect(system_address=self._address)
        await self._system.core.set_mavlink_timeout(self._timeout)

        await sleep(wait_time)  # Delay to give some time for plugins to register

        self.tasks.extend(
            [
                create_task(self._telemetry_status_text()),
                create_task(self._telemetry_position()),
                create_task(self._telemetry_attitude()),
                create_task(self._gimbal_attitude()),
                # create_task(self._telemetry_velocity()),
                create_task(self._telemetry_landed_state()),
            ]
        )

        await self._wait_for_connection()
        await self._wait_for_gps_fix()

    async def wait_for_stream(self) -> None:
        async for video_stream_update in self._system.camera.video_stream_info():
            self.status.video_stream_info = video_stream_update.video_stream_info

            log.info("--started video stream")
            return

    async def rotate_gimbal(
        self,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
        ignore_errors: bool = True,
    ) -> None:
        log.info("Start rotating gimbal")
        if ignore_errors:

            async def execute_ignore[**P](
                fn: Callable[P, None], *args, **kwargs
            ) -> None:
                try:
                    await fn(*args, **kwargs)
                except GimbalError:
                    pass

            await execute_ignore(
                self._system.gimbal.take_control, 0, ControlMode.PRIMARY
            )
            await execute_ignore(
                self._system.gimbal.set_angles,
                0,
                roll_deg,
                pitch_deg,
                yaw_deg,
                GimbalMode.YAW_LOCK,
                SendMode.ONCE,
            )
            await execute_ignore(self._system.gimbal.release_control, 0)
            log.info("-- Rotated gimbal")
            return

        await self._system.gimbal.take_control(0, ControlMode.PRIMARY)
        await self._system.gimbal.set_angles(
            0, roll_deg, pitch_deg, yaw_deg, GimbalMode.YAW_LOCK, SendMode.ONCE
        )
        await self._system.gimbal.release_control(0)
        log.info("-- Rotated gimbal")

    async def arm(self) -> None:
        """Arm the drone."""
        if not self.status.connected:
            raise NotConnectedException("Cannot arm drone!")

        async for armed in self._system.telemetry.armed():
            if armed:
                self.status.armed = True
                break

            log.info("-- Arming")
            await self._system.action.arm()

    async def takeoff(self) -> None:
        """Take off the drone."""
        if not self.status.connected and self.status.gps_fixed:
            raise NotConnectedException("Drone is not connected!")

        if not self.status.armed:
            raise NotArmedException("Drone is not armed!")

        if self.status.land_state == LandedState.IN_AIR:
            log.info("-- Drone is already in air!")
            return

        start_takeoff = False

        async for land_state in self._system.telemetry.landed_state():
            self.status.land_state = land_state

            if land_state == land_state.IN_AIR:
                log.info("-- Drone is in air!")
                break

            elif land_state == land_state.ON_GROUND and not start_takeoff:
                start_takeoff = True

                log.info("-- Taking off")
                await self._system.action.takeoff()

    async def land(self) -> None:
        log.info("Waiting for the drone to land...")
        start_landing = False
        async for land_state in self._system.telemetry.landed_state():
            if land_state == land_state.ON_GROUND:
                log.info("-- Landing succeeded!")
                break
            elif land_state == land_state.IN_AIR and not start_landing:
                start_landing = True

                log.info("-- Start landing")
                await self._system.action.land()

    async def return_to_launch(self) -> None:
        await self._system.action.return_to_launch()

    async def disconnect(self) -> None:
        for task in self.tasks:
            task.cancel()

        await gather(*self.tasks, return_exceptions=True)

        self.tasks.clear()

        log.info("Drone disconnected")

    async def fly_to_location(
        self,
        goal: NDArray[np.float32],
        speed: float = 10.0,
        gimbal_pitch: int = float("nan"),
        gimbal_yaw: int = float("nan"),
        take_photo: bool = False,
        drone_hold_time: int = 0,
        drone_yaw: float = float("nan"),
        blocking: bool = True,
    ) -> None:
        if self.status.land_state != LandedState.IN_AIR:
            raise DroneNotInAirException(
                "Drone is not in the air! Cannot fly to location!"
            )

        self.status.mission_items = create_mission(
            self.status.position,
            goal,
            speed,
            take_photo,
            drone_hold_time,
            drone_yaw,
        )

        await self.upload_mission()
        await self.start_mission()

        if (not np.isnan(gimbal_pitch) or not np.isnan(gimbal_yaw)) and (
            not np.isclose(gimbal_pitch, self.status.gimbal_angle[1], 0.4)
            or not np.isclose(gimbal_yaw, self.status.gimbal_angle[2], 0.4)
        ):
            await self.rotate_gimbal(
                roll_deg=0.0,
                pitch_deg=gimbal_pitch,
                yaw_deg=gimbal_yaw,
                ignore_errors=True,
            )

        if blocking:
            await self.wait_till_mission_finished()

    async def start_video_stream(self, component_id: int, stream_id: int) -> None:
        await self._system.camera.start_video_streaming(component_id, stream_id)

    async def stop_video_stream(self, component_id: int, stream_id: int) -> None:
        await self._system.camera.stop_video_streaming(component_id, stream_id)

        async for video_stream_update in self._system.camera.video_stream_info():
            if (
                video_stream_update.video_stream_info.status
                == video_stream_update.video_stream_info.status.NOT_RUNNING
            ):
                log.info("--stopped video stream")
                break

    async def download_mission(self) -> None:
        """Download the mission from the drone."""
        self.status.mission_items = await self._system.mission_raw.download_mission()

    async def upload_qgc_mission(self, qgc_plan_path: str) -> None:
        """
        Upload a mission exported from QGroundControl.

        Args:
            qgc_plan_path: Path to the QGC mission plan file.
        """
        import_data = await self._system.mission_raw.import_qgroundcontrol_mission(
            qgc_plan_path
        )
        self.status.mission_items = import_data.mission_items
        self.status.geofence_items = import_data.geofence_items

        await self.upload_mission()
        await self.upload_geofence()

    async def upload_geofence(self) -> bool:
        """Upload the geofence to the drone."""
        if not self.status.connected:
            raise NotConnectedException("Cannot upload geofence!")

        raise NotImplementedError()

    async def upload_mission(self) -> bool:
        """Upload the mission to the drone."""
        if not self.status.connected:
            raise NotConnectedException("Cannot upload mission!")

        if self.status.mission_items is None:
            raise NoMissionException("Cannot upload mission!")

        log.info("-- Uploading new mission")
        attempt = 0
        uploaded = False
        while attempt < self._upload_attempts:
            attempt += 1

            try:
                await self._system.mission_raw.upload_mission(self.status.mission_items)

                uploaded = True
                break
            except MissionRawError as e:
                log.warning(
                    f"-- Failed uploading mission, attempt {attempt}/{self._upload_attempts}: {e}"
                )
                await sleep(5)

        if not uploaded:
            log.error("-- Could not upload mission!")
            return False

        log.info("-- Mission uploaded")
        return True

    async def start_mission(self) -> bool:
        """
        Start a currently loaded mission on the drone.

        Returns:
            True if the mission started successfully, False otherwise.
        """
        if not self.status.connected and self.status.gps_fixed:
            raise NotConnectedException("Drone not connected!")

        if not self.status.armed:
            raise NotArmedException("Drone not armed!")

        if self.status.mission_items is None:
            raise NoMissionException("No mission items!")

        attempt = 0
        started = False
        while attempt < self._start_attempts:
            attempt += 1

            try:
                log.info("Try to start mission")

                await self._system.mission_raw.start_mission()

                started = True
                break
            except MissionRawError as e:
                log.warning(
                    f"-- Failed starting mission, attempt {attempt}/{self._start_attempts}: {e}"
                )
                await sleep(5)

        if not started:
            log.error("-- Could not start mission!")
            return False

        log.info("-- Successfully started mission!")
        return True

    async def wait_till_mission_finished(self) -> None:
        async for progress in self._system.mission_raw.mission_progress():
            log.info(f"Mission status update: {progress.current} / {progress.total}")
            if progress.current >= progress.total:
                log.info("-- Mission finished")
                break

    async def _wait_for_connection(self) -> None:
        """Wait for the drone to connect."""
        self.status.connected = False

        log.info("Waiting for drone to connect...")
        async for state in self._system.core.connection_state():
            if state.is_connected:
                log.info("-- Connected to drone!")
                break

        self.status.connected = True

    async def _wait_for_gps_fix(self) -> None:
        """Wait for the drone to have a GPS fix."""
        self.status.gps_fixed = False

        log.info("Waiting for drone to have a global position estimate...")
        async for health in self._system.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                log.info("-- Global position estimate OK")
                break

        self.status.gps_fixed = True

    async def _telemetry_status_text(self) -> None:
        """Log the status text received from the drone."""
        try:
            async for status_text in self._system.telemetry.status_text():
                log.info(f"Status: {status_text.type}: {status_text.text}")
        except CancelledError:
            return

    async def _telemetry_position(self) -> None:
        """Keep track of the drone's location"""
        try:
            async for location in self._system.telemetry.position():
                self.status.position[0] = location.latitude_deg
                self.status.position[1] = location.longitude_deg
                self.status.position[2] = location.relative_altitude_m
        except CancelledError:
            return

    async def _telemetry_attitude(self) -> None:
        """Keep track of the drone's attitude"""
        try:
            async for attitude in self._system.telemetry.attitude_euler():
                self.status.attitude[0] = attitude.roll_deg
                self.status.attitude[1] = attitude.pitch_deg
                self.status.attitude[2] = attitude.yaw_deg
        except CancelledError:
            return

    async def _gimbal_attitude(self) -> None:
        """Keep track of the gimbal's attitude"""
        try:
            async for gimbal_attitude in self._system.gimbal.attitude():
                self.status.gimbal_angle[0] = (
                    gimbal_attitude.euler_angle_forward.roll_deg
                )
                self.status.gimbal_angle[1] = (
                    gimbal_attitude.euler_angle_forward.pitch_deg
                )
                self.status.gimbal_angle[2] = (
                    gimbal_attitude.euler_angle_forward.yaw_deg
                )
        except CancelledError:
            return

    async def _telemetry_velocity(self) -> None:
        """Keep track of the drone's velocity"""
        try:
            async for velocity in self._system.telemetry.imu():
                pass
                # self.status.velocity[0] = velocity.
                # self.status.velocity[1] = velocity
                # self.status.velocity[2] = velocity
        except CancelledError:
            return

    async def _telemetry_landed_state(self) -> None:
        try:
            async for land_state in self._system.telemetry.landed_state():
                self.status.land_state = land_state
        except CancelledError:
            return
