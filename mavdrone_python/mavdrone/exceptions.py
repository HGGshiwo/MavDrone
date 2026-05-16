from __future__ import annotations


class NotConnectedException(Exception):
    pass


class NotArmedException(Exception):
    pass


class NoMissionException(Exception):
    pass


class DroneNotInAirException(Exception):
    pass


class NoStreamException(Exception):
    pass


class MissionHasTooLowAltitudeException(Exception):
    pass

class OpenCVNotCompiledWithGstreamerException(Exception):
    pass