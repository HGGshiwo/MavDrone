package nl.wur.mavdrone.communication.utils

import io.dronefleet.mavlink.common.CommandAck
import io.dronefleet.mavlink.common.MavCmd
import io.dronefleet.mavlink.common.MavMissionResult
import io.dronefleet.mavlink.common.MavMissionType
import io.dronefleet.mavlink.common.MavResult
import io.dronefleet.mavlink.common.MissionAck

class StandardResponses {
    companion object {
        /**
         * Command is valid (is supported and has valid parameters), and was executed.
         */
        fun commandAccepted(command: MavCmd): CommandAck {
            return CommandAck.builder()
                .command(command)
                .result(MavResult.MAV_RESULT_ACCEPTED)
                .build()
        }

        /**
         * Command is valid (is supported and has valid parameters), and was executed.
         */
        fun commandAccepted(
            command: MavCmd,
            targetSystemID: Int,
            targetComponentID: Int
        ): CommandAck {
            return CommandAck.builder()
                .command(command)
                .result(MavResult.MAV_RESULT_ACCEPTED)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .build()
        }

        /**
         * Command is valid, but cannot be executed at this time. This is used to indicate a
         * problem that should be fixed just by waiting (e.g. a state machine is busy, can't arm
         * because have not got GPS lock, etc.). Retrying later should work.
         */
        fun commandTemporarilyRejected(command: MavCmd): CommandAck {
            return CommandAck.builder()
                .command(command)
                .result(MavResult.MAV_RESULT_TEMPORARILY_REJECTED)
                .build()
        }

        /**
         * Command is valid, but cannot be executed at this time. This is used to indicate a
         * problem that should be fixed just by waiting (e.g. a state machine is busy, can't arm
         * because have not got GPS lock, etc.). Retrying later should work.
         */
        fun commandTemporarilyRejected(
            command: MavCmd,
            targetSystemID: Int,
            targetComponentID: Int
        ): CommandAck {
            return CommandAck.builder()
                .command(command)
                .result(MavResult.MAV_RESULT_TEMPORARILY_REJECTED)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .build()
        }

        /**
         * Command is invalid (is supported but has invalid parameters). Retrying same command and
         * parameters will not work.
         */
        fun commandDenied(command: MavCmd): CommandAck {
            return CommandAck.builder()
                .command(command)
                .result(MavResult.MAV_RESULT_DENIED)
                .build()
        }

        /**
         * Command is invalid (is supported but has invalid parameters). Retrying same command and
         * parameters will not work.
         */
        fun commandDenied(
            command: MavCmd,
            targetSystemID: Int,
            targetComponentID: Int
        ): CommandAck {
            return CommandAck.builder()
                .command(command)
                .result(MavResult.MAV_RESULT_DENIED)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .build()
        }

        /**
         * Command is not supported (unknown).
         */
        fun commandUnsupported(command: MavCmd): CommandAck {
            return CommandAck.builder()
                .command(command)
                .result(MavResult.MAV_RESULT_UNSUPPORTED)
                .build()
        }

        /**
         * Command is not supported (unknown).
         */
        fun commandUnsupported(
            command: MavCmd,
            targetSystemID: Int,
            targetComponentID: Int
        ): CommandAck {
            return CommandAck.builder()
                .command(command)
                .result(MavResult.MAV_RESULT_UNSUPPORTED)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .build()
        }

        /**
         * Command is valid, but execution has failed. This is used to indicate any non-temporary
         * or unexpected problem, i.e. any problem that must be fixed before the command can
         * succeed/be retried. For example, attempting to write a file when out of memory,
         * attempting to arm when sensors are not calibrated, etc.
         */
        fun commandFailed(command: MavCmd): CommandAck {
            return CommandAck.builder()
                .command(command)
                .result(MavResult.MAV_RESULT_FAILED)
                .build()
        }

        /**
         * Command is valid, but execution has failed. This is used to indicate any non-temporary
         * or unexpected problem, i.e. any problem that must be fixed before the command can
         * succeed/be retried. For example, attempting to write a file when out of memory,
         * attempting to arm when sensors are not calibrated, etc.
         */
        fun commandFailed(
            command: MavCmd,
            targetSystemID: Int,
            targetComponentID: Int
        ): CommandAck {
            return CommandAck.builder()
                .command(command)
                .result(MavResult.MAV_RESULT_FAILED)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .build()
        }

        /**
         * Command is valid and is being executed. This will be followed by further progress
         * updates, i.e. the component may send further COMMAND_ACK messages with result
         * MAV_RESULT_IN_PROGRESS (at a rate decided by the implementation), and must terminate by
         * sending a COMMAND_ACK message with final result of the operation. The
         * COMMAND_ACK.progress field can be used to indicate the progress of the operation.
         */
        fun commandInProgress(command: MavCmd): CommandAck {
            return CommandAck.builder()
                .command(command)
                .result(MavResult.MAV_RESULT_IN_PROGRESS)
                .build()
        }

        /**
         * Command is valid and is being executed. This will be followed by further progress
         * updates, i.e. the component may send further COMMAND_ACK messages with result
         * MAV_RESULT_IN_PROGRESS (at a rate decided by the implementation), and must terminate by
         * sending a COMMAND_ACK message with final result of the operation. The
         * COMMAND_ACK.progress field can be used to indicate the progress of the operation.
         */
        fun commandInProgress(
            command: MavCmd,
            targetSystemID: Int,
            targetComponentID: Int
        ): CommandAck {
            return CommandAck.builder()
                .command(command)
                .result(MavResult.MAV_RESULT_IN_PROGRESS)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .build()
        }

        /**
         * Command is valid and is being executed. This will be followed by further progress
         * updates, i.e. the component may send further COMMAND_ACK messages with result
         * MAV_RESULT_IN_PROGRESS (at a rate decided by the implementation), and must terminate by
         * sending a COMMAND_ACK message with final result of the operation. The
         * COMMAND_ACK.progress field can be used to indicate the progress of the operation.
         */
        fun commandInProgress(command: MavCmd, progress: Int): CommandAck {
            return CommandAck.builder()
                .command(command)
                .result(MavResult.MAV_RESULT_IN_PROGRESS)
                .progress(progress)
                .build()
        }

        /**
         * Command is valid and is being executed. This will be followed by further progress
         * updates, i.e. the component may send further COMMAND_ACK messages with result
         * MAV_RESULT_IN_PROGRESS (at a rate decided by the implementation), and must terminate by
         * sending a COMMAND_ACK message with final result of the operation. The
         * COMMAND_ACK.progress field can be used to indicate the progress of the operation.
         */
        fun commandInProgress(
            command: MavCmd,
            progress: Int,
            targetSystemID: Int,
            targetComponentID: Int
        ): CommandAck {
            return CommandAck.builder()
                .command(command)
                .result(MavResult.MAV_RESULT_IN_PROGRESS)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .progress(progress)
                .build()
        }

        /**
         * Command has been cancelled (as a result of receiving a COMMAND_CANCEL message).
         */
        fun commandCanceled(command: MavCmd): CommandAck {
            return CommandAck.builder()
                .command(command)
                .result(MavResult.MAV_RESULT_CANCELLED)
                .build()
        }

        /**
         * Command has been cancelled (as a result of receiving a COMMAND_CANCEL message).
         */
        fun commandCanceled(
            command: MavCmd,
            targetSystemID: Int,
            targetComponentID: Int
        ): CommandAck {
            return CommandAck.builder()
                .command(command)
                .result(MavResult.MAV_RESULT_CANCELLED)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .build()
        }

        fun missionAccepted(): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_ACCEPTED)
                .build()
        }

        fun missionAccepted(missionType: MavMissionType?): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_ACCEPTED)
                .missionType(missionType)
                .build()
        }

        fun missionAccepted(targetSystemID: Int, targetComponentID: Int): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_ACCEPTED)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .build()
        }

        fun missionAccepted(
            missionType: MavMissionType?,
            targetSystemID: Int,
            targetComponentID: Int
        ): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_ACCEPTED)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .missionType(missionType)
                .build()
        }

        fun missionError(): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_ERROR)
                .build()
        }

        fun missionError(missionType: MavMissionType?): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_ERROR)
                .missionType(missionType)
                .build()
        }

        fun missionError(targetSystemID: Int, targetComponentID: Int): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_ERROR)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .build()
        }

        fun missionError(
            missionType: MavMissionType?,
            targetSystemID: Int,
            targetComponentID: Int
        ): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_ERROR)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .missionType(missionType)
                .build()
        }

        fun missionUnsupportedFrame(): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_UNSUPPORTED_FRAME)
                .build()
        }

        fun missionUnsupportedFrame(missionType: MavMissionType?): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_UNSUPPORTED_FRAME)
                .missionType(missionType)
                .build()
        }

        fun missionUnsupportedFrame(targetSystemID: Int, targetComponentID: Int): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_UNSUPPORTED_FRAME)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .build()
        }

        fun missionUnsupportedFrame(
            missionType: MavMissionType?,
            targetSystemID: Int,
            targetComponentID: Int
        ): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_UNSUPPORTED_FRAME)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .missionType(missionType)
                .build()
        }

        fun missionUnsupported(): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_UNSUPPORTED)
                .build()
        }

        fun missionUnsupported(missionType: MavMissionType?): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_UNSUPPORTED)
                .missionType(missionType)
                .build()
        }

        fun missionUnsupported(targetSystemID: Int, targetComponentID: Int): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_UNSUPPORTED)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .build()
        }

        fun missionUnsupported(
            missionType: MavMissionType?,
            targetSystemID: Int,
            targetComponentID: Int
        ): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_UNSUPPORTED)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .missionType(missionType)
                .build()
        }

        fun missionNoSpace(): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_NO_SPACE)
                .build()
        }

        fun missionNoSpace(missionType: MavMissionType?): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_NO_SPACE)
                .missionType(missionType)
                .build()
        }

        fun missionNoSpace(targetSystemID: Int, targetComponentID: Int): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_NO_SPACE)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .build()
        }

        fun missionNoSpace(
            missionType: MavMissionType?,
            targetSystemID: Int,
            targetComponentID: Int
        ): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_NO_SPACE)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .missionType(missionType)
                .build()
        }

        fun missionInvalid(): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID)
                .build()
        }

        fun missionInvalid(missionType: MavMissionType?): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID)
                .missionType(missionType)
                .build()
        }

        fun missionInvalid(targetSystemID: Int, targetComponentID: Int): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .build()
        }

        fun missionInvalid(
            missionType: MavMissionType?,
            targetSystemID: Int,
            targetComponentID: Int
        ): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .missionType(missionType)
                .build()
        }

        fun missionInvalidParam1(): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM1)
                .build()
        }

        fun missionInvalidParam1(missionType: MavMissionType?): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM1)
                .missionType(missionType)
                .build()
        }

        fun missionInvalidParam1(targetSystemID: Int, targetComponentID: Int): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM1)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .build()
        }

        fun missionInvalidParam1(
            missionType: MavMissionType?,
            targetSystemID: Int,
            targetComponentID: Int
        ): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM1)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .missionType(missionType)
                .build()
        }

        fun missionInvalidParam2(): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM2)
                .build()
        }

        fun missionInvalidParam2(missionType: MavMissionType?): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM2)
                .missionType(missionType)
                .build()
        }

        fun missionInvalidParam2(targetSystemID: Int, targetComponentID: Int): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM2)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .build()
        }

        fun missionInvalidParam2(
            missionType: MavMissionType?,
            targetSystemID: Int,
            targetComponentID: Int
        ): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM2)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .missionType(missionType)
                .build()
        }

        fun missionInvalidParam3(): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM3)
                .build()
        }

        fun missionInvalidParam3(missionType: MavMissionType?): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM3)
                .missionType(missionType)
                .build()
        }

        fun missionInvalidParam3(targetSystemID: Int, targetComponentID: Int): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM3)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .build()
        }

        fun missionInvalidParam3(
            missionType: MavMissionType?,
            targetSystemID: Int,
            targetComponentID: Int
        ): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM3)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .missionType(missionType)
                .build()
        }

        fun missionInvalidParam4(): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM4)
                .build()
        }

        fun missionInvalidParam4(missionType: MavMissionType?): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM4)
                .missionType(missionType)
                .build()
        }

        fun missionInvalidParam4(targetSystemID: Int, targetComponentID: Int): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM4)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .build()
        }

        fun missionInvalidParam4(
            missionType: MavMissionType?,
            targetSystemID: Int,
            targetComponentID: Int
        ): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM4)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .missionType(missionType)
                .build()
        }

        fun missionInvalidParam5X(): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM5_X)
                .build()
        }

        fun missionInvalidParam5X(missionType: MavMissionType?): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM5_X)
                .missionType(missionType)
                .build()
        }

        fun missionInvalidParam5X(targetSystemID: Int, targetComponentID: Int): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM5_X)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .build()
        }

        fun missionInvalidParam5X(
            missionType: MavMissionType?,
            targetSystemID: Int,
            targetComponentID: Int
        ): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM5_X)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .missionType(missionType)
                .build()
        }

        fun missionInvalidParam6Y(): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM6_Y)
                .build()
        }

        fun missionInvalidParam6Y(missionType: MavMissionType?): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM6_Y)
                .missionType(missionType)
                .build()
        }

        fun missionInvalidParam6Y(targetSystemID: Int, targetComponentID: Int): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM6_Y)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .build()
        }

        fun missionInvalidParam6Y(
            missionType: MavMissionType?,
            targetSystemID: Int,
            targetComponentID: Int
        ): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM6_Y)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .missionType(missionType)
                .build()
        }

        fun missionInvalidParam7(): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM7)
                .build()
        }

        fun missionInvalidParam7(missionType: MavMissionType?): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM7)
                .missionType(missionType)
                .build()
        }

        fun missionInvalidParam7(targetSystemID: Int, targetComponentID: Int): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM7)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .build()
        }

        fun missionInvalidParam7(
            missionType: MavMissionType?,
            targetSystemID: Int,
            targetComponentID: Int
        ): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_PARAM7)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .missionType(missionType)
                .build()
        }

        fun missionInvalidSequence(): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_SEQUENCE)
                .build()
        }

        fun missionInvalidSequence(missionType: MavMissionType?): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_SEQUENCE)
                .missionType(missionType)
                .build()
        }

        fun missionInvalidSequence(targetSystemID: Int, targetComponentID: Int): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_SEQUENCE)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .build()
        }

        fun missionInvalidSequence(
            missionType: MavMissionType?,
            targetSystemID: Int,
            targetComponentID: Int
        ): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_INVALID_SEQUENCE)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .missionType(missionType)
                .build()
        }

        fun missionDenied(): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_DENIED)
                .build()
        }

        fun missionDenied(missionType: MavMissionType?): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_DENIED)
                .missionType(missionType)
                .build()
        }

        fun missionDenied(targetSystemID: Int, targetComponentID: Int): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_DENIED)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .build()
        }

        fun missionDenied(
            missionType: MavMissionType?,
            targetSystemID: Int,
            targetComponentID: Int
        ): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_DENIED)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .missionType(missionType)
                .build()
        }

        fun missionOperationCancelled(): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_OPERATION_CANCELLED)
                .build()
        }

        fun missionOperationCancelled(missionType: MavMissionType?): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_OPERATION_CANCELLED)
                .missionType(missionType)
                .build()
        }

        fun missionOperationCancelled(targetSystemID: Int, targetComponentID: Int): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_OPERATION_CANCELLED)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .build()
        }

        fun missionOperationCancelled(
            missionType: MavMissionType?,
            targetSystemID: Int,
            targetComponentID: Int
        ): MissionAck {
            return MissionAck.builder()
                .type(MavMissionResult.MAV_MISSION_OPERATION_CANCELLED)
                .targetSystem(targetSystemID)
                .targetComponent(targetComponentID)
                .missionType(missionType)
                .build()
        }
    }
}
