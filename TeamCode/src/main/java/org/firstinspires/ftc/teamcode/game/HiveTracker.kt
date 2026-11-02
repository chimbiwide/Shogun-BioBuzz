package org.firstinspires.ftc.teamcode.game

import com.pedropathing.math.Pose
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.RobotConstants

/**
 * Remembers which of our HIVE's CELLS points up. That is the only CELL we can shoot into.
 *
 * Every time the HIVE tips, the other CELL points up. In teleop the operator tells us (dpad).
 * The webcam can also give its opinion: cameraUpCell.
 */
class HiveTracker(val alliance: Alliance) {
    /** The CELL we aim at. */
    var upCell = Field.startingUpCell(alliance)

    /**
     * What the webcam sees as up right now (it must say the same thing for CAMERA_CONFIRM_SECONDS first).
     * null = it cannot tell right now. The tags face down, tilted toward the side the up CELL opens to,
     * so the webcam sees them best from there.
     */
    var cameraUpCell: HiveCell? = null
        private set

    private var lastAnswer: HiveCell? = null
    private val sameAnswerTimer = ElapsedTime()

    /**
     * Give this the webcam's answer every loop (null if it cannot see our tags).
     * We only believe an answer once the camera has said it for CAMERA_CONFIRM_SECONDS in a row.
     */
    fun updateFromCamera(answer: HiveCell?) {
        if (answer == null || answer != lastAnswer) {
            // No answer, or a different answer: forget the old one and start timing again.
            cameraUpCell = null
            lastAnswer = answer
            sameAnswerTimer.reset()
            return
        }
        if (sameAnswerTimer.seconds() > RobotConstants.CAMERA_CONFIRM_SECONDS) {
            cameraUpCell = answer
        }
    }

    /** The spot on the field to aim at: the opening of the up CELL. */
    fun target(): Pose {
        return Field.openingPosition(alliance, upCell)
    }
}
