package frc.robot.subsystems.chassis

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.wpilibj.XboxController

class StandardDrive (
    private val controller: XboxController
) {
    private val normalSpeed: Double = 0.5
    private val slowSpeed: Double = 0.25
    private val fastSpeed: Double = 1.0 //Should always be kept at 1

    val standardDrive: SwerveRequest() {

    }
}