package frc.robot.subsystems.chassis

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.swerve.SwerveDrivetrain
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import com.ctre.phoenix6.swerve.SwerveRequest
import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.robot.classes.TunerConstants
import java.util.function.Supplier

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
class CommandSwerveDrivetrain(
    driveTrainConstants: SwerveDrivetrainConstants?,
    aprilTagFieldLayout: AprilTagFieldLayout?,
    vararg modules: SwerveModuleConstants?
) : SwerveDrivetrain(driveTrainConstants, modules),
    Subsystem {
    private var m_simNotifier: Notifier? = null
    private var m_lastSimTime = 0.0

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private val BlueAlliancePerspectiveRotation: Rotation2d = Rotation2d.fromDegrees(0.0)

    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private val RedAlliancePerspectiveRotation: Rotation2d = Rotation2d.fromDegrees(180.0)

    /* Keep track if we've ever applied the operator perspective before or not */
    private var hasAppliedOperatorPerspective = false

    private val autoRequest: SwerveRequest.ApplyChassisSpeeds = ApplyChassisSpeeds()

    private val translationCharacterization: SwerveRequest.SysIdSwerveTranslation = SysIdSwerveTranslation()
    private val rotationCharacterization: SwerveRequest.SysIdSwerveRotation = SysIdSwerveRotation()
    private val steerCharacterization: SwerveRequest.SysIdSwerveSteerGains = SysIdSwerveSteerGains()

    /* Use one of these sysidroutines for your particular test */
    var sysIdRoutineTranslation: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            Units.Volts.of(4.0),
            Units.Seconds.of(6.0)
        ) { state: SysIdRoutineLog.State -> SignalLogger.writeString("state", state.toString()) },
        Mechanism(
            { volts: Voltage? -> setControl(translationCharacterization.withVolts(volts)) }, null, this
        )
    )

    val sysIdRoutineRotation: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            Units.Volts.of(4.0),
            null
        ) { state: SysIdRoutineLog.State -> SignalLogger.writeString("state", state.toString()) },
        Mechanism(
            { volts: Voltage? -> setControl(rotationCharacterization.withVolts(volts)) }, null, this
        )
    )
    val sysIdRoutineSteer: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            Units.Volts.of(7.0),
            null
        ) { state: SysIdRoutineLog.State -> SignalLogger.writeString("state", state.toString()) },
        Mechanism(
            { volts: Voltage? -> setControl(steerCharacterization.withVolts(volts)) }, null, this
        )
    )

    init {
        if (Utils.isSimulation()) {
            startSimThread()
        }

        configurePathPlanner()
    }

    fun applyRequest(requestSupplier: Supplier<SwerveRequest?>): Command {
        return run { this.setControl(requestSupplier.get()) }
    }

    fun sysIdRotation(): Command {
        return Commands.sequence(
            Commands.runOnce(SignalLogger::start),
            sysIdRoutineRotation.quasistatic(SysIdRoutine.Direction.kForward),
            sysIdRoutineRotation.quasistatic(SysIdRoutine.Direction.kReverse),
            sysIdRoutineRotation.dynamic(SysIdRoutine.Direction.kForward),
            sysIdRoutineRotation.dynamic(SysIdRoutine.Direction.kReverse),
            Commands.runOnce(SignalLogger::stop)
        )
    }

    fun sysIdTranslation(): Command {
        return Commands.sequence(
            Commands.runOnce(SignalLogger::start),
            sysIdRoutineTranslation.quasistatic(SysIdRoutine.Direction.kForward),
            sysIdRoutineTranslation.quasistatic(SysIdRoutine.Direction.kReverse),
            sysIdRoutineTranslation.dynamic(SysIdRoutine.Direction.kForward),
            sysIdRoutineTranslation.dynamic(SysIdRoutine.Direction.kReverse),
            Commands.runOnce(SignalLogger::stop)
        )
    }

    fun sysIdSteer(): Command {
        return Commands.sequence(
            Commands.runOnce(SignalLogger::start),
            sysIdRoutineSteer.quasistatic(SysIdRoutine.Direction.kForward),
            sysIdRoutineSteer.quasistatic(SysIdRoutine.Direction.kReverse),
            sysIdRoutineSteer.dynamic(SysIdRoutine.Direction.kForward),
            sysIdRoutineSteer.dynamic(SysIdRoutine.Direction.kReverse),
            Commands.runOnce(SignalLogger::stop)
        )
    }

    private fun startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds()

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier =
            Notifier {
                val currentTime: Double = Utils.getCurrentTimeSeconds()
                val deltaTime = currentTime - m_lastSimTime
                m_lastSimTime = currentTime

                /* use the measured time delta, get battery voltage from WPILib */
                updateSimState(deltaTime, RobotController.getBatteryVoltage())
            }
        m_simNotifier!!.startPeriodic(kSimLoopPeriod)
    }

    private fun configurePathPlanner() {
        var driveBaseRadius = 0.0
        for (moduleLocation in m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm())
        }

        AutoBuilder.configureHolonomic(
            { this.getState().Pose },  // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            { this.estimatedVelocitey },
            { speeds -> this.setControl(autoRequest.withSpeeds(speeds)) },  // Consumer of ChassisSpeeds to drive the robot
            HolonomicPathFollowerConfig(
                PIDConstants(10, 0, 0),
                PIDConstants(10, 0, 0),
                TunerConstants.kSpeedAt12VoltsMps,
                driveBaseRadius,
                ReplanningConfig()
            ),  // mirror when on red
            { DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red },
            this
        ) // Subsystem for requirements
    }

    val estimatedVelocitey: ChassisSpeeds
        get() = m_kinematics.toChassisSpeeds(getState().ModuleStates)

    override fun periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance()
                .ifPresent { allianceColor: Alliance ->
                    this.setOperatorPerspectiveForward(
                        if (allianceColor == Alliance.Red)
                            RedAlliancePerspectiveRotation
                        else
                            BlueAlliancePerspectiveRotation
                    )
                    hasAppliedOperatorPerspective = true
                }
        }
        Logger.recordOutput("Estimated Pose", this.getState().Pose)
        Logger.recordOutput("Estimated Velocity", this.getState().speeds)
    }

    companion object {
        private const val kSimLoopPeriod = 0.005 // 5 ms
    }
}