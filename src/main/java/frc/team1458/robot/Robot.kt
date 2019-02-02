package frc.team1458.robot

import frc.team1458.lib.actuator.SmartMotor
import frc.team1458.lib.core.BaseRobot
import frc.team1458.lib.drive.TankDrive
import frc.team1458.lib.util.flow.delay

class  Robot : BaseRobot() {

    val oi: OI = OI()
    val dt: TankDrive = TankDrive(
        leftMaster = SmartMotor.CANtalonSRX(10), // TODO change IDs to real IDs
        rightMaster = SmartMotor.CANtalonSRX(11),
        leftMotors = arrayOf(SmartMotor.CANtalonSRX(12)),
        rightMotors = arrayOf(SmartMotor.CANtalonSRX(13))
    )

    val drivetrainInverted: Boolean = false
    val testing: String = "testing"


    // Runs when the robot is setup (once)
    override fun robotSetup() {
        println("Setup running...")

    }

    // Runs when auto mode is enabled (put actual autonomous code in the while loop)
    override fun runAuto() {
        println("Warning: Sandstorm")

        while (isAutonomous && isEnabled) {
            delay(5)
        }
    }

    // Runs when teleoperated mode is enabled (runs once)
    override fun teleopInit() {

    }

    // Runs when the robot receives commands from the diver station (about 50 times a second)
    override fun teleopPeriodic() {

    }

    // Runs when test mode is being ran, not of any concern right now probably
    override fun runTest() {

    }

    // Runs when the robot is disabled (runs once)
    override fun robotDisabled() {

    }

    // Runs while the robot is disabled (many times a second whiled disabled)
    override fun disabledPeriodic() {

    }
}
