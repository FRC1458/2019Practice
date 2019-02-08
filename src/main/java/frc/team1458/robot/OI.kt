package frc.team1458.robot

import frc.team1458.lib.input.FlightStick

class OI {
    val leftStick: FlightStick = FlightStick.flightStick(1)
    val rightStick: FlightStick = FlightStick.flightStick(0)

    var steerAxis = leftStick.rollAxis.scale(0.5) //controlBoard.xbox.rightX.scale(0.35) //
    var throttleAxis = rightStick.pitchAxis.inverted

    var slowDownButton = rightStick.trigger
    var exampleButton = rightStick.trigger

    var intakeIn = leftStick.getButton(5)
    var intakeOut = leftStick.getButton(2)
    var mastersolenoid = rightStick.getButton()
}