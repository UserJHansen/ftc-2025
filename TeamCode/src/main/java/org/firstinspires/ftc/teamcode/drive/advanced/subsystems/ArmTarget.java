package org.firstinspires.ftc.teamcode.drive.advanced.subsystems;

public enum ArmTarget {
    Caught, // For if the pixel is caught and needs to be outtaked
    Waiting, // Waiting for a new pixel
    ShieldUp, // Stop the motor and move the shield out of the way
    Nab, // Put the Claw down and in
    Primed, // Pixels are held
    ArmOut, // Move the arm out from the intake
    LiftRaised,
    Drop,
    //    Transient state for when the lift is returning to neutral
    Return;

    static {
        Caught.timeOut = 2000;
        Caught.next = Waiting;

        Waiting.timeOut = -1;
        Waiting.next = ShieldUp;

        ShieldUp.timeOut = 1000;
        ShieldUp.next = Nab;

        Nab.timeOut = 500;
        Nab.next = Primed;

        Primed.timeOut = -1;
        Primed.next = ArmOut;

        ArmOut.timeOut = 200;
        ArmOut.next = LiftRaised;

        LiftRaised.timeOut = -1;
        LiftRaised.next = Drop;

        Drop.timeOut = 500;
        Drop.next = Return;

        Return.timeOut = 500;
        Return.next = Waiting;
    }

    public double timeOut;
    public ArmTarget next;
}
