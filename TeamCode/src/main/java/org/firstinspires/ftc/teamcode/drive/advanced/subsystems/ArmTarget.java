package org.firstinspires.ftc.teamcode.drive.advanced.subsystems;

public enum ArmTarget {
    Caught,
    Waiting,
    Nab,
    Primed,
    LiftRaised,
    Drop,
    //    Transient state for when the lift is returning to neutral
    Return;

    static {
        Caught.timeOut = 2000;
        Caught.next = Waiting;

        Waiting.timeOut = -1;
        Waiting.next = Nab;

        Nab.timeOut = 500;
        Nab.next = Primed;

        Primed.timeOut = -1;
        Primed.next = LiftRaised;

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
