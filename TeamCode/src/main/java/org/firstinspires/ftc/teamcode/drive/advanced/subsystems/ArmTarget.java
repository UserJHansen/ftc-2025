package org.firstinspires.ftc.teamcode.drive.advanced.subsystems;

public enum ArmTarget {
    Retracted,
    IntakeExtend,
    Searching,
    Captured, // Flip back the intake and make sure the pixel is secured
    IntakeRetract,
    InternalTransfer, // Transfer to the outtake
    InternalWait,
    OuttakeExtend,
    OuttakeReady,
    Outtake,
    OuttakeRetract;

    static {
        Retracted.timeOut = -1;
        Retracted.next = IntakeExtend;
        Retracted.safeBack = Retracted;

        IntakeExtend.timeOut = 1500;
        IntakeExtend.next = Searching;
        IntakeExtend.safeBack = Retracted;

        Searching.timeOut = -1;
        Searching.next = Captured;
        Searching.safeBack = Retracted; // Maybe we don't actually want to collect

        Captured.timeOut = 500;
        Captured.next = IntakeRetract;
        Captured.safeBack = Searching; // Assume sample wasn't collected properly

        IntakeRetract.timeOut = 1500;
        IntakeRetract.next = InternalTransfer;
        IntakeRetract.safeBack = IntakeExtend; // We want to put the slides back out, sample probably fell

        InternalTransfer.timeOut = 800;
        InternalTransfer.next = InternalWait;
        InternalTransfer.safeBack = Retracted;

        InternalWait.timeOut = -1;
        InternalWait.next = OuttakeExtend;
        InternalWait.safeBack = InternalWait; // Just go forward, we've got it now

        OuttakeExtend.timeOut = 2000;
        OuttakeExtend.next = OuttakeReady;
        OuttakeExtend.safeBack = Retracted; // Pixel probably fell out somewhere

        OuttakeReady.timeOut = -1;
        OuttakeReady.next = Outtake;
        OuttakeReady.safeBack = Outtake; // Just go forward please

        Outtake.timeOut = 1500;
        Outtake.next = OuttakeRetract;
        Outtake.safeBack = Outtake; // Sample still isn't out

        OuttakeRetract.timeOut = 1500;
        OuttakeRetract.next = Retracted;
        OuttakeRetract.safeBack = OuttakeReady; // Lift isn't going down nicely, keep it up
        // Or the sample wasn't deployed
    }

    public double timeOut;
    public ArmTarget next;
    public ArmTarget safeBack;
}
