package org.firstinspires.ftc.teamcode.drive.advanced.subsystems;

public enum ArmTarget {
    Retracted,
    Intake,
    InternalTransferPt1, // Transfer to the outtake
    InternalTransferPt2, // Hover to grab
    InternalWait, // Grab
    OuttakePartialExtend,
    OuttakeExtend,
    OuttakeReady,
    Outtake,
    OuttakeRetract,

    SpecimenSmallLift,
    SpecimenLiftFold,
    SpecimenWait,
    SpecimenReady;

    static {
        Retracted.timeOut = -1;
        Retracted.next = Intake;
        Retracted.safeBack = Retracted;

        Intake.timeOut = -1;
        Intake.next = InternalTransferPt1;
        Intake.safeBack = Retracted;

        InternalTransferPt1.timeOut = -1;
        InternalTransferPt1.next = InternalTransferPt2;
        InternalTransferPt1.safeBack = Intake;

        InternalTransferPt2.timeOut = 300;
        InternalTransferPt2.next = InternalWait;
        InternalTransferPt2.safeBack = InternalTransferPt1;

        InternalWait.timeOut = -1;
        InternalWait.next = OuttakePartialExtend;
        InternalWait.safeBack = InternalTransferPt1; // Didn't grab sample properly

        OuttakePartialExtend.timeOut = 500;
        OuttakePartialExtend.next = OuttakeExtend;
        OuttakePartialExtend.safeBack = Retracted; // Sample probably fell out somewhere

        OuttakeExtend.timeOut = 2000;
        OuttakeExtend.next = OuttakeReady;
        OuttakeExtend.safeBack = Retracted; // Sample probably fell out somewhere

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

        //////////////////////////////
        // Special states for specimen
        //////////////////////////////

        SpecimenSmallLift.timeOut = 500;
        SpecimenSmallLift.next = SpecimenLiftFold;
        SpecimenSmallLift.safeBack = Retracted;

        SpecimenLiftFold.timeOut = 500;
        SpecimenLiftFold.next = SpecimenWait;
        SpecimenLiftFold.safeBack = Outtake;

        SpecimenWait.timeOut = -1;
        SpecimenWait.next = SpecimenReady;
        SpecimenWait.safeBack = Outtake;

        SpecimenReady.timeOut = -1;
        SpecimenReady.next = OuttakeExtend;
        SpecimenReady.safeBack = Outtake;

    }

    public double timeOut;
    public ArmTarget next;
    public ArmTarget safeBack;
}
