package frc.robot.utils.PID;

public class PIDValues {
    

    private static final PIDValue WristPID = new PIDValue(0.00073, 0.0001, 0.0001);
    private static final PIDValue ElbowPID = new PIDValue(0.00138, 0.000, 0.0001);
    private static final PIDValue ElevatorPID = new PIDValue(0.0005, 0, 0);
    private static final PIDValue PivotPID = new PIDValue(.005, 0, 0);
    
    private static final PIDValue ClimbPID = new PIDValue(0.0005, 0, 0);

    private static final PIDValue DefaultPID = new PIDValue(0.0005, 0, 0);


    public static PIDValue getPID(String name) {
        // System.out.println("PIDValue for name "+name);
        if (name.contains("Wrist")) {
            return WristPID;
        }
        else if (name.contains("Elbow")) {
            return ElbowPID;
        }
        else if (name.contains("Elevator")) {
            return ElevatorPID;
        }
        else if (name.contains("Pivot")) {
            return PivotPID;
        }
        else if (name.contains("Climb") ) {
            return ClimbPID;
        }

        return DefaultPID;
    }

}
