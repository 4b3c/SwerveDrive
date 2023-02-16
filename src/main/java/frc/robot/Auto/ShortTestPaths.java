package frc.robot.Auto;

public class ShortTestPaths {

    public static boolean onBoard = false;
    
    public static void driveThenBalance() {
        if (!onBoard) {
            if (DriveTo.goToCoordsPID(-2200, 0, 100)) {
                onBoard = true;
            }
        } else {
            Balance.balanceRobot();
        }
    }
}
