package frc.robot.constants;

import static frc.robot.constants.ROBOT.ROBOT_ID;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class TestRobotConstants {
  @Test
  public void testRobotSerial() {
    String serialNumber = "032381FB";

    var robotID = ROBOT_ID.fromSerial(serialNumber);

    assertEquals(ROBOT_ID.V2, robotID);
  }
}
