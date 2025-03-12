package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.ELEVATOR;
import org.junit.jupiter.api.Test;

public class TestElevator {
  @Test
  public void testElevatorConversion() {
    Angle motorRotations = Rotations.of(8.11403);

    //    Distance height = ELEVATOR.drumRotationsToDistance.times(motorRotations.magnitude());
    Distance height =
        Meters.of(motorRotations.magnitude() * ELEVATOR.drumRotationsToDistance.in(Meters));

    assertEquals(ELEVATOR.upperLimit.in(Inches), height.in(Inches), Inches.of(0.01).magnitude());
  }
}
