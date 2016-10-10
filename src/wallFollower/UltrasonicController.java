package wallFollower;

public interface UltrasonicController {
	public enum TurnDirection{
		right, left
	}

	public void processUSData(int distance);
		
	public int readUSDistance();
		
}
