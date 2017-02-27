package RobustnessCode;

import java.io.FileNotFoundException;

public class DriverRobustness {
	public static void main(String[] args) throws FileNotFoundException{
		String fileName = "case9IIRsAtTimeStep1";
		double rhoVal = 0.5;
		
		System.out.println("ILP Output");
		long startTime = System.currentTimeMillis();
		RobustnessILP Object = new RobustnessILP(fileName, rhoVal);
		Object.optimize();
		long endTime   = System.currentTimeMillis();
		long totalTime = endTime - startTime;
		System.out.println("==============================================");	
		Object.printReport();
		System.out.println("Total Time: " + (double) totalTime / 1000.0 + " seconds"); 
		System.out.println("==============================================");	
		
		System.out.println("Heuristic Output");
		startTime = System.currentTimeMillis();
		RobustnessHeuristic Object2 = new RobustnessHeuristic(fileName, rhoVal);
		Object2.computeRobustness();
		endTime   = System.currentTimeMillis();
		totalTime = endTime - startTime;
		System.out.println("Components Killed Initially: " +  Object2.getRobustness());
		System.out.println("Total Time: " + (double) totalTime / 1000.0 + " seconds"); 
		System.out.println("==============================================");	
	}
}
