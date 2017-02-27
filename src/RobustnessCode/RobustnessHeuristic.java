package RobustnessCode;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Scanner;

public class RobustnessHeuristic {
	private HashMap<String, Integer> entityLabeltoIndexMap; 
	private HashMap<String, List<List<String>>> IIRs;
	private double rho;
	private int Robustness = 0;
	
	RobustnessHeuristic(String file, double rhoVal) throws FileNotFoundException{
		rho = rhoVal;
		entityLabeltoIndexMap = new HashMap<String, Integer>();
		IIRs = new HashMap<String, List<List<String>>>();
		File caseFile = new File("OutputFiles/" + file + ".txt");
		Scanner scan = new Scanner(caseFile);
		int eIndex = 0;
		while(scan.hasNext()){
			String exp = scan.nextLine();
			StringBuilder firstEntity = new StringBuilder();
			int index = 0;
			while(exp.charAt(index) != ' '){
				firstEntity.append(exp.charAt(index));
				index ++;
			}
			if(!entityLabeltoIndexMap.containsKey(firstEntity.toString())){
				entityLabeltoIndexMap.put(firstEntity.toString(), eIndex);
				eIndex ++;
			}
			index ++;
			while(exp.charAt(index) != ' '){
				index ++;
			}
			String[] minterms = exp.substring(index + 1, exp.length()).split("   ");
			List<List<String>> dependency = new ArrayList<List<String>>();
			for(String str: minterms){
				dependency.add(Arrays.asList(str.split(" ")));
				for(String entity: str.split(" ")){
					if(!entityLabeltoIndexMap.containsKey(entity)){
						entityLabeltoIndexMap.put(entity, eIndex);
						eIndex ++;
					}
				}
			}
			IIRs.put(firstEntity.toString(), dependency);
		}
		scan.close();
	}
	
	public void computeRobustness(){
		List<String> failedEntities = new ArrayList<String>();
		while(failedEntities.size() <= rho * (double) entityLabeltoIndexMap.size()){
			Robustness ++;
			HashMap<String, List<List<String>>> IIRsForIteration = new HashMap<String, List<List<String>>>();
			List<String> killSetForIteration = new ArrayList<String>();
			int mintermCovNum = 0;
			for(String entity: entityLabeltoIndexMap.keySet()){
				if(failedEntities.contains(entity)) continue;
				
				HashMap<String, List<List<String>>> IIRdummy = new HashMap<String, List<List<String>>>();
				List<String> curFailedEntity = new ArrayList<String>(); 
				curFailedEntity.add(entity);
				int curMintermCovNum = 0;
				
				for(String str: IIRs.keySet())
					if(!failedEntities.contains(str))
						IIRdummy.put(str, IIRs.get(str));
				int start = 0;
				while(start < curFailedEntity.size()){
					String entityToKill = curFailedEntity.get(start);
					List<String> termsToIterate = new ArrayList<String>();
					for(String str: IIRdummy.keySet()) termsToIterate.add(str);
					for(String firstTerm: termsToIterate){
						List<List<String>> mintermToAdd = new ArrayList<List<String>>();
						for(List<String> minterm: IIRdummy.get(firstTerm)){
							if(minterm.contains(entityToKill)) curMintermCovNum ++;
							else mintermToAdd.add(minterm);
						}
						IIRdummy.replace(firstTerm, mintermToAdd);
					}
					for(String str: termsToIterate){
						if(IIRdummy.get(str).size() == 0){
							IIRdummy.remove(str);
							if(!curFailedEntity.contains(str)) curFailedEntity.add(str);
						}
					}
					start ++;
				}
				if(curFailedEntity.size() > killSetForIteration.size()){
					killSetForIteration = new ArrayList<String>();
					IIRsForIteration = new HashMap<String, List<List<String>>>();
					for(String str: curFailedEntity)
						killSetForIteration.add(str);
					for(String str: IIRdummy.keySet())
						IIRsForIteration.put(str,  IIRdummy.get(str));
				}
				
				else if(curFailedEntity.size() == killSetForIteration.size()){
					if(mintermCovNum < curMintermCovNum){
						mintermCovNum = curMintermCovNum;
						killSetForIteration = new ArrayList<String>();
						IIRsForIteration = new HashMap<String, List<List<String>>>();
						for(String str: curFailedEntity)
							killSetForIteration.add(str);
						for(String str: IIRdummy.keySet())
							IIRsForIteration.put(str,  IIRdummy.get(str));
					}
				}
				
			}
			
			for(String str: killSetForIteration)
				failedEntities.add(str);

			IIRs = new HashMap<String, List<List<String>>>();
			for(String str: IIRsForIteration.keySet()){
				IIRs.put(str, IIRsForIteration.get(str));
			}
			
		}
	}
	
	public int getRobustness() { return Robustness;}
	
	public static void main(String[] args) throws FileNotFoundException{
		RobustnessHeuristic Object = new RobustnessHeuristic("case9IIRsAtTimeStep1", 0.2);
		Object.computeRobustness();
		System.out.println("Robustness: " +  Object.getRobustness());
	}
}
