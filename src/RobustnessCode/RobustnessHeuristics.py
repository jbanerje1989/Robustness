import os;
import copy;
os.chdir('/Users/joydeepbanerjee/Dropbox/papers/WorkUnderProgress/RobustnessJournal/Paper Code/Robustness/OutputFiles/');
InputFile = open('case30IIRsAtTimeStep1.txt','r');
EntitySetOrg = InputFile.readline().strip().split(" "); # Set of all failed entities
for vals in InputFile.readline().strip().split(" ") :
    EntitySetOrg.append(vals);
IDROrg = []; # Dependencies of the entities

while True:
    line = InputFile.readline();
    if line == '': break;
    index = 0;
    leftSideEntity = [];
    minterms= [];
    for val in line.strip().split("   "):
        if index == 0:
            val2 = val.split(" ");
            leftSideEntity.append(val2[0]);
            minterms.append(leftSideEntity);
            minterms.append(val2[2].split(" "));
        else :
            minterms.append(val.split(" "))
        index = index + 1;
    IDROrg.append(minterms); 

os.chdir('/Users/joydeepbanerjee/Dropbox/papers/WorkUnderProgress/RobustnessJournal/Paper Code/Robustness/src/RobustnessCode/');
rho=0.02; # the paramter rho in the robustness problem
itr=0;
while rho <= 1:
    Robustness=0;
    EntitySet = copy.deepcopy(EntitySetOrg);
    IDR = copy.deepcopy(IDROrg);
    FailedEntities=[];#initialize the set of failed entities
    #kill an entity till the desired number of entities are not killed
    while len(FailedEntities) < rho*(len(EntitySet)):
        killSet=[];
        IDRdummy=[];
        mintermCovNum=[];
        unfailedEntities=copy.deepcopy(EntitySet);
        
        for i in range(len(FailedEntities)):
            unfailedEntities.remove(FailedEntities[i]);
        
        #create IDR corresponding to each unfailed entity for modification
        for i in range(len(unfailedEntities)):
            killSet.append([unfailedEntities[i]]);
            IDRdummy.append(copy.deepcopy(IDR));
            mintermCovNum.append(0);
            for j in range(len(IDR)):
                if IDRdummy[i][j][0][0]==killSet[i][0]:
                    IDRdummy[i].remove(IDR[j]);
                    break;

        #compute kill set and minterm coverage number for all unfailed entities
        for i in range(len(killSet)):
            start=0;
            IDRcheck=copy.deepcopy(IDRdummy[i]);
            
            while(start < len(killSet[i])):
                
                for j in range(len(IDRcheck)):
                    
                    for k in range(len(IDRcheck[j])-1):
                        
                        for p in range(len(IDRcheck[j][k+1])):
                            
                            if IDRcheck[j][k+1][p]==killSet[i][start]:
                                IDRdummy[i][j].remove(IDRcheck[j][k+1]);
                                mintermCovNum[i]=mintermCovNum[i]+1;
                                break;


                IDRcheck=copy.deepcopy(IDRdummy[i]);
                for j in range(len(IDRdummy[i])):
                    if len(IDRdummy[i][j])==1:
                        killSet[i].append(IDRdummy[i][j][0][0]);
                        IDRcheck.remove(IDRdummy[i][j]);


                     
                start=start+1;
                IDRdummy[i]=copy.deepcopy(IDRcheck);


        x=0;#set the entity to be killed to first entity
            
        #select the entity to be killed
        for i in range(1,len(killSet)):
            if len(killSet[i])>len(killSet[x]):
                x=i;
            elif len(killSet[i])==len(killSet[x]):
                if mintermCovNum[i]>mintermCovNum[x]:
                    x=i;
            
        #refine the IDR based on the kill set of the killed entity
        IDR=copy.deepcopy(IDRdummy[x]);

        #add the kill set to the failure set
        for i in range(len(killSet[x])):
            FailedEntities.append(killSet[x][i]);

        Robustness=Robustness+1;

    print("rho: " + str(rho) + "   Robustness: " + str(Robustness));
    rho=rho+0.02;
    

            

    
                    
                            
                    
                    
    

    
        
    


                                           
                        
                




        
    
