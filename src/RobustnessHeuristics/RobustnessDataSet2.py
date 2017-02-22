import copy;

rho=0.02;#the paramter rho in the robustness problem
print("Robustness");
itr=0;
while rho<=1:
    Robustness=0;
    #Entities in DataSet 1
    EntitySet=['a0','a1','a2','a3','a4','a5','a6','a7','a8','a9','a10','a11','a12',
               'a13','a14','a15','a16','a17','a18','a19','a20','a21','a22','a23',
               'a24','a25','a26','a27','b0','b1','b2','b3','b4','b5','b6','b7','b8',
               'b9','b10','b11','b12','b13','b14','b15','b16','b17'];

    #IDRs of dataset 1
    IDR=[[['a0'],['b5'],['b8','b12']],
         [['a1'],['b3'],['b9','b13']],
         [['a2'],['b2']],
         [['a3'],['b0'],['b6','b14']],
         [['a4'],['b1'],['b6','b15']],
         [['a5'],['b10','b16']],
         [['a6'],['b4'],['b10','b17']],
         [['a7'],['b4']],
         
         [['b0'],['a3','a8'],['a4','a9']],
         [['b1'],['a4','a10']],
         [['b2'],['a2','a11'],['a4','a12']],
         [['b3'],['a1','a13']],
         [['b4'],['a6','a14'],['a7','a15']],
         [['b5'],['a0','a16']],
         [['b6'],['a3','a17'],['a4','a18']],
         [['b7'],['a3','a19'],['a5','a20']],
         [['b8'],['a0','a21'],['a3','a22']],
         [['b9'],['a1','a23']],
         [['b10'],['a5','a24'],['a6','a25']],
         [['b11'],['a1','a26'],['a4','a27']]];

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

    print(Robustness);
    rho=rho+0.02;
    

            

    
                    
                            
                    
                    
    

    
        
    


                                           
                        
                




        
    
