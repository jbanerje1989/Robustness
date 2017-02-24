import copy;

rho=0.02;#the paramter rho in the robustness problem
print("Robustness");
itr=0;
while rho<=1:
    Robustness=0;
    #Entities in DataSet 1
    EntitySet=['a0','a1','a2','a3','a4','a5','a6','a7','a8','a9','a10','a11','a12',
               'a13','a14','a15','a16','a17','a18','a19','a20','a21','a22','a23',
               'a24','a25','a26','a27','a28','b0','b1','b2','b3','b4','b5','b6','b7','b8',
               'b9','b10','b11','b12','b13','b14','b15','b16','b17','b18'];

    #IDRs of dataset 1
    IDR=[[['a0'],['b3'],['b10','b15']],
         [['a1'],['b5']],
         [['a2'],['b4'],['b11','b16']],
         [['a3'],['b4'],['b14','b17']],
         [['a4'],['b9'],['b13','b18']],
         [['b0'],['a0','a28'],['a1','a5']],
         [['b1'],['a4','a6']],
         [['b2'],['a0','a7'],['a4','a8']],
         [['b3'],['a0','a9']],
         [['b4'],['a2','a10'],['a3','a11']],
         [['b5'],['a1','a12'],['a2','a13']],
         [['b6'],['a1','a14']],
         [['b7'],['a2','a15'],['a4','a16']],
         [['b8'],['a1','a17'],['a4','a18']],
         [['b9'],['a4','a19']],
         [['b10'],['a2','a20'],['a4','a21']],
         [['b11'],['a0','a22'],['a4','a23']],
         [['b12'],['a0','a24'],['a3','a25']],
         [['b13'],['a2','a26']],
         [['b14'],['a4','a27']]];

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
    

            

    
                    
                            
                    
                    
    

    
        
    


                                           
                        
                




        
    
