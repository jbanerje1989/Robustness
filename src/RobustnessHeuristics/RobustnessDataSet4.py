import copy;

rho=0.02;#the paramter rho in the robustness problem
print("Robustness");
itr=0;
while rho<=1:
    Robustness=0;
    #Entities in DataSet 1
    EntitySet=['a0','a1','a2','a3','a4','a5','a6','a7','a8','a9','a10','a11','a12',
               'a13','a14','a15','a16','a17','a18','a19','a20','a21','a22','a23',
               'a24','a25','a26','a27','a28','a29','a30','a31','a32','b0','b1',
               'b2','b3','b4','b5','b6','b7','b8','b9','b10','b11','b12','b13',
               'b14','b15','b16','b17','b18','b19'];

    #IDRs of dataset 1
    IDR=[[['a0'],['b1']],
         [['a1'],['b3'],['b11','b16']],
         [['a2'],['b2'],['b8','b17']],
         [['a3'],['b7'],['b13','b18']],
         [['a4'],['b0'],['b10','b19']],
         
         [['b0'],['a3','a5'],['a4','a6']],
         [['b1'],['a0','a7'],['a3','a8']],
         [['b2'],['a2','a9']],
         [['b3'],['a1','a10'],['a4','a11']],
         [['b4'],['a1','a12'],['a3','a13']],
         [['b5'],['a0','a14'],['a3','a15']],
         [['b6'],['a0','a16'],['a1','a17']],
         [['b7'],['a3','a18'],['a4','a19']],
         [['b8'],['a2','a20']],
         [['b9'],['a2','a21']],
         [['b10'],['a3','a22'],['a4','a23']],
         [['b11'],['a1','a24'],['a4','a25']],
         [['b12'],['a1','a26'],['a3','a27']],
         [['b13'],['a3','a28']],
         [['b14'],['a1','a29'],['a4','a30']],
         [['b15'],['a2','a31'],['a3','a32']]];

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
    
