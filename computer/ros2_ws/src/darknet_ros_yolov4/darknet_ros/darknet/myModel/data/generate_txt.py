import os 

path_train = os.getcwd() + '/class_complete_train'
path_test = os.getcwd() + '/class_complete_test'
    
path_train_txt=os.getcwd() + '/train.txt'
path_test_txt=os.getcwd() + '/test.txt'

list_train = os.listdir(path_train)  
list_test = os.listdir(path_test) 

# loop on all the images data
for img in list_train:     
    # open both files
    with open(path_train + "/" + img,'r') as firstfile, open(path_train_txt,'a') as secondfile :
        line= path_train + "/" + img + "\n"
        secondfile.write(line)
        
# loop on all the images data
for img in list_test: 
    # open both files
    with open(path_test + "/" + img,'r') as firstfile, open(path_test_txt,'a') as secondfile :
        line= path_test + "/" + img + "\n"
        secondfile.write(line)
