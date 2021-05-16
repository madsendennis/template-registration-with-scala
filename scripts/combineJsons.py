import json
import glob
import os, math, re


path = "/Volumes/storage/Dropbox/Workspace/femur/data/femur/log/"
file1 = f"experiments_semi-big_1.json"
file2 = f"experiments_semi-big_2.json"
# files = [file1, file2]

with open(path+file1) as json_data:
	data1 = json.load(json_data)

with open(path+file2) as json_data:
	data2 = json.load(json_data)

combined = data1 + data2


with open(path+"merged_file.json", 'w') as fp:
    json.dump(combined, fp, indent=4)