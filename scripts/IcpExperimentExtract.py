import matplotlib
import matplotlib.pyplot as plt

# import tkinter
import numpy as np
import json, os, math, re
import seaborn as sns
import pandas as pd
from datetime import datetime

matplotlib.rc('xtick', labelsize=20)     
matplotlib.rc('ytick', labelsize=20)

sns.set(style="whitegrid")
# sns.set(style="darkgrid")
sns.color_palette("pastel") # deep, muted, pastel, bright, dark, and colorblind
# sns.set_palette("husl", 8)
# sns.set_palette("bright", 20)

datetimeformat = "%Y-%m-%d %H:%M:%S"

# sns.palplot(sns.color_palette("husl", 8))

print("GiNGR vs P-GiNGR (ICP and CPD)")

path = "/Volumes/storage/Dropbox/Workspace/femur/data/femur/log/" #thesis_base_50_experiment/"

allfiles = []

filesname = 'experiments_small.json'


with open(path+filesname) as json_data:
	data = json.load(json_data)

targets = {}
for x in range(0, len(data)):
	target = data[x]['targetPath']
	tname = int(os.path.basename(target).replace('.stl', ''))
	targets[tname] = target

for x in targets.keys():
	print(x, targets[x], type(x))

# measure = 'avg'
measure = 'hausdorff'
# measure = 'dice'

print("Number of targets: {}".format(len(targets.keys())))
print(targets.keys())
							
d = {}
icpTotal = []
samICPTotal = []
samCPDTotal = []
cpdTotal = []
rangeStart = 45
rangeEnd = 50
rangeName = f"{rangeStart}until{rangeEnd}"
indexSeq = range(rangeStart, rangeEnd) # 0, 15, 22, 34
limitPlot = True
for key in sorted(targets.keys()):
	# if int(key) in indexSeq or not limitPlot:
	icp = []
	samICP = []
	samCPD = []
	cpd = []
	for x in range(0, len(data)):
		item = data[x]
		if item['targetPath'] == targets[key]:
			# if item['sampling'][measure] < 1.5:
			icp.append(item['icp'][measure])
			samICP.append(item['samplingICP'][measure])
			cpd.append(item['cpd'][measure])
			samCPD.append(item['samplingCPD'][measure])

		# if item['sampling'][measure] > 1.8:
			# print("\n")
			# for k in item.keys():
				# print('{}: {}'.format(k, item[k]))
	print('Target: {} numOfMeasures: {}'.format(key, len(icp)))
	if(len(icp) > 0):
		icpTotal = icpTotal + icp
		samICPTotal = samICPTotal + samICP
		cpdTotal = cpdTotal + cpd
		samCPDTotal = samCPDTotal + samCPD

		if(int(key) in indexSeq or not limitPlot):
			d[str(key)+"-"+"I"] = icp
			d[str(key)+"-"+"PI"] = samICP
			d[str(key)+"-"+"C"] = cpd
			d[str(key)+"-"+"PC"] = samCPD
			# d[str(key)+"-"+"MH-H"] = samHau


# d['A-I'] = icpTotal
# d['A-PI'] = samICPTotal
# d['A-C'] = cpdTotal
# d['A-PC'] = samCPDTotal
# append None so all columns are of same length
maxlen = 0
for val in d.values():
	if len(val) > maxlen:
		maxlen = len(val)
print("maxlen: {}".format(maxlen))
for key in d.keys():
	val = d[key]
	loclen = len(val)
	d[key] = d[key] + ([None] * int(maxlen-loclen))

# d = {'ICP-{}'.format(measure): icp, 'SAMPLING-{}'.format(measure): sam}
df = pd.DataFrame(data = d)


# plt.legend(mylegends, loc='lower right')
# axes.set_ylim([-2800,-2400])
# axes.set_xlim([10000, 100000])

fig = plt.figure(figsize=(16, 4))
# plt.title("BoxPlot {} distance".format(measure))

xposition = [3.5, 7.5, 11.5, 15.5]
for xc in xposition:
    plt.axvline(x=xc, color='k', linestyle='-')

ax = sns.boxplot(data = df)
if measure == 'avg':
	ax.set_ylabel("Euclidean distance (mm)")
else:
	ax.set_ylabel("Hausdorff distance (mm)")
# ax.set_ylim([0,1.1])
# ax.set_ylim([0.25,0.5])
# ax.set_ylim([0.25,4.0])



# xcoords = [1.5, 3.5]
# xcoords = [2.5, 5.5]
# for xc in xcoords:
# 	plt.axvline(x=xc, linewidth=1, color='0.5')

for item in ([ax.title, ax.xaxis.label, ax.yaxis.label] + ax.get_xticklabels() + ax.get_yticklabels()):
	item.set_fontsize(20)

fig.tight_layout()
# plt.legend(loc='lower right')
# plt.title("BoxPlot {} distance".format(measure))
# sns.boxplot(data = pd.DataFrame(data = {'ICP': icpTotal, 'Sampling': samTotal}))
if measure == 'avg':
	saveName = f'{rangeName}_100'
else:	
	saveName = f'haus{rangeName}_100'
fig.savefig(f'/Volumes/storage/Dropbox/Apps/Overleaf/PhD_Thesis/img/mcmc/comparison/{saveName}.pdf', bbox_inches='tight')
plt.show()
