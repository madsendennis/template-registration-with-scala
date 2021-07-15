import matplotlib
import matplotlib.pyplot as plt
# import tkinter
import numpy as np
import json, os, math, re
import seaborn as sns
from datetime import datetime

sns.set(style="whitegrid")
# sns.set(style="darkgrid")
# sns.color_palette("dark") # deep, muted, pastel, bright, dark, and colorblind
# sns.set_palette("husl", 8)
sns.set_palette("bright", 6)

# sns.palplot(sns.color_palette("husl", 8))
datetimeformat = "%Y-%m-%d %H:%M:%S"

matplotlib.rc('xtick', labelsize=20)     
matplotlib.rc('ytick', labelsize=20)

print("RND vs ICP comparison")

def sortOnFloat(val):
    return float(val[-2])

path = "/export/skulls/projects/icp-proposal/data/femur/log_3DV/icpVsSamplingConvergence/"
allfiles = []

start = 'multiscale_100-49'
start = 'multiscale_50-49'
end   = 'index.json'

'RNDinit_'
'ICPinitUNIFstep'

#         RNDinit_gaussian-multiscale_50-49-samples-1000000-0-index.json
# ICPinitUNIFstep_gaussian-multiscale_50-49-samples-100000-0-index.json
# ICPinitUNIFstep-gaussian-multiscale_50-49-samples-100000-1.5-normalNoise.json
for root, dirs, fs in os.walk(path):  
	for f in fs:
		if root == path and start in f and end in f:
			name = f.replace(start,'').replace(end,'').replace("_gaussian", "").replace("Noise3", "").replace("Noise6", "").replace("Noise4", "").replace("Noise5", "")
			split = name.split('-')
			allfiles.append([f] + split)

# allfiles.sort(key = sortOnInt, reverse = False)

# for f in allfiles:
# 	print(f)

icpfiles = []
rndfiles = []

# /ICPinitUNIFstepNoise4_gaussian-multiscale_100-49-samples-5000-7-index.json

for f in allfiles:
	# if f[0].startswith('ICPinitUNIFstepNoise6_'):
	if f[0].startswith('CPinit'):# and '200000' in f[0]:
		icpfiles.append(f)
	elif f[0].startswith('RNDinit') and '1000000' in f[0]:
		rndfiles.append(f)

icpfiles.sort(key = sortOnFloat, reverse = False)
rndfiles.sort(key = sortOnFloat, reverse = False)



for f in icpfiles:
	print(f)
# for f in icpMfiles:
# 	print(f)
# for f in icpNoiseFiles:
	# print(f)
for f in rndfiles:
	print(f)


def plotSomeStuff(files, minlen, step, burnIn, timeMeasure=False):
	fig = plt.figure(figsize=(14, 5))
	# plt.title("Femur Surface Registration")
	ax1 = fig.add_subplot(111)

	mylegends = []
	for myfile in files:
		best = -999999999
		acceptCnt = 0
		mylegends.append(myfile[1]+"-"+myfile[-2])

		with open(path+myfile[0]) as json_data:
			data = json.load(json_data)

		logvalue = []
		samples = []
		timeing = []
		datalen = len(data)
		mindatalen = min(minlen,datalen)

		mean = 0.0
		for x in range(0, mindatalen, 1):
			if(x>burnIn):
				mean += data[x]['logvalue']['product']
		mean = mean/(mindatalen-burnIn-1)
		var = 0.0
		
		time0 = datetime.strptime(data[0]['datetime'], datetimeformat)
		for x in range(0, mindatalen, step):
			item = data[x]
			logval = item['logvalue']['product'] # product / distance
			logvalue.append(logval)
			# if timeMeasure == True:
			timeing.append((datetime.strptime(data[x]['datetime'], datetimeformat)-time0).total_seconds())
			samples.append(item['index']/1000)
			if(item['status']):
				acceptCnt+=1
			if(logval > best):
				best = logval
			if(x>burnIn):
				var += math.pow(logval-mean, 2)
		var = var/(mindatalen-burnIn-1)
		print('{}-{}-{} - BEST: {} ACCEPTRATE: {} MEAN: {} VAR: {}'.format(myfile[1], myfile[2],myfile[-2], best, acceptCnt/datalen, mean, var))

		ax1.plot(timeing, logvalue)

	ax2 = ax1.twiny()
	ax2.set_xlim(0, samples[-1])
	ax2.set_xlabel('Iteration x 1000')



	ax1.legend(mylegends, loc='lower right')
	ax1.set_ylabel('Unnormalised posterior value [log]')
	ax1.set_xlabel('Seconds')

	# axes = plt.gca()
	# ax1.set_ylim([-3500,-2000]) # 50 experiment
	ax1.set_ylim([-3000,-1800]) # 

	for item in ([ax1.title, ax1.xaxis.label, ax1.yaxis.label] + ax1.get_xticklabels() + ax1.get_yticklabels()):
		item.set_fontsize(19)
	for item in ([ax2.title, ax2.xaxis.label, ax2.yaxis.label] + ax2.get_xticklabels() + ax2.get_yticklabels()):
		item.set_fontsize(19)
	fig.tight_layout()


	# ax1.set_xlim([0, 650])
	# ax1.set_xlim([0, 10])


# 50 all
plotSomeStuff(rndfiles[0:5], 10000000, 1, 20000, True)
plotSomeStuff(icpfiles[0:5], 10000000, 1, 1000, True)

# 50 medium
# plotSomeStuff(rndfiles[0:5], 24000, 1, 20000, True)
# plotSomeStuff(icpfiles[0:5], 10000, 1, 1000, True)

# 50 zoom
# plotSomeStuff(rndfiles[0:5], 2400, 1, 20000, True)
# plotSomeStuff(icpfiles[0:5], 1000, 1, 1000, True)

# 100 zoom
# plotSomeStuff(rndfiles[0:5], 5000, 1, 20000, True)
# plotSomeStuff(icpfiles[0:5], 1000, 1, 1000, True)


# 100 all
# plotSomeStuff(rndfiles[0:5], 1000000, 1, 20000, True)
# plotSomeStuff(icpfiles[0:5], 1000000, 1, 1000, True)

plt.show()
