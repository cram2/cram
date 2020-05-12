#!/bin/bash

#make fancy colors available
RED='\033[0;31m'
GREEN='\033[0;32m'
ORANGE='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# set this to where you want your episode files to end up.
# should be same as the OpenEase episode path
episodePath="/home/cram/ros_workspace/episode_data/episodes/Own-Episodes/set-clean-table/"

cd episodes

# find all directories in episodes folder
folder=$(find . -maxdepth 1 -mindepth 1 -type d -printf '%f\n')

echo "found following directories:"
echo -e "${ORANGE} $folder ${NC}"

# go into each directory
for dir in $folder
do
	echo "\n ---"
	echo -e "going into ${ORANGE} $dir ${NC}"
	# go into each directory and find *.json file
	cd $dir
	file=$(find . -maxdepth 1 -mindepth 1 -type f -name *.json)
	#cut of path extension from name
	jsonfile=$(basename -- "${file}")
	echo -e "found this json file: ${GREEN} ${jsonFile} ${NC}"
	#note: only one file per folder!
	
	#make folder for the file splitting, navigate into it
	mkdir split
	cd split
	
	# split file into smaller pieces for import
	cat ../${jsonfile} | jq -c -M '.' | split -l 2000
	
	#find name of file without json extension for import:
	filename=${jsonfile%*.json}
	cd ..
	
	#import all the parts into mongodb
	for filepart in split/* 
	do
		echo -e "\n${BLUE}mongoimport --db Own-Episodes_set-clean-table --collection ${filename} --file ${filepart} ${NC}"
		mongoimport --db Own-Episodes_set-clean-table --collection ${filename} --file ${filepart}
		echo -e "(hopefully) imported ${ORANGE} $filepart ${NC} into mongo collection ${GREEN} $filename ${NC}"
	done
	
	#now dump the file as a *.bson so that OpenEase can use it later on

	mongodump --db Own-Episodes_set-clean-table --collection $filename
	cd dump/Own-Episodes_set-clean-table
	bsonfile=$(find . -maxdepth 1 -mindepth 1 -type f -name *.bson)
	echo -e "created ${GREEN} $bsonfile ${NC}"
	
	#create new folder in episode path and copy files there
	#copy bson
	mkdir ${episodePath}${dir}
	scp $bsonfile ${episodePath}${dir}/${bsonfile}
	#copy semantic map
	cd ../..
	scp SemanticMap.owl ${episodePath}${dir}/.
	#copy Episodes folder
	mkdir ${episodePath}${dir}/Episodes
	eventdata=$(find . -maxdepth 1 -mindepth 1 -type d -name EventData*)
	scp -r ${eventdata}  ${episodePath}${dir}/Episodes/.
	
	#go back out for next folder
	cd ..
done

echo -e "\n${GREEN}all done, hopefully successfull! Have fun with VR data :) ${NC}"
