# useful_scripts
some scripts which make importing episode data into essentially KnowRob/OpenEase easier. For any bugs or improvement ideas for code or documentation, please open an issue.

So far these scripts import the data in the format it results in from using [RobCog Semantic Logger](http://robcog.org/components.html). Meaning a .json containing all the event data and a SemanticMap.owl describing the environment.

## prerequisites
In order for these scripts to run, please install [jq](https://stedolan.github.io/jq/) first.

```bash
sudo apt-get install jq
```

### import_episodes:
This script will import all episode data into the currently running MongoDB instance. This script was written at a time, where MongoDB didn't support large files. This is why the files have to be split up first into multiple parts, loaded into the databse, and then dumped out again for further use. The newer versions of MongoDB don't require that extra step but it doesn't really harm either, so it's currently kept. 

#### prerequisites
Needs a directroy called "episodes", which should lay one layer above the directory containing the scripts. E.g. if you pull this repo onto your Desktop directory, it will look for the "episodes" directory on your Desktop. 
Before you call this file, please open it in an editor of your choice and set the parameter "episodePath" to wherever your Episode data should be located for KnowRob and OpenEase to be able to read it. 
*(This is equivalent to the *episode-path* parameter in CRAM, if you are following the 
[http://cram-system.org/tutorials/advanced/unreal](http://cram-system.org/tutorials/advanced/unreal) tutorial)* 

#### execution
Simply call the following in the terminal:

```bash
./import_episodes
```

### indexing_timestamps:
This script fill add an *timestamp* index to all the laoded episodes. This qill make quering the data a lot faster.

#### execution
Call the following in your terminal:

```bash
mongo < indexing_timestamp.js 
```
