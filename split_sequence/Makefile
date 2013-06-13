
SPLIT_SEQ_ARCHIVE=split-sequence.tar.gz
SPLIT_SEQ_URL=http://code.in.tum.de/files/$(SPLIT_SEQ_ARCHIVE)

all: installed

downloaded: Makefile
	make clean
	$(shell echo $$ROS_ROOT)/core/rosbuild/bin/download_checkmd5.py $(SPLIT_SEQ_URL) $(SPLIT_SEQ_ARCHIVE)
	touch downloaded

installed: downloaded
	tar xvzf $(SPLIT_SEQ_ARCHIVE)
	mkdir -p asdf
	touch rospack_nosubdirs
	ln -s ../split-sequence/split-sequence.asd asdf
	touch installed

clean:
	rm -rf split-sequence asdf
	rm -f $(SPLIT_SEQ_ARCHIVE)
	rm -f rospack_nosubdirs
	rm -f installed downloaded

