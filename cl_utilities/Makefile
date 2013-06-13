
CL_UTILS_ARCHIVE=cl-utilities-1.2.4.tar.gz
CL_UTILS_URL=http://ai.uni-bremen.de/public/$(CL_UTILS_ARCHIVE)

all: installed

downloaded: Makefile
	make clean
	$(shell echo $$ROS_ROOT)/core/rosbuild/bin/download_checkmd5.py $(CL_UTILS_URL) $(CL_UTILS_ARCHIVE)
	touch downloaded

installed: downloaded
	tar xvzf $(CL_UTILS_ARCHIVE)
	mkdir -p asdf
	touch rospack_nosubdirs
	ln -s ../cl-utilities-1.2.4/cl-utilities.asd asdf/cl-utilities.asd
	touch installed

clean:
	rm -rf cl-utilities-1.2.4 asdf
	rm -f $(CL_UTILS_ARCHIVE)
	rm -f rospack_nosubdirs
	rm -f installed downloaded
