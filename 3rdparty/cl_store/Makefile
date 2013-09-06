
TARBALL=cl-store-20110210.tar.bz2
TARBALL_URL=http://ai.uni-bremen.de/public/$(TARBALL)

all: installed

downloaded: Makefile
	make clean
	$(shell echo $$ROS_ROOT)/core/rosbuild/bin/download_checkmd5.py $(TARBALL_URL) $(TARBALL)
	mkdir -p asdf
	touch rospack_nosubdirs
	touch downloaded

unpacked: downloaded
	tar xvjf $(TARBALL)
	touch unpacked

installed: unpacked
	ln -s ../cl-store/cl-store.asd asdf/cl-store.asd
	touch installed

clean:
	rm -rf cl-store asdf downloaded rospack_no_subdirs installed unpacked $(TARBALL)
