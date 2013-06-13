
TARBALL=lisp-unit-20120718.tar.bz2
TARBALL_URL=http://ai.uni-bremen.de/public/$(TARBALL)

all: installed

downloaded: Makefile
	make clean
	$(shell echo $$ROS_ROOT)/core/rosbuild/bin/download_checkmd5.py $(TARBALL_URL) $(TARBALL)
	touch rospack_nosubdirs
	touch downloaded

unpacked: downloaded
	tar xvjf $(TARBALL)
	touch unpacked

installed: unpacked
	mkdir -p asdf
	ln -s ../lisp-unit/lisp-unit.asd asdf/
	touch installed

clean:
	rm -rf lisp-unit asdf
	rm -f downloaded rospack_nosubdirs installed unpacked $(TARBALL)
