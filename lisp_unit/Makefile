
TARBALL=lisp-unit-20120718.tar.bz2
TARBALL_URL=http://code.in.tum.de/files/$(TARBALL)

all: installed

downloaded: Makefile
	make clean
	`rospack find rosbuild`/bin/download_checkmd5.py $(TARBALL_URL) $(TARBALL)
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
