
TARBALL=trivial-garbage-20110210.tar.bz2
TARBALL_URL=http://code.in.tum.de/files/$(TARBALL)

all: installed

downloaded: Makefile
	make clean
	`rospack find rosbuild`/bin/download_checkmd5.py $(TARBALL_URL) $(TARBALL)
	mkdir -p asdf
	touch rospack_nosubdirs
	touch downloaded

unpacked: downloaded
	tar xvjf $(TARBALL)
	touch unpacked

installed: unpacked
	mkdir -p asdf
	ln -s ../trivial-garbage/trivial-garbage.asd asdf
	touch installed

clean:
	rm -rf trivial-garbage asdf
	rm -f downloaded rospack_nosubdirs installed unpacked $(TARBALL)
