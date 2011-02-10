
TARBALL=alexandria-20110210.tar.bz2
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
	ln -s ../alexandria/alexandria.asd asdf/alexandria.asd
	touch installed

clean:
	rm -rf alexandria asdf $(TARBALL)
	rm -f downloaded rospack_nosubdirs installed unpacked
