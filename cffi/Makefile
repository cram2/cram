
TARBALL=cffi-20110810.tar.gz
TARBALL_URL=http://code.in.tum.de/files/$(TARBALL)

DIR_BASENAME=cffi-dev

all: installed

downloaded: Makefile
	make clean
	`rospack find rosbuild`/bin/download_checkmd5.py $(TARBALL_URL) $(TARBALL)
	mkdir -p asdf
	touch rospack_nosubdirs
	touch downloaded

installed: downloaded
	tar xvzf $(TARBALL)
	mkdir -p asdf
	touch rospack_nosubdirs
	ln -s ../$(DIR_BASENAME)/cffi.asd asdf/cffi.asd
	ln -s ../$(DIR_BASENAME)/cffi-uffi-compat.asd asdf/cffi-uffi-compat.asd
	ln -s ../$(DIR_BASENAME)/cffi-grovel.asd asdf/cffi-grovel.asd
	touch installed

clean:
	rm -rf $(DIR_BASENAME) asdf
	rm -f $(TARBALL) rospack_nosubdirs downloaded installed
