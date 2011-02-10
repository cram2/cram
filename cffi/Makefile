
TARBALL=cffi_0.10.6.tar.gz
TARBALL_URL=http://code.in.tum.de/files/$(TARBALL)

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
	ln -s ../cffi_0.10.6/cffi.asd asdf/cffi.asd
	ln -s ../cffi_0.10.6/cffi-uffi-compat.asd asdf/cffi-uffi-compat.asd
	ln -s ../cffi_0.10.6/cffi-grovel.asd asdf/cffi-grovel.asd
	touch installed

clean:
	rm -rf cffi_0.10.6 asdf
	rm -f $(TARBALL) rospack_nosubdirs downloaded installed
