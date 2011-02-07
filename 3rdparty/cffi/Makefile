
CFFI_ARCHIVE=cffi_0.10.5.tar.gz

all: installed

downloaded: Makefile
	make clean
	wget http://common-lisp.net/project/cffi/releases/$(CFFI_ARCHIVE)
	touch downloaded

installed: downloaded
	tar xvzf $(CFFI_ARCHIVE)
	mkdir -p asdf
	touch rospack_nosubdirs
	ln -s ../cffi_0.10.5/cffi.asd asdf/cffi.asd
	ln -s ../cffi_0.10.5/cffi-uffi-compat.asd asdf/cffi-uffi-compat.asd
	ln -s ../cffi_0.10.5/cffi-grovel.asd asdf/cffi-grovel.asd
	touch installed

clean:
	rm -rf cffi_0.10.5 asdf
	rm -f $(CFFI_ARCHIVE)
	rm -f rospack_nosubdirs downloaded
	rm -f installed

