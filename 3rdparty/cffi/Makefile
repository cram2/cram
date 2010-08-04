
CFFI_ARCHIVE=cffi_0.10.5.tar.gz
PACKAGE_DIR=$(shell rospack find cffi)

all: installed

downloaded: Makefile
	make clean
	wget http://common-lisp.net/project/cffi/releases/$(CFFI_ARCHIVE)
	touch downloaded

installed: downloaded
	tar xvzf $(CFFI_ARCHIVE)
	mkdir -p asdf
	touch rospack_nosubdirs
	ln -s ${PACKAGE_DIR}/cffi_0.10.5/cffi.asd ${PACKAGE_DIR}/asdf
	ln -s ${PACKAGE_DIR}/cffi_0.10.5/cffi-uffi-compat.asd ${PACKAGE_DIR}/asdf
	ln -s ${PACKAGE_DIR}/cffi_0.10.5/cffi-grovel.asd ${PACKAGE_DIR}/asdf
	touch installed

clean:
	rm -rf cffi_0.10.5 asdf
	rm -f $(CFFI_ARCHIVE)
	rm -f rospack_nosubdirs
	rm -f installed

