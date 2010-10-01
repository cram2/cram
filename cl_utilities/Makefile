
CL_UTILS_ARCHIVE=cl-utilities-1.2.4.tar.gz
CL_UTILS_URL=http://www.common-lisp.net/project/cl-utilities/$(CL_UTILS_ARCHIVE)
PACKAGE_DIR=$(shell rospack find cl_utilities)

all: installed

downloaded: Makefile
	make clean
	wget $(CL_UTILS_URL)
	touch downloaded

installed: downloaded
	tar xvzf $(CL_UTILS_ARCHIVE)
	mkdir -p asdf
	touch rospack_nosubdirs
	ln -s ${PACKAGE_DIR}/cl-utilities-1.2.4/cl-utilities.asd ${PACKAGE_DIR}/asdf
	touch installed

clean:
	rm -rf cl-utilities-1.2.4 asdf
	rm -f $(CL_UTILS_ARCHIVE)
	rm -f rospack_nosubdirs
	rm -f installed downloaded

